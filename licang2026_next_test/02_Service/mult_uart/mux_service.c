/**
 * @file    mux_service.c
 * @brief   UART7复用器的CMSIS-RTOS2队列、worker和UART路由实现。
 */

#include "mux_service.h"

#include <stdbool.h>
#include <string.h>

#include "cmsis_os.h"
#include "uart_dispatch.h"

#define MUX_FLAG_REQUEST    (1UL << 0)
#define MUX_FLAG_EVENT      (1UL << 1)
#define MUX_WAIT_TICKS      1U
#define MUX_WORKER_STACK    (512U * 4U)

typedef struct {
    uint32_t request_id;
    mux_device_t device;
    mult_uart_operation_t operation;
    mult_uart_channel_t channel;
    uint8_t tx_data[MUX_TX_MAX];
    size_t tx_len;
    uint8_t rx_data[MUX_RX_MAX];
    size_t rx_capacity;
    uint32_t io_timeout_ms;
    uint32_t deadline_ms;
    mux_done_fn_t done_cb;
    void *user_ctx;
} mux_job_t;

typedef struct {
    volatile bool tx_done;
    volatile bool rx_done;
    volatile bool error;
    volatile size_t rx_len;
    volatile mult_uart_status_t error_status;
} mux_events_t;

typedef struct {
    bool initialized;
    bool active;
    mult_uart_bus_t bus;
    mux_job_t active_job;
    mux_events_t events;
    osMessageQueueId_t queue;
    osThreadId_t worker;
    uint32_t next_request_id;
    uint8_t busy_mask;
} mux_service_t;

static mux_service_t g_mux;

static const osThreadAttr_t g_mux_worker_attr = {
    .name = "muxSvc",
    .stack_size = MUX_WORKER_STACK,
    .priority = (osPriority_t)osPriorityAboveNormal,
};

static bool mux_needs_tx(mult_uart_operation_t operation)
{
    return (operation == MULT_UART_OP_WRITE) ||
           (operation == MULT_UART_OP_WRITE_READ);
}

static bool mux_needs_rx(mult_uart_operation_t operation)
{
    return (operation == MULT_UART_OP_READ) ||
           (operation == MULT_UART_OP_WRITE_READ);
}

/** 把RTOS tick转换为毫秒，保持原事务超时语义。 */
static uint32_t mux_now_ms(void)
{
    uint64_t ms;
    uint32_t freq = osKernelGetTickFreq();

    if (freq == 0U) return 0U;
    ms = ((uint64_t)osKernelGetTickCount() * 1000ULL) / (uint64_t)freq;
    return (ms > UINT32_MAX) ? UINT32_MAX : (uint32_t)ms;
}

static uint32_t mux_ms_to_ticks(uint32_t timeout_ms)
{
    uint64_t ticks;
    uint32_t freq;

    if ((timeout_ms == 0U) || (timeout_ms == osWaitForever)) {
        return timeout_ms;
    }
    freq = osKernelGetTickFreq();
    ticks = ((uint64_t)timeout_ms * (uint64_t)freq + 999ULL) / 1000ULL;
    if (ticks == 0ULL) ticks = 1ULL;
    return (ticks > UINT32_MAX) ? UINT32_MAX : (uint32_t)ticks;
}

static mult_uart_status_t mux_map_os(osStatus_t status)
{
    if (status == osOK) return MULT_UART_OK;
    if ((status == osErrorTimeout) || (status == osErrorResource)) {
        return MULT_UART_ERR_QUEUE_FULL;
    }
    return (status == osErrorParameter) ?
        MULT_UART_ERR_PARAM : MULT_UART_ERR_IO;
}

static mult_uart_status_t mux_validate(const mux_transfer_t *transfer)
{
    if ((transfer == NULL) ||
        ((uint32_t)transfer->device >= MULT_UART_CHANNEL_COUNT) ||
        ((uint32_t)transfer->operation > (uint32_t)MULT_UART_OP_WRITE_READ)) {
        return MULT_UART_ERR_PARAM;
    }
    if (mux_needs_tx(transfer->operation)) {
        if ((transfer->tx_data == NULL) || (transfer->tx_len == 0U)) {
            return MULT_UART_ERR_PARAM;
        }
        if (transfer->tx_len > MUX_TX_MAX) {
            return MULT_UART_ERR_OVERFLOW;
        }
    } else if ((transfer->tx_data != NULL) || (transfer->tx_len != 0U)) {
        return MULT_UART_ERR_PARAM;
    }
    if (mux_needs_rx(transfer->operation)) {
        if ((transfer->rx_capacity == 0U) ||
            (transfer->rx_capacity > MUX_RX_MAX)) {
            return MULT_UART_ERR_OVERFLOW;
        }
    } else if (transfer->rx_capacity != 0U) {
        return MULT_UART_ERR_PARAM;
    }
    return MULT_UART_OK;
}

static void mux_make_job(
    mux_job_t *job,
    const mux_transfer_t *transfer)
{
    (void)memset(job, 0, sizeof(*job));
    job->device = transfer->device;
    job->operation = transfer->operation;
    job->channel = (mult_uart_channel_t)transfer->device;
    job->tx_len = transfer->tx_len;
    job->rx_capacity = transfer->rx_capacity;
    job->io_timeout_ms = (transfer->io_timeout_ms != 0U) ?
        transfer->io_timeout_ms : MUX_DEFAULT_IO_MS;
    job->done_cb = transfer->done_cb;
    job->user_ctx = transfer->user_ctx;
    if (transfer->tx_len > 0U) {
        (void)memcpy(job->tx_data, transfer->tx_data, transfer->tx_len);
    }
}

static void mux_clear_events(mux_service_t *ctx)
{
    ctx->events.tx_done = false;
    ctx->events.rx_done = false;
    ctx->events.error = false;
    ctx->events.rx_len = 0U;
    ctx->events.error_status = MULT_UART_OK;
}

/** ISR只登记结果并唤醒worker。 */
static void mux_on_event(void *user_ctx, const mult_uart_event_t *event)
{
    mux_service_t *ctx = (mux_service_t *)user_ctx;

    if ((ctx == NULL) || (event == NULL)) return;
    if (event->type == MULT_UART_EVENT_TX_COMPLETE) {
        ctx->events.tx_done = true;
    } else if (event->type == MULT_UART_EVENT_RX_COMPLETE) {
        ctx->events.rx_len = event->rx_len;
        ctx->events.rx_done = true;
    } else {
        ctx->events.error_status = event->status;
        ctx->events.error = true;
    }
    if (ctx->worker != NULL) {
        (void)osThreadFlagsSet(ctx->worker, MUX_FLAG_EVENT);
    }
}

static bool mux_dispatch_tx(void *user_ctx, UART_HandleTypeDef *huart)
{
    mux_service_t *ctx = (mux_service_t *)user_ctx;
    return (ctx != NULL) && mult_uart_handle_tx(&ctx->bus, huart);
}

static bool mux_dispatch_rx(
    void *user_ctx,
    UART_HandleTypeDef *huart,
    uint16_t rx_len)
{
    mux_service_t *ctx = (mux_service_t *)user_ctx;
    return (ctx != NULL) && mult_uart_handle_rx(&ctx->bus, huart, rx_len);
}

static bool mux_dispatch_error(void *user_ctx, UART_HandleTypeDef *huart)
{
    mux_service_t *ctx = (mux_service_t *)user_ctx;
    return (ctx != NULL) && mult_uart_handle_error(&ctx->bus, huart);
}

static mult_uart_status_t mux_select(
    mux_service_t *ctx,
    mult_uart_channel_t channel)
{
    mult_uart_channel_t current;
    mult_uart_status_t status = mult_uart_get_channel(&ctx->bus, &current);

    if (status != MULT_UART_OK) return status;
    if (current != channel) {
        status = mult_uart_select(&ctx->bus, channel);
        if (status != MULT_UART_OK) return status;
    }
    return mult_uart_enable(&ctx->bus);
}

/** READ和WRITE_READ固定先挂RX，再启动TX。 */
static mult_uart_status_t mux_start_active(mux_service_t *ctx)
{
    mux_job_t *job = &ctx->active_job;
    mult_uart_status_t status;

    mux_clear_events(ctx);
    status = mux_select(ctx, job->channel);
    if ((status != MULT_UART_OK) ||
        (job->operation == MULT_UART_OP_SELECT)) {
        return status;
    }
    if (mux_needs_rx(job->operation)) {
        status = mult_uart_start_rx_dma(
            &ctx->bus,
            job->rx_data,
            job->rx_capacity);
        if (status != MULT_UART_OK) return status;
    }
    if (mux_needs_tx(job->operation)) {
        status = mult_uart_start_tx_dma(
            &ctx->bus,
            job->tx_data,
            job->tx_len);
        if (status != MULT_UART_OK) {
            if (mux_needs_rx(job->operation)) {
                (void)mult_uart_abort(&ctx->bus);
            }
            return status;
        }
    }
    return MULT_UART_OK;
}

static void mux_finish(
    mux_service_t *ctx,
    mult_uart_status_t status,
    size_t rx_len)
{
    mux_completion_t completion;
    mux_done_fn_t done_cb = ctx->active_job.done_cb;
    void *done_ctx = ctx->active_job.user_ctx;

    completion.request_id = ctx->active_job.request_id;
    completion.device = ctx->active_job.device;
    completion.status = status;
    completion.operation = ctx->active_job.operation;
    completion.rx_data = ((status == MULT_UART_OK) &&
        mux_needs_rx(ctx->active_job.operation)) ?
        ctx->active_job.rx_data : NULL;
    completion.rx_len = (status == MULT_UART_OK) ? rx_len : 0U;

    ctx->busy_mask &= (uint8_t)~(1U << (uint32_t)completion.device);
    ctx->active = false;
    mux_clear_events(ctx);
    if (done_cb != NULL) done_cb(done_ctx, &completion);
    (void)memset(&ctx->active_job, 0, sizeof(ctx->active_job));
}

static bool mux_active_complete(const mux_service_t *ctx)
{
    if (mux_needs_tx(ctx->active_job.operation) && !ctx->events.tx_done) {
        return false;
    }
    if (mux_needs_rx(ctx->active_job.operation) && !ctx->events.rx_done) {
        return false;
    }
    return true;
}

static void mux_process(mux_service_t *ctx)
{
    mult_uart_status_t status;

    if (ctx->active) {
        if (ctx->events.error) {
            status = ctx->events.error_status;
            (void)mult_uart_abort(&ctx->bus);
            mux_finish(
                ctx,
                (status == MULT_UART_OK) ? MULT_UART_ERR_IO : status,
                0U);
        } else if ((ctx->active_job.io_timeout_ms > 0U) &&
                   ((int32_t)(mux_now_ms() -
                    ctx->active_job.deadline_ms) >= 0)) {
            status = mult_uart_abort(&ctx->bus);
            mux_finish(
                ctx,
                (status == MULT_UART_OK) ? MULT_UART_ERR_TIMEOUT : status,
                0U);
        } else if (mux_active_complete(ctx)) {
            mux_finish(
                ctx,
                MULT_UART_OK,
                ctx->events.rx_done ? ctx->events.rx_len : 0U);
        }
    }

    while (!ctx->active) {
        if (osMessageQueueGet(
                ctx->queue,
                &ctx->active_job,
                NULL,
                0U) != osOK) {
            return;
        }
        ctx->active = true;
        ctx->active_job.deadline_ms =
            mux_now_ms() + ctx->active_job.io_timeout_ms;
        status = mux_start_active(ctx);
        if ((status == MULT_UART_OK) &&
            (ctx->active_job.operation != MULT_UART_OP_SELECT)) {
            return;
        }
        mux_finish(ctx, status, 0U);
    }
}

static void mux_worker(void *argument)
{
    mux_service_t *ctx = (mux_service_t *)argument;

    for (;;) {
        mux_process(ctx);
        (void)osThreadFlagsWait(
            MUX_FLAG_REQUEST | MUX_FLAG_EVENT,
            osFlagsWaitAny,
            MUX_WAIT_TICKS);
    }
}

mult_uart_status_t mux_init(void)
{
    mux_service_t *ctx = &g_mux;
    uart_dispatch_handler_t handler = {0};
    uart_dispatch_handle_t dispatch_handle;
    mult_uart_status_t status;

    if (ctx->initialized) return MULT_UART_ERR_STATE;
    (void)memset(ctx, 0, sizeof(*ctx));
    ctx->queue = osMessageQueueNew(
        MUX_QUEUE_DEPTH,
        sizeof(mux_job_t),
        NULL);
    if (ctx->queue == NULL) return MULT_UART_ERR_IO;

    status = mult_uart_init(&ctx->bus);
    if (status != MULT_UART_OK) return status;
    status = mult_uart_bind_event(&ctx->bus, mux_on_event, ctx);
    if (status != MULT_UART_OK) return status;

    handler.tx_complete = mux_dispatch_tx;
    handler.rx_event = mux_dispatch_rx;
    handler.error = mux_dispatch_error;
    handler.user_ctx = ctx;
    if (!uart_dispatch_register(&handler, &dispatch_handle)) {
        return MULT_UART_ERR_IO;
    }
    ctx->worker = osThreadNew(mux_worker, ctx, &g_mux_worker_attr);
    if (ctx->worker == NULL) return MULT_UART_ERR_IO;
    ctx->initialized = true;
    return MULT_UART_OK;
}

mult_uart_status_t mux_submit(const mux_transfer_t *transfer)
{
    mux_job_t job;
    mult_uart_status_t status;
    osStatus_t os_status;
    uint8_t device_mask;

    if (!g_mux.initialized) {
        return (transfer == NULL) ? MULT_UART_ERR_PARAM :
            MULT_UART_ERR_NOT_INIT;
    }
    status = mux_validate(transfer);
    if (status != MULT_UART_OK) return status;
    device_mask = (uint8_t)(1U << (uint32_t)transfer->device);
    if ((g_mux.busy_mask & device_mask) != 0U) return MULT_UART_ERR_BUSY;
    g_mux.busy_mask |= device_mask;
    ++g_mux.next_request_id;
    if (g_mux.next_request_id == 0U) ++g_mux.next_request_id;
    mux_make_job(&job, transfer);
    job.request_id = g_mux.next_request_id;
    os_status = osMessageQueuePut(
        g_mux.queue,
        &job,
        0U,
        mux_ms_to_ticks(transfer->queue_timeout_ms));
    if (os_status != osOK) {
        g_mux.busy_mask &= (uint8_t)~device_mask;
        return mux_map_os(os_status);
    }
    (void)osThreadFlagsSet(g_mux.worker, MUX_FLAG_REQUEST);
    return MULT_UART_OK;
}
