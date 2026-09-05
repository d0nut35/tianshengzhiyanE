/**
 * @file    ic_card_service.c
 * @brief   IC卡读写器请求排队、响应匹配和超时恢复实现。
 */

#include "ic_card_service.h"

#include <string.h>

#include "mux_service.h"

#if !LICANG_RELEASE_MINIMAL
#include "cmsis_os.h"
#include "uart_dispatch.h"
#endif

typedef struct {
    bool initialized;
    bool active;
    uint32_t next_request_id;
    uint32_t request_id;
    ic_done_fn_t done_cb;
    void *user_ctx;
    uint8_t tx[IC_CARD_COMMAND_FRAME_SIZE];
} ic_context_t;

static ic_context_t g_ic;

static ic_card_status_t ic_map_mux(mult_uart_status_t status)
{
    if (status == MULT_UART_OK) return IC_CARD_OK;
    if (status == MULT_UART_ERR_TIMEOUT) return IC_CARD_ERR_TIMEOUT;
    if (status == MULT_UART_ERR_BUSY) return IC_CARD_ERR_BUSY;
    if (status == MULT_UART_ERR_QUEUE_FULL) return IC_CARD_ERR_QUEUE_FULL;
    if ((status == MULT_UART_ERR_PARAM) || (status == MULT_UART_ERR_OVERFLOW)) {
        return IC_CARD_ERR_PARAM;
    }
    return IC_CARD_ERR_IO;
}

bool ic_decode_ball(
    const uint8_t data[IC_CARD_BLOCK_DATA_SIZE],
    ic_ball_t *ball)
{
    uint8_t i;
    uint8_t row;
    uint8_t column;

    if ((data == NULL) || (ball == NULL)) return false;
    (void)memset(ball, 0, sizeof(*ball));
    for (i = 1U; i < IC_CARD_BLOCK_DATA_SIZE; ++i) {
        if (data[i] != data[0]) return false;
    }
    row = (uint8_t)(data[0] >> 4);
    column = (uint8_t)(data[0] & 0x0FU);
    if ((row < 1U) || (row > 3U) || (column < 1U) || (column > 4U)) {
        return false;
    }
    ball->kind = IC_BALL_TARGET;
    ball->code = data[0];
    ball->row = row;
    ball->column = column;
    return true;
}

static void ic_mux_done(void *user_ctx, const mux_completion_t *completion)
{
    ic_context_t *ctx = (ic_context_t *)user_ctx;
    ic_card_response_t response;
    ic_result_t result;
    uint8_t block[IC_CARD_BLOCK_DATA_SIZE];
    ic_done_fn_t done_cb;
    void *done_ctx;
    uint32_t request_id;
    ic_card_status_t status;

    if ((ctx == NULL) || !ctx->active || (completion == NULL)) return;
    done_cb = ctx->done_cb;
    done_ctx = ctx->user_ctx;
    request_id = ctx->request_id;
    ctx->active = false;
    ctx->done_cb = NULL;
    ctx->user_ctx = NULL;

    status = ic_map_mux(completion->status);
    if (status == IC_CARD_OK) {
        status = ic_parse_frame(
            completion->rx_data, completion->rx_len, &response);
    }
    if (status == IC_CARD_OK) {
        status = ic_block_data(&response, IC_ADDRESS, block);
    }
    if ((status == IC_CARD_OK) && !ic_decode_ball(block, &result.ball)) {
        status = IC_CARD_ERR_PROTOCOL;
    }
    if (done_cb != NULL) {
        done_cb(
            done_ctx,
            request_id,
            status,
            (status == IC_CARD_OK) ? &result : NULL);
    }
}

ic_card_status_t ic_init(void)
{
    if (g_ic.initialized) return IC_CARD_ERR_STATE;
    (void)memset(&g_ic, 0, sizeof(g_ic));
    g_ic.initialized = true;
    return IC_CARD_OK;
}

ic_card_status_t ic_read(
    bool led_beep_prompt,
    ic_done_fn_t done_cb,
    void *user_ctx)
{
    mux_transfer_t transfer;
    mult_uart_status_t mux_status;
    ic_card_status_t status;
    size_t tx_len;

    if (!g_ic.initialized) return IC_CARD_ERR_NOT_INIT;
    if (g_ic.active) return IC_CARD_ERR_BUSY;
    status = ic_read_frame(
        IC_ADDRESS,
        IC_DATA_BLOCK,
        led_beep_prompt,
        g_ic.tx,
        sizeof(g_ic.tx),
        &tx_len);
    if (status != IC_CARD_OK) return status;

    ++g_ic.next_request_id;
    if (g_ic.next_request_id == 0U) ++g_ic.next_request_id;
    g_ic.request_id = g_ic.next_request_id;
    g_ic.done_cb = done_cb;
    g_ic.user_ctx = user_ctx;
    g_ic.active = true;

    (void)memset(&transfer, 0, sizeof(transfer));
    transfer.device = MUX_DEVICE_1;
    transfer.operation = MULT_UART_OP_WRITE_READ;
    transfer.tx_data = g_ic.tx;
    transfer.tx_len = tx_len;
    transfer.rx_capacity = IC_CARD_FRAME_SIZE_MAX;
    transfer.io_timeout_ms = IC_READ_TIMEOUT_MS;
    transfer.done_cb = ic_mux_done;
    transfer.user_ctx = &g_ic;
    mux_status = mux_submit(&transfer);
    if (mux_status != MULT_UART_OK) {
        g_ic.active = false;
        g_ic.done_cb = NULL;
        g_ic.user_ctx = NULL;
    }
    return ic_map_mux(mux_status);
}

/**
 * @brief 判断32位毫秒期限是否已经到达。
 * @param now 当前毫秒计数。
 * @param deadline 截止毫秒计数。
 * @return 到期返回true，否则返回false。
 * @note 使用有符号差值以兼容计数器自然回绕。
 */
static bool ic_card_time_reached(uint32_t now, uint32_t deadline)
{
    return ((int32_t)(now - deadline) >= 0);
}

/**
 * @brief 校验请求能否转换为当前支持的厂家命令。
 * @param request 待检查请求。
 * @return 请求及超时参数有效时返回true。
 * @note 提交前拒绝坏请求，避免无效事务长期占用UART7。
 */
static bool ic_card_request_is_valid(const ic_card_request_t *request)
{
    if ((request == NULL) || (request->timeout_ms == 0U)) {
        return false;
    }
    if (request->type == IC_CARD_REQUEST_READ_BLOCK) {
        return true;
    }
    if (request->type == IC_CARD_REQUEST_QUERY) {
        return ((request->data.query.command == IC_CARD_CMD_QUERY_ADDRESS) ||
                (request->data.query.command == IC_CARD_CMD_QUERY_WORK_MODE) ||
                (request->data.query.command == IC_CARD_CMD_QUERY_BEEPER) ||
                (request->data.query.command == IC_CARD_CMD_QUERY_AUTO_READ));
    }
    return false;
}

/**
 * @brief 接收Core ISR事件并唤醒Service worker。
 * @param user_ctx 绑定的Service对象。
 * @param event Core报告的TX、RX或错误事件。
 * @warning 本函数运行在ISR上下文，只更新单调序号并通知worker，禁止解析和回调用户代码。
 */
static void ic_card_service_isr_notify(
    void *user_ctx,
    ic_card_isr_event_t event)
{
    ic_card_service_t *service = (ic_card_service_t *)user_ctx;

    if (service == NULL) {
        return;
    }
    if (event == IC_CARD_ISR_EVENT_TX_COMPLETE) {
        ++service->tx_sequence;
    } else if (event == IC_CARD_ISR_EVENT_RX_READY) {
        ++service->rx_sequence;
    } else {
        ++service->error_sequence;
    }
    if (service->config.notify_worker != NULL) {
        service->config.notify_worker(service->config.notify_ctx);
    }
}

/**
 * @brief 取得当前请求期望匹配的响应命令号。
 * @param request 当前活动请求。
 * @return A3读块命令号或请求指定的查询命令号。
 */
static uint8_t ic_card_expected_command(const ic_card_request_t *request)
{
    return (request->type == IC_CARD_REQUEST_READ_BLOCK) ?
        IC_CARD_CMD_READ_BLOCK_KEY_A : (uint8_t)request->data.query.command;
}

/**
 * @brief 判断一帧响应是否属于当前活动请求。
 * @param request 当前活动请求。
 * @param response 已通过Core校验的响应。
 * @return 命令号和地址语义匹配时返回true。
 * @note B0查询地址的响应地址位位于payload中，因此不按普通地址字段匹配。
 */
static bool ic_card_response_matches(
    const ic_card_request_t *request,
    const ic_card_response_t *response)
{
    if ((request == NULL) || (response == NULL) ||
        (response->command != ic_card_expected_command(request))) {
        return false;
    }
    if ((request->type == IC_CARD_REQUEST_QUERY) &&
        (request->data.query.command == IC_CARD_CMD_QUERY_ADDRESS)) {
        return true;
    }
    return (response->address == request->address);
}

/**
 * @brief 收敛当前事务状态并发布最终完成回调。
 * @param service Service对象。
 * @param status 最终事务状态。
 * @param response 成功或卡错误时的响应；无响应失败时为NULL。
 * @note 先清除活动请求再调用用户回调，允许回调安全提交下一笔事务。
 * @warning response仅在回调期间有效，需要长期保存时由调用者立即复制。
 */
static void ic_card_service_complete(
    ic_card_service_t *service,
    ic_card_status_t status,
    const ic_card_response_t *response)
{
    ic_card_request_t finished;

    if (!service->active_valid) {
        return;
    }
    finished = service->active;
    service->active_valid = false;
    service->tx_completed = false;
    (void)memset(&service->active, 0, sizeof(service->active));
    if (status == IC_CARD_OK) {
#if IC_CARD_SERVICE_DIAGNOSTICS_ENABLE
        ++service->stats.completed;
#endif
    }
    if (finished.done_cb != NULL) {
        finished.done_cb(
            finished.user_ctx,
            finished.request_id,
            status,
            response);
    }
}

#if !LICANG_RELEASE_MINIMAL
#define IC_SERVICE_FLAG_REQUEST       (1UL << 0)
#define IC_SERVICE_FLAG_ISR_EVENT     (1UL << 1)
#define IC_SERVICE_WAIT_SLICE_MS      10U
#define IC_SERVICE_QUEUE_DEPTH        4U
#define IC_SERVICE_WORKER_STACK       (512U * 4U)

typedef struct {
    bool initialized;
    ic_card_t device;
    ic_card_port_t port;
    ic_card_uart7_hal_t adapter;
    ic_card_service_t service;
    osMessageQueueId_t request_queue;
    osThreadId_t worker_thread;
    uart_dispatch_handle_t dispatch_handle;
    bool dispatch_registered;
} ic_service_context_t;

static ic_service_context_t g_ic_service;

static const osThreadAttr_t g_ic_worker_attr = {
    .name = "icCardSvc",
    .stack_size = IC_SERVICE_WORKER_STACK,
    .priority = (osPriority_t)osPriorityNormal,
};

static uint32_t ic_service_ticks(uint32_t timeout_ms)
{
    uint64_t ticks;
    uint32_t frequency;

    if ((timeout_ms == 0U) || (timeout_ms == osWaitForever)) {
        return timeout_ms;
    }
    frequency = osKernelGetTickFreq();
    ticks = ((uint64_t)timeout_ms * frequency + 999ULL) / 1000ULL;
    if (ticks == 0ULL) ticks = 1ULL;
    return (ticks > UINT32_MAX) ? UINT32_MAX : (uint32_t)ticks;
}

static uint32_t ic_service_now(void *ctx)
{
    (void)ctx;
    return HAL_GetTick();
}

static void ic_service_notify(void *ctx)
{
    ic_service_context_t *service = (ic_service_context_t *)ctx;

    if ((service != NULL) && (service->worker_thread != NULL)) {
        (void)osThreadFlagsSet(
            service->worker_thread, IC_SERVICE_FLAG_ISR_EVENT);
    }
}

static bool ic_service_tx_isr(void *ctx, UART_HandleTypeDef *huart)
{
    ic_service_context_t *service = (ic_service_context_t *)ctx;
    return (service != NULL) && ic_uart7_tx_isr(&service->adapter, huart);
}

static bool ic_service_rx_isr(
    void *ctx,
    UART_HandleTypeDef *huart,
    uint16_t rx_len)
{
    ic_service_context_t *service = (ic_service_context_t *)ctx;
    return (service != NULL) &&
        ic_uart7_rx_isr(&service->adapter, huart, rx_len);
}

static bool ic_service_error_isr(void *ctx, UART_HandleTypeDef *huart)
{
    ic_service_context_t *service = (ic_service_context_t *)ctx;
    return (service != NULL) && ic_uart7_error_isr(&service->adapter, huart);
}

static void ic_service_rollback(ic_service_context_t *ctx)
{
    if (ctx->dispatch_registered) {
        (void)uart_dispatch_unregister(ctx->dispatch_handle);
    }
    if (ctx->service.initialized && !ctx->service.active_valid &&
        (ctx->service.queue_count == 0U)) {
        (void)ic_card_service_deinit(&ctx->service);
    }
    if (ctx->device.initialized) {
        (void)ic_bsp_deinit(&ctx->device);
    }
    if (ctx->request_queue != NULL) {
        (void)osMessageQueueDelete(ctx->request_queue);
    }
    (void)memset(ctx, 0, sizeof(*ctx));
}

static void ic_service_drain(ic_service_context_t *ctx)
{
    ic_card_request_t request;
    ic_card_status_t status;

    while (ctx->service.queue_count < IC_CARD_SERVICE_QUEUE_DEPTH) {
        if (osMessageQueueGet(ctx->request_queue, &request, NULL, 0U) != osOK) {
            return;
        }
        status = ic_card_service_submit(&ctx->service, &request);
        if ((status != IC_CARD_OK) && (request.done_cb != NULL)) {
            request.done_cb(
                request.user_ctx, request.request_id, status, NULL);
        }
    }
}

static void ic_service_worker(void *argument)
{
    ic_service_context_t *ctx = (ic_service_context_t *)argument;

    for (;;) {
        ic_service_drain(ctx);
        ic_card_service_process_once(&ctx->service);
        (void)osThreadFlagsWait(
            IC_SERVICE_FLAG_REQUEST | IC_SERVICE_FLAG_ISR_EVENT,
            osFlagsWaitAny,
            ic_service_ticks(IC_SERVICE_WAIT_SLICE_MS));
    }
}

ic_card_status_t ic_service_init(void)
{
    ic_service_context_t *ctx = &g_ic_service;
    ic_card_uart7_hal_config_t hal_config;
    ic_card_service_config_t service_config;
    uart_dispatch_handler_t handler = {0};
    ic_card_status_t status;

    if (ctx->initialized) return IC_CARD_ERR_STATE;
    (void)memset(ctx, 0, sizeof(*ctx));
    ctx->dispatch_handle = UART_DISPATCH_HANDLE_INVALID;
    ctx->request_queue = osMessageQueueNew(
        IC_SERVICE_QUEUE_DEPTH, sizeof(ic_card_request_t), NULL);
    if (ctx->request_queue == NULL) return IC_CARD_ERR_IO;

    ic_uart7_config(&hal_config);
    status = ic_uart7_bind(
        &ctx->adapter, &ctx->device, &hal_config, &ctx->port);
    if (status != IC_CARD_OK) {
        ic_service_rollback(ctx);
        return status;
    }

    handler.tx_complete = ic_service_tx_isr;
    handler.rx_event = ic_service_rx_isr;
    handler.error = ic_service_error_isr;
    handler.user_ctx = ctx;
    if (!uart_dispatch_register(&handler, &ctx->dispatch_handle)) {
        ic_service_rollback(ctx);
        return IC_CARD_ERR_IO;
    }
    ctx->dispatch_registered = true;

    status = ic_bsp_init(&ctx->device, &ctx->port);
    if (status != IC_CARD_OK) {
        ic_service_rollback(ctx);
        return status;
    }

    (void)memset(&service_config, 0, sizeof(service_config));
    service_config.device = &ctx->device;
    service_config.now_ms = ic_service_now;
    service_config.notify_worker = ic_service_notify;
    service_config.notify_ctx = ctx;
    status = ic_card_service_init(&ctx->service, &service_config);
    if (status != IC_CARD_OK) {
        ic_service_rollback(ctx);
        return status;
    }

    ctx->worker_thread = osThreadNew(
        ic_service_worker, ctx, &g_ic_worker_attr);
    if (ctx->worker_thread == NULL) {
        ic_service_rollback(ctx);
        return IC_CARD_ERR_IO;
    }
    ctx->initialized = true;
    return IC_CARD_OK;
}

ic_card_status_t ic_service_submit(
    const ic_card_request_t *request,
    uint32_t queue_timeout_ms)
{
    osStatus_t status;

    if (request == NULL) return IC_CARD_ERR_PARAM;
    if (!g_ic_service.initialized) return IC_CARD_ERR_NOT_INIT;
    status = osMessageQueuePut(
        g_ic_service.request_queue,
        request,
        0U,
        ic_service_ticks(queue_timeout_ms));
    if (status != osOK) {
        return ((status == osErrorResource) || (status == osErrorTimeout)) ?
            IC_CARD_ERR_QUEUE_FULL : IC_CARD_ERR_IO;
    }
    (void)osThreadFlagsSet(
        g_ic_service.worker_thread, IC_SERVICE_FLAG_REQUEST);
    return IC_CARD_OK;
}

void ic_service_process(void)
{
    if (g_ic_service.initialized) {
        ic_service_drain(&g_ic_service);
        ic_card_service_process_once(&g_ic_service.service);
    }
}

ic_card_status_t ic_service_get_stats(ic_card_service_stats_t *stats)
{
    if (!g_ic_service.initialized) return IC_CARD_ERR_NOT_INIT;
    return ic_card_service_get_stats(&g_ic_service.service, stats);
}
#endif

/**
 * @brief 把当前活动请求翻译为Core命令并启动事务超时计时。
 * @param service 已装载active请求的Service对象。
 * @return Core命令启动状态。
 * @note 请求启动后直到匹配响应、错误或超时前独占该直连UART7。
 */
static ic_card_status_t ic_card_service_start_active(
    ic_card_service_t *service)
{
    ic_card_status_t status;

    if (service->active.type == IC_CARD_REQUEST_READ_BLOCK) {
        status = ic_bsp_read(
            service->config.device,
            service->active.address,
            service->active.data.read_block.block,
            service->active.data.read_block.led_beep_prompt);
    } else {
        status = ic_bsp_query(
            service->config.device,
            service->active.data.query.command,
            service->active.address);
    }
    if (status == IC_CARD_OK) {
        service->active_deadline_ms = service->config.now_ms(
            service->config.time_ctx) + service->active.timeout_ms;
    }
    return status;
}

/**
 * @brief 初始化平台无关IC卡Service并绑定Core ISR通知。
 * @param service 待初始化Service对象。
 * @param config Core对象、时钟和worker通知配置。
 * @return 初始化和回调绑定结果。
 */
ic_card_status_t ic_card_service_init(
    ic_card_service_t *service,
    const ic_card_service_config_t *config)
{
    if ((service == NULL) || (config == NULL) || (config->device == NULL) ||
        (config->now_ms == NULL) || !config->device->initialized) {
        return IC_CARD_ERR_PARAM;
    }
    if (service->initialized) {
        return IC_CARD_ERR_STATE;
    }
    (void)memset(service, 0, sizeof(*service));
    service->config = *config;
    service->response_sequence = config->device->response_sequence;
    service->initialized = true;
    return ic_bsp_bind_notify(
        config->device,
        ic_card_service_isr_notify,
        service);
}

/**
 * @brief 在空闲状态解除Service与Core的事件绑定。
 * @param service Service对象。
 * @return 成功返回IC_CARD_OK；存在活动或排队请求时返回BUSY。
 */
ic_card_status_t ic_card_service_deinit(ic_card_service_t *service)
{
    if (service == NULL) {
        return IC_CARD_ERR_PARAM;
    }
    if (!service->initialized) {
        return IC_CARD_ERR_NOT_INIT;
    }
    if (service->active_valid || (service->queue_count > 0U)) {
        return IC_CARD_ERR_BUSY;
    }
    (void)ic_bsp_bind_notify(service->config.device, NULL, NULL);
    (void)memset(service, 0, sizeof(*service));
    return IC_CARD_OK;
}

/**
 * @brief 按值复制一笔请求到平台无关静态队列。
 * @param service Service对象。
 * @param request 待提交请求。
 * @return 请求入队状态。
 * @note 完整复制请求结构，执行期间不再依赖调用者的临时请求对象。
 */
ic_card_status_t ic_card_service_submit(
    ic_card_service_t *service,
    const ic_card_request_t *request)
{
    if ((service == NULL) || !service->initialized) {
        return (service == NULL) ? IC_CARD_ERR_PARAM : IC_CARD_ERR_NOT_INIT;
    }
    if (!ic_card_request_is_valid(request)) {
#if IC_CARD_SERVICE_DIAGNOSTICS_ENABLE
        ++service->stats.rejected;
#endif
        return IC_CARD_ERR_PARAM;
    }
    if (service->queue_count >= IC_CARD_SERVICE_QUEUE_DEPTH) {
#if IC_CARD_SERVICE_DIAGNOSTICS_ENABLE
        ++service->stats.rejected;
#endif
        return IC_CARD_ERR_QUEUE_FULL;
    }
    service->queue[service->queue_head] = *request;
    service->queue_head = (uint8_t)(
        (service->queue_head + 1U) % IC_CARD_SERVICE_QUEUE_DEPTH);
    ++service->queue_count;
#if IC_CARD_SERVICE_DIAGNOSTICS_ENABLE
    ++service->stats.submitted;
#endif
    return IC_CARD_OK;
}

/**
 * @brief 在普通上下文推进一次响应、错误、超时和队列状态机。
 * @param service Service对象。
 * @note UART错误优先于完成事件；任何用户回调和协议匹配都不会在ISR中执行。
 */
void ic_card_service_process_once(ic_card_service_t *service)
{
    ic_card_response_t response;
    ic_card_status_t status;
    uint32_t now;

    if ((service == NULL) || !service->initialized) {
        return;
    }

    ic_bsp_process(service->config.device);

    /* UART错误优先，避免同一轮同时到达的旧TX/RX事件误完成请求。 */
    if (service->handled_error_sequence != service->error_sequence) {
        service->handled_error_sequence = service->error_sequence;
        service->handled_tx_sequence = service->tx_sequence;
        service->handled_rx_sequence = service->rx_sequence;
#if IC_CARD_SERVICE_DIAGNOSTICS_ENABLE
        ++service->stats.uart_errors;
#endif
        status = ic_bsp_recover(service->config.device);
        ic_card_service_complete(
            service,
            (status == IC_CARD_OK) ? IC_CARD_ERR_IO : status,
            NULL);
    } else {
        if (service->handled_tx_sequence != service->tx_sequence) {
            service->handled_tx_sequence = service->tx_sequence;
            service->tx_completed = true;
        }
        if (service->handled_rx_sequence != service->rx_sequence) {
            service->handled_rx_sequence = service->rx_sequence;
        }

        while (ic_bsp_take_response(
            service->config.device,
            &service->response_sequence,
            &response)) {
            if (service->active_valid &&
                ic_card_response_matches(&service->active, &response)) {
                status = (response.device_status == 0U) ?
                    IC_CARD_OK : IC_CARD_ERR_CARD;
                ic_card_service_complete(service, status, &response);
            } else {
#if IC_CARD_SERVICE_DIAGNOSTICS_ENABLE
                ++service->stats.unrelated_responses;
#endif
            }
        }
    }

    if (service->active_valid) {
        now = service->config.now_ms(service->config.time_ctx);
        if (ic_card_time_reached(now, service->active_deadline_ms)) {
#if IC_CARD_SERVICE_DIAGNOSTICS_ENABLE
            ++service->stats.timed_out;
#endif
            status = ic_bsp_recover(service->config.device);
            ic_card_service_complete(
                service,
                (status == IC_CARD_OK) ? IC_CARD_ERR_TIMEOUT : status,
                NULL);
        }
    }

    if (service->active_valid || (service->queue_count == 0U)) {
        return;
    }
    service->active = service->queue[service->queue_tail];
    service->queue_tail = (uint8_t)(
        (service->queue_tail + 1U) % IC_CARD_SERVICE_QUEUE_DEPTH);
    --service->queue_count;
    service->active_valid = true;
    service->tx_completed = false;
    status = ic_card_service_start_active(service);
    if (status != IC_CARD_OK) {
        ic_card_service_complete(service, status, NULL);
    }
}

/**
 * @brief 获取IC卡Service统计快照。
 * @param service Service对象。
 * @param stats 接收统计值的输出对象。
 * @return 获取结果。
 */
#if IC_CARD_SERVICE_DIAGNOSTICS_ENABLE
ic_card_status_t ic_card_service_get_stats(
    const ic_card_service_t *service,
    ic_card_service_stats_t *stats)
{
    if ((service == NULL) || (stats == NULL)) {
        return IC_CARD_ERR_PARAM;
    }
    if (!service->initialized) {
        return IC_CARD_ERR_NOT_INIT;
    }
    *stats = service->stats;
    return IC_CARD_OK;
}
#endif
