/**
 * @file    mult_uart_freertos_test.c
 * @brief   UART7复用板四通道CH340 FreeRTOS验收实现。
 *
 * 本测试不直接访问HAL、DMA或A/B/EN，而是经由Device -> Service OS
 * -> Service -> Core -> F7 adapter完成真实收发，用来验证完整RTOS链路。
 *
 * 为了在发送提示/回显的同时保持接收窗口，每笔都使用
 * WRITE_READ：Service先启动RX DMA，再启动TX DMA。其I/O超时被设置为
 * 距离下一个3秒提示的剩余时间，因此普通回显不会不断推迟周期提示。
 */

#include "mult_uart_freertos_test.h"

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "cmsis_os.h"
#include "mult_uart_device.h"
#include "mult_uart_test_protocol.h"
#include "test_config.h"

#if MULT_UART_FREERTOS_TEST_ENABLED

#define MULT_UART_FREERTOS_EVENT_QUEUE_DEPTH     4U
#define MULT_UART_FREERTOS_TASK_STACK            (256U * 4U)
#define MULT_UART_FREERTOS_RETRY_MS               10U

typedef struct {
    mult_uart_status_t status;
    size_t rx_len;
    uint8_t rx_data[MULT_UART_TEST_RX_CAPACITY];
} mult_uart_freertos_completion_event_t;

typedef struct {
    bool initialized;
    uint8_t current_channel;
    uint8_t tx_buffer[MULT_UART_TEST_TX_CAPACITY];
    uint32_t next_announcement_ms;
    osMessageQueueId_t completion_queue;
    osThreadId_t task;

    /* 这些计数可直接放入Keil Watch，不参与控制逻辑。 */
    volatile uint32_t tx_count;
    volatile uint32_t rx_count;
    volatile uint32_t switch_count;
    volatile uint32_t error_count;
    volatile uint32_t dropped_completion;
    volatile mult_uart_status_t last_status;
} mult_uart_freertos_test_context_t;

/**
 * @brief 复用模块FreeRTOS验收任务入口，定义位于下方。
 * @param argument FreeRTOS测试上下文。
 */
static void mult_uart_freertos_task_entry(void *argument);

static mult_uart_freertos_test_context_t g_mult_uart_freertos_test;

static const osThreadAttr_t g_mult_uart_freertos_task_attr = {
    .name = "muxTest",
    .stack_size = MULT_UART_FREERTOS_TASK_STACK,
    .priority = (osPriority_t)osPriorityNormal,
};

/**
 * @brief 把RTOS tick换算为测试状态机使用的毫秒数。
 * @return 当前毫秒数，溢出时饱和。
 */
static uint32_t mult_uart_freertos_now_ms(void)
{
    uint32_t frequency = osKernelGetTickFreq();
    uint64_t value;

    if (frequency == 0U) {
        return 0U;
    }

    value = ((uint64_t)osKernelGetTickCount() * 1000ULL) /
            (uint64_t)frequency;
    return (value > UINT32_MAX) ? UINT32_MAX : (uint32_t)value;
}

/**
 * @brief 判断32位毫秒期限是否已经到达。
 * @param now 当前毫秒数。
 * @param deadline 截止时间。
 * @return 已到期返回true。
 */
static bool mult_uart_freertos_time_reached(
    uint32_t now,
    uint32_t deadline)
{
    return ((int32_t)(now - deadline) >= 0);
}

/**
 * @brief 计算距截止时间的剩余毫秒数。
 * @param now 当前毫秒数。
 * @param deadline 截止时间。
 * @return 已到期返回0，否则返回剩余时间。
 */
static uint32_t mult_uart_freertos_remaining_ms(
    uint32_t now,
    uint32_t deadline)
{
    uint32_t remaining;

    if (mult_uart_freertos_time_reached(now, deadline)) {
        return 1U;
    }

    remaining = deadline - now;
    return (remaining == 0U) ? 1U : remaining;
}

/**
 * @brief Device完成回调仍在mult_uart worker任务中，只复制数据并投递事件。
 *
 * @brief 将Device完成事件按值复制到测试事件队列。
 * @param user_ctx FreeRTOS测试上下文。
 * @param completion Device完成信息。
 * @warning completion->rx_data只在本回调期间有效，必须在返回前复制，
 *          不能只把指针交给测试任务。
 */
static void mult_uart_freertos_device_done(
    void *user_ctx,
    const mult_uart_device_completion_t *completion)
{
    mult_uart_freertos_test_context_t *ctx =
        (mult_uart_freertos_test_context_t *)user_ctx;
    mult_uart_freertos_completion_event_t event;

    if ((ctx == NULL) || (completion == NULL) ||
        (ctx->completion_queue == NULL)) {
        return;
    }

    (void)memset(&event, 0, sizeof(event));
    event.status = completion->status;
    if ((completion->status == MULT_UART_OK) &&
        (completion->rx_data != NULL)) {
        if (completion->rx_len > sizeof(event.rx_data)) {
            event.status = MULT_UART_ERR_OVERFLOW;
        } else {
            event.rx_len = completion->rx_len;
            if (completion->rx_len > 0U) {
                (void)memcpy(
                    event.rx_data,
                    completion->rx_data,
                    completion->rx_len);
            }
        }
    }

    if (osMessageQueuePut(
            ctx->completion_queue,
            &event,
            0U,
            0U) != osOK) {
        /*
         * 同一时刻只有一笔测试事务，正常不应填满4深度队列。
         * 若这里增长，说明测试任务被长时间阻塞或出现了重复完成。
         */
        ctx->dropped_completion++;
    }
}

/**
 * @brief 提交一次当前通道的WRITE_READ测试事务。
 * @param ctx FreeRTOS测试上下文。
 * @param tx_len 待发送文本长度。
 * @param io_timeout_ms 本次I/O超时。
 * @return Device提交状态。
 */
static mult_uart_status_t mult_uart_freertos_submit(
    mult_uart_freertos_test_context_t *ctx,
    size_t tx_len,
    uint32_t io_timeout_ms)
{
    mult_uart_device_transfer_t transfer;
    mult_uart_status_t status;

    if ((tx_len == 0U) || (io_timeout_ms == 0U)) {
        return MULT_UART_ERR_PARAM;
    }

    (void)memset(&transfer, 0, sizeof(transfer));
    transfer.device_id =
        (mult_uart_device_id_t)ctx->current_channel;
    transfer.operation = MULT_UART_OP_WRITE_READ;
    transfer.tx_data = ctx->tx_buffer;
    transfer.tx_len = tx_len;
    transfer.rx_capacity = MULT_UART_TEST_RX_CAPACITY;
    transfer.io_timeout_ms = io_timeout_ms;
    transfer.queue_timeout_ms = 0U;
    transfer.done_cb = mult_uart_freertos_device_done;
    transfer.user_ctx = ctx;

    status = mult_uart_device_submit(&transfer);
    ctx->last_status = status;
    if (status == MULT_UART_OK) {
        ctx->tx_count++;
    }
    return status;
}

/**
 * @brief 在限定时间内重试暂时BUSY或队列满的提交。
 * @param ctx FreeRTOS测试上下文。
 * @param tx_len 待发送长度。
 * @param timeout_ms 重试截止时间。
 * @return 最终提交状态。
 */
static mult_uart_status_t mult_uart_freertos_submit_with_retry(
    mult_uart_freertos_test_context_t *ctx,
    size_t tx_len,
    uint32_t timeout_ms)
{
    mult_uart_status_t status;

    do {
        status = mult_uart_freertos_submit(ctx, tx_len, timeout_ms);
        if ((status == MULT_UART_ERR_BUSY) ||
            (status == MULT_UART_ERR_QUEUE_FULL)) {
            ctx->error_count++;
            (void)osDelay(MULT_UART_FREERTOS_RETRY_MS);
        }
    } while ((status == MULT_UART_ERR_BUSY) ||
             (status == MULT_UART_ERR_QUEUE_FULL));

    return status;
}

/**
 * @brief 不可恢复的提交错误后保留任务和调试变量，不继续乱发DMA。
 *
 * @brief 记录不可恢复错误并停止测试循环。
 * @param ctx FreeRTOS测试上下文。
 * @param status 错误原因。
 * @note 队列满或Device busy会在上一层重试；其他错误应停下供Watch检查。
 */
static void mult_uart_freertos_stop_on_error(
    mult_uart_freertos_test_context_t *ctx,
    mult_uart_status_t status)
{
    ctx->last_status = status;
    ctx->error_count++;
    for (;;) {
        (void)osDelay(1000U);
    }
}

/**
 * @brief 构造当前通道提示并更新下一次周期截止时间。
 * @param ctx FreeRTOS测试上下文。
 * @param now 当前毫秒数。
 * @param reset_period true表示从当前时刻重新计算三秒周期。
 * @return 构造出的文本长度。
 */
static size_t mult_uart_freertos_make_announcement(
    mult_uart_freertos_test_context_t *ctx,
    uint32_t now,
    bool reset_period)
{
    if (reset_period) {
        ctx->next_announcement_ms = now +
                                    MULT_UART_TEST_HEARTBEAT_MS;
    } else {
        do {
            ctx->next_announcement_ms +=
                MULT_UART_TEST_HEARTBEAT_MS;
        } while (mult_uart_freertos_time_reached(
            now,
            ctx->next_announcement_ms));
    }

    return mult_uart_test_format_announcement(
        ctx->current_channel,
        ctx->tx_buffer,
        sizeof(ctx->tx_buffer));
}

/**
 * @brief 处理一笔WRITE_READ收到的PC输入。
 * @param ctx FreeRTOS测试上下文。
 * @param event 已按值复制的完成事件。
 * @param now 当前毫秒数。
 * @return 下一条需要发送的回复长度，0表示无需回复。
 */
static size_t mult_uart_freertos_handle_input(
    mult_uart_freertos_test_context_t *ctx,
    const mult_uart_freertos_completion_event_t *event,
    uint32_t now)
{
    mult_uart_test_input_t input;

    if ((event->rx_len == 0U) ||
        (event->rx_len > sizeof(event->rx_data))) {
        ctx->error_count++;
        return mult_uart_freertos_make_announcement(ctx, now, true);
    }

    ctx->rx_count++;
    input = mult_uart_test_parse_input(event->rx_data, event->rx_len);
    if (input.kind == MULT_UART_TEST_INPUT_SWITCH) {
        if (ctx->current_channel != input.requested_channel) {
            ctx->switch_count++;
        }
        ctx->current_channel = input.requested_channel;
        /* 合法切换后在新通道立即提示，并从此时重新3秒周期。 */
        return mult_uart_freertos_make_announcement(ctx, now, true);
    }

    if (input.kind == MULT_UART_TEST_INPUT_INVALID_CHANNEL) {
        return mult_uart_test_format_invalid_channel(
            ctx->tx_buffer,
            sizeof(ctx->tx_buffer));
    }

    return mult_uart_test_format_echo(
        ctx->current_channel,
        event->rx_data,
        event->rx_len,
        ctx->tx_buffer,
        sizeof(ctx->tx_buffer));
}

/**
 * @brief 复用模块FreeRTOS验收任务入口。
 * @param argument FreeRTOS测试上下文。
 * @note 本任务串行提交事务并消费完成队列，不与其他测试或正式App并发运行。
 */
static void mult_uart_freertos_task_entry(void *argument)
{
    mult_uart_freertos_test_context_t *ctx =
        (mult_uart_freertos_test_context_t *)argument;
    mult_uart_freertos_completion_event_t event;
    uint32_t now;
    uint32_t timeout_ms;
    size_t tx_len;

    now = mult_uart_freertos_now_ms();
    tx_len = mult_uart_freertos_make_announcement(ctx, now, true);
    timeout_ms = MULT_UART_TEST_HEARTBEAT_MS;
    if (mult_uart_freertos_submit_with_retry(
            ctx,
            tx_len,
            timeout_ms) != MULT_UART_OK) {
        mult_uart_freertos_stop_on_error(ctx, ctx->last_status);
    }

    for (;;) {
        if (osMessageQueueGet(
                ctx->completion_queue,
                &event,
                NULL,
                osWaitForever) != osOK) {
            ctx->error_count++;
            continue;
        }

        now = mult_uart_freertos_now_ms();
        if (event.status == MULT_UART_OK) {
            tx_len = mult_uart_freertos_handle_input(ctx, &event, now);
        } else if (event.status == MULT_UART_ERR_TIMEOUT) {
            /* 这是到达周期提示时刻的正常唤醒，不计作通信错误。 */
            tx_len = mult_uart_freertos_make_announcement(ctx, now, false);
        } else {
            /* Service已中止失败事务，下一笔会重新选通道并启动DMA。 */
            ctx->error_count++;
            ctx->last_status = event.status;
            (void)osDelay(MULT_UART_FREERTOS_RETRY_MS);
            now = mult_uart_freertos_now_ms();
            tx_len = mult_uart_freertos_make_announcement(ctx, now, true);
        }

        if (tx_len == 0U) {
            ctx->error_count++;
            tx_len = mult_uart_freertos_make_announcement(ctx, now, true);
        }

        timeout_ms = mult_uart_freertos_remaining_ms(
            now,
            ctx->next_announcement_ms);
        if (mult_uart_freertos_submit_with_retry(
                ctx,
                tx_len,
                timeout_ms) != MULT_UART_OK) {
            mult_uart_freertos_stop_on_error(ctx, ctx->last_status);
        }
    }
}

/**
 * @brief 初始化复用Device层、完成事件队列和独立测试任务。
 * @return 初始化结果。
 */
mult_uart_status_t mult_uart_freertos_test_init(void)
{
    mult_uart_freertos_test_context_t *ctx =
        &g_mult_uart_freertos_test;

    if (ctx->initialized) {
        return MULT_UART_ERR_STATE;
    }

    (void)memset(ctx, 0, sizeof(*ctx));
    ctx->completion_queue = osMessageQueueNew(
        MULT_UART_FREERTOS_EVENT_QUEUE_DEPTH,
        sizeof(mult_uart_freertos_completion_event_t),
        NULL);
    if (ctx->completion_queue == NULL) {
        return MULT_UART_ERR_IO;
    }

    ctx->current_channel = 0U;
    ctx->task = osThreadNew(
        mult_uart_freertos_task_entry,
        ctx,
        &g_mult_uart_freertos_task_attr);
    if (ctx->task == NULL) {
        (void)osMessageQueueDelete(ctx->completion_queue);
        (void)memset(ctx, 0, sizeof(*ctx));
        return MULT_UART_ERR_IO;
    }

    ctx->last_status = MULT_UART_OK;
    ctx->initialized = true;
    return MULT_UART_OK;
}

#else

/**
 * @brief 测试未启用时的空初始化实现。
 * @return 固定返回MULT_UART_ERR_UNSUPPORTED。
 */
mult_uart_status_t mult_uart_freertos_test_init(void)
{
    return MULT_UART_ERR_UNSUPPORTED;
}

#endif /* MULT_UART_FREERTOS_TEST_ENABLED */
