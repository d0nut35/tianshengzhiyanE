/**
 * @file    mult_uart_baremetal_test.c
 * @brief   UART7复用板四通道CH340裸机验收实现。
 *
 * ISR只经由uart_dispatch和HAL adapter向Core报告事件，本文件的Core
 * callback只置位标志。命令解析、通道切换和文本格式化全部在
 * main普通上下文的process()中完成。
 */

#include "mult_uart_baremetal_test.h"

#include <string.h>

#include "main.h"
#include "mult_uart_board_config.h"
#include "mult_uart_stm32_hal.h"
#include "mult_uart_test_protocol.h"
#include "test_config.h"
#include "uart_dispatch.h"

#if MULT_UART_BAREMETAL_TEST_ENABLED

typedef enum {
    MULT_UART_BAREMETAL_PENDING_NONE = 0,
    MULT_UART_BAREMETAL_PENDING_ECHO,
    MULT_UART_BAREMETAL_PENDING_INVALID,
    MULT_UART_BAREMETAL_PENDING_SWITCH,
} mult_uart_baremetal_pending_t;

typedef struct {
    bool initialized;
    bool running;
    mult_uart_bus_t bus;
    mult_uart_port_t port;
    mult_uart_stm32_hal_t hal_adapter;
    uart_dispatch_handle_t dispatch_handle;
    bool dispatch_registered;

    volatile bool tx_done_event;
    volatile bool rx_done_event;
    volatile bool error_event;
    volatile size_t isr_rx_len;
    volatile mult_uart_status_t isr_error_status;

    bool tx_in_flight;
    bool rx_in_flight;
    mult_uart_baremetal_pending_t pending;
    uint8_t pending_channel;
    uint8_t current_channel;
    uint8_t rx_buffer[MULT_UART_TEST_RX_CAPACITY];
    size_t received_len;
    uint8_t tx_buffer[MULT_UART_TEST_TX_CAPACITY];
    uint32_t next_announcement_ms;

    uint32_t tx_count;
    uint32_t rx_count;
    uint32_t switch_count;
    uint32_t error_count;
    mult_uart_status_t last_status;
} mult_uart_baremetal_test_context_t;

typedef struct {
    bool tx_done;
    bool rx_done;
    bool error;
    size_t rx_len;
    mult_uart_status_t error_status;
} mult_uart_baremetal_event_snapshot_t;

static mult_uart_baremetal_test_context_t g_mult_uart_baremetal_test;

/**
 * @brief 判断32位毫秒期限是否到达。
 * @param now 当前毫秒计数。
 * @param deadline 截止时间。
 * @return 已到期返回true。
 */
static bool mult_uart_baremetal_time_reached(
    uint32_t now,
    uint32_t deadline)
{
    return ((int32_t)(now - deadline) >= 0);
}

/**
 * @brief 在ISR中记录复用Core事件供主循环消费。
 * @param user_ctx 裸机测试上下文。
 * @param event Core事件。
 * @warning 只复制小型事件和递增序号，禁止在ISR中格式化或重新提交DMA。
 */
static void mult_uart_baremetal_core_event(
    void *user_ctx,
    const mult_uart_event_t *event)
{
    mult_uart_baremetal_test_context_t *ctx =
        (mult_uart_baremetal_test_context_t *)user_ctx;

    if ((ctx == NULL) || (event == NULL)) {
        return;
    }

    if (event->type == MULT_UART_EVENT_TX_COMPLETE) {
        ctx->tx_done_event = true;
    } else if (event->type == MULT_UART_EVENT_RX_COMPLETE) {
        ctx->isr_rx_len = event->rx_len;
        ctx->rx_done_event = true;
    } else {
        ctx->isr_error_status =
            (event->status == MULT_UART_OK) ?
                MULT_UART_ERR_IO : event->status;
        ctx->error_event = true;
    }
}

/**
 * @brief 把公共路由TX完成事件转交复用HAL适配器。
 * @param user_ctx 裸机测试上下文。
 * @param huart 产生回调的UART句柄。
 * @return 事件被认领时返回true。
 */
static bool mult_uart_baremetal_dispatch_tx(
    void *user_ctx,
    UART_HandleTypeDef *huart)
{
    mult_uart_baremetal_test_context_t *ctx =
        (mult_uart_baremetal_test_context_t *)user_ctx;

    return (ctx != NULL) && mult_uart_stm32_hal_handle_tx_complete(
        &ctx->hal_adapter,
        huart);
}

/**
 * @brief 把公共路由ReceiveToIdle事件转交复用HAL适配器。
 * @param user_ctx 裸机测试上下文。
 * @param huart 产生回调的UART句柄。
 * @param rx_len 本次有效字节数。
 * @return 事件被认领时返回true。
 */
static bool mult_uart_baremetal_dispatch_rx(
    void *user_ctx,
    UART_HandleTypeDef *huart,
    uint16_t rx_len)
{
    mult_uart_baremetal_test_context_t *ctx =
        (mult_uart_baremetal_test_context_t *)user_ctx;

    return (ctx != NULL) && mult_uart_stm32_hal_handle_rx_event(
        &ctx->hal_adapter,
        huart,
        rx_len);
}

/**
 * @brief 把公共路由UART错误事件转交复用HAL适配器。
 * @param user_ctx 裸机测试上下文。
 * @param huart 产生错误的UART句柄。
 * @return 事件被认领时返回true。
 */
static bool mult_uart_baremetal_dispatch_error(
    void *user_ctx,
    UART_HandleTypeDef *huart)
{
    mult_uart_baremetal_test_context_t *ctx =
        (mult_uart_baremetal_test_context_t *)user_ctx;

    return (ctx != NULL) && mult_uart_stm32_hal_handle_error(
        &ctx->hal_adapter,
        huart);
}

/**
 * @brief 原子取走ISR标志。
 *
 * 如果在“读标志”和“清标志”之间被UART中断打断，新事件可能被
 * 主循环误清。因此这里只在极短的快照期间关中断，恢复时保留
 * 调用前的PRIMASK状态。
 */
static mult_uart_baremetal_event_snapshot_t
mult_uart_baremetal_take_events(
    mult_uart_baremetal_test_context_t *ctx)
{
    mult_uart_baremetal_event_snapshot_t snapshot;
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    snapshot.tx_done = ctx->tx_done_event;
    snapshot.rx_done = ctx->rx_done_event;
    snapshot.error = ctx->error_event;
    snapshot.rx_len = ctx->isr_rx_len;
    snapshot.error_status = ctx->isr_error_status;
    ctx->tx_done_event = false;
    ctx->rx_done_event = false;
    ctx->error_event = false;
    if (primask == 0U) {
        __enable_irq();
    }

    return snapshot;
}

/**
 * @brief 在当前通道重新挂接裸机测试RX DMA。
 * @param ctx 裸机测试上下文。
 * @return Core接收启动状态。
 */
static mult_uart_status_t mult_uart_baremetal_start_rx(
    mult_uart_baremetal_test_context_t *ctx)
{
    mult_uart_status_t status;

    if (ctx->rx_in_flight) {
        return MULT_UART_OK;
    }

    status = mult_uart_start_rx_dma(
        &ctx->bus,
        ctx->rx_buffer,
        sizeof(ctx->rx_buffer));
    if (status == MULT_UART_OK) {
        ctx->rx_in_flight = true;
    }
    return status;
}

/**
 * @brief 发送已在上下文TX缓冲中构造的测试文本。
 * @param ctx 裸机测试上下文。
 * @param tx_len 有效发送长度。
 * @return Core发送启动状态。
 * @note 发送完成前上下文TX缓冲不会被下一条消息覆盖。
 */
static mult_uart_status_t mult_uart_baremetal_send(
    mult_uart_baremetal_test_context_t *ctx,
    size_t tx_len)
{
    mult_uart_status_t status;

    if ((tx_len == 0U) || ctx->tx_in_flight) {
        return MULT_UART_ERR_STATE;
    }

    /*
     * 回显和提示文本发送期间仍保持RX DMA，使UART7全双工接收窗口
     * 尽可能连续。如果TX启动失败，不继续伪装运行，交给统一错误
     * 停止路径中止DMA并保留last_status，便于查看真实首发故障。
     */
    status = mult_uart_baremetal_start_rx(ctx);
    if (status != MULT_UART_OK) {
        return status;
    }

    status = mult_uart_start_tx_dma(&ctx->bus, ctx->tx_buffer, tx_len);
    if (status == MULT_UART_OK) {
        ctx->tx_in_flight = true;
        ctx->tx_count++;
    }
    return status;
}

/**
 * @brief 记录失败并停止裸机测试继续发起新事务。
 * @param ctx 裸机测试上下文。
 * @param status 失败原因。
 */
static void mult_uart_baremetal_mark_failed(
    mult_uart_baremetal_test_context_t *ctx,
    mult_uart_status_t status)
{
    (void)mult_uart_abort(&ctx->bus);
    ctx->tx_in_flight = false;
    ctx->rx_in_flight = false;
    ctx->pending = MULT_UART_BAREMETAL_PENDING_NONE;
    ctx->running = false;
    ctx->last_status = status;
    ctx->error_count++;
}

/**
 * @brief 安全切换测试通道并准备立即提示文本。
 * @param ctx 裸机测试上下文。
 * @param channel 目标通道0~3。
 * @return 通道切换结果。
 */
static mult_uart_status_t mult_uart_baremetal_select_channel(
    mult_uart_baremetal_test_context_t *ctx,
    uint8_t channel)
{
    mult_uart_status_t status;

    /* 选通道要求Core处于IDLE，因此先确保无在途TX/RX。 */
    status = mult_uart_abort(&ctx->bus);
    if (status != MULT_UART_OK) {
        return status;
    }
    ctx->tx_in_flight = false;
    ctx->rx_in_flight = false;

    status = mult_uart_select(
        &ctx->bus,
        (mult_uart_channel_t)channel);
    if (status != MULT_UART_OK) {
        return status;
    }

    status = mult_uart_enable(&ctx->bus);
    if (status == MULT_UART_OK) {
        if (ctx->current_channel != channel) {
            ctx->switch_count++;
        }
        ctx->current_channel = channel;
    }
    return status;
}

/**
 * @brief 解析本次PC输入并安排切换提示或HEX回显。
 * @param ctx 裸机测试上下文。
 * @param rx_len 本次有效输入长度。
 */
static void mult_uart_baremetal_handle_received(
    mult_uart_baremetal_test_context_t *ctx,
    size_t rx_len)
{
    mult_uart_test_input_t input;

    ctx->rx_in_flight = false;
    ctx->rx_count++;
    if ((rx_len == 0U) || (rx_len > sizeof(ctx->rx_buffer))) {
        mult_uart_baremetal_mark_failed(ctx, MULT_UART_ERR_OVERFLOW);
        return;
    }

    ctx->received_len = rx_len;
    input = mult_uart_test_parse_input(ctx->rx_buffer, rx_len);
    if (input.kind == MULT_UART_TEST_INPUT_SWITCH) {
        ctx->pending = MULT_UART_BAREMETAL_PENDING_SWITCH;
        ctx->pending_channel = input.requested_channel;
    } else if (input.kind == MULT_UART_TEST_INPUT_INVALID_CHANNEL) {
        ctx->pending = MULT_UART_BAREMETAL_PENDING_INVALID;
    } else {
        ctx->pending = MULT_UART_BAREMETAL_PENDING_ECHO;
    }
}

/**
 * @brief 在总线空闲后执行待发送文本或重新挂接接收。
 * @param ctx 裸机测试上下文。
 * @param now 当前毫秒数。
 */
static void mult_uart_baremetal_run_pending(
    mult_uart_baremetal_test_context_t *ctx,
    uint32_t now)
{
    size_t tx_len = 0U;
    mult_uart_status_t status;

    if ((ctx->pending == MULT_UART_BAREMETAL_PENDING_NONE) ||
        ctx->tx_in_flight) {
        return;
    }

    if (ctx->pending == MULT_UART_BAREMETAL_PENDING_SWITCH) {
        status = mult_uart_baremetal_select_channel(
            ctx,
            ctx->pending_channel);
        if (status != MULT_UART_OK) {
            mult_uart_baremetal_mark_failed(ctx, status);
            return;
        }
        tx_len = mult_uart_test_format_announcement(
            ctx->current_channel,
            ctx->tx_buffer,
            sizeof(ctx->tx_buffer));
        ctx->next_announcement_ms = now + MULT_UART_TEST_HEARTBEAT_MS;
    } else if (ctx->pending == MULT_UART_BAREMETAL_PENDING_INVALID) {
        tx_len = mult_uart_test_format_invalid_channel(
            ctx->tx_buffer,
            sizeof(ctx->tx_buffer));
    } else {
        tx_len = mult_uart_test_format_echo(
            ctx->current_channel,
            ctx->rx_buffer,
            ctx->received_len,
            ctx->tx_buffer,
            sizeof(ctx->tx_buffer));
    }

    ctx->pending = MULT_UART_BAREMETAL_PENDING_NONE;
    status = mult_uart_baremetal_send(ctx, tx_len);
    if (status != MULT_UART_OK) {
        mult_uart_baremetal_mark_failed(ctx, status);
    }
}

/**
 * @brief 到期时安排三秒一次的当前通道提示。
 * @param ctx 裸机测试上下文。
 * @param now 当前毫秒数。
 */
static void mult_uart_baremetal_send_periodic(
    mult_uart_baremetal_test_context_t *ctx,
    uint32_t now)
{
    size_t tx_len;
    mult_uart_status_t status;

    if (ctx->tx_in_flight ||
        (ctx->pending != MULT_UART_BAREMETAL_PENDING_NONE) ||
        !mult_uart_baremetal_time_reached(now, ctx->next_announcement_ms)) {
        return;
    }

    tx_len = mult_uart_test_format_announcement(
        ctx->current_channel,
        ctx->tx_buffer,
        sizeof(ctx->tx_buffer));
    status = mult_uart_baremetal_send(ctx, tx_len);
    if (status != MULT_UART_OK) {
        mult_uart_baremetal_mark_failed(ctx, status);
        return;
    }

    /* 以旧deadline递增而不是简单now+3000，避免主循环抖动长期累积。 */
    do {
        ctx->next_announcement_ms += MULT_UART_TEST_HEARTBEAT_MS;
    } while (mult_uart_baremetal_time_reached(
        now,
        ctx->next_announcement_ms));
}

/**
 * @brief 按初始化逆序撤销裸机测试资源。
 * @param ctx 裸机测试上下文。
 */
static void mult_uart_baremetal_rollback(
    mult_uart_baremetal_test_context_t *ctx)
{
    if (ctx->bus.initialized) {
        (void)mult_uart_abort(&ctx->bus);
    }
    if (ctx->dispatch_registered) {
        (void)uart_dispatch_unregister(ctx->dispatch_handle);
    }
    if (ctx->bus.initialized) {
        (void)mult_uart_deinit(&ctx->bus);
    }
    (void)memset(ctx, 0, sizeof(*ctx));
}

/**
 * @brief 装配UART7四通道CH340裸机验收测试。
 * @return 初始化结果。
 * @note 上电选择通道0并立即提示，但不会启动其他模块或正式App。
 */
mult_uart_status_t mult_uart_baremetal_test_init(void)
{
    mult_uart_baremetal_test_context_t *ctx =
        &g_mult_uart_baremetal_test;
    mult_uart_stm32_hal_config_t hal_config;
    mult_uart_config_t core_config = {
        (MULT_UART_BOARD_MANAGE_ENABLE != 0),
        (MULT_UART_BOARD_ENABLE_ACTIVE_LOW != 0),
        (MULT_UART_BOARD_BREAK_BEFORE_SWITCH != 0),
        MULT_UART_BOARD_SWITCH_SETTLE_US,
    };
    uart_dispatch_handler_t dispatch_handler = {0};
    mult_uart_status_t status;
    size_t tx_len;

    if (ctx->initialized) {
        return MULT_UART_ERR_STATE;
    }

    (void)memset(ctx, 0, sizeof(*ctx));
    ctx->dispatch_handle = UART_DISPATCH_HANDLE_INVALID;
    ctx->current_channel = 0U;

    mult_uart_stm32_hal_make_uart7_config(&hal_config);
    status = mult_uart_stm32_hal_bind(
        &ctx->hal_adapter,
        &ctx->bus,
        &hal_config,
        &ctx->port);
    if (status != MULT_UART_OK) {
        mult_uart_baremetal_rollback(ctx);
        return status;
    }

    status = mult_uart_init(&ctx->bus, &core_config, &ctx->port);
    if (status != MULT_UART_OK) {
        mult_uart_baremetal_rollback(ctx);
        return status;
    }

    status = mult_uart_bind_event(
        &ctx->bus,
        mult_uart_baremetal_core_event,
        ctx);
    if (status != MULT_UART_OK) {
        mult_uart_baremetal_rollback(ctx);
        return status;
    }

    dispatch_handler.tx_complete = mult_uart_baremetal_dispatch_tx;
    dispatch_handler.rx_event = mult_uart_baremetal_dispatch_rx;
    dispatch_handler.error = mult_uart_baremetal_dispatch_error;
    dispatch_handler.user_ctx = ctx;
    if (!uart_dispatch_register(&dispatch_handler, &ctx->dispatch_handle)) {
        mult_uart_baremetal_rollback(ctx);
        return MULT_UART_ERR_IO;
    }
    ctx->dispatch_registered = true;

    status = mult_uart_baremetal_select_channel(ctx, 0U);
    if (status != MULT_UART_OK) {
        mult_uart_baremetal_rollback(ctx);
        return status;
    }

    tx_len = mult_uart_test_format_announcement(
        0U,
        ctx->tx_buffer,
        sizeof(ctx->tx_buffer));
    status = mult_uart_baremetal_send(ctx, tx_len);
    if (status != MULT_UART_OK) {
        mult_uart_baremetal_rollback(ctx);
        return status;
    }

    ctx->next_announcement_ms = HAL_GetTick() +
                                MULT_UART_TEST_HEARTBEAT_MS;
    ctx->last_status = MULT_UART_OK;
    ctx->running = true;
    ctx->initialized = true;
    return MULT_UART_OK;
}

/**
 * @brief 在主循环推进DMA事件、输入解析和周期提示状态机。
 * @note 必须被非阻塞地高频调用。
 */
void mult_uart_baremetal_test_process(void)
{
    mult_uart_baremetal_test_context_t *ctx =
        &g_mult_uart_baremetal_test;
    mult_uart_baremetal_event_snapshot_t events;
    uint32_t now;

    if (!ctx->initialized || !ctx->running) {
        return;
    }

    events = mult_uart_baremetal_take_events(ctx);
    if (events.tx_done) {
        ctx->tx_in_flight = false;
    }
    if (events.rx_done) {
        mult_uart_baremetal_handle_received(ctx, events.rx_len);
    }
    if (events.error) {
        mult_uart_baremetal_mark_failed(ctx, events.error_status);
        return;
    }

    now = HAL_GetTick();
    mult_uart_baremetal_run_pending(ctx, now);
    if (!ctx->running) {
        return;
    }
    mult_uart_baremetal_send_periodic(ctx, now);
}

/**
 * @brief 获取复用裸机测试状态快照。
 * @param status 接收快照的输出对象。
 * @return 获取结果。
 */
mult_uart_status_t mult_uart_baremetal_test_get_status(
    mult_uart_baremetal_test_status_t *status)
{
    mult_uart_baremetal_test_context_t *ctx =
        &g_mult_uart_baremetal_test;
    uint32_t primask;

    if (status == NULL) {
        return MULT_UART_ERR_PARAM;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    status->initialized = ctx->initialized;
    status->running = ctx->running;
    status->current_channel = ctx->current_channel;
    status->tx_count = ctx->tx_count;
    status->rx_count = ctx->rx_count;
    status->switch_count = ctx->switch_count;
    status->error_count = ctx->error_count;
    status->last_status = ctx->last_status;
    if (primask == 0U) {
        __enable_irq();
    }

    return ctx->initialized ? MULT_UART_OK : MULT_UART_ERR_NOT_INIT;
}

#else

/**
 * @brief 测试未启用时的空初始化实现。
 * @return 固定返回MULT_UART_ERR_UNSUPPORTED。
 */
mult_uart_status_t mult_uart_baremetal_test_init(void)
{
    return MULT_UART_ERR_UNSUPPORTED;
}

/**
 * @brief 测试未启用时的空处理函数。
 * @note 保留同名接口可避免main.c因测试开关变化而增加条件编译分支。
 */
void mult_uart_baremetal_test_process(void)
{
}

/**
 * @brief 测试未启用时的状态查询实现。
 * @param status 输出对象；若非NULL会被清零。
 * @return 固定返回MULT_UART_ERR_UNSUPPORTED。
 */
mult_uart_status_t mult_uart_baremetal_test_get_status(
    mult_uart_baremetal_test_status_t *status)
{
    (void)status;
    return MULT_UART_ERR_UNSUPPORTED;
}

#endif /* MULT_UART_BAREMETAL_TEST_ENABLED */
