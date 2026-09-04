/**
 * @file    lsc16_baremetal_test.c
 * @brief   USART1命令触发的LSC16 UART8裸机验收测试。
 *
 * 上电只建立UART8和USART1接收窗口，不自动发送运动命令。电脑发送SERVO、
 * ACTION、STOP、BATTERY或STATUS后，主循环才推进对应事务并输出可读结果。
 */

#include "lsc16_baremetal_test.h"

#include <stdio.h>
#include <string.h>

#include "debug_uart1.h"
#include "lsc16_stm32_hal.h"
#include "lsc16_test_common.h"
#include "lsc16_test_config.h"
#include "test_config.h"
#include "uart_dispatch.h"

#if LSC16_BAREMETAL_TEST_ENABLED

typedef struct {
    lsc16_baremetal_test_status_t public_status;
    lsc16_t device;
    lsc16_port_t port;
    lsc16_stm32_hal_t adapter;
    debug_uart1_t debug;
    uart_dispatch_handle_t dispatch_handle;
    bool dispatch_registered;
    volatile bool tx_complete_event;
    volatile bool rx_ready_event;
    volatile bool error_event;
    lsc16_test_command_t pending_command;
    char text[160];
} lsc16_baremetal_context_t;

static lsc16_baremetal_context_t g_lsc16_baremetal;

static void lsc16_baremetal_notify(void *user_ctx, lsc16_isr_event_t event)
{
    lsc16_baremetal_context_t *ctx = (lsc16_baremetal_context_t *)user_ctx;

    if (ctx == NULL) {
        return;
    }
    if (event == LSC16_ISR_EVENT_TX_COMPLETE) {
        ctx->tx_complete_event = true;
    } else if (event == LSC16_ISR_EVENT_RX_READY) {
        ctx->rx_ready_event = true;
    } else {
        ctx->error_event = true;
    }
}

static bool lsc16_baremetal_dispatch_tx(
    void *user_ctx,
    UART_HandleTypeDef *huart)
{
    lsc16_baremetal_context_t *ctx = (lsc16_baremetal_context_t *)user_ctx;
    return (ctx != NULL) && lsc16_stm32_hal_handle_tx_complete(
        &ctx->adapter, huart);
}

static bool lsc16_baremetal_dispatch_rx(
    void *user_ctx,
    UART_HandleTypeDef *huart,
    uint16_t rx_len)
{
    lsc16_baremetal_context_t *ctx = (lsc16_baremetal_context_t *)user_ctx;
    return (ctx != NULL) && lsc16_stm32_hal_handle_rx_event(
        &ctx->adapter, huart, rx_len);
}

static bool lsc16_baremetal_dispatch_error(
    void *user_ctx,
    UART_HandleTypeDef *huart)
{
    lsc16_baremetal_context_t *ctx = (lsc16_baremetal_context_t *)user_ctx;
    return (ctx != NULL) && lsc16_stm32_hal_handle_error(
        &ctx->adapter, huart);
}

static void lsc16_baremetal_rollback(lsc16_baremetal_context_t *ctx)
{
    if (ctx->dispatch_registered) {
        (void)uart_dispatch_unregister(ctx->dispatch_handle);
    }
    if (ctx->device.initialized) {
        (void)lsc16_deinit(&ctx->device);
    }
    debug_uart1_deinit(&ctx->debug);
    (void)memset(ctx, 0, sizeof(*ctx));
}

static void lsc16_baremetal_write_status(lsc16_baremetal_context_t *ctx)
{
    int count = snprintf(
        ctx->text,
        sizeof(ctx->text),
        "STATUS ARMED=%u BUSY=%u SERVO_TX=%lu ACTION_TX=%lu STARTED=%lu COMPLETED=%lu INVALID=%lu LAST=%u\r\n",
        ctx->public_status.motion_armed ? 1U : 0U,
        lsc16_is_tx_busy(&ctx->device) ? 1U : 0U,
        (unsigned long)ctx->public_status.servo_tx_completed,
        (unsigned long)ctx->public_status.action_tx_completed,
        (unsigned long)ctx->public_status.action_started_reports,
        (unsigned long)ctx->public_status.action_completed_reports,
        (unsigned long)ctx->public_status.invalid_reports,
        (unsigned)ctx->public_status.last_status);

    if ((count > 0) && ((size_t)count < sizeof(ctx->text))) {
        (void)debug_uart1_write(
            &ctx->debug, (const uint8_t *)ctx->text, (size_t)count);
    }
}

static void lsc16_baremetal_write_tx_done(lsc16_baremetal_context_t *ctx)
{
    switch (ctx->pending_command) {
    case LSC16_TEST_COMMAND_SERVO:
        ++ctx->public_status.servo_tx_completed;
        (void)debug_uart1_write_text(&ctx->debug,
            "SERVO TX DONE. OBSERVE ACTUAL MOTION\r\n");
        break;
    case LSC16_TEST_COMMAND_ACTION:
        ++ctx->public_status.action_tx_completed;
        (void)debug_uart1_write_text(&ctx->debug,
            "ACTION TX DONE. WAIT ACTION STARTED/COMPLETED\r\n");
        break;
    case LSC16_TEST_COMMAND_STOP:
        (void)debug_uart1_write_text(&ctx->debug,
            "STOP TX DONE. WAIT ACTION STOPPED OR OBSERVE MOTION\r\n");
        break;
    case LSC16_TEST_COMMAND_BATTERY:
        (void)debug_uart1_write_text(&ctx->debug,
            "BATTERY TX DONE. WAIT BATTERY REPORT\r\n");
        break;
    default:
        break;
    }
    ctx->pending_command = LSC16_TEST_COMMAND_INVALID;
    ctx->public_status.phase = LSC16_BAREMETAL_PHASE_IDLE;
}

static void lsc16_baremetal_emit_report(
    lsc16_baremetal_context_t *ctx,
    uint32_t event,
    const lsc16_report_t *report)
{
    size_t text_len = lsc16_test_format_report(
        event, report, ctx->text, sizeof(ctx->text));
    if (text_len > 0U) {
        (void)debug_uart1_write(
            &ctx->debug, (const uint8_t *)ctx->text, text_len);
    }
}

static void lsc16_baremetal_process_reports(lsc16_baremetal_context_t *ctx)
{
    uint32_t events;
    lsc16_report_t report;

    lsc16_process(&ctx->device);
    events = lsc16_take_report_events(&ctx->device);
    if ((events == LSC16_REPORT_EVENT_NONE) ||
        (lsc16_get_last_report(&ctx->device, &report) != LSC16_OK)) {
        return;
    }
    ctx->public_status.last_report = report;
    if ((events & LSC16_REPORT_EVENT_INVALID_FRAME) != 0U) {
        ++ctx->public_status.invalid_reports;
        lsc16_baremetal_emit_report(
            ctx, LSC16_REPORT_EVENT_INVALID_FRAME, &report);
    }
    if ((events & LSC16_REPORT_EVENT_ACTION_STARTED) != 0U) {
        ++ctx->public_status.action_started_reports;
        lsc16_baremetal_emit_report(
            ctx, LSC16_REPORT_EVENT_ACTION_STARTED, &report);
    }
    if ((events & LSC16_REPORT_EVENT_ACTION_STOPPED) != 0U) {
        lsc16_baremetal_emit_report(
            ctx, LSC16_REPORT_EVENT_ACTION_STOPPED, &report);
    }
    if ((events & LSC16_REPORT_EVENT_ACTION_COMPLETED) != 0U) {
        ++ctx->public_status.action_completed_reports;
        lsc16_baremetal_emit_report(
            ctx, LSC16_REPORT_EVENT_ACTION_COMPLETED, &report);
    }
    if ((events & LSC16_REPORT_EVENT_BATTERY_UPDATED) != 0U) {
        lsc16_baremetal_emit_report(
            ctx, LSC16_REPORT_EVENT_BATTERY_UPDATED, &report);
    }
}

static lsc16_status_t lsc16_baremetal_submit_command(
    lsc16_baremetal_context_t *ctx,
    lsc16_test_command_t command)
{
    lsc16_servo_target_t target = {
        LSC16_TEST_SERVO_ID,
        LSC16_TEST_SERVO_POSITION,
    };

    if (command == LSC16_TEST_COMMAND_STATUS) {
        lsc16_baremetal_write_status(ctx);
        return LSC16_OK;
    }
    if (lsc16_is_tx_busy(&ctx->device) ||
        (ctx->pending_command != LSC16_TEST_COMMAND_INVALID)) {
        return LSC16_ERR_BUSY;
    }
    if ((command == LSC16_TEST_COMMAND_SERVO) ||
        (command == LSC16_TEST_COMMAND_ACTION)) {
#if !LSC16_TEST_MOTION_ARMED
        (void)debug_uart1_write_text(&ctx->debug,
            "MOTION LOCKED. SET LSC16_TEST_MOTION_ARMED=1\r\n");
        return LSC16_ERR_UNSUPPORTED;
#endif
    }
    switch (command) {
    case LSC16_TEST_COMMAND_SERVO:
        return lsc16_move_servos(
            &ctx->device, &target, 1U, LSC16_TEST_SERVO_MOVE_TIME_MS);
    case LSC16_TEST_COMMAND_ACTION:
        return lsc16_run_action_group(
            &ctx->device,
            LSC16_TEST_ACTION_GROUP,
            LSC16_TEST_ACTION_REPEAT_COUNT);
    case LSC16_TEST_COMMAND_STOP:
        return lsc16_stop_action_group(&ctx->device);
    case LSC16_TEST_COMMAND_BATTERY:
        return lsc16_request_battery_voltage(&ctx->device);
    default:
        return LSC16_ERR_UNSUPPORTED;
    }
}

lsc16_status_t lsc16_baremetal_test_init(void)
{
    lsc16_baremetal_context_t *ctx = &g_lsc16_baremetal;
    lsc16_stm32_hal_config_t hal_config;
    uart_dispatch_handler_t handler = {0};
    lsc16_status_t status;

    if (ctx->public_status.initialized) {
        return LSC16_ERR_STATE;
    }
    (void)memset(ctx, 0, sizeof(*ctx));
    ctx->dispatch_handle = UART_DISPATCH_HANDLE_INVALID;
    if (!debug_uart1_init(&ctx->debug)) {
        return LSC16_ERR_IO;
    }
    lsc16_stm32_hal_make_uart8_config(&hal_config);
    status = lsc16_stm32_hal_bind(
        &ctx->adapter, &ctx->device, &hal_config, &ctx->port);
    if (status != LSC16_OK) {
        lsc16_baremetal_rollback(ctx);
        return status;
    }
    handler.tx_complete = lsc16_baremetal_dispatch_tx;
    handler.rx_event = lsc16_baremetal_dispatch_rx;
    handler.error = lsc16_baremetal_dispatch_error;
    handler.user_ctx = ctx;
    if (!uart_dispatch_register(&handler, &ctx->dispatch_handle)) {
        lsc16_baremetal_rollback(ctx);
        return LSC16_ERR_IO;
    }
    ctx->dispatch_registered = true;
    status = lsc16_init(&ctx->device, &ctx->port);
    if (status != LSC16_OK) {
        lsc16_baremetal_rollback(ctx);
        return status;
    }
    (void)lsc16_bind_isr_notify(&ctx->device, lsc16_baremetal_notify, ctx);
    ctx->public_status.initialized = true;
    ctx->public_status.motion_armed = (LSC16_TEST_MOTION_ARMED != 0U);
    ctx->public_status.phase = LSC16_BAREMETAL_PHASE_IDLE;
    ctx->public_status.last_status = LSC16_OK;
    (void)debug_uart1_write_text(&ctx->debug,
        "LSC16 BAREMETAL READY. USE SERVO ACTION STOP BATTERY STATUS\r\n");
    return LSC16_OK;
}

void lsc16_baremetal_test_process(void)
{
    lsc16_baremetal_context_t *ctx = &g_lsc16_baremetal;
    uint8_t data[DEBUG_UART1_RX_BUFFER_SIZE];
    size_t data_len;
    lsc16_test_command_t command;
    lsc16_status_t status;

    if (!ctx->public_status.initialized) {
        return;
    }
    if (ctx->error_event) {
        ctx->error_event = false;
        status = lsc16_recover(&ctx->device);
        ctx->public_status.last_status =
            (status == LSC16_OK) ? LSC16_ERR_IO : status;
        ctx->pending_command = LSC16_TEST_COMMAND_INVALID;
        ctx->public_status.phase = LSC16_BAREMETAL_PHASE_ERROR;
        (void)debug_uart1_write_text(&ctx->debug,
            "LSC16 UART ERROR. RECEIVE WINDOW RECOVERED\r\n");
    }
    if (ctx->rx_ready_event) {
        ctx->rx_ready_event = false;
        lsc16_baremetal_process_reports(ctx);
    }
    if (ctx->tx_complete_event) {
        ctx->tx_complete_event = false;
        ctx->public_status.last_status = LSC16_OK;
        lsc16_baremetal_write_tx_done(ctx);
    }
    if (!debug_uart1_take_message(
            &ctx->debug, data, sizeof(data), &data_len)) {
        return;
    }
    command = lsc16_test_parse_command(data, data_len);
    if (command == LSC16_TEST_COMMAND_INVALID) {
        (void)debug_uart1_write_text(&ctx->debug,
            "USE SERVO ACTION STOP BATTERY STATUS\r\n");
        return;
    }
    status = lsc16_baremetal_submit_command(ctx, command);
    ctx->public_status.last_status = status;
    if ((status == LSC16_OK) && (command != LSC16_TEST_COMMAND_STATUS)) {
        ctx->pending_command = command;
        ctx->public_status.phase = LSC16_BAREMETAL_PHASE_TX;
        (void)snprintf(ctx->text, sizeof(ctx->text),
            "%s REQUEST START\r\n", lsc16_test_command_name(command));
        (void)debug_uart1_write_text(&ctx->debug, ctx->text);
    } else if (status == LSC16_ERR_BUSY) {
        (void)debug_uart1_write_text(&ctx->debug, "LSC16 BUSY\r\n");
    } else if ((status != LSC16_OK) &&
               (status != LSC16_ERR_UNSUPPORTED)) {
        (void)snprintf(ctx->text, sizeof(ctx->text),
            "LSC16 SUBMIT ERROR %u\r\n", (unsigned)status);
        (void)debug_uart1_write_text(&ctx->debug, ctx->text);
    }
}

lsc16_status_t lsc16_baremetal_test_get_status(
    lsc16_baremetal_test_status_t *status)
{
    uint32_t primask;

    if (status == NULL) {
        return LSC16_ERR_PARAM;
    }
    primask = __get_PRIMASK();
    __disable_irq();
    *status = g_lsc16_baremetal.public_status;
    if (primask == 0U) {
        __enable_irq();
    }
    return status->initialized ? LSC16_OK : LSC16_ERR_NOT_INIT;
}

#else

lsc16_status_t lsc16_baremetal_test_init(void)
{
    return LSC16_ERR_UNSUPPORTED;
}

void lsc16_baremetal_test_process(void)
{
}

lsc16_status_t lsc16_baremetal_test_get_status(
    lsc16_baremetal_test_status_t *status)
{
    (void)status;
    return LSC16_ERR_UNSUPPORTED;
}

#endif /* LSC16_BAREMETAL_TEST_ENABLED */
