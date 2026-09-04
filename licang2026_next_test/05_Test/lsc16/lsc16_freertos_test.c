/**
 * @file    lsc16_freertos_test.c
 * @brief   USART1命令触发的LSC16完整FreeRTOS链路测试。
 *
 * 测试任务解析电脑命令并调用Device接口。Service worker的事务完成和主动回报
 * 先按值写入测试消息队列，再由测试任务统一输出USART1文本。
 */

#include "lsc16_freertos_test.h"

#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "cmsis_os.h"
#include "debug_uart1.h"
#include "lsc16_device.h"
#include "lsc16_test_common.h"
#include "lsc16_test_config.h"
#include "test_config.h"

#if LSC16_FREERTOS_TEST_ENABLED

#define LSC16_RTOS_TASK_STACK       (512U * 4U)
#define LSC16_RTOS_EVENT_QUEUE_SIZE 8U

typedef enum {
    LSC16_RTOS_EVENT_TX_DONE = 0,
    LSC16_RTOS_EVENT_REPORT,
} lsc16_rtos_event_kind_t;

typedef struct {
    lsc16_rtos_event_kind_t kind;
    lsc16_test_command_t command;
    lsc16_status_t status;
    uint32_t report_events;
    lsc16_report_t report;
} lsc16_rtos_event_t;

typedef struct {
    bool initialized;
    bool motion_armed;
    bool pending;
    lsc16_test_command_t pending_command;
    lsc16_test_action_guard_t action_guard;
    debug_uart1_t debug;
    osThreadId_t task;
    osMessageQueueId_t event_queue;
    uint32_t servo_tx_completed;
    uint32_t action_tx_completed;
    uint32_t action_started_reports;
    uint32_t action_completed_reports;
    uint32_t invalid_reports;
    uint32_t dropped_events;
    lsc16_status_t last_status;
    char text[320];
} lsc16_freertos_context_t;

static lsc16_freertos_context_t g_lsc16_freertos;

static const osThreadAttr_t g_lsc16_test_task_attr = {
    .name = "lsc16Test",
    .stack_size = LSC16_RTOS_TASK_STACK,
    .priority = (osPriority_t)osPriorityNormal,
};

static uint32_t lsc16_freertos_now_ms(void)
{
    uint32_t frequency = osKernelGetTickFreq();
    uint64_t milliseconds;

    if (frequency == 0U) {
        return 0U;
    }
    milliseconds = ((uint64_t)osKernelGetTickCount() * 1000ULL) /
        (uint64_t)frequency;
    return (uint32_t)milliseconds;
}

static void lsc16_freertos_done(
    void *user_ctx,
    uint32_t request_id,
    lsc16_status_t status)
{
    lsc16_freertos_context_t *ctx = (lsc16_freertos_context_t *)user_ctx;
    lsc16_rtos_event_t event;

    (void)request_id;
    if ((ctx == NULL) || (ctx->event_queue == NULL)) {
        return;
    }
    (void)memset(&event, 0, sizeof(event));
    event.kind = LSC16_RTOS_EVENT_TX_DONE;
    event.command = ctx->pending_command;
    event.status = status;
    if (osMessageQueuePut(ctx->event_queue, &event, 0U, 0U) != osOK) {
        ++ctx->dropped_events;
    }
}

static void lsc16_freertos_report(
    void *user_ctx,
    uint32_t report_events,
    const lsc16_report_t *report)
{
    lsc16_freertos_context_t *ctx = (lsc16_freertos_context_t *)user_ctx;
    lsc16_rtos_event_t event;

    if ((ctx == NULL) || (report == NULL) || (ctx->event_queue == NULL)) {
        return;
    }
    (void)memset(&event, 0, sizeof(event));
    event.kind = LSC16_RTOS_EVENT_REPORT;
    event.report_events = report_events;
    event.report = *report;
    if (osMessageQueuePut(ctx->event_queue, &event, 0U, 0U) != osOK) {
        ++ctx->dropped_events;
    }
}

static void lsc16_freertos_write_status(lsc16_freertos_context_t *ctx)
{
    int count = snprintf(
        ctx->text,
        sizeof(ctx->text),
        "STATUS ARMED=%u BUSY=%u ACTION_PHASE=%u FAULT=%u TIMEOUT=%lu REJECTED=%lu SERVO_TX=%lu ACTION_TX=%lu STARTED=%lu COMPLETED=%lu INVALID=%lu DROPPED=%lu LAST=%u\r\n",
        ctx->motion_armed ? 1U : 0U,
        ctx->pending ? 1U : 0U,
        (unsigned)ctx->action_guard.phase,
        ctx->action_guard.fault_latched ? 1U : 0U,
        (unsigned long)ctx->action_guard.timeout_count,
        (unsigned long)ctx->action_guard.repeated_reject_count,
        (unsigned long)ctx->servo_tx_completed,
        (unsigned long)ctx->action_tx_completed,
        (unsigned long)ctx->action_started_reports,
        (unsigned long)ctx->action_completed_reports,
        (unsigned long)ctx->invalid_reports,
        (unsigned long)ctx->dropped_events,
        (unsigned)ctx->last_status);

    if ((count > 0) && ((size_t)count < sizeof(ctx->text))) {
        (void)debug_uart1_write(
            &ctx->debug, (const uint8_t *)ctx->text, (size_t)count);
    }
}

static lsc16_status_t lsc16_freertos_submit(
    lsc16_freertos_context_t *ctx,
    lsc16_test_command_t command)
{
    lsc16_status_t status;
    bool action_guard_acquired = false;

    if (command == LSC16_TEST_COMMAND_STATUS) {
        lsc16_freertos_write_status(ctx);
        return LSC16_OK;
    }
    if ((command == LSC16_TEST_COMMAND_SERVO) ||
        (command == LSC16_TEST_COMMAND_ACTION)) {
#if !LSC16_TEST_MOTION_ARMED
        (void)debug_uart1_write_text(&ctx->debug,
            "MOTION LOCKED. SET LSC16_TEST_MOTION_ARMED=1\r\n");
        return LSC16_ERR_UNSUPPORTED;
#endif
        if (ctx->action_guard.fault_latched) {
            (void)debug_uart1_write_text(&ctx->debug,
                "MOTION FAULT LATCHED. RESET F7 BEFORE NEW MOTION\r\n");
            return LSC16_ERR_STATE;
        }
        if ((command == LSC16_TEST_COMMAND_SERVO) &&
            lsc16_test_action_guard_is_active(&ctx->action_guard)) {
            return LSC16_ERR_BUSY;
        }
    }
    if (command == LSC16_TEST_COMMAND_ACTION) {
        status = lsc16_test_action_guard_begin(
            &ctx->action_guard,
            lsc16_freertos_now_ms(),
            LSC16_TEST_ACTION_TX_TIMEOUT_MS);
        if (status != LSC16_OK) {
            return status;
        }
        action_guard_acquired = true;
    }
    if (ctx->pending) {
        if (action_guard_acquired) {
            lsc16_test_action_guard_cancel_begin(&ctx->action_guard);
        }
        return LSC16_ERR_BUSY;
    }
    ctx->pending_command = command;
    switch (command) {
    case LSC16_TEST_COMMAND_SERVO:
        status = lsc16_device_move_servo(
            LSC16_TEST_SERVO_ID,
            LSC16_TEST_SERVO_POSITION,
            LSC16_TEST_SERVO_MOVE_TIME_MS,
            lsc16_freertos_done,
            ctx);
        break;
    case LSC16_TEST_COMMAND_ACTION:
        status = lsc16_device_run_action_group(
            LSC16_TEST_ACTION_GROUP,
            LSC16_TEST_ACTION_REPEAT_COUNT,
            lsc16_freertos_done,
            ctx);
        break;
    case LSC16_TEST_COMMAND_STOP:
        status = lsc16_device_stop_action_group(
            lsc16_freertos_done, ctx);
        break;
    case LSC16_TEST_COMMAND_BATTERY:
        status = lsc16_device_request_battery_voltage(
            lsc16_freertos_done, ctx);
        break;
    default:
        status = LSC16_ERR_UNSUPPORTED;
        break;
    }
    if (status == LSC16_OK) {
        ctx->pending = true;
    } else {
        if (action_guard_acquired) {
            lsc16_test_action_guard_cancel_begin(&ctx->action_guard);
        }
        ctx->pending_command = LSC16_TEST_COMMAND_INVALID;
    }
    return status;
}

static void lsc16_freertos_submit_auto_stop(lsc16_freertos_context_t *ctx)
{
    lsc16_status_t stop_status;

    if (ctx->pending) {
        (void)debug_uart1_write_text(&ctx->debug,
            "AUTO STOP DEFERRED: LSC16 REQUEST STILL BUSY\r\n");
        return;
    }
    ctx->pending_command = LSC16_TEST_COMMAND_STOP;
    stop_status = lsc16_device_stop_action_group(lsc16_freertos_done, ctx);
    ctx->last_status = stop_status;
    if (stop_status == LSC16_OK) {
        ctx->pending = true;
        (void)debug_uart1_write_text(&ctx->debug,
            "AUTO STOP REQUEST START. PHYSICAL POWER CUT REMAINS PRIMARY\r\n");
    } else {
        ctx->pending_command = LSC16_TEST_COMMAND_INVALID;
        (void)snprintf(ctx->text, sizeof(ctx->text),
            "AUTO STOP SUBMIT ERROR %u. CUT POWER IF MOVING\r\n",
            (unsigned)stop_status);
        (void)debug_uart1_write_text(&ctx->debug, ctx->text);
    }
}

static void lsc16_freertos_handle_tx_done(
    lsc16_freertos_context_t *ctx,
    const lsc16_rtos_event_t *event)
{
    ctx->pending = false;
    ctx->pending_command = LSC16_TEST_COMMAND_INVALID;
    ctx->last_status = event->status;
    if (event->status != LSC16_OK) {
        if (event->command == LSC16_TEST_COMMAND_ACTION) {
            lsc16_test_action_guard_latch_fault(&ctx->action_guard);
        }
        (void)snprintf(ctx->text, sizeof(ctx->text),
            "%s ERROR %u\r\n",
            lsc16_test_command_name(event->command),
            (unsigned)event->status);
        (void)debug_uart1_write_text(&ctx->debug, ctx->text);
        if (event->command == LSC16_TEST_COMMAND_ACTION) {
            lsc16_freertos_submit_auto_stop(ctx);
        }
        return;
    }
    switch (event->command) {
    case LSC16_TEST_COMMAND_SERVO:
        ++ctx->servo_tx_completed;
        (void)debug_uart1_write_text(&ctx->debug,
            "SERVO TX DONE. OBSERVE ACTUAL MOTION\r\n");
        break;
    case LSC16_TEST_COMMAND_ACTION:
        ++ctx->action_tx_completed;
        if (ctx->action_guard.fault_latched) {
            (void)debug_uart1_write_text(&ctx->debug,
                "LATE ACTION TX DONE AFTER FAULT; SUBMIT AUTO STOP\r\n");
            lsc16_freertos_submit_auto_stop(ctx);
            break;
        }
        lsc16_test_action_guard_on_tx_done(
            &ctx->action_guard,
            lsc16_freertos_now_ms(),
            LSC16_TEST_ACTION_STARTED_TIMEOUT_MS);
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
}

static void lsc16_freertos_handle_report(
    lsc16_freertos_context_t *ctx,
    const lsc16_rtos_event_t *event)
{
    uint32_t report_events = event->report_events;
    static const uint32_t ordered_events[] = {
        LSC16_REPORT_EVENT_INVALID_FRAME,
        LSC16_REPORT_EVENT_ACTION_STARTED,
        LSC16_REPORT_EVENT_ACTION_STOPPED,
        LSC16_REPORT_EVENT_ACTION_COMPLETED,
        LSC16_REPORT_EVENT_BATTERY_UPDATED,
    };
    size_t i;

    if ((report_events & LSC16_REPORT_EVENT_INVALID_FRAME) != 0U) {
        ++ctx->invalid_reports;
    }
    if ((report_events & LSC16_REPORT_EVENT_ACTION_STARTED) != 0U) {
        ++ctx->action_started_reports;
        lsc16_test_action_guard_on_started(
            &ctx->action_guard,
            lsc16_freertos_now_ms(),
            LSC16_TEST_ACTION_COMPLETE_TIMEOUT_MS);
    }
    if ((report_events & LSC16_REPORT_EVENT_ACTION_COMPLETED) != 0U) {
        ++ctx->action_completed_reports;
        lsc16_test_action_guard_on_completed(&ctx->action_guard);
    }
    if ((report_events & LSC16_REPORT_EVENT_ACTION_STOPPED) != 0U) {
        lsc16_test_action_guard_on_stopped(&ctx->action_guard);
    }
    for (i = 0U; i < (sizeof(ordered_events) / sizeof(ordered_events[0]));
         ++i) {
        size_t text_len;
        if ((report_events & ordered_events[i]) == 0U) {
            continue;
        }
        text_len = lsc16_test_format_report(
            ordered_events[i],
            &event->report,
            ctx->text,
            sizeof(ctx->text));
        if (text_len > 0U) {
            (void)debug_uart1_write(
                &ctx->debug, (const uint8_t *)ctx->text, text_len);
        }
    }
}

static void lsc16_freertos_check_action_timeout(
    lsc16_freertos_context_t *ctx)
{
    lsc16_test_action_timeout_t timeout;
    const char *timeout_name;

    timeout = lsc16_test_action_guard_poll(
        &ctx->action_guard,
        lsc16_freertos_now_ms());
    if (timeout == LSC16_TEST_ACTION_TIMEOUT_NONE) {
        return;
    }
    timeout_name = (timeout == LSC16_TEST_ACTION_TIMEOUT_TX) ? "TX" :
        ((timeout == LSC16_TEST_ACTION_TIMEOUT_STARTED) ? "START" : "COMPLETE");
    (void)snprintf(
        ctx->text,
        sizeof(ctx->text),
        "ACTION %s TIMEOUT. MOTION FAULT LATCHED; CUT POWER IF MOVING\r\n",
        timeout_name);
    (void)debug_uart1_write_text(&ctx->debug, ctx->text);

    lsc16_freertos_submit_auto_stop(ctx);
}

static void lsc16_freertos_task_entry(void *argument)
{
    lsc16_freertos_context_t *ctx = (lsc16_freertos_context_t *)argument;
    uint8_t data[DEBUG_UART1_RX_BUFFER_SIZE];
    size_t data_len;
    lsc16_test_command_t command;
    lsc16_status_t status;
    lsc16_rtos_event_t event;

    (void)debug_uart1_write_text(&ctx->debug,
        "LSC16 RTOS READY. USE SERVO ACTION STOP BATTERY STATUS\r\n");
    for (;;) {
        while (osMessageQueueGet(
                   ctx->event_queue, &event, NULL, 0U) == osOK) {
            if (event.kind == LSC16_RTOS_EVENT_TX_DONE) {
                lsc16_freertos_handle_tx_done(ctx, &event);
            } else {
                lsc16_freertos_handle_report(ctx, &event);
            }
        }
        /* 先处理已经入队的回报，避免截止时刻误判 STARTED/COMPLETED 超时。 */
        lsc16_freertos_check_action_timeout(ctx);
        if (debug_uart1_take_message(
                &ctx->debug, data, sizeof(data), &data_len)) {
            command = lsc16_test_parse_command(data, data_len);
            if (command == LSC16_TEST_COMMAND_INVALID) {
                (void)debug_uart1_write_text(&ctx->debug,
                    "USE SERVO ACTION STOP BATTERY STATUS\r\n");
            } else {
                status = lsc16_freertos_submit(ctx, command);
                ctx->last_status = status;
                if ((status == LSC16_OK) &&
                    (command != LSC16_TEST_COMMAND_STATUS)) {
                    (void)snprintf(ctx->text, sizeof(ctx->text),
                        "%s REQUEST START\r\n",
                        lsc16_test_command_name(command));
                    (void)debug_uart1_write_text(&ctx->debug, ctx->text);
                } else if (status == LSC16_ERR_BUSY) {
                    (void)debug_uart1_write_text(
                        &ctx->debug, "LSC16 BUSY\r\n");
                } else if ((status != LSC16_OK) &&
                           (status != LSC16_ERR_UNSUPPORTED)) {
                    (void)snprintf(ctx->text, sizeof(ctx->text),
                        "LSC16 SUBMIT ERROR %u\r\n", (unsigned)status);
                    (void)debug_uart1_write_text(&ctx->debug, ctx->text);
                }
            }
        }
        (void)osDelay(10U);
    }
}

lsc16_status_t lsc16_freertos_test_init(void)
{
    lsc16_freertos_context_t *ctx = &g_lsc16_freertos;
    lsc16_status_t status;

    if (ctx->initialized) {
        return LSC16_ERR_STATE;
    }
    (void)memset(ctx, 0, sizeof(*ctx));
    lsc16_test_action_guard_init(&ctx->action_guard);
    ctx->motion_armed = (LSC16_TEST_MOTION_ARMED != 0U);
    ctx->last_status = LSC16_OK;
    if (!debug_uart1_init(&ctx->debug)) {
        return LSC16_ERR_IO;
    }
    ctx->event_queue = osMessageQueueNew(
        LSC16_RTOS_EVENT_QUEUE_SIZE, sizeof(lsc16_rtos_event_t), NULL);
    if (ctx->event_queue == NULL) {
        debug_uart1_deinit(&ctx->debug);
        return LSC16_ERR_IO;
    }
    status = lsc16_device_set_report_callback(lsc16_freertos_report, ctx);
    if (status != LSC16_OK) {
        (void)osMessageQueueDelete(ctx->event_queue);
        debug_uart1_deinit(&ctx->debug);
        return status;
    }
    ctx->task = osThreadNew(
        lsc16_freertos_task_entry, ctx, &g_lsc16_test_task_attr);
    if (ctx->task == NULL) {
        (void)lsc16_device_set_report_callback(NULL, NULL);
        (void)osMessageQueueDelete(ctx->event_queue);
        debug_uart1_deinit(&ctx->debug);
        return LSC16_ERR_IO;
    }
    ctx->initialized = true;
    return LSC16_OK;
}

#else

lsc16_status_t lsc16_freertos_test_init(void)
{
    return LSC16_ERR_UNSUPPORTED;
}

#endif /* LSC16_FREERTOS_TEST_ENABLED */
