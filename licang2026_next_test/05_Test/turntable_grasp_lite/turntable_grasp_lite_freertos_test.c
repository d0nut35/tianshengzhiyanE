/**
 * @file turntable_grasp_lite_freertos_test.c
 * @brief Nano事件、LSC16、IC读卡和ZDT逐槽的轻量多球实机闭环。
 *
 * 本测试只保留一个普通优先级任务。UART7和UART8仍分别由既有worker串行
 * 驱动；完成回调只复制单槽结果并设置线程标志，不创建应用事件队列。
 */

#include "turntable_grasp_lite_freertos_test.h"

#include <stdio.h>
#include <string.h>

#include "FreeRTOS.h"
#include "ball_manifest_core.h"
#include "cmsis_os.h"
#include "debug_uart1.h"
#include "ic_card_device.h"
#include "lsc16_device.h"
#include "mult_uart_device.h"
#include "nano_vision_core.h"
#include "gate.h"
#include "task.h"
#include "test_config.h"
#include "turntable_grasp_lite_config.h"
#include "zdt_turntable_device.h"

#if TURNTABLE_GRASP_LITE_FREERTOS_TEST_ENABLED

#define LITE_FLAG_VISION_DONE (1UL << 0)
#define LITE_FLAG_ARM_TX_DONE (1UL << 1)
#define LITE_FLAG_ARM_REPORT  (1UL << 2)
#define LITE_FLAG_IC_DONE     (1UL << 3)
#define LITE_FLAG_ZDT_DONE    (1UL << 4)
#define LITE_FLAGS_ALL (LITE_FLAG_VISION_DONE | LITE_FLAG_ARM_TX_DONE | \
                        LITE_FLAG_ARM_REPORT | LITE_FLAG_IC_DONE | \
                        LITE_FLAG_ZDT_DONE)

typedef enum {
    LITE_POSE_UNKNOWN = 0,
    LITE_POSE_HOME_10,
    LITE_POSE_VISION_11,
} lite_pose_t;

typedef enum {
    LITE_VISION_IDLE = 0,
    LITE_VISION_STARTING,
    LITE_VISION_LISTENING,
    LITE_VISION_ACKING,
    LITE_VISION_STOPPING,
} lite_vision_phase_t;

typedef enum {
    LITE_MULTI_IDLE = 0,
    LITE_MULTI_WAIT_VISION,
    LITE_MULTI_WAIT_GRASP12,
    LITE_MULTI_WAIT_RETURN11,
    LITE_MULTI_WAIT_IC_SUBMIT,
    LITE_MULTI_WAIT_IC,
    LITE_MULTI_WAIT_SLOT,
    LITE_MULTI_COMPLETE,
    LITE_MULTI_STOPPED,
    LITE_MULTI_FAULT,
} lite_multi_phase_t;

typedef enum {
    LITE_SLOT_IDLE = 0,
    LITE_SLOT_WAIT_COARSE_ACK,
    LITE_SLOT_WAIT_COARSE_POLL,
    LITE_SLOT_WAIT_COARSE_STATUS,
    LITE_SLOT_WAIT_FINE_ACK,
    LITE_SLOT_WAIT_FINE_POLL,
    LITE_SLOT_WAIT_FINE_STATUS,
} lite_slot_phase_t;

typedef struct {
    bool active;
    uint8_t tx[IC_CARD_FRAME_SIZE_MAX];
    ic_card_request_t request;
} lite_ic_transport_t;

typedef struct {
    bool active;
    zdt_turntable_request_t request;
} lite_zdt_transport_t;

typedef struct {
    bool active;
    lite_slot_phase_t phase;
    zdt_turntable_direction_t direction;
    uint8_t fine_steps;
    uint32_t started_tick;
    uint32_t next_poll_tick;
} lite_slot_t;

typedef struct {
    bool matched;
    nano_vision_frame_t frame;
} lite_capture_t;

typedef struct {
    debug_uart1_t debug;
    osThreadId_t task;
    nano_vision_parser_t parser;
    bool vision_enabled;
    bool grasp_armed;
    bool multi_grasp;
    bool vision_inflight;
    bool vision_stop_requested;
    lite_vision_phase_t vision_phase;
    nano_vision_color_t target_color;
    uint8_t next_sequence;
    uint8_t active_sequence;
    uint16_t next_session_id;
    uint16_t active_session_id;
    uint16_t pending_event_frame;
    uint8_t vision_tx[NANO_VISION_FRAME_MAX];
    uint32_t armed_tick;
    uint16_t last_event_frame;
    bool have_last_event_frame;
    volatile mult_uart_status_t vision_mail_status;
    volatile uint16_t vision_mail_len;
    uint8_t vision_mail_data[NANO_VISION_FRAME_MAX];
    bool action_active;
    bool arm_tx_inflight;
    bool fault_latched;
    uint8_t active_group;
    lite_pose_t pose;
    uint32_t action_deadline;
    volatile lsc16_status_t arm_mail_status;
    volatile uint32_t arm_report_events;
    lsc16_report_t arm_report;
    lite_ic_transport_t ic_transport;
    volatile ic_card_status_t ic_mail_status;
    ic_card_ball_result_t ic_mail_ball;
    lite_zdt_transport_t zdt_transport;
    zdt_turntable_device_t zdt;
    volatile zdt_turntable_status_t zdt_mail_status;
    volatile bool zdt_mail_has_response;
    zdt_turntable_response_t zdt_mail_response;
    lite_slot_t slot;
    ball_manifest_t manifest;
    lite_multi_phase_t multi_phase;
    bool multi_stop_requested;
    bool run_pending;
    nano_vision_color_t run_color;
    uint8_t multi_ic_attempts;
    uint32_t multi_next_action_tick;
    const char *multi_fault;
    uint32_t session_count;
    uint32_t read_wait_count;
    uint32_t event_count;
    uint32_t trigger_count;
    uint8_t multi_completed;
    uint32_t vision_error_count;
    uint32_t rejected_count;
    char text[192];
} lite_context_t;

static lite_context_t g_lite;

static void lite_run_continue(lite_context_t *ctx);
static void lite_run_cancel(lite_context_t *ctx, const char *reason);

static const osThreadAttr_t g_lite_task_attr = {
    .name = "graspLite",
    .stack_size = 1024U * 4U,
    .priority = (osPriority_t)osPriorityNormal,
};

static uint32_t lite_now(void)
{
    return osKernelGetTickCount();
}

static uint32_t lite_ms_to_ticks(uint32_t milliseconds)
{
    uint32_t ticks = (uint32_t)pdMS_TO_TICKS(milliseconds);
    return ((milliseconds > 0U) && (ticks == 0U)) ? 1U : ticks;
}

static uint32_t lite_ticks_to_ms(uint32_t ticks)
{
    return (uint32_t)(((uint64_t)ticks * 1000ULL) /
                      (uint64_t)configTICK_RATE_HZ);
}

static bool lite_time_reached(uint32_t now, uint32_t deadline)
{
    return (int32_t)(now - deadline) >= 0;
}

static bool lite_command_equals(const uint8_t *data, size_t len, const char *text)
{
    size_t text_len;
    while ((len > 0U) && ((data[len - 1U] == '\r') ||
           (data[len - 1U] == '\n') || (data[len - 1U] == ' '))) {
        --len;
    }
    text_len = strlen(text);
    return (len == text_len) && (memcmp(data, text, len) == 0);
}

static const char *lite_pose_name(lite_pose_t pose)
{
    if (pose == LITE_POSE_HOME_10) return "HOME10";
    if (pose == LITE_POSE_VISION_11) return "VISION11";
    return "UNKNOWN";
}

static nano_vision_status_t lite_map_mult_status(mult_uart_status_t status)
{
    if (status == MULT_UART_OK) return NANO_VISION_OK;
    if (status == MULT_UART_ERR_TIMEOUT) return NANO_VISION_ERR_TIMEOUT;
    if (status == MULT_UART_ERR_BUSY) return NANO_VISION_ERR_BUSY;
    if (status == MULT_UART_ERR_QUEUE_FULL) return NANO_VISION_ERR_QUEUE_FULL;
    return NANO_VISION_ERR_IO;
}

static ic_card_status_t lite_map_ic_status(mult_uart_status_t status)
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

static zdt_turntable_status_t lite_map_zdt_status(mult_uart_status_t status)
{
    if (status == MULT_UART_OK) return ZDT_TURNTABLE_OK;
    if (status == MULT_UART_ERR_TIMEOUT) return ZDT_TURNTABLE_ERR_TIMEOUT;
    if (status == MULT_UART_ERR_BUSY) return ZDT_TURNTABLE_ERR_BUSY;
    if (status == MULT_UART_ERR_QUEUE_FULL) {
        return ZDT_TURNTABLE_ERR_QUEUE_FULL;
    }
    if ((status == MULT_UART_ERR_PARAM) || (status == MULT_UART_ERR_OVERFLOW)) {
        return ZDT_TURNTABLE_ERR_PARAM;
    }
    return ZDT_TURNTABLE_ERR_IO;
}

static void lite_ic_transfer_done(
    void *user_ctx, const mult_uart_device_completion_t *completion)
{
    lite_ic_transport_t *transport = (lite_ic_transport_t *)user_ctx;
    ic_card_request_t request;
    ic_card_response_t response;
    ic_card_status_t status;

    if ((transport == NULL) || !transport->active || (completion == NULL)) {
        return;
    }
    request = transport->request;
    transport->active = false;
    status = lite_map_ic_status(completion->status);
    if (status == IC_CARD_OK) {
        status = ic_card_parse_response_frame(
            completion->rx_data, completion->rx_len, &response);
        if ((status == IC_CARD_OK) &&
            (response.command != ((request.type == IC_CARD_REQUEST_READ_BLOCK) ?
                IC_CARD_CMD_READ_BLOCK_KEY_A : request.data.query.command))) {
            status = IC_CARD_ERR_PROTOCOL;
        }
        if ((status == IC_CARD_OK) && (response.device_status != 0U)) {
            status = IC_CARD_ERR_CARD;
        }
    }
    if (request.done_cb != NULL) {
        request.done_cb(
            request.user_ctx, request.request_id, status,
            ((status == IC_CARD_OK) || (status == IC_CARD_ERR_CARD)) ?
                &response : NULL);
    }
}

static ic_card_status_t lite_ic_submit(
    void *submit_ctx,
    const ic_card_request_t *request,
    uint32_t queue_timeout_ms)
{
    lite_ic_transport_t *transport = (lite_ic_transport_t *)submit_ctx;
    mult_uart_device_transfer_t transfer;
    mult_uart_status_t mult_status;
    ic_card_status_t status;
    size_t tx_len = 0U;

    if ((transport == NULL) || (request == NULL)) return IC_CARD_ERR_PARAM;
    if (transport->active) return IC_CARD_ERR_BUSY;
    if (request->type == IC_CARD_REQUEST_READ_BLOCK) {
        status = ic_card_build_read_block_key_a_frame(
            request->address,
            request->data.read_block.block,
            request->data.read_block.led_beep_prompt,
            transport->tx, sizeof(transport->tx), &tx_len);
    } else if (request->type == IC_CARD_REQUEST_QUERY) {
        status = ic_card_build_query_frame(
            request->data.query.command, request->address,
            transport->tx, sizeof(transport->tx), &tx_len);
    } else {
        return IC_CARD_ERR_UNSUPPORTED;
    }
    if (status != IC_CARD_OK) return status;

    transport->request = *request;
    transport->active = true;
    (void)memset(&transfer, 0, sizeof(transfer));
    transfer.device_id = TURN_GRASP_LITE_IC_DEVICE_ID;
    transfer.operation = MULT_UART_OP_WRITE_READ;
    transfer.tx_data = transport->tx;
    transfer.tx_len = tx_len;
    transfer.rx_capacity = IC_CARD_FRAME_SIZE_MAX;
    transfer.io_timeout_ms = request->timeout_ms;
    transfer.queue_timeout_ms = queue_timeout_ms;
    transfer.done_cb = lite_ic_transfer_done;
    transfer.user_ctx = transport;
    mult_status = mult_uart_device_submit(&transfer);
    if (mult_status != MULT_UART_OK) transport->active = false;
    return lite_map_ic_status(mult_status);
}

static void lite_zdt_transfer_done(
    void *user_ctx, const mult_uart_device_completion_t *completion)
{
    lite_zdt_transport_t *transport = (lite_zdt_transport_t *)user_ctx;
    zdt_turntable_request_t request;
    zdt_turntable_response_t response;
    zdt_turntable_status_t status;

    if ((transport == NULL) || !transport->active || (completion == NULL)) {
        return;
    }
    request = transport->request;
    transport->active = false;
    status = lite_map_zdt_status(completion->status);
    if (status == ZDT_TURNTABLE_OK) {
        status = zdt_turntable_parse_response(
            completion->rx_data, completion->rx_len,
            request.expected_address, request.expected_function, &response);
    }
    if (request.done_cb != NULL) {
        request.done_cb(
            request.user_ctx, request.request_id, status,
            ((status == ZDT_TURNTABLE_OK) ||
             (status == ZDT_TURNTABLE_ERR_DEVICE)) ? &response : NULL);
    }
}

static zdt_turntable_status_t lite_zdt_submit(
    void *submit_ctx, const zdt_turntable_request_t *request)
{
    lite_zdt_transport_t *transport = (lite_zdt_transport_t *)submit_ctx;
    mult_uart_device_transfer_t transfer;
    mult_uart_status_t mult_status;

    if ((transport == NULL) || (request == NULL)) {
        return ZDT_TURNTABLE_ERR_PARAM;
    }
    if (transport->active) return ZDT_TURNTABLE_ERR_BUSY;
    transport->request = *request;
    transport->active = true;
    (void)memset(&transfer, 0, sizeof(transfer));
    transfer.device_id = TURN_GRASP_LITE_ZDT_DEVICE_ID;
    transfer.operation = MULT_UART_OP_WRITE_READ;
    transfer.tx_data = transport->request.frame;
    transfer.tx_len = transport->request.frame_len;
    transfer.rx_capacity = ZDT_TURNTABLE_RESPONSE_MAX;
    transfer.io_timeout_ms = request->timeout_ms;
    transfer.done_cb = lite_zdt_transfer_done;
    transfer.user_ctx = transport;
    mult_status = mult_uart_device_submit(&transfer);
    if (mult_status != MULT_UART_OK) transport->active = false;
    return lite_map_zdt_status(mult_status);
}

static void lite_capture_frame(void *user_ctx, const nano_vision_frame_t *frame)
{
    lite_capture_t *capture = (lite_capture_t *)user_ctx;
    if ((capture == NULL) || (frame == NULL) || capture->matched) return;
    capture->frame = *frame;
    capture->matched = true;
}

static void lite_vision_done(
    void *user_ctx, const mult_uart_device_completion_t *completion)
{
    lite_context_t *ctx = (lite_context_t *)user_ctx;
    size_t copy_len;
    if ((ctx == NULL) || (completion == NULL)) return;
    ctx->vision_mail_status = completion->status;
    copy_len = completion->rx_len;
    if (copy_len > sizeof(ctx->vision_mail_data)) {
        copy_len = sizeof(ctx->vision_mail_data);
        ctx->vision_mail_status = MULT_UART_ERR_OVERFLOW;
    }
    if ((copy_len > 0U) && (completion->rx_data != NULL)) {
        (void)memcpy(ctx->vision_mail_data, completion->rx_data, copy_len);
    }
    ctx->vision_mail_len = (uint16_t)copy_len;
    __DMB();
    (void)osThreadFlagsSet(ctx->task, LITE_FLAG_VISION_DONE);
}

static void lite_ic_read_done(
    void *user_ctx,
    uint32_t request_id,
    ic_card_status_t status,
    const ic_card_ball_result_t *result)
{
    lite_context_t *ctx = (lite_context_t *)user_ctx;
    (void)request_id;
    if (ctx == NULL) return;
    ctx->ic_mail_status = status;
    if ((status == IC_CARD_OK) && (result != NULL)) {
        ctx->ic_mail_ball = *result;
    }
    __DMB();
    (void)osThreadFlagsSet(ctx->task, LITE_FLAG_IC_DONE);
}

static void lite_zdt_done(
    void *user_ctx,
    uint32_t request_id,
    zdt_turntable_status_t status,
    const zdt_turntable_response_t *response)
{
    lite_context_t *ctx = (lite_context_t *)user_ctx;
    (void)request_id;
    if (ctx == NULL) return;
    ctx->zdt_mail_status = status;
    ctx->zdt_mail_has_response = (response != NULL);
    if (response != NULL) ctx->zdt_mail_response = *response;
    __DMB();
    (void)osThreadFlagsSet(ctx->task, LITE_FLAG_ZDT_DONE);
}

static nano_vision_status_t lite_submit_vision_transfer(
    lite_context_t *ctx,
    mult_uart_operation_t operation,
    size_t tx_len,
    uint32_t timeout_ms)
{
    mult_uart_device_transfer_t transfer;
    mult_uart_status_t mult_status;

    if (ctx->vision_inflight) return NANO_VISION_ERR_BUSY;
    nano_vision_parser_init(&ctx->parser);
    (void)memset(&transfer, 0, sizeof(transfer));
    transfer.device_id = TURN_GRASP_LITE_VISION_DEVICE_ID;
    transfer.operation = operation;
    transfer.tx_data = (tx_len > 0U) ? ctx->vision_tx : NULL;
    transfer.tx_len = tx_len;
    transfer.rx_capacity =
        ((operation == MULT_UART_OP_READ) ||
         (operation == MULT_UART_OP_WRITE_READ)) ? NANO_VISION_FRAME_MAX : 0U;
    transfer.io_timeout_ms = timeout_ms;
    transfer.done_cb = lite_vision_done;
    transfer.user_ctx = ctx;
    ctx->vision_inflight = true;
    mult_status = mult_uart_device_submit(&transfer);
    if (mult_status != MULT_UART_OK) ctx->vision_inflight = false;
    return lite_map_mult_status(mult_status);
}

static uint8_t lite_next_sequence(lite_context_t *ctx)
{
    ++ctx->next_sequence;
    if (ctx->next_sequence == 0U) ++ctx->next_sequence;
    ctx->active_sequence = ctx->next_sequence;
    return ctx->active_sequence;
}

static nano_vision_status_t lite_submit_session_start(lite_context_t *ctx)
{
    nano_vision_session_t session;
    nano_vision_status_t status;
    size_t tx_len = 0U;

    session.session_id = ctx->active_session_id;
    session.scene = NANO_VISION_SCENE_TURNTABLE;
    session.target_color = ctx->target_color;
    status = nano_vision_build_session_start_frame(
        lite_next_sequence(ctx), &session, ctx->vision_tx,
        sizeof(ctx->vision_tx), &tx_len);
    if (status != NANO_VISION_OK) return status;
    ctx->vision_phase = LITE_VISION_STARTING;
    return lite_submit_vision_transfer(
        ctx, MULT_UART_OP_WRITE_READ, tx_len,
        TURN_GRASP_LITE_VISION_TIMEOUT_MS);
}

static nano_vision_status_t lite_submit_event_read(lite_context_t *ctx)
{
    ctx->vision_phase = LITE_VISION_LISTENING;
    return lite_submit_vision_transfer(
        ctx, MULT_UART_OP_READ, 0U,
        TURN_GRASP_LITE_EVENT_READ_TIMEOUT_MS);
}

static nano_vision_status_t lite_submit_event_ack(
    lite_context_t *ctx,
    uint16_t frame_id)
{
    nano_vision_event_ack_t ack;
    nano_vision_status_t status;
    size_t tx_len = 0U;

    ack.session_id = ctx->active_session_id;
    ack.frame_id = frame_id;
    status = nano_vision_build_event_ack_frame(
        lite_next_sequence(ctx), &ack, ctx->vision_tx,
        sizeof(ctx->vision_tx), &tx_len);
    if (status != NANO_VISION_OK) return status;
    ctx->pending_event_frame = frame_id;
    ctx->vision_phase = LITE_VISION_ACKING;
    return lite_submit_vision_transfer(
        ctx, MULT_UART_OP_WRITE, tx_len,
        TURN_GRASP_LITE_VISION_TIMEOUT_MS);
}

static nano_vision_status_t lite_submit_session_stop(lite_context_t *ctx)
{
    nano_vision_status_t status;
    size_t tx_len = 0U;

    status = nano_vision_build_session_stop_frame(
        lite_next_sequence(ctx), ctx->active_session_id,
        ctx->vision_tx, sizeof(ctx->vision_tx), &tx_len);
    if (status != NANO_VISION_OK) return status;
    ctx->vision_phase = LITE_VISION_STOPPING;
    return lite_submit_vision_transfer(
        ctx, MULT_UART_OP_WRITE_READ, tx_len,
        TURN_GRASP_LITE_VISION_TIMEOUT_MS);
}

static void lite_arm_done(
    void *user_ctx, uint32_t request_id, lsc16_status_t status)
{
    lite_context_t *ctx = (lite_context_t *)user_ctx;
    (void)request_id;
    if (ctx == NULL) return;
    ctx->arm_mail_status = status;
    __DMB();
    (void)osThreadFlagsSet(ctx->task, LITE_FLAG_ARM_TX_DONE);
}

static void lite_arm_report(
    void *user_ctx, uint32_t events, const lsc16_report_t *report)
{
    lite_context_t *ctx = (lite_context_t *)user_ctx;
    if ((ctx == NULL) || (report == NULL)) return;
    ctx->arm_report = *report;
    ctx->arm_report_events |= events;
    __DMB();
    (void)osThreadFlagsSet(ctx->task, LITE_FLAG_ARM_REPORT);
}

static lsc16_status_t lite_submit_action(lite_context_t *ctx, uint8_t group)
{
    lsc16_status_t status;
    if (ctx->action_active || ctx->arm_tx_inflight || ctx->fault_latched) {
        return LSC16_ERR_BUSY;
    }
    status = lsc16_device_run_action_group(group, 1U, lite_arm_done, ctx);
    if (status != LSC16_OK) return status;
    ctx->active_group = group;
    ctx->action_active = true;
    ctx->arm_tx_inflight = true;
    ctx->action_deadline = lite_now() +
        lite_ms_to_ticks(TURN_GRASP_LITE_ACTION_TIMEOUT_MS);
    (void)snprintf(ctx->text, sizeof(ctx->text),
        "ARM REQUEST GROUP=%u\r\n", (unsigned)group);
    (void)debug_uart1_write_text(&ctx->debug, ctx->text);
    return LSC16_OK;
}

static void lite_reset_session(lite_context_t *ctx, bool clear_multi)
{
    ctx->vision_enabled = false;
    ctx->grasp_armed = false;
    ctx->vision_stop_requested = false;
    ctx->vision_phase = LITE_VISION_IDLE;
    ctx->active_session_id = 0U;
    ctx->pending_event_frame = 0U;
    if (clear_multi) ctx->multi_grasp = false;
}

static void lite_stop_session(lite_context_t *ctx)
{
    ctx->vision_enabled = false;
    ctx->grasp_armed = false;
    ctx->multi_grasp = false;
    if ((ctx->active_session_id != 0U) &&
        (ctx->vision_phase != LITE_VISION_IDLE)) {
        ctx->vision_stop_requested = true;
    } else {
        lite_reset_session(ctx, true);
    }
}

static nano_vision_status_t lite_start_session(
    lite_context_t *ctx, nano_vision_color_t color, bool armed, bool multi)
{
    nano_vision_status_t status;

    if (ctx->vision_inflight ||
        (ctx->vision_phase != LITE_VISION_IDLE) ||
        (ctx->active_session_id != 0U)) {
        return NANO_VISION_ERR_BUSY;
    }
    ++ctx->next_session_id;
    if (ctx->next_session_id == 0U) ++ctx->next_session_id;
    ctx->active_session_id = ctx->next_session_id;
    ctx->vision_enabled = true;
    ctx->grasp_armed = armed;
    ctx->multi_grasp = multi;
    ctx->target_color = color;
    ctx->armed_tick = lite_now();
    ctx->have_last_event_frame = false;
    status = lite_submit_session_start(ctx);
    if (status != NANO_VISION_OK) {
        lite_reset_session(ctx, true);
        return status;
    }
    (void)snprintf(ctx->text, sizeof(ctx->text),
        "%s COLOR=%s SESSION=%u EVENT_MODE=1 MOTION=%u\r\n",
        multi ? "MULTI GRASP ARMED" :
            (armed ? "GRASP ARMED" : "WATCH START"),
        (color == NANO_VISION_COLOR_RED) ? "RED" : "BLUE",
        (unsigned)ctx->active_session_id,
        armed ? 1U : 0U);
    (void)debug_uart1_write_text(&ctx->debug, ctx->text);
    return NANO_VISION_OK;
}

static const char *lite_multi_phase_name(lite_multi_phase_t phase)
{
    switch (phase) {
    case LITE_MULTI_IDLE: return "IDLE";
    case LITE_MULTI_WAIT_VISION: return "VISION";
    case LITE_MULTI_WAIT_GRASP12: return "GRASP12";
    case LITE_MULTI_WAIT_RETURN11: return "RETURN11";
    case LITE_MULTI_WAIT_IC_SUBMIT: return "IC_SUBMIT";
    case LITE_MULTI_WAIT_IC: return "IC";
    case LITE_MULTI_WAIT_SLOT: return "SLOT";
    case LITE_MULTI_COMPLETE: return "COMPLETE";
    case LITE_MULTI_STOPPED: return "STOPPED";
    case LITE_MULTI_FAULT: return "FAULT";
    default: return "INVALID";
    }
}

static bool lite_gate_is_stably_high(void)
{
    uint8_t sample;

    for (sample = 0U; sample < TURN_GRASP_LITE_GATE_CONFIRM_SAMPLES; ++sample) {
        if (!gate_read()) return false;
        if ((sample + 1U) < TURN_GRASP_LITE_GATE_CONFIRM_SAMPLES) {
            (void)osDelay(lite_ms_to_ticks(
                TURN_GRASP_LITE_GATE_CONFIRM_INTERVAL_MS));
        }
    }
    return true;
}

static void lite_multi_finish(
    lite_context_t *ctx, lite_multi_phase_t final_phase)
{
    ctx->multi_grasp = false;
    ctx->grasp_armed = false;
    ctx->multi_stop_requested = false;
    ctx->multi_phase = final_phase;
    ctx->multi_completed = ball_manifest_region_count(
        &ctx->manifest, BALL_MANIFEST_REGION_TURNTABLE);
    (void)snprintf(ctx->text, sizeof(ctx->text),
        "MULTI %s COUNT=%u/%u RECORDS=%u\r\n",
        lite_multi_phase_name(final_phase),
        (unsigned)ctx->multi_completed,
        (unsigned)ball_manifest_region_expected(
            BALL_MANIFEST_REGION_TURNTABLE),
        (unsigned)ctx->manifest.count);
    (void)debug_uart1_write_text(&ctx->debug, ctx->text);
}

static void lite_multi_fail(lite_context_t *ctx, const char *reason)
{
    ctx->multi_grasp = false;
    ctx->grasp_armed = false;
    ctx->multi_stop_requested = false;
    ctx->multi_phase = LITE_MULTI_FAULT;
    ctx->multi_fault = (reason != NULL) ? reason : "UNKNOWN";
    if (ctx->vision_phase != LITE_VISION_IDLE) {
        ctx->vision_enabled = false;
        ctx->vision_stop_requested = true;
    }
    (void)snprintf(ctx->text, sizeof(ctx->text),
        "MULTI FAULT=%s COUNT=%u RECORDS=%u. NO AUTO CONTINUE\r\n",
        ctx->multi_fault, (unsigned)ctx->multi_completed,
        (unsigned)ctx->manifest.count);
    (void)debug_uart1_write_text(&ctx->debug, ctx->text);
}

static bool lite_multi_start_next_vision(lite_context_t *ctx)
{
    if ((ctx->pose != LITE_POSE_VISION_11) ||
        (lite_start_session(ctx, ctx->target_color, true, true) !=
         NANO_VISION_OK)) {
        lite_multi_fail(ctx, "VISION_REARM");
        return false;
    }
    ctx->multi_phase = LITE_MULTI_WAIT_VISION;
    (void)snprintf(ctx->text, sizeof(ctx->text),
        "MULTI WAIT BALL %u/%u SESSION=%u\r\n",
        (unsigned)(ball_manifest_region_count(
            &ctx->manifest, BALL_MANIFEST_REGION_TURNTABLE) + 1U),
        (unsigned)ball_manifest_region_expected(
            BALL_MANIFEST_REGION_TURNTABLE),
        (unsigned)ctx->active_session_id);
    (void)debug_uart1_write_text(&ctx->debug, ctx->text);
    return true;
}

static zdt_turntable_status_t lite_submit_motion(
    lite_context_t *ctx,
    zdt_turntable_direction_t direction,
    uint32_t angle_0p1deg,
    uint16_t emm_speed_rpm)
{
    zdt_turntable_position_command_t command = {0};

    command.direction = direction;
    command.mode = ZDT_TURNTABLE_POS_RELATIVE_LAST_TARGET;
    command.speed = 600U;
    command.acceleration = 200U;
    command.deceleration = 200U;
    command.angle_0p1deg = angle_0p1deg;
    command.emm_acceleration = TURN_GRASP_LITE_ZDT_ACCEL;
    if (ctx->zdt.firmware == ZDT_TURNTABLE_FIRMWARE_EMM) {
        command.speed = emm_speed_rpm;
    }
    return zdt_turntable_device_move_angle(
        &ctx->zdt, &command, lite_zdt_done, ctx);
}

static void lite_slot_finish(
    lite_context_t *ctx, bool success, const char *reason)
{
    uint8_t fine_steps = ctx->slot.fine_steps;
    const char *direction =
        (ctx->slot.direction == ZDT_TURNTABLE_DIR_CW) ? "CW" : "CCW";

    ctx->slot.active = false;
    ctx->slot.phase = LITE_SLOT_IDLE;
    if (success) {
        (void)snprintf(ctx->text, sizeof(ctx->text),
            "SLOT OK DIR=%s FINE=%u PB0=1\r\n",
            direction, (unsigned)fine_steps);
    } else {
        (void)snprintf(ctx->text, sizeof(ctx->text),
            "SLOT ERROR DIR=%s FINE=%u PB0=%u %s\r\n",
            direction, (unsigned)fine_steps,
            gate_read() ? 1U : 0U,
            (reason != NULL) ? reason : "UNKNOWN");
    }
    (void)debug_uart1_write_text(&ctx->debug, ctx->text);
    if (ctx->multi_phase != LITE_MULTI_WAIT_SLOT) return;
    if (!success) {
        lite_multi_fail(ctx, (reason != NULL) ? reason : "SLOT");
    } else if (ctx->multi_stop_requested) {
        lite_multi_finish(ctx, LITE_MULTI_STOPPED);
    } else {
        (void)lite_multi_start_next_vision(ctx);
    }
}

static void lite_slot_schedule_poll(lite_context_t *ctx, bool after_fine)
{
    ctx->slot.phase = after_fine ?
        LITE_SLOT_WAIT_FINE_POLL : LITE_SLOT_WAIT_COARSE_POLL;
    ctx->slot.next_poll_tick = lite_now() +
        lite_ms_to_ticks(TURN_GRASP_LITE_ZDT_STATUS_POLL_MS);
}

static bool lite_slot_submit_fine(lite_context_t *ctx)
{
    zdt_turntable_status_t status = lite_submit_motion(
        ctx, ctx->slot.direction,
        TURN_GRASP_LITE_ZDT_FINE_ANGLE_0P1DEG,
        TURN_GRASP_LITE_ZDT_FINE_SPEED_RPM);

    if (status != ZDT_TURNTABLE_OK) {
        lite_slot_finish(ctx, false, "FINE_SUBMIT");
        return false;
    }
    ++ctx->slot.fine_steps;
    ctx->slot.phase = LITE_SLOT_WAIT_FINE_ACK;
    return true;
}

static void lite_slot_handle_zdt(lite_context_t *ctx)
{
    bool ack_phase;
    bool status_phase;
    bool after_fine;
    const zdt_turntable_response_t *response = &ctx->zdt_mail_response;

    if ((ctx->zdt_mail_status != ZDT_TURNTABLE_OK) ||
        !ctx->zdt_mail_has_response) {
        lite_slot_finish(ctx, false, "ZDT_RESPONSE");
        return;
    }
    ack_phase = (ctx->slot.phase == LITE_SLOT_WAIT_COARSE_ACK) ||
                (ctx->slot.phase == LITE_SLOT_WAIT_FINE_ACK);
    if (ack_phase) {
        if ((response->kind != ZDT_TURNTABLE_REPLY_ACK) &&
            (response->kind != ZDT_TURNTABLE_REPLY_REACHED)) {
            lite_slot_finish(ctx, false, "MOTION_ACK");
            return;
        }
        lite_slot_schedule_poll(
            ctx, ctx->slot.phase == LITE_SLOT_WAIT_FINE_ACK);
        return;
    }
    status_phase = (ctx->slot.phase == LITE_SLOT_WAIT_COARSE_STATUS) ||
                   (ctx->slot.phase == LITE_SLOT_WAIT_FINE_STATUS);
    if (!status_phase || (response->kind != ZDT_TURNTABLE_REPLY_STATUS)) {
        lite_slot_finish(ctx, false, "STATE_MISMATCH");
        return;
    }
    after_fine = (ctx->slot.phase == LITE_SLOT_WAIT_FINE_STATUS);
    if (!response->data.motor_status.enabled ||
        response->data.motor_status.stalled ||
        response->data.motor_status.stall_protected ||
        response->data.motor_status.power_loss_latched) {
        lite_slot_finish(ctx, false, "MOTOR_FAULT");
        return;
    }
    if (!response->data.motor_status.reached) {
        lite_slot_schedule_poll(ctx, after_fine);
    } else if (lite_gate_is_stably_high()) {
        lite_slot_finish(ctx, true, NULL);
    } else if (ctx->slot.fine_steps >=
               TURN_GRASP_LITE_ZDT_FINE_MAX_STEPS) {
        lite_slot_finish(ctx, false, "GATE_NOT_FOUND");
    } else {
        (void)lite_slot_submit_fine(ctx);
    }
}

static void lite_handle_zdt(lite_context_t *ctx)
{
    bool options_ok;

    if (ctx->slot.active) {
        lite_slot_handle_zdt(ctx);
        return;
    }
    options_ok = (ctx->zdt_mail_status == ZDT_TURNTABLE_OK) &&
        ctx->zdt_mail_has_response &&
        (ctx->zdt_mail_response.kind == ZDT_TURNTABLE_REPLY_OPTIONS);
    if (options_ok) {
        (void)snprintf(ctx->text, sizeof(ctx->text),
            "ZDT OPTIONS FW=%s CLOSED=%u SCALE=%u RAW=%04X\r\n",
            (ctx->zdt.firmware == ZDT_TURNTABLE_FIRMWARE_EMM) ? "EMM" : "X",
            ctx->zdt.closed_loop ? 1U : 0U,
            ctx->zdt.scaled_input ? 1U : 0U,
            (unsigned)ctx->zdt_mail_response.data.options.raw_flags);
    } else {
        (void)snprintf(ctx->text, sizeof(ctx->text),
            "ZDT ERROR=%u HAS_RESPONSE=%u\r\n",
            (unsigned)ctx->zdt_mail_status,
            ctx->zdt_mail_has_response ? 1U : 0U);
    }
    (void)debug_uart1_write_text(&ctx->debug, ctx->text);
    if (ctx->run_pending) {
        if (options_ok && ctx->zdt.firmware_known && ctx->zdt.closed_loop) {
            lite_run_continue(ctx);
        } else {
            lite_run_cancel(ctx, "ZDT_OPTIONS");
        }
    }
}

static zdt_turntable_status_t lite_slot_start(lite_context_t *ctx)
{
    zdt_turntable_status_t status;

    ctx->slot.active = true;
    ctx->slot.phase = LITE_SLOT_WAIT_COARSE_ACK;
    ctx->slot.direction = TURN_GRASP_LITE_SLOT_USE_CW ?
        ZDT_TURNTABLE_DIR_CW : ZDT_TURNTABLE_DIR_CCW;
    ctx->slot.fine_steps = 0U;
    ctx->slot.started_tick = lite_now();
    status = lite_submit_motion(
        ctx, ctx->slot.direction,
        TURN_GRASP_LITE_ZDT_COARSE_ANGLE_0P1DEG,
        TURN_GRASP_LITE_ZDT_SPEED_RPM);
    if (status != ZDT_TURNTABLE_OK) {
        ctx->slot.active = false;
        ctx->slot.phase = LITE_SLOT_IDLE;
    }
    return status;
}

static void lite_slot_process(lite_context_t *ctx)
{
    zdt_turntable_status_t status;
    bool poll_due;

    if (!ctx->slot.active) return;
    if ((lite_now() - ctx->slot.started_tick) >=
        lite_ms_to_ticks(TURN_GRASP_LITE_ZDT_SLOT_TIMEOUT_MS)) {
        lite_slot_finish(ctx, false, "TIMEOUT");
        return;
    }
    poll_due = (ctx->slot.phase == LITE_SLOT_WAIT_COARSE_POLL) ||
               (ctx->slot.phase == LITE_SLOT_WAIT_FINE_POLL);
    if (!poll_due || !lite_time_reached(lite_now(), ctx->slot.next_poll_tick)) {
        return;
    }
    ctx->slot.phase = (ctx->slot.phase == LITE_SLOT_WAIT_COARSE_POLL) ?
        LITE_SLOT_WAIT_COARSE_STATUS : LITE_SLOT_WAIT_FINE_STATUS;
    status = zdt_turntable_device_query(
        &ctx->zdt, 0x3AU, lite_zdt_done, ctx);
    if (status != ZDT_TURNTABLE_OK) {
        lite_slot_finish(ctx, false, "STATUS_SUBMIT");
    }
}

static void lite_multi_process_ic(lite_context_t *ctx)
{
    ic_card_status_t status;

    if (!ctx->multi_grasp ||
        (ctx->multi_phase != LITE_MULTI_WAIT_IC_SUBMIT) ||
        !lite_time_reached(lite_now(), ctx->multi_next_action_tick) ||
        ctx->ic_transport.active || ctx->zdt_transport.active ||
        ctx->vision_inflight || ctx->slot.active) {
        return;
    }
    ++ctx->multi_ic_attempts;
    status = ic_card_device_read_competition_ball(
        TURN_GRASP_LITE_IC_OPERATION_PROMPT != 0U,
        lite_ic_read_done, ctx);
    if (status == IC_CARD_OK) {
        ctx->multi_phase = LITE_MULTI_WAIT_IC;
        (void)snprintf(ctx->text, sizeof(ctx->text),
            "MULTI IC REQUEST ATTEMPT=%u/%u\r\n",
            (unsigned)ctx->multi_ic_attempts,
            (unsigned)TURN_GRASP_LITE_IC_MAX_ATTEMPTS);
        (void)debug_uart1_write_text(&ctx->debug, ctx->text);
    } else if (ctx->multi_ic_attempts >=
               TURN_GRASP_LITE_IC_MAX_ATTEMPTS) {
        lite_multi_fail(ctx, "IC_SUBMIT");
    } else {
        ctx->multi_next_action_tick = lite_now() +
            lite_ms_to_ticks(TURN_GRASP_LITE_IC_RETRY_MS);
    }
}

static void lite_handle_ic(lite_context_t *ctx)
{
    ball_manifest_color_t color;
    ball_manifest_status_t manifest_status;
    zdt_turntable_status_t zdt_status;

    if (!ctx->multi_grasp ||
        (ctx->multi_phase != LITE_MULTI_WAIT_IC)) return;
    if (ctx->ic_mail_status != IC_CARD_OK) {
        if (ctx->multi_ic_attempts >= TURN_GRASP_LITE_IC_MAX_ATTEMPTS) {
            lite_multi_fail(ctx, "IC_READ");
        } else {
            ctx->multi_phase = LITE_MULTI_WAIT_IC_SUBMIT;
            ctx->multi_next_action_tick = lite_now() +
                lite_ms_to_ticks(TURN_GRASP_LITE_IC_RETRY_MS);
            (void)snprintf(ctx->text, sizeof(ctx->text),
                "MULTI IC ERROR=%u RETRY=%u/%u\r\n",
                (unsigned)ctx->ic_mail_status,
                (unsigned)(ctx->multi_ic_attempts + 1U),
                (unsigned)TURN_GRASP_LITE_IC_MAX_ATTEMPTS);
            (void)debug_uart1_write_text(&ctx->debug, ctx->text);
        }
        return;
    }

    color = (ctx->target_color == NANO_VISION_COLOR_RED) ?
        BALL_MANIFEST_COLOR_RED : BALL_MANIFEST_COLOR_BLUE;
    manifest_status = ball_manifest_append(
        &ctx->manifest, BALL_MANIFEST_REGION_TURNTABLE, color,
        ctx->ic_mail_ball.ball.code, ctx->ic_mail_ball.ball.row,
        ctx->ic_mail_ball.ball.column, ctx->manifest.count);
    if (manifest_status != BALL_MANIFEST_OK) {
        lite_multi_fail(ctx, "MANIFEST_APPEND");
        return;
    }
    ctx->multi_completed = ball_manifest_region_count(
        &ctx->manifest, BALL_MANIFEST_REGION_TURNTABLE);
    (void)snprintf(ctx->text, sizeof(ctx->text),
        "MULTI RECORD SEQ=%u CODE=0x%02X ROW=%u COL=%u SLOT=%u COUNT=%u/%u\r\n",
        (unsigned)(ctx->manifest.count - 1U),
        (unsigned)ctx->ic_mail_ball.ball.code,
        (unsigned)ctx->ic_mail_ball.ball.row,
        (unsigned)ctx->ic_mail_ball.ball.column,
        (unsigned)(ctx->manifest.count - 1U),
        (unsigned)ctx->multi_completed,
        (unsigned)ball_manifest_region_expected(
            BALL_MANIFEST_REGION_TURNTABLE));
    (void)debug_uart1_write_text(&ctx->debug, ctx->text);

    if (ball_manifest_region_is_complete(
            &ctx->manifest, BALL_MANIFEST_REGION_TURNTABLE)) {
        lite_multi_finish(ctx, LITE_MULTI_COMPLETE);
        return;
    }
    if (ctx->multi_stop_requested) {
        lite_multi_finish(ctx, LITE_MULTI_STOPPED);
        return;
    }
    if ((ctx->pose != LITE_POSE_VISION_11) ||
        !lite_gate_is_stably_high()) {
        lite_multi_fail(ctx, "SLOT_PRECHECK");
        return;
    }
#if TURN_GRASP_LITE_ZDT_MOTION_ARMED
    ctx->multi_phase = LITE_MULTI_WAIT_SLOT;
    zdt_status = lite_slot_start(ctx);
    if (zdt_status != ZDT_TURNTABLE_OK) {
        lite_multi_fail(ctx, "SLOT_SUBMIT");
        return;
    }
    (void)snprintf(ctx->text, sizeof(ctx->text),
        "MULTI SLOT START DIR=%s AFTER RECORD=%u\r\n",
        TURN_GRASP_LITE_SLOT_USE_CW ? "CW" : "CCW",
        (unsigned)(ctx->manifest.count - 1U));
    (void)debug_uart1_write_text(&ctx->debug, ctx->text);
#else
    (void)zdt_status;
    lite_multi_fail(ctx, "ZDT_LOCKED");
#endif
}

static bool lite_multi_start(
    lite_context_t *ctx, nano_vision_color_t target_color)
{
    uint8_t i;
    ball_manifest_record_t record;

    if (ctx->multi_grasp || ctx->slot.active || ctx->action_active ||
        ctx->vision_inflight || (ctx->vision_phase != LITE_VISION_IDLE) ||
        ctx->ic_transport.active || ctx->zdt_transport.active ||
        ctx->fault_latched) {
        (void)debug_uart1_write_text(&ctx->debug,
            "MULTI START REJECTED: BUSY\r\n");
        return false;
    }
    if (ctx->multi_phase == LITE_MULTI_FAULT) {
        (void)debug_uart1_write_text(&ctx->debug,
            "MULTI START REJECTED: FAULT. CHECK HARDWARE THEN BALL_RESET\r\n");
        return false;
    }
    if (ctx->pose != LITE_POSE_VISION_11) {
        (void)debug_uart1_write_text(&ctx->debug,
            "MULTI START REJECTED: ARM NOT VISION11\r\n");
        return false;
    }
    if ((target_color != NANO_VISION_COLOR_RED) &&
        (target_color != NANO_VISION_COLOR_BLUE)) {
        return false;
    }
    if (ball_manifest_validate(&ctx->manifest) != BALL_MANIFEST_OK) {
        (void)debug_uart1_write_text(&ctx->debug,
            "MULTI START REJECTED: MANIFEST CORRUPT\r\n");
        return false;
    }
    if (ball_manifest_region_is_complete(
            &ctx->manifest, BALL_MANIFEST_REGION_TURNTABLE)) {
        (void)debug_uart1_write_text(&ctx->debug,
            "MULTI START REJECTED: TURNTABLE COMPLETE 5/5\r\n");
        return false;
    }
    if (!ctx->zdt.firmware_known || !ctx->zdt.closed_loop) {
        (void)debug_uart1_write_text(&ctx->debug,
            "MULTI START REJECTED: SEND ZDT_OPTIONS FIRST\r\n");
        return false;
    }
    if (!lite_gate_is_stably_high()) {
        (void)debug_uart1_write_text(&ctx->debug,
            "MULTI START REJECTED: PB0 NOT STABLY HIGH\r\n");
        return false;
    }
    for (i = 0U; i < ctx->manifest.count; ++i) {
        if ((ball_manifest_get(&ctx->manifest, i, &record) !=
             BALL_MANIFEST_OK) ||
            ((record.color == BALL_MANIFEST_COLOR_RED) !=
             (target_color == NANO_VISION_COLOR_RED))) {
            (void)debug_uart1_write_text(&ctx->debug,
                "MULTI START REJECTED: RECORD COLOR MISMATCH\r\n");
            return false;
        }
    }
    ctx->target_color = target_color;
    ctx->multi_completed = ball_manifest_region_count(
        &ctx->manifest, BALL_MANIFEST_REGION_TURNTABLE);
    ctx->multi_ic_attempts = 0U;
    ctx->multi_stop_requested = false;
    ctx->multi_fault = NULL;
    if (!lite_multi_start_next_vision(ctx)) return false;
    (void)snprintf(ctx->text, sizeof(ctx->text),
        "MULTI START COLOR=%u COUNT=%u/%u PB0=1 EVENT_MODE=1\r\n",
        (unsigned)target_color, (unsigned)ctx->multi_completed,
        (unsigned)ball_manifest_region_expected(
            BALL_MANIFEST_REGION_TURNTABLE));
    (void)debug_uart1_write_text(&ctx->debug, ctx->text);
    return true;
}

static void lite_run_cancel(lite_context_t *ctx, const char *reason)
{
    if (!ctx->run_pending) return;
    ctx->run_pending = false;
    (void)snprintf(ctx->text, sizeof(ctx->text),
        "RUN ABORTED=%s\r\n", (reason != NULL) ? reason : "UNKNOWN");
    (void)debug_uart1_write_text(&ctx->debug, ctx->text);
}

static void lite_run_continue(lite_context_t *ctx)
{
    zdt_turntable_status_t zdt_status;
    lsc16_status_t arm_status;
    nano_vision_color_t color;

    if (!ctx->run_pending) return;
    if (ctx->fault_latched || (ctx->multi_phase == LITE_MULTI_FAULT)) {
        lite_run_cancel(ctx, "FAULT");
        return;
    }
    if (!ctx->zdt.firmware_known) {
        if (ctx->zdt_transport.active) return;
        zdt_status = zdt_turntable_device_query_options(
            &ctx->zdt, lite_zdt_done, ctx);
        if (zdt_status != ZDT_TURNTABLE_OK) {
            lite_run_cancel(ctx, "ZDT_SUBMIT");
        } else {
            (void)debug_uart1_write_text(&ctx->debug,
                "RUN PREP ZDT OPTIONS\r\n");
        }
        return;
    }
    if (!ctx->zdt.closed_loop) {
        lite_run_cancel(ctx, "ZDT_NOT_CLOSED_LOOP");
        return;
    }
    if (ctx->action_active || ctx->arm_tx_inflight) return;
    if (ctx->pose == LITE_POSE_UNKNOWN) {
        arm_status = lite_submit_action(ctx, TURN_GRASP_LITE_HOME_GROUP);
        if (arm_status != LSC16_OK) lite_run_cancel(ctx, "ARM_HOME");
        return;
    }
    if (ctx->pose == LITE_POSE_HOME_10) {
        arm_status = lite_submit_action(ctx, TURN_GRASP_LITE_VISION_GROUP);
        if (arm_status != LSC16_OK) lite_run_cancel(ctx, "ARM_VISION");
        return;
    }
    if (ctx->pose != LITE_POSE_VISION_11) {
        lite_run_cancel(ctx, "ARM_POSE");
        return;
    }

    color = ctx->run_color;
    ctx->run_pending = false;
    if (!lite_multi_start(ctx, color)) {
        (void)debug_uart1_write_text(&ctx->debug,
            "RUN ABORTED=MULTI_START\r\n");
    } else {
        (void)debug_uart1_write_text(&ctx->debug,
            "RUN ACTIVE\r\n");
    }
}

static bool lite_run_start(
    lite_context_t *ctx, nano_vision_color_t target_color)
{
    if (ctx->run_pending || ctx->multi_grasp || ctx->slot.active ||
        ctx->action_active || ctx->arm_tx_inflight || ctx->vision_inflight ||
        (ctx->vision_phase != LITE_VISION_IDLE) || ctx->ic_transport.active ||
        ctx->zdt_transport.active || ctx->fault_latched ||
        (ctx->multi_phase == LITE_MULTI_FAULT)) {
        return false;
    }
    ctx->run_pending = true;
    ctx->run_color = target_color;
    (void)snprintf(ctx->text, sizeof(ctx->text),
        "RUN START COLOR=%s. AUTO ZDT->ARM10->ARM11->MULTI\r\n",
        (target_color == NANO_VISION_COLOR_RED) ? "RED" : "BLUE");
    (void)debug_uart1_write_text(&ctx->debug, ctx->text);
    lite_run_continue(ctx);
    return ctx->run_pending || ctx->multi_grasp;
}

static void lite_multi_request_stop(lite_context_t *ctx)
{
    if (!ctx->multi_grasp) {
        (void)debug_uart1_write_text(&ctx->debug, "MULTI NOT ACTIVE\r\n");
        return;
    }
    ctx->multi_stop_requested = true;
    if (ctx->multi_phase == LITE_MULTI_WAIT_VISION) {
        lite_stop_session(ctx);
        lite_multi_finish(ctx, LITE_MULTI_STOPPED);
        return;
    }
    (void)snprintf(ctx->text, sizeof(ctx->text),
        "MULTI STOP REQUESTED PHASE=%s. WAIT SAFE BOUNDARY\r\n",
        lite_multi_phase_name(ctx->multi_phase));
    (void)debug_uart1_write_text(&ctx->debug, ctx->text);
}

static void lite_print_manifest(lite_context_t *ctx)
{
    uint8_t i;
    ball_manifest_record_t record;

    (void)snprintf(ctx->text, sizeof(ctx->text),
        "BALL STATUS VALID=%u TOTAL=%u TURNTABLE=%u/%u\r\n",
        (ball_manifest_validate(&ctx->manifest) == BALL_MANIFEST_OK) ? 1U : 0U,
        (unsigned)ctx->manifest.count,
        (unsigned)ball_manifest_region_count(
            &ctx->manifest, BALL_MANIFEST_REGION_TURNTABLE),
        (unsigned)ball_manifest_region_expected(
            BALL_MANIFEST_REGION_TURNTABLE));
    (void)debug_uart1_write_text(&ctx->debug, ctx->text);
    for (i = 0U; i < ctx->manifest.count; ++i) {
        if (ball_manifest_get(&ctx->manifest, i, &record) != BALL_MANIFEST_OK) {
            break;
        }
        (void)snprintf(ctx->text, sizeof(ctx->text),
            "BALL SEQ=%u COLOR=%u CODE=0x%02X ROW=%u COL=%u SLOT=%u STATE=%u\r\n",
            (unsigned)record.sequence, (unsigned)record.color,
            (unsigned)record.ic_code, (unsigned)record.target_row,
            (unsigned)record.target_column, (unsigned)record.storage_slot,
            (unsigned)record.state);
        (void)debug_uart1_write_text(&ctx->debug, ctx->text);
    }
}

static void lite_handle_vision(lite_context_t *ctx)
{
    lite_capture_t capture;
    nano_vision_status_t status;
    size_t emitted = 0U;
    uint32_t elapsed;
    lite_vision_phase_t completed_phase = ctx->vision_phase;
    nano_vision_session_t session;
    nano_vision_event_t event;
    uint16_t stopped_session = 0U;
    bool should_grasp;
    bool was_multi;

    status = lite_map_mult_status(ctx->vision_mail_status);
    if (ctx->vision_stop_requested &&
        (completed_phase != LITE_VISION_STOPPING)) {
        if (lite_submit_session_stop(ctx) != NANO_VISION_OK) {
            ++ctx->vision_error_count;
            lite_reset_session(ctx, true);
            (void)debug_uart1_write_text(&ctx->debug,
                "VISION STOP SEND FAILED. LOCAL SESSION CLEARED\r\n");
        }
        return;
    }
    if ((completed_phase == LITE_VISION_LISTENING) &&
        (status == NANO_VISION_ERR_TIMEOUT)) {
        ++ctx->read_wait_count;
        return;
    }
    if ((completed_phase == LITE_VISION_ACKING) &&
        (status == NANO_VISION_OK)) {
        should_grasp = ctx->grasp_armed;
        lite_reset_session(ctx, false);
        if (!should_grasp) {
            (void)debug_uart1_write_text(&ctx->debug,
                "WATCH EVENT ACKED. SESSION COMPLETE\r\n");
            return;
        }
        if (ctx->multi_grasp) {
            if (ctx->multi_phase != LITE_MULTI_WAIT_VISION) {
                lite_multi_fail(ctx, "VISION_PHASE");
                return;
            }
            ctx->multi_phase = LITE_MULTI_WAIT_GRASP12;
        }
        ++ctx->trigger_count;
        if (lite_submit_action(ctx, TURN_GRASP_LITE_GRASP_GROUP) != LSC16_OK) {
            if (ctx->multi_grasp) {
                lite_multi_fail(ctx, "GRASP12_SUBMIT");
            } else {
                ctx->fault_latched = true;
            }
            (void)debug_uart1_write_text(&ctx->debug,
                "GRASP SUBMIT FAILED. FAULT LATCHED\r\n");
        } else {
            (void)debug_uart1_write_text(&ctx->debug,
                ctx->multi_grasp ?
                    "GRASP TRIGGER GROUP=12 MULTI\r\n" :
                    "GRASP TRIGGER GROUP=12 ONE_SHOT\r\n");
        }
        return;
    }
    if ((completed_phase == LITE_VISION_ACKING) &&
        (status != NANO_VISION_OK)) {
        ++ctx->vision_error_count;
        ctx->vision_phase = LITE_VISION_LISTENING;
        return;
    }
    if ((completed_phase == LITE_VISION_STOPPING) &&
        (status != NANO_VISION_OK)) {
        ++ctx->vision_error_count;
        lite_reset_session(ctx, true);
        (void)debug_uart1_write_text(&ctx->debug,
            "VISION STOP TIMEOUT. LOCAL SESSION CLEARED\r\n");
        return;
    }
    if (status != NANO_VISION_OK) {
        ++ctx->vision_error_count;
        was_multi = ctx->multi_grasp;
        lite_reset_session(ctx, true);
        if (was_multi) lite_multi_fail(ctx, "VISION_TRANSFER");
        (void)snprintf(ctx->text, sizeof(ctx->text),
            "VISION ERROR=%u PHASE=%u ERR=%lu\r\n",
            (unsigned)status, (unsigned)completed_phase,
            (unsigned long)ctx->vision_error_count);
        (void)debug_uart1_write_text(&ctx->debug, ctx->text);
        return;
    }

    (void)memset(&capture, 0, sizeof(capture));
    status = nano_vision_parser_feed(
        &ctx->parser, ctx->vision_mail_data, ctx->vision_mail_len,
        lite_capture_frame, &capture, &emitted);
    if ((status != NANO_VISION_OK) || !capture.matched) {
        status = (ctx->parser.crc_errors > 0U) ?
            NANO_VISION_ERR_CRC : NANO_VISION_ERR_LENGTH;
        ++ctx->vision_error_count;
        if (completed_phase == LITE_VISION_STARTING) {
            was_multi = ctx->multi_grasp;
            lite_reset_session(ctx, true);
            if (was_multi) lite_multi_fail(ctx, "VISION_FRAME");
        }
        return;
    }

    if (completed_phase == LITE_VISION_STARTING) {
        status = nano_vision_parse_session_ready(&capture.frame, &session);
        if ((status != NANO_VISION_OK) ||
            (capture.frame.sequence != ctx->active_sequence) ||
            (session.session_id != ctx->active_session_id) ||
            (session.scene != NANO_VISION_SCENE_TURNTABLE) ||
            (session.target_color != ctx->target_color)) {
            ++ctx->vision_error_count;
            was_multi = ctx->multi_grasp;
            lite_reset_session(ctx, true);
            if (was_multi) lite_multi_fail(ctx, "VISION_READY");
            (void)debug_uart1_write_text(&ctx->debug,
                "VISION READY MISMATCH. SESSION CLEARED\r\n");
            return;
        }
        ++ctx->session_count;
        ctx->vision_phase = LITE_VISION_LISTENING;
        (void)snprintf(ctx->text, sizeof(ctx->text),
            "VISION READY SESSION=%u\r\n",
            (unsigned)ctx->active_session_id);
        (void)debug_uart1_write_text(&ctx->debug, ctx->text);
        return;
    }
    if (completed_phase == LITE_VISION_STOPPING) {
        status = nano_vision_parse_session_stopped(
            &capture.frame, &stopped_session);
        if ((status != NANO_VISION_OK) ||
            (stopped_session != ctx->active_session_id)) {
            ++ctx->vision_error_count;
        }
        lite_reset_session(ctx, true);
        (void)debug_uart1_write_text(&ctx->debug, "VISION STOPPED\r\n");
        return;
    }
    if (completed_phase != LITE_VISION_LISTENING) return;

    status = nano_vision_parse_event(&capture.frame, &event);
    if ((status != NANO_VISION_OK) || !ctx->vision_enabled ||
        (event.session_id != ctx->active_session_id) ||
        (event.observation.status != NANO_VISION_OBS_VALID) ||
        (event.observation.scene != NANO_VISION_SCENE_TURNTABLE) ||
        (event.observation.color != ctx->target_color) ||
        (event.observation.age_ms > TURN_GRASP_LITE_EVENT_MAX_AGE_MS)) {
        ++ctx->vision_error_count;
        return;
    }
    elapsed = lite_ticks_to_ms(lite_now() - ctx->armed_tick);
    if (event.observation.age_ms > elapsed) return;
    if (ctx->have_last_event_frame &&
        (event.observation.frame_id == ctx->last_event_frame)) {
        if (lite_submit_event_ack(ctx, event.observation.frame_id) !=
            NANO_VISION_OK) {
            ++ctx->vision_error_count;
            ctx->vision_phase = LITE_VISION_LISTENING;
        }
        return;
    }
    ctx->have_last_event_frame = true;
    ctx->last_event_frame = event.observation.frame_id;
    ++ctx->event_count;
    (void)snprintf(ctx->text, sizeof(ctx->text),
        "VISION EVENT SESSION=%u FRAME=%u AGE=%u DX=%d DY=%d\r\n",
        (unsigned)event.session_id,
        (unsigned)event.observation.frame_id,
        (unsigned)event.observation.age_ms,
        (int)event.observation.offset_x_px,
        (int)event.observation.offset_y_px);
    (void)debug_uart1_write_text(&ctx->debug, ctx->text);
    if (lite_submit_event_ack(ctx, event.observation.frame_id) !=
        NANO_VISION_OK) {
        ++ctx->vision_error_count;
        ctx->vision_phase = LITE_VISION_LISTENING;
        (void)debug_uart1_write_text(&ctx->debug,
            "VISION ACK SUBMIT FAILED. WAIT RETRY\r\n");
    }
}

static void lite_handle_arm_report(lite_context_t *ctx)
{
    uint32_t events;
    lsc16_report_t report;

    taskENTER_CRITICAL();
    events = ctx->arm_report_events;
    ctx->arm_report_events = 0U;
    report = ctx->arm_report;
    taskEXIT_CRITICAL();

    if ((events & LSC16_REPORT_EVENT_INVALID_FRAME) != 0U) {
        (void)debug_uart1_write_text(&ctx->debug,
            "ARM INVALID REPORT IGNORED\r\n");
    }
    if ((events & LSC16_REPORT_EVENT_ACTION_STOPPED) != 0U) {
        ctx->action_active = false;
        ctx->fault_latched = false;
        ctx->pose = LITE_POSE_UNKNOWN;
        if (ctx->multi_grasp) {
            lite_multi_finish(ctx, LITE_MULTI_STOPPED);
        }
        lite_run_cancel(ctx, "ARM_STOPPED");
        (void)debug_uart1_write_text(&ctx->debug,
            "ARM STOPPED. FAULT CLEARED\r\n");
    }
    if ((events & (LSC16_REPORT_EVENT_ACTION_STARTED |
                   LSC16_REPORT_EVENT_ACTION_COMPLETED)) == 0U) return;
    if (!ctx->action_active || (report.action_group != ctx->active_group)) {
        (void)debug_uart1_write_text(&ctx->debug,
            "ARM REPORT GROUP MISMATCH IGNORED\r\n");
        return;
    }
    if ((events & LSC16_REPORT_EVENT_ACTION_STARTED) != 0U) {
        (void)snprintf(ctx->text, sizeof(ctx->text),
            "ARM STARTED GROUP=%u\r\n", (unsigned)report.action_group);
        (void)debug_uart1_write_text(&ctx->debug, ctx->text);
    }
    if ((events & LSC16_REPORT_EVENT_ACTION_COMPLETED) == 0U) return;

    ctx->action_active = false;
    (void)snprintf(ctx->text, sizeof(ctx->text),
        "ARM COMPLETED GROUP=%u\r\n", (unsigned)report.action_group);
    (void)debug_uart1_write_text(&ctx->debug, ctx->text);
    if (report.action_group == TURN_GRASP_LITE_HOME_GROUP) {
        ctx->pose = LITE_POSE_HOME_10;
        lite_run_continue(ctx);
    } else if (report.action_group == TURN_GRASP_LITE_VISION_GROUP) {
        ctx->pose = LITE_POSE_VISION_11;
        if (ctx->multi_grasp) {
            if (ctx->multi_phase != LITE_MULTI_WAIT_RETURN11) {
                lite_multi_fail(ctx, "RETURN11_PHASE");
            } else if (ctx->multi_stop_requested) {
                lite_multi_finish(ctx, LITE_MULTI_STOPPED);
            } else {
                ctx->multi_ic_attempts = 0U;
                ctx->multi_next_action_tick = lite_now();
                ctx->multi_phase = LITE_MULTI_WAIT_IC_SUBMIT;
                (void)debug_uart1_write_text(&ctx->debug,
                    "MULTI ARM VISION11 CONFIRMED. START IC READ\r\n");
            }
        } else {
            lite_run_continue(ctx);
        }
    } else if (report.action_group == TURN_GRASP_LITE_GRASP_GROUP) {
        ctx->pose = LITE_POSE_UNKNOWN;
        if (ctx->multi_grasp) {
            if (ctx->multi_phase != LITE_MULTI_WAIT_GRASP12) {
                lite_multi_fail(ctx, "GRASP12_PHASE");
                return;
            }
            ctx->multi_phase = LITE_MULTI_WAIT_RETURN11;
        }
        if (lite_submit_action(ctx, TURN_GRASP_LITE_VISION_GROUP) != LSC16_OK) {
            if (ctx->multi_grasp) {
                lite_multi_fail(ctx, "RETURN11_SUBMIT");
            } else {
                ctx->fault_latched = true;
            }
            (void)debug_uart1_write_text(&ctx->debug,
                "AUTO RETURN GROUP11 FAILED. FAULT LATCHED\r\n");
        }
    }
}

static void lite_print_status(lite_context_t *ctx)
{
    (void)snprintf(ctx->text, sizeof(ctx->text),
        "STATUS POSE=%s ACTION=%u GROUP=%u FAULT=%u VISION=%u ARMED=%u "
        "PHASE=%u SESSION=%u COLOR=%u RUN=%u "
        "SESSIONS=%lu READ_WAIT=%lu EVENT=%lu "
        "TRIGGER=%lu VISION_ERR=%lu REJECT=%lu\r\n",
        lite_pose_name(ctx->pose), ctx->action_active ? 1U : 0U,
        (unsigned)ctx->active_group, ctx->fault_latched ? 1U : 0U,
        ctx->vision_enabled ? 1U : 0U, ctx->grasp_armed ? 1U : 0U,
        (unsigned)ctx->vision_phase, (unsigned)ctx->active_session_id,
        (unsigned)ctx->target_color, ctx->run_pending ? 1U : 0U,
        (unsigned long)ctx->session_count,
        (unsigned long)ctx->read_wait_count,
        (unsigned long)ctx->event_count, (unsigned long)ctx->trigger_count,
        (unsigned long)ctx->vision_error_count,
        (unsigned long)ctx->rejected_count);
    (void)debug_uart1_write_text(&ctx->debug, ctx->text);
    (void)snprintf(ctx->text, sizeof(ctx->text),
        "MULTI ACTIVE=%u PHASE=%s STOP=%u COUNT=%u/%u RECORDS=%u "
        "IC_ACTIVE=%u IC_TRY=%u SLOT_ACTIVE=%u SLOT_PHASE=%u FINE=%u "
        "ZDT_KNOWN=%u PB0=%u LAST_FAULT=%s\r\n",
        ctx->multi_grasp ? 1U : 0U,
        lite_multi_phase_name(ctx->multi_phase),
        ctx->multi_stop_requested ? 1U : 0U,
        (unsigned)ctx->multi_completed,
        (unsigned)ball_manifest_region_expected(
            BALL_MANIFEST_REGION_TURNTABLE),
        (unsigned)ctx->manifest.count,
        ctx->ic_transport.active ? 1U : 0U,
        (unsigned)ctx->multi_ic_attempts,
        ctx->slot.active ? 1U : 0U,
        (unsigned)ctx->slot.phase,
        (unsigned)ctx->slot.fine_steps,
        ctx->zdt.firmware_known ? 1U : 0U,
        gate_read() ? 1U : 0U,
        (ctx->multi_fault != NULL) ? ctx->multi_fault : "NONE");
    (void)debug_uart1_write_text(&ctx->debug, ctx->text);
}

static void lite_handle_command(lite_context_t *ctx)
{
    uint8_t command[DEBUG_UART1_RX_BUFFER_SIZE];
    size_t len = 0U;
    lsc16_status_t status;
    zdt_turntable_status_t zdt_status;

    if (!debug_uart1_take_message(
            &ctx->debug, command, sizeof(command), &len)) return;
    if (lite_command_equals(command, len, "HELP")) {
        (void)debug_uart1_write_text(&ctx->debug,
            "HELP STATUS RUN_RED RUN_BLUE ARM_HOME ARM_VISION WATCH_RED WATCH_BLUE "
            "GRASP_RED GRASP_BLUE GRASP_MULTI_RED GRASP_MULTI_BLUE "
            "GRASP_SINGLE VISION_STOP MULTI_STOP ARM_STOP ZDT_OPTIONS "
            "BALL_STATUS BALL_RESET\r\n");
        return;
    }
    if (lite_command_equals(command, len, "STATUS")) {
        lite_print_status(ctx);
        return;
    }
    if (lite_command_equals(command, len, "VISION_STOP")) {
        lite_run_cancel(ctx, "USER_STOP");
        if (ctx->multi_grasp) {
            lite_multi_request_stop(ctx);
        } else {
            lite_stop_session(ctx);
            (void)debug_uart1_write_text(&ctx->debug,
                "VISION STOP REQUESTED\r\n");
        }
        return;
    }
    if (lite_command_equals(command, len, "MULTI_STOP")) {
        if (ctx->run_pending) {
            lite_run_cancel(ctx, "USER_STOP");
        } else {
            lite_multi_request_stop(ctx);
        }
        return;
    }
    if (lite_command_equals(command, len, "BALL_STATUS")) {
        lite_print_manifest(ctx);
        return;
    }
    if (lite_command_equals(command, len, "BALL_RESET")) {
        if (ctx->run_pending || ctx->multi_grasp || ctx->action_active ||
            ctx->vision_inflight ||
            (ctx->vision_phase != LITE_VISION_IDLE) ||
            ctx->ic_transport.active || ctx->zdt_transport.active ||
            ctx->slot.active) {
            (void)debug_uart1_write_text(&ctx->debug,
                "BALL RESET REJECTED: BUSY\r\n");
        } else {
            ball_manifest_init(&ctx->manifest);
            ctx->multi_phase = LITE_MULTI_IDLE;
            ctx->multi_stop_requested = false;
            ctx->multi_ic_attempts = 0U;
            ctx->multi_completed = 0U;
            ctx->multi_fault = NULL;
            (void)debug_uart1_write_text(&ctx->debug,
                "BALL RESET OK. CONFIRM PHYSICAL STORAGE EMPTY\r\n");
        }
        return;
    }
    if (lite_command_equals(command, len, "ZDT_OPTIONS")) {
        if (ctx->run_pending || ctx->multi_grasp || ctx->action_active ||
            ctx->vision_inflight ||
            (ctx->vision_phase != LITE_VISION_IDLE) ||
            ctx->ic_transport.active || ctx->zdt_transport.active ||
            ctx->slot.active) {
            (void)debug_uart1_write_text(&ctx->debug,
                "ZDT OPTIONS REJECTED: BUSY\r\n");
            return;
        }
        zdt_status = zdt_turntable_device_query_options(
            &ctx->zdt, lite_zdt_done, ctx);
        (void)snprintf(ctx->text, sizeof(ctx->text),
            "ZDT OPTIONS REQUEST=%u\r\n", (unsigned)zdt_status);
        (void)debug_uart1_write_text(&ctx->debug, ctx->text);
        return;
    }
    if (lite_command_equals(command, len, "ARM_STOP")) {
        lite_run_cancel(ctx, "USER_STOP");
        if (ctx->multi_grasp) {
            lite_multi_finish(ctx, LITE_MULTI_STOPPED);
        }
        lite_stop_session(ctx);
        status = lsc16_device_stop_action_group(lite_arm_done, ctx);
        if (status == LSC16_OK) ctx->arm_tx_inflight = true;
        ctx->pose = LITE_POSE_UNKNOWN;
        (void)debug_uart1_write_text(&ctx->debug, "ARM STOP REQUESTED\r\n");
        return;
    }
    if (lite_command_equals(command, len, "WATCH_RED") ||
        lite_command_equals(command, len, "WATCH_BLUE")) {
        if (ctx->run_pending || ctx->multi_grasp || (lite_start_session(ctx,
            lite_command_equals(command, len, "WATCH_RED") ?
                NANO_VISION_COLOR_RED : NANO_VISION_COLOR_BLUE,
            false, false) != NANO_VISION_OK)) {
            ++ctx->rejected_count;
            (void)debug_uart1_write_text(&ctx->debug,
                "WATCH REJECTED: VISION BUSY\r\n");
        }
        return;
    }

#if TURN_GRASP_LITE_MOTION_ARMED
    if (lite_command_equals(command, len, "RUN_RED") ||
        lite_command_equals(command, len, "RUN_BLUE")) {
        if (!lite_run_start(ctx,
                lite_command_equals(command, len, "RUN_RED") ?
                    NANO_VISION_COLOR_RED : NANO_VISION_COLOR_BLUE)) {
            status = LSC16_ERR_STATE;
        } else {
            return;
        }
    } else if (lite_command_equals(command, len, "ARM_HOME")) {
        if (ctx->run_pending || ctx->multi_grasp) {
            status = LSC16_ERR_STATE;
        } else {
            lite_stop_session(ctx);
            status = lite_submit_action(ctx, TURN_GRASP_LITE_HOME_GROUP);
        }
    } else if (lite_command_equals(command, len, "ARM_VISION")) {
        status = (!ctx->run_pending && !ctx->multi_grasp &&
                  (ctx->pose == LITE_POSE_HOME_10)) ?
            lite_submit_action(ctx, TURN_GRASP_LITE_VISION_GROUP) :
            LSC16_ERR_STATE;
    } else if (lite_command_equals(command, len, "GRASP_SINGLE")) {
        if (!ctx->run_pending && !ctx->multi_grasp) lite_stop_session(ctx);
        status = (!ctx->run_pending && !ctx->multi_grasp &&
                  (ctx->pose == LITE_POSE_VISION_11)) ?
            lite_submit_action(ctx, TURN_GRASP_LITE_GRASP_GROUP) :
            LSC16_ERR_STATE;
        if (status == LSC16_OK) ++ctx->trigger_count;
    } else if (lite_command_equals(command, len, "GRASP_RED") ||
               lite_command_equals(command, len, "GRASP_BLUE")) {
        if (ctx->run_pending || ctx->multi_grasp ||
            (ctx->pose != LITE_POSE_VISION_11) ||
            ctx->action_active ||
            ctx->fault_latched) {
            status = LSC16_ERR_STATE;
        } else {
            if (lite_start_session(ctx,
                lite_command_equals(command, len, "GRASP_RED") ?
                    NANO_VISION_COLOR_RED : NANO_VISION_COLOR_BLUE,
                true, false) != NANO_VISION_OK) {
                ++ctx->rejected_count;
                (void)debug_uart1_write_text(&ctx->debug,
                    "GRASP REJECTED: VISION BUSY\r\n");
            }
            return;
        }
    } else if (lite_command_equals(command, len, "GRASP_MULTI_RED") ||
               lite_command_equals(command, len, "GRASP_MULTI_BLUE")) {
        if (ctx->run_pending || (ctx->pose != LITE_POSE_VISION_11) ||
            ctx->action_active ||
            ctx->fault_latched ||
            !lite_multi_start(
                ctx, lite_command_equals(command, len, "GRASP_MULTI_RED") ?
                    NANO_VISION_COLOR_RED : NANO_VISION_COLOR_BLUE)) {
            status = LSC16_ERR_STATE;
        } else {
            return;
        }
    } else {
        (void)debug_uart1_write_text(&ctx->debug, "UNKNOWN COMMAND\r\n");
        return;
    }
    if (status != LSC16_OK) {
        ++ctx->rejected_count;
        (void)snprintf(ctx->text, sizeof(ctx->text),
            "COMMAND REJECTED ERROR=%u\r\n", (unsigned)status);
        (void)debug_uart1_write_text(&ctx->debug, ctx->text);
    }
#else
    (void)debug_uart1_write_text(&ctx->debug,
        "MOTION LOCKED. WATCH COMMANDS REMAIN AVAILABLE\r\n");
#endif
}

static void lite_task_entry(void *argument)
{
    lite_context_t *ctx = (lite_context_t *)argument;
    uint32_t flags;

    (void)debug_uart1_write_text(&ctx->debug,
        "TURNTABLE GRASP IC SLOT READY. SEND HELP\r\n");
    for (;;) {
        flags = osThreadFlagsWait(
            LITE_FLAGS_ALL, osFlagsWaitAny, lite_ms_to_ticks(2U));
        if ((flags & osFlagsError) == 0U) {
            if ((flags & LITE_FLAG_VISION_DONE) != 0U) {
                ctx->vision_inflight = false;
                lite_handle_vision(ctx);
            }
            if ((flags & LITE_FLAG_ARM_TX_DONE) != 0U) {
                ctx->arm_tx_inflight = false;
                if (ctx->arm_mail_status != LSC16_OK) {
                    ctx->fault_latched = true;
                    ctx->action_active = false;
                    if (ctx->multi_grasp) {
                        lite_multi_fail(ctx, "ARM_TX");
                    }
                    lite_run_cancel(ctx, "ARM_TX");
                    (void)debug_uart1_write_text(&ctx->debug,
                        "ARM TX ERROR. FAULT LATCHED\r\n");
                }
            }
            if ((flags & LITE_FLAG_ARM_REPORT) != 0U) {
                lite_handle_arm_report(ctx);
            }
            if ((flags & LITE_FLAG_IC_DONE) != 0U) {
                lite_handle_ic(ctx);
            }
            if ((flags & LITE_FLAG_ZDT_DONE) != 0U) {
                lite_handle_zdt(ctx);
            }
        }
        lite_handle_command(ctx);
        if (ctx->action_active &&
            lite_time_reached(lite_now(), ctx->action_deadline)) {
            ctx->fault_latched = true;
            ctx->action_active = false;
            if (ctx->multi_grasp) {
                lite_multi_fail(ctx, "ARM_TIMEOUT");
            }
            lite_run_cancel(ctx, "ARM_TIMEOUT");
            lite_stop_session(ctx);
            (void)debug_uart1_write_text(&ctx->debug,
                "ARM ACTION TIMEOUT. FAULT LATCHED; SEND ARM_STOP\r\n");
        }
        if (!ctx->vision_inflight &&
            (ctx->vision_phase == LITE_VISION_LISTENING) &&
            ctx->vision_enabled) {
            if (lite_submit_event_read(ctx) != NANO_VISION_OK) {
                ++ctx->vision_error_count;
            }
        }
        lite_multi_process_ic(ctx);
        lite_slot_process(ctx);
    }
}

mult_uart_status_t turntable_grasp_lite_freertos_test_init(void)
{
    lite_context_t *ctx = &g_lite;
    zdt_turntable_device_config_t zdt_config = {
        TURN_GRASP_LITE_ZDT_ADDRESS,
        TURN_GRASP_LITE_IO_TIMEOUT_MS,
        TURN_GRASP_LITE_ZDT_EMM_PULSES_PER_REV,
    };

    (void)memset(ctx, 0, sizeof(*ctx));
    ctx->pose = LITE_POSE_UNKNOWN;
    ctx->multi_phase = LITE_MULTI_IDLE;
    ball_manifest_init(&ctx->manifest);
    if (!debug_uart1_init(&ctx->debug)) return MULT_UART_ERR_IO;
    if (ic_card_device_init_with_transport(
            lite_ic_submit, &ctx->ic_transport) != IC_CARD_OK) {
        debug_uart1_deinit(&ctx->debug);
        return MULT_UART_ERR_IO;
    }
    if (zdt_turntable_device_init_with_submit(
            &ctx->zdt, lite_zdt_submit, &ctx->zdt_transport,
            &zdt_config) != ZDT_TURNTABLE_OK) {
        debug_uart1_deinit(&ctx->debug);
        return MULT_UART_ERR_IO;
    }
    if (lsc16_device_set_report_callback(lite_arm_report, ctx) != LSC16_OK) {
        debug_uart1_deinit(&ctx->debug);
        return MULT_UART_ERR_IO;
    }
    ctx->task = osThreadNew(lite_task_entry, ctx, &g_lite_task_attr);
    if (ctx->task == NULL) {
        (void)lsc16_device_set_report_callback(NULL, NULL);
        debug_uart1_deinit(&ctx->debug);
        return MULT_UART_ERR_IO;
    }
    return MULT_UART_OK;
}

#endif /* TURNTABLE_GRASP_LITE_FREERTOS_TEST_ENABLED */
