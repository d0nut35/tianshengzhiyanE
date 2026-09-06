/**
 * @file    mult_uart_modules_freertos_test.c
 * @brief   USART1命令控制UART7复用器并测试IC卡与ZDT的FreeRTOS场景。
 */

#include "mult_uart_modules_freertos_test.h"

#include <stdio.h>
#include <string.h>

#include "cmsis_os.h"
#include "ball_manifest_core.h"
#include "debug_uart1.h"
#include "ic_card_device.h"
#include "ic_card_test_common.h"
#include "lsc16_device.h"
#include "lsc16_test_common.h"
#include "mult_uart_device.h"
#include "mult_uart_modules_test_config.h"
#include "mult_uart_service_os.h"
#include "nano_vision_core.h"
#include "gate.h"
#include "test_config.h"
#include "zdt_turntable_device.h"
#include "zdt_turntable_test_common.h"

#if MULT_UART_MODULES_FREERTOS_TEST_ENABLED

#define MODULES_EVENT_QUEUE_DEPTH  8U
#define MODULES_TASK_STACK_SIZE   (1024U * 4U)
#define MODULES_TEXT_SIZE          256U
#define MODULES_COMMAND_WATCHDOG_MS 2000U

typedef enum {
    MODULES_EVENT_SELECT = 0,
    MODULES_EVENT_VISION,
    MODULES_EVENT_IC_READ,
    MODULES_EVENT_IC_QUERY,
    MODULES_EVENT_ZDT,
    MODULES_EVENT_LSC16_TX_DONE,
    MODULES_EVENT_LSC16_REPORT,
} modules_event_kind_t;

typedef struct {
    modules_event_kind_t kind;
    mult_uart_status_t mult_status;
    ic_card_status_t ic_status;
    zdt_turntable_status_t zdt_status;
    lsc16_status_t lsc16_status;
    nano_vision_status_t vision_status;
    uint8_t channel;
    nano_vision_observation_t vision_observation;
    uint8_t vision_streak;
    uint32_t vision_timeouts;
    uint32_t vision_crc_errors;
    uint32_t vision_sequence_errors;
    bool vision_online;
    bool vision_aligned;
    ic_card_ball_result_t ball;
    ic_card_response_t ic_response;
    zdt_turntable_response_t zdt_response;
    lsc16_report_t lsc16_report;
    uint32_t lsc16_report_events;
    uint8_t lsc16_action_group;
    bool lsc16_is_stop;
    bool has_response;
} modules_event_t;

typedef struct {
    bool active;
    ic_card_request_t request;
    uint8_t tx[IC_CARD_COMMAND_FRAME_SIZE];
} modules_ic_transport_t;

typedef struct {
    bool active;
    zdt_turntable_request_t request;
} modules_zdt_transport_t;

/** 通道0一笔轮询及跨轮询连续对齐状态。 */
typedef struct {
    bool active;
    bool tracker_initialized;
    uint8_t next_sequence;
    uint8_t active_sequence;
    uint8_t rx_chunk_count;
    nano_vision_parser_t parser;
    nano_vision_tracker_t tracker;
    nano_vision_tracker_config_t tracker_config;
    uint8_t tx[NANO_VISION_FRAME_MAX];
} modules_vision_transport_t;

/** USART1开始/停止命令控制的连续视觉轮询会话。 */
typedef struct {
    bool enabled;
    bool stop_pending;
    bool have_last_result;
    bool last_online;
    bool last_aligned;
    bool early_compare_enabled;
    bool early_ready;
    bool strict_ready;
    nano_vision_scene_t scene;
    nano_vision_color_t target_color;
    nano_vision_early_config_t early_config;
    nano_vision_status_t last_transport_status;
    uint8_t last_observation_status;
    uint32_t next_poll_tick;
    uint32_t poll_count;
    uint32_t completed_count;
    uint32_t error_count;
    uint32_t submit_error_count;
    uint32_t early_enter_count;
    uint32_t strict_enter_count;
} modules_vision_session_t;

/** 已由LSC16自然完成回报确认的机械臂姿态。 */
typedef enum {
    MODULES_ARM_POSE_UNKNOWN = 0,
    MODULES_ARM_POSE_HOME_10,
    MODULES_ARM_POSE_VISION_11,
    MODULES_ARM_POSE_GRASP_PLACED_12,
} modules_arm_pose_t;

/** 单球动作组12的一次性触发来源。 */
typedef enum {
    MODULES_GRASP_TRIGGER_NONE = 0,
    MODULES_GRASP_TRIGGER_MANUAL,
    MODULES_GRASP_TRIGGER_STATIC,
    MODULES_GRASP_TRIGGER_MOVING,
} modules_grasp_trigger_t;

/** UART8动作生命周期；只由uart7Modules任务修改，回调只投递事件。 */
typedef struct {
    lsc16_test_action_guard_t guard;
    modules_arm_pose_t pose;
    modules_grasp_trigger_t trigger;
    bool tx_pending;
    bool pending_is_stop;
    bool stop_deferred;
    bool returning_to_vision;
    bool auto_return_pending;
    uint8_t active_group;
    uint8_t last_completed_group;
    uint32_t action_submit_count;
    uint32_t action_started_count;
    uint32_t action_completed_count;
    uint32_t trigger_count;
    uint32_t rejected_count;
    uint32_t mismatched_report_count;
} modules_grasp_control_t;

/** 自动逐槽命令在多次ZDT事务之间保持的阶段。 */
typedef enum {
    MODULES_SLOT_IDLE = 0,
    MODULES_SLOT_WAIT_COARSE_ACK,
    MODULES_SLOT_WAIT_COARSE_POLL,
    MODULES_SLOT_WAIT_COARSE_STATUS,
    MODULES_SLOT_WAIT_FINE_ACK,
    MODULES_SLOT_WAIT_FINE_POLL,
    MODULES_SLOT_WAIT_FINE_STATUS,
} modules_slot_state_t;

/** 自动逐槽状态；只由uart7Modules测试任务读写。 */
typedef struct {
    bool active;
    modules_slot_state_t state;
    zdt_turntable_direction_t direction;
    uint8_t fine_steps;
    uint32_t started_tick;
    uint32_t next_poll_tick;
} modules_slot_control_t;

/** 高速多球抓取在视觉、机械臂、IC和内部转盘之间的唯一推进阶段。 */
typedef enum {
    MODULES_MULTI_IDLE = 0,
    MODULES_MULTI_WAIT_VISION,
    MODULES_MULTI_WAIT_GRASP12,
    MODULES_MULTI_WAIT_RETURN11,
    MODULES_MULTI_WAIT_IC_SUBMIT,
    MODULES_MULTI_WAIT_IC,
    MODULES_MULTI_WAIT_SLOT,
    MODULES_MULTI_COMPLETE,
    MODULES_MULTI_STOPPED,
    MODULES_MULTI_FAULT,
} modules_multi_phase_t;

/**
 * 高速多球状态只由uart7Modules任务修改。回调只投递事件，避免IC、ZDT和
 * LSC16回调并发修改球档案或跨设备状态。
 */
typedef struct {
    bool active;
    bool stop_requested;
    modules_multi_phase_t phase;
    nano_vision_color_t target_color;
    ball_manifest_region_t region;
    uint8_t ic_attempts;
    uint32_t next_action_tick;
    uint32_t completed_cycles;
    uint32_t fault_count;
    const char *last_fault;
} modules_multi_control_t;

typedef struct {
    bool initialized;
    bool command_pending;
    uint32_t command_started_tick;
    uint8_t selected_channel;
    bool selected_channel_valid;
    volatile bool select_completion_ready;
    volatile mult_uart_status_t select_completion_status;
    volatile uint8_t select_completion_channel;
    volatile uint32_t select_callback_count;
    volatile uint32_t event_drop_count;
    uint32_t command_watchdog_count;
    debug_uart1_t debug;
    osMessageQueueId_t event_queue;
    osThreadId_t task;
    zdt_turntable_device_t zdt;
    modules_vision_transport_t vision_transport;
    modules_vision_session_t vision_session;
    modules_grasp_control_t grasp;
    modules_ic_transport_t ic_transport;
    modules_zdt_transport_t zdt_transport;
    modules_slot_control_t slot;
    ball_manifest_t manifest;
    modules_multi_control_t multi;
    char text[MODULES_TEXT_SIZE];
} modules_test_context_t;

static modules_test_context_t g_modules_test;

static void modules_multi_fail(
    modules_test_context_t *ctx, const char *reason);
static void modules_multi_on_action_completed(
    modules_test_context_t *ctx, uint8_t completed_group, bool was_returning);
static void modules_multi_on_slot_finished(
    modules_test_context_t *ctx, bool success, const char *reason);
static void modules_multi_handle_ic_event(
    modules_test_context_t *ctx, const modules_event_t *event);
static void modules_multi_process(modules_test_context_t *ctx);
static bool modules_gate_is_stably_high(void);

static const osThreadAttr_t g_modules_task_attr = {
    .name = "uart7Modules",
    .stack_size = MODULES_TASK_STACK_SIZE,
    .priority = (osPriority_t)osPriorityNormal,
};

/** @brief 把mult_uart错误映射到IC卡Device使用的统一错误域。 */
static ic_card_status_t modules_map_ic_status(mult_uart_status_t status)
{
    if (status == MULT_UART_OK) return IC_CARD_OK;
    if (status == MULT_UART_ERR_TIMEOUT) return IC_CARD_ERR_TIMEOUT;
    if (status == MULT_UART_ERR_BUSY) return IC_CARD_ERR_BUSY;
    if (status == MULT_UART_ERR_QUEUE_FULL) return IC_CARD_ERR_QUEUE_FULL;
    if ((status == MULT_UART_ERR_PARAM) || (status == MULT_UART_ERR_OVERFLOW))
        return IC_CARD_ERR_PARAM;
    return IC_CARD_ERR_IO;
}

/** @brief 把mult_uart错误映射到ZDT Device使用的统一错误域。 */
static zdt_turntable_status_t modules_map_zdt_status(mult_uart_status_t status)
{
    if (status == MULT_UART_OK) return ZDT_TURNTABLE_OK;
    if (status == MULT_UART_ERR_TIMEOUT) return ZDT_TURNTABLE_ERR_TIMEOUT;
    if (status == MULT_UART_ERR_BUSY) return ZDT_TURNTABLE_ERR_BUSY;
    if (status == MULT_UART_ERR_QUEUE_FULL) return ZDT_TURNTABLE_ERR_QUEUE_FULL;
    if ((status == MULT_UART_ERR_PARAM) || (status == MULT_UART_ERR_OVERFLOW))
        return ZDT_TURNTABLE_ERR_PARAM;
    return ZDT_TURNTABLE_ERR_IO;
}

/** @brief 把mult_uart事务结果映射到Nano视觉错误域。 */
static nano_vision_status_t modules_map_vision_status(mult_uart_status_t status)
{
    if (status == MULT_UART_OK) return NANO_VISION_OK;
    if (status == MULT_UART_ERR_TIMEOUT) return NANO_VISION_ERR_TIMEOUT;
    if (status == MULT_UART_ERR_BUSY) return NANO_VISION_ERR_BUSY;
    if (status == MULT_UART_ERR_QUEUE_FULL) return NANO_VISION_ERR_QUEUE_FULL;
    return NANO_VISION_ERR_IO;
}

/**
 * @brief 将协议结果按值投递给唯一测试任务。
 * @return 投递成功返回true；参数、队列或RTOS投递异常返回false。
 * @warning 本函数可能运行在mult_uart worker中，不允许阻塞或写USART1。
 * @note 必须检查osMessageQueuePut返回值，否则一次通知丢失就会让命令门永久BUSY。
 */
static bool modules_put_event(const modules_event_t *event)
{
    if ((event == NULL) || (g_modules_test.event_queue == NULL)) {
        g_modules_test.event_drop_count++;
        return false;
    }
    if (osMessageQueuePut(g_modules_test.event_queue, event, 0U, 0U) != osOK) {
        g_modules_test.event_drop_count++;
        return false;
    }
    return true;
}

/** @brief 完成一次IC卡复用事务并立即解析临时RX缓冲。 */
static void modules_ic_transfer_done(
    void *user_ctx,
    const mult_uart_device_completion_t *completion)
{
    modules_ic_transport_t *transport = (modules_ic_transport_t *)user_ctx;
    ic_card_response_t response;
    ic_card_status_t status;
    ic_card_request_t request;

    if ((transport == NULL) || !transport->active || (completion == NULL)) return;
    request = transport->request;
    transport->active = false;
    status = modules_map_ic_status(completion->status);
    if (status == IC_CARD_OK) {
        status = ic_parse_frame(
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
            request.user_ctx,
            request.request_id,
            status,
            ((status == IC_CARD_OK) || (status == IC_CARD_ERR_CARD)) ?
                &response : NULL);
    }
}

/** @brief 把IC卡Device请求构帧后提交到复用通道1。 */
static ic_card_status_t modules_ic_submit(
    void *submit_ctx,
    const ic_card_request_t *request,
    uint32_t queue_timeout_ms)
{
    modules_ic_transport_t *transport = (modules_ic_transport_t *)submit_ctx;
    mult_uart_device_transfer_t transfer;
    mult_uart_status_t mult_status;
    ic_card_status_t status;
    size_t tx_len;

    if ((transport == NULL) || (request == NULL)) return IC_CARD_ERR_PARAM;
    if (transport->active) return IC_CARD_ERR_BUSY;
    if (request->type == IC_CARD_REQUEST_READ_BLOCK) {
        status = ic_read_frame(
            request->address,
            request->data.read_block.block,
            request->data.read_block.led_beep_prompt,
            transport->tx,
            sizeof(transport->tx),
            &tx_len);
    } else if (request->type == IC_CARD_REQUEST_QUERY) {
        status = ic_query_frame(
            request->data.query.command,
            request->address,
            transport->tx,
            sizeof(transport->tx),
            &tx_len);
    } else {
        return IC_CARD_ERR_UNSUPPORTED;
    }
    if (status != IC_CARD_OK) return status;

    transport->request = *request;
    transport->active = true;
    (void)memset(&transfer, 0, sizeof(transfer));
    transfer.device_id = MULT_UART_MODULES_IC_DEVICE_ID;
    transfer.operation = MULT_UART_OP_WRITE_READ;
    transfer.tx_data = transport->tx;
    transfer.tx_len = tx_len;
    transfer.rx_capacity = IC_CARD_FRAME_SIZE_MAX;
    transfer.io_timeout_ms = request->timeout_ms;
    transfer.queue_timeout_ms = queue_timeout_ms;
    transfer.done_cb = modules_ic_transfer_done;
    transfer.user_ctx = transport;
    mult_status = mult_uart_device_submit(&transfer);
    if (mult_status != MULT_UART_OK) transport->active = false;
    return modules_map_ic_status(mult_status);
}

/** @brief 完成一次ZDT复用事务并在RX指针失效前解析响应。 */
static void modules_zdt_transfer_done(
    void *user_ctx,
    const mult_uart_device_completion_t *completion)
{
    modules_zdt_transport_t *transport = (modules_zdt_transport_t *)user_ctx;
    zdt_turntable_request_t request;
    zdt_turntable_response_t response;
    zdt_turntable_status_t status;

    if ((transport == NULL) || !transport->active || (completion == NULL)) return;
    request = transport->request;
    transport->active = false;
    status = modules_map_zdt_status(completion->status);
    if (status == ZDT_TURNTABLE_OK) {
        status = turn_parse(
            completion->rx_data,
            completion->rx_len,
            request.expected_address,
            request.expected_function,
            &response);
    }
    if (request.done_cb != NULL) {
        request.done_cb(
            request.user_ctx,
            request.request_id,
            status,
            ((status == ZDT_TURNTABLE_OK) ||
             (status == ZDT_TURNTABLE_ERR_DEVICE)) ? &response : NULL);
    }
}

/** @brief 把ZDT Device事务转换为复用通道2的WRITE_READ。 */
static zdt_turntable_status_t modules_zdt_submit(
    void *submit_ctx,
    const zdt_turntable_request_t *request)
{
    modules_zdt_transport_t *transport = (modules_zdt_transport_t *)submit_ctx;
    mult_uart_device_transfer_t transfer;
    mult_uart_status_t status;

    if ((transport == NULL) || (request == NULL)) return ZDT_TURNTABLE_ERR_PARAM;
    if (transport->active) return ZDT_TURNTABLE_ERR_BUSY;
    transport->request = *request;
    transport->active = true;
    (void)memset(&transfer, 0, sizeof(transfer));
    transfer.device_id = MULT_UART_MODULES_ZDT_DEVICE_ID;
    transfer.operation = MULT_UART_OP_WRITE_READ;
    transfer.tx_data = request->frame;
    transfer.tx_len = request->frame_len;
    transfer.rx_capacity = ZDT_TURNTABLE_RESPONSE_MAX;
    transfer.io_timeout_ms = request->timeout_ms;
    transfer.done_cb = modules_zdt_transfer_done;
    transfer.user_ctx = transport;
    status = mult_uart_device_submit(&transfer);
    if (status != MULT_UART_OK) transport->active = false;
    return modules_map_zdt_status(status);
}

typedef struct {
    bool saw_observation;
    bool found_matching;
    uint8_t observed_sequence;
    nano_vision_status_t parse_status;
    nano_vision_observation_t observation;
} modules_vision_capture_t;

static void modules_vision_transfer_done(
    void *user_ctx,
    const mult_uart_device_completion_t *completion);

/**
 * @brief 在同一通道继续等待下一段RX数据。
 * @note 每次READ仍是一笔mult_uart_device事务；不绕过通道选择、Service或DMA。
 */
static nano_vision_status_t modules_submit_vision_read_continue(
    modules_vision_transport_t *transport)
{
    mult_uart_device_transfer_t transfer;
    mult_uart_status_t mult_status;

    if ((transport == NULL) || transport->active) {
        return NANO_VISION_ERR_BUSY;
    }
    transport->active = true;
    (void)memset(&transfer, 0, sizeof(transfer));
    transfer.device_id = MULT_UART_MODULES_VISION_DEVICE_ID;
    transfer.operation = MULT_UART_OP_READ;
    transfer.rx_capacity = MULT_UART_SERVICE_RX_MAX;
    transfer.io_timeout_ms = MULT_UART_MODULES_VISION_IO_TIMEOUT_MS;
    transfer.done_cb = modules_vision_transfer_done;
    transfer.user_ctx = transport;
    mult_status = mult_uart_device_submit(&transfer);
    if (mult_status != MULT_UART_OK) transport->active = false;
    return modules_map_vision_status(mult_status);
}

/** @brief 从一次DMA结果中的分包/粘包流里寻找当前SEQ的观测帧。 */
static void modules_vision_capture_frame(
    void *user_ctx,
    const nano_vision_frame_t *frame)
{
    modules_vision_capture_t *capture =
        (modules_vision_capture_t *)user_ctx;

    if ((capture == NULL) || (frame == NULL) ||
        (frame->type != NANO_VISION_MSG_OBSERVATION)) {
        return;
    }
    capture->saw_observation = true;
    capture->observed_sequence = frame->sequence;
    if (frame->sequence != g_modules_test.vision_transport.active_sequence) {
        return;
    }
    capture->parse_status = nano_vision_parse_observation(
        frame, &capture->observation);
    capture->found_matching = true;
}

/** @brief 完成一次通道0WRITE_READ，并在RX指针失效前完成协议解析。 */
static void modules_vision_transfer_done(
    void *user_ctx,
    const mult_uart_device_completion_t *completion)
{
    modules_vision_transport_t *transport =
        (modules_vision_transport_t *)user_ctx;
    modules_vision_capture_t capture;
    modules_event_t event;
    nano_vision_status_t status;
    uint32_t crc_before;
    size_t emitted = 0U;

    if ((transport == NULL) || !transport->active || (completion == NULL)) return;
    transport->active = false;
    if (transport->rx_chunk_count < 0xFFU) ++transport->rx_chunk_count;
    (void)memset(&capture, 0, sizeof(capture));
    capture.parse_status = NANO_VISION_ERR_TYPE;
    status = modules_map_vision_status(completion->status);
    crc_before = transport->parser.crc_errors;

    if (status == NANO_VISION_OK) {
        status = nano_vision_parser_feed(
            &transport->parser,
            completion->rx_data,
            completion->rx_len,
            modules_vision_capture_frame,
            &capture,
            &emitted);
        if ((status == NANO_VISION_OK) && capture.found_matching) {
            status = capture.parse_status;
        } else if ((status == NANO_VISION_OK) &&
                   (transport->parser.crc_errors != crc_before)) {
            status = NANO_VISION_ERR_CRC;
        } else if ((status == NANO_VISION_OK) &&
                   (transport->rx_chunk_count <
                    MULT_UART_MODULES_VISION_MAX_RX_CHUNKS)) {
            if (capture.saw_observation) {
                ++transport->tracker.sequence_errors;
            }
            status = modules_submit_vision_read_continue(transport);
            if (status == NANO_VISION_OK) return;
        } else if (status == NANO_VISION_OK) {
            if (capture.saw_observation) {
                ++transport->tracker.sequence_errors;
                status = NANO_VISION_ERR_SEQUENCE;
            } else {
                status = (emitted == 0U) ?
                    NANO_VISION_ERR_LENGTH : NANO_VISION_ERR_TYPE;
            }
        }
        if ((status == NANO_VISION_OK) && capture.found_matching) {
            status = nano_vision_tracker_on_observation(
                &transport->tracker,
                &transport->tracker_config,
                transport->active_sequence,
                &capture.observation,
                0U);
        }
    }

    if (status != NANO_VISION_OK) {
        if (status == NANO_VISION_ERR_TIMEOUT) {
            nano_vision_tracker_on_timeout(
                &transport->tracker, &transport->tracker_config);
        } else {
            transport->tracker.waiting_response = false;
            transport->tracker.aligned_streak = 0U;
        }
    }

    (void)memset(&event, 0, sizeof(event));
    event.kind = MODULES_EVENT_VISION;
    event.vision_status = status;
    event.has_response = (status == NANO_VISION_OK);
    if (event.has_response) event.vision_observation = capture.observation;
    event.vision_online = transport->tracker.online;
    event.vision_aligned = nano_vision_tracker_is_aligned(
        &transport->tracker, &transport->tracker_config);
    event.vision_streak = transport->tracker.aligned_streak;
    event.vision_timeouts = transport->tracker.timeout_count;
    event.vision_crc_errors = transport->parser.crc_errors;
    event.vision_sequence_errors = transport->tracker.sequence_errors;
    (void)modules_put_event(&event);
}

/** @brief 通过mult_uart_device_submit向通道0提交一笔视觉轮询。 */
static nano_vision_status_t modules_submit_vision_poll(
    nano_vision_scene_t scene,
    nano_vision_color_t target_color)
{
    modules_vision_transport_t *transport =
        &g_modules_test.vision_transport;
    mult_uart_device_transfer_t transfer;
    nano_vision_poll_t poll;
    nano_vision_tracker_config_t config;
    nano_vision_status_t status;
    mult_uart_status_t mult_status;
    size_t tx_len = 0U;

    if (transport->active) return NANO_VISION_ERR_BUSY;
    config.scene = scene;
    config.target_color = target_color;
    config.tolerance_x_px = MULT_UART_MODULES_VISION_TOLERANCE_X_PX;
    config.tolerance_y_px = MULT_UART_MODULES_VISION_TOLERANCE_Y_PX;
    config.max_observation_age_ms = MULT_UART_MODULES_VISION_MAX_AGE_MS;
    config.link_timeout_ms = MULT_UART_MODULES_VISION_LINK_TIMEOUT_MS;
    config.required_confirm_frames = MULT_UART_MODULES_VISION_CONFIRM_FRAMES;
    config.disconnect_after_timeouts =
        MULT_UART_MODULES_VISION_DISCONNECT_TIMEOUTS;
    if (!transport->tracker_initialized ||
        (transport->tracker_config.scene != scene) ||
        (transport->tracker_config.target_color != target_color)) {
        status = nano_vision_tracker_init(&transport->tracker, &config);
        if (status != NANO_VISION_OK) return status;
        transport->tracker_config = config;
        transport->tracker_initialized = true;
    }

    transport->next_sequence++;
    if (transport->next_sequence == 0U) transport->next_sequence++;
    transport->active_sequence = transport->next_sequence;
    poll.scene = scene;
    poll.target_color = target_color;
    status = nano_vision_build_poll_frame(
        transport->active_sequence,
        &poll,
        transport->tx,
        sizeof(transport->tx),
        &tx_len);
    if (status != NANO_VISION_OK) return status;
    status = nano_vision_tracker_on_poll_sent(
        &transport->tracker, transport->active_sequence);
    if (status != NANO_VISION_OK) return status;

    transport->parser.count = 0U;
    transport->parser.expected_size = 0U;
    transport->rx_chunk_count = 0U;
    transport->active = true;
    (void)memset(&transfer, 0, sizeof(transfer));
    transfer.device_id = MULT_UART_MODULES_VISION_DEVICE_ID;
    transfer.operation = MULT_UART_OP_WRITE_READ;
    transfer.tx_data = transport->tx;
    transfer.tx_len = tx_len;
    transfer.rx_capacity = MULT_UART_SERVICE_RX_MAX;
    transfer.io_timeout_ms = MULT_UART_MODULES_VISION_IO_TIMEOUT_MS;
    transfer.done_cb = modules_vision_transfer_done;
    transfer.user_ctx = transport;
    mult_status = mult_uart_device_submit(&transfer);
    if (mult_status != MULT_UART_OK) {
        transport->active = false;
        transport->tracker.waiting_response = false;
    }
    return modules_map_vision_status(mult_status);
}

/**
 * @brief SELECT事务完成回调，把小结果写入专用单槽完成邮箱。
 * @note SELECT不依赖外设响应且全局只允许一条命令在途，因此一个单槽邮箱足够；
 *       它不再与IC/ZDT的大结果竞争通用事件队列。
 * @warning 本函数运行在mult_uart worker中，只复制数据，不写USART1。
 */
static void modules_select_done(
    void *user_ctx,
    const mult_uart_device_completion_t *completion)
{
    modules_test_context_t *ctx = &g_modules_test;
    (void)user_ctx;
    if (completion == NULL) return;
    ctx->select_completion_status = completion->status;
    ctx->select_completion_channel = (uint8_t)completion->device_id;
    ctx->select_callback_count++;
    __DMB();
    ctx->select_completion_ready = true;
}

/** @brief IC读球Device回调，把结果复制到测试事件。 */
static void modules_ic_read_done(
    void *user_ctx,
    uint32_t request_id,
    ic_card_status_t status,
    const ic_card_ball_result_t *result)
{
    modules_event_t event;
    (void)user_ctx;
    (void)request_id;
    (void)memset(&event, 0, sizeof(event));
    event.kind = MODULES_EVENT_IC_READ;
    event.ic_status = status;
    if ((status == IC_CARD_OK) && (result != NULL)) event.ball = *result;
    (void)modules_put_event(&event);
}

/** @brief IC参数查询回调，把完整响应复制到测试事件。 */
static void modules_ic_query_done(
    void *user_ctx,
    uint32_t request_id,
    ic_card_status_t status,
    const ic_card_response_t *response)
{
    modules_event_t event;
    (void)user_ctx;
    (void)request_id;
    (void)memset(&event, 0, sizeof(event));
    event.kind = MODULES_EVENT_IC_QUERY;
    event.ic_status = status;
    event.has_response = (response != NULL);
    if (response != NULL) event.ic_response = *response;
    (void)modules_put_event(&event);
}

/** @brief ZDT Device完成回调，把解析结果复制到测试事件。 */
static void modules_zdt_done(
    void *user_ctx,
    uint32_t request_id,
    zdt_turntable_status_t status,
    const zdt_turntable_response_t *response)
{
    modules_event_t event;
    (void)user_ctx;
    (void)request_id;
    (void)memset(&event, 0, sizeof(event));
    event.kind = MODULES_EVENT_ZDT;
    event.zdt_status = status;
    event.has_response = (response != NULL);
    if (response != NULL) event.zdt_response = *response;
    (void)modules_put_event(&event);
}

/** @brief 忽略首尾空白和大小写比较一条ASCII命令。 */
static bool modules_command_equals(
    const uint8_t *data, size_t len, const char *expected)
{
    size_t i;
    while ((len > 0U) && ((*data == ' ') || (*data == '\t'))) { data++; len--; }
    while ((len > 0U) && ((data[len - 1U] == ' ') ||
        (data[len - 1U] == '\t') || (data[len - 1U] == '\r') ||
        (data[len - 1U] == '\n'))) len--;
    if (len != strlen(expected)) return false;
    for (i = 0U; i < len; ++i) {
        uint8_t ch = data[i];
        if ((ch >= 'a') && (ch <= 'z')) ch = (uint8_t)(ch - ('a' - 'A'));
        if (ch != (uint8_t)expected[i]) return false;
    }
    return true;
}

/** @brief 解析SELECT 0..3并返回通道，非SELECT返回false。 */
static bool modules_parse_select(
    const uint8_t *data, size_t len, uint8_t *channel)
{
    while ((len > 0U) && ((data[len - 1U] == '\r') ||
        (data[len - 1U] == '\n') || (data[len - 1U] == ' '))) len--;
    if ((len != 8U) || (channel == NULL)) return false;
    if (!modules_command_equals(data, 6U, "SELECT")) return false;
    if ((data[6] != ' ') || (data[7] < '0') || (data[7] > '3')) return false;
    *channel = (uint8_t)(data[7] - '0');
    return true;
}

/** @brief 提交只切通道、不发送协议数据的SELECT事务。 */
static mult_uart_status_t modules_submit_select(uint8_t channel)
{
    mult_uart_device_transfer_t transfer;
    (void)memset(&transfer, 0, sizeof(transfer));
    transfer.device_id = (mult_uart_device_id_t)channel;
    transfer.operation = MULT_UART_OP_SELECT;
    transfer.done_cb = modules_select_done;
    return mult_uart_device_submit(&transfer);
}

/** @brief 记录一条命令开始等待完成的时刻，供BUSY门和看门狗共同使用。 */
static void modules_mark_command_pending(modules_test_context_t *ctx)
{
    ctx->command_started_tick = osKernelGetTickCount();
    ctx->command_pending = true;
}

/**
 * @brief 构造统一的固定角度逐槽命令。
 * @note 使用相对上一输入目标位置模式，使连续等角度命令以理论目标为基准；
 *       正式流程仍必须等待REACHED，不能把接收ACK当作机械到位。
 */
static zdt_turntable_status_t modules_submit_motion(
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
    command.emm_acceleration = MULT_UART_MODULES_ZDT_ACCEL;
    if (g_modules_test.zdt.firmware == ZDT_TURNTABLE_FIRMWARE_EMM) {
        command.speed = emm_speed_rpm;
    }
    return zdt_turntable_device_move_angle(
        &g_modules_test.zdt, &command, modules_zdt_done, NULL);
}

/** @brief 将毫秒向上换算为RTOS tick，避免短延时被截成0。 */
static uint32_t modules_ms_to_ticks(uint32_t milliseconds)
{
    uint32_t frequency = osKernelGetTickFreq();
    uint32_t ticks;

    if (frequency == 0U) return 1U;
    ticks = (uint32_t)(((uint64_t)milliseconds * frequency + 999ULL) / 1000ULL);
    return (ticks == 0U) ? 1U : ticks;
}

/** @brief 将内核tick换算为LSC16动作保护器使用的毫秒时间。 */
static uint32_t modules_now_ms(void)
{
    uint32_t frequency = osKernelGetTickFreq();
    uint32_t ticks = osKernelGetTickCount();

    if (frequency == 0U) return ticks;
    return (uint32_t)(((uint64_t)ticks * 1000ULL) / frequency);
}

/** @brief 返回机械臂姿态的固定诊断文本。 */
static const char *modules_arm_pose_name(modules_arm_pose_t pose)
{
    if (pose == MODULES_ARM_POSE_HOME_10) return "HOME10";
    if (pose == MODULES_ARM_POSE_VISION_11) return "VISION11";
    if (pose == MODULES_ARM_POSE_GRASP_PLACED_12) return "PLACED12";
    return "UNKNOWN";
}

/** @brief 返回一次性抓取触发方式的固定诊断文本。 */
static const char *modules_grasp_trigger_name(modules_grasp_trigger_t trigger)
{
    if (trigger == MODULES_GRASP_TRIGGER_MANUAL) return "MANUAL";
    if (trigger == MODULES_GRASP_TRIGGER_STATIC) return "STATIC";
    if (trigger == MODULES_GRASP_TRIGGER_MOVING) return "MOVING";
    return "NONE";
}

/**
 * @brief 关闭视觉会话并清除当前候选，保证动作12开始后不再产生第二次触发。
 * @note 已在途的UART7轮询允许自然完成，但抓取授权已在调用前清除。
 */
static void modules_disable_vision_for_grasp(modules_test_context_t *ctx)
{
    modules_vision_session_t *session = &ctx->vision_session;

    session->enabled = false;
    session->stop_pending = ctx->vision_transport.active;
    session->early_ready = false;
    session->strict_ready = false;
}

/** @brief 返回高速多球阶段的固定诊断文本。 */
static const char *modules_multi_phase_name(modules_multi_phase_t phase)
{
    if (phase == MODULES_MULTI_WAIT_VISION) return "WAIT_VISION";
    if (phase == MODULES_MULTI_WAIT_GRASP12) return "WAIT_GRASP12";
    if (phase == MODULES_MULTI_WAIT_RETURN11) return "WAIT_RETURN11";
    if (phase == MODULES_MULTI_WAIT_IC_SUBMIT) return "WAIT_IC_SUBMIT";
    if (phase == MODULES_MULTI_WAIT_IC) return "WAIT_IC";
    if (phase == MODULES_MULTI_WAIT_SLOT) return "WAIT_SLOT";
    if (phase == MODULES_MULTI_COMPLETE) return "COMPLETE";
    if (phase == MODULES_MULTI_STOPPED) return "STOPPED";
    if (phase == MODULES_MULTI_FAULT) return "FAULT";
    return "IDLE";
}

/** @brief 锁存多球故障并撤销后续视觉触发；不会伪装成已完成。 */
static void modules_multi_fail(
    modules_test_context_t *ctx, const char *reason)
{
    if (ctx == NULL) return;
    modules_disable_vision_for_grasp(ctx);
    ctx->grasp.trigger = MODULES_GRASP_TRIGGER_NONE;
    ctx->multi.active = false;
    ctx->multi.stop_requested = false;
    ctx->multi.phase = MODULES_MULTI_FAULT;
    ctx->multi.last_fault = (reason != NULL) ? reason : "UNKNOWN";
    ctx->multi.fault_count++;
    (void)snprintf(ctx->text, sizeof(ctx->text),
        "MULTI FAULT=%s COUNT=%lu. RECORDS PRESERVED; RESET REQUIRED\r\n",
        ctx->multi.last_fault,
        (unsigned long)ctx->multi.fault_count);
    (void)debug_uart1_write_text(&ctx->debug, ctx->text);
}

/** @brief 在安全边界结束多球流程，档案始终保留到显式BALL_RESET。 */
static void modules_multi_finish(
    modules_test_context_t *ctx, modules_multi_phase_t phase)
{
    modules_disable_vision_for_grasp(ctx);
    ctx->grasp.trigger = MODULES_GRASP_TRIGGER_NONE;
    ctx->multi.active = false;
    ctx->multi.stop_requested = false;
    ctx->multi.phase = phase;
    ctx->multi.last_fault = NULL;
    (void)snprintf(ctx->text, sizeof(ctx->text),
        "MULTI %s REGION=%u COUNT=%u/%u TOTAL=%u CYCLES=%lu\r\n",
        modules_multi_phase_name(phase),
        (unsigned)ctx->multi.region,
        (unsigned)ball_manifest_region_count(
            &ctx->manifest, ctx->multi.region),
        (unsigned)ball_manifest_region_expected(ctx->multi.region),
        (unsigned)ctx->manifest.count,
        (unsigned long)ctx->multi.completed_cycles);
    (void)debug_uart1_write_text(&ctx->debug, ctx->text);
}

/** @brief LSC16发送完成回调；只复制结果到测试任务队列。 */
static void modules_lsc16_done(
    void *user_ctx,
    uint32_t request_id,
    lsc16_status_t status)
{
    modules_test_context_t *ctx = (modules_test_context_t *)user_ctx;
    modules_event_t event;

    (void)request_id;
    if (ctx == NULL) return;
    (void)memset(&event, 0, sizeof(event));
    event.kind = MODULES_EVENT_LSC16_TX_DONE;
    event.lsc16_status = status;
    event.lsc16_action_group = ctx->grasp.active_group;
    event.lsc16_is_stop = ctx->grasp.pending_is_stop;
    (void)modules_put_event(&event);
}

/** @brief LSC16主动回报回调；复制完整回报后交给唯一测试任务推进状态。 */
static void modules_lsc16_report(
    void *user_ctx,
    uint32_t report_events,
    const lsc16_report_t *report)
{
    modules_test_context_t *ctx = (modules_test_context_t *)user_ctx;
    modules_event_t event;

    if ((ctx == NULL) || (report == NULL)) return;
    (void)memset(&event, 0, sizeof(event));
    event.kind = MODULES_EVENT_LSC16_REPORT;
    event.lsc16_report_events = report_events;
    event.lsc16_report = *report;
    (void)modules_put_event(&event);
}

/** @brief 提交一个重复1次的受保护动作组。 */
static lsc16_status_t modules_submit_arm_action(
    modules_test_context_t *ctx,
    uint8_t action_group)
{
    lsc16_status_t status;

    if (ctx->grasp.tx_pending) return LSC16_ERR_BUSY;
    status = lsc16_test_action_guard_begin(
        &ctx->grasp.guard,
        modules_now_ms(),
        MULT_UART_MODULES_LSC16_TX_TIMEOUT_MS);
    if (status != LSC16_OK) return status;

    ctx->grasp.active_group = action_group;
    ctx->grasp.pending_is_stop = false;
    status = lsc16_device_run_action_group(
        action_group,
        MULT_UART_MODULES_LSC16_REPEAT_COUNT,
        modules_lsc16_done,
        ctx);
    if (status != LSC16_OK) {
        lsc16_test_action_guard_cancel_begin(&ctx->grasp.guard);
        return status;
    }
    ctx->grasp.tx_pending = true;
    ctx->grasp.action_submit_count++;
    (void)snprintf(ctx->text, sizeof(ctx->text),
        "ARM ACTION REQUEST GROUP=%u REPEAT=1\r\n",
        (unsigned)action_group);
    (void)debug_uart1_write_text(&ctx->debug, ctx->text);
    return LSC16_OK;
}

/** @brief 请求停止机械臂动作；发送事务尚未完成时延后到完成事件处理。 */
static lsc16_status_t modules_request_arm_stop(
    modules_test_context_t *ctx,
    bool automatic)
{
    lsc16_status_t status;

    ctx->grasp.trigger = MODULES_GRASP_TRIGGER_NONE;
    ctx->grasp.returning_to_vision = false;
    ctx->grasp.auto_return_pending = false;
    ctx->grasp.pose = MODULES_ARM_POSE_UNKNOWN;
    modules_disable_vision_for_grasp(ctx);
    if (ctx->grasp.tx_pending) {
        ctx->grasp.stop_deferred = true;
        (void)debug_uart1_write_text(&ctx->debug,
            automatic ?
                "ARM AUTO STOP DEFERRED: WAIT TX DONE; CUT POWER IF MOVING\r\n" :
                "ARM STOP DEFERRED: WAIT TX DONE; CUT POWER IF MOVING\r\n");
        return LSC16_OK;
    }

    ctx->grasp.pending_is_stop = true;
    status = lsc16_device_stop_action_group(modules_lsc16_done, ctx);
    if (status == LSC16_OK) {
        ctx->grasp.tx_pending = true;
        (void)debug_uart1_write_text(&ctx->debug,
            automatic ? "ARM AUTO STOP REQUESTED\r\n" :
                        "ARM STOP REQUESTED\r\n");
    }
    return status;
}

/** @brief 在动作12自然完成后提交动作11；失败即锁存故障并停止。 */
static void modules_start_auto_return_to_vision(modules_test_context_t *ctx)
{
    lsc16_status_t status;

    ctx->grasp.auto_return_pending = false;
    ctx->grasp.returning_to_vision = true;
    status = modules_submit_arm_action(
        ctx, MULT_UART_MODULES_LSC16_VISION_GROUP);
    if (status != LSC16_OK) {
        lsc16_test_action_guard_latch_fault(&ctx->grasp.guard);
        ctx->grasp.pose = MODULES_ARM_POSE_UNKNOWN;
        if (ctx->multi.active) {
            modules_multi_fail(ctx, "RETURN11_SUBMIT");
        }
        (void)snprintf(ctx->text, sizeof(ctx->text),
            "ARM AUTO RETURN GROUP11 SUBMIT ERROR=%u. RESET BEFORE MOTION\r\n",
            (unsigned)status);
        (void)debug_uart1_write_text(&ctx->debug, ctx->text);
        (void)modules_request_arm_stop(ctx, true);
        return;
    }
    (void)debug_uart1_write_text(&ctx->debug,
        "ARM GROUP12 DONE. AUTO RETURN GROUP11 STARTED\r\n");
}

/** @brief 处理LSC16发送完成事件，发送成功后仍继续等待真实动作回报。 */
static void modules_handle_lsc16_tx_done(
    modules_test_context_t *ctx,
    const modules_event_t *event)
{
    bool submit_deferred_stop;

    ctx->grasp.tx_pending = false;
    submit_deferred_stop = ctx->grasp.stop_deferred;
    ctx->grasp.stop_deferred = false;
    if (event->lsc16_status != LSC16_OK) {
        if (!event->lsc16_is_stop) {
            lsc16_test_action_guard_latch_fault(&ctx->grasp.guard);
        }
        ctx->grasp.pose = MODULES_ARM_POSE_UNKNOWN;
        if (ctx->multi.active) {
            modules_multi_fail(ctx, "ARM_TX");
        }
        (void)snprintf(ctx->text, sizeof(ctx->text),
            "ARM %s TX ERROR=%u GROUP=%u\r\n",
            event->lsc16_is_stop ? "STOP" : "ACTION",
            (unsigned)event->lsc16_status,
            (unsigned)event->lsc16_action_group);
        (void)debug_uart1_write_text(&ctx->debug, ctx->text);
        if (!event->lsc16_is_stop) {
            (void)modules_request_arm_stop(ctx, true);
        }
        return;
    }

    if (event->lsc16_is_stop) {
        (void)debug_uart1_write_text(&ctx->debug,
            "ARM STOP TX DONE. WAIT ACTION STOPPED OR OBSERVE MOTION\r\n");
    } else {
        lsc16_test_action_guard_on_tx_done(
            &ctx->grasp.guard,
            modules_now_ms(),
            MULT_UART_MODULES_LSC16_STARTED_TIMEOUT_MS);
        (void)snprintf(ctx->text, sizeof(ctx->text),
            "ARM ACTION TX DONE GROUP=%u. WAIT STARTED/COMPLETED\r\n",
            (unsigned)event->lsc16_action_group);
        (void)debug_uart1_write_text(&ctx->debug, ctx->text);
    }
    if (ctx->grasp.auto_return_pending && !ctx->grasp.tx_pending) {
        modules_start_auto_return_to_vision(ctx);
    }
    if (submit_deferred_stop) {
        (void)modules_request_arm_stop(ctx, true);
    }
}

/** @brief 处理与当前动作组匹配的开始/完成/停止主动回报。 */
static void modules_handle_lsc16_report(
    modules_test_context_t *ctx,
    const modules_event_t *event)
{
    uint32_t events = event->lsc16_report_events;
    const lsc16_report_t *report = &event->lsc16_report;
    bool action_event = (events & (LSC16_REPORT_EVENT_ACTION_STARTED |
        LSC16_REPORT_EVENT_ACTION_COMPLETED)) != 0U;
    bool active = lsc16_test_action_guard_is_active(&ctx->grasp.guard);

    if ((events & LSC16_REPORT_EVENT_INVALID_FRAME) != 0U) {
        (void)debug_uart1_write_text(&ctx->debug,
            "ARM INVALID REPORT IGNORED\r\n");
    }
    if (action_event && (!active ||
        (report->action_group != ctx->grasp.active_group))) {
        ctx->grasp.mismatched_report_count++;
        (void)snprintf(ctx->text, sizeof(ctx->text),
            "ARM REPORT IGNORED GROUP=%u EXPECTED=%u ACTIVE=%u\r\n",
            (unsigned)report->action_group,
            (unsigned)ctx->grasp.active_group,
            active ? 1U : 0U);
        (void)debug_uart1_write_text(&ctx->debug, ctx->text);
        events &= ~(LSC16_REPORT_EVENT_ACTION_STARTED |
                    LSC16_REPORT_EVENT_ACTION_COMPLETED);
    }

    if ((events & LSC16_REPORT_EVENT_ACTION_STARTED) != 0U) {
        ctx->grasp.action_started_count++;
        lsc16_test_action_guard_on_started(
            &ctx->grasp.guard,
            modules_now_ms(),
            MULT_UART_MODULES_LSC16_COMPLETE_TIMEOUT_MS);
        (void)snprintf(ctx->text, sizeof(ctx->text),
            "ARM ACTION STARTED GROUP=%u REPEAT=%u\r\n",
            (unsigned)report->action_group,
            (unsigned)report->repeat_count);
        (void)debug_uart1_write_text(&ctx->debug, ctx->text);
    }
    if ((events & LSC16_REPORT_EVENT_ACTION_STOPPED) != 0U) {
        lsc16_test_action_guard_on_stopped(&ctx->grasp.guard);
        ctx->grasp.pose = MODULES_ARM_POSE_UNKNOWN;
        ctx->grasp.returning_to_vision = false;
        (void)debug_uart1_write_text(&ctx->debug, "ARM ACTION STOPPED\r\n");
    }
    if ((events & LSC16_REPORT_EVENT_ACTION_COMPLETED) != 0U) {
        uint8_t completed_group = report->action_group;
        bool was_returning = ctx->grasp.returning_to_vision;

        ctx->grasp.action_completed_count++;
        ctx->grasp.last_completed_group = completed_group;
        lsc16_test_action_guard_on_completed(&ctx->grasp.guard);
        (void)snprintf(ctx->text, sizeof(ctx->text),
            "ARM ACTION COMPLETED GROUP=%u REPEAT=%u\r\n",
            (unsigned)completed_group,
            (unsigned)report->repeat_count);
        (void)debug_uart1_write_text(&ctx->debug, ctx->text);

        if (completed_group == MULT_UART_MODULES_LSC16_HOME_GROUP) {
            ctx->grasp.pose = MODULES_ARM_POSE_HOME_10;
        } else if (completed_group == MULT_UART_MODULES_LSC16_VISION_GROUP) {
            ctx->grasp.pose = MODULES_ARM_POSE_VISION_11;
            ctx->grasp.returning_to_vision = false;
            if (was_returning && !ctx->multi.active) {
                (void)debug_uart1_write_text(&ctx->debug,
                    "SINGLE BALL CYCLE COMPLETE. VISION11 READY. GRASP DISARMED\r\n");
            }
        } else if (completed_group ==
                   MULT_UART_MODULES_LSC16_GRASP_PLACE_GROUP) {
            ctx->grasp.pose = MODULES_ARM_POSE_GRASP_PLACED_12;
#if MULT_UART_MODULES_LSC16_AUTO_RETURN_VISION
            ctx->grasp.returning_to_vision = true;
            if (ctx->grasp.tx_pending) {
                ctx->grasp.auto_return_pending = true;
                (void)debug_uart1_write_text(&ctx->debug,
                    "ARM GROUP12 DONE. AUTO RETURN GROUP11 WAIT TX EVENT\r\n");
            } else {
                modules_start_auto_return_to_vision(ctx);
            }
#endif
        }
        modules_multi_on_action_completed(ctx, completed_group, was_returning);
    }
}

/** @brief 静止严格对齐或运动候选命中后，只触发一次动作组12。 */
static void modules_maybe_trigger_grasp(
    modules_test_context_t *ctx,
    const nano_vision_observation_t *observation,
    bool static_ready,
    bool moving_ready)
{
    modules_grasp_trigger_t trigger = ctx->grasp.trigger;
    bool ready = (trigger == MODULES_GRASP_TRIGGER_STATIC) && static_ready;
    lsc16_status_t status;

    if (trigger == MODULES_GRASP_TRIGGER_MOVING) {
        /*
         * 轮询响应已由当前SEQ约束；高速抓取直接使用本次返回的候选，避免
         * 额外等待一帧。候选仍必须通过质量、年龄和实机抓取窗口过滤。
         */
        ready = moving_ready;
    }
    if (!ready) return;
    if (ctx->multi.active) {
        if (ctx->multi.phase != MODULES_MULTI_WAIT_VISION) {
            modules_multi_fail(ctx, "VISION_PHASE");
            return;
        }
        ctx->multi.phase = MODULES_MULTI_WAIT_GRASP12;
    }
    ctx->grasp.trigger = MODULES_GRASP_TRIGGER_NONE;
    ctx->grasp.trigger_count++;
    modules_disable_vision_for_grasp(ctx);
    (void)snprintf(ctx->text, sizeof(ctx->text),
        "GRASP TRIGGER MODE=%s FRAME=%u STATUS=%u DX=%d DY=%d "
        "QUALITY=%u AGE=%u ONE_SHOT\r\n",
        modules_grasp_trigger_name(trigger),
        (unsigned)observation->frame_id,
        (unsigned)observation->status,
        (int)observation->offset_x_px,
        (int)observation->offset_y_px,
        (unsigned)observation->quality,
        (unsigned)observation->age_ms);
    (void)debug_uart1_write_text(&ctx->debug, ctx->text);
    status = modules_submit_arm_action(
        ctx, MULT_UART_MODULES_LSC16_GRASP_PLACE_GROUP);
    if (status != LSC16_OK) {
        ctx->grasp.rejected_count++;
        if (ctx->multi.active) {
            modules_multi_fail(ctx, "GRASP12_SUBMIT");
        }
        (void)snprintf(ctx->text, sizeof(ctx->text),
            "GRASP GROUP12 SUBMIT ERROR=%u. REARM REQUIRED\r\n",
            (unsigned)status);
        (void)debug_uart1_write_text(&ctx->debug, ctx->text);
    }
}

/** @brief 轮询动作发送、开始和完成超时；超时后锁存故障并请求停止。 */
static void modules_check_arm_timeout(modules_test_context_t *ctx)
{
    lsc16_test_action_timeout_t timeout = lsc16_test_action_guard_poll(
        &ctx->grasp.guard, modules_now_ms());
    const char *name;

    if (timeout == LSC16_TEST_ACTION_TIMEOUT_NONE) return;
    name = (timeout == LSC16_TEST_ACTION_TIMEOUT_TX) ? "TX" :
        ((timeout == LSC16_TEST_ACTION_TIMEOUT_STARTED) ? "START" : "COMPLETE");
    (void)snprintf(ctx->text, sizeof(ctx->text),
        "ARM ACTION %s TIMEOUT. FAULT LATCHED; CUT POWER IF MOVING\r\n",
        name);
    (void)debug_uart1_write_text(&ctx->debug, ctx->text);
    (void)modules_request_arm_stop(ctx, true);
    if (ctx->multi.active) {
        modules_multi_fail(ctx, "ARM_TIMEOUT");
    }
}

/** @brief 开始固定场景、固定目标颜色的连续视觉轮询，不包含任何机械动作。 */
static bool modules_start_vision_session(
    modules_test_context_t *ctx,
    nano_vision_scene_t scene,
    nano_vision_color_t target_color)
{
    modules_vision_session_t *session = &ctx->vision_session;

    if (ctx->command_pending || ctx->slot.active ||
        ctx->vision_transport.active || session->enabled ||
        session->stop_pending) {
        (void)debug_uart1_write_text(&ctx->debug,
            "VISION SESSION START REJECTED: BUSY\r\n");
        return false;
    }

    (void)memset(session, 0, sizeof(*session));
    /* 新会话一律从未授权开始，禁止沿用上一次STATIC/MOVING一次性触发。 */
    ctx->grasp.trigger = MODULES_GRASP_TRIGGER_NONE;
    session->enabled = true;
    session->scene = scene;
    session->target_color = target_color;
    session->early_compare_enabled = (scene == NANO_VISION_SCENE_TURNTABLE);
    session->early_config.scene = scene;
    session->early_config.target_color = target_color;
    session->early_config.min_quality = MULT_UART_MODULES_VISION_EARLY_MIN_QUALITY;
    session->early_config.max_observation_age_ms =
        MULT_UART_MODULES_VISION_EARLY_MAX_AGE_MS;
    session->early_config.min_offset_x_px =
        MULT_UART_MODULES_VISION_EARLY_MIN_DX_PX;
    session->early_config.max_offset_x_px =
        MULT_UART_MODULES_VISION_EARLY_MAX_DX_PX;
    session->early_config.min_offset_y_px =
        MULT_UART_MODULES_VISION_EARLY_MIN_DY_PX;
    session->early_config.max_offset_y_px =
        MULT_UART_MODULES_VISION_EARLY_MAX_DY_PX;
    session->next_poll_tick = osKernelGetTickCount();

    /* 每次会话重新建立连续帧，禁止沿用上一次抓取前的ALIGNED状态。 */
    ctx->vision_transport.tracker_initialized = false;
    (void)snprintf(ctx->text, sizeof(ctx->text),
        "VISION SESSION START SCENE=%u COLOR=%u RATE=%uHZ EARLY_COMPARE=%u GRASP_DISARMED\r\n",
        (unsigned)scene,
        (unsigned)target_color,
        (unsigned)(1000U / MULT_UART_MODULES_VISION_POLL_PERIOD_MS),
        session->early_compare_enabled ? 1U : 0U);
    (void)debug_uart1_write_text(&ctx->debug, ctx->text);
    return true;
}

/** @brief 停止连续轮询；若已有事务在途，等其安全完成后再宣布停止。 */
static void modules_stop_vision_session(modules_test_context_t *ctx)
{
    modules_vision_session_t *session = &ctx->vision_session;

    /* STOP同时撤销抓取授权，重新START后必须再次显式ARM。 */
    ctx->grasp.trigger = MODULES_GRASP_TRIGGER_NONE;
    if (!session->enabled && !session->stop_pending) {
        (void)debug_uart1_write_text(&ctx->debug,
            "VISION SESSION NOT ACTIVE\r\n");
        return;
    }

    session->enabled = false;
    if (ctx->vision_transport.active) {
        session->stop_pending = true;
        (void)debug_uart1_write_text(&ctx->debug,
            "VISION SESSION STOP REQUESTED. WAIT INFLIGHT POLL\r\n");
        return;
    }

    session->stop_pending = false;
    (void)snprintf(ctx->text, sizeof(ctx->text),
        "VISION SESSION STOPPED POLL=%lu DONE=%lu ERR=%lu EARLY_ENTER=%lu STRICT_ENTER=%lu\r\n",
        (unsigned long)session->poll_count,
        (unsigned long)session->completed_count,
        (unsigned long)session->error_count,
        (unsigned long)session->early_enter_count,
        (unsigned long)session->strict_enter_count);
    (void)debug_uart1_write_text(&ctx->debug, ctx->text);
}

/** @brief 为下一球重新建立fast候选会话和一次性授权。 */
static bool modules_multi_arm_next_vision(modules_test_context_t *ctx)
{
    if (ctx->grasp.pose != MODULES_ARM_POSE_VISION_11) {
        modules_multi_fail(ctx, "ARM_NOT_VISION11");
        return false;
    }
    if (!modules_start_vision_session(
            ctx, NANO_VISION_SCENE_TURNTABLE, ctx->multi.target_color)) {
        modules_multi_fail(ctx, "VISION_START");
        return false;
    }
    ctx->grasp.trigger = MODULES_GRASP_TRIGGER_MOVING;
    ctx->multi.phase = MODULES_MULTI_WAIT_VISION;
    (void)snprintf(ctx->text, sizeof(ctx->text),
        "MULTI WAIT BALL %u/%u. WAIT FAST WINDOW CANDIDATE\r\n",
        (unsigned)(ball_manifest_region_count(
            &ctx->manifest, ctx->multi.region) + 1U),
        (unsigned)ball_manifest_region_expected(ctx->multi.region));
    (void)debug_uart1_write_text(&ctx->debug, ctx->text);
    return true;
}

/** @brief 启动圆盘机本方5球高速连续抓取；保留既有未完成档案以支持安全续跑。 */
static bool modules_multi_start(
    modules_test_context_t *ctx, nano_vision_color_t target_color)
{
#if !MULT_UART_MODULES_LSC16_MOTION_ARMED
    (void)target_color;
    (void)debug_uart1_write_text(&ctx->debug,
        "MULTI LOCKED: SET MULT_UART_MODULES_LSC16_MOTION_ARMED=1\r\n");
    return false;
#elif !MULT_UART_MODULES_ZDT_MOTION_ARMED
    (void)target_color;
    (void)debug_uart1_write_text(&ctx->debug,
        "MULTI LOCKED: SET MULT_UART_MODULES_ZDT_MOTION_ARMED=1 AFTER SAFETY CHECK\r\n");
    return false;
#else
    uint8_t i;
    ball_manifest_record_t record;

    if (ctx->multi.active || ctx->slot.active || ctx->command_pending ||
        ctx->vision_session.enabled || ctx->vision_session.stop_pending ||
        ctx->vision_transport.active || ctx->grasp.tx_pending ||
        lsc16_test_action_guard_is_active(&ctx->grasp.guard)) {
        (void)debug_uart1_write_text(&ctx->debug,
            "MULTI START REJECTED: BUSY\r\n");
        return false;
    }
    if (ctx->multi.phase == MODULES_MULTI_FAULT) {
        (void)debug_uart1_write_text(&ctx->debug,
            "MULTI START REJECTED: FAULT. SEND BALL_RESET WHEN SAFE\r\n");
        return false;
    }
    if (ctx->grasp.pose != MODULES_ARM_POSE_VISION_11) {
        (void)debug_uart1_write_text(&ctx->debug,
            "MULTI START REJECTED: COMPLETE ARM_HOME THEN ARM_VISION\r\n");
        return false;
    }
    if ((target_color != NANO_VISION_COLOR_RED) &&
        (target_color != NANO_VISION_COLOR_BLUE)) {
        return false;
    }
    if (ball_manifest_validate(&ctx->manifest) != BALL_MANIFEST_OK) {
        modules_multi_fail(ctx, "MANIFEST_CORRUPT");
        return false;
    }
    if (ball_manifest_region_is_complete(
            &ctx->manifest, BALL_MANIFEST_REGION_TURNTABLE)) {
        (void)debug_uart1_write_text(&ctx->debug,
            "MULTI START REJECTED: TURNTABLE ALREADY COMPLETE 5/5\r\n");
        return false;
    }
    for (i = 0U; i < ctx->manifest.count; ++i) {
        if ((ball_manifest_get(&ctx->manifest, i, &record) != BALL_MANIFEST_OK) ||
            ((record.color == BALL_MANIFEST_COLOR_RED) !=
             (target_color == NANO_VISION_COLOR_RED))) {
            (void)debug_uart1_write_text(&ctx->debug,
                "MULTI START REJECTED: EXISTING RECORD COLOR MISMATCH; BALL_RESET\r\n");
            return false;
        }
    }
    if (!ctx->zdt.firmware_known) {
        (void)debug_uart1_write_text(&ctx->debug,
            "MULTI START REJECTED: SEND ZDT_OPTIONS FIRST\r\n");
        return false;
    }
    if (!modules_gate_is_stably_high()) {
        (void)debug_uart1_write_text(&ctx->debug,
            "MULTI START REJECTED: PB0 NOT STABLY HIGH\r\n");
        return false;
    }

    ctx->multi.active = true;
    ctx->multi.stop_requested = false;
    ctx->multi.target_color = target_color;
    ctx->multi.region = BALL_MANIFEST_REGION_TURNTABLE;
    ctx->multi.ic_attempts = 0U;
    ctx->multi.completed_cycles = ball_manifest_region_count(
        &ctx->manifest, ctx->multi.region);
    ctx->multi.last_fault = NULL;
    if (!modules_multi_arm_next_vision(ctx)) return false;
    (void)snprintf(ctx->text, sizeof(ctx->text),
        "MULTI START SCENE=TURNTABLE COLOR=%u COUNT=%u/5 SLOT=%u\r\n",
        (unsigned)target_color,
        (unsigned)ctx->multi.completed_cycles,
        (unsigned)ctx->manifest.count);
    (void)debug_uart1_write_text(&ctx->debug, ctx->text);
    return true;
#endif
}

/** @brief 在动作完成回报后推进多球流程；动作11完成才允许进入IC阶段。 */
static void modules_multi_on_action_completed(
    modules_test_context_t *ctx, uint8_t completed_group, bool was_returning)
{
    if (!ctx->multi.active) return;
    if (completed_group == MULT_UART_MODULES_LSC16_GRASP_PLACE_GROUP) {
        if (ctx->multi.phase != MODULES_MULTI_WAIT_GRASP12) {
            modules_multi_fail(ctx, "GRASP12_PHASE");
            return;
        }
        ctx->multi.phase = MODULES_MULTI_WAIT_RETURN11;
        return;
    }
    if ((completed_group != MULT_UART_MODULES_LSC16_VISION_GROUP) ||
        !was_returning) {
        return;
    }
    if (ctx->multi.phase != MODULES_MULTI_WAIT_RETURN11) {
        modules_multi_fail(ctx, "RETURN11_PHASE");
        return;
    }
    ctx->multi.ic_attempts = 0U;
    ctx->multi.next_action_tick = osKernelGetTickCount();
    ctx->multi.phase = MODULES_MULTI_WAIT_IC_SUBMIT;
    (void)debug_uart1_write_text(&ctx->debug,
        "MULTI ARM VISION11 CONFIRMED. START IC READ\r\n");
}

/** @brief 请求在最近安全边界停止，不用软件STOP冒充硬件急停。 */
static void modules_multi_request_stop(modules_test_context_t *ctx)
{
    if (!ctx->multi.active) {
        (void)debug_uart1_write_text(&ctx->debug,
            "MULTI NOT ACTIVE\r\n");
        return;
    }
    ctx->multi.stop_requested = true;
    if (ctx->multi.phase == MODULES_MULTI_WAIT_VISION) {
        modules_multi_finish(ctx, MODULES_MULTI_STOPPED);
        return;
    }
    (void)snprintf(ctx->text, sizeof(ctx->text),
        "MULTI STOP REQUESTED PHASE=%s. WAIT SAFE BOUNDARY; USE HARD E-STOP FOR DANGER\r\n",
        modules_multi_phase_name(ctx->multi.phase));
    (void)debug_uart1_write_text(&ctx->debug, ctx->text);
}

/** @brief 在动作11完成后异步提交IC读取，并对提交失败做有限重试。 */
static void modules_multi_process(modules_test_context_t *ctx)
{
    ic_card_status_t status;
    uint32_t now;

    if (!ctx->multi.active ||
        (ctx->multi.phase != MODULES_MULTI_WAIT_IC_SUBMIT)) {
        return;
    }
    if (ctx->vision_transport.active || ctx->vision_session.stop_pending ||
        ctx->ic_transport.active || ctx->slot.active) {
        return;
    }
    now = osKernelGetTickCount();
    if ((int32_t)(now - ctx->multi.next_action_tick) < 0) return;

    ctx->multi.ic_attempts++;
    status = ic_card_device_read_competition_ball(
        (MULT_UART_MODULES_IC_OPERATION_PROMPT != 0U),
        modules_ic_read_done,
        NULL);
    if (status == IC_CARD_OK) {
        ctx->multi.phase = MODULES_MULTI_WAIT_IC;
        (void)snprintf(ctx->text, sizeof(ctx->text),
            "MULTI IC REQUEST ATTEMPT=%u/%u\r\n",
            (unsigned)ctx->multi.ic_attempts,
            (unsigned)MULT_UART_MODULES_MULTI_IC_MAX_ATTEMPTS);
        (void)debug_uart1_write_text(&ctx->debug, ctx->text);
        return;
    }
    if (ctx->multi.ic_attempts >= MULT_UART_MODULES_MULTI_IC_MAX_ATTEMPTS) {
        modules_multi_fail(ctx, "IC_SUBMIT");
        return;
    }
    ctx->multi.next_action_tick = now + modules_ms_to_ticks(
        MULT_UART_MODULES_MULTI_IC_RETRY_MS);
}

/** @brief 到周期且总线空闲时提交下一次通道0轮询；延误时不追赶连发。 */
static void modules_process_vision_session(modules_test_context_t *ctx)
{
    modules_vision_session_t *session = &ctx->vision_session;
    nano_vision_status_t status;
    uint32_t now;

    if (!session->enabled || session->stop_pending ||
        ctx->command_pending || ctx->slot.active ||
        ctx->vision_transport.active) return;

    now = osKernelGetTickCount();
    if ((int32_t)(now - session->next_poll_tick) < 0) return;
    session->next_poll_tick = now + modules_ms_to_ticks(
        MULT_UART_MODULES_VISION_POLL_PERIOD_MS);
    status = modules_submit_vision_poll(session->scene, session->target_color);
    if (status == NANO_VISION_OK) {
        session->poll_count++;
        return;
    }

    session->submit_error_count++;
    (void)snprintf(ctx->text, sizeof(ctx->text),
        "VISION SESSION SUBMIT ERROR=%u COUNT=%lu\r\n",
        (unsigned)status,
        (unsigned long)session->submit_error_count);
    (void)debug_uart1_write_text(&ctx->debug, ctx->text);
}

/**
 * @brief 连续采样PB0，只有所有样本均为高才认为螺丝可靠对准。
 * @note 本函数只在普通测试任务中调用；短暂osDelay不会发生在ISR或worker回调中。
 */
static bool modules_gate_is_stably_high(void)
{
    uint8_t sample;

    for (sample = 0U; sample < MULT_UART_MODULES_GATE_CONFIRM_SAMPLES; ++sample) {
        if (!gate_read()) return false;
        if ((sample + 1U) < MULT_UART_MODULES_GATE_CONFIRM_SAMPLES) {
            (void)osDelay(modules_ms_to_ticks(
                MULT_UART_MODULES_GATE_CONFIRM_INTERVAL_MS));
        }
    }
    return true;
}

/** @brief 结束自动逐槽流程，释放外部命令门并输出唯一最终结果。 */
static void modules_slot_finish(
    modules_test_context_t *ctx, bool success, const char *reason)
{
    uint8_t fine_steps = ctx->slot.fine_steps;
    const char *direction =
        (ctx->slot.direction == ZDT_TURNTABLE_DIR_CW) ? "CW" : "CCW";

    ctx->slot.active = false;
    ctx->slot.state = MODULES_SLOT_IDLE;
    ctx->command_pending = false;
    if (success) {
        (void)snprintf(ctx->text, sizeof(ctx->text),
            "SLOT OK DIR=%s FINE=%u PB0=1\r\n",
            direction, (unsigned)fine_steps);
    } else {
        (void)snprintf(ctx->text, sizeof(ctx->text),
            "SLOT ERROR DIR=%s %s FINE=%u PB0=%u\r\n",
            direction,
            (reason != NULL) ? reason : "UNKNOWN",
            (unsigned)fine_steps,
            gate_read() ? 1U : 0U);
    }
    (void)debug_uart1_write_text(&ctx->debug, ctx->text);
    modules_multi_on_slot_finished(ctx, success, reason);
}

/** @brief 在ACK之后安排下一次到位状态查询，避免5ms任务循环刷爆UART7。 */
static void modules_slot_schedule_status_poll(
    modules_test_context_t *ctx, bool after_fine)
{
    ctx->slot.state = after_fine ?
        MODULES_SLOT_WAIT_FINE_POLL : MODULES_SLOT_WAIT_COARSE_POLL;
    ctx->slot.next_poll_tick = osKernelGetTickCount() +
        modules_ms_to_ticks(MULT_UART_MODULES_ZDT_SLOT_STATUS_POLL_MS);
}

/** @brief 提交一次CW微调，并在提交成功后增加已用次数。 */
static bool modules_slot_submit_fine(modules_test_context_t *ctx)
{
    zdt_turntable_status_t status;

    status = modules_submit_motion(
        ctx->slot.direction,
        MULT_UART_MODULES_ZDT_FINE_ANGLE_0P1DEG,
        MULT_UART_MODULES_ZDT_FINE_SPEED_RPM);
    if (status != ZDT_TURNTABLE_OK) {
        modules_slot_finish(ctx, false, "FINE_SUBMIT");
        return false;
    }
    ctx->slot.fine_steps++;
    ctx->slot.state = MODULES_SLOT_WAIT_FINE_ACK;
    return true;
}

/**
 * @brief 消费自动逐槽流程的一次ZDT完成事件并推进状态机。
 * @warning 0xFD的ACK只表示命令被接收；必须继续查询0x3A并确认REACHED。
 */
static void modules_slot_handle_zdt_event(
    modules_test_context_t *ctx, const modules_event_t *event)
{
    bool is_ack_state;
    bool is_status_state;
    bool after_fine;

    if ((event->zdt_status != ZDT_TURNTABLE_OK) || !event->has_response) {
        modules_slot_finish(ctx, false, "ZDT_RESPONSE");
        return;
    }
    is_ack_state = (ctx->slot.state == MODULES_SLOT_WAIT_COARSE_ACK) ||
                   (ctx->slot.state == MODULES_SLOT_WAIT_FINE_ACK);
    if (is_ack_state) {
        if ((event->zdt_response.kind != ZDT_TURNTABLE_REPLY_ACK) &&
            (event->zdt_response.kind != ZDT_TURNTABLE_REPLY_REACHED)) {
            modules_slot_finish(ctx, false, "MOTION_ACK");
            return;
        }
        after_fine = (ctx->slot.state == MODULES_SLOT_WAIT_FINE_ACK);
        modules_slot_schedule_status_poll(ctx, after_fine);
        return;
    }

    is_status_state =
        (ctx->slot.state == MODULES_SLOT_WAIT_COARSE_STATUS) ||
        (ctx->slot.state == MODULES_SLOT_WAIT_FINE_STATUS);
    if (!is_status_state ||
        (event->zdt_response.kind != ZDT_TURNTABLE_REPLY_STATUS)) {
        modules_slot_finish(ctx, false, "STATE_MISMATCH");
        return;
    }
    after_fine = (ctx->slot.state == MODULES_SLOT_WAIT_FINE_STATUS);
    if (!event->zdt_response.data.motor_status.enabled ||
        event->zdt_response.data.motor_status.stalled ||
        event->zdt_response.data.motor_status.stall_protected ||
        event->zdt_response.data.motor_status.power_loss_latched) {
        modules_slot_finish(ctx, false, "MOTOR_FAULT");
        return;
    }
    if (!event->zdt_response.data.motor_status.reached) {
        modules_slot_schedule_status_poll(ctx, after_fine);
        return;
    }
    if (modules_gate_is_stably_high()) {
        modules_slot_finish(ctx, true, NULL);
        return;
    }
    if (ctx->slot.fine_steps >= MULT_UART_MODULES_ZDT_SLOT_FINE_MAX_STEPS) {
        modules_slot_finish(ctx, false, "GATE_NOT_FOUND");
        return;
    }
    (void)modules_slot_submit_fine(ctx);
}

/** @brief 启动一次从当前已对准槽位到相邻槽位的自动定位。 */
static zdt_turntable_status_t modules_slot_start(
    modules_test_context_t *ctx,
    zdt_turntable_direction_t direction)
{
    zdt_turntable_status_t status;

    ctx->slot.active = true;
    ctx->slot.state = MODULES_SLOT_WAIT_COARSE_ACK;
    ctx->slot.direction = direction;
    ctx->slot.fine_steps = 0U;
    ctx->slot.started_tick = osKernelGetTickCount();
    status = modules_submit_motion(
        direction,
        MULT_UART_MODULES_ZDT_COARSE_ANGLE_0P1DEG,
        MULT_UART_MODULES_ZDT_SPEED_RPM);
    if (status != ZDT_TURNTABLE_OK) {
        ctx->slot.active = false;
        ctx->slot.state = MODULES_SLOT_IDLE;
    }
    return status;
}

/**
 * @brief 在测试任务中执行定时状态查询和总超时保护。
 * @note 每个UART完成事件仍经队列返回本任务，因此自动流程始终只有一个执行者。
 */
static void modules_slot_process(modules_test_context_t *ctx)
{
    uint32_t now;
    uint32_t timeout_ticks;
    bool poll_due;
    zdt_turntable_status_t status;

    if (!ctx->slot.active) return;
    now = osKernelGetTickCount();
    timeout_ticks = modules_ms_to_ticks(MULT_UART_MODULES_ZDT_SLOT_TIMEOUT_MS);
    if ((now - ctx->slot.started_tick) >= timeout_ticks) {
        modules_slot_finish(ctx, false, "TIMEOUT");
        return;
    }
    poll_due = (ctx->slot.state == MODULES_SLOT_WAIT_COARSE_POLL) ||
               (ctx->slot.state == MODULES_SLOT_WAIT_FINE_POLL);
    if (!poll_due || ((int32_t)(now - ctx->slot.next_poll_tick) < 0)) return;

    if (ctx->slot.state == MODULES_SLOT_WAIT_COARSE_POLL) {
        ctx->slot.state = MODULES_SLOT_WAIT_COARSE_STATUS;
    } else {
        ctx->slot.state = MODULES_SLOT_WAIT_FINE_STATUS;
    }
    status = zdt_turntable_device_query(
        &ctx->zdt, 0x3AU, modules_zdt_done, NULL);
    if (status != ZDT_TURNTABLE_OK) {
        modules_slot_finish(ctx, false, "STATUS_SUBMIT");
    }
}

/** @brief 内部转盘到下一槽后重新授权下一球；失败则保留档案并锁存故障。 */
static void modules_multi_on_slot_finished(
    modules_test_context_t *ctx, bool success, const char *reason)
{
    if (!ctx->multi.active ||
        (ctx->multi.phase != MODULES_MULTI_WAIT_SLOT)) {
        return;
    }
    if (!success) {
        modules_multi_fail(ctx, (reason != NULL) ? reason : "SLOT");
        return;
    }
    if (ctx->multi.stop_requested) {
        modules_multi_finish(ctx, MODULES_MULTI_STOPPED);
        return;
    }
    (void)modules_multi_arm_next_vision(ctx);
}

/** @brief 消费连续抓取的一次IC结果，提交不可覆盖档案后才允许内部转盘动作。 */
static void modules_multi_handle_ic_event(
    modules_test_context_t *ctx, const modules_event_t *event)
{
    ball_manifest_color_t color;
    ball_manifest_status_t manifest_status;
    zdt_turntable_status_t zdt_status;
    zdt_turntable_direction_t direction;

    if (!ctx->multi.active ||
        (ctx->multi.phase != MODULES_MULTI_WAIT_IC)) {
        return;
    }
    if (event->ic_status != IC_CARD_OK) {
        if (ctx->multi.ic_attempts >=
            MULT_UART_MODULES_MULTI_IC_MAX_ATTEMPTS) {
            modules_multi_fail(ctx, "IC_READ");
            return;
        }
        ctx->multi.phase = MODULES_MULTI_WAIT_IC_SUBMIT;
        ctx->multi.next_action_tick = osKernelGetTickCount() +
            modules_ms_to_ticks(MULT_UART_MODULES_MULTI_IC_RETRY_MS);
        (void)snprintf(ctx->text, sizeof(ctx->text),
            "MULTI IC ERROR=%u RETRY=%u/%u\r\n",
            (unsigned)event->ic_status,
            (unsigned)(ctx->multi.ic_attempts + 1U),
            (unsigned)MULT_UART_MODULES_MULTI_IC_MAX_ATTEMPTS);
        (void)debug_uart1_write_text(&ctx->debug, ctx->text);
        return;
    }

    color = (ctx->multi.target_color == NANO_VISION_COLOR_RED) ?
        BALL_MANIFEST_COLOR_RED : BALL_MANIFEST_COLOR_BLUE;
    manifest_status = ball_manifest_append(
        &ctx->manifest,
        ctx->multi.region,
        color,
        event->ball.ball.code,
        event->ball.ball.row,
        event->ball.ball.column,
        ctx->manifest.count);
    if (manifest_status != BALL_MANIFEST_OK) {
        (void)snprintf(ctx->text, sizeof(ctx->text),
            "MULTI MANIFEST APPEND ERROR=%u\r\n",
            (unsigned)manifest_status);
        (void)debug_uart1_write_text(&ctx->debug, ctx->text);
        modules_multi_fail(ctx, "MANIFEST_APPEND");
        return;
    }
    ctx->multi.completed_cycles++;
    (void)snprintf(ctx->text, sizeof(ctx->text),
        "MULTI RECORD COMMIT SEQ=%u CODE=0x%02X ROW=%u COL=%u SLOT=%u REGION=%u/%u\r\n",
        (unsigned)(ctx->manifest.count - 1U),
        (unsigned)event->ball.ball.code,
        (unsigned)event->ball.ball.row,
        (unsigned)event->ball.ball.column,
        (unsigned)(ctx->manifest.count - 1U),
        (unsigned)ball_manifest_region_count(
            &ctx->manifest, ctx->multi.region),
        (unsigned)ball_manifest_region_expected(ctx->multi.region));
    (void)debug_uart1_write_text(&ctx->debug, ctx->text);

    if (ball_manifest_region_is_complete(
            &ctx->manifest, ctx->multi.region)) {
        modules_multi_finish(ctx, MODULES_MULTI_COMPLETE);
        return;
    }
    if (ctx->grasp.pose != MODULES_ARM_POSE_VISION_11) {
        modules_multi_fail(ctx, "SLOT_ARM_NOT_VISION11");
        return;
    }
    if (!modules_gate_is_stably_high()) {
        modules_multi_fail(ctx, "SLOT_START_GATE");
        return;
    }
#if MULT_UART_MODULES_ZDT_MOTION_ARMED
    direction = MULT_UART_MODULES_MULTI_SLOT_USE_CW ?
        ZDT_TURNTABLE_DIR_CW : ZDT_TURNTABLE_DIR_CCW;
    ctx->multi.phase = MODULES_MULTI_WAIT_SLOT;
    zdt_status = modules_slot_start(ctx, direction);
    if (zdt_status != ZDT_TURNTABLE_OK) {
        modules_multi_fail(ctx, "SLOT_SUBMIT");
        return;
    }
    (void)snprintf(ctx->text, sizeof(ctx->text),
        "MULTI SLOT START DIR=%s AFTER RECORD=%u\r\n",
        (direction == ZDT_TURNTABLE_DIR_CW) ? "CW" : "CCW",
        (unsigned)(ctx->manifest.count - 1U));
    (void)debug_uart1_write_text(&ctx->debug, ctx->text);
#else
    (void)zdt_status;
    (void)direction;
    modules_multi_fail(ctx, "ZDT_LOCKED");
#endif
}

/** @brief 输出多球流程和三个区域的只读计数。 */
static void modules_multi_print_status(modules_test_context_t *ctx)
{
    (void)snprintf(ctx->text, sizeof(ctx->text),
        "MULTI ACTIVE=%u PHASE=%s STOP=%u COLOR=%u IC_TRY=%u "
        "TURN=%u/5 STAIR=%u/2 PILLAR=%u/2 TOTAL=%u/9 VALID=%u FAULT=%s\r\n",
        ctx->multi.active ? 1U : 0U,
        modules_multi_phase_name(ctx->multi.phase),
        ctx->multi.stop_requested ? 1U : 0U,
        (unsigned)ctx->multi.target_color,
        (unsigned)ctx->multi.ic_attempts,
        (unsigned)ball_manifest_region_count(
            &ctx->manifest, BALL_MANIFEST_REGION_TURNTABLE),
        (unsigned)ball_manifest_region_count(
            &ctx->manifest, BALL_MANIFEST_REGION_STAIR),
        (unsigned)ball_manifest_region_count(
            &ctx->manifest, BALL_MANIFEST_REGION_PILLAR),
        (unsigned)ctx->manifest.count,
        (ball_manifest_validate(&ctx->manifest) == BALL_MANIFEST_OK) ? 1U : 0U,
        (ctx->multi.last_fault != NULL) ? ctx->multi.last_fault : "NONE");
    (void)debug_uart1_write_text(&ctx->debug, ctx->text);
}

/** @brief 逐条复制输出球档案，避免向调试命令暴露内部可写指针。 */
static void modules_manifest_print(modules_test_context_t *ctx)
{
    ball_manifest_record_t record;
    uint8_t i;

    modules_multi_print_status(ctx);
    for (i = 0U; i < ctx->manifest.count; ++i) {
        if (ball_manifest_get(&ctx->manifest, i, &record) != BALL_MANIFEST_OK) {
            (void)debug_uart1_write_text(&ctx->debug,
                "BALL MANIFEST CORRUPT\r\n");
            return;
        }
        (void)snprintf(ctx->text, sizeof(ctx->text),
            "BALL SEQ=%u REGION=%u COLOR=%u CODE=0x%02X ROW=%u COL=%u "
            "SLOT=%u STATE=%u CRC=%04X COMMIT=%u\r\n",
            (unsigned)record.sequence,
            (unsigned)record.region,
            (unsigned)record.color,
            (unsigned)record.ic_code,
            (unsigned)record.target_row,
            (unsigned)record.target_column,
            (unsigned)record.storage_slot,
            (unsigned)record.state,
            (unsigned)record.checksum,
            record.committed ? 1U : 0U);
        (void)debug_uart1_write_text(&ctx->debug, ctx->text);
    }
}

/** @brief 只有所有异步设备空闲时才允许显式清空整场档案和故障。 */
static bool modules_manifest_reset(modules_test_context_t *ctx)
{
    if (ctx->multi.active || ctx->slot.active || ctx->command_pending ||
        ctx->vision_session.enabled || ctx->vision_session.stop_pending ||
        ctx->vision_transport.active || ctx->grasp.tx_pending ||
        lsc16_test_action_guard_is_active(&ctx->grasp.guard) ||
        ctx->ic_transport.active || ctx->zdt_transport.active) {
        (void)debug_uart1_write_text(&ctx->debug,
            "BALL RESET REJECTED: SYSTEM BUSY\r\n");
        return false;
    }
    ball_manifest_init(&ctx->manifest);
    (void)memset(&ctx->multi, 0, sizeof(ctx->multi));
    ctx->multi.phase = MODULES_MULTI_IDLE;
    (void)debug_uart1_write_text(&ctx->debug,
        "BALL MANIFEST RESET TOTAL=0/9\r\n");
    return true;
}

/** @brief 识别并提交一条USART1命令。 */
static bool modules_handle_command(const uint8_t *data, size_t len)
{
    modules_test_context_t *ctx = &g_modules_test;
    mult_uart_status_t mult_status = MULT_UART_ERR_PARAM;
    nano_vision_status_t vision_status = NANO_VISION_ERR_TYPE;
    ic_card_status_t ic_status = IC_CARD_ERR_UNSUPPORTED;
    zdt_turntable_status_t zdt_status = ZDT_TURNTABLE_ERR_UNSUPPORTED;
    uint8_t channel;

    if (modules_command_equals(data, len, "HELP")) {
        (void)debug_uart1_write_text(&ctx->debug,
            "HELP STATUS GATE SELECT 0..3 ARM_HOME ARM_VISION ARM_STOP "
            "GRASP_SINGLE GRASP_ARM_STATIC GRASP_ARM_MOVING "
            "GRASP_START_TURNTABLE_RED GRASP_START_TURNTABLE_BLUE "
            "GRASP_FAST_SINGLE_RED GRASP_FAST_SINGLE_BLUE "
            "GRASP_FAST_MULTI_RED GRASP_FAST_MULTI_BLUE GRASP_MULTI_STOP "
            "GRASP_MULTI_STATUS BALL_MANIFEST BALL_RESET "
            "VISION_START_TURNTABLE_RED "
            "VISION_START_TURNTABLE_BLUE VISION_START_STAIR_RED VISION_START_STAIR_BLUE "
            "VISION_STOP VISION_TURNTABLE_RED VISION_TURNTABLE_BLUE "
            "VISION_STAIR_RED VISION_STAIR_BLUE IC_READ IC_ADDRESS IC_MODE IC_BEEPER IC_AUTO "
            "ZDT_VERSION ZDT_OPTIONS ZDT_STATUS ZDT_POSITION ZDT_CW ZDT_CCW "
            "ZDT_COARSE_CW ZDT_COARSE_CCW ZDT_FINE_CW ZDT_FINE_CCW "
            "ZDT_SLOT_CW ZDT_SLOT_CCW ZDT_STOP\r\n");
        return false;
    }
    if (modules_command_equals(data, len, "GATE")) {
        (void)snprintf(ctx->text, sizeof(ctx->text),
            "GATE PB0=%u\r\n",
            gate_read() ? 1U : 0U);
        (void)debug_uart1_write_text(&ctx->debug, ctx->text);
        return false;
    }
    if (modules_command_equals(data, len, "STATUS")) {
        mult_uart_service_os_diagnostics_t diagnostics;
        (void)snprintf(ctx->text, sizeof(ctx->text),
            "READY SELECTED=%d PENDING=%u MOTION_ARMED=%u SLOT_ACTIVE=%u "
            "SLOT_STATE=%u SLOT_FINE=%u SELECT_CB=%lu EVENT_DROP=%lu WATCHDOG=%lu "
            "CH0=VISION CH1=IC CH2=ZDT CH3=RESERVED\r\n",
            ctx->selected_channel_valid ? (int)ctx->selected_channel : -1,
            ctx->command_pending ? 1U : 0U,
            MULT_UART_MODULES_ZDT_MOTION_ARMED,
            ctx->slot.active ? 1U : 0U,
            (unsigned)ctx->slot.state,
            (unsigned)ctx->slot.fine_steps,
            (unsigned long)ctx->select_callback_count,
            (unsigned long)ctx->event_drop_count,
            (unsigned long)ctx->command_watchdog_count);
        (void)debug_uart1_write_text(&ctx->debug, ctx->text);
        (void)snprintf(ctx->text, sizeof(ctx->text),
            "ARM MOTION_ARMED=%u POSE=%s PHASE=%u FAULT=%u TX_PENDING=%u "
            "GROUP=%u LAST=%u TRIGGER=%s RETURN11=%u RETURN_WAIT=%u SUB=%lu START=%lu "
            "DONE=%lu FIRE=%lu REJECT=%lu MISMATCH=%lu\r\n",
            MULT_UART_MODULES_LSC16_MOTION_ARMED,
            modules_arm_pose_name(ctx->grasp.pose),
            (unsigned)ctx->grasp.guard.phase,
            ctx->grasp.guard.fault_latched ? 1U : 0U,
            ctx->grasp.tx_pending ? 1U : 0U,
            (unsigned)ctx->grasp.active_group,
            (unsigned)ctx->grasp.last_completed_group,
            modules_grasp_trigger_name(ctx->grasp.trigger),
            ctx->grasp.returning_to_vision ? 1U : 0U,
            ctx->grasp.auto_return_pending ? 1U : 0U,
            (unsigned long)ctx->grasp.action_submit_count,
            (unsigned long)ctx->grasp.action_started_count,
            (unsigned long)ctx->grasp.action_completed_count,
            (unsigned long)ctx->grasp.trigger_count,
            (unsigned long)ctx->grasp.rejected_count,
            (unsigned long)ctx->grasp.mismatched_report_count);
        (void)debug_uart1_write_text(&ctx->debug, ctx->text);
        (void)snprintf(ctx->text, sizeof(ctx->text),
            "VISION ONLINE=%u ACTIVE=%u SCENE=%u COLOR=%u STREAK=%u "
            "TIMEOUT=%lu CRC=%lu SEQ_ERR=%lu\r\n",
            ctx->vision_transport.tracker.online ? 1U : 0U,
            ctx->vision_transport.active ? 1U : 0U,
            (unsigned)ctx->vision_transport.tracker_config.scene,
            (unsigned)ctx->vision_transport.tracker_config.target_color,
            (unsigned)ctx->vision_transport.tracker.aligned_streak,
            (unsigned long)ctx->vision_transport.tracker.timeout_count,
            (unsigned long)ctx->vision_transport.parser.crc_errors,
            (unsigned long)ctx->vision_transport.tracker.sequence_errors);
        (void)debug_uart1_write_text(&ctx->debug, ctx->text);
        (void)snprintf(ctx->text, sizeof(ctx->text),
            "VISION SESSION=%u STOPPING=%u RATE=%uHZ POLL=%lu DONE=%lu "
            "ERR=%lu SUBMIT_ERR=%lu EARLY=%u STRICT=%u "
            "EARLY_ENTER=%lu STRICT_ENTER=%lu\r\n",
            ctx->vision_session.enabled ? 1U : 0U,
            ctx->vision_session.stop_pending ? 1U : 0U,
            (unsigned)(1000U / MULT_UART_MODULES_VISION_POLL_PERIOD_MS),
            (unsigned long)ctx->vision_session.poll_count,
            (unsigned long)ctx->vision_session.completed_count,
            (unsigned long)ctx->vision_session.error_count,
            (unsigned long)ctx->vision_session.submit_error_count,
            ctx->vision_session.early_ready ? 1U : 0U,
            ctx->vision_session.strict_ready ? 1U : 0U,
            (unsigned long)ctx->vision_session.early_enter_count,
            (unsigned long)ctx->vision_session.strict_enter_count);
        (void)debug_uart1_write_text(&ctx->debug, ctx->text);
        if (mult_uart_service_os_get_diagnostics(&diagnostics) == MULT_UART_OK) {
            (void)snprintf(ctx->text, sizeof(ctx->text),
                "OS WORKER_LOOP=%lu OS_SUB=%lu OS_DEQ=%lu OS_Q=%lu "
                "NOTIFY_ERR=%lu SVC_SUB=%lu SVC_DONE=%lu "
                "UART_ERR=%lu UART_LAST=%08lX\r\n",
                (unsigned long)diagnostics.worker_loop_count,
                (unsigned long)diagnostics.os_submit_count,
                (unsigned long)diagnostics.os_dequeue_count,
                (unsigned long)diagnostics.os_queue_count,
                (unsigned long)diagnostics.notify_error_count,
                (unsigned long)diagnostics.service_submit_count,
                (unsigned long)diagnostics.service_complete_count,
                (unsigned long)diagnostics.uart_error_count,
                (unsigned long)diagnostics.last_uart_error);
            (void)debug_uart1_write_text(&ctx->debug, ctx->text);
        }
        modules_multi_print_status(ctx);
        return false;
    }
    if (modules_command_equals(data, len, "GRASP_MULTI_STATUS")) {
        modules_multi_print_status(ctx);
        return false;
    }
    if (modules_command_equals(data, len, "BALL_MANIFEST")) {
        modules_manifest_print(ctx);
        return false;
    }
    if (modules_command_equals(data, len, "BALL_RESET")) {
        (void)modules_manifest_reset(ctx);
        return false;
    }
    if (modules_command_equals(data, len, "GRASP_MULTI_STOP")) {
        modules_multi_request_stop(ctx);
        return false;
    }
    if (modules_command_equals(data, len, "ARM_STOP")) {
        lsc16_status_t arm_status = modules_request_arm_stop(ctx, false);
        if (ctx->multi.active) {
            modules_multi_fail(ctx, "ARM_STOP");
        }
        if (arm_status != LSC16_OK) {
            (void)snprintf(ctx->text, sizeof(ctx->text),
                "ARM STOP SUBMIT ERROR=%u\r\n", (unsigned)arm_status);
            (void)debug_uart1_write_text(&ctx->debug, ctx->text);
        }
        return false;
    }
    if (modules_command_equals(data, len, "GRASP_FAST_MULTI_RED") ||
        modules_command_equals(data, len, "GRASP_FAST_MULTI_BLUE")) {
        nano_vision_color_t color =
            modules_command_equals(data, len, "GRASP_FAST_MULTI_RED") ?
                NANO_VISION_COLOR_RED : NANO_VISION_COLOR_BLUE;
        (void)modules_multi_start(ctx, color);
        return false;
    }
    if (ctx->multi.active) {
        (void)debug_uart1_write_text(&ctx->debug,
            "MULTI ACTIVE: ONLY STATUS, BALL_MANIFEST, GRASP_MULTI_STOP OR ARM_STOP ALLOWED\r\n");
        return false;
    }
    if (modules_command_equals(data, len, "ARM_HOME") ||
        modules_command_equals(data, len, "ARM_VISION") ||
        modules_command_equals(data, len, "GRASP_SINGLE") ||
        modules_command_equals(data, len, "GRASP_ARM_STATIC") ||
        modules_command_equals(data, len, "GRASP_ARM_MOVING") ||
        modules_command_equals(data, len, "GRASP_START_TURNTABLE_RED") ||
        modules_command_equals(data, len, "GRASP_START_TURNTABLE_BLUE") ||
        modules_command_equals(data, len, "GRASP_FAST_SINGLE_RED") ||
        modules_command_equals(data, len, "GRASP_FAST_SINGLE_BLUE")) {
#if MULT_UART_MODULES_LSC16_MOTION_ARMED
        lsc16_status_t arm_status = LSC16_ERR_UNSUPPORTED;

        if (ctx->grasp.guard.fault_latched) {
            (void)debug_uart1_write_text(&ctx->debug,
                "ARM FAULT LATCHED. RESET F7 BEFORE NEW MOTION\r\n");
            return false;
        }
        if (ctx->grasp.tx_pending ||
            lsc16_test_action_guard_is_active(&ctx->grasp.guard)) {
            ctx->grasp.rejected_count++;
            (void)debug_uart1_write_text(&ctx->debug,
                "ARM BUSY: WAIT CURRENT ACTION\r\n");
            return false;
        }
        if (modules_command_equals(data, len, "ARM_HOME")) {
            if (ctx->vision_session.enabled || ctx->vision_session.stop_pending) {
                (void)debug_uart1_write_text(&ctx->debug,
                    "SEND VISION_STOP BEFORE ARM_HOME\r\n");
                return false;
            }
            ctx->grasp.trigger = MODULES_GRASP_TRIGGER_NONE;
            ctx->grasp.returning_to_vision = false;
            arm_status = modules_submit_arm_action(
                ctx, MULT_UART_MODULES_LSC16_HOME_GROUP);
        } else if (modules_command_equals(data, len, "ARM_VISION")) {
            if (ctx->grasp.pose != MODULES_ARM_POSE_HOME_10) {
                (void)debug_uart1_write_text(&ctx->debug,
                    "ARM_VISION REJECTED: COMPLETE ARM_HOME FIRST\r\n");
                return false;
            }
            if (ctx->vision_session.enabled || ctx->vision_session.stop_pending) {
                (void)debug_uart1_write_text(&ctx->debug,
                    "ARM_VISION REJECTED: VISION SESSION ACTIVE\r\n");
                return false;
            }
            arm_status = modules_submit_arm_action(
                ctx, MULT_UART_MODULES_LSC16_VISION_GROUP);
        } else if (modules_command_equals(data, len, "GRASP_SINGLE")) {
            if (ctx->grasp.pose != MODULES_ARM_POSE_VISION_11) {
                (void)debug_uart1_write_text(&ctx->debug,
                    "GRASP_SINGLE REJECTED: COMPLETE ARM_HOME THEN ARM_VISION\r\n");
                return false;
            }
            if (ctx->vision_session.enabled || ctx->vision_session.stop_pending) {
                (void)debug_uart1_write_text(&ctx->debug,
                    "GRASP_SINGLE REJECTED: SEND VISION_STOP FIRST\r\n");
                return false;
            }
            ctx->grasp.trigger = MODULES_GRASP_TRIGGER_NONE;
            ctx->grasp.trigger_count++;
            (void)debug_uart1_write_text(&ctx->debug,
                "GRASP TRIGGER MODE=MANUAL ONE_SHOT\r\n");
            arm_status = modules_submit_arm_action(
                ctx, MULT_UART_MODULES_LSC16_GRASP_PLACE_GROUP);
        } else {
            bool combined_moving_start =
                modules_command_equals(data, len, "GRASP_START_TURNTABLE_RED") ||
                modules_command_equals(data, len, "GRASP_START_TURNTABLE_BLUE") ||
                modules_command_equals(data, len, "GRASP_FAST_SINGLE_RED") ||
                modules_command_equals(data, len, "GRASP_FAST_SINGLE_BLUE");
            modules_grasp_trigger_t trigger =
                modules_command_equals(data, len, "GRASP_ARM_STATIC") ?
                    MODULES_GRASP_TRIGGER_STATIC : MODULES_GRASP_TRIGGER_MOVING;
            if (ctx->grasp.pose != MODULES_ARM_POSE_VISION_11) {
                (void)debug_uart1_write_text(&ctx->debug,
                    "GRASP ARM REJECTED: COMPLETE ARM_HOME THEN ARM_VISION\r\n");
                return false;
            }
            if (combined_moving_start) {
                nano_vision_color_t color =
                    (modules_command_equals(data, len, "GRASP_START_TURNTABLE_RED") ||
                     modules_command_equals(data, len, "GRASP_FAST_SINGLE_RED")) ?
                        NANO_VISION_COLOR_RED : NANO_VISION_COLOR_BLUE;
                if (!modules_start_vision_session(
                        ctx, NANO_VISION_SCENE_TURNTABLE, color)) {
                    return false;
                }
            } else {
                if (!ctx->vision_session.enabled ||
                    (ctx->vision_session.scene != NANO_VISION_SCENE_TURNTABLE)) {
                    (void)debug_uart1_write_text(&ctx->debug,
                        "GRASP ARM REJECTED: START TURNTABLE VISION SESSION FIRST\r\n");
                    return false;
                }
            }
            ctx->grasp.trigger = trigger;
            (void)snprintf(ctx->text, sizeof(ctx->text),
                "GRASP ARMED MODE=%s ONE_SHOT. %s\r\n",
                modules_grasp_trigger_name(trigger),
                (trigger == MODULES_GRASP_TRIGGER_MOVING) ?
                    "WAIT FAST WINDOW CANDIDATE" :
                    "WAIT STRICT VISION CANDIDATE");
            (void)debug_uart1_write_text(&ctx->debug, ctx->text);
            return false;
        }
        if (arm_status != LSC16_OK) {
            ctx->grasp.rejected_count++;
            (void)snprintf(ctx->text, sizeof(ctx->text),
                "ARM ACTION SUBMIT ERROR=%u\r\n", (unsigned)arm_status);
            (void)debug_uart1_write_text(&ctx->debug, ctx->text);
        }
#else
        (void)debug_uart1_write_text(&ctx->debug,
            "ARM MOTION LOCKED: SET MULT_UART_MODULES_LSC16_MOTION_ARMED=1\r\n");
#endif
        return false;
    }
    if (modules_command_equals(data, len, "VISION_STOP")) {
        modules_stop_vision_session(ctx);
        return false;
    }
    if (modules_command_equals(data, len, "VISION_START_TURNTABLE_RED")) {
        modules_start_vision_session(
            ctx, NANO_VISION_SCENE_TURNTABLE, NANO_VISION_COLOR_RED);
        return false;
    }
    if (modules_command_equals(data, len, "VISION_START_TURNTABLE_BLUE")) {
        modules_start_vision_session(
            ctx, NANO_VISION_SCENE_TURNTABLE, NANO_VISION_COLOR_BLUE);
        return false;
    }
    if (modules_command_equals(data, len, "VISION_START_STAIR_RED")) {
        modules_start_vision_session(
            ctx, NANO_VISION_SCENE_STAIR, NANO_VISION_COLOR_RED);
        return false;
    }
    if (modules_command_equals(data, len, "VISION_START_STAIR_BLUE")) {
        modules_start_vision_session(
            ctx, NANO_VISION_SCENE_STAIR, NANO_VISION_COLOR_BLUE);
        return false;
    }
    if (ctx->vision_session.enabled || ctx->vision_session.stop_pending) {
        (void)debug_uart1_write_text(&ctx->debug,
            "VISION SESSION ACTIVE. SEND VISION_STOP FIRST\r\n");
        return false;
    }
    if (ctx->command_pending) {
        (void)debug_uart1_write_text(&ctx->debug, "BUSY: WAIT CURRENT COMMAND\r\n");
        return false;
    }
    if (modules_parse_select(data, len, &channel)) {
        /* 清除旧邮箱必须发生在提交前，防止worker快速完成时误删新通知。 */
        ctx->select_completion_ready = false;
        mult_status = modules_submit_select(channel);
        if (mult_status == MULT_UART_OK) {
            modules_mark_command_pending(ctx);
            return true;
        }
    } else if (modules_command_equals(data, len, "VISION_TURNTABLE_RED")) {
        vision_status = modules_submit_vision_poll(
            NANO_VISION_SCENE_TURNTABLE, NANO_VISION_COLOR_RED);
    } else if (modules_command_equals(data, len, "VISION_TURNTABLE_BLUE")) {
        vision_status = modules_submit_vision_poll(
            NANO_VISION_SCENE_TURNTABLE, NANO_VISION_COLOR_BLUE);
    } else if (modules_command_equals(data, len, "VISION_STAIR_RED")) {
        vision_status = modules_submit_vision_poll(
            NANO_VISION_SCENE_STAIR, NANO_VISION_COLOR_RED);
    } else if (modules_command_equals(data, len, "VISION_STAIR_BLUE")) {
        vision_status = modules_submit_vision_poll(
            NANO_VISION_SCENE_STAIR, NANO_VISION_COLOR_BLUE);
    } else if (modules_command_equals(data, len, "IC_READ")) {
        ic_status = ic_card_device_read_competition_ball(
            (MULT_UART_MODULES_IC_OPERATION_PROMPT != 0U),
            modules_ic_read_done,
            NULL);
    } else if (modules_command_equals(data, len, "IC_ADDRESS")) {
        ic_status = ic_card_device_query(
            IC_CARD_CMD_QUERY_ADDRESS, modules_ic_query_done, NULL);
    } else if (modules_command_equals(data, len, "IC_MODE")) {
        ic_status = ic_card_device_query(
            IC_CARD_CMD_QUERY_WORK_MODE, modules_ic_query_done, NULL);
    } else if (modules_command_equals(data, len, "IC_BEEPER")) {
        ic_status = ic_card_device_query(
            IC_CARD_CMD_QUERY_BEEPER, modules_ic_query_done, NULL);
    } else if (modules_command_equals(data, len, "IC_AUTO")) {
        ic_status = ic_card_device_query(
            IC_CARD_CMD_QUERY_AUTO_READ, modules_ic_query_done, NULL);
    } else if (modules_command_equals(data, len, "ZDT_VERSION")) {
        zdt_status = zdt_turntable_device_query(
            &ctx->zdt, 0x1FU, modules_zdt_done, NULL);
    } else if (modules_command_equals(data, len, "ZDT_OPTIONS")) {
        zdt_status = zdt_turntable_device_query_options(
            &ctx->zdt, modules_zdt_done, NULL);
    } else if (modules_command_equals(data, len, "ZDT_STATUS")) {
        zdt_status = zdt_turntable_device_query(
            &ctx->zdt, 0x3AU, modules_zdt_done, NULL);
    } else if (modules_command_equals(data, len, "ZDT_POSITION")) {
        zdt_status = zdt_turntable_device_query(
            &ctx->zdt, 0x36U, modules_zdt_done, NULL);
    } else if (modules_command_equals(data, len, "ZDT_SLOT_CW") ||
               modules_command_equals(data, len, "ZDT_SLOT_CCW")) {
#if MULT_UART_MODULES_ZDT_MOTION_ARMED
        zdt_turntable_direction_t slot_direction =
            modules_command_equals(data, len, "ZDT_SLOT_CW") ?
                ZDT_TURNTABLE_DIR_CW : ZDT_TURNTABLE_DIR_CCW;
        if (!ctx->zdt.firmware_known) {
            (void)debug_uart1_write_text(&ctx->debug,
                "SLOT ERROR SEND ZDT_OPTIONS FIRST\r\n");
            return false;
        }
        if (!modules_gate_is_stably_high()) {
            (void)debug_uart1_write_text(&ctx->debug,
                "SLOT ERROR START PB0 NOT STABLY HIGH\r\n");
            return false;
        }
        zdt_status = modules_slot_start(ctx, slot_direction);
        if (zdt_status == ZDT_TURNTABLE_OK) {
            (void)snprintf(ctx->text, sizeof(ctx->text),
                "SLOT START DIR=%s COARSE=140.0DEG\r\n",
                (slot_direction == ZDT_TURNTABLE_DIR_CW) ? "CW" : "CCW");
            (void)debug_uart1_write_text(&ctx->debug, ctx->text);
        }
#else
        (void)debug_uart1_write_text(&ctx->debug,
            "MOTION LOCKED: SET MULT_UART_MODULES_ZDT_MOTION_ARMED=1\r\n");
        return false;
#endif
    } else if (modules_command_equals(data, len, "ZDT_CW") ||
               modules_command_equals(data, len, "ZDT_CCW") ||
               modules_command_equals(data, len, "ZDT_COARSE_CW") ||
               modules_command_equals(data, len, "ZDT_COARSE_CCW") ||
               modules_command_equals(data, len, "ZDT_FINE_CW") ||
               modules_command_equals(data, len, "ZDT_FINE_CCW")) {
#if MULT_UART_MODULES_ZDT_MOTION_ARMED
        zdt_turntable_direction_t direction =
            (modules_command_equals(data, len, "ZDT_CW") ||
             modules_command_equals(data, len, "ZDT_COARSE_CW") ||
             modules_command_equals(data, len, "ZDT_FINE_CW")) ?
                ZDT_TURNTABLE_DIR_CW : ZDT_TURNTABLE_DIR_CCW;
        uint32_t angle_0p1deg = MULT_UART_MODULES_ZDT_MOTOR_ANGLE_0P1DEG;
        uint16_t speed_rpm = MULT_UART_MODULES_ZDT_SPEED_RPM;
        if (modules_command_equals(data, len, "ZDT_COARSE_CW") ||
            modules_command_equals(data, len, "ZDT_COARSE_CCW")) {
            angle_0p1deg = MULT_UART_MODULES_ZDT_COARSE_ANGLE_0P1DEG;
        } else if (modules_command_equals(data, len, "ZDT_FINE_CW") ||
                   modules_command_equals(data, len, "ZDT_FINE_CCW")) {
            angle_0p1deg = MULT_UART_MODULES_ZDT_FINE_ANGLE_0P1DEG;
            speed_rpm = MULT_UART_MODULES_ZDT_FINE_SPEED_RPM;
        }
        zdt_status = modules_submit_motion(
            direction, angle_0p1deg, speed_rpm);
#else
        (void)debug_uart1_write_text(&ctx->debug,
            "MOTION LOCKED: SET MULT_UART_MODULES_ZDT_MOTION_ARMED=1\r\n");
        return false;
#endif
    } else if (modules_command_equals(data, len, "ZDT_STOP")) {
        zdt_status = zdt_turntable_device_stop(
            &ctx->zdt, modules_zdt_done, NULL);
    } else {
        (void)debug_uart1_write_text(&ctx->debug, "UNKNOWN COMMAND. SEND HELP\r\n");
        return false;
    }

    if ((vision_status == NANO_VISION_OK) ||
        (ic_status == IC_CARD_OK) || (zdt_status == ZDT_TURNTABLE_OK)) {
        modules_mark_command_pending(ctx);
        return true;
    }
    (void)snprintf(ctx->text, sizeof(ctx->text),
        "SUBMIT ERROR MULT=%u VISION=%u IC=%u ZDT=%u\r\n",
        (unsigned)mult_status, (unsigned)vision_status,
        (unsigned)ic_status, (unsigned)zdt_status);
    (void)debug_uart1_write_text(&ctx->debug, ctx->text);
    return false;
}

/** @brief 输出查询响应的原始字节，便于与手册逐字节核对。 */
static void modules_format_ic_query(
    const modules_event_t *event, char *text, size_t capacity)
{
    size_t used = 0U;
    int count;
    uint8_t i;
    if ((event->ic_status != IC_CARD_OK) || !event->has_response) {
        (void)snprintf(text, capacity, "IC ERROR %u\r\n", event->ic_status);
        return;
    }
    count = snprintf(text, capacity, "IC CMD=%02X STATUS=%02X RAW=",
        event->ic_response.command, event->ic_response.device_status);
    if ((count <= 0) || ((size_t)count >= capacity)) return;
    used = (size_t)count;
    for (i = 0U; (i < event->ic_response.raw_len) && (used + 4U < capacity); ++i) {
        count = snprintf(&text[used], capacity - used, "%02X ",
            event->ic_response.raw[i]);
        if (count <= 0) break;
        used += (size_t)count;
    }
    if (used + 2U < capacity) { text[used++] = '\r'; text[used++] = '\n'; text[used] = '\0'; }
}

/** @brief 在唯一测试任务中格式化并输出完成事件。 */
static void modules_handle_event(const modules_event_t *event)
{
    modules_test_context_t *ctx = &g_modules_test;
    modules_vision_session_t *session = &ctx->vision_session;
    size_t len = 0U;
    bool session_event;
    bool should_log = true;

    ctx->text[0] = '\0';

    if (event->kind == MODULES_EVENT_LSC16_TX_DONE) {
        modules_handle_lsc16_tx_done(ctx, event);
        return;
    }
    if (event->kind == MODULES_EVENT_LSC16_REPORT) {
        modules_handle_lsc16_report(ctx, event);
        return;
    }
    if ((event->kind == MODULES_EVENT_ZDT) && ctx->slot.active) {
        ctx->selected_channel = (uint8_t)MULT_UART_MODULES_ZDT_DEVICE_ID;
        ctx->selected_channel_valid = true;
        modules_slot_handle_zdt_event(ctx, event);
        return;
    }
    ctx->command_pending = false;
    if (event->kind == MODULES_EVENT_SELECT) {
        if (event->mult_status == MULT_UART_OK) {
            ctx->selected_channel = event->channel;
            ctx->selected_channel_valid = true;
        }
        (void)snprintf(ctx->text, sizeof(ctx->text),
            "SELECT CH%u %s\r\n", event->channel,
            (event->mult_status == MULT_UART_OK) ? "OK" : "ERROR");
    } else if (event->kind == MODULES_EVENT_VISION) {
        ctx->selected_channel = (uint8_t)MULT_UART_MODULES_VISION_DEVICE_ID;
        ctx->selected_channel_valid = true;
        session_event = session->enabled || session->stop_pending;
        if (session_event) {
            bool status_changed;
            bool aligned_changed;
            bool online_changed;
            uint8_t observation_status = (event->vision_status == NANO_VISION_OK) ?
                event->vision_observation.status : 0xFFU;

            session->completed_count++;
            if (event->vision_status != NANO_VISION_OK) session->error_count++;
            status_changed = !session->have_last_result ||
                (session->last_transport_status != event->vision_status) ||
                (session->last_observation_status != observation_status);
            aligned_changed = !session->have_last_result ||
                (session->last_aligned != event->vision_aligned);
            online_changed = !session->have_last_result ||
                (session->last_online != event->vision_online);
            should_log = (event->vision_status != NANO_VISION_OK) ||
                status_changed || aligned_changed || online_changed ||
                ((session->completed_count %
                  MULT_UART_MODULES_VISION_LOG_EVERY_POLLS) == 0U);
            session->have_last_result = true;
            session->last_transport_status = event->vision_status;
            session->last_observation_status = observation_status;
            session->last_online = event->vision_online;
            session->last_aligned = event->vision_aligned;

            if (session->stop_pending) {
                session->stop_pending = false;
                session->early_ready = false;
                session->strict_ready = false;
                (void)snprintf(ctx->text, sizeof(ctx->text),
                    "VISION SESSION STOPPED POLL=%lu DONE=%lu ERR=%lu "
                    "EARLY_ENTER=%lu STRICT_ENTER=%lu\r\n",
                    (unsigned long)session->poll_count,
                    (unsigned long)session->completed_count,
                    (unsigned long)session->error_count,
                    (unsigned long)session->early_enter_count,
                    (unsigned long)session->strict_enter_count);
                should_log = false;
            } else {
                bool early_ready = false;
                bool strict_ready = event->vision_aligned;

                if ((event->vision_status == NANO_VISION_OK) &&
                    session->early_compare_enabled) {
                    early_ready = nano_vision_observation_is_early_candidate(
                        &event->vision_observation, &session->early_config);
                }
                if (early_ready != session->early_ready) {
                    session->early_ready = early_ready;
                    if (early_ready) session->early_enter_count++;
                    (void)snprintf(ctx->text, sizeof(ctx->text),
                        "VISION EARLY %s FRAME=%u QUALITY=%u DX=%d DY=%d "
                        "AGE=%u\r\n",
                        early_ready ? "ENTER" : "EXIT",
                        (unsigned)event->vision_observation.frame_id,
                        (unsigned)event->vision_observation.quality,
                        (int)event->vision_observation.offset_x_px,
                        (int)event->vision_observation.offset_y_px,
                        (unsigned)event->vision_observation.age_ms);
                    (void)debug_uart1_write_text(&ctx->debug, ctx->text);
                    ctx->text[0] = '\0';
                }
                if (strict_ready != session->strict_ready) {
                    session->strict_ready = strict_ready;
                    if (strict_ready) session->strict_enter_count++;
                    (void)snprintf(ctx->text, sizeof(ctx->text),
                        "VISION STRICT %s FRAME=%u STREAK=%u DX=%d DY=%d\r\n",
                        strict_ready ? "ENTER" : "EXIT",
                        (unsigned)event->vision_observation.frame_id,
                        (unsigned)event->vision_streak,
                        (int)event->vision_observation.offset_x_px,
                        (int)event->vision_observation.offset_y_px);
                    (void)debug_uart1_write_text(&ctx->debug, ctx->text);
                    ctx->text[0] = '\0';
                }
                if ((event->vision_status == NANO_VISION_OK) &&
                    (ctx->grasp.trigger != MODULES_GRASP_TRIGGER_NONE)) {
                    modules_maybe_trigger_grasp(
                        ctx,
                        &event->vision_observation,
                        strict_ready,
                        early_ready);
                    ctx->text[0] = '\0';
                }
            }
        }
        if (event->vision_status == NANO_VISION_OK) {
            if (should_log) (void)snprintf(ctx->text, sizeof(ctx->text),
                "VISION OK SCENE=%u STATUS=%u COLOR=%u QUALITY=%u "
                "DX=%d DY=%d FRAME=%u AGE=%u ONLINE=%u STREAK=%u "
                "ALIGNED=%u EARLY=%u STRICT=%u\r\n",
                (unsigned)event->vision_observation.scene,
                (unsigned)event->vision_observation.status,
                (unsigned)event->vision_observation.color,
                (unsigned)event->vision_observation.quality,
                (int)event->vision_observation.offset_x_px,
                (int)event->vision_observation.offset_y_px,
                (unsigned)event->vision_observation.frame_id,
                (unsigned)event->vision_observation.age_ms,
                event->vision_online ? 1U : 0U,
                (unsigned)event->vision_streak,
                event->vision_aligned ? 1U : 0U,
                (session_event && session->early_ready) ? 1U : 0U,
                (session_event && session->strict_ready) ? 1U : 0U);
        } else {
            if (should_log) (void)snprintf(ctx->text, sizeof(ctx->text),
                "VISION ERROR=%u ONLINE=%u TIMEOUT=%lu CRC=%lu SEQ_ERR=%lu\r\n",
                (unsigned)event->vision_status,
                event->vision_online ? 1U : 0U,
                (unsigned long)event->vision_timeouts,
                (unsigned long)event->vision_crc_errors,
                (unsigned long)event->vision_sequence_errors);
        }
        if (ctx->multi.active &&
            (ctx->multi.phase == MODULES_MULTI_WAIT_VISION) &&
            !event->vision_online &&
            (event->vision_status == NANO_VISION_ERR_TIMEOUT)) {
            modules_multi_fail(ctx, "VISION_OFFLINE");
        }
    } else if (event->kind == MODULES_EVENT_IC_READ) {
        ctx->selected_channel = (uint8_t)MULT_UART_MODULES_IC_DEVICE_ID;
        ctx->selected_channel_valid = true;
        if (ctx->multi.active &&
            (ctx->multi.phase == MODULES_MULTI_WAIT_IC)) {
            modules_multi_handle_ic_event(ctx, event);
            return;
        }
        if (event->ic_status == IC_CARD_OK) {
            /* 保持与直连测试相同的比赛球输出格式。 */
            len = ic_card_test_format_success(
                &event->ball, ctx->text, sizeof(ctx->text));
        } else {
            (void)snprintf(ctx->text, sizeof(ctx->text),
                "IC READ ERROR %u\r\n", event->ic_status);
        }
    } else if (event->kind == MODULES_EVENT_IC_QUERY) {
        ctx->selected_channel = (uint8_t)MULT_UART_MODULES_IC_DEVICE_ID;
        ctx->selected_channel_valid = true;
        modules_format_ic_query(event, ctx->text, sizeof(ctx->text));
    } else {
        ctx->selected_channel = (uint8_t)MULT_UART_MODULES_ZDT_DEVICE_ID;
        ctx->selected_channel_valid = true;
        len = zdt_turntable_test_format_result(
            event->zdt_status,
            event->has_response ? &event->zdt_response : NULL,
            ctx->text,
            sizeof(ctx->text));
    }
    if (len == 0U) len = strlen(ctx->text);
    if (len > 0U) (void)debug_uart1_write(&ctx->debug, (uint8_t *)ctx->text, len);
}

/**
 * @brief 在测试任务中取出SELECT专用完成邮箱并复用统一输出逻辑。
 * @note 内存屏障保证看到ready后，先前由worker写入的状态和通道也已经可见。
 */
static void modules_handle_select_completion(modules_test_context_t *ctx)
{
    modules_event_t event;

    if (!ctx->select_completion_ready) return;
    __DMB();
    (void)memset(&event, 0, sizeof(event));
    event.kind = MODULES_EVENT_SELECT;
    event.mult_status = ctx->select_completion_status;
    event.channel = ctx->select_completion_channel;
    ctx->select_completion_ready = false;
    modules_handle_event(&event);
}

/**
 * @brief 防止完成通知异常时命令门永久锁死。
 * @note 正常IC/ZDT事务在配置的IO超时内结束；2秒仅是测试界面的最后保护，
 *       触发后说明完成通知链异常，不代表外设通信成功。
 */
static void modules_release_stale_command(modules_test_context_t *ctx)
{
    uint32_t tick_frequency;
    uint32_t timeout_ticks;
    uint32_t elapsed_ticks;

    if (!ctx->command_pending || ctx->slot.active) return;
    tick_frequency = osKernelGetTickFreq();
    if (tick_frequency == 0U) return;
    timeout_ticks = (uint32_t)((
        (uint64_t)MODULES_COMMAND_WATCHDOG_MS * tick_frequency + 999ULL) /
        1000ULL);
    if (timeout_ticks == 0U) timeout_ticks = 1U;
    elapsed_ticks = osKernelGetTickCount() - ctx->command_started_tick;
    if (elapsed_ticks < timeout_ticks) return;

    ctx->command_pending = false;
    ctx->command_watchdog_count++;
    (void)debug_uart1_write_text(&ctx->debug,
        "INTERNAL COMPLETION TIMEOUT: COMMAND GATE RELEASED\r\n");
}

/** @brief USART1命令轮询和完成事件输出任务。 */
static void modules_task_entry(void *argument)
{
    modules_test_context_t *ctx = (modules_test_context_t *)argument;
    modules_event_t event;
    uint8_t command[DEBUG_UART1_RX_BUFFER_SIZE];
    size_t command_len;

    /* 读卡器资料要求参数设置后保留保护时间；上电统一等待后才接收命令。 */
    (void)osDelay(MULT_UART_MODULES_STARTUP_WAIT_MS);
    (void)snprintf(ctx->text, sizeof(ctx->text),
        "UART7 MODULES FREERTOS READY. ARM_LOCK=%u ZDT_LOCK=%u GROUPS=10/11/12. SEND HELP\r\n",
        MULT_UART_MODULES_LSC16_MOTION_ARMED,
        MULT_UART_MODULES_ZDT_MOTION_ARMED);
    (void)debug_uart1_write_text(&ctx->debug, ctx->text);
    for (;;) {
        modules_handle_select_completion(ctx);
        while (osMessageQueueGet(ctx->event_queue, &event, NULL, 0U) == osOK) {
            modules_handle_event(&event);
        }
        modules_slot_process(ctx);
        modules_multi_process(ctx);
        modules_release_stale_command(ctx);
        modules_check_arm_timeout(ctx);
        if (debug_uart1_take_message(
                &ctx->debug, command, sizeof(command), &command_len)) {
            (void)modules_handle_command(command, command_len);
        }
        modules_process_vision_session(ctx);
        (void)osDelay(5U);
    }
}

/** @copydoc mult_uart_modules_freertos_test_init() */
mult_uart_status_t mult_uart_modules_freertos_test_init(void)
{
    modules_test_context_t *ctx = &g_modules_test;
    zdt_turntable_device_config_t zdt_config = {
        MULT_UART_MODULES_ZDT_ADDRESS,
        MULT_UART_MODULES_IO_TIMEOUT_MS,
        MULT_UART_MODULES_ZDT_EMM_PULSES_PER_REV,
    };

    if (ctx->initialized) return MULT_UART_ERR_STATE;
    (void)memset(ctx, 0, sizeof(*ctx));
    ball_manifest_init(&ctx->manifest);
    ctx->multi.phase = MODULES_MULTI_IDLE;
    if (!debug_uart1_init(&ctx->debug)) return MULT_UART_ERR_IO;
    ctx->event_queue = osMessageQueueNew(
        MODULES_EVENT_QUEUE_DEPTH, sizeof(modules_event_t), NULL);
    if (ctx->event_queue == NULL) {
        debug_uart1_deinit(&ctx->debug);
        return MULT_UART_ERR_IO;
    }
    if (ic_card_device_init_with_transport(
            modules_ic_submit, &ctx->ic_transport) != IC_CARD_OK) {
        debug_uart1_deinit(&ctx->debug);
        (void)osMessageQueueDelete(ctx->event_queue);
        return MULT_UART_ERR_IO;
    }
    if (zdt_turntable_device_init_with_submit(
            &ctx->zdt,
            modules_zdt_submit,
            &ctx->zdt_transport,
            &zdt_config) != ZDT_TURNTABLE_OK) {
        debug_uart1_deinit(&ctx->debug);
        (void)osMessageQueueDelete(ctx->event_queue);
        return MULT_UART_ERR_IO;
    }
    lsc16_test_action_guard_init(&ctx->grasp.guard);
    if (lsc16_device_set_report_callback(
            modules_lsc16_report, ctx) != LSC16_OK) {
        debug_uart1_deinit(&ctx->debug);
        (void)osMessageQueueDelete(ctx->event_queue);
        return MULT_UART_ERR_IO;
    }
    ctx->task = osThreadNew(modules_task_entry, ctx, &g_modules_task_attr);
    if (ctx->task == NULL) {
        (void)lsc16_device_set_report_callback(NULL, NULL);
        debug_uart1_deinit(&ctx->debug);
        (void)osMessageQueueDelete(ctx->event_queue);
        return MULT_UART_ERR_IO;
    }
    ctx->initialized = true;
    return MULT_UART_OK;
}

#else

mult_uart_status_t mult_uart_modules_freertos_test_init(void)
{
    return MULT_UART_ERR_UNSUPPORTED;
}

#endif /* MULT_UART_MODULES_FREERTOS_TEST_ENABLED */
