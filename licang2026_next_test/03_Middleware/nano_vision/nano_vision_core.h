/**
 * @file    nano_vision_core.h
 * @brief   Jetson Nano与F7之间的视觉轮询/会话事件协议Core。
 *
 * 本层不包含UART、DMA、HAL或FreeRTOS。F7必须由上层通过
 * mult_uart_device_submit()访问UART7通道0；Nano只有在F7建立会话后才允许
 * 上报事件，不能控制UART7复用板。流式解析器允许任意分包或粘包数据。
 */

#ifndef NANO_VISION_CORE_H
#define NANO_VISION_CORE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define NANO_VISION_SOF_0                    0xA5U
#define NANO_VISION_SOF_1                    0x5AU
#define NANO_VISION_PROTOCOL_VERSION         0x01U
#define NANO_VISION_HEADER_SIZE              6U
#define NANO_VISION_CRC_SIZE                 2U
#define NANO_VISION_PAYLOAD_MAX             24U
#define NANO_VISION_FRAME_MAX               32U
#define NANO_VISION_POLL_PAYLOAD_SIZE        2U
#define NANO_VISION_OBSERVATION_PAYLOAD_SIZE 12U
#define NANO_VISION_SESSION_PAYLOAD_SIZE     4U
#define NANO_VISION_SESSION_ID_PAYLOAD_SIZE  2U
#define NANO_VISION_EVENT_ACK_PAYLOAD_SIZE   4U
#define NANO_VISION_EVENT_PAYLOAD_SIZE       14U

typedef enum {
    NANO_VISION_OK = 0,
    NANO_VISION_ERR_PARAM,
    NANO_VISION_ERR_LENGTH,
    NANO_VISION_ERR_VERSION,
    NANO_VISION_ERR_TYPE,
    NANO_VISION_ERR_CRC,
    NANO_VISION_ERR_VALUE,
    NANO_VISION_ERR_SEQUENCE,
    NANO_VISION_ERR_TIMEOUT,
    NANO_VISION_ERR_BUSY,
    NANO_VISION_ERR_QUEUE_FULL,
    NANO_VISION_ERR_IO,
} nano_vision_status_t;

typedef enum {
    NANO_VISION_MSG_POLL        = 0x01U,
    NANO_VISION_MSG_SESSION_START = 0x02U,
    NANO_VISION_MSG_SESSION_STOP  = 0x03U,
    NANO_VISION_MSG_EVENT_ACK     = 0x04U,
    NANO_VISION_MSG_OBSERVATION = 0x81U,
    NANO_VISION_MSG_SESSION_READY = 0x82U,
    NANO_VISION_MSG_EVENT         = 0x83U,
    NANO_VISION_MSG_SESSION_STOPPED = 0x84U,
} nano_vision_message_type_t;

typedef enum {
    NANO_VISION_SCENE_NONE      = 0U,
    NANO_VISION_SCENE_TURNTABLE = 1U,
    NANO_VISION_SCENE_STAIR     = 2U,
} nano_vision_scene_t;

typedef enum {
    NANO_VISION_COLOR_ANY  = 0U,
    NANO_VISION_COLOR_RED  = 1U,
    NANO_VISION_COLOR_BLUE = 2U,
} nano_vision_color_t;

typedef enum {
    NANO_VISION_OBS_NO_TARGET      = 0U,
    NANO_VISION_OBS_VALID          = 1U,
    NANO_VISION_OBS_CAMERA_ERROR   = 2U,
    NANO_VISION_OBS_MODE_NOT_READY = 3U,
} nano_vision_observation_status_t;

typedef struct {
    nano_vision_scene_t scene;
    nano_vision_color_t target_color;
} nano_vision_poll_t;

typedef struct {
    uint16_t session_id;
    nano_vision_scene_t scene;
    nano_vision_color_t target_color;
} nano_vision_session_t;

typedef struct {
    uint16_t session_id;
    uint16_t frame_id;
} nano_vision_event_ack_t;

/**
 * @brief Nano输出的一次视觉观测。
 * @note offset_x_px为正表示球心在抓取中心右侧，offset_y_px为正表示下侧。
 *       quality范围0~100；age_ms是生成响应时观测结果距当前时刻的年龄。
 */
typedef struct {
    nano_vision_scene_t scene;
    nano_vision_observation_status_t status;
    nano_vision_color_t color;
    uint8_t quality;
    int16_t offset_x_px;
    int16_t offset_y_px;
    uint16_t frame_id;
    uint16_t age_ms;
} nano_vision_observation_t;

typedef struct {
    uint16_t session_id;
    nano_vision_observation_t observation;
} nano_vision_event_t;

typedef struct {
    uint8_t version;
    nano_vision_message_type_t type;
    uint8_t sequence;
    uint8_t payload[NANO_VISION_PAYLOAD_MAX];
    uint8_t payload_len;
} nano_vision_frame_t;

typedef void (*nano_vision_frame_fn_t)(
    void *user_ctx,
    const nano_vision_frame_t *frame);

typedef struct {
    uint8_t buffer[NANO_VISION_FRAME_MAX];
    uint8_t count;
    uint8_t expected_size;
    uint32_t frames_ok;
    uint32_t crc_errors;
    uint32_t format_errors;
    uint32_t discarded_bytes;
} nano_vision_parser_t;

typedef struct {
    nano_vision_scene_t scene;
    nano_vision_color_t target_color;
    uint16_t tolerance_x_px;
    uint16_t tolerance_y_px;
    uint16_t max_observation_age_ms;
    uint16_t link_timeout_ms;
    uint8_t required_confirm_frames;
    uint8_t disconnect_after_timeouts;
} nano_vision_tracker_config_t;

/** 高速场景单帧提前候选窗口；只给上层决策提供证据，不直接触发机械动作。 */
typedef struct {
    nano_vision_scene_t scene;
    nano_vision_color_t target_color;
    uint8_t min_quality;
    uint16_t max_observation_age_ms;
    int16_t min_offset_x_px;
    int16_t max_offset_x_px;
    int16_t min_offset_y_px;
    int16_t max_offset_y_px;
} nano_vision_early_config_t;

/** F7侧链路和连续对齐确认状态，不产生任何机械动作。 */
typedef struct {
    bool online;
    bool waiting_response;
    bool has_last_sequence;
    uint8_t expected_sequence;
    uint8_t last_sequence;
    uint8_t aligned_streak;
    uint8_t consecutive_timeouts;
    uint32_t last_rx_ms;
    uint32_t observations;
    uint32_t timeout_count;
    uint32_t disconnect_count;
    uint32_t sequence_errors;
    nano_vision_observation_t last_observation;
} nano_vision_tracker_t;

uint16_t nano_vision_crc16_ccitt_false(const uint8_t *data, size_t len);

nano_vision_status_t nano_vision_build_poll_frame(
    uint8_t sequence,
    const nano_vision_poll_t *poll,
    uint8_t *frame,
    size_t capacity,
    size_t *frame_len);

nano_vision_status_t nano_vision_build_observation_frame(
    uint8_t sequence,
    const nano_vision_observation_t *observation,
    uint8_t *frame,
    size_t capacity,
    size_t *frame_len);

nano_vision_status_t nano_vision_build_session_start_frame(
    uint8_t sequence,
    const nano_vision_session_t *session,
    uint8_t *frame,
    size_t capacity,
    size_t *frame_len);

nano_vision_status_t nano_vision_build_session_ready_frame(
    uint8_t sequence,
    const nano_vision_session_t *session,
    uint8_t *frame,
    size_t capacity,
    size_t *frame_len);

nano_vision_status_t nano_vision_build_session_stop_frame(
    uint8_t sequence,
    uint16_t session_id,
    uint8_t *frame,
    size_t capacity,
    size_t *frame_len);

nano_vision_status_t nano_vision_build_session_stopped_frame(
    uint8_t sequence,
    uint16_t session_id,
    uint8_t *frame,
    size_t capacity,
    size_t *frame_len);

nano_vision_status_t nano_vision_build_event_frame(
    uint8_t sequence,
    const nano_vision_event_t *event,
    uint8_t *frame,
    size_t capacity,
    size_t *frame_len);

nano_vision_status_t nano_vision_build_event_ack_frame(
    uint8_t sequence,
    const nano_vision_event_ack_t *ack,
    uint8_t *frame,
    size_t capacity,
    size_t *frame_len);

nano_vision_status_t nano_vision_decode_frame(
    const uint8_t *data,
    size_t len,
    nano_vision_frame_t *frame);

nano_vision_status_t nano_vision_parse_poll(
    const nano_vision_frame_t *frame,
    nano_vision_poll_t *poll);

nano_vision_status_t nano_vision_parse_observation(
    const nano_vision_frame_t *frame,
    nano_vision_observation_t *observation);

nano_vision_status_t nano_vision_parse_session_start(
    const nano_vision_frame_t *frame,
    nano_vision_session_t *session);

nano_vision_status_t nano_vision_parse_session_ready(
    const nano_vision_frame_t *frame,
    nano_vision_session_t *session);

nano_vision_status_t nano_vision_parse_session_stop(
    const nano_vision_frame_t *frame,
    uint16_t *session_id);

nano_vision_status_t nano_vision_parse_session_stopped(
    const nano_vision_frame_t *frame,
    uint16_t *session_id);

nano_vision_status_t nano_vision_parse_event(
    const nano_vision_frame_t *frame,
    nano_vision_event_t *event);

nano_vision_status_t nano_vision_parse_event_ack(
    const nano_vision_frame_t *frame,
    nano_vision_event_ack_t *ack);

void nano_vision_parser_init(nano_vision_parser_t *parser);

nano_vision_status_t nano_vision_parser_feed(
    nano_vision_parser_t *parser,
    const uint8_t *data,
    size_t len,
    nano_vision_frame_fn_t frame_cb,
    void *user_ctx,
    size_t *emitted_frames);

nano_vision_status_t nano_vision_tracker_init(
    nano_vision_tracker_t *tracker,
    const nano_vision_tracker_config_t *config);

nano_vision_status_t nano_vision_tracker_on_poll_sent(
    nano_vision_tracker_t *tracker,
    uint8_t sequence);

nano_vision_status_t nano_vision_tracker_on_observation(
    nano_vision_tracker_t *tracker,
    const nano_vision_tracker_config_t *config,
    uint8_t sequence,
    const nano_vision_observation_t *observation,
    uint32_t now_ms);

void nano_vision_tracker_on_timeout(
    nano_vision_tracker_t *tracker,
    const nano_vision_tracker_config_t *config);

void nano_vision_tracker_tick(
    nano_vision_tracker_t *tracker,
    const nano_vision_tracker_config_t *config,
    uint32_t now_ms);

bool nano_vision_tracker_is_aligned(
    const nano_vision_tracker_t *tracker,
    const nano_vision_tracker_config_t *config);

bool nano_vision_observation_is_early_candidate(
    const nano_vision_observation_t *observation,
    const nano_vision_early_config_t *config);

#ifdef __cplusplus
}
#endif

#endif /* NANO_VISION_CORE_H */
