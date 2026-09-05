/** @file nano_vision_core.c @brief Nano视觉协议与F7侧链路判定实现。 */

#include "nano_vision_core.h"

#include <string.h>

static bool nano_vision_scene_is_valid(nano_vision_scene_t scene)
{
    return (scene == NANO_VISION_SCENE_TURNTABLE) ||
           (scene == NANO_VISION_SCENE_STAIR_LOW) ||
           (scene == NANO_VISION_SCENE_STAIR_HIGH) ||
           (scene == NANO_VISION_SCENE_STAIR_MID);
}

static bool nano_vision_color_is_valid(nano_vision_color_t color)
{
    return (color == NANO_VISION_COLOR_ANY) ||
           (color == NANO_VISION_COLOR_RED) ||
           (color == NANO_VISION_COLOR_BLUE);
}

static bool nano_vision_session_is_valid(
    const nano_vision_session_t *session)
{
    return (session != NULL) && (session->session_id != 0U) &&
           nano_vision_scene_is_valid(session->scene) &&
           ((session->target_color == NANO_VISION_COLOR_RED) ||
            (session->target_color == NANO_VISION_COLOR_BLUE));
}

static bool nano_vision_observation_is_valid_value(
    const nano_vision_observation_t *observation)
{
    return (observation != NULL) &&
           nano_vision_scene_is_valid(observation->scene) &&
           nano_vision_color_is_valid(observation->color) &&
           ((uint32_t)observation->status <=
            (uint32_t)NANO_VISION_OBS_MODE_NOT_READY) &&
           (observation->quality <= 100U) &&
           ((observation->status != NANO_VISION_OBS_VALID) ||
            (observation->color != NANO_VISION_COLOR_ANY));
}

static void nano_vision_write_u16_le(uint8_t *data, uint16_t value)
{
    data[0] = (uint8_t)(value & 0xFFU);
    data[1] = (uint8_t)(value >> 8U);
}

static uint16_t nano_vision_read_u16_le(const uint8_t *data)
{
    return (uint16_t)((uint16_t)data[0] | ((uint16_t)data[1] << 8U));
}

uint16_t nano_vision_crc16_ccitt_false(const uint8_t *data, size_t len)
{
    uint16_t crc = 0xFFFFU;
    size_t i;
    uint8_t bit;

    if ((data == NULL) && (len != 0U)) return 0U;
    for (i = 0U; i < len; ++i) {
        crc ^= (uint16_t)((uint16_t)data[i] << 8U);
        for (bit = 0U; bit < 8U; ++bit) {
            crc = ((crc & 0x8000U) != 0U) ?
                (uint16_t)((crc << 1U) ^ 0x1021U) :
                (uint16_t)(crc << 1U);
        }
    }
    return crc;
}

static nano_vision_status_t nano_vision_build_frame(
    nano_vision_message_type_t type,
    uint8_t sequence,
    const uint8_t *payload,
    uint8_t payload_len,
    uint8_t *frame,
    size_t capacity,
    size_t *frame_len)
{
    size_t total;
    uint16_t crc;

    if ((payload == NULL) || (frame == NULL) || (frame_len == NULL)) {
        return NANO_VISION_ERR_PARAM;
    }
    if (payload_len > NANO_VISION_PAYLOAD_MAX) return NANO_VISION_ERR_LENGTH;
    total = NANO_VISION_HEADER_SIZE + (size_t)payload_len + NANO_VISION_CRC_SIZE;
    if (capacity < total) return NANO_VISION_ERR_LENGTH;

    frame[0] = NANO_VISION_SOF_0;
    frame[1] = NANO_VISION_SOF_1;
    frame[2] = NANO_VISION_PROTOCOL_VERSION;
    frame[3] = (uint8_t)type;
    frame[4] = sequence;
    frame[5] = payload_len;
    memcpy(&frame[NANO_VISION_HEADER_SIZE], payload, payload_len);
    crc = nano_vision_crc16_ccitt_false(&frame[2], 4U + payload_len);
    nano_vision_write_u16_le(&frame[NANO_VISION_HEADER_SIZE + payload_len], crc);
    *frame_len = total;
    return NANO_VISION_OK;
}

nano_vision_status_t nano_vision_build_poll_frame(
    uint8_t sequence,
    const nano_vision_poll_t *poll,
    uint8_t *frame,
    size_t capacity,
    size_t *frame_len)
{
    uint8_t payload[NANO_VISION_POLL_PAYLOAD_SIZE];

    if ((poll == NULL) || !nano_vision_scene_is_valid(poll->scene) ||
        !nano_vision_color_is_valid(poll->target_color)) {
        return NANO_VISION_ERR_VALUE;
    }
    payload[0] = (uint8_t)poll->scene;
    payload[1] = (uint8_t)poll->target_color;
    return nano_vision_build_frame(
        NANO_VISION_MSG_POLL, sequence, payload, sizeof(payload),
        frame, capacity, frame_len);
}

nano_vision_status_t nano_vision_build_observation_frame(
    uint8_t sequence,
    const nano_vision_observation_t *observation,
    uint8_t *frame,
    size_t capacity,
    size_t *frame_len)
{
    uint8_t payload[NANO_VISION_OBSERVATION_PAYLOAD_SIZE];

    if (!nano_vision_observation_is_valid_value(observation)) {
        return NANO_VISION_ERR_VALUE;
    }

    payload[0] = (uint8_t)observation->scene;
    payload[1] = (uint8_t)observation->status;
    payload[2] = (uint8_t)observation->color;
    payload[3] = observation->quality;
    nano_vision_write_u16_le(&payload[4], (uint16_t)observation->offset_x_px);
    nano_vision_write_u16_le(&payload[6], (uint16_t)observation->offset_y_px);
    nano_vision_write_u16_le(&payload[8], observation->frame_id);
    nano_vision_write_u16_le(&payload[10], observation->age_ms);
    return nano_vision_build_frame(
        NANO_VISION_MSG_OBSERVATION, sequence, payload, sizeof(payload),
        frame, capacity, frame_len);
}

static nano_vision_status_t nano_vision_build_session_frame(
    nano_vision_message_type_t type,
    uint8_t sequence,
    const nano_vision_session_t *session,
    uint8_t *frame,
    size_t capacity,
    size_t *frame_len)
{
    uint8_t payload[NANO_VISION_SESSION_PAYLOAD_SIZE];

    if (!nano_vision_session_is_valid(session)) {
        return NANO_VISION_ERR_VALUE;
    }
    nano_vision_write_u16_le(payload, session->session_id);
    payload[2] = (uint8_t)session->scene;
    payload[3] = (uint8_t)session->target_color;
    return nano_vision_build_frame(
        type, sequence, payload, sizeof(payload), frame, capacity, frame_len);
}

nano_vision_status_t nano_vision_build_session_start_frame(
    uint8_t sequence,
    const nano_vision_session_t *session,
    uint8_t *frame,
    size_t capacity,
    size_t *frame_len)
{
    return nano_vision_build_session_frame(
        NANO_VISION_MSG_SESSION_START, sequence, session,
        frame, capacity, frame_len);
}

nano_vision_status_t nano_vision_build_session_ready_frame(
    uint8_t sequence,
    const nano_vision_session_t *session,
    uint8_t *frame,
    size_t capacity,
    size_t *frame_len)
{
    return nano_vision_build_session_frame(
        NANO_VISION_MSG_SESSION_READY, sequence, session,
        frame, capacity, frame_len);
}

static nano_vision_status_t nano_vision_build_session_id_frame(
    nano_vision_message_type_t type,
    uint8_t sequence,
    uint16_t session_id,
    uint8_t *frame,
    size_t capacity,
    size_t *frame_len)
{
    uint8_t payload[NANO_VISION_SESSION_ID_PAYLOAD_SIZE];

    if (session_id == 0U) return NANO_VISION_ERR_VALUE;
    nano_vision_write_u16_le(payload, session_id);
    return nano_vision_build_frame(
        type, sequence, payload, sizeof(payload), frame, capacity, frame_len);
}

nano_vision_status_t nano_vision_build_session_stop_frame(
    uint8_t sequence,
    uint16_t session_id,
    uint8_t *frame,
    size_t capacity,
    size_t *frame_len)
{
    return nano_vision_build_session_id_frame(
        NANO_VISION_MSG_SESSION_STOP, sequence, session_id,
        frame, capacity, frame_len);
}

nano_vision_status_t nano_vision_build_session_stopped_frame(
    uint8_t sequence,
    uint16_t session_id,
    uint8_t *frame,
    size_t capacity,
    size_t *frame_len)
{
    return nano_vision_build_session_id_frame(
        NANO_VISION_MSG_SESSION_STOPPED, sequence, session_id,
        frame, capacity, frame_len);
}

nano_vision_status_t nano_vision_build_event_frame(
    uint8_t sequence,
    const nano_vision_event_t *event,
    uint8_t *frame,
    size_t capacity,
    size_t *frame_len)
{
    uint8_t payload[NANO_VISION_EVENT_PAYLOAD_SIZE];
    const nano_vision_observation_t *observation;

    if ((event == NULL) || (event->session_id == 0U) ||
        !nano_vision_observation_is_valid_value(&event->observation)) {
        return NANO_VISION_ERR_VALUE;
    }
    observation = &event->observation;
    nano_vision_write_u16_le(payload, event->session_id);
    payload[2] = (uint8_t)observation->scene;
    payload[3] = (uint8_t)observation->status;
    payload[4] = (uint8_t)observation->color;
    payload[5] = observation->quality;
    nano_vision_write_u16_le(&payload[6],
        (uint16_t)observation->offset_x_px);
    nano_vision_write_u16_le(&payload[8],
        (uint16_t)observation->offset_y_px);
    nano_vision_write_u16_le(&payload[10], observation->frame_id);
    nano_vision_write_u16_le(&payload[12], observation->age_ms);
    return nano_vision_build_frame(
        NANO_VISION_MSG_EVENT, sequence, payload, sizeof(payload),
        frame, capacity, frame_len);
}

nano_vision_status_t nano_vision_build_event_ack_frame(
    uint8_t sequence,
    const nano_vision_event_ack_t *ack,
    uint8_t *frame,
    size_t capacity,
    size_t *frame_len)
{
    uint8_t payload[NANO_VISION_EVENT_ACK_PAYLOAD_SIZE];

    if ((ack == NULL) || (ack->session_id == 0U)) {
        return NANO_VISION_ERR_VALUE;
    }
    nano_vision_write_u16_le(payload, ack->session_id);
    nano_vision_write_u16_le(&payload[2], ack->frame_id);
    return nano_vision_build_frame(
        NANO_VISION_MSG_EVENT_ACK, sequence, payload, sizeof(payload),
        frame, capacity, frame_len);
}

/** 校验完整帧头、长度和CRC；不复制载荷。 */
static nano_vision_status_t nano_vision_validate_frame(
    const uint8_t *data,
    size_t len)
{
    size_t expected;
    uint16_t actual_crc;
    uint16_t expected_crc;

    if (data == NULL) return NANO_VISION_ERR_PARAM;
    if (len < (NANO_VISION_HEADER_SIZE + NANO_VISION_CRC_SIZE)) {
        return NANO_VISION_ERR_LENGTH;
    }
    if ((data[0] != NANO_VISION_SOF_0) || (data[1] != NANO_VISION_SOF_1)) {
        return NANO_VISION_ERR_VALUE;
    }
    if (data[2] != NANO_VISION_PROTOCOL_VERSION) return NANO_VISION_ERR_VERSION;
    if (data[5] > NANO_VISION_PAYLOAD_MAX) return NANO_VISION_ERR_LENGTH;
    expected = NANO_VISION_HEADER_SIZE + (size_t)data[5] + NANO_VISION_CRC_SIZE;
    if (len != expected) return NANO_VISION_ERR_LENGTH;

    actual_crc = nano_vision_crc16_ccitt_false(&data[2], 4U + data[5]);
    expected_crc = nano_vision_read_u16_le(&data[NANO_VISION_HEADER_SIZE + data[5]]);
    if (actual_crc != expected_crc) return NANO_VISION_ERR_CRC;

    return NANO_VISION_OK;
}

nano_vision_status_t nano_vision_decode_frame(
    const uint8_t *data,
    size_t len,
    nano_vision_frame_t *frame)
{
    nano_vision_status_t status;

    if (frame == NULL) return NANO_VISION_ERR_PARAM;
    status = nano_vision_validate_frame(data, len);
    if (status != NANO_VISION_OK) return status;

    frame->version = data[2];
    frame->type = (nano_vision_message_type_t)data[3];
    frame->sequence = data[4];
    frame->payload_len = data[5];
    memcpy(frame->payload, &data[NANO_VISION_HEADER_SIZE], frame->payload_len);
    return NANO_VISION_OK;
}

/** 校验正式会话帧的通用头和期望消息类型、载荷长度。 */
static nano_vision_status_t nano_vision_validate_message(
    const uint8_t *data,
    size_t len,
    nano_vision_message_type_t expected_type,
    uint8_t expected_payload_len)
{
    nano_vision_status_t status = nano_vision_validate_frame(data, len);

    if (status != NANO_VISION_OK) return status;
    if (data[3] != (uint8_t)expected_type) return NANO_VISION_ERR_TYPE;
    return (data[5] == expected_payload_len) ?
        NANO_VISION_OK : NANO_VISION_ERR_LENGTH;
}

nano_vision_status_t nano_vision_decode_session_ready(
    const uint8_t *data,
    size_t len,
    nano_vision_session_t *session)
{
    nano_vision_status_t status;

    if (session == NULL) return NANO_VISION_ERR_PARAM;
    status = nano_vision_validate_message(
        data, len, NANO_VISION_MSG_SESSION_READY,
        NANO_VISION_SESSION_PAYLOAD_SIZE);
    if (status != NANO_VISION_OK) return status;
    session->session_id = nano_vision_read_u16_le(&data[6]);
    session->scene = (nano_vision_scene_t)data[8];
    session->target_color = (nano_vision_color_t)data[9];
    return nano_vision_session_is_valid(session) ?
        NANO_VISION_OK : NANO_VISION_ERR_VALUE;
}

nano_vision_status_t nano_vision_decode_session_stopped(
    const uint8_t *data,
    size_t len,
    uint16_t *session_id)
{
    nano_vision_status_t status;

    if (session_id == NULL) return NANO_VISION_ERR_PARAM;
    status = nano_vision_validate_message(
        data, len, NANO_VISION_MSG_SESSION_STOPPED,
        NANO_VISION_SESSION_ID_PAYLOAD_SIZE);
    if (status != NANO_VISION_OK) return status;
    *session_id = nano_vision_read_u16_le(&data[6]);
    return (*session_id != 0U) ? NANO_VISION_OK : NANO_VISION_ERR_VALUE;
}

nano_vision_status_t nano_vision_decode_event(
    const uint8_t *data,
    size_t len,
    nano_vision_event_t *event)
{
    nano_vision_observation_t *observation;
    nano_vision_status_t status;

    if (event == NULL) return NANO_VISION_ERR_PARAM;
    status = nano_vision_validate_message(
        data, len, NANO_VISION_MSG_EVENT, NANO_VISION_EVENT_PAYLOAD_SIZE);
    if (status != NANO_VISION_OK) return status;
    event->session_id = nano_vision_read_u16_le(&data[6]);
    observation = &event->observation;
    observation->scene = (nano_vision_scene_t)data[8];
    observation->status = (nano_vision_observation_status_t)data[9];
    observation->color = (nano_vision_color_t)data[10];
    observation->quality = data[11];
    observation->offset_x_px = (int16_t)nano_vision_read_u16_le(&data[12]);
    observation->offset_y_px = (int16_t)nano_vision_read_u16_le(&data[14]);
    observation->frame_id = nano_vision_read_u16_le(&data[16]);
    observation->age_ms = nano_vision_read_u16_le(&data[18]);
    if ((event->session_id == 0U) ||
        !nano_vision_observation_is_valid_value(observation)) {
        return NANO_VISION_ERR_VALUE;
    }
    return NANO_VISION_OK;
}

nano_vision_status_t nano_vision_parse_poll(
    const nano_vision_frame_t *frame,
    nano_vision_poll_t *poll)
{
    if ((frame == NULL) || (poll == NULL)) return NANO_VISION_ERR_PARAM;
    if (frame->type != NANO_VISION_MSG_POLL) return NANO_VISION_ERR_TYPE;
    if (frame->payload_len != NANO_VISION_POLL_PAYLOAD_SIZE) {
        return NANO_VISION_ERR_LENGTH;
    }
    poll->scene = (nano_vision_scene_t)frame->payload[0];
    poll->target_color = (nano_vision_color_t)frame->payload[1];
    if (!nano_vision_scene_is_valid(poll->scene) ||
        !nano_vision_color_is_valid(poll->target_color)) {
        return NANO_VISION_ERR_VALUE;
    }
    return NANO_VISION_OK;
}

nano_vision_status_t nano_vision_parse_observation(
    const nano_vision_frame_t *frame,
    nano_vision_observation_t *observation)
{
    if ((frame == NULL) || (observation == NULL)) return NANO_VISION_ERR_PARAM;
    if (frame->type != NANO_VISION_MSG_OBSERVATION) return NANO_VISION_ERR_TYPE;
    if (frame->payload_len != NANO_VISION_OBSERVATION_PAYLOAD_SIZE) {
        return NANO_VISION_ERR_LENGTH;
    }
    observation->scene = (nano_vision_scene_t)frame->payload[0];
    observation->status = (nano_vision_observation_status_t)frame->payload[1];
    observation->color = (nano_vision_color_t)frame->payload[2];
    observation->quality = frame->payload[3];
    observation->offset_x_px = (int16_t)nano_vision_read_u16_le(&frame->payload[4]);
    observation->offset_y_px = (int16_t)nano_vision_read_u16_le(&frame->payload[6]);
    observation->frame_id = nano_vision_read_u16_le(&frame->payload[8]);
    observation->age_ms = nano_vision_read_u16_le(&frame->payload[10]);
    if (!nano_vision_observation_is_valid_value(observation)) {
        return NANO_VISION_ERR_VALUE;
    }
    return NANO_VISION_OK;
}

static nano_vision_status_t nano_vision_parse_session_frame(
    const nano_vision_frame_t *frame,
    nano_vision_message_type_t expected_type,
    nano_vision_session_t *session)
{
    if ((frame == NULL) || (session == NULL)) return NANO_VISION_ERR_PARAM;
    if (frame->type != expected_type) return NANO_VISION_ERR_TYPE;
    if (frame->payload_len != NANO_VISION_SESSION_PAYLOAD_SIZE) {
        return NANO_VISION_ERR_LENGTH;
    }
    session->session_id = nano_vision_read_u16_le(frame->payload);
    session->scene = (nano_vision_scene_t)frame->payload[2];
    session->target_color = (nano_vision_color_t)frame->payload[3];
    return nano_vision_session_is_valid(session) ?
        NANO_VISION_OK : NANO_VISION_ERR_VALUE;
}

nano_vision_status_t nano_vision_parse_session_start(
    const nano_vision_frame_t *frame,
    nano_vision_session_t *session)
{
    return nano_vision_parse_session_frame(
        frame, NANO_VISION_MSG_SESSION_START, session);
}

nano_vision_status_t nano_vision_parse_session_ready(
    const nano_vision_frame_t *frame,
    nano_vision_session_t *session)
{
    return nano_vision_parse_session_frame(
        frame, NANO_VISION_MSG_SESSION_READY, session);
}

static nano_vision_status_t nano_vision_parse_session_id_frame(
    const nano_vision_frame_t *frame,
    nano_vision_message_type_t expected_type,
    uint16_t *session_id)
{
    if ((frame == NULL) || (session_id == NULL)) {
        return NANO_VISION_ERR_PARAM;
    }
    if (frame->type != expected_type) return NANO_VISION_ERR_TYPE;
    if (frame->payload_len != NANO_VISION_SESSION_ID_PAYLOAD_SIZE) {
        return NANO_VISION_ERR_LENGTH;
    }
    *session_id = nano_vision_read_u16_le(frame->payload);
    return (*session_id != 0U) ? NANO_VISION_OK : NANO_VISION_ERR_VALUE;
}

nano_vision_status_t nano_vision_parse_session_stop(
    const nano_vision_frame_t *frame,
    uint16_t *session_id)
{
    return nano_vision_parse_session_id_frame(
        frame, NANO_VISION_MSG_SESSION_STOP, session_id);
}

nano_vision_status_t nano_vision_parse_session_stopped(
    const nano_vision_frame_t *frame,
    uint16_t *session_id)
{
    return nano_vision_parse_session_id_frame(
        frame, NANO_VISION_MSG_SESSION_STOPPED, session_id);
}

nano_vision_status_t nano_vision_parse_event(
    const nano_vision_frame_t *frame,
    nano_vision_event_t *event)
{
    nano_vision_observation_t *observation;

    if ((frame == NULL) || (event == NULL)) return NANO_VISION_ERR_PARAM;
    if (frame->type != NANO_VISION_MSG_EVENT) return NANO_VISION_ERR_TYPE;
    if (frame->payload_len != NANO_VISION_EVENT_PAYLOAD_SIZE) {
        return NANO_VISION_ERR_LENGTH;
    }
    event->session_id = nano_vision_read_u16_le(frame->payload);
    observation = &event->observation;
    observation->scene = (nano_vision_scene_t)frame->payload[2];
    observation->status =
        (nano_vision_observation_status_t)frame->payload[3];
    observation->color = (nano_vision_color_t)frame->payload[4];
    observation->quality = frame->payload[5];
    observation->offset_x_px =
        (int16_t)nano_vision_read_u16_le(&frame->payload[6]);
    observation->offset_y_px =
        (int16_t)nano_vision_read_u16_le(&frame->payload[8]);
    observation->frame_id =
        nano_vision_read_u16_le(&frame->payload[10]);
    observation->age_ms =
        nano_vision_read_u16_le(&frame->payload[12]);
    if ((event->session_id == 0U) ||
        !nano_vision_observation_is_valid_value(observation)) {
        return NANO_VISION_ERR_VALUE;
    }
    return NANO_VISION_OK;
}

nano_vision_status_t nano_vision_parse_event_ack(
    const nano_vision_frame_t *frame,
    nano_vision_event_ack_t *ack)
{
    if ((frame == NULL) || (ack == NULL)) return NANO_VISION_ERR_PARAM;
    if (frame->type != NANO_VISION_MSG_EVENT_ACK) {
        return NANO_VISION_ERR_TYPE;
    }
    if (frame->payload_len != NANO_VISION_EVENT_ACK_PAYLOAD_SIZE) {
        return NANO_VISION_ERR_LENGTH;
    }
    ack->session_id = nano_vision_read_u16_le(frame->payload);
    ack->frame_id = nano_vision_read_u16_le(&frame->payload[2]);
    return (ack->session_id != 0U) ?
        NANO_VISION_OK : NANO_VISION_ERR_VALUE;
}

void nano_vision_parser_init(nano_vision_parser_t *parser)
{
    if (parser != NULL) memset(parser, 0, sizeof(*parser));
}

static void nano_vision_parser_reset(nano_vision_parser_t *parser)
{
    parser->count = 0U;
    parser->expected_size = 0U;
}

nano_vision_status_t nano_vision_parser_feed(
    nano_vision_parser_t *parser,
    const uint8_t *data,
    size_t len,
    nano_vision_frame_fn_t frame_cb,
    void *user_ctx,
    size_t *emitted_frames)
{
    size_t i;
    size_t emitted = 0U;

    if ((parser == NULL) || ((data == NULL) && (len != 0U))) {
        return NANO_VISION_ERR_PARAM;
    }
    for (i = 0U; i < len; ++i) {
        uint8_t byte = data[i];

        if (parser->count == 0U) {
            if (byte == NANO_VISION_SOF_0) parser->buffer[parser->count++] = byte;
            else ++parser->discarded_bytes;
            continue;
        }
        if (parser->count == 1U) {
            if (byte == NANO_VISION_SOF_1) parser->buffer[parser->count++] = byte;
            else if (byte == NANO_VISION_SOF_0) parser->buffer[0] = byte;
            else {
                ++parser->discarded_bytes;
                nano_vision_parser_reset(parser);
            }
            continue;
        }

        parser->buffer[parser->count++] = byte;
        if (parser->count == NANO_VISION_HEADER_SIZE) {
            if ((parser->buffer[2] != NANO_VISION_PROTOCOL_VERSION) ||
                (parser->buffer[5] > NANO_VISION_PAYLOAD_MAX)) {
                ++parser->format_errors;
                nano_vision_parser_reset(parser);
                continue;
            }
            parser->expected_size = (uint8_t)(
                NANO_VISION_HEADER_SIZE + parser->buffer[5] + NANO_VISION_CRC_SIZE);
        }
        if ((parser->expected_size != 0U) &&
            (parser->count == parser->expected_size)) {
            nano_vision_frame_t frame;
            nano_vision_status_t status = nano_vision_decode_frame(
                parser->buffer, parser->count, &frame);
            if (status == NANO_VISION_OK) {
                ++parser->frames_ok;
                ++emitted;
                if (frame_cb != NULL) frame_cb(user_ctx, &frame);
            } else if (status == NANO_VISION_ERR_CRC) {
                ++parser->crc_errors;
            } else {
                ++parser->format_errors;
            }
            nano_vision_parser_reset(parser);
        }
    }
    if (emitted_frames != NULL) *emitted_frames = emitted;
    return NANO_VISION_OK;
}

static bool nano_vision_tracker_config_is_valid(
    const nano_vision_tracker_config_t *config)
{
    return (config != NULL) && nano_vision_scene_is_valid(config->scene) &&
           nano_vision_color_is_valid(config->target_color) &&
           (config->target_color != NANO_VISION_COLOR_ANY) &&
           (config->required_confirm_frames > 0U) &&
           (config->disconnect_after_timeouts > 0U) &&
           (config->link_timeout_ms > 0U);
}

nano_vision_status_t nano_vision_tracker_init(
    nano_vision_tracker_t *tracker,
    const nano_vision_tracker_config_t *config)
{
    if ((tracker == NULL) || !nano_vision_tracker_config_is_valid(config)) {
        return NANO_VISION_ERR_PARAM;
    }
    memset(tracker, 0, sizeof(*tracker));
    return NANO_VISION_OK;
}

nano_vision_status_t nano_vision_tracker_on_poll_sent(
    nano_vision_tracker_t *tracker,
    uint8_t sequence)
{
    if (tracker == NULL) return NANO_VISION_ERR_PARAM;
    if (tracker->waiting_response) return NANO_VISION_ERR_SEQUENCE;
    tracker->expected_sequence = sequence;
    tracker->waiting_response = true;
    return NANO_VISION_OK;
}

static uint16_t nano_vision_abs_i16(int16_t value)
{
    int32_t wide = value;
    if (wide < 0) wide = -wide;
    return (uint16_t)wide;
}

nano_vision_status_t nano_vision_tracker_on_observation(
    nano_vision_tracker_t *tracker,
    const nano_vision_tracker_config_t *config,
    uint8_t sequence,
    const nano_vision_observation_t *observation,
    uint32_t now_ms)
{
    bool aligned_sample;

    if ((tracker == NULL) || (observation == NULL) ||
        !nano_vision_tracker_config_is_valid(config)) {
        return NANO_VISION_ERR_PARAM;
    }
    if (!tracker->waiting_response || (sequence != tracker->expected_sequence) ||
        (tracker->has_last_sequence && (sequence == tracker->last_sequence))) {
        ++tracker->sequence_errors;
        return NANO_VISION_ERR_SEQUENCE;
    }

    tracker->waiting_response = false;
    tracker->has_last_sequence = true;
    tracker->last_sequence = sequence;
    tracker->online = true;
    tracker->last_rx_ms = now_ms;
    tracker->consecutive_timeouts = 0U;
    tracker->last_observation = *observation;
    ++tracker->observations;

    aligned_sample =
        (observation->scene == config->scene) &&
        (observation->status == NANO_VISION_OBS_VALID) &&
        (observation->color == config->target_color) &&
        (observation->age_ms <= config->max_observation_age_ms) &&
        (nano_vision_abs_i16(observation->offset_x_px) <= config->tolerance_x_px) &&
        (nano_vision_abs_i16(observation->offset_y_px) <= config->tolerance_y_px);
    if (aligned_sample) {
        if (tracker->aligned_streak < config->required_confirm_frames) {
            ++tracker->aligned_streak;
        }
    } else {
        tracker->aligned_streak = 0U;
    }
    return NANO_VISION_OK;
}

void nano_vision_tracker_on_timeout(
    nano_vision_tracker_t *tracker,
    const nano_vision_tracker_config_t *config)
{
    if ((tracker == NULL) || !nano_vision_tracker_config_is_valid(config)) return;
    if (!tracker->waiting_response) return;
    tracker->waiting_response = false;
    ++tracker->timeout_count;
    if (tracker->consecutive_timeouts < 0xFFU) ++tracker->consecutive_timeouts;
    tracker->aligned_streak = 0U;
    if (tracker->consecutive_timeouts >= config->disconnect_after_timeouts) {
        if (tracker->online) ++tracker->disconnect_count;
        tracker->online = false;
    }
}

void nano_vision_tracker_tick(
    nano_vision_tracker_t *tracker,
    const nano_vision_tracker_config_t *config,
    uint32_t now_ms)
{
    if ((tracker == NULL) || !nano_vision_tracker_config_is_valid(config) ||
        !tracker->online) {
        return;
    }
    if ((uint32_t)(now_ms - tracker->last_rx_ms) >= config->link_timeout_ms) {
        tracker->online = false;
        tracker->aligned_streak = 0U;
        ++tracker->disconnect_count;
    }
}

bool nano_vision_tracker_is_aligned(
    const nano_vision_tracker_t *tracker,
    const nano_vision_tracker_config_t *config)
{
    if ((tracker == NULL) || !nano_vision_tracker_config_is_valid(config)) {
        return false;
    }
    return tracker->online &&
           (tracker->aligned_streak >= config->required_confirm_frames);
}

bool nano_vision_observation_is_early_candidate(
    const nano_vision_observation_t *observation,
    const nano_vision_early_config_t *config)
{
    bool detectable_status;

    if ((observation == NULL) || (config == NULL) ||
        !nano_vision_scene_is_valid(config->scene) ||
        !nano_vision_color_is_valid(config->target_color) ||
        (config->target_color == NANO_VISION_COLOR_ANY) ||
        (config->min_quality > 100U) ||
        (config->min_offset_x_px > config->max_offset_x_px) ||
        (config->min_offset_y_px > config->max_offset_y_px)) {
        return false;
    }

    /* 高速运动允许稳定VALID或仍在跟踪建立期的MODE_NOT_READY进入候选。 */
    detectable_status =
        (observation->status == NANO_VISION_OBS_VALID) ||
        (observation->status == NANO_VISION_OBS_MODE_NOT_READY);
    return detectable_status &&
           (observation->scene == config->scene) &&
           (observation->color == config->target_color) &&
           (observation->quality >= config->min_quality) &&
           (observation->age_ms <= config->max_observation_age_ms) &&
           (observation->offset_x_px >= config->min_offset_x_px) &&
           (observation->offset_x_px <= config->max_offset_x_px) &&
           (observation->offset_y_px >= config->min_offset_y_px) &&
           (observation->offset_y_px <= config->max_offset_y_px);
}
