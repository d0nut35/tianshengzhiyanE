#include <assert.h>
#include <stdio.h>
#include <string.h>

#include "nano_vision_core.h"

typedef struct {
    nano_vision_frame_t frames[4];
    size_t count;
} capture_t;

static void capture_frame(void *user_ctx, const nano_vision_frame_t *frame)
{
    capture_t *capture = (capture_t *)user_ctx;
    assert(capture->count < 4U);
    capture->frames[capture->count++] = *frame;
}

static size_t build_observation(uint8_t sequence, uint8_t *frame)
{
    nano_vision_observation_t observation = {
        NANO_VISION_SCENE_TURNTABLE,
        NANO_VISION_OBS_VALID,
        NANO_VISION_COLOR_RED,
        91U,
        -12,
        7,
        513U,
        24U,
    };
    size_t frame_len = 0U;
    assert(nano_vision_build_observation_frame(
        sequence, &observation, frame, NANO_VISION_FRAME_MAX, &frame_len) ==
        NANO_VISION_OK);
    return frame_len;
}

static size_t refresh_frame_crc(uint8_t *frame)
{
    size_t frame_len = NANO_VISION_HEADER_SIZE + frame[5] +
                       NANO_VISION_CRC_SIZE;
    uint16_t crc = nano_vision_crc16_ccitt_false(&frame[2], 4U + frame[5]);

    frame[frame_len - 2U] = (uint8_t)(crc & 0xFFU);
    frame[frame_len - 1U] = (uint8_t)(crc >> 8U);
    return frame_len;
}

static void test_crc_and_codec(void)
{
    static const uint8_t check[] = "123456789";
    static const uint8_t golden[] = {
        0xA5U, 0x5AU, 0x01U, 0x81U, 0x42U, 0x0CU,
        0x01U, 0x01U, 0x01U, 0x5BU, 0xF4U, 0xFFU,
        0x07U, 0x00U, 0x01U, 0x02U, 0x18U, 0x00U,
        0x71U, 0xEAU,
    };
    uint8_t frame[NANO_VISION_FRAME_MAX];
    size_t frame_len;
    nano_vision_frame_t decoded;
    nano_vision_observation_t observation;

    assert(nano_vision_crc16_ccitt_false(check, 9U) == 0x29B1U);
    frame_len = build_observation(0x42U, frame);
    assert(frame_len == 20U);
    assert(memcmp(frame, golden, sizeof(golden)) == 0);
    assert(nano_vision_decode_frame(frame, frame_len, &decoded) == NANO_VISION_OK);
    assert(decoded.sequence == 0x42U);
    assert(nano_vision_parse_observation(&decoded, &observation) == NANO_VISION_OK);
    assert(observation.scene == NANO_VISION_SCENE_TURNTABLE);
    assert(observation.color == NANO_VISION_COLOR_RED);
    assert(observation.quality == 91U);
    assert(observation.offset_x_px == -12);
    assert(observation.offset_y_px == 7);
    assert(observation.frame_id == 513U);
    assert(observation.age_ms == 24U);

    frame[8] ^= 0x01U;
    assert(nano_vision_decode_frame(frame, frame_len, &decoded) == NANO_VISION_ERR_CRC);
}

static void test_poll_codec(void)
{
    static const uint8_t golden[] = {
        0xA5U, 0x5AU, 0x01U, 0x01U, 0x09U,
        0x02U, 0x02U, 0x02U, 0xD6U, 0x3AU,
    };
    nano_vision_poll_t poll = {
        NANO_VISION_SCENE_STAIR,
        NANO_VISION_COLOR_BLUE,
    };
    nano_vision_poll_t parsed;
    nano_vision_frame_t decoded;
    uint8_t frame[NANO_VISION_FRAME_MAX];
    size_t frame_len = 0U;

    assert(nano_vision_build_poll_frame(
        9U, &poll, frame, sizeof(frame), &frame_len) == NANO_VISION_OK);
    assert(frame_len == 10U);
    assert(memcmp(frame, golden, sizeof(golden)) == 0);
    assert(nano_vision_decode_frame(frame, frame_len, &decoded) == NANO_VISION_OK);
    assert(nano_vision_parse_poll(&decoded, &parsed) == NANO_VISION_OK);
    assert(parsed.scene == NANO_VISION_SCENE_STAIR);
    assert(parsed.target_color == NANO_VISION_COLOR_BLUE);
}

static void test_layered_stair_scenes(void)
{
    static const nano_vision_scene_t scenes[] = {
        NANO_VISION_SCENE_STAIR_LOW,
        NANO_VISION_SCENE_STAIR_HIGH,
        NANO_VISION_SCENE_STAIR_MID,
    };
    nano_vision_session_t session = {
        0x1234U,
        NANO_VISION_SCENE_STAIR_LOW,
        NANO_VISION_COLOR_RED,
    };
    nano_vision_session_t parsed;
    nano_vision_frame_t decoded;
    uint8_t frame[NANO_VISION_FRAME_MAX];
    size_t frame_len;
    size_t i;

    for (i = 0U; i < (sizeof(scenes) / sizeof(scenes[0])); ++i) {
        session.scene = scenes[i];
        frame_len = 0U;
        assert(nano_vision_build_session_start_frame(
            (uint8_t)(i + 1U), &session, frame, sizeof(frame), &frame_len) ==
            NANO_VISION_OK);
        assert(nano_vision_decode_frame(frame, frame_len, &decoded) ==
               NANO_VISION_OK);
        assert(nano_vision_parse_session_start(&decoded, &parsed) ==
               NANO_VISION_OK);
        assert(parsed.scene == scenes[i]);
    }
}

static void test_fragmented_frame(void)
{
    nano_vision_parser_t parser;
    capture_t capture = {0};
    uint8_t frame[NANO_VISION_FRAME_MAX];
    size_t frame_len = build_observation(1U, frame);
    size_t i;

    nano_vision_parser_init(&parser);
    for (i = 0U; i < frame_len; ++i) {
        size_t emitted = 99U;
        assert(nano_vision_parser_feed(
            &parser, &frame[i], 1U, capture_frame, &capture, &emitted) ==
            NANO_VISION_OK);
        assert(emitted == ((i + 1U == frame_len) ? 1U : 0U));
    }
    assert(capture.count == 1U);
    assert(parser.frames_ok == 1U);
}

static void test_sticky_frames_and_crc_recovery(void)
{
    nano_vision_parser_t parser;
    capture_t capture = {0};
    uint8_t first[NANO_VISION_FRAME_MAX];
    uint8_t second[NANO_VISION_FRAME_MAX];
    uint8_t sticky[NANO_VISION_FRAME_MAX * 2U];
    size_t first_len = build_observation(2U, first);
    size_t second_len = build_observation(3U, second);
    size_t emitted;

    memcpy(sticky, first, first_len);
    memcpy(&sticky[first_len], second, second_len);
    nano_vision_parser_init(&parser);
    assert(nano_vision_parser_feed(
        &parser, sticky, first_len + second_len,
        capture_frame, &capture, &emitted) == NANO_VISION_OK);
    assert(emitted == 2U);
    assert(capture.count == 2U);
    assert(capture.frames[0].sequence == 2U);
    assert(capture.frames[1].sequence == 3U);

    first[10] ^= 0x80U;
    memcpy(sticky, first, first_len);
    memcpy(&sticky[first_len], second, second_len);
    memset(&capture, 0, sizeof(capture));
    nano_vision_parser_init(&parser);
    assert(nano_vision_parser_feed(
        &parser, sticky, first_len + second_len,
        capture_frame, &capture, &emitted) == NANO_VISION_OK);
    assert(emitted == 1U);
    assert(capture.count == 1U);
    assert(capture.frames[0].sequence == 3U);
    assert(parser.crc_errors == 1U);
}

static nano_vision_tracker_config_t tracker_config(void)
{
    nano_vision_tracker_config_t config = {
        NANO_VISION_SCENE_TURNTABLE,
        NANO_VISION_COLOR_RED,
        10U,
        8U,
        100U,
        500U,
        3U,
        3U,
    };
    return config;
}

static nano_vision_observation_t aligned_observation(void)
{
    nano_vision_observation_t observation = {
        NANO_VISION_SCENE_TURNTABLE,
        NANO_VISION_OBS_VALID,
        NANO_VISION_COLOR_RED,
        90U,
        4,
        -3,
        1U,
        20U,
    };
    return observation;
}

static void test_alignment_requires_consecutive_frames(void)
{
    nano_vision_tracker_t tracker;
    nano_vision_tracker_config_t config = tracker_config();
    nano_vision_observation_t observation = aligned_observation();
    uint8_t sequence;

    assert(nano_vision_tracker_init(&tracker, &config) == NANO_VISION_OK);
    for (sequence = 1U; sequence <= 2U; ++sequence) {
        assert(nano_vision_tracker_on_poll_sent(&tracker, sequence) == NANO_VISION_OK);
        observation.frame_id = sequence;
        assert(nano_vision_tracker_on_observation(
            &tracker, &config, sequence, &observation, sequence * 20U) ==
            NANO_VISION_OK);
        assert(!nano_vision_tracker_is_aligned(&tracker, &config));
    }
    assert(nano_vision_tracker_on_poll_sent(&tracker, 3U) == NANO_VISION_OK);
    assert(nano_vision_tracker_on_observation(
        &tracker, &config, 3U, &observation, 60U) == NANO_VISION_OK);
    assert(nano_vision_tracker_is_aligned(&tracker, &config));

    observation.offset_x_px = 11;
    assert(nano_vision_tracker_on_poll_sent(&tracker, 4U) == NANO_VISION_OK);
    assert(nano_vision_tracker_on_observation(
        &tracker, &config, 4U, &observation, 80U) == NANO_VISION_OK);
    assert(!nano_vision_tracker_is_aligned(&tracker, &config));
    assert(tracker.aligned_streak == 0U);
}

static void test_timeout_disconnect_and_sequence_guard(void)
{
    nano_vision_tracker_t tracker;
    nano_vision_tracker_config_t config = tracker_config();
    nano_vision_observation_t observation = aligned_observation();

    assert(nano_vision_tracker_init(&tracker, &config) == NANO_VISION_OK);
    assert(nano_vision_tracker_on_poll_sent(&tracker, 7U) == NANO_VISION_OK);
    assert(nano_vision_tracker_on_observation(
        &tracker, &config, 6U, &observation, 100U) == NANO_VISION_ERR_SEQUENCE);
    assert(tracker.sequence_errors == 1U);
    assert(nano_vision_tracker_on_observation(
        &tracker, &config, 7U, &observation, 100U) == NANO_VISION_OK);
    assert(tracker.online);

    assert(nano_vision_tracker_on_poll_sent(&tracker, 8U) == NANO_VISION_OK);
    nano_vision_tracker_on_timeout(&tracker, &config);
    assert(nano_vision_tracker_on_poll_sent(&tracker, 9U) == NANO_VISION_OK);
    nano_vision_tracker_on_timeout(&tracker, &config);
    assert(tracker.online);
    assert(nano_vision_tracker_on_poll_sent(&tracker, 10U) == NANO_VISION_OK);
    nano_vision_tracker_on_timeout(&tracker, &config);
    assert(!tracker.online);
    assert(tracker.timeout_count == 3U);
    assert(tracker.disconnect_count == 1U);

    assert(nano_vision_tracker_on_poll_sent(&tracker, 11U) == NANO_VISION_OK);
    assert(nano_vision_tracker_on_observation(
        &tracker, &config, 11U, &observation, 1000U) == NANO_VISION_OK);
    assert(tracker.online);
    nano_vision_tracker_tick(&tracker, &config, 1499U);
    assert(tracker.online);
    nano_vision_tracker_tick(&tracker, &config, 1500U);
    assert(!tracker.online);
    assert(tracker.disconnect_count == 2U);
}

static void test_early_candidate_accepts_fast_hold_but_keeps_guards(void)
{
    nano_vision_early_config_t config = {
        NANO_VISION_SCENE_TURNTABLE,
        NANO_VISION_COLOR_RED,
        20U,
        100U,
        -301,
        338,
        -196,
        283,
    };
    nano_vision_observation_t observation = {
        NANO_VISION_SCENE_TURNTABLE,
        NANO_VISION_OBS_MODE_NOT_READY,
        NANO_VISION_COLOR_RED,
        25U,
        120,
        234,
        10U,
        56U,
    };

    assert(nano_vision_observation_is_early_candidate(&observation, &config));
    observation.quality = 18U;
    assert(!nano_vision_observation_is_early_candidate(&observation, &config));
    observation.quality = 50U;
    observation.offset_y_px = -207;
    assert(!nano_vision_observation_is_early_candidate(&observation, &config));
    observation.offset_y_px = 10;
    observation.age_ms = 101U;
    assert(!nano_vision_observation_is_early_candidate(&observation, &config));
    observation.age_ms = 20U;
    observation.status = NANO_VISION_OBS_NO_TARGET;
    observation.color = NANO_VISION_COLOR_ANY;
    assert(!nano_vision_observation_is_early_candidate(&observation, &config));
    observation.status = NANO_VISION_OBS_VALID;
    observation.color = NANO_VISION_COLOR_RED;
    assert(nano_vision_observation_is_early_candidate(&observation, &config));
}

static void test_event_session_codec(void)
{
    static const uint8_t start_golden[] = {
        0xA5U, 0x5AU, 0x01U, 0x02U, 0x01U, 0x04U,
        0x34U, 0x12U, 0x01U, 0x01U, 0xA3U, 0x32U,
    };
    static const uint8_t event_golden[] = {
        0xA5U, 0x5AU, 0x01U, 0x83U, 0x02U, 0x0EU,
        0x34U, 0x12U, 0x01U, 0x01U, 0x01U, 0x5AU,
        0xF4U, 0xFFU, 0x07U, 0x00U, 0x01U, 0x02U,
        0x18U, 0x00U, 0x04U, 0x15U,
    };
    nano_vision_session_t session = {
        0x1234U,
        NANO_VISION_SCENE_TURNTABLE,
        NANO_VISION_COLOR_RED,
    };
    nano_vision_event_t event;
    nano_vision_event_t parsed_event;
    nano_vision_event_ack_t ack = {0x1234U, 513U};
    nano_vision_event_ack_t parsed_ack;
    nano_vision_session_t parsed_session;
    nano_vision_frame_t decoded;
    uint8_t frame[NANO_VISION_FRAME_MAX];
    size_t frame_len = 0U;
    uint16_t parsed_session_id = 0U;

    event.session_id = session.session_id;
    event.observation = aligned_observation();
    event.observation.offset_x_px = -12;
    event.observation.offset_y_px = 7;
    event.observation.frame_id = 513U;
    event.observation.age_ms = 24U;

    assert(nano_vision_build_session_start_frame(
        1U, &session, frame, sizeof(frame), &frame_len) == NANO_VISION_OK);
    assert(frame_len == sizeof(start_golden));
    assert(memcmp(frame, start_golden, sizeof(start_golden)) == 0);
    assert(nano_vision_decode_frame(frame, frame_len, &decoded) == NANO_VISION_OK);
    assert(nano_vision_parse_session_start(
        &decoded, &parsed_session) == NANO_VISION_OK);
    assert(parsed_session.session_id == session.session_id);
    assert(parsed_session.scene == session.scene);
    assert(parsed_session.target_color == session.target_color);

    assert(nano_vision_build_session_ready_frame(
        1U, &session, frame, sizeof(frame), &frame_len) == NANO_VISION_OK);
    assert(nano_vision_decode_frame(frame, frame_len, &decoded) == NANO_VISION_OK);
    assert(nano_vision_parse_session_ready(
        &decoded, &parsed_session) == NANO_VISION_OK);
    assert(nano_vision_decode_session_ready(
        frame, frame_len, &parsed_session) == NANO_VISION_OK);

    assert(nano_vision_build_event_frame(
        2U, &event, frame, sizeof(frame), &frame_len) == NANO_VISION_OK);
    assert(frame_len == 22U);
    assert(memcmp(frame, event_golden, sizeof(event_golden)) == 0);
    assert(nano_vision_decode_frame(frame, frame_len, &decoded) == NANO_VISION_OK);
    assert(nano_vision_parse_event(&decoded, &parsed_event) == NANO_VISION_OK);
    assert(nano_vision_decode_event(
        frame, frame_len, &parsed_event) == NANO_VISION_OK);
    assert(parsed_event.session_id == event.session_id);
    assert(parsed_event.observation.frame_id == event.observation.frame_id);
    assert(parsed_event.observation.color == event.observation.color);

    assert(nano_vision_build_event_ack_frame(
        3U, &ack, frame, sizeof(frame), &frame_len) == NANO_VISION_OK);
    assert(nano_vision_decode_frame(frame, frame_len, &decoded) == NANO_VISION_OK);
    assert(nano_vision_parse_event_ack(&decoded, &parsed_ack) == NANO_VISION_OK);
    assert(parsed_ack.session_id == ack.session_id);
    assert(parsed_ack.frame_id == ack.frame_id);

    assert(nano_vision_build_session_stop_frame(
        4U, session.session_id, frame, sizeof(frame), &frame_len) ==
        NANO_VISION_OK);
    assert(nano_vision_decode_frame(frame, frame_len, &decoded) == NANO_VISION_OK);
    assert(nano_vision_parse_session_stop(
        &decoded, &parsed_session_id) == NANO_VISION_OK);
    assert(parsed_session_id == session.session_id);

    assert(nano_vision_build_session_stopped_frame(
        4U, session.session_id, frame, sizeof(frame), &frame_len) ==
        NANO_VISION_OK);
    assert(nano_vision_decode_frame(frame, frame_len, &decoded) == NANO_VISION_OK);
    assert(nano_vision_parse_session_stopped(
        &decoded, &parsed_session_id) == NANO_VISION_OK);
    assert(nano_vision_decode_session_stopped(
        frame, frame_len, &parsed_session_id) == NANO_VISION_OK);
    assert(parsed_session_id == session.session_id);

    session.session_id = 0U;
    assert(nano_vision_build_session_start_frame(
        5U, &session, frame, sizeof(frame), &frame_len) ==
        NANO_VISION_ERR_VALUE);
}

static void test_warehouse_digit_codec(void)
{
    static const uint8_t digit_golden[] = {
        0xA5U, 0x5AU, 0x01U, 0x85U, 0x03U, 0x08U,
        0x34U, 0x12U, 0x02U, 0x58U, 0x01U, 0x02U,
        0x18U, 0x00U, 0x60U, 0x0FU,
    };
    nano_vision_session_t session = {
        0x1234U,
        NANO_VISION_SCENE_WAREHOUSE_DIGIT,
        NANO_VISION_COLOR_ANY,
    };
    nano_vision_session_t parsed_session;
    nano_vision_digit_event_t event = {
        0x1234U,
        2U,
        88U,
        513U,
        24U,
    };
    nano_vision_digit_event_t parsed_event;
    nano_vision_frame_t decoded;
    uint8_t frame[NANO_VISION_FRAME_MAX];
    size_t frame_len = 0U;
    uint8_t digit;

    assert(nano_vision_build_session_start_frame(
        1U, &session, frame, sizeof(frame), &frame_len) == NANO_VISION_OK);
    assert(nano_vision_decode_frame(frame, frame_len, &decoded) == NANO_VISION_OK);
    assert(nano_vision_parse_session_start(
        &decoded, &parsed_session) == NANO_VISION_OK);
    assert(parsed_session.scene == NANO_VISION_SCENE_WAREHOUSE_DIGIT);
    assert(parsed_session.target_color == NANO_VISION_COLOR_ANY);

    assert(nano_vision_build_session_ready_frame(
        1U, &session, frame, sizeof(frame), &frame_len) == NANO_VISION_OK);
    assert(nano_vision_decode_session_ready(
        frame, frame_len, &parsed_session) == NANO_VISION_OK);
    assert(parsed_session.scene == NANO_VISION_SCENE_WAREHOUSE_DIGIT);
    assert(parsed_session.target_color == NANO_VISION_COLOR_ANY);

    for (digit = 1U; digit <= 3U; ++digit) {
        event.digit = digit;
        assert(nano_vision_build_digit_event_frame(
            3U, &event, frame, sizeof(frame), &frame_len) == NANO_VISION_OK);
        if (digit == 2U) {
            assert(frame_len == sizeof(digit_golden));
            assert(memcmp(frame, digit_golden, sizeof(digit_golden)) == 0);
        }
        assert(nano_vision_decode_frame(
            frame, frame_len, &decoded) == NANO_VISION_OK);
        assert(nano_vision_parse_digit_event(
            &decoded, &parsed_event) == NANO_VISION_OK);
        assert(parsed_event.session_id == event.session_id);
        assert(parsed_event.digit == digit);
        assert(parsed_event.quality == event.quality);
        assert(parsed_event.frame_id == event.frame_id);
        assert(parsed_event.age_ms == event.age_ms);
        assert(nano_vision_decode_digit_event(
            frame, frame_len, &parsed_event) == NANO_VISION_OK);
        assert(parsed_event.digit == digit);
    }

    session.target_color = NANO_VISION_COLOR_RED;
    assert(nano_vision_build_session_start_frame(
        4U, &session, frame, sizeof(frame), &frame_len) ==
        NANO_VISION_ERR_VALUE);
    session.scene = NANO_VISION_SCENE_TURNTABLE;
    session.target_color = NANO_VISION_COLOR_ANY;
    assert(nano_vision_build_session_start_frame(
        4U, &session, frame, sizeof(frame), &frame_len) ==
        NANO_VISION_ERR_VALUE);
}

static void test_warehouse_digit_errors(void)
{
    nano_vision_digit_event_t event = {
        0x1234U,
        2U,
        88U,
        513U,
        24U,
    };
    nano_vision_digit_event_t parsed_event;
    nano_vision_frame_t decoded;
    uint8_t frame[NANO_VISION_FRAME_MAX];
    size_t frame_len = 0U;

    /* 编码入口必须拒绝协议未定义的数字和超范围质量。 */
    event.digit = 0U;
    assert(nano_vision_build_digit_event_frame(
        1U, &event, frame, sizeof(frame), &frame_len) ==
        NANO_VISION_ERR_VALUE);
    event.digit = 4U;
    assert(nano_vision_build_digit_event_frame(
        1U, &event, frame, sizeof(frame), &frame_len) ==
        NANO_VISION_ERR_VALUE);
    event.digit = 2U;
    event.quality = 101U;
    assert(nano_vision_build_digit_event_frame(
        1U, &event, frame, sizeof(frame), &frame_len) ==
        NANO_VISION_ERR_VALUE);
    event.quality = 88U;

    /* 重算CRC后确认非法数字由字段校验拒绝，而不是被CRC先拦截。 */
    assert(nano_vision_build_digit_event_frame(
        1U, &event, frame, sizeof(frame), &frame_len) == NANO_VISION_OK);
    frame[8] = 0U;
    frame_len = refresh_frame_crc(frame);
    assert(nano_vision_decode_digit_event(
        frame, frame_len, &parsed_event) == NANO_VISION_ERR_VALUE);
    assert(nano_vision_decode_frame(
        frame, frame_len, &decoded) == NANO_VISION_OK);
    assert(nano_vision_parse_digit_event(
        &decoded, &parsed_event) == NANO_VISION_ERR_VALUE);

    /* 长度字段和CRC均自洽时，数字事件仍必须严格要求8字节载荷。 */
    assert(nano_vision_build_digit_event_frame(
        1U, &event, frame, sizeof(frame), &frame_len) == NANO_VISION_OK);
    frame[5] = 7U;
    frame_len = refresh_frame_crc(frame);
    assert(nano_vision_decode_digit_event(
        frame, frame_len, &parsed_event) == NANO_VISION_ERR_LENGTH);
    assert(nano_vision_decode_frame(
        frame, frame_len, &decoded) == NANO_VISION_OK);
    assert(nano_vision_parse_digit_event(
        &decoded, &parsed_event) == NANO_VISION_ERR_LENGTH);

    assert(nano_vision_build_digit_event_frame(
        1U, &event, frame, sizeof(frame), &frame_len) == NANO_VISION_OK);
    frame[5] = 9U;
    frame[14] = 0xAAU;
    frame_len = refresh_frame_crc(frame);
    assert(nano_vision_decode_digit_event(
        frame, frame_len, &parsed_event) == NANO_VISION_ERR_LENGTH);

    /* 保持原CRC后破坏载荷，验证完整帧入口能报告CRC错误。 */
    assert(nano_vision_build_digit_event_frame(
        1U, &event, frame, sizeof(frame), &frame_len) == NANO_VISION_OK);
    frame[8] ^= 0x01U;
    assert(nano_vision_decode_digit_event(
        frame, frame_len, &parsed_event) == NANO_VISION_ERR_CRC);
}

int main(void)
{
    test_crc_and_codec();
    test_poll_codec();
    test_layered_stair_scenes();
    test_fragmented_frame();
    test_sticky_frames_and_crc_recovery();
    test_alignment_requires_consecutive_frames();
    test_timeout_disconnect_and_sequence_guard();
    test_early_candidate_accepts_fast_hold_but_keeps_guards();
    test_event_session_codec();
    test_warehouse_digit_codec();
    test_warehouse_digit_errors();
    puts("nano_vision_core fake tests passed");
    return 0;
}
