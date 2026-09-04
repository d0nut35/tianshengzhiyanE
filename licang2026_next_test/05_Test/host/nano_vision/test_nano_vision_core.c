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

    assert(nano_vision_build_event_frame(
        2U, &event, frame, sizeof(frame), &frame_len) == NANO_VISION_OK);
    assert(frame_len == 22U);
    assert(memcmp(frame, event_golden, sizeof(event_golden)) == 0);
    assert(nano_vision_decode_frame(frame, frame_len, &decoded) == NANO_VISION_OK);
    assert(nano_vision_parse_event(&decoded, &parsed_event) == NANO_VISION_OK);
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
    assert(parsed_session_id == session.session_id);

    session.session_id = 0U;
    assert(nano_vision_build_session_start_frame(
        5U, &session, frame, sizeof(frame), &frame_len) ==
        NANO_VISION_ERR_VALUE);
}

int main(void)
{
    test_crc_and_codec();
    test_poll_codec();
    test_fragmented_frame();
    test_sticky_frames_and_crc_recovery();
    test_alignment_requires_consecutive_frames();
    test_timeout_disconnect_and_sequence_guard();
    test_early_candidate_accepts_fast_hold_but_keeps_guards();
    test_event_session_codec();
    puts("nano_vision_core fake tests passed");
    return 0;
}
