/**
 * @file    test_zdt_turntable_core.c
 * @brief   ZDT平台无关Core的PC构帧与响应解析测试。
 */

#include <stdio.h>
#include <string.h>

#include "turn_bsp.h"

static int failures;
/** 记录失败但继续执行同一进程中的其余断言。 */
#define CHECK(x) do { if (!(x)) { printf("FAIL line %d: %s\n", __LINE__, #x); failures++; } } while (0)

/** @brief 验证X固件0xFD位置帧的字段顺序和大端编码。 */
static void test_x_position(void)
{
    uint8_t frame[ZDT_TURNTABLE_FRAME_MAX];
    const uint8_t expected[] = {
        0x01, 0xFD, 0x01, 0x01, 0xFF, 0x01, 0xFA, 0x27,
        0x10, 0x00, 0x00, 0x8C, 0xA0, 0x00, 0x00, 0x6B
    };
    size_t len = 0U;
    zdt_turntable_position_command_t cmd = {
        ZDT_TURNTABLE_DIR_CCW,
        ZDT_TURNTABLE_POS_RELATIVE_LAST_TARGET,
        10000U, 511U, 506U, 36000U, 0U
    };
    CHECK(turn_x_frame(
        1U, &cmd, frame, sizeof(frame), &len) == ZDT_TURNTABLE_OK);
    CHECK(len == sizeof(expected));
    CHECK(memcmp(frame, expected, sizeof(expected)) == 0);
}

/** @brief 验证Emm固件0xFD位置帧及脉冲字段编码。 */
static void test_emm_position(void)
{
    uint8_t frame[ZDT_TURNTABLE_FRAME_MAX];
    const uint8_t expected[] = {
        0x01, 0xFD, 0x01, 0x05, 0xDC, 0x00, 0x00,
        0x00, 0x7D, 0x00, 0x00, 0x00, 0x6B
    };
    size_t len = 0U;
    zdt_turntable_position_command_t cmd = {
        ZDT_TURNTABLE_DIR_CCW,
        ZDT_TURNTABLE_POS_RELATIVE_LAST_TARGET,
        1500U, 0U, 0U, 36000U, 0U
    };
    CHECK(turn_emm_frame(
        1U, &cmd, 32000U, frame, sizeof(frame), &len) == ZDT_TURNTABLE_OK);
    CHECK(len == sizeof(expected));
    CHECK(memcmp(frame, expected, sizeof(expected)) == 0);
}

/** @brief 验证手册允许的绝对零位置不会被本地参数校验拒绝。 */
static void test_zero_position(void)
{
    uint8_t frame[ZDT_TURNTABLE_FRAME_MAX];
    size_t len = 0U;
    zdt_turntable_position_command_t cmd = {
        ZDT_TURNTABLE_DIR_CW,
        ZDT_TURNTABLE_POS_ABSOLUTE_ZERO,
        60U, 0U, 0U, 0U, 50U
    };

    CHECK(turn_emm_frame(
        1U, &cmd, 0U, frame, sizeof(frame), &len) == ZDT_TURNTABLE_OK);
    CHECK(len == 13U);
    CHECK(frame[6] == 0U && frame[7] == 0U &&
          frame[8] == 0U && frame[9] == 0U);
    CHECK(frame[10] == ZDT_TURNTABLE_POS_ABSOLUTE_ZERO);

    CHECK(turn_x_frame(
        1U, &cmd, frame, sizeof(frame), &len) == ZDT_TURNTABLE_OK);
    CHECK(len == 16U);
    CHECK(frame[9] == 0U && frame[10] == 0U &&
          frame[11] == 0U && frame[12] == 0U);
}

/** @brief 验证选项、状态、位置、版本和通用ACK的响应分派。 */
static void test_responses(void)
{
    zdt_turntable_response_t response;
    const uint8_t options_all_fields[] = {0x01, 0x1A, 0x03, 0xB7, 0x6B};
    const uint8_t options_real_emm[] = {0x01, 0x1A, 0x00, 0x06, 0x6B};
    const uint8_t options_legacy[] = {0x01, 0x1A, 0x06, 0x6B};
    const uint8_t options_error[] = {0x01, 0x1A, 0xE2, 0x6B};
    const uint8_t status[] = {0x01, 0x3A, 0xBB, 0x6B};
    const uint8_t status_error[] = {0x01, 0x3A, 0xEE, 0x6B};
    const uint8_t status_reserved[] = {0x01, 0x3A, 0x40, 0x6B};
    const uint8_t position[] = {0x01, 0x36, 0x01, 0x00, 0x00, 0x00, 0x10, 0x6B};
    const uint8_t version[] = {0x01, 0x1F, 0x00, 0xC8, 0x03, 0x14, 0x6B};
    const uint8_t invalid_version_ack[] = {0x01, 0x1F, 0x02, 0x6B};
    const uint8_t ack[] = {0x01, 0xFD, 0x02, 0x6B};

    CHECK(turn_parse(
        options_all_fields, sizeof(options_all_fields), 1U, 0x1AU,
        &response) == ZDT_TURNTABLE_OK);
    CHECK(response.kind == ZDT_TURNTABLE_REPLY_OPTIONS);
    CHECK(response.data.options.raw_flags == 0x03B7U);
    CHECK(response.data.options.motor_step_angle_0p9deg);
    CHECK(response.data.options.firmware == ZDT_TURNTABLE_FIRMWARE_EMM);
    CHECK(response.data.options.closed_loop);
    CHECK(response.data.options.positive_direction_ccw);
    CHECK(response.data.options.buttons_locked);
    CHECK(response.data.options.scaled_input);
    CHECK(response.data.options.parameter_lock_level == 3U);

    /* 覆盖2.01实机已观测到的原始帧01 1A 00 06 6B。 */
    CHECK(turn_parse(
        options_real_emm, sizeof(options_real_emm), 1U, 0x1AU,
        &response) == ZDT_TURNTABLE_OK);
    CHECK(response.data.options.raw_flags == 0x0006U);
    CHECK(!response.data.options.motor_step_angle_0p9deg);
    CHECK(response.data.options.firmware == ZDT_TURNTABLE_FIRMWARE_EMM);
    CHECK(response.data.options.closed_loop);
    CHECK(!response.data.options.scaled_input);

    CHECK(turn_parse(
        options_legacy, sizeof(options_legacy), 1U, 0x1AU,
        &response) == ZDT_TURNTABLE_ERR_PROTOCOL);
    CHECK(turn_parse(
        options_error, sizeof(options_error), 1U, 0x1AU,
        &response) == ZDT_TURNTABLE_ERR_DEVICE);
    CHECK(response.kind == ZDT_TURNTABLE_REPLY_PARAM_ERROR);

    CHECK(turn_parse(status, sizeof(status), 1U, 0x3AU,
        &response) == ZDT_TURNTABLE_OK);
    CHECK(response.data.motor_status.enabled);
    CHECK(response.data.motor_status.reached);
    CHECK(response.data.motor_status.stall_protected);
    CHECK(response.data.motor_status.left_limit_high);
    CHECK(response.data.motor_status.right_limit_high);
    CHECK(response.data.motor_status.power_loss_latched);
    CHECK(turn_parse(
        status_error, sizeof(status_error), 1U, 0x3AU,
        &response) == ZDT_TURNTABLE_ERR_DEVICE);
    CHECK(response.kind == ZDT_TURNTABLE_REPLY_FORMAT_ERROR);
    CHECK(turn_parse(
        status_reserved, sizeof(status_reserved), 1U, 0x3AU,
        &response) == ZDT_TURNTABLE_ERR_PROTOCOL);

    CHECK(turn_parse(position, sizeof(position), 1U, 0x36U,
        &response) == ZDT_TURNTABLE_OK);
    CHECK(response.data.position.negative);
    CHECK(response.data.position.magnitude == 16U);

    CHECK(turn_parse(version, sizeof(version), 1U, 0x1FU,
        &response) == ZDT_TURNTABLE_OK);
    CHECK(response.data.version.firmware_version == 200U);
    CHECK(response.data.version.hardware_type == 3U);
    CHECK(turn_parse(
        invalid_version_ack, sizeof(invalid_version_ack), 1U, 0x1FU,
        &response) == ZDT_TURNTABLE_ERR_PROTOCOL);

    CHECK(turn_parse(ack, sizeof(ack), 1U, 0xFDU,
        &response) == ZDT_TURNTABLE_OK);
    CHECK(response.kind == ZDT_TURNTABLE_REPLY_ACK);
}

/** @brief 运行全部ZDT Core PC测试。 */
int main(void)
{
    test_x_position();
    test_emm_position();
    test_zero_position();
    test_responses();
    if (failures != 0) return 1;
    puts("zdt_turntable_core tests passed");
    return 0;
}
