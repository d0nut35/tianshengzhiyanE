/** @file zdt_turntable_test_common.c @brief ZDT测试公共逻辑实现。 */

#include "zdt_turntable_test_common.h"

#include <stdio.h>
#include <string.h>

/**
 * @brief 比较一条调试口命令，忽略末尾CR/LF/空格和ASCII大小写。
 * @param data 输入字节。
 * @param len 输入长度。
 * @param expected 预期的大写零结尾命令。
 * @return 完整匹配时返回true。
 */
static bool zdt_command_equals(
    const uint8_t *data, size_t len, const char *expected)
{
    size_t i;
    size_t expected_len;

    while ((len > 0U) && ((data[len - 1U] == '\r') ||
                          (data[len - 1U] == '\n') ||
                          (data[len - 1U] == ' '))) {
        len--;
    }
    expected_len = strlen(expected);
    if (len != expected_len) {
        return false;
    }
    for (i = 0U; i < len; ++i) {
        uint8_t ch = data[i];
        if ((ch >= 'a') && (ch <= 'z')) {
            ch = (uint8_t)(ch - ('a' - 'A'));
        }
        if (ch != (uint8_t)expected[i]) {
            return false;
        }
    }
    return true;
}

/** @copydoc zdt_turntable_test_parse_command() */
zdt_turntable_test_command_t zdt_turntable_test_parse_command(
    const uint8_t *data, size_t len)
{
    if ((data == NULL) || (len == 0U)) return ZDT_TEST_COMMAND_INVALID;
    if (zdt_command_equals(data, len, "VERSION")) return ZDT_TEST_COMMAND_VERSION;
    if (zdt_command_equals(data, len, "OPTIONS")) return ZDT_TEST_COMMAND_OPTIONS;
    if (zdt_command_equals(data, len, "STATUS")) return ZDT_TEST_COMMAND_STATUS;
    if (zdt_command_equals(data, len, "POSITION")) return ZDT_TEST_COMMAND_POSITION;
    if (zdt_command_equals(data, len, "MOVE_CW")) return ZDT_TEST_COMMAND_MOVE_CW;
    if (zdt_command_equals(data, len, "MOVE_CCW")) return ZDT_TEST_COMMAND_MOVE_CCW;
    if (zdt_command_equals(data, len, "STOP")) return ZDT_TEST_COMMAND_STOP;
    return ZDT_TEST_COMMAND_INVALID;
}

/** @copydoc zdt_turntable_test_format_result() */
size_t zdt_turntable_test_format_result(
    zdt_turntable_status_t status,
    const zdt_turntable_response_t *response,
    char *text,
    size_t capacity)
{
    int count;
    if ((text == NULL) || (capacity == 0U)) return 0U;
    if ((status != ZDT_TURNTABLE_OK) &&
        (status != ZDT_TURNTABLE_ERR_DEVICE)) {
        count = snprintf(text, capacity, "ZDT ERROR %u\r\n", (unsigned)status);
    } else if (response == NULL) {
        count = snprintf(text, capacity, "ZDT ERROR NO RESPONSE\r\n");
    } else if (response->kind == ZDT_TURNTABLE_REPLY_OPTIONS) {
        count = snprintf(text, capacity,
            "OPTIONS RAW=%04X MOTOR=%s FW=%s CLOSED=%u DIR=%s KEYLOCK=%u SCALE10=%u PARAMLOCK=%u\r\n",
            response->data.options.raw_flags,
            response->data.options.motor_step_angle_0p9deg ? "0.9" : "1.8",
            response->data.options.firmware == ZDT_TURNTABLE_FIRMWARE_X ? "X" : "EMM",
            response->data.options.closed_loop ? 1U : 0U,
            response->data.options.positive_direction_ccw ? "CCW" : "CW",
            response->data.options.buttons_locked ? 1U : 0U,
            response->data.options.scaled_input ? 1U : 0U,
            response->data.options.parameter_lock_level);
    } else if (response->kind == ZDT_TURNTABLE_REPLY_STATUS) {
        count = snprintf(text, capacity,
            "STATUS ENABLE=%u REACHED=%u STALL=%u PROTECT=%u LIMIT_L=%u LIMIT_R=%u POWERLOSS_LATCHED=%u\r\n",
            response->data.motor_status.enabled ? 1U : 0U,
            response->data.motor_status.reached ? 1U : 0U,
            response->data.motor_status.stalled ? 1U : 0U,
            response->data.motor_status.stall_protected ? 1U : 0U,
            response->data.motor_status.left_limit_high ? 1U : 0U,
            response->data.motor_status.right_limit_high ? 1U : 0U,
            response->data.motor_status.power_loss_latched ? 1U : 0U);
    } else if (response->kind == ZDT_TURNTABLE_REPLY_POSITION) {
        count = snprintf(text, capacity, "POSITION SIGN=%c RAW=%lu\r\n",
            response->data.position.negative ? '-' : '+',
            (unsigned long)response->data.position.magnitude);
    } else if (response->kind == ZDT_TURNTABLE_REPLY_VERSION) {
        count = snprintf(text, capacity,
            "VERSION FW=%u HW_SERIES=%u HW_TYPE=%u HW_VER=%u\r\n",
            response->data.version.firmware_version,
            response->data.version.hardware_series,
            response->data.version.hardware_type,
            response->data.version.hardware_version);
    } else if (response->kind == ZDT_TURNTABLE_REPLY_ACK) {
        count = snprintf(text, capacity,
            "ACK RECEIVED. NOT POSITION REACHED\r\n");
    } else if (response->kind == ZDT_TURNTABLE_REPLY_REACHED) {
        count = snprintf(text, capacity, "POSITION REACHED 0x9F\r\n");
    } else if (response->kind == ZDT_TURNTABLE_REPLY_PARAM_ERROR) {
        count = snprintf(text, capacity, "DEVICE PARAM/CONDITION ERROR E2\r\n");
    } else if (response->kind == ZDT_TURNTABLE_REPLY_FORMAT_ERROR) {
        count = snprintf(text, capacity, "DEVICE FORMAT ERROR EE\r\n");
    } else {
        count = snprintf(text, capacity, "DEVICE REPLY KIND=%u\r\n",
            (unsigned)response->kind);
    }
    if ((count <= 0) || ((size_t)count >= capacity)) return 0U;
    return (size_t)count;
}
