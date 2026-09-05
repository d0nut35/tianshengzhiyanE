/**
 * @file    ic_card_test_common.c
 * @brief   IC卡测试命令解析和无printf文本格式化实现。
 */

#include "ic_card_test_common.h"

#include <string.h>

#include "ic_card_test_config.h"

/**
 * @brief 判断字节是否为允许裁剪的ASCII空白。
 * @param data 待检查字节。
 * @return 空格、制表、CR或LF返回true。
 */
static bool ic_card_test_is_space(uint8_t data)
{
    return (data == ' ') || (data == '\t') || (data == '\r') ||
           (data == '\n');
}

/**
 * @brief 将ASCII小写字母转换为大写。
 * @param data 输入字节。
 * @return 转换后的字节；非小写字母原样返回。
 */
static uint8_t ic_card_test_to_upper(uint8_t data)
{
    if ((data >= 'a') && (data <= 'z')) {
        return (uint8_t)(data - ('a' - 'A'));
    }
    return data;
}

/**
 * @brief 忽略首尾空白和ASCII大小写比较调试命令。
 * @param data 串口接收数据。
 * @param len 数据长度。
 * @param expected 期望命令文本。
 * @return 命令匹配返回true。
 */
static bool ic_card_test_command_equals(
    const uint8_t *data,
    size_t len,
    const char *expected)
{
    size_t expected_len = strlen(expected);
    size_t i;

    if (len != expected_len) {
        return false;
    }
    for (i = 0U; i < len; ++i) {
        if (ic_card_test_to_upper(data[i]) != (uint8_t)expected[i]) {
            return false;
        }
    }
    return true;
}

/**
 * @brief 判断输入是否为BALL_READY或READ触发命令。
 * @param data 串口接收数据。
 * @param len 数据长度。
 * @return 匹配任一触发命令返回true。
 */
bool ic_card_test_is_read_trigger(const uint8_t *data, size_t len)
{
    if (data == NULL) {
        return false;
    }
    while ((len > 0U) && ic_card_test_is_space(*data)) {
        ++data;
        --len;
    }
    while ((len > 0U) && ic_card_test_is_space(data[len - 1U])) {
        --len;
    }
    return ic_card_test_command_equals(
               data, len, IC_CARD_TEST_TRIGGER_COMMAND) ||
           ic_card_test_command_equals(
               data, len, IC_CARD_TEST_TRIGGER_ALIAS);
}

/**
 * @brief 向文本缓冲区追加一个字符并保持NUL结尾。
 * @param text 输出缓冲区。
 * @param capacity 缓冲区容量。
 * @param used 当前已用长度，成功时递增。
 * @param value 待追加字符。
 * @return 容量足够时返回true。
 */
static bool ic_card_test_append_char(
    char *text,
    size_t capacity,
    size_t *used,
    char value)
{
    if ((*used + 1U) >= capacity) {
        return false;
    }
    text[(*used)++] = value;
    text[*used] = '\0';
    return true;
}

/**
 * @brief 受容量限制地追加一段NUL结尾文本。
 * @param text 输出缓冲区。
 * @param capacity 缓冲区容量。
 * @param used 当前已用长度。
 * @param source 待追加文本。
 * @return 全部追加成功返回true。
 */
static bool ic_card_test_append_text(
    char *text,
    size_t capacity,
    size_t *used,
    const char *source)
{
    while (*source != '\0') {
        if (!ic_card_test_append_char(text, capacity, used, *source++)) {
            return false;
        }
    }
    return true;
}

/**
 * @brief 将一个字节格式化为两位大写十六进制文本。
 * @param text 输出缓冲区。
 * @param capacity 缓冲区容量。
 * @param used 当前已用长度。
 * @param value 待格式化字节。
 * @return 追加成功返回true。
 */
static bool ic_card_test_append_hex(
    char *text,
    size_t capacity,
    size_t *used,
    uint8_t value)
{
    static const char hex[] = "0123456789ABCDEF";
    return ic_card_test_append_char(text, capacity, used, hex[value >> 4]) &&
           ic_card_test_append_char(text, capacity, used, hex[value & 0x0FU]);
}

/**
 * @brief 格式化一次成功读球结果。
 * @param result 已解析的球结果。
 * @param text 输出文本缓冲区。
 * @param capacity 缓冲区容量。
 * @return 文本有效长度；参数错误或容量不足返回0。
 */
size_t ic_card_test_format_success(
    const ic_result_t *result,
    char *text,
    size_t capacity)
{
    size_t used = 0U;

    if ((result == NULL) || (text == NULL) || (capacity == 0U)) {
        return 0U;
    }
    text[0] = '\0';
    if (!ic_card_test_append_text(text, capacity, &used, "BALL OK CODE=0x") ||
        !ic_card_test_append_hex(text, capacity, &used, result->ball.code) ||
        !ic_card_test_append_text(text, capacity, &used, " ROW=") ||
        !ic_card_test_append_char(
            text, capacity, &used, (char)('0' + result->ball.row)) ||
        !ic_card_test_append_text(text, capacity, &used, " COL=") ||
        !ic_card_test_append_char(
            text, capacity, &used, (char)('0' + result->ball.column))) {
        return 0U;
    }
    if (!ic_card_test_append_text(text, capacity, &used, "\r\n")) {
        return 0U;
    }
    return used;
}

/**
 * @brief 把统一错误码格式化为串口助手可读文本。
 * @param status IC卡错误码。
 * @param text 输出文本缓冲区。
 * @param capacity 缓冲区容量。
 * @return 文本有效长度；容量不足返回0。
 */
size_t ic_card_test_format_error(
    ic_card_status_t status,
    char *text,
    size_t capacity)
{
    const char *reason;
    size_t used = 0U;

    if ((text == NULL) || (capacity == 0U)) {
        return 0U;
    }
    if (status == IC_CARD_ERR_TIMEOUT) {
        reason = "TIMEOUT";
    } else if (status == IC_CARD_ERR_CARD) {
        reason = "NO_CARD_OR_KEY";
    } else if (status == IC_CARD_ERR_PROTOCOL) {
        reason = "INVALID_BALL_DATA";
    } else if (status == IC_CARD_ERR_BUSY) {
        reason = "BUSY";
    } else if (status == IC_CARD_ERR_QUEUE_FULL) {
        reason = "QUEUE_FULL";
    } else {
        reason = "IO_OR_INTERNAL";
    }
    text[0] = '\0';
    if (!ic_card_test_append_text(text, capacity, &used, "BALL ERROR ") ||
        !ic_card_test_append_text(text, capacity, &used, reason) ||
        !ic_card_test_append_text(text, capacity, &used, " STATUS=") ||
        !ic_card_test_append_hex(text, capacity, &used, (uint8_t)status) ||
        !ic_card_test_append_text(text, capacity, &used, "\r\n")) {
        return 0U;
    }
    return used;
}
