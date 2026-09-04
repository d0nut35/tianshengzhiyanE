/**
 * @file    mult_uart_test_protocol.c
 * @brief   UART复用板四通道验收测试协议实现。
 */

#include "mult_uart_test_protocol.h"

#include <string.h>

static const uint8_t g_hex_digits[] = "0123456789ABCDEF";

/**
 * @brief 受容量限制地复制一段测试文本。
 * @param text NUL结尾源文本。
 * @param output 输出字节缓冲区。
 * @param capacity 输出容量。
 * @return 复制长度，容量不足返回0。
 */
static size_t mult_uart_test_copy_text(
    const char *text,
    uint8_t *output,
    size_t capacity)
{
    size_t len;

    if ((text == NULL) || (output == NULL)) {
        return 0U;
    }

    len = strlen(text);
    if (len > capacity) {
        return 0U;
    }

    (void)memcpy(output, text, len);
    return len;
}

/**
 * @brief 解析通道切换帧或普通回显数据。
 * @param data 输入字节。
 * @param len 输入长度。
 * @return 输入类型和请求通道。
 */
mult_uart_test_input_t mult_uart_test_parse_input(
    const uint8_t *data,
    size_t len)
{
    mult_uart_test_input_t result = {
        MULT_UART_TEST_INPUT_ECHO,
        0U,
    };

    if ((data == NULL) || (len == 0U)) {
        return result;
    }

    /*
     * 本测试把一次UART IDLE分隔的数据当作一条消息。
     * 仅精确的3字节命令会切换通道，避免普通回显数据中偶然出现
     * FF xx AA子序列时误切通道。
     */
    if ((len == 3U) && (data[0] == 0xFFU) && (data[2] == 0xAAU)) {
        result.requested_channel = data[1];
        result.kind = (data[1] < MULT_UART_TEST_CHANNEL_COUNT) ?
            MULT_UART_TEST_INPUT_SWITCH :
            MULT_UART_TEST_INPUT_INVALID_CHANNEL;
    }

    return result;
}

/**
 * @brief 构造“now is channel N”纯文本提示。
 * @param channel 当前通道。
 * @param output 输出缓冲区。
 * @param capacity 输出容量。
 * @return 文本长度，参数或容量错误返回0。
 */
size_t mult_uart_test_format_announcement(
    uint8_t channel,
    uint8_t *output,
    size_t capacity)
{
    static const char prefix[] = "now is channel ";
    size_t prefix_len = sizeof(prefix) - 1U;
    size_t required = prefix_len + 3U;

    if ((output == NULL) ||
        (channel >= MULT_UART_TEST_CHANNEL_COUNT) ||
        (capacity < required)) {
        return 0U;
    }

    (void)memcpy(output, prefix, prefix_len);
    output[prefix_len] = (uint8_t)('0' + channel);
    output[prefix_len + 1U] = (uint8_t)'\r';
    output[prefix_len + 2U] = (uint8_t)'\n';
    return required;
}

/**
 * @brief 将普通输入构造成“channel N:XX YY”大写HEX回显。
 * @param channel 当前通道。
 * @param data 输入字节。
 * @param len 输入长度。
 * @param output 输出缓冲区。
 * @param capacity 输出容量。
 * @return 文本长度，容量不足返回0。
 */
size_t mult_uart_test_format_echo(
    uint8_t channel,
    const uint8_t *data,
    size_t len,
    uint8_t *output,
    size_t capacity)
{
    static const char prefix[] = "channel ";
    size_t pos = 0U;
    size_t i;
    size_t required;

    if ((output == NULL) || (data == NULL) || (len == 0U) ||
        (channel >= MULT_UART_TEST_CHANNEL_COUNT)) {
        return 0U;
    }

    /* prefix + channel + ':' + 2*HEX + (len-1)*space + CRLF */
    required = (sizeof(prefix) - 1U) + 2U + (len * 2U) +
               (len - 1U) + 2U;
    if (required > capacity) {
        return 0U;
    }

    (void)memcpy(&output[pos], prefix, sizeof(prefix) - 1U);
    pos += sizeof(prefix) - 1U;
    output[pos++] = (uint8_t)('0' + channel);
    output[pos++] = (uint8_t)':';

    for (i = 0U; i < len; ++i) {
        output[pos++] = g_hex_digits[(data[i] >> 4U) & 0x0FU];
        output[pos++] = g_hex_digits[data[i] & 0x0FU];
        if ((i + 1U) < len) {
            output[pos++] = (uint8_t)' ';
        }
    }

    output[pos++] = (uint8_t)'\r';
    output[pos++] = (uint8_t)'\n';
    return pos;
}

/**
 * @brief 构造非法通道纯文本提示。
 * @param output 输出缓冲区。
 * @param capacity 输出容量。
 * @return 文本长度，容量不足返回0。
 */
size_t mult_uart_test_format_invalid_channel(
    uint8_t *output,
    size_t capacity)
{
    return mult_uart_test_copy_text(
        "invalid channel\r\n",
        output,
        capacity);
}
