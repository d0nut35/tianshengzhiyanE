/**
 * @file    mult_uart_test_protocol.h
 * @brief   UART复用板四通道验收测试的平台无关命令解析与文本格式化。
 *
 * 这些命令只是临时测试协议，不属于任何正式外设协议，因此必须留在05_Test。
 */

#ifndef MULT_UART_TEST_PROTOCOL_H
#define MULT_UART_TEST_PROTOCOL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stddef.h>
#include <stdint.h>

#define MULT_UART_TEST_CHANNEL_COUNT             4U
#define MULT_UART_TEST_HEARTBEAT_MS               3000U
#define MULT_UART_TEST_RX_CAPACITY                32U
#define MULT_UART_TEST_TX_CAPACITY                128U

typedef enum {
    MULT_UART_TEST_INPUT_ECHO = 0,
    MULT_UART_TEST_INPUT_SWITCH,
    MULT_UART_TEST_INPUT_INVALID_CHANNEL,
} mult_uart_test_input_kind_t;

typedef struct {
    mult_uart_test_input_kind_t kind;
    uint8_t requested_channel;
} mult_uart_test_input_t;

/**
 * @brief 解析一次ReceiveToIdle交付的数据块。
 *
 * 只有数据长度恰好为3，且格式为FF xx AA时才视为切换命令。
 * xx为00~03时返回SWITCH，其他值返回INVALID_CHANNEL。
 * 所有其他数据均视为普通数据，以纯文本HEX格式回显。
 * @param data ReceiveToIdle交付的数据块。
 * @param len 数据长度。
 * @return 解析后的输入类型和目标通道。
 */
mult_uart_test_input_t mult_uart_test_parse_input(
    const uint8_t *data,
    size_t len);

/**
 * @brief 生成当前通道提示文本。
 * @param channel 当前通道序号。
 * @param output 输出字节缓冲区。
 * @param capacity 输出缓冲区容量。
 * @return 生成的字节数；容量不足或参数非法时返回0。
 */
size_t mult_uart_test_format_announcement(
    uint8_t channel,
    uint8_t *output,
    size_t capacity);

/**
 * @brief 生成"channel N:XX YY\r\n"。
 * @param channel 当前通道序号。
 * @param data 待回显原始字节。
 * @param len 原始字节长度。
 * @param output 输出字节缓冲区。
 * @param capacity 输出缓冲区容量。
 * @return 生成的字节数；容量不足或参数非法时返回0。
 * @note 输入字节统一转换为两位大写HEX，字节之间用空格分隔。
 */
size_t mult_uart_test_format_echo(
    uint8_t channel,
    const uint8_t *data,
    size_t len,
    uint8_t *output,
    size_t capacity);

/**
 * @brief 生成非法通道提示文本。
 * @param output 输出字节缓冲区。
 * @param capacity 输出缓冲区容量。
 * @return 生成的字节数；容量不足或参数非法时返回0。
 */
size_t mult_uart_test_format_invalid_channel(
    uint8_t *output,
    size_t capacity);

#ifdef __cplusplus
}
#endif

#endif /* MULT_UART_TEST_PROTOCOL_H */
