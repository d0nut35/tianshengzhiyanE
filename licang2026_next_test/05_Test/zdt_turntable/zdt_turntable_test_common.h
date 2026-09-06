/** @file zdt_turntable_test_common.h @brief ZDT直连测试命令与文本格式。 */

#ifndef ZDT_TURNTABLE_TEST_COMMON_H
#define ZDT_TURNTABLE_TEST_COMMON_H

#include <stddef.h>
#include <stdint.h>

#include "turn_bsp.h"

/** USART1调试口支持的人工测试命令。 */
typedef enum {
    ZDT_TEST_COMMAND_INVALID = 0,
    ZDT_TEST_COMMAND_VERSION,
    ZDT_TEST_COMMAND_OPTIONS,
    ZDT_TEST_COMMAND_STATUS,
    ZDT_TEST_COMMAND_POSITION,
    ZDT_TEST_COMMAND_MOVE_CW,
    ZDT_TEST_COMMAND_MOVE_CCW,
    ZDT_TEST_COMMAND_STOP,
} zdt_turntable_test_command_t;

/**
 * @brief 解析USART1文本命令，忽略末尾CR/LF、空格和ASCII大小写。
 * @param data 调试口收到的原始字节。
 * @param len 有效字节数。
 * @return 识别出的命令，无法识别时返回INVALID。
 */
zdt_turntable_test_command_t zdt_turntable_test_parse_command(
    const uint8_t *data, size_t len);

/**
 * @brief 将事务结果格式化为串口助手可读文本。
 * @param status Service完成状态。
 * @param response 成功或设备错误时的解析响应。
 * @param text 输出文本缓冲区。
 * @param capacity 输出容量。
 * @return 有效文本长度；参数错误或缓冲区不足时返回0。
 * @note ACK文本明确提示“未确认到位”，避免把命令接收误判为机械完成；
 *       OPTIONS和STATUS会输出手册中当前实现覆盖的全部标志位。
 */
size_t zdt_turntable_test_format_result(
    zdt_turntable_status_t status,
    const zdt_turntable_response_t *response,
    char *text,
    size_t capacity);

#endif /* ZDT_TURNTABLE_TEST_COMMON_H */
