/**
 * @file    ic_card_test_common.h
 * @brief   裸机和FreeRTOS IC卡测试共用的命令识别与结果格式化。
 */

#ifndef IC_CARD_TEST_COMMON_H
#define IC_CARD_TEST_COMMON_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "ic_card_service.h"

/**
 * @brief 判断电脑输入是否为读球触发命令。
 * @param data 输入字节。
 * @param len 输入长度。
 * @return 忽略首尾空白和ASCII大小写后匹配BALL_READY或READ时返回true。
 */
bool ic_card_test_is_read_trigger(const uint8_t *data, size_t len);

/**
 * @brief 把成功读球结果格式化为纯ASCII文本。
 * @param result 比赛球解析结果。
 * @param text 输出字符缓冲区。
 * @param capacity 输出缓冲区容量。
 * @return 实际字符数；容量不足或参数无效时返回0。
 */
size_t ic_card_test_format_success(
    const ic_result_t *result,
    char *text,
    size_t capacity);

/**
 * @brief 把统一错误码格式化为纯ASCII文本。
 * @param status IC卡统一状态码。
 * @param text 输出字符缓冲区。
 * @param capacity 输出缓冲区容量。
 * @return 实际字符数；容量不足或参数无效时返回0。
 */
size_t ic_card_test_format_error(
    ic_card_status_t status,
    char *text,
    size_t capacity);

#ifdef __cplusplus
}
#endif

#endif /* IC_CARD_TEST_COMMON_H */
