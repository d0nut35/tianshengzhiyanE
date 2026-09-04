/**
 * @file    mult_uart_baremetal_test.h
 * @brief   UART7复用板四通道CH340裸机验收入口。
 */

#ifndef MULT_UART_BAREMETAL_TEST_H
#define MULT_UART_BAREMETAL_TEST_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#include "mult_uart_core.h"

typedef struct {
    bool initialized;
    bool running;
    uint8_t current_channel;
    uint32_t tx_count;
    uint32_t rx_count;
    uint32_t switch_count;
    uint32_t error_count;
    mult_uart_status_t last_status;
} mult_uart_baremetal_test_status_t;

/**
 * @brief 初始化裸机验收，默认选择通道0并立即发送提示。
 * @return MULT_UART_OK表示测试链路初始化成功，否则返回具体错误。
 * @pre CubeMX已完成GPIO、DMA和UART7初始化，且RTOS Service尚未启动。
 */
mult_uart_status_t mult_uart_baremetal_test_init(void);

/**
 * @brief 在main.c裸机循环中持续调用，处理ISR事件、回显和3秒提示。
 * @note 必须被非阻塞地高频调用。
 */
void mult_uart_baremetal_test_process(void);

/**
 * @brief 读取调试快照，方便在Keil Watch中查看。
 * @param status 输出测试状态对象。
 * @return MULT_UART_OK表示复制成功，否则返回参数或未初始化错误。
 */
mult_uart_status_t mult_uart_baremetal_test_get_status(
    mult_uart_baremetal_test_status_t *status);

#ifdef __cplusplus
}
#endif

#endif /* MULT_UART_BAREMETAL_TEST_H */
