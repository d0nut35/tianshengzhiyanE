/**
 * @file    mult_uart_freertos_test.h
 * @brief   UART7复用板四通道CH340 FreeRTOS验收入口。
 */

#ifndef MULT_UART_FREERTOS_TEST_H
#define MULT_UART_FREERTOS_TEST_H

#ifdef __cplusplus
extern "C" {
#endif

#include "mult_uart_core.h"

/**
 * @brief 创建测试事件队列和测试任务。
 * @pre mult_uart_service_os_init()和mult_uart_device_init()已成功；
 *      当前在osKernelInitialize()之后、osKernelStart()之前。
 */
mult_uart_status_t mult_uart_freertos_test_init(void);

#ifdef __cplusplus
}
#endif

#endif /* MULT_UART_FREERTOS_TEST_H */
