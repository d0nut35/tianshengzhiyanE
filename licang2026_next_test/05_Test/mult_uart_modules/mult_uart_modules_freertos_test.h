/** @file mult_uart_modules_freertos_test.h @brief UART7复用模块命令式测试入口。 */

#ifndef MULT_UART_MODULES_FREERTOS_TEST_H
#define MULT_UART_MODULES_FREERTOS_TEST_H

#ifdef __cplusplus
extern "C" {
#endif

#include "mult_uart_core.h"

/**
 * @brief 创建USART1命令任务并绑定IC卡、ZDT到复用Device层。
 * @return 初始化结果。
 * @pre mult_uart_service_os和mult_uart_device必须已经初始化。
 */
mult_uart_status_t mult_uart_modules_freertos_test_init(void);

#ifdef __cplusplus
}
#endif

#endif /* MULT_UART_MODULES_FREERTOS_TEST_H */
