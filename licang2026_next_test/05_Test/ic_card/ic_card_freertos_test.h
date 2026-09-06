/**
 * @file    ic_card_freertos_test.h
 * @brief   IC卡UART7直连与USART1电脑触发的FreeRTOS测试入口。
 */

#ifndef IC_CARD_FREERTOS_TEST_H
#define IC_CARD_FREERTOS_TEST_H

#include "ic_bsp.h"

/**
 * @brief 创建IC卡独立FreeRTOS测试所需资源和任务。
 * @return IC_CARD_OK表示初始化成功，否则返回资源或底层装配错误。
 * @pre 在osKernelInitialize()之后、osKernelStart()之前调用。
 */
ic_card_status_t ic_card_freertos_test_init(void);

#endif /* IC_CARD_FREERTOS_TEST_H */
