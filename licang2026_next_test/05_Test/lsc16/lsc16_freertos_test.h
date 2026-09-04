/**
 * @file    lsc16_freertos_test.h
 * @brief   LSC16 Device/Service/RTOS与USART1命令式测试入口。
 */

#ifndef LSC16_FREERTOS_TEST_H
#define LSC16_FREERTOS_TEST_H

#ifdef __cplusplus
extern "C" {
#endif

#include "lsc16_core.h"

/**
 * @brief 创建LSC16与USART1命令测试所需资源和任务，上电不自动运动。
 * @return LSC16_OK表示初始化成功，否则返回资源或底层装配错误。
 * @pre 在osKernelInitialize()之后、osKernelStart()之前调用。
 */
lsc16_status_t lsc16_freertos_test_init(void);

#ifdef __cplusplus
}
#endif

#endif /* LSC16_FREERTOS_TEST_H */
