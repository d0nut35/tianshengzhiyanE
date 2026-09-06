/** @file zdt_turntable_freertos_test.h @brief ZDT UART7直连RTOS测试入口。 */
#ifndef ZDT_TURNTABLE_FREERTOS_TEST_H
#define ZDT_TURNTABLE_FREERTOS_TEST_H
#include "turn_bsp.h"

/**
 * @brief 装配ZDT直连链路并创建唯一测试worker。
 * @return 初始化及任务创建结果；成功后不会自动发送运动命令。
 * @note 必须在内核启动前调用，任务在调度器启动后运行。
 */
zdt_turntable_status_t zdt_turntable_freertos_test_init(void);
#endif
