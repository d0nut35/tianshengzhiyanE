/** @file zdt_turntable_baremetal_test.h @brief ZDT UART7直连裸机测试入口。 */
#ifndef ZDT_TURNTABLE_BAREMETAL_TEST_H
#define ZDT_TURNTABLE_BAREMETAL_TEST_H
#include "zdt_turntable_core.h"

/**
 * @brief 装配USART1调试口、UART7直连适配器、Service和Device。
 * @return 初始化结果；成功后不会自动发送运动命令。
 */
zdt_turntable_status_t zdt_turntable_baremetal_test_init(void);

/**
 * @brief 在裸机主循环中推进ZDT事务并消费USART1人工命令。
 * @note 仅在ZDT裸机测试开关启用且init成功后调用。
 */
void zdt_turntable_baremetal_test_process(void);
#endif
