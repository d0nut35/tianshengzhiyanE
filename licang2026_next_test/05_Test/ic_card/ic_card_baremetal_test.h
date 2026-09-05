/**
 * @file    ic_card_baremetal_test.h
 * @brief   UART7直连读卡器、USART1连接电脑的裸机比赛流程测试。
 */

#ifndef IC_CARD_BAREMETAL_TEST_H
#define IC_CARD_BAREMETAL_TEST_H

#include "ic_bsp.h"

/**
 * @brief 初始化UART7直连IC卡裸机测试。
 * @return IC_CARD_OK表示测试链路初始化成功，否则返回具体错误。
 */
ic_card_status_t ic_card_baremetal_test_init(void);

/**
 * @brief 在裸机主循环推进电脑触发、读卡事务和结果回传。
 * @note 必须被非阻塞地高频调用。
 */
void ic_card_baremetal_test_process(void);

#endif /* IC_CARD_BAREMETAL_TEST_H */
