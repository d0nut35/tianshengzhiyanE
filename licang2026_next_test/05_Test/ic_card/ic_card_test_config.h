/**
 * @file    ic_card_test_config.h
 * @brief   IC卡直连测试的协议参数与电脑命令约定。
 */

#ifndef IC_CARD_TEST_CONFIG_H
#define IC_CARD_TEST_CONFIG_H

#define IC_CARD_TEST_TRIGGER_COMMAND       "BALL_READY"
#define IC_CARD_TEST_TRIGGER_ALIAS         "READ"
/* 厂家手册及STM32示例规定：A3第2参数1=开启本次操作的蜂鸣/灯提示。 */
#define IC_CARD_TEST_LED_BEEP_PROMPT       1U
#define IC_CARD_TEST_STARTUP_DELAY_MS      5200U

#endif /* IC_CARD_TEST_CONFIG_H */
