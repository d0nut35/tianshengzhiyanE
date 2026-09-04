/**
 * @file    lsc16_test_config.h
 * @brief   LSC16裸机/FreeRTOS运动测试的唯一参数入口。
 *
 * 上电不会自动运动。选择测试模式后仍需通过USART1发送SERVO或ACTION，且只有
 * LSC16_TEST_MOTION_ARMED为1才会下发运动帧，防止误发电脑命令造成机械碰撞。
 */

#ifndef LSC16_TEST_CONFIG_H
#define LSC16_TEST_CONFIG_H

#include "lsc16_core.h"

#define LSC16_TEST_MOTION_ARMED                 0U

#define LSC16_TEST_SERVO_ID                     1U
#define LSC16_TEST_SERVO_POSITION               1500U
#define LSC16_TEST_SERVO_MOVE_TIME_MS           1000U

/* 用户指定200号为实机测试安全动作组；运行前必须确认已下载到当前舵控板。 */
#define LSC16_TEST_ACTION_GROUP               200U
#define LSC16_TEST_ACTION_REPEAT_COUNT          1U

/*
 * ACTION请求分别等待UART发送完成、0x06开始回报和0x08自然完成回报。
 * 任一阶段超时都会锁存运行期故障，并拒绝后续SERVO/ACTION直到F7复位。
 */
#define LSC16_TEST_ACTION_TX_TIMEOUT_MS       1000U
#define LSC16_TEST_ACTION_STARTED_TIMEOUT_MS  1000U
#define LSC16_TEST_ACTION_COMPLETE_TIMEOUT_MS 30000U

#if ((LSC16_TEST_MOTION_ARMED != 0U) && (LSC16_TEST_MOTION_ARMED != 1U))
#error "LSC16_TEST_MOTION_ARMED must be 0 or 1"
#endif

#if (LSC16_TEST_SERVO_ID > LSC16_SERVO_ID_MAX)
#error "LSC16 test servo ID must be 0..15"
#endif

#if ((LSC16_TEST_SERVO_POSITION < LSC16_SERVO_POSITION_MIN) || \
     (LSC16_TEST_SERVO_POSITION > LSC16_SERVO_POSITION_MAX))
#error "LSC16 test servo position must be 500..2500"
#endif

#if (LSC16_TEST_ACTION_REPEAT_COUNT == LSC16_REPEAT_FOREVER)
#error "Hardware test refuses an infinite action group; use a finite repeat count"
#endif

#if ((LSC16_TEST_ACTION_TX_TIMEOUT_MS == 0U) || \
     (LSC16_TEST_ACTION_STARTED_TIMEOUT_MS == 0U) || \
     (LSC16_TEST_ACTION_COMPLETE_TIMEOUT_MS == 0U))
#error "LSC16 action test timeouts must be non-zero"
#endif

#endif /* LSC16_TEST_CONFIG_H */
