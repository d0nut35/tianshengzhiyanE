/** @file mission_config.h @brief Mission初版状态机的任务与流程配置。 */

#ifndef MISSION_CONFIG_H
#define MISSION_CONFIG_H

/* 1：只运行上层单格联调；0：恢复底盘桥和正式Mission。 */
#define MISSION_AP_TEST_ENABLE             1U

#define MISSION_TASK_STACK_SIZE        3072U
#define MISSION_COMMAND_QUEUE_DEPTH       4U
#define MISSION_OPERATION_TIMEOUT_MS  30000U
#define MISSION_READY_TIMEOUT_MS       8000U

#define MISSION_HOME_ACTION_GROUP        10U
#define MISSION_PLATFORM_VISION_GROUP    11U
#define MISSION_PLATFORM_GRASP_GROUP     12U
#define MISSION_STAIR_VISION_GROUP       13U
#define MISSION_STAIR_LOW_GROUP          14U
#define MISSION_STAIR_HIGH_GROUP         15U
#define MISSION_STAIR_MID_GROUP          16U
#define MISSION_PLATFORM_RELEASE_GROUP   17U /* 圆盘第五球安全送入下一格 */

#define MISSION_PLATFORM_BALL_COUNT       5U
#define MISSION_STAIR_BALL_COUNT          2U

/* UART7复用器设备映射：Nano=通道0；IC和转盘映射收在各自Service。 */
#define MISSION_VISION_DEVICE_ID MUX_DEVICE_0

#define MISSION_VISION_TIMEOUT_MS           200U
#define MISSION_VISION_READ_TIMEOUT_MS      250U
#define MISSION_VISION_EVENT_MAX_AGE_MS     120U

#define MISSION_IC_OPERATION_PROMPT           1U
#define MISSION_IC_MAX_ATTEMPTS                5U
#define MISSION_IC_RETRY_MS                  150U

#define MISSION_ZDT_ADDRESS                    1U
#define MISSION_ZDT_IO_TIMEOUT_MS            500U
#define MISSION_ZDT_EMM_PULSES_PER_REV      3200U
#define MISSION_ZDT_COARSE_ANGLE_0P1DEG     1400U
#define MISSION_ZDT_FINE_ANGLE_0P1DEG         10U
#define MISSION_ZDT_SPEED_RPM                 120U
#define MISSION_ZDT_FINE_SPEED_RPM             50U
#define MISSION_ZDT_ACCEL                      50U
#define MISSION_ZDT_FINE_MAX_STEPS             35U
#define MISSION_ZDT_STATUS_POLL_MS             50U
#define MISSION_ZDT_SLOT_TIMEOUT_MS          8000U
#define MISSION_GATE_CONFIRM_SAMPLES            3U
#define MISSION_GATE_CONFIRM_INTERVAL_MS        5U
#define MISSION_SLOT_USE_CW                      1U

#endif /* MISSION_CONFIG_H */
