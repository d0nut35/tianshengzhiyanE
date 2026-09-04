/** @file turntable_grasp_lite_config.h @brief 转盘视觉、读卡和逐槽轻量闭环参数。 */

#ifndef TURNTABLE_GRASP_LITE_CONFIG_H
#define TURNTABLE_GRASP_LITE_CONFIG_H

#define TURN_GRASP_LITE_VISION_DEVICE_ID       MULT_UART_DEVICE_0
#define TURN_GRASP_LITE_IC_DEVICE_ID           MULT_UART_DEVICE_1
#define TURN_GRASP_LITE_ZDT_DEVICE_ID          MULT_UART_DEVICE_2
#define TURN_GRASP_LITE_VISION_TIMEOUT_MS                    200U
#define TURN_GRASP_LITE_EVENT_READ_TIMEOUT_MS                250U
#define TURN_GRASP_LITE_EVENT_MAX_AGE_MS                     120U

#define TURN_GRASP_LITE_IC_ADDRESS                          0x20U
#define TURN_GRASP_LITE_IC_OPERATION_PROMPT                    1U
#define TURN_GRASP_LITE_IO_TIMEOUT_MS                         500U
#define TURN_GRASP_LITE_IC_MAX_ATTEMPTS                         5U
#define TURN_GRASP_LITE_IC_RETRY_MS                           150U

#define TURN_GRASP_LITE_ZDT_ADDRESS                             1U
#define TURN_GRASP_LITE_ZDT_EMM_PULSES_PER_REV               3200U
#define TURN_GRASP_LITE_ZDT_COARSE_ANGLE_0P1DEG              1400U
#define TURN_GRASP_LITE_ZDT_FINE_ANGLE_0P1DEG                  10U
#define TURN_GRASP_LITE_ZDT_SPEED_RPM                          60U
#define TURN_GRASP_LITE_ZDT_FINE_SPEED_RPM                     15U
#define TURN_GRASP_LITE_ZDT_ACCEL                              50U
#define TURN_GRASP_LITE_ZDT_FINE_MAX_STEPS                     20U
#define TURN_GRASP_LITE_ZDT_STATUS_POLL_MS                     50U
#define TURN_GRASP_LITE_ZDT_SLOT_TIMEOUT_MS                  8000U
#define TURN_GRASP_LITE_GATE_CONFIRM_SAMPLES                    3U
#define TURN_GRASP_LITE_GATE_CONFIRM_INTERVAL_MS                5U
#define TURN_GRASP_LITE_SLOT_USE_CW                              1U

#define TURN_GRASP_LITE_HOME_GROUP                            10U
#define TURN_GRASP_LITE_VISION_GROUP                          11U
#define TURN_GRASP_LITE_GRASP_GROUP                           12U
#define TURN_GRASP_LITE_ACTION_TIMEOUT_MS                  30000U

/* 只允许显式GRASP命令触发；上电和WATCH命令永远不运动。 */
#define TURN_GRASP_LITE_MOTION_ARMED                           1U

/* ZDT逐槽已完成独立实机验证；危险时仍必须切断电机动力电源。 */
#define TURN_GRASP_LITE_ZDT_MOTION_ARMED                       1U

#endif /* TURNTABLE_GRASP_LITE_CONFIG_H */
