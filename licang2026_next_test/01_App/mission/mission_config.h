/** @file mission_config.h @brief Mission初版状态机的任务与流程配置。 */

#ifndef MISSION_CONFIG_H
#define MISSION_CONFIG_H

#define MISSION_TASK_STACK_SIZE        3072U
#define MISSION_COMMAND_QUEUE_DEPTH       4U
#define MISSION_OPERATION_TIMEOUT_MS  30000U
/* 底盘四轮握手不设上限且每1s重发MISSION_READY，Mission侧同样不计时。 */
#define MISSION_READY_TIMEOUT_MS          0U
#define MISSION_AUTO_START_DELAY_MS    4000U

/*
 * Mission无线联调开关：1=USART1指令测试任务，0=正式比赛流程。
 * 测试任务支持纯路径分段控制，以及只在指定区域启用完整视觉抓取。
 */
#define MISSION_CHASSIS_ROUTE_TEST_ENABLED  0U
/* 正式仓库/阶梯诊断：USART1输出；现场定位后改0即可关闭。 */
#define MISSION_DEPOT_TRACE_ENABLED          1U
/* 正式假按钮选色：0=红方，1=蓝方；无线仍由RED/BLUE选择，实体按键尚未接入。 */
#define MISSION_FORMAL_BLUE_SIDE              1U
#define MISSION_CHASSIS_ROUTE_TEST_PAUSE_MS 2000U
#define MISSION_DEPOT_DIGIT_WAIT_MS        5000U /* 仓库每次READY后等数字，和积木期限独立。 */
#define MISSION_WIRELESS_POLL_MS               10U

#define MISSION_HOME_ACTION_GROUP        10U
#define MISSION_PLATFORM_VISION_GROUP    11U
#define MISSION_PLATFORM_GRASP_GROUP     12U
#define MISSION_STAIR_VISION_GROUP       13U
#define MISSION_STAIR_LOW_GROUP          14U
#define MISSION_STAIR_HIGH_GROUP         15U
#define MISSION_STAIR_MID_GROUP          16U
#define MISSION_STAIR_EXIT_GROUP         18U /* 阶梯结束后的撤离过渡姿态 */
#define MISSION_SMALL_DISC_VISION_GROUP  19U /* 小圆盘绕行识别姿态 */
#define MISSION_SMALL_DISC_GRASP_GROUP   20U /* 小圆盘抓球并放入车载转盘 */
#define MISSION_SMALL_DISC_EXIT_GROUP    21U /* 历史小圆盘撤离过渡；当前流程退出后直接10，不调用21。 */
#define MISSION_DEPOT_PICK_GROUP         22U /* 仓库从车载转盘取球 */
#define MISSION_DEPOT_ROW3_GROUP         23U /* 三层放球，用户已将撤离过渡包含在动作内 */
#define MISSION_DEPOT_ROW2_GROUP         25U /* 二层放球及撤离 */
#define MISSION_DEPOT_ROW1_GROUP         26U /* 一层放球 */

#define MISSION_PLATFORM_BALL_COUNT       5U
#define MISSION_PLATFORM_MAX_ATTEMPTS     7U /* 仅圆盘：读卡失败不占槽，允许再抓。 */
#define MISSION_PLATFORM_SETTLE_MS      200U /* 仅圆盘，开视觉前等待机械晃动消退。 */
#define MISSION_STAIR_BALL_COUNT          2U
#define MISSION_STAIR_TEST_BALL_COUNT     8U /* ROUTE STAIRS无线测试允许抓8球。 */
#define MISSION_STAIR_BALL_LIMIT          (MISSION_CHASSIS_ROUTE_TEST_ENABLED ? MISSION_STAIR_TEST_BALL_COUNT : MISSION_STAIR_BALL_COUNT)
#define MISSION_SMALL_DISC_BALL_COUNT     2U

/* UART7复用器设备映射：Nano=通道0；IC和转盘映射收在各自Service。 */
#define MISSION_VISION_DEVICE_ID MUX_DEVICE_0

#define MISSION_VISION_TIMEOUT_MS           200U
#define MISSION_VISION_READ_TIMEOUT_MS      250U
#define MISSION_VISION_EVENT_MAX_AGE_MS     120U
/* 模型在Nano启动时预加载；起点每秒查询，不把加载时间算入识别限时。 */
#define MISSION_MODEL_QUERY_INTERVAL_MS   1000U
#define MISSION_BLOCK_PREPARE_TIMEOUT_MS 10000U
/* READY后结果及ACK的总等待保护；正常回报立即继续，不改变Nano采样期限。 */
#define MISSION_BLOCK_RESULT_TIMEOUT_MS   8000U
#define MISSION_BLOCK_SETTLE_MS            200U
#define MISSION_BLOCK_FOUND_PAUSE_MS      1000U
#define MISSION_BLOCK_HIGH_VISION_GROUP     28U
#define MISSION_BLOCK_MID_VISION_GROUP      29U
#define MISSION_BLOCK_LOW_VISION_GROUP      30U
#define MISSION_BLOCK_HIGH_GRASP_GROUP      31U
#define MISSION_BLOCK_HIGH_PLACE_GROUP      32U
#define MISSION_BLOCK_MID_GRASP_GROUP       33U
#define MISSION_BLOCK_MID_PLACE_GROUP       34U
#define MISSION_BLOCK_LOW_GRASP_GROUP       35U
#define MISSION_BLOCK_LOW_PLACE_GROUP       36U

#define MISSION_IC_OPERATION_PROMPT           1U
#define MISSION_IC_MAX_ATTEMPTS                5U
#define MISSION_IC_RETRY_MS                  150U

#define MISSION_ZDT_ADDRESS                    1U
#define MISSION_ZDT_IO_TIMEOUT_MS            500U
#define MISSION_ZDT_EMM_PULSES_PER_REV      3200U
#define MISSION_ZDT_COARSE_ANGLE_0P1DEG     1400U
/* 反向粗角独立保留；先与正向相同，实机测得偏差后单独调整。 */
#define MISSION_ZDT_REVERSE_COARSE_ANGLE_0P1DEG 1400U
#define MISSION_ZDT_FINE_ANGLE_0P1DEG         10U
#define MISSION_ZDT_SPEED_RPM                 600U
#define MISSION_ZDT_FINE_SPEED_RPM             600U /* 用户要求微调与粗调同速，保留PB0校准。 */
#define MISSION_ZDT_ACCEL                      50U
#define MISSION_ZDT_FINE_MAX_STEPS             10U
#define MISSION_ZDT_DEPOT_FINE_MAX_STEPS       15U /* 正式/无线放球：多格粗调后统一微调。 */
#define MISSION_ZDT_DEPOT_SLOT_TIMEOUT_MS    8000U /* 放球总超时=此值乘目标格数。 */
#define MISSION_ZDT_STATUS_POLL_MS             50U
#define MISSION_ZDT_SLOT_TIMEOUT_MS          8000U
#define MISSION_GATE_CONFIRM_SAMPLES            3U
#define MISSION_GATE_CONFIRM_INTERVAL_MS        5U
#define MISSION_SLOT_USE_CW                      1U

#if (MISSION_CHASSIS_ROUTE_TEST_ENABLED > 1U)
#error "MISSION_CHASSIS_ROUTE_TEST_ENABLED must be 0 or 1"
#endif
#if (MISSION_FORMAL_BLUE_SIDE > 1U)
#error "MISSION_FORMAL_BLUE_SIDE must be 0 or 1"
#endif

#endif /* MISSION_CONFIG_H */
