/** @file zdt_turntable_test_config.h @brief ZDT直连运动测试安全参数。 */

#ifndef ZDT_TURNTABLE_TEST_CONFIG_H
#define ZDT_TURNTABLE_TEST_CONFIG_H

/** 电机地址暂按手册默认1；实机先用只读命令核对。 */
#define ZDT_TURNTABLE_TEST_ADDRESS                 1U

/**
 * Emm换算按1.8度电机、16细分，即3200脉冲/电机轴圈。
 * OPTIONS只能确认1.8度电机，不能读取MStep；每次运动测试前仍须在电机菜单
 * 人工确认MStep=16，否则实际角度会按真实细分值成比例偏离。
 */
#define ZDT_TURNTABLE_TEST_EMM_PULSES_PER_REV   3200U

/**
 * 首次卸载测试的小角度占位值，单位0.1度，100表示10.0度。
 * 这不是最终槽距；确认槽数、传动比和机械方向后再修改。
 */
#define ZDT_TURNTABLE_TEST_STEP_ANGLE_0P1DEG     100U

/** X固件速度单位0.1RPM，加减速度单位RPM/S。 */
#define ZDT_TURNTABLE_TEST_X_SPEED_0P1RPM         600U
#define ZDT_TURNTABLE_TEST_X_ACCEL_RPM_S          200U
#define ZDT_TURNTABLE_TEST_X_DECEL_RPM_S          200U

/** Emm固件速度单位RPM，加速度为0~255档。 */
#define ZDT_TURNTABLE_TEST_EMM_SPEED_RPM           60U
#define ZDT_TURNTABLE_TEST_EMM_ACCEL                50U

/** 单笔直连查询/命令等待完整响应的超时时间。 */
#define ZDT_TURNTABLE_TEST_TIMEOUT_MS              500U

/**
 * 正式默认必须锁住所有运动。只有完成供电、共地、急停和机械安全检查后，
 * 才可在本轮实机测试中临时改1；测试结束必须恢复0。当前值1表示调试口
 * MOVE_CW/MOVE_CCW会真实驱动电机，烧录前必须再次人工检查机械空间。
 */
#define ZDT_TURNTABLE_TEST_MOTION_ARMED              1U

#endif /* ZDT_TURNTABLE_TEST_CONFIG_H */
