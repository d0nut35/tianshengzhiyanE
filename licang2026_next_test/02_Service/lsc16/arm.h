/**
 * @file    arm.h
 * @brief   机械臂异步命令Service接口。
 */

#ifndef ARM_H
#define ARM_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#include "arm_bsp.h"

#define ARM_QUEUE_DEPTH                 4U

/** 命令事务完成回调；request_id为Service分配并随原请求保存的非零编号。 */
typedef void (*arm_done_fn_t)(
    void *user_ctx,
    uint32_t request_id,
    lsc16_status_t status);

/** 舵控板主动回报回调；在机械臂worker任务上下文执行。 */
typedef void (*arm_report_fn_t)(
    void *user_ctx,
    uint32_t report_events,
    const lsc16_report_t *report);

typedef struct {
    uint32_t submitted;
    uint32_t completed;
    uint32_t rejected;
    uint32_t io_errors;
    uint32_t reports;
} arm_stats_t;

/**
 * @brief 初始化UART8机械臂BSP、队列、公共路由和唯一worker。
 * @return 全部资源建立成功返回LSC16_OK。
 * @pre 在osKernelInitialize()之后、osKernelStart()之前调用。
 */
lsc16_status_t arm_init(void);

/** 设置动作组状态和电池回报订阅者；传NULL表示只解析不上报。 */
lsc16_status_t arm_on_report(arm_report_fn_t callback, void *user_ctx);

/**
 * @brief 异步移动一个舵机。
 * @note 完成回调只表示UART命令发送结束，不表示舵机已经到位。
 */
lsc16_status_t arm_move(
    uint8_t servo_id,
    uint16_t position,
    uint16_t move_time_ms,
    arm_done_fn_t done_cb,
    void *user_ctx);

/**
 * @brief 异步同步移动1至16个舵机。
 * @note targets在返回前按值复制，调用者可立即释放原数组。
 */
lsc16_status_t arm_move_all(
    const lsc16_servo_target_t *targets,
    uint8_t servo_count,
    uint16_t move_time_ms,
    arm_done_fn_t done_cb,
    void *user_ctx);

/**
 * @brief 异步运行动作组。
 * @note 命令发送完成由done_cb报告，机械动作完成由0x08主动回报报告。
 */
lsc16_status_t arm_run(
    uint8_t action_group,
    uint16_t repeat_count,
    arm_done_fn_t done_cb,
    void *user_ctx);

/** 异步发送动作组停止命令。 */
lsc16_status_t arm_stop(arm_done_fn_t done_cb, void *user_ctx);

/** 异步设置动作组速度；action_group为0xFF时应用于全部动作组。 */
lsc16_status_t arm_speed(
    uint8_t action_group,
    uint16_t speed_percent,
    arm_done_fn_t done_cb,
    void *user_ctx);

/** 异步查询舵控板电池电压；数值通过主动回报回调交付。 */
lsc16_status_t arm_battery(arm_done_fn_t done_cb, void *user_ctx);

/** 获取机械臂Service累计统计快照。 */
lsc16_status_t arm_get_stats(arm_stats_t *stats);

#ifdef __cplusplus
}
#endif

#endif /* ARM_H */
