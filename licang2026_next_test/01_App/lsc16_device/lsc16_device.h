/**
 * @file    lsc16_device.h
 * @brief   机械臂对LSC16舵控板的语义化Device接口。
 *
 * App只调用“移动舵机/运行动作组”等接口，不接触UART8、DMA、HAL回调和
 * CMSIS队列。这样后续夹取状态机可以替换动作组编号，而不破坏底层通信。
 */

#ifndef LSC16_DEVICE_H
#define LSC16_DEVICE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stddef.h>
#include <stdint.h>

#include "lsc16_service.h"

/**
 * @brief LSC16语义请求最终完成回调类型。
 * @param user_ctx 提交请求时绑定的用户上下文。
 * @param request_id 已完成请求编号。
 * @param status 最终通信状态。
 * @warning 回调成功只表示命令事务完成，不自动证明舵机已经到达目标位置。
 */
typedef void (*lsc16_device_done_fn_t)(
    void *user_ctx,
    uint32_t request_id,
    lsc16_status_t status);

/**
 * @brief 控制板主动状态回报订阅回调类型。
 * @param user_ctx 注册回调时绑定的用户上下文。
 * @param report_events 本轮回报事件位。
 * @param report 最近一次回报内容，仅在回调期间有效。
 */
typedef void (*lsc16_device_report_fn_t)(
    void *user_ctx,
    uint32_t report_events,
    const lsc16_report_t *report);

/**
 * @brief 初始化LSC16 Device层和其Service OS依赖。
 * @return 初始化结果。
 */
lsc16_status_t lsc16_device_init(void);

/**
 * @brief 设置动作组状态和电池回报订阅者。
 * @param report_cb App订阅回调；传NULL表示只解析但不上报应用。
 * @param user_ctx 原样传给report_cb的上下文。
 * @return 设置结果。
 */
lsc16_status_t lsc16_device_set_report_callback(
    lsc16_device_report_fn_t report_cb,
    void *user_ctx);

/**
 * @brief 异步提交一次单舵机运动命令。
 * @param servo_id 舵机ID，范围0~15。
 * @param position 目标脉宽，范围500~2500。
 * @param move_time_ms 运动时间，单位ms。
 * @param done_cb 命令事务完成回调，允许为NULL。
 * @param user_ctx 原样传给done_cb的上下文。
 * @return 请求入队状态。
 */
lsc16_status_t lsc16_device_move_servo(
    uint8_t servo_id,
    uint16_t position,
    uint16_t move_time_ms,
    lsc16_device_done_fn_t done_cb,
    void *user_ctx);

/**
 * @brief 异步提交一次多舵机同步运动命令。
 * @param targets 舵机目标数组。
 * @param servo_count 数组元素个数，范围1~16。
 * @param move_time_ms 共用运动时间，单位ms。
 * @param done_cb 命令事务完成回调，允许为NULL。
 * @param user_ctx 原样传给done_cb的上下文。
 * @return 请求入队状态。
 */
lsc16_status_t lsc16_device_move_servos(
    const lsc16_servo_target_t *targets,
    uint8_t servo_count,
    uint16_t move_time_ms,
    lsc16_device_done_fn_t done_cb,
    void *user_ctx);

/**
 * @brief 异步提交运行动作组命令。
 * @param action_group 动作组编号。
 * @param repeat_count 重复次数，0表示循环执行。
 * @param done_cb 命令事务完成回调，允许为NULL。
 * @param user_ctx 原样传给done_cb的上下文。
 * @return 请求入队状态。
 */
lsc16_status_t lsc16_device_run_action_group(
    uint8_t action_group,
    uint16_t repeat_count,
    lsc16_device_done_fn_t done_cb,
    void *user_ctx);

/**
 * @brief 异步提交停止动作组命令。
 * @param done_cb 命令事务完成回调，允许为NULL。
 * @param user_ctx 原样传给done_cb的上下文。
 * @return 请求入队状态。
 */
lsc16_status_t lsc16_device_stop_action_group(
    lsc16_device_done_fn_t done_cb,
    void *user_ctx);

/**
 * @brief 异步提交动作组速度设置命令。
 * @param action_group 动作组编号，0xFF表示全部动作组。
 * @param speed_percent 速度百分比参数。
 * @param done_cb 命令事务完成回调，允许为NULL。
 * @param user_ctx 原样传给done_cb的上下文。
 * @return 请求入队状态。
 */
lsc16_status_t lsc16_device_set_action_group_speed(
    uint8_t action_group,
    uint16_t speed_percent,
    lsc16_device_done_fn_t done_cb,
    void *user_ctx);

/**
 * @brief 异步提交电池电压查询命令。
 * @param done_cb 查询命令事务完成回调，允许为NULL。
 * @param user_ctx 原样传给done_cb的上下文。
 * @return 请求入队状态；电压值通过主动回报回调获得。
 */
lsc16_status_t lsc16_device_request_battery_voltage(
    lsc16_device_done_fn_t done_cb,
    void *user_ctx);

#ifdef __cplusplus
}
#endif

#endif /* LSC16_DEVICE_H */
