/**
 * @file    lsc16_service.h
 * @brief   机械臂CMSIS-RTOS2串行请求Service。
 *
 * UART8同一时刻只发送一帧。Service按值保存请求，由唯一worker推进发送、
 * 接收解析和业务回调；ISR只登记事件并唤醒worker。
 */

#ifndef LSC16_SERVICE_H
#define LSC16_SERVICE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#include "arm_bsp.h"

#define LSC16_SERVICE_QUEUE_DEPTH       4U

typedef enum {
    LSC16_REQUEST_MOVE_SERVOS = 0,
    LSC16_REQUEST_RUN_ACTION_GROUP,
    LSC16_REQUEST_STOP_ACTION_GROUP,
    LSC16_REQUEST_SET_ACTION_SPEED,
    LSC16_REQUEST_GET_BATTERY,
} lsc16_request_type_t;

typedef struct {
    uint8_t count;
    uint16_t move_time_ms;
    lsc16_servo_target_t targets[LSC16_SERVO_COUNT_MAX];
} lsc16_move_request_t;

typedef struct {
    uint8_t action_group;
    uint16_t repeat_count;
} lsc16_action_run_request_t;

typedef struct {
    uint8_t action_group;
    uint16_t speed_percent;
} lsc16_action_speed_request_t;

typedef struct {
    uint32_t request_id;
    lsc16_request_type_t type;
    union {
        lsc16_move_request_t move;
        lsc16_action_run_request_t action_run;
        lsc16_action_speed_request_t action_speed;
    } data;
    void (*done_cb)(void *user_ctx, uint32_t request_id, lsc16_status_t status);
    void *user_ctx;
} lsc16_request_t;

/** 舵控板主动回报在Service worker中交付。 */
typedef void (*lsc16_report_fn_t)(
    void *user_ctx,
    uint32_t report_events,
    const lsc16_report_t *report);

typedef struct {
    uint32_t submitted;
    uint32_t completed;
    uint32_t rejected;
    uint32_t io_errors;
    uint32_t reports;
} lsc16_service_stats_t;

/**
 * @brief 初始化机械臂BSP、队列、UART路由和唯一worker。
 * @param report_cb 主动回报回调，允许为NULL。
 * @param report_ctx 原样传给report_cb的上下文。
 * @return 全部资源建立成功返回LSC16_OK。
 * @pre 在osKernelInitialize()之后、osKernelStart()之前调用。
 */
lsc16_status_t lsc16_service_init(
    lsc16_report_fn_t report_cb,
    void *report_ctx);

/**
 * @brief 按值提交一笔机械臂请求。
 * @param request 请求、原始request_id和回调信息。
 * @return 入队结果；成功只表示请求已接纳。
 */
lsc16_status_t lsc16_service_submit(const lsc16_request_t *request);

/** 获取Service累计统计快照。 */
lsc16_status_t lsc16_service_get_stats(lsc16_service_stats_t *stats);

#ifdef __cplusplus
}
#endif

#endif /* LSC16_SERVICE_H */
