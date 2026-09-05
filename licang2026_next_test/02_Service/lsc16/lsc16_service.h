/**
 * @file    lsc16_service.h
 * @brief   LSC16命令串行队列与异步完成管理（平台无关）。
 *
 * LSC控制板只有一条UART8链路，同一时刻只能有一帧占用Core的DMA发送缓冲。
 * Service把多个上层请求排队，并在TX完成事件后才启动下一帧，防止任务并发
 * 覆盖发送数据。动作组0x06/0x08回报属于设备状态事件，不等同于TX完成。
 */

#ifndef LSC16_SERVICE_H
#define LSC16_SERVICE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#include "arm_bsp.h"

#define LSC16_SERVICE_QUEUE_DEPTH       4U

typedef enum {
    /** 同一帧控制一个或多个舵机，完成条件为 UART8 TX DMA 结束。 */
    LSC16_REQUEST_MOVE_SERVOS = 0,
    /** 启动已存储在舵控板中的动作组。 */
    LSC16_REQUEST_RUN_ACTION_GROUP,
    /** 发送动作组停止命令，不等待舵控板主动回报。 */
    LSC16_REQUEST_STOP_ACTION_GROUP,
    /** 修改动作组速度倍率。 */
    LSC16_REQUEST_SET_ACTION_SPEED,
    /** 请求电池电压；电压值通过 report 回调异步交付。 */
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

/**
 * @brief 舵控板主动回报交付回调。
 * @param user_ctx 配置时保存的用户上下文。
 * @param report_events 本批回报事件位。
 * @param report 最近一次已解析的回报快照。
 * @note 在Service worker或process_once()调用者上下文执行，不在ISR执行。
 */
typedef void (*lsc16_report_fn_t)(
    void *user_ctx,
    uint32_t report_events,
    const lsc16_report_t *report);

/**
 * @brief 唤醒Service worker的轻量通知函数。
 * @warning 可能由ISR路径调用，实现不得阻塞。
 */
typedef void (*lsc16_service_notify_fn_t)(void *user_ctx);

typedef struct {
    lsc16_t *device;
    lsc16_report_fn_t report_cb;
    void *report_ctx;
    lsc16_service_notify_fn_t notify_cb;
    void *notify_ctx;
} lsc16_service_config_t;

typedef struct {
    /** 已进入队列的请求数。 */
    uint32_t submitted;
    /** 已完成回调的请求数，包含底层发送失败。 */
    uint32_t completed;
    /** 参数非法或队列已满而未接纳的请求数。 */
    uint32_t rejected;
    /** DMA 启动失败、UART 错误等底层错误数。 */
    uint32_t io_errors;
    /** 已交付给上层的舵控板主动回报批次数。 */
    uint32_t reports;
} lsc16_service_stats_t;

typedef struct {
    bool initialized;
    bool active_valid;
    lsc16_service_config_t config;
    lsc16_request_t queue[LSC16_SERVICE_QUEUE_DEPTH];
    uint8_t queue_head;
    uint8_t queue_tail;
    uint8_t queue_count;
    lsc16_request_t active;

    /* ISR只递增序号；worker比较已消费序号，避免读清标志时丢事件。 */
    volatile uint32_t tx_complete_sequence;
    volatile uint32_t rx_ready_sequence;
    volatile uint32_t error_sequence;
    uint32_t handled_tx_sequence;
    uint32_t handled_rx_sequence;
    uint32_t handled_error_sequence;
    lsc16_service_stats_t stats;
} lsc16_service_t;

/**
 * @brief 绑定已初始化的LSC16 Core并接管其ISR事件通知。
 * @param service Service实例。
 * @param config Core对象、回报出口和worker通知配置。
 * @return LSC16_OK表示成功，否则返回参数或状态错误。
 */
lsc16_status_t lsc16_service_init(
    lsc16_service_t *service,
    const lsc16_service_config_t *config);

/**
 * @brief 解除Service与Core的事件绑定。
 * @param service Service实例。
 * @return LSC16_OK表示成功；有在途或排队请求时返回状态错误。
 */
lsc16_status_t lsc16_service_deinit(lsc16_service_t *service);

/**
 * @brief 将舵控请求复制到静态队列。
 * @param service Service实例。
 * @param request 待提交请求。
 * @return LSC16_OK表示已排队，否则返回参数、状态或队列满错误。
 * @note 返回成功不代表机械动作已经完成，只表示请求被Service接纳。
 */
lsc16_status_t lsc16_service_submit(
    lsc16_service_t *service,
    const lsc16_request_t *request);
/**
 * @brief 推进发送完成、下一请求和主动回报交付。
 * @param service Service实例。
 * @note 只能由唯一worker或裸机主循环串行调用，禁止在ISR中调用。
 */
void lsc16_service_process_once(lsc16_service_t *service);

/**
 * @brief 获取Service累计统计快照。
 * @param service Service实例。
 * @param stats 输出统计对象。
 * @return LSC16_OK表示成功，否则返回参数或未初始化错误。
 */
lsc16_status_t lsc16_service_get_stats(
    const lsc16_service_t *service,
    lsc16_service_stats_t *stats);

#ifdef __cplusplus
}
#endif

#endif /* LSC16_SERVICE_H */
