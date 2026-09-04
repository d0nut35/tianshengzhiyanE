/**
 * @file    mult_uart_service.h
 * @brief   单条 mult_uart 物理总线的请求队列与服务层。
 *
 * Service 层位于 BSP/Core 之上、应用任务之下。它负责把应用提交的
 * “切通道/发送/接收/先收后发”请求复制进内部静态队列，再按顺序驱动
 * Core 总线。这里仍然保持平台无关：头文件不包含 HAL、FreeRTOS 或
 * CMSIS-RTOS2 类型，后续 RTOS 线程/队列只会作为更外层 adapter 接入。
 */

#ifndef MULT_UART_SERVICE_H
#define MULT_UART_SERVICE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "mult_uart_core.h"

#define MULT_UART_SERVICE_QUEUE_DEPTH    4U
#define MULT_UART_SERVICE_TX_MAX         128U
#define MULT_UART_SERVICE_RX_MAX         128U

/**
 * @brief 上层可以提交的总线操作类型。
 *
 * WRITE_READ 表示典型“问答式”事务：先打开 RX DMA，再启动 TX DMA。
 * 这样做是为了避免外设响应很快时，回复在 RX 还没准备好之前就到达。
 */
typedef enum {
    MULT_UART_OP_SELECT = 0,
    MULT_UART_OP_WRITE,
    MULT_UART_OP_READ,
    MULT_UART_OP_WRITE_READ,
} mult_uart_operation_t;

/**
 * @brief Service 自身生命周期状态。
 *
 * 当前 P4a 版本提供的是可轮询的 worker engine：初始化后 READY，
 * start 后 RUNNING，stop 会短暂进入 STOPPING 并取消未完成请求。
 */
typedef enum {
    MULT_UART_SERVICE_STATE_UNINIT = 0,
    MULT_UART_SERVICE_STATE_READY,
    MULT_UART_SERVICE_STATE_RUNNING,
    MULT_UART_SERVICE_STATE_STOPPING,
    MULT_UART_SERVICE_STATE_ERROR,
} mult_uart_service_state_t;

/**
 * @brief 单个请求结束时交给上层的完成信息。
 *
 * rx_data 指向 Service 内部缓冲区，只在 done_cb 回调执行期间有效；
 * 如果需要长期保存，应用必须在回调里立刻复制。
 */
typedef struct {
    uint32_t request_id;
    mult_uart_status_t status;
    mult_uart_operation_t operation;
    mult_uart_channel_t channel;
    const uint8_t *rx_data;
    size_t rx_len;
} mult_uart_completion_t;

/**
 * @brief 请求完成回调。
 *
 * 该回调在调用 mult_uart_service_process_once() 的上下文中执行，
 * 不是 ISR 上下文；后续接入 RTOS 后，它会自然落在 Service worker
 * 任务上下文中。
 */
typedef void (*mult_uart_done_fn_t)(
    void *user_ctx,
    const mult_uart_completion_t *completion);

/**
 * @brief Service 内部事件通知回调。
 *
 * 该回调用于把 Core 的 TX/RX/error 事件“轻轻拍一下”外层 worker。
 * 平台无关 Service 不知道 FreeRTOS 的线程句柄，所以只保存一个抽象函数
 * 指针；RTOS adapter 可以在这里调用 osThreadFlagsSet()。
 */
typedef void (*mult_uart_service_notify_fn_t)(void *user_ctx);

/**
 * @brief 应用提交给 Service 的请求描述。
 *
 * submit 成功后，Service 会复制 TX 数据和请求元数据，因此调用者的
 * tx_data 原始数组可以复用；RX 数据由 Service 内部缓冲区承载，通过
 * completion 回调短暂交给应用查看。
 */
typedef struct {
    uint32_t request_id;
    mult_uart_operation_t operation;
    mult_uart_channel_t channel;
    const uint8_t *tx_data;
    size_t tx_len;
    size_t rx_capacity;
    uint32_t io_timeout_ms;
    mult_uart_done_fn_t done_cb;
    void *user_ctx;
} mult_uart_request_t;

/**
 * @brief Service 运行统计，用于调试队列压力和错误来源。
 */
typedef struct {
    uint32_t submitted;
    uint32_t completed;
    uint32_t queue_full;
    uint32_t invalid_request;
    uint32_t switch_count;
    uint32_t tx_error;
    uint32_t rx_error;
    uint32_t timeout;
    uint32_t overflow;
    uint32_t cancelled;
    uint32_t queue_high_watermark;
} mult_uart_service_stats_t;

/**
 * @brief Service 初始化配置。
 *
 * now_ms 可以为空；为空时表示本层没有可用的软件时间源，超时检测将
 * 不具备实际推进能力。目标板接入时应传入 HAL_GetTick 或 RTOS tick。
 */
typedef struct {
    mult_uart_bus_t *bus;
    uint32_t (*now_ms)(void *time_ctx);
    void *time_ctx;
    mult_uart_service_notify_fn_t notify_cb;
    void *notify_ctx;
} mult_uart_service_config_t;

/**
 * @brief Service 内部队列节点。
 *
 * 该结构暴露在头文件里是为了保持 service 对象可静态分配；上层不应
 * 直接读写这些字段，应该只通过 public API 操作。
 */
typedef struct {
    uint32_t request_id;
    mult_uart_operation_t operation;
    mult_uart_channel_t channel;
    uint8_t tx_data[MULT_UART_SERVICE_TX_MAX];
    size_t tx_len;
    uint8_t rx_data[MULT_UART_SERVICE_RX_MAX];
    size_t rx_capacity;
    uint32_t io_timeout_ms;
    uint32_t deadline_ms;
    mult_uart_done_fn_t done_cb;
    void *user_ctx;
} mult_uart_service_job_t;

/**
 * @brief Core ISR 事件在 Service 层的暂存区。
 *
 * Core 回调可能由 ISR 间接触发，因此这里使用 volatile 标记事件位。
 * 复杂处理不在 ISR 里做，而是在 process_once() 中收敛。
 */
typedef struct {
    volatile bool tx_done;
    volatile bool rx_done;
    volatile bool error;
    volatile size_t rx_len;
    volatile mult_uart_status_t error_status;
    volatile uint32_t port_error;
} mult_uart_service_events_t;

/**
 * @brief 一个 mult_uart Service 实例。
 *
 * 一个实例只管理一个 mult_uart_bus_t，也就是“一套公共 UART + 一片
 * 复用器”。如果未来有两套独立公共 UART，就创建两个 service 实例。
 */
typedef struct {
    bool initialized;
    bool running;
    bool active;
    mult_uart_service_state_t state;
    mult_uart_bus_t *bus;
    uint32_t (*now_ms)(void *time_ctx);
    void *time_ctx;
    mult_uart_service_notify_fn_t notify_cb;
    void *notify_ctx;
    mult_uart_service_job_t queue[MULT_UART_SERVICE_QUEUE_DEPTH];
    size_t queue_head;
    size_t queue_tail;
    size_t queue_count;
    mult_uart_service_job_t active_job;
    mult_uart_service_events_t events;
    mult_uart_service_stats_t stats;
} mult_uart_service_t;

/**
 * @brief 绑定 Core bus 和时间源，进入 READY 状态。
 * @param service Service实例。
 * @param config Core总线、时基和worker通知配置。
 * @return MULT_UART_OK表示成功，否则返回参数或状态错误。
 */
mult_uart_status_t mult_uart_service_init(
    mult_uart_service_t *service,
    const mult_uart_service_config_t *config);

/**
 * @brief 允许接收新请求，进入 RUNNING 状态。
 * @param service Service实例。
 * @return MULT_UART_OK表示成功，否则返回参数或状态错误。
 */
mult_uart_status_t mult_uart_service_start(mult_uart_service_t *service);

/**
 * @brief 提交一个请求到内部静态队列。
 *
 * 本函数只表示“入队成功/失败”，不表示 UART 操作已经完成。
 * @param service Service实例。
 * @param request 待提交事务；TX数据在返回前复制到内部队列。
 * @return MULT_UART_OK表示已排队，否则返回参数、状态或队列满错误。
 */
mult_uart_status_t mult_uart_service_submit(
    mult_uart_service_t *service,
    const mult_uart_request_t *request);

/**
 * @brief 推进一次 Service 状态机。
 *
 * PC fake 测试和裸轮询场景可在主循环周期调用；后续 RTOS adapter 会把
 * 它放到 worker 任务循环里调用。
 * @param service Service实例。
 * @note 只能由唯一worker串行调用，禁止在ISR中调用。
 */
void mult_uart_service_process_once(mult_uart_service_t *service);

/**
 * @brief 停止接收新请求，并取消 active/queued 请求。
 * @param service Service实例。
 * @return MULT_UART_OK表示已停止，否则返回参数或底层恢复错误。
 */
mult_uart_status_t mult_uart_service_stop(mult_uart_service_t *service);

/**
 * @brief 解除与 Core bus 的事件绑定并回到 UNINIT。
 * @param service Service实例。
 * @return MULT_UART_OK表示成功，否则返回参数或状态错误。
 */
mult_uart_status_t mult_uart_service_deinit(mult_uart_service_t *service);

/**
 * @brief 读取当前统计快照。
 * @param service Service实例。
 * @param stats 输出统计对象。
 * @return MULT_UART_OK表示成功，否则返回参数或未初始化错误。
 */
mult_uart_status_t mult_uart_service_get_stats(
    const mult_uart_service_t *service,
    mult_uart_service_stats_t *stats);

#ifdef __cplusplus
}
#endif

#endif /* MULT_UART_SERVICE_H */
