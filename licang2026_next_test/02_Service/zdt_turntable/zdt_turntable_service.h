/**
 * @file    zdt_turntable_service.h
 * @brief   ZDT转盘单总线事务Service。
 *
 * Service串行化“先挂RX、再发TX、等待完整返回、超时abort”的事务。ISR只
 * 发布TX/RX/error事件，协议解析和完成回调均在process_once()普通上下文执行。
 * 当前一笔事务只消费一帧响应，因此动作命令要求设备Response=Receive；
 * Response=Both产生的后续0x9F不属于本Service当前支持的事务契约。
 * HAL transport还必须把一个短响应完整交付给一次RX事件；若后续实测出现
 * 字节间IDLE导致的半帧，需要在Service增加按功能码期望长度的累积拆包。
 */

#ifndef ZDT_TURNTABLE_SERVICE_H
#define ZDT_TURNTABLE_SERVICE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "zdt_turntable_core.h"

/** Service按值保存的等待队列深度。 */
#define ZDT_TURNTABLE_SERVICE_QUEUE_DEPTH 4U

/** 启动一次异步发送；数据在TX完成或abort前必须保持有效。 */
typedef zdt_turntable_status_t (*zdt_turntable_tx_start_fn_t)(
    void *ctx, const uint8_t *data, size_t len);
/** 启动一次异步接收；缓冲区在RX事件或abort前必须保持有效。 */
typedef zdt_turntable_status_t (*zdt_turntable_rx_start_fn_t)(
    void *ctx, uint8_t *data, size_t capacity);
/** 同步中止当前收发并清理底层传输状态。 */
typedef zdt_turntable_status_t (*zdt_turntable_abort_fn_t)(void *ctx);

/** Service依赖的最小平台传输能力。 */
typedef struct {
    zdt_turntable_tx_start_fn_t tx_start;
    zdt_turntable_rx_start_fn_t rx_start;
    zdt_turntable_abort_fn_t abort;
    void *ctx;
} zdt_turntable_port_t;

/**
 * @brief 一笔事务的异步完成回调。
 * @param user_ctx 提交请求时保存的调用者上下文。
 * @param request_id 请求标识。
 * @param status 最终事务状态。
 * @param response 成功或设备错误时的解析结果；超时/I/O/协议错误时为NULL。
 * @warning 回调在process_once()调用上下文执行，不在UART ISR中执行。
 */
typedef void (*zdt_turntable_done_fn_t)(
    void *user_ctx,
    uint32_t request_id,
    zdt_turntable_status_t status,
    const zdt_turntable_response_t *response);

/**
 * @brief Service队列中的一笔完整事务。
 * @note frame按值保存，避免调用者栈缓冲在DMA发送前失效。
 */
typedef struct {
    uint32_t request_id;
    uint8_t frame[ZDT_TURNTABLE_FRAME_MAX];
    size_t frame_len;
    uint8_t expected_address;
    uint8_t expected_function;
    uint32_t timeout_ms;
    zdt_turntable_done_fn_t done_cb;
    void *user_ctx;
} zdt_turntable_request_t;

/** 初始化Service时注入的平台能力、时基和worker唤醒函数。 */
typedef struct {
    zdt_turntable_port_t port;
    uint32_t (*now_ms)(void *ctx);
    void *time_ctx;
    void (*notify_worker)(void *ctx);
    void *notify_ctx;
} zdt_turntable_service_config_t;

/**
 * @brief 单UART、单在途事务的Service状态。
 *
 * active.frame和rx_buffer由Service长期持有，覆盖完整DMA生命周期；volatile
 * 标志由UART ISR写、唯一worker或裸机主循环读。Service不允许多个普通上下文
 * 并发调用submit/process_once，RTOS集成必须由上层保证单worker所有权。
 */
typedef struct {
    bool initialized;
    zdt_turntable_service_config_t config;
    zdt_turntable_request_t queue[ZDT_TURNTABLE_SERVICE_QUEUE_DEPTH];
    uint8_t queue_head;
    uint8_t queue_tail;
    uint8_t queue_count;
    bool active_valid;
    zdt_turntable_request_t active;
    uint8_t rx_buffer[ZDT_TURNTABLE_RESPONSE_MAX];
    volatile bool tx_done;
    volatile bool rx_ready;
    volatile bool uart_error;
    volatile uint16_t rx_len;
    uint32_t deadline_ms;
} zdt_turntable_service_t;

/**
 * @brief 初始化平台无关事务Service。
 * @param service 待初始化实例。
 * @param config 已绑定传输、时基和可选唤醒出口的配置。
 * @return 初始化结果；本函数不启动UART收发。
 */
zdt_turntable_status_t zdt_turntable_service_init(
    zdt_turntable_service_t *service,
    const zdt_turntable_service_config_t *config);

/**
 * @brief 按值复制并排队一笔请求。
 * @param service 已初始化实例。
 * @param request 完整事务描述。
 * @return OK表示已入队，不表示DMA启动、设备应答或机械到位。
 * @warning 该接口和process_once必须由同一普通上下文串行调用。
 */
zdt_turntable_status_t zdt_turntable_service_submit(
    zdt_turntable_service_t *service,
    const zdt_turntable_request_t *request);

/**
 * @brief 在唯一worker或裸机主循环中推进事务。
 * @param service 已初始化实例。
 * @note 负责处理ISR事件、协议解析、超时/错误abort、完成回调及下一笔启动。
 */
void zdt_turntable_service_process_once(zdt_turntable_service_t *service);

/**
 * @brief HAL adapter在TX完成ISR中调用。
 * @param service 当前独占UART的Service。
 * @warning 仅置标志并唤醒worker，禁止从任务上下文直接调用。
 */
void zdt_turntable_service_on_tx_complete_isr(zdt_turntable_service_t *service);

/**
 * @brief HAL adapter在ReceiveToIdle ISR中调用。
 * @param service 当前独占UART的Service。
 * @param rx_len DMA缓冲区本次有效字节数。
 * @warning 仅发布长度和事件；解析与回调延后到process_once()。
 */
void zdt_turntable_service_on_rx_event_isr(
    zdt_turntable_service_t *service, uint16_t rx_len);

/**
 * @brief HAL adapter在UART错误ISR中调用。
 * @param service 当前独占UART的Service。
 * @warning 仅置错误标志并唤醒worker，实际abort在普通上下文执行。
 */
void zdt_turntable_service_on_error_isr(zdt_turntable_service_t *service);

#ifdef __cplusplus
}
#endif

#endif /* ZDT_TURNTABLE_SERVICE_H */
