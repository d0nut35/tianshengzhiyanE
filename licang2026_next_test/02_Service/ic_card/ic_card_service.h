/**
 * @file    ic_card_service.h
 * @brief   IC卡读写器平台无关单事务Service。
 *
 * Service把“发送命令”和“等待对应响应”合并成一次完整事务，并负责超时、
 * 错误恢复和请求排队。Core只认协议字节，Device只认读球语义。
 */

#ifndef IC_CARD_SERVICE_H
#define IC_CARD_SERVICE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#include "ic_card_core.h"

#ifndef LICANG_RELEASE_MINIMAL
#define LICANG_RELEASE_MINIMAL 0
#endif

#define IC_CARD_SERVICE_DIAGNOSTICS_ENABLE (!LICANG_RELEASE_MINIMAL)

#define IC_CARD_SERVICE_QUEUE_DEPTH 4U

typedef enum {
    IC_CARD_REQUEST_READ_BLOCK = 0,
    IC_CARD_REQUEST_QUERY,
} ic_card_request_type_t;

/**
 * @brief IC卡事务完成回调。
 * @param user_ctx 提交请求时保存的用户上下文。
 * @param request_id 请求标识，用于上层匹配业务事务。
 * @param status 事务最终状态。
 * @param response 成功时的响应快照；失败时可能为NULL。
 * @note 回调在Service worker或调用process_once()的普通上下文执行，不在ISR执行。
 */
typedef void (*ic_card_request_done_fn_t)(
    void *user_ctx,
    uint32_t request_id,
    ic_card_status_t status,
    const ic_card_response_t *response);

/** @brief 返回单调递增的毫秒时基，供Service判断事务超时。 */
typedef uint32_t (*ic_card_now_ms_fn_t)(void *ctx);

/**
 * @brief 唤醒Service worker的轻量通知函数。
 * @warning 可能由ISR路径调用，实现不得阻塞。
 */
typedef void (*ic_card_worker_notify_fn_t)(void *ctx);

/** 请求会被Service按值复制，调用者返回后可立即释放栈对象。 */
typedef struct {
    uint32_t request_id;
    ic_card_request_type_t type;
    uint8_t address;
    uint32_t timeout_ms;
    union {
        struct {
            uint8_t block;
            bool led_beep_prompt;
        } read_block;
        struct {
            ic_card_command_t command;
        } query;
    } data;
    ic_card_request_done_fn_t done_cb;
    void *user_ctx;
} ic_card_request_t;

#if IC_CARD_SERVICE_DIAGNOSTICS_ENABLE
/** 仅供测试和调试读取的累计统计；正式任务不保存这些计数。 */
typedef struct {
    uint32_t submitted;
    uint32_t completed;
    uint32_t rejected;
    uint32_t timed_out;
    uint32_t uart_errors;
    uint32_t unrelated_responses;
} ic_card_service_stats_t;
#endif

typedef struct {
    ic_card_t *device;
    ic_card_now_ms_fn_t now_ms;
    void *time_ctx;
    ic_card_worker_notify_fn_t notify_worker;
    void *notify_ctx;
} ic_card_service_config_t;

typedef struct {
    bool initialized;
    ic_card_service_config_t config;
    ic_card_request_t queue[IC_CARD_SERVICE_QUEUE_DEPTH];
    uint8_t queue_head;
    uint8_t queue_tail;
    uint8_t queue_count;
    bool active_valid;
    bool tx_completed;
    ic_card_request_t active;
    uint32_t active_deadline_ms;
    volatile uint32_t tx_sequence;
    volatile uint32_t rx_sequence;
    volatile uint32_t error_sequence;
    uint32_t handled_tx_sequence;
    uint32_t handled_rx_sequence;
    uint32_t handled_error_sequence;
    uint32_t response_sequence;
#if IC_CARD_SERVICE_DIAGNOSTICS_ENABLE
    ic_card_service_stats_t stats;
#endif
} ic_card_service_t;

/**
 * @brief 初始化Service并把Core ISR轻量通知绑定到本对象。
 * @param service Service实例。
 * @param config 已初始化Core、时基和worker通知配置。
 * @return IC_CARD_OK表示成功，否则返回参数或状态错误。
 */
ic_card_status_t ic_card_service_init(
    ic_card_service_t *service,
    const ic_card_service_config_t *config);

/**
 * @brief 解除Service与Core的事件绑定。
 * @param service Service实例。
 * @return IC_CARD_OK表示成功；存在活动或排队请求时返回状态错误。
 */
ic_card_status_t ic_card_service_deinit(ic_card_service_t *service);

/**
 * @brief 将请求按值压入静态队列。
 * @param service Service实例。
 * @param request 待提交请求，函数返回前会完整复制。
 * @return IC_CARD_OK表示已排队，否则返回参数、状态或队列满错误。
 * @note 成功只表示排队，不表示读卡已经成功。
 */
ic_card_status_t ic_card_service_submit(
    ic_card_service_t *service,
    const ic_card_request_t *request);

/**
 * @brief 推进解析、完成、超时和下一请求。
 * @param service Service实例。
 * @note 只能由唯一worker或裸机主循环串行调用，禁止在ISR中调用。
 */
void ic_card_service_process_once(ic_card_service_t *service);

/**
 * @brief 获取Service累计统计快照。
 * @param service Service实例。
 * @param stats 输出统计对象。
 * @return IC_CARD_OK表示复制成功，否则返回参数或未初始化错误。
 */
#if IC_CARD_SERVICE_DIAGNOSTICS_ENABLE
ic_card_status_t ic_card_service_get_stats(
    const ic_card_service_t *service,
    ic_card_service_stats_t *stats);
#endif

#ifdef __cplusplus
}
#endif

#endif /* IC_CARD_SERVICE_H */
