/** @file zdt_turntable_service.c @brief ZDT转盘事务Service实现。 */

#include "zdt_turntable_service.h"

#include <string.h>

/**
 * @brief 使用有符号差值判断32位毫秒计数是否到期。
 * @param now 当前计数。
 * @param deadline 截止计数。
 * @return 到期或已超期时返回true。
 * @note 对不超过INT32_MAX毫秒的超时可正确跨越uint32_t回绕。
 */
static bool zdt_time_reached(uint32_t now, uint32_t deadline)
{
    return ((int32_t)(now - deadline) >= 0);
}

/**
 * @brief 从ISR或提交路径唤醒可选worker。
 * @param service Service实例。
 * @note 裸机配置可注入空通知函数；本函数不推进事务。
 */
static void zdt_notify(const zdt_turntable_service_t *service)
{
    if (service->config.notify_worker != NULL) {
        service->config.notify_worker(service->config.notify_ctx);
    }
}

/**
 * @brief 结束当前事务、清除在途标志并调用上层完成回调。
 * @param service Service实例。
 * @param status 最终事务状态。
 * @param response 可选解析响应。
 * @note 先释放active状态再回调，允许回调后续提交新请求。
 */
static void zdt_complete(
    zdt_turntable_service_t *service,
    zdt_turntable_status_t status,
    const zdt_turntable_response_t *response)
{
    zdt_turntable_done_fn_t done_cb = service->active.done_cb;
    void *user_ctx = service->active.user_ctx;
    uint32_t request_id = service->active.request_id;

    service->active_valid = false;
    service->tx_done = false;
    service->rx_ready = false;
    service->uart_error = false;
    service->rx_len = 0U;
    if (done_cb != NULL) {
        done_cb(user_ctx, request_id, status, response);
    }
}

/**
 * @brief 从FIFO取出下一笔请求并按“先RX、后TX”顺序启动DMA。
 * @param service Service实例。
 * @note 若任一步启动失败，会同步abort并在当前普通上下文完成该请求。
 */
static void zdt_start_next(zdt_turntable_service_t *service)
{
    zdt_turntable_status_t status;

    if (service->active_valid || (service->queue_count == 0U)) {
        return;
    }
    service->active = service->queue[service->queue_head];
    service->queue_head = (uint8_t)((service->queue_head + 1U) %
                                    ZDT_TURNTABLE_SERVICE_QUEUE_DEPTH);
    service->queue_count--;
    service->active_valid = true;
    service->deadline_ms = service->config.now_ms(service->config.time_ctx) +
                           service->active.timeout_ms;

    /* 快速设备可能在TX结束前回复，因此始终先发布RX DMA窗口。 */
    status = service->config.port.rx_start(
        service->config.port.ctx,
        service->rx_buffer,
        sizeof(service->rx_buffer));
    if (status == ZDT_TURNTABLE_OK) {
        status = service->config.port.tx_start(
            service->config.port.ctx,
            service->active.frame,
            service->active.frame_len);
    }
    if (status != ZDT_TURNTABLE_OK) {
        (void)service->config.port.abort(service->config.port.ctx);
        zdt_complete(service, status, NULL);
    }
}

/** @copydoc zdt_turntable_service_init() */
zdt_turntable_status_t zdt_turntable_service_init(
    zdt_turntable_service_t *service,
    const zdt_turntable_service_config_t *config)
{
    if ((service == NULL) || (config == NULL) ||
        (config->port.tx_start == NULL) ||
        (config->port.rx_start == NULL) ||
        (config->port.abort == NULL) || (config->now_ms == NULL)) {
        return ZDT_TURNTABLE_ERR_PARAM;
    }
    (void)memset(service, 0, sizeof(*service));
    service->config = *config;
    service->initialized = true;
    return ZDT_TURNTABLE_OK;
}

/** @copydoc zdt_turntable_service_submit() */
zdt_turntable_status_t zdt_turntable_service_submit(
    zdt_turntable_service_t *service,
    const zdt_turntable_request_t *request)
{
    if ((service == NULL) || (request == NULL) ||
        !service->initialized || (request->frame_len == 0U) ||
        (request->frame_len > ZDT_TURNTABLE_FRAME_MAX) ||
        (request->timeout_ms == 0U)) {
        return ZDT_TURNTABLE_ERR_PARAM;
    }
    if (service->queue_count >= ZDT_TURNTABLE_SERVICE_QUEUE_DEPTH) {
        return ZDT_TURNTABLE_ERR_QUEUE_FULL;
    }
    service->queue[service->queue_tail] = *request;
    service->queue_tail = (uint8_t)((service->queue_tail + 1U) %
                                    ZDT_TURNTABLE_SERVICE_QUEUE_DEPTH);
    service->queue_count++;
    zdt_notify(service);
    return ZDT_TURNTABLE_OK;
}

/** @copydoc zdt_turntable_service_process_once() */
void zdt_turntable_service_process_once(zdt_turntable_service_t *service)
{
    zdt_turntable_response_t response;
    zdt_turntable_status_t status;
    uint16_t rx_len;

    if ((service == NULL) || !service->initialized) {
        return;
    }
    if (service->active_valid && service->uart_error) {
        /* 错误优先于同一轮观察到的RX事件，避免解析受损帧。 */
        (void)service->config.port.abort(service->config.port.ctx);
        zdt_complete(service, ZDT_TURNTABLE_ERR_IO, NULL);
    } else if (service->active_valid && service->rx_ready) {
        /* ISR写入rx_len后才置rx_ready；普通上下文只消费一次该事件。 */
        rx_len = service->rx_len;
        service->rx_ready = false;
        status = turn_parse(
            service->rx_buffer,
            rx_len,
            service->active.expected_address,
            service->active.expected_function,
            &response);
        (void)service->config.port.abort(service->config.port.ctx);
        zdt_complete(service, status,
                     (status == ZDT_TURNTABLE_OK ||
                      status == ZDT_TURNTABLE_ERR_DEVICE) ? &response : NULL);
    } else if (service->active_valid && zdt_time_reached(
                   service->config.now_ms(service->config.time_ctx),
                   service->deadline_ms)) {
        /* 超时后必须abort，隔离迟到DMA/IDLE事件与下一笔事务。 */
        (void)service->config.port.abort(service->config.port.ctx);
        zdt_complete(service, ZDT_TURNTABLE_ERR_TIMEOUT, NULL);
    }
    zdt_start_next(service);
}

/** @copydoc zdt_turntable_service_on_tx_complete_isr() */
void zdt_turntable_service_on_tx_complete_isr(zdt_turntable_service_t *service)
{
    if ((service != NULL) && service->active_valid) {
        service->tx_done = true;
        zdt_notify(service);
    }
}

/** @copydoc zdt_turntable_service_on_rx_event_isr() */
void zdt_turntable_service_on_rx_event_isr(
    zdt_turntable_service_t *service, uint16_t rx_len)
{
    if ((service != NULL) && service->active_valid) {
        service->rx_len = rx_len;
        service->rx_ready = true;
        zdt_notify(service);
    }
}

/** @copydoc zdt_turntable_service_on_error_isr() */
void zdt_turntable_service_on_error_isr(zdt_turntable_service_t *service)
{
    if ((service != NULL) && service->active_valid) {
        service->uart_error = true;
        zdt_notify(service);
    }
}
