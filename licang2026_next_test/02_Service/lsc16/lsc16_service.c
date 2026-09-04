/**
 * @file    lsc16_service.c
 * @brief   LSC16平台无关串行请求Service实现。
 */

#include "lsc16_service.h"

#include <string.h>

/**
 * @brief 校验LSC16请求类型及舵机参数范围。
 * @param request 待检查请求。
 * @return 请求可以安全转换为Core命令时返回true。
 */
static bool lsc16_service_request_is_valid(const lsc16_request_t *request)
{
    uint8_t i;

    if ((request == NULL) ||
        ((uint32_t)request->type > (uint32_t)LSC16_REQUEST_GET_BATTERY)) {
        return false;
    }
    if (request->type == LSC16_REQUEST_MOVE_SERVOS) {
        if ((request->data.move.count == 0U) ||
            (request->data.move.count > LSC16_SERVO_COUNT_MAX)) {
            return false;
        }
        for (i = 0U; i < request->data.move.count; ++i) {
            if ((request->data.move.targets[i].id > LSC16_SERVO_ID_MAX) ||
                (request->data.move.targets[i].position < LSC16_SERVO_POSITION_MIN) ||
                (request->data.move.targets[i].position > LSC16_SERVO_POSITION_MAX)) {
                return false;
            }
        }
    }
    return true;
}

/**
 * @brief 接收Core ISR事件并唤醒Service worker。
 * @param user_ctx 绑定的Service对象。
 * @param event TX完成、RX就绪或UART错误事件。
 * @warning 本函数运行在ISR上下文，只递增事件序号并通知worker。
 */
static void lsc16_service_isr_notify(
    void *user_ctx,
    lsc16_isr_event_t event)
{
    lsc16_service_t *service = (lsc16_service_t *)user_ctx;

    if (service == NULL) {
        return;
    }
    if (event == LSC16_ISR_EVENT_TX_COMPLETE) {
        ++service->tx_complete_sequence;
    } else if (event == LSC16_ISR_EVENT_RX_READY) {
        ++service->rx_ready_sequence;
    } else {
        ++service->error_sequence;
    }
    if (service->config.notify_cb != NULL) {
        service->config.notify_cb(service->config.notify_ctx);
    }
}

/**
 * @brief 收敛当前活动请求并发布最终完成回调。
 * @param service Service对象。
 * @param status 当前请求的最终状态。
 * @note 先释放active状态再回调，允许上层从回调安全提交下一动作。
 */
static void lsc16_service_complete_active(
    lsc16_service_t *service,
    lsc16_status_t status)
{
    lsc16_request_t completed;

    if (!service->active_valid) {
        return;
    }
    completed = service->active;
    service->active_valid = false;
    ++service->stats.completed;
    if (status != LSC16_OK) {
        ++service->stats.io_errors;
    }
    if (completed.done_cb != NULL) {
        completed.done_cb(
            completed.user_ctx,
            completed.request_id,
            status);
    }
}

/**
 * @brief 把统一Service请求翻译为对应的LSC16 Core API。
 * @param service Service对象。
 * @param request 当前活动请求。
 * @return Core命令启动状态。
 * @note UART8访问仍由Core的tx_busy状态串行化。
 */
static lsc16_status_t lsc16_service_start_request(
    lsc16_service_t *service,
    const lsc16_request_t *request)
{
    switch (request->type) {
    case LSC16_REQUEST_MOVE_SERVOS:
        return lsc16_move_servos(
            service->config.device,
            request->data.move.targets,
            request->data.move.count,
            request->data.move.move_time_ms);
    case LSC16_REQUEST_RUN_ACTION_GROUP:
        return lsc16_run_action_group(
            service->config.device,
            request->data.action_run.action_group,
            request->data.action_run.repeat_count);
    case LSC16_REQUEST_STOP_ACTION_GROUP:
        return lsc16_stop_action_group(service->config.device);
    case LSC16_REQUEST_SET_ACTION_SPEED:
        return lsc16_set_action_group_speed(
            service->config.device,
            request->data.action_speed.action_group,
            request->data.action_speed.speed_percent);
    case LSC16_REQUEST_GET_BATTERY:
        return lsc16_request_battery_voltage(service->config.device);
    default:
        return LSC16_ERR_PARAM;
    }
}

/**
 * @brief 解析并发布控制板主动回报事件。
 * @param service Service对象。
 * @note 主动回报与DMA发送完成是两条独立事件流，不能互相替代。
 */
static void lsc16_service_publish_reports(lsc16_service_t *service)
{
    uint32_t events;
    lsc16_report_t report;

    lsc16_process(service->config.device);
    events = lsc16_take_report_events(service->config.device);
    if (events == LSC16_REPORT_EVENT_NONE) {
        return;
    }
    ++service->stats.reports;
    if ((service->config.report_cb != NULL) &&
        (lsc16_get_last_report(service->config.device, &report) == LSC16_OK)) {
        service->config.report_cb(service->config.report_ctx, events, &report);
    }
}

/**
 * @brief 初始化平台无关LSC16 Service并绑定Core ISR通知。
 * @param service 待初始化Service对象。
 * @param config Core对象及通知/回报配置。
 * @return 初始化结果。
 */
lsc16_status_t lsc16_service_init(
    lsc16_service_t *service,
    const lsc16_service_config_t *config)
{
    lsc16_status_t status;

    if ((service == NULL) || (config == NULL) || (config->device == NULL) ||
        !config->device->initialized) {
        return LSC16_ERR_PARAM;
    }
    (void)memset(service, 0, sizeof(*service));
    service->config = *config;
    status = lsc16_bind_isr_notify(
        config->device,
        lsc16_service_isr_notify,
        service);
    if (status == LSC16_OK) {
        service->initialized = true;
    }
    return status;
}

/**
 * @brief 在Service空闲时解除回调并清空对象。
 * @param service Service对象。
 * @return 成功返回LSC16_OK；有活动或排队请求时返回BUSY。
 */
lsc16_status_t lsc16_service_deinit(lsc16_service_t *service)
{
    if (service == NULL) {
        return LSC16_ERR_PARAM;
    }
    if (!service->initialized) {
        return LSC16_ERR_NOT_INIT;
    }
    if (service->active_valid || (service->queue_count > 0U)) {
        return LSC16_ERR_BUSY;
    }
    (void)lsc16_bind_isr_notify(service->config.device, NULL, NULL);
    (void)memset(service, 0, sizeof(*service));
    return LSC16_OK;
}

/**
 * @brief 按值复制一笔LSC16请求到静态队列。
 * @param service Service对象。
 * @param request 待提交请求。
 * @return 请求入队状态。
 * @note 完整复制请求，异步执行期间不引用调用者栈上的临时变量。
 */
lsc16_status_t lsc16_service_submit(
    lsc16_service_t *service,
    const lsc16_request_t *request)
{
    if ((service == NULL) || !service->initialized) {
        return (service == NULL) ? LSC16_ERR_PARAM : LSC16_ERR_NOT_INIT;
    }
    if (!lsc16_service_request_is_valid(request)) {
        ++service->stats.rejected;
        return LSC16_ERR_PARAM;
    }
    if (service->queue_count >= LSC16_SERVICE_QUEUE_DEPTH) {
        ++service->stats.rejected;
        return LSC16_ERR_QUEUE_FULL;
    }
    /* 完整复制请求，异步执行期间不再引用调用者栈上的临时变量。 */
    service->queue[service->queue_head] = *request;
    service->queue_head = (uint8_t)(
        (service->queue_head + 1U) % LSC16_SERVICE_QUEUE_DEPTH);
    ++service->queue_count;
    ++service->stats.submitted;
    return LSC16_OK;
}

/**
 * @brief 在普通上下文推进一次LSC16请求和主动回报状态机。
 * @param service Service对象。
 * @note UART错误优先于TX完成；单在途请求防止DMA发送缓冲区被覆盖。
 */
void lsc16_service_process_once(lsc16_service_t *service)
{
    lsc16_status_t status;

    if ((service == NULL) || !service->initialized) {
        return;
    }

    /* 序号可保留“消费瞬间ISR再次到达”的事件，比读清布尔标志可靠。 */
    if (service->handled_rx_sequence != service->rx_ready_sequence) {
        service->handled_rx_sequence = service->rx_ready_sequence;
    }
    lsc16_service_publish_reports(service);

    /* UART错误优先于TX完成，避免同一轮里把失败帧误报为成功。 */
    if (service->handled_error_sequence != service->error_sequence) {
        service->handled_error_sequence = service->error_sequence;
        /* 丢弃错误之前可能同时到达的旧TX完成，不能拿它完成下一笔请求。 */
        service->handled_tx_sequence = service->tx_complete_sequence;
        status = lsc16_recover(service->config.device);
        lsc16_service_complete_active(
            service,
            (status == LSC16_OK) ? LSC16_ERR_IO : status);
    } else if (service->handled_tx_sequence != service->tx_complete_sequence) {
        service->handled_tx_sequence = service->tx_complete_sequence;
        lsc16_service_complete_active(service, LSC16_OK);
    }

    /* 单在途帧可防止下一笔请求覆盖 Core 正被 DMA 使用的 TX 缓冲区。 */
    if (service->active_valid || (service->queue_count == 0U)) {
        return;
    }
    service->active = service->queue[service->queue_tail];
    service->queue_tail = (uint8_t)(
        (service->queue_tail + 1U) % LSC16_SERVICE_QUEUE_DEPTH);
    --service->queue_count;
    service->active_valid = true;

    status = lsc16_service_start_request(service, &service->active);
    if (status != LSC16_OK) {
        lsc16_service_complete_active(service, status);
    }
}

/**
 * @brief 获取LSC16 Service统计快照。
 * @param service Service对象。
 * @param stats 接收统计值的输出对象。
 * @return 获取结果。
 */
lsc16_status_t lsc16_service_get_stats(
    const lsc16_service_t *service,
    lsc16_service_stats_t *stats)
{
    if ((service == NULL) || (stats == NULL)) {
        return LSC16_ERR_PARAM;
    }
    if (!service->initialized) {
        return LSC16_ERR_NOT_INIT;
    }
    *stats = service->stats;
    return LSC16_OK;
}
