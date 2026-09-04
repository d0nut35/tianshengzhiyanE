/**
 * @file    ic_card_service.c
 * @brief   IC卡读写器请求排队、响应匹配和超时恢复实现。
 */

#include "ic_card_service.h"

#include <string.h>

/**
 * @brief 判断32位毫秒期限是否已经到达。
 * @param now 当前毫秒计数。
 * @param deadline 截止毫秒计数。
 * @return 到期返回true，否则返回false。
 * @note 使用有符号差值以兼容计数器自然回绕。
 */
static bool ic_card_time_reached(uint32_t now, uint32_t deadline)
{
    return ((int32_t)(now - deadline) >= 0);
}

/**
 * @brief 校验请求能否转换为当前支持的厂家命令。
 * @param request 待检查请求。
 * @return 请求及超时参数有效时返回true。
 * @note 提交前拒绝坏请求，避免无效事务长期占用UART7。
 */
static bool ic_card_request_is_valid(const ic_card_request_t *request)
{
    if ((request == NULL) || (request->timeout_ms == 0U)) {
        return false;
    }
    if (request->type == IC_CARD_REQUEST_READ_BLOCK) {
        return true;
    }
    if (request->type == IC_CARD_REQUEST_QUERY) {
        return ((request->data.query.command == IC_CARD_CMD_QUERY_ADDRESS) ||
                (request->data.query.command == IC_CARD_CMD_QUERY_WORK_MODE) ||
                (request->data.query.command == IC_CARD_CMD_QUERY_BEEPER) ||
                (request->data.query.command == IC_CARD_CMD_QUERY_AUTO_READ));
    }
    return false;
}

/**
 * @brief 接收Core ISR事件并唤醒Service worker。
 * @param user_ctx 绑定的Service对象。
 * @param event Core报告的TX、RX或错误事件。
 * @warning 本函数运行在ISR上下文，只更新单调序号并通知worker，禁止解析和回调用户代码。
 */
static void ic_card_service_isr_notify(
    void *user_ctx,
    ic_card_isr_event_t event)
{
    ic_card_service_t *service = (ic_card_service_t *)user_ctx;

    if (service == NULL) {
        return;
    }
    if (event == IC_CARD_ISR_EVENT_TX_COMPLETE) {
        ++service->tx_sequence;
    } else if (event == IC_CARD_ISR_EVENT_RX_READY) {
        ++service->rx_sequence;
    } else {
        ++service->error_sequence;
    }
    if (service->config.notify_worker != NULL) {
        service->config.notify_worker(service->config.notify_ctx);
    }
}

/**
 * @brief 取得当前请求期望匹配的响应命令号。
 * @param request 当前活动请求。
 * @return A3读块命令号或请求指定的查询命令号。
 */
static uint8_t ic_card_expected_command(const ic_card_request_t *request)
{
    return (request->type == IC_CARD_REQUEST_READ_BLOCK) ?
        IC_CARD_CMD_READ_BLOCK_KEY_A : (uint8_t)request->data.query.command;
}

/**
 * @brief 判断一帧响应是否属于当前活动请求。
 * @param request 当前活动请求。
 * @param response 已通过Core校验的响应。
 * @return 命令号和地址语义匹配时返回true。
 * @note B0查询地址的响应地址位位于payload中，因此不按普通地址字段匹配。
 */
static bool ic_card_response_matches(
    const ic_card_request_t *request,
    const ic_card_response_t *response)
{
    if ((request == NULL) || (response == NULL) ||
        (response->command != ic_card_expected_command(request))) {
        return false;
    }
    if ((request->type == IC_CARD_REQUEST_QUERY) &&
        (request->data.query.command == IC_CARD_CMD_QUERY_ADDRESS)) {
        return true;
    }
    return (response->address == request->address);
}

/**
 * @brief 收敛当前事务状态并发布最终完成回调。
 * @param service Service对象。
 * @param status 最终事务状态。
 * @param response 成功或卡错误时的响应；无响应失败时为NULL。
 * @note 先清除活动请求再调用用户回调，允许回调安全提交下一笔事务。
 * @warning response仅在回调期间有效，需要长期保存时由调用者立即复制。
 */
static void ic_card_service_complete(
    ic_card_service_t *service,
    ic_card_status_t status,
    const ic_card_response_t *response)
{
    ic_card_request_t finished;

    if (!service->active_valid) {
        return;
    }
    finished = service->active;
    service->active_valid = false;
    service->tx_completed = false;
    (void)memset(&service->active, 0, sizeof(service->active));
    if (status == IC_CARD_OK) {
#if IC_CARD_SERVICE_DIAGNOSTICS_ENABLE
        ++service->stats.completed;
#endif
    }
    if (finished.done_cb != NULL) {
        finished.done_cb(
            finished.user_ctx,
            finished.request_id,
            status,
            response);
    }
}

/**
 * @brief 把当前活动请求翻译为Core命令并启动事务超时计时。
 * @param service 已装载active请求的Service对象。
 * @return Core命令启动状态。
 * @note 请求启动后直到匹配响应、错误或超时前独占该直连UART7。
 */
static ic_card_status_t ic_card_service_start_active(
    ic_card_service_t *service)
{
    ic_card_status_t status;

    if (service->active.type == IC_CARD_REQUEST_READ_BLOCK) {
        status = ic_card_read_block_key_a(
            service->config.device,
            service->active.address,
            service->active.data.read_block.block,
            service->active.data.read_block.led_beep_prompt);
    } else {
        status = ic_card_query(
            service->config.device,
            service->active.data.query.command,
            service->active.address);
    }
    if (status == IC_CARD_OK) {
        service->active_deadline_ms = service->config.now_ms(
            service->config.time_ctx) + service->active.timeout_ms;
    }
    return status;
}

/**
 * @brief 初始化平台无关IC卡Service并绑定Core ISR通知。
 * @param service 待初始化Service对象。
 * @param config Core对象、时钟和worker通知配置。
 * @return 初始化和回调绑定结果。
 */
ic_card_status_t ic_card_service_init(
    ic_card_service_t *service,
    const ic_card_service_config_t *config)
{
    if ((service == NULL) || (config == NULL) || (config->device == NULL) ||
        (config->now_ms == NULL) || !config->device->initialized) {
        return IC_CARD_ERR_PARAM;
    }
    if (service->initialized) {
        return IC_CARD_ERR_STATE;
    }
    (void)memset(service, 0, sizeof(*service));
    service->config = *config;
    service->response_sequence = config->device->response_sequence;
    service->initialized = true;
    return ic_card_bind_isr_notify(
        config->device,
        ic_card_service_isr_notify,
        service);
}

/**
 * @brief 在空闲状态解除Service与Core的事件绑定。
 * @param service Service对象。
 * @return 成功返回IC_CARD_OK；存在活动或排队请求时返回BUSY。
 */
ic_card_status_t ic_card_service_deinit(ic_card_service_t *service)
{
    if (service == NULL) {
        return IC_CARD_ERR_PARAM;
    }
    if (!service->initialized) {
        return IC_CARD_ERR_NOT_INIT;
    }
    if (service->active_valid || (service->queue_count > 0U)) {
        return IC_CARD_ERR_BUSY;
    }
    (void)ic_card_bind_isr_notify(service->config.device, NULL, NULL);
    (void)memset(service, 0, sizeof(*service));
    return IC_CARD_OK;
}

/**
 * @brief 按值复制一笔请求到平台无关静态队列。
 * @param service Service对象。
 * @param request 待提交请求。
 * @return 请求入队状态。
 * @note 完整复制请求结构，执行期间不再依赖调用者的临时请求对象。
 */
ic_card_status_t ic_card_service_submit(
    ic_card_service_t *service,
    const ic_card_request_t *request)
{
    if ((service == NULL) || !service->initialized) {
        return (service == NULL) ? IC_CARD_ERR_PARAM : IC_CARD_ERR_NOT_INIT;
    }
    if (!ic_card_request_is_valid(request)) {
#if IC_CARD_SERVICE_DIAGNOSTICS_ENABLE
        ++service->stats.rejected;
#endif
        return IC_CARD_ERR_PARAM;
    }
    if (service->queue_count >= IC_CARD_SERVICE_QUEUE_DEPTH) {
#if IC_CARD_SERVICE_DIAGNOSTICS_ENABLE
        ++service->stats.rejected;
#endif
        return IC_CARD_ERR_QUEUE_FULL;
    }
    service->queue[service->queue_head] = *request;
    service->queue_head = (uint8_t)(
        (service->queue_head + 1U) % IC_CARD_SERVICE_QUEUE_DEPTH);
    ++service->queue_count;
#if IC_CARD_SERVICE_DIAGNOSTICS_ENABLE
    ++service->stats.submitted;
#endif
    return IC_CARD_OK;
}

/**
 * @brief 在普通上下文推进一次响应、错误、超时和队列状态机。
 * @param service Service对象。
 * @note UART错误优先于完成事件；任何用户回调和协议匹配都不会在ISR中执行。
 */
void ic_card_service_process_once(ic_card_service_t *service)
{
    ic_card_response_t response;
    ic_card_status_t status;
    uint32_t now;

    if ((service == NULL) || !service->initialized) {
        return;
    }

    ic_card_process(service->config.device);

    /* UART错误优先，避免同一轮同时到达的旧TX/RX事件误完成请求。 */
    if (service->handled_error_sequence != service->error_sequence) {
        service->handled_error_sequence = service->error_sequence;
        service->handled_tx_sequence = service->tx_sequence;
        service->handled_rx_sequence = service->rx_sequence;
#if IC_CARD_SERVICE_DIAGNOSTICS_ENABLE
        ++service->stats.uart_errors;
#endif
        status = ic_card_recover(service->config.device);
        ic_card_service_complete(
            service,
            (status == IC_CARD_OK) ? IC_CARD_ERR_IO : status,
            NULL);
    } else {
        if (service->handled_tx_sequence != service->tx_sequence) {
            service->handled_tx_sequence = service->tx_sequence;
            service->tx_completed = true;
        }
        if (service->handled_rx_sequence != service->rx_sequence) {
            service->handled_rx_sequence = service->rx_sequence;
        }

        while (ic_card_take_response(
            service->config.device,
            &service->response_sequence,
            &response)) {
            if (service->active_valid &&
                ic_card_response_matches(&service->active, &response)) {
                status = (response.device_status == 0U) ?
                    IC_CARD_OK : IC_CARD_ERR_CARD;
                ic_card_service_complete(service, status, &response);
            } else {
#if IC_CARD_SERVICE_DIAGNOSTICS_ENABLE
                ++service->stats.unrelated_responses;
#endif
            }
        }
    }

    if (service->active_valid) {
        now = service->config.now_ms(service->config.time_ctx);
        if (ic_card_time_reached(now, service->active_deadline_ms)) {
#if IC_CARD_SERVICE_DIAGNOSTICS_ENABLE
            ++service->stats.timed_out;
#endif
            status = ic_card_recover(service->config.device);
            ic_card_service_complete(
                service,
                (status == IC_CARD_OK) ? IC_CARD_ERR_TIMEOUT : status,
                NULL);
        }
    }

    if (service->active_valid || (service->queue_count == 0U)) {
        return;
    }
    service->active = service->queue[service->queue_tail];
    service->queue_tail = (uint8_t)(
        (service->queue_tail + 1U) % IC_CARD_SERVICE_QUEUE_DEPTH);
    --service->queue_count;
    service->active_valid = true;
    service->tx_completed = false;
    status = ic_card_service_start_active(service);
    if (status != IC_CARD_OK) {
        ic_card_service_complete(service, status, NULL);
    }
}

/**
 * @brief 获取IC卡Service统计快照。
 * @param service Service对象。
 * @param stats 接收统计值的输出对象。
 * @return 获取结果。
 */
#if IC_CARD_SERVICE_DIAGNOSTICS_ENABLE
ic_card_status_t ic_card_service_get_stats(
    const ic_card_service_t *service,
    ic_card_service_stats_t *stats)
{
    if ((service == NULL) || (stats == NULL)) {
        return IC_CARD_ERR_PARAM;
    }
    if (!service->initialized) {
        return IC_CARD_ERR_NOT_INIT;
    }
    *stats = service->stats;
    return IC_CARD_OK;
}
#endif
