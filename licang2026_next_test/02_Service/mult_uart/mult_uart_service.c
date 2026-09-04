/**
 * @file    mult_uart_service.c
 * @brief   平台无关的 mult_uart 请求队列与 worker 推进引擎。
 *
 * 本文件只依赖 04_Bsp/mult_uart 暴露的 Core API，不直接包含 STM32 HAL
 * 或 RTOS。当前版本可以被 PC fake 单元测试直接调用；后续接入 FreeRTOS
 * 时，CMSIS adapter 只需要负责“什么时候调用 process_once()、如何把
 * 应用请求安全送进 submit()”。
 */

#include "mult_uart_service.h"

#include <string.h>

/**
 * @brief 默认时间源。
 *
 * 没有真实 tick 时返回 0，适合“不启用超时”的纯逻辑测试。若请求配置了
 * io_timeout_ms，又没有传入真实 now_ms，时间不会前进，因此不会触发超时。
 */
static uint32_t mult_uart_service_default_now(void *time_ctx)
{
    (void)time_ctx;
    return 0U;
}

/**
 * @brief 校验通道枚举是否落在 Core 支持范围内。
 */
static bool mult_uart_service_channel_is_valid(mult_uart_channel_t channel)
{
    return ((uint32_t)channel < MULT_UART_CHANNEL_COUNT);
}

/**
 * @brief 判断 32 bit tick 是否已经到期。
 *
 * 使用有符号差值比较可以自然处理 uint32_t 回绕，这是嵌入式超时判断里
 * 很常见的写法：只要单次超时时间小于 2^31 tick，就不会被回绕影响。
 */
static bool mult_uart_service_time_reached(uint32_t now, uint32_t deadline)
{
    return (int32_t)(now - deadline) >= 0;
}

/**
 * @brief 根据操作类型判断是否需要启动 TX DMA。
 */
static bool mult_uart_service_op_needs_tx(mult_uart_operation_t operation)
{
    return (operation == MULT_UART_OP_WRITE) ||
           (operation == MULT_UART_OP_WRITE_READ);
}

/**
 * @brief 根据操作类型判断是否需要启动 RX DMA。
 */
static bool mult_uart_service_op_needs_rx(mult_uart_operation_t operation)
{
    return (operation == MULT_UART_OP_READ) ||
           (operation == MULT_UART_OP_WRITE_READ);
}

/**
 * @brief 校验操作枚举，避免非法值进入状态机。
 */
static bool mult_uart_service_op_is_valid(mult_uart_operation_t operation)
{
    return ((operation == MULT_UART_OP_SELECT) ||
            (operation == MULT_UART_OP_WRITE) ||
            (operation == MULT_UART_OP_READ) ||
            (operation == MULT_UART_OP_WRITE_READ));
}

/**
 * @brief 统一读取时间源，避免各处直接访问函数指针。
 */
static uint32_t mult_uart_service_now(const mult_uart_service_t *service)
{
    return service->now_ms(service->time_ctx);
}

/**
 * @brief 把 service 对象恢复到未初始化状态。
 *
 * reset 只在 init 失败或 deinit 时使用；正常 stop 不调用它，因为统计信息
 * 和对象绑定关系仍需要保留，方便调试和再次 start。
 */
static void mult_uart_service_reset(mult_uart_service_t *service)
{
    (void)memset(service, 0, sizeof(*service));
    service->state = MULT_UART_SERVICE_STATE_UNINIT;
}

/**
 * @brief 固定长度环形队列的下一个下标。
 */
static size_t mult_uart_service_queue_next(size_t index)
{
    ++index;
    if (index >= MULT_UART_SERVICE_QUEUE_DEPTH) {
        index = 0U;
    }
    return index;
}

/**
 * @brief 更新队列水位峰值，用于判断队列深度是否够用。
 */
#if MULT_UART_SERVICE_DIAGNOSTICS_ENABLE
static void mult_uart_service_update_high_watermark(
    mult_uart_service_t *service)
{
    if (service->queue_count > service->stats.queue_high_watermark) {
        service->stats.queue_high_watermark = (uint32_t)service->queue_count;
    }
}
#endif

/**
 * @brief 清空 active job 相关的异步事件暂存。
 *
 * 每次启动新 job 前必须清空，避免上一笔事务的 TX/RX/error 事件污染
 * 当前事务；完成/取消后也清空，方便下一笔从干净状态开始。
 */
static void mult_uart_service_clear_events(mult_uart_service_t *service)
{
    service->events.tx_done = false;
    service->events.rx_done = false;
    service->events.error = false;
    service->events.rx_len = 0U;
    service->events.error_status = MULT_UART_OK;
    service->events.port_error = 0U;
}

/**
 * @brief 通知外层 worker 有新事件需要处理。
 *
 * Service 本身不包含 RTOS 类型，因此不知道“唤醒线程”具体怎么做；它只在
 * Core 事件到达后调用抽象 notify_cb。PC fake 测试可以不配置该回调。
 */
static void mult_uart_service_notify(mult_uart_service_t *service)
{
    if (service->notify_cb != NULL) {
        service->notify_cb(service->notify_ctx);
    }
}

/**
 * @brief 把完成状态归类到统计计数。
 */
#if MULT_UART_SERVICE_DIAGNOSTICS_ENABLE
static void mult_uart_service_count_status(
    mult_uart_service_t *service,
    mult_uart_operation_t operation,
    mult_uart_status_t status)
{
    if (status == MULT_UART_OK) {
        return;
    }

    if (status == MULT_UART_ERR_TIMEOUT) {
        service->stats.timeout++;
    } else if (status == MULT_UART_ERR_OVERFLOW) {
        service->stats.overflow++;
    } else if (status == MULT_UART_ERR_CANCELLED) {
        service->stats.cancelled++;
    } else if (operation == MULT_UART_OP_READ) {
        service->stats.rx_error++;
    } else if ((operation == MULT_UART_OP_WRITE) ||
               (operation == MULT_UART_OP_WRITE_READ)) {
        service->stats.tx_error++;
    }
}
#endif

/**
 * @brief 统一生成 completion 并调用上层回调。
 *
 * completion.rx_data 指向 job 内部缓冲区，只在回调期间有效。失败、超时、
 * 取消时不暴露 RX 指针，避免上层误读旧数据或半包数据。
 */
static void mult_uart_service_complete_job(
    mult_uart_service_t *service,
    mult_uart_service_job_t *job,
    mult_uart_status_t status,
    size_t rx_len)
{
    mult_uart_completion_t completion;

#if !MULT_UART_SERVICE_DIAGNOSTICS_ENABLE
    (void)service;
#endif

    completion.request_id = job->request_id;
    completion.status = status;
    completion.operation = job->operation;
    completion.channel = job->channel;
    completion.rx_data =
        ((status == MULT_UART_OK) &&
         mult_uart_service_op_needs_rx(job->operation)) ? job->rx_data : NULL;
    completion.rx_len = (status == MULT_UART_OK) ? rx_len : 0U;

#if MULT_UART_SERVICE_DIAGNOSTICS_ENABLE
    service->stats.completed++;
    mult_uart_service_count_status(service, job->operation, status);
#endif

    if (job->done_cb != NULL) {
        job->done_cb(job->user_ctx, &completion);
    }
}

/**
 * @brief 在入队前检查请求契约。
 *
 * 这里把“操作类型、通道、TX/RX 缓冲长度”集中验证，避免后续状态机中
 * 到处散落参数判断。OVERFLOW 与 PARAM 分开返回，方便应用区分“调用错了”
 * 和“请求尺寸超过本模块固定资源上限”。
 */
static mult_uart_status_t mult_uart_service_validate_request(
    const mult_uart_request_t *request)
{
    if (request == NULL) {
        return MULT_UART_ERR_PARAM;
    }

    if (!mult_uart_service_op_is_valid(request->operation) ||
        !mult_uart_service_channel_is_valid(request->channel)) {
        return MULT_UART_ERR_PARAM;
    }

    if (mult_uart_service_op_needs_tx(request->operation)) {
        if ((request->tx_data == NULL) || (request->tx_len == 0U)) {
            return MULT_UART_ERR_PARAM;
        }
        if (request->tx_len > MULT_UART_SERVICE_TX_MAX) {
            return MULT_UART_ERR_OVERFLOW;
        }
    } else if ((request->tx_data != NULL) || (request->tx_len != 0U)) {
        return MULT_UART_ERR_PARAM;
    }

    if (mult_uart_service_op_needs_rx(request->operation)) {
        if ((request->rx_capacity == 0U) ||
            (request->rx_capacity > MULT_UART_SERVICE_RX_MAX)) {
            return MULT_UART_ERR_OVERFLOW;
        }
    } else if (request->rx_capacity != 0U) {
        return MULT_UART_ERR_PARAM;
    }

    return MULT_UART_OK;
}

/**
 * @brief 把应用请求复制成内部 job。
 *
 * submit 返回后，应用侧 tx_data 生命周期不再影响这笔事务；这是 Service
 * 层存在的一个关键价值：隔离应用缓冲区与 DMA 实际启动时间。
 */
static void mult_uart_service_make_job(
    mult_uart_service_t *service,
    mult_uart_service_job_t *job,
    const mult_uart_request_t *request)
{
    (void)memset(job, 0, sizeof(*job));
    job->request_id = request->request_id;
    job->operation = request->operation;
    job->channel = request->channel;
    job->tx_len = request->tx_len;
    job->rx_capacity = request->rx_capacity;
    job->io_timeout_ms = request->io_timeout_ms;
    job->deadline_ms = mult_uart_service_now(service) +
                       request->io_timeout_ms;
    job->done_cb = request->done_cb;
    job->user_ctx = request->user_ctx;

    if (request->tx_len > 0U) {
        (void)memcpy(job->tx_data, request->tx_data, request->tx_len);
    }
}

/**
 * @brief Core 事件回调，只做轻量事件登记。
 *
 * 这个回调可能被 HAL ISR handler 间接触发，因此不在这里启动下一笔请求、
 * 不调用用户回调、不做复杂状态迁移，只把 TX/RX/error 结果压缩成事件位。
 */
static void mult_uart_service_event_callback(
    void *user_ctx,
    const mult_uart_event_t *event)
{
    mult_uart_service_t *service = (mult_uart_service_t *)user_ctx;

    if ((service == NULL) || (event == NULL)) {
        return;
    }

    if (event->type == MULT_UART_EVENT_TX_COMPLETE) {
        service->events.tx_done = true;
    } else if (event->type == MULT_UART_EVENT_RX_COMPLETE) {
        service->events.rx_len = event->rx_len;
        service->events.rx_done = true;
    } else {
        service->events.error_status = event->status;
        service->events.port_error = event->port_error;
        service->events.error = true;
    }

    mult_uart_service_notify(service);
}

/**
 * @brief 确保 Core bus 已切到目标通道并处于使能状态。
 *
 * 如果已经在目标通道，就不重复切换；如果需要切换，则交给 Core 执行
 * break-before-switch，Service 只统计一次真实通道切换。
 */
static mult_uart_status_t mult_uart_service_select_channel(
    mult_uart_service_t *service,
    mult_uart_channel_t channel)
{
    mult_uart_channel_t current;
    mult_uart_status_t status;

    status = mult_uart_get_channel(service->bus, &current);
    if (status != MULT_UART_OK) {
        return status;
    }

    if (current != channel) {
        status = mult_uart_select(service->bus, channel);
        if (status != MULT_UART_OK) {
            return status;
        }
#if MULT_UART_SERVICE_DIAGNOSTICS_ENABLE
        service->stats.switch_count++;
#endif
    }

    return mult_uart_enable(service->bus);
}

/**
 * @brief 将 active_job 真正提交给 Core 执行。
 *
 * READ/WRITE_READ 会先启动 RX，再启动 TX；如果 TX 启动失败且 RX 已经启动，
 * 这里必须 abort，把 Core 和 HAL adapter 拉回一致的空闲/可恢复状态。
 */
static mult_uart_status_t mult_uart_service_start_active(
    mult_uart_service_t *service)
{
    mult_uart_service_job_t *job = &service->active_job;
    mult_uart_status_t status;

    mult_uart_service_clear_events(service);

    status = mult_uart_service_select_channel(service, job->channel);
    if (status != MULT_UART_OK) {
        return status;
    }

    if (job->operation == MULT_UART_OP_SELECT) {
        return MULT_UART_OK;
    }

    if (mult_uart_service_op_needs_rx(job->operation)) {
        status = mult_uart_start_rx_dma(
            service->bus,
            job->rx_data,
            job->rx_capacity);
        if (status != MULT_UART_OK) {
            return status;
        }
    }

    if (mult_uart_service_op_needs_tx(job->operation)) {
        status = mult_uart_start_tx_dma(
            service->bus,
            job->tx_data,
            job->tx_len);
        if (status != MULT_UART_OK) {
            if (mult_uart_service_op_needs_rx(job->operation)) {
                (void)mult_uart_abort(service->bus);
            }
            return status;
        }
    }

    return MULT_UART_OK;
}

/**
 * @brief 判断 active_job 所需的异步事件是否已经全部到齐。
 */
static bool mult_uart_service_active_complete(
    const mult_uart_service_t *service)
{
    const mult_uart_service_job_t *job = &service->active_job;

    if (mult_uart_service_op_needs_tx(job->operation) &&
        !service->events.tx_done) {
        return false;
    }

    if (mult_uart_service_op_needs_rx(job->operation) &&
        !service->events.rx_done) {
        return false;
    }

    return true;
}

/**
 * @brief 结束 active_job，并清理 active 状态。
 */
static void mult_uart_service_finish_active(
    mult_uart_service_t *service,
    mult_uart_status_t status,
    size_t rx_len)
{
    mult_uart_service_complete_job(
        service,
        &service->active_job,
        status,
        rx_len);
    service->active = false;
    (void)memset(&service->active_job, 0, sizeof(service->active_job));
    mult_uart_service_clear_events(service);
}

/**
 * @brief 检查 active_job 的 error、timeout 和正常完成条件。
 *
 * 这就是 worker engine 的“收敛点”：ISR 只设置事件，本函数在普通上下文
 * 中根据事件决定 abort、完成回调和状态清理。
 */
static void mult_uart_service_poll_active(mult_uart_service_t *service)
{
    uint32_t now;
    mult_uart_status_t status;

    if (!service->active) {
        return;
    }

    if (service->events.error) {
        status = service->events.error_status;
        if (status == MULT_UART_OK) {
            status = MULT_UART_ERR_IO;
        }
        (void)mult_uart_abort(service->bus);
        mult_uart_service_finish_active(service, status, 0U);
        return;
    }

    if ((service->active_job.io_timeout_ms > 0U)) {
        now = mult_uart_service_now(service);
        if (mult_uart_service_time_reached(
                now,
                service->active_job.deadline_ms)) {
            status = mult_uart_abort(service->bus);
            if (status == MULT_UART_OK) {
                status = MULT_UART_ERR_TIMEOUT;
            }
            mult_uart_service_finish_active(service, status, 0U);
            return;
        }
    }

    if (mult_uart_service_active_complete(service)) {
        mult_uart_service_finish_active(
            service,
            MULT_UART_OK,
            service->events.rx_done ? service->events.rx_len : 0U);
    }
}

/**
 * @brief 从环形队列取出下一笔 job。
 */
static bool mult_uart_service_dequeue(
    mult_uart_service_t *service,
    mult_uart_service_job_t *job)
{
    if (service->queue_count == 0U) {
        return false;
    }

    *job = service->queue[service->queue_head];
    service->queue_head = mult_uart_service_queue_next(service->queue_head);
    service->queue_count--;
    return true;
}

/**
 * @brief 如果当前没有 active_job，就尝试启动队首请求。
 *
 * SELECT 请求没有 DMA 异步完成事件，因此通道切换成功后可立即完成。
 */
static void mult_uart_service_start_next(mult_uart_service_t *service)
{
    mult_uart_status_t status;

    if (service->active) {
        return;
    }

    if (!mult_uart_service_dequeue(service, &service->active_job)) {
        return;
    }

    service->active = true;
    status = mult_uart_service_start_active(service);
    if (status == MULT_UART_OK) {
        if (service->active_job.operation == MULT_UART_OP_SELECT) {
            mult_uart_service_finish_active(service, MULT_UART_OK, 0U);
        }
    } else {
        mult_uart_service_finish_active(service, status, 0U);
    }
}

/**
 * @brief 初始化 Service 并绑定 Core 事件回调。
 */
mult_uart_status_t mult_uart_service_init(
    mult_uart_service_t *service,
    const mult_uart_service_config_t *config)
{
    mult_uart_status_t status;

    if ((service == NULL) || (config == NULL) || (config->bus == NULL)) {
        return MULT_UART_ERR_PARAM;
    }

    if (service->initialized) {
        return MULT_UART_ERR_STATE;
    }

    mult_uart_service_reset(service);
    service->bus = config->bus;
    service->now_ms = (config->now_ms != NULL) ?
        config->now_ms : mult_uart_service_default_now;
    service->time_ctx = config->time_ctx;
    service->notify_cb = config->notify_cb;
    service->notify_ctx = config->notify_ctx;

    status = mult_uart_bind_event(
        service->bus,
        mult_uart_service_event_callback,
        service);
    if (status != MULT_UART_OK) {
        mult_uart_service_reset(service);
        return status;
    }

    service->initialized = true;
    service->state = MULT_UART_SERVICE_STATE_READY;
    return MULT_UART_OK;
}

/**
 * @brief 进入 RUNNING 状态，允许 submit 和 process_once 推进。
 */
mult_uart_status_t mult_uart_service_start(mult_uart_service_t *service)
{
    if (service == NULL) {
        return MULT_UART_ERR_PARAM;
    }

    if (!service->initialized) {
        return MULT_UART_ERR_NOT_INIT;
    }

    if (service->state != MULT_UART_SERVICE_STATE_READY) {
        return MULT_UART_ERR_STATE;
    }

    service->running = true;
    service->state = MULT_UART_SERVICE_STATE_RUNNING;
    return MULT_UART_OK;
}

/**
 * @brief 提交请求到内部队列。
 *
 * 当前 P4a 版本没有加锁；在 PC fake 和单线程轮询中是安全的。接入 RTOS
 * 后，CMSIS adapter 需要保证 submit 与 worker 队列访问不会并发破坏，
 * 例如由同一个 worker 任务串行处理或在外层加临界区。
 */
mult_uart_status_t mult_uart_service_submit(
    mult_uart_service_t *service,
    const mult_uart_request_t *request)
{
    mult_uart_status_t status;
    mult_uart_service_job_t job;

    if (service == NULL) {
        return MULT_UART_ERR_PARAM;
    }

    if (!service->initialized) {
        return MULT_UART_ERR_NOT_INIT;
    }

    if (!service->running ||
        (service->state != MULT_UART_SERVICE_STATE_RUNNING)) {
        return MULT_UART_ERR_STATE;
    }

    status = mult_uart_service_validate_request(request);
    if (status != MULT_UART_OK) {
#if MULT_UART_SERVICE_DIAGNOSTICS_ENABLE
        service->stats.invalid_request++;
        if (status == MULT_UART_ERR_OVERFLOW) {
            service->stats.overflow++;
        }
#endif
        return status;
    }

    if (service->queue_count >= MULT_UART_SERVICE_QUEUE_DEPTH) {
#if MULT_UART_SERVICE_DIAGNOSTICS_ENABLE
        service->stats.queue_full++;
#endif
        return MULT_UART_ERR_QUEUE_FULL;
    }

    mult_uart_service_make_job(service, &job, request);
    service->queue[service->queue_tail] = job;
    service->queue_tail = mult_uart_service_queue_next(service->queue_tail);
    service->queue_count++;
#if MULT_UART_SERVICE_DIAGNOSTICS_ENABLE
    service->stats.submitted++;
    mult_uart_service_update_high_watermark(service);
#endif
    return MULT_UART_OK;
}

/**
 * @brief 推进一次 Service 状态机。
 */
void mult_uart_service_process_once(mult_uart_service_t *service)
{
    if ((service == NULL) || !service->running ||
        (service->state != MULT_UART_SERVICE_STATE_RUNNING)) {
        return;
    }

    mult_uart_service_poll_active(service);
    mult_uart_service_start_next(service);
}

/**
 * @brief 停止 Service，并把 active/queued 请求以 CANCELLED 完成。
 *
 * stop 的目标是让系统回到 READY：之后可以再次 start，而不是销毁对象。
 */
mult_uart_status_t mult_uart_service_stop(mult_uart_service_t *service)
{
    mult_uart_service_job_t job;

    if (service == NULL) {
        return MULT_UART_ERR_PARAM;
    }

    if (!service->initialized) {
        return MULT_UART_ERR_NOT_INIT;
    }

    if (service->state == MULT_UART_SERVICE_STATE_READY) {
        return MULT_UART_OK;
    }

    if (service->state != MULT_UART_SERVICE_STATE_RUNNING) {
        return MULT_UART_ERR_STATE;
    }

    service->state = MULT_UART_SERVICE_STATE_STOPPING;
    service->running = false;

    if (service->active) {
        (void)mult_uart_abort(service->bus);
        mult_uart_service_complete_job(
            service,
            &service->active_job,
            MULT_UART_ERR_CANCELLED,
            0U);
        service->active = false;
    }

    while (mult_uart_service_dequeue(service, &job)) {
        mult_uart_service_complete_job(
            service,
            &job,
            MULT_UART_ERR_CANCELLED,
            0U);
    }

    service->queue_head = 0U;
    service->queue_tail = 0U;
    service->queue_count = 0U;
    mult_uart_service_clear_events(service);
    service->state = MULT_UART_SERVICE_STATE_READY;
    return MULT_UART_OK;
}

/**
 * @brief 解除 Core 回调绑定并释放 Service 逻辑所有权。
 */
#if MULT_UART_SERVICE_TEST_API_ENABLE
mult_uart_status_t mult_uart_service_deinit(mult_uart_service_t *service)
{
    if (service == NULL) {
        return MULT_UART_ERR_PARAM;
    }

    if (!service->initialized) {
        return MULT_UART_ERR_NOT_INIT;
    }

    if (service->state == MULT_UART_SERVICE_STATE_RUNNING) {
        return MULT_UART_ERR_BUSY;
    }

    (void)mult_uart_bind_event(service->bus, NULL, NULL);
    mult_uart_service_reset(service);
    return MULT_UART_OK;
}
#endif

/**
 * @brief 复制一份统计快照给调用者。
 */
#if MULT_UART_SERVICE_DIAGNOSTICS_ENABLE
mult_uart_status_t mult_uart_service_get_stats(
    const mult_uart_service_t *service,
    mult_uart_service_stats_t *stats)
{
    if ((service == NULL) || (stats == NULL)) {
        return MULT_UART_ERR_PARAM;
    }

    if (!service->initialized) {
        return MULT_UART_ERR_NOT_INIT;
    }

    *stats = service->stats;
    return MULT_UART_OK;
}
#endif
