/**
 * @file    lsc16_service_os.c
 * @brief   LSC16 CMSIS-RTOS2队列、worker与公共UART回调路由装配。
 */

#include "lsc16_service_os.h"

#include <string.h>

#include "cmsis_os.h"
#include "uart_dispatch.h"

#define LSC16_OS_FLAG_REQUEST           (1UL << 0)
#define LSC16_OS_FLAG_ISR_EVENT         (1UL << 1)
#define LSC16_OS_QUEUE_DEPTH            4U
#define LSC16_OS_WORKER_STACK           (512U * 4U)

struct lsc16_service_os {
    bool initialized;
    lsc16_t device;
    lsc16_service_t service;
    osMessageQueueId_t request_queue;
    osThreadId_t worker_thread;
    uart_dispatch_handle_t dispatch_handle;
    bool dispatch_registered;
};

static lsc16_service_os_t g_lsc16_service_os;

static const osThreadAttr_t g_lsc16_worker_attr = {
    .name = "lsc16Svc",
    .stack_size = LSC16_OS_WORKER_STACK,
    .priority = (osPriority_t)osPriorityNormal,
};

/**
 * @brief 将毫秒等待时间向上换算为CMSIS-RTOS2 tick。
 * @param timeout_ms 毫秒等待时间。
 * @return 饱和后的tick数量。
 */
static uint32_t lsc16_os_ms_to_ticks(uint32_t timeout_ms)
{
    uint64_t ticks;
    uint32_t frequency;

    if ((timeout_ms == 0U) || (timeout_ms == osWaitForever)) {
        return timeout_ms;
    }
    frequency = osKernelGetTickFreq();
    ticks = ((uint64_t)timeout_ms * frequency + 999ULL) / 1000ULL;
    if (ticks == 0ULL) {
        ticks = 1ULL;
    }
    return (ticks > UINT32_MAX) ? UINT32_MAX : (uint32_t)ticks;
}

/**
 * @brief 从ISR通知入口唤醒LSC16 worker。
 * @param user_ctx OS适配上下文。
 * @warning 只设置线程标志，不在ISR中解析协议或调用用户回调。
 */
static void lsc16_os_notify_worker(void *user_ctx)
{
    lsc16_service_os_t *ctx = (lsc16_service_os_t *)user_ctx;

    /* Service已经登记事件序号；OS adapter只负责唤醒阻塞中的worker。 */
    if ((ctx != NULL) && (ctx->worker_thread != NULL)) {
        (void)osThreadFlagsSet(ctx->worker_thread, LSC16_OS_FLAG_ISR_EVENT);
    }
}

/**
 * @brief 把TX完成事件转交给机械臂BSP。
 * @param user_ctx OS适配上下文。
 * @param huart 产生事件的UART句柄。
 * @return 事件属于UART8时返回true。
 */
static bool lsc16_os_dispatch_tx(void *user_ctx, UART_HandleTypeDef *huart)
{
    lsc16_service_os_t *ctx = (lsc16_service_os_t *)user_ctx;
    return (ctx != NULL) && lsc16_handle_tx_complete(&ctx->device, huart);
}

/**
 * @brief 把ReceiveToIdle事件转交给机械臂BSP。
 * @param user_ctx OS适配上下文。
 * @param huart 产生事件的UART句柄。
 * @param rx_len 本次DMA有效字节数。
 * @return 事件属于UART8时返回true。
 */
static bool lsc16_os_dispatch_rx(
    void *user_ctx,
    UART_HandleTypeDef *huart,
    uint16_t rx_len)
{
    lsc16_service_os_t *ctx = (lsc16_service_os_t *)user_ctx;
    return (ctx != NULL) &&
        lsc16_handle_rx_event(&ctx->device, huart, rx_len);
}

/**
 * @brief 把UART错误事件转交给机械臂BSP。
 * @param user_ctx OS适配上下文。
 * @param huart 产生错误的UART句柄。
 * @return 事件属于UART8时返回true。
 */
static bool lsc16_os_dispatch_error(void *user_ctx, UART_HandleTypeDef *huart)
{
    lsc16_service_os_t *ctx = (lsc16_service_os_t *)user_ctx;
    return (ctx != NULL) && lsc16_handle_error(&ctx->device, huart);
}

/**
 * @brief 按初始化逆序撤销LSC16 OS适配资源。
 * @param ctx OS适配上下文。
 * @note 支持从任意初始化失败点安全回滚。
 */
static void lsc16_os_rollback(lsc16_service_os_t *ctx)
{
    if (ctx->dispatch_registered) {
        (void)uart_dispatch_unregister(ctx->dispatch_handle);
    }
    if (ctx->service.initialized && !ctx->service.active_valid &&
        (ctx->service.queue_count == 0U)) {
        (void)lsc16_service_deinit(&ctx->service);
    }
    if (ctx->device.initialized) {
        (void)lsc16_deinit(&ctx->device);
    }
    if (ctx->request_queue != NULL) {
        (void)osMessageQueueDelete(ctx->request_queue);
    }
    (void)memset(ctx, 0, sizeof(*ctx));
}

/**
 * @brief 将CMSIS请求队列搬入平台无关Service队列。
 * @param ctx OS适配上下文。
 * @note Service拒绝请求时立即通过完成回调报告失败，避免请求丢失。
 */
static void lsc16_os_drain_queue(lsc16_service_os_t *ctx)
{
    lsc16_request_t request;
    lsc16_status_t status;

    while (ctx->service.queue_count < LSC16_SERVICE_QUEUE_DEPTH) {
        if (osMessageQueueGet(ctx->request_queue, &request, NULL, 0U) != osOK) {
            return;
        }
        status = lsc16_service_submit(&ctx->service, &request);
        if (status != LSC16_OK) {
            if (request.done_cb != NULL) {
                request.done_cb(
                    request.user_ctx,
                    request.request_id,
                    status);
            }
        }
    }
}

/**
 * @brief LSC16 Service唯一worker任务入口。
 * @param argument OS适配上下文。
 * @note 所有请求推进、协议解析和用户回调均在本任务上下文执行。
 */
static void lsc16_os_worker_entry(void *argument)
{
    lsc16_service_os_t *ctx = (lsc16_service_os_t *)argument;

    for (;;) {
        lsc16_os_drain_queue(ctx);
        lsc16_service_process_once(&ctx->service);
        (void)osThreadFlagsWait(
            LSC16_OS_FLAG_REQUEST | LSC16_OS_FLAG_ISR_EVENT,
            osFlagsWaitAny,
            osWaitForever);
    }
}

/**
 * @brief 装配机械臂BSP、公共路由、Service、队列和worker。
 * @param report_cb 控制板主动回报回调，允许为NULL。
 * @param report_ctx 原样传给report_cb的上下文。
 * @return 全部资源建立成功返回LSC16_OK，否则回滚并返回错误。
 */
lsc16_status_t lsc16_service_os_init(
    lsc16_report_fn_t report_cb,
    void *report_ctx)
{
    lsc16_service_os_t *ctx = &g_lsc16_service_os;
    lsc16_service_config_t service_config;
    uart_dispatch_handler_t handler = {0};
    lsc16_status_t status;

    if (ctx->initialized) {
        return LSC16_ERR_STATE;
    }
    (void)memset(ctx, 0, sizeof(*ctx));
    ctx->dispatch_handle = UART_DISPATCH_HANDLE_INVALID;
    ctx->request_queue = osMessageQueueNew(
        LSC16_OS_QUEUE_DEPTH,
        sizeof(lsc16_request_t),
        NULL);
    if (ctx->request_queue == NULL) {
        return LSC16_ERR_IO;
    }

    status = lsc16_init(&ctx->device);
    if (status != LSC16_OK) {
        lsc16_os_rollback(ctx);
        return status;
    }

    (void)memset(&service_config, 0, sizeof(service_config));
    service_config.device = &ctx->device;
    service_config.report_cb = report_cb;
    service_config.report_ctx = report_ctx;
    service_config.notify_cb = lsc16_os_notify_worker;
    service_config.notify_ctx = ctx;
    status = lsc16_service_init(&ctx->service, &service_config);
    if (status != LSC16_OK) {
        lsc16_os_rollback(ctx);
        return status;
    }
    handler.tx_complete = lsc16_os_dispatch_tx;
    handler.rx_event = lsc16_os_dispatch_rx;
    handler.error = lsc16_os_dispatch_error;
    handler.user_ctx = ctx;
    if (!uart_dispatch_register(&handler, &ctx->dispatch_handle)) {
        lsc16_os_rollback(ctx);
        return LSC16_ERR_IO;
    }
    ctx->dispatch_registered = true;

    ctx->worker_thread = osThreadNew(
        lsc16_os_worker_entry,
        ctx,
        &g_lsc16_worker_attr);
    if (ctx->worker_thread == NULL) {
        lsc16_os_rollback(ctx);
        return LSC16_ERR_IO;
    }
    ctx->initialized = true;
    return LSC16_OK;
}

/**
 * @brief 按值复制LSC16请求到CMSIS队列并唤醒worker。
 * @param request 待提交请求。
 * @param queue_timeout_ms 等待队列空位的最长毫秒数。
 * @return 队列提交状态。
 */
lsc16_status_t lsc16_service_os_submit(
    const lsc16_request_t *request,
    uint32_t queue_timeout_ms)
{
    osStatus_t os_status;

    if (!g_lsc16_service_os.initialized) {
        return (request == NULL) ? LSC16_ERR_PARAM : LSC16_ERR_NOT_INIT;
    }
    if (request == NULL) {
        return LSC16_ERR_PARAM;
    }
    os_status = osMessageQueuePut(
        g_lsc16_service_os.request_queue,
        request,
        0U,
        lsc16_os_ms_to_ticks(queue_timeout_ms));
    if (os_status != osOK) {
        return ((os_status == osErrorResource) || (os_status == osErrorTimeout)) ?
            LSC16_ERR_QUEUE_FULL : LSC16_ERR_IO;
    }
    if (g_lsc16_service_os.worker_thread != NULL) {
        (void)osThreadFlagsSet(
            g_lsc16_service_os.worker_thread,
            LSC16_OS_FLAG_REQUEST);
    }
    return LSC16_OK;
}

/**
 * @brief 手动推进一次OS队列和平台无关Service。
 * @note 供确定性测试使用；正常运行由worker自动调用。
 */
void lsc16_service_os_process_once(void)
{
    if (g_lsc16_service_os.initialized) {
        lsc16_os_drain_queue(&g_lsc16_service_os);
        lsc16_service_process_once(&g_lsc16_service_os.service);
    }
}

/**
 * @brief 获取LSC16 Service统计快照。
 * @param stats 接收统计值的输出对象。
 * @return 获取结果。
 */
lsc16_status_t lsc16_service_os_get_stats(lsc16_service_stats_t *stats)
{
    if (!g_lsc16_service_os.initialized) {
        return LSC16_ERR_NOT_INIT;
    }
    return lsc16_service_get_stats(&g_lsc16_service_os.service, stats);
}

lsc16_service_os_t *lsc16_service_os_get_default(void)
{
    return &g_lsc16_service_os;
}
