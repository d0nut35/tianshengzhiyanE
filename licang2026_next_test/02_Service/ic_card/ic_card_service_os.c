/**
 * @file    ic_card_service_os.c
 * @brief   IC卡CMSIS-RTOS2消息队列、worker和UART7回调路由装配。
 */

#include "ic_card_service_os.h"

#include <string.h>

#include "cmsis_os.h"
#include "ic_bsp.h"
#include "uart_dispatch.h"

#if !LICANG_RELEASE_MINIMAL
/* 直连UART7装配只用于独立测试；正式任务统一经mult_uart通道1访问IC卡。 */
#define IC_CARD_OS_FLAG_REQUEST       (1UL << 0)
#define IC_CARD_OS_FLAG_ISR_EVENT     (1UL << 1)
#define IC_CARD_OS_WAIT_SLICE_MS      10U
#define IC_CARD_OS_QUEUE_DEPTH        4U
#define IC_CARD_OS_WORKER_STACK       (512U * 4U)

typedef struct {
    bool initialized;
    ic_card_t device;
    ic_card_port_t port;
    ic_card_uart7_hal_t adapter;
    ic_card_service_t service;
    osMessageQueueId_t request_queue;
    osThreadId_t worker_thread;
    uart_dispatch_handle_t dispatch_handle;
    bool dispatch_registered;
} ic_card_service_os_context_t;

static ic_card_service_os_context_t g_ic_card_service_os;

static const osThreadAttr_t g_ic_card_worker_attr = {
    .name = "icCardSvc",
    .stack_size = IC_CARD_OS_WORKER_STACK,
    .priority = (osPriority_t)osPriorityNormal,
};

/**
 * @brief 将毫秒等待时间向上换算为CMSIS-RTOS2 tick。
 * @param timeout_ms 毫秒等待时间。
 * @return 饱和到UINT32_MAX的tick数量。
 * @note 向上取整可避免短超时因整数截断变成零等待。
 */
static uint32_t ic_card_os_ms_to_ticks(uint32_t timeout_ms)
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
 * @brief 为平台无关Service提供当前毫秒时基。
 * @param ctx 未使用的时间上下文。
 * @return HAL_GetTick()当前值。
 */
static uint32_t ic_card_os_now_ms(void *ctx)
{
    (void)ctx;
    return HAL_GetTick();
}

/**
 * @brief 从ISR通知入口唤醒阻塞的IC卡worker。
 * @param ctx OS适配上下文。
 * @warning 事件内容已经登记在Service序号中，本函数不能阻塞或解析协议。
 */
static void ic_card_os_notify_worker(void *ctx)
{
    ic_card_service_os_context_t *os_ctx =
        (ic_card_service_os_context_t *)ctx;

    if ((os_ctx != NULL) && (os_ctx->worker_thread != NULL)) {
        (void)osThreadFlagsSet(os_ctx->worker_thread, IC_CARD_OS_FLAG_ISR_EVENT);
    }
}

/**
 * @brief 把公共路由的TX完成事件转交给UART7适配器。
 * @param ctx OS适配上下文。
 * @param huart 产生事件的HAL UART句柄。
 * @return UART7适配器认领该事件时返回true。
 */
static bool ic_card_os_dispatch_tx(void *ctx, UART_HandleTypeDef *huart)
{
    ic_card_service_os_context_t *os_ctx =
        (ic_card_service_os_context_t *)ctx;
    return (os_ctx != NULL) && ic_uart7_tx_isr(
        &os_ctx->adapter,
        huart);
}

/**
 * @brief 把公共路由的ReceiveToIdle事件转交给UART7适配器。
 * @param ctx OS适配上下文。
 * @param huart 产生事件的HAL UART句柄。
 * @param rx_len 本次DMA有效字节数。
 * @return UART7适配器认领该事件时返回true。
 */
static bool ic_card_os_dispatch_rx(
    void *ctx,
    UART_HandleTypeDef *huart,
    uint16_t rx_len)
{
    ic_card_service_os_context_t *os_ctx =
        (ic_card_service_os_context_t *)ctx;
    return (os_ctx != NULL) && ic_uart7_rx_isr(
        &os_ctx->adapter,
        huart,
        rx_len);
}

/**
 * @brief 把公共路由的UART错误事件转交给UART7适配器。
 * @param ctx OS适配上下文。
 * @param huart 产生错误的HAL UART句柄。
 * @return UART7适配器认领该事件时返回true。
 */
static bool ic_card_os_dispatch_error(void *ctx, UART_HandleTypeDef *huart)
{
    ic_card_service_os_context_t *os_ctx =
        (ic_card_service_os_context_t *)ctx;
    return (os_ctx != NULL) && ic_uart7_error_isr(
        &os_ctx->adapter,
        huart);
}

/**
 * @brief 按初始化逆序撤销已建立的OS、路由、Service和HAL资源。
 * @param ctx OS适配上下文。
 * @note 只释放标志指示已经成功建立的资源，允许任意中间步骤安全回滚。
 */
static void ic_card_os_rollback(ic_card_service_os_context_t *ctx)
{
    if (ctx->dispatch_registered) {
        (void)uart_dispatch_unregister(ctx->dispatch_handle);
    }
    if (ctx->service.initialized && !ctx->service.active_valid &&
        (ctx->service.queue_count == 0U)) {
        (void)ic_card_service_deinit(&ctx->service);
    }
    if (ctx->device.initialized) {
        (void)ic_bsp_deinit(&ctx->device);
    }
    if (ctx->request_queue != NULL) {
        (void)osMessageQueueDelete(ctx->request_queue);
    }
    (void)memset(ctx, 0, sizeof(*ctx));
}

/**
 * @brief 将CMSIS队列中的请求搬入平台无关Service静态队列。
 * @param ctx OS适配上下文。
 * @note 搬运失败时直接执行该请求完成回调，使请求不会无声丢失。
 */
static void ic_card_os_drain_queue(ic_card_service_os_context_t *ctx)
{
    ic_card_request_t request;
    ic_card_status_t status;

    while (ctx->service.queue_count < IC_CARD_SERVICE_QUEUE_DEPTH) {
        if (osMessageQueueGet(ctx->request_queue, &request, NULL, 0U) != osOK) {
            return;
        }
        status = ic_card_service_submit(&ctx->service, &request);
        if ((status != IC_CARD_OK) && (request.done_cb != NULL)) {
            request.done_cb(
                request.user_ctx,
                request.request_id,
                status,
                NULL);
        }
    }
}

/**
 * @brief IC卡Service唯一worker任务入口。
 * @param argument OS适配上下文。
 * @note worker按10ms上限周期醒来，使“设备完全不回包”的超时也能推进；ISR
 *       和新请求会提前唤醒。协议解析与用户回调均在本任务上下文中执行。
 */
static void ic_card_os_worker_entry(void *argument)
{
    ic_card_service_os_context_t *ctx =
        (ic_card_service_os_context_t *)argument;

    for (;;) {
        ic_card_os_drain_queue(ctx);
        ic_card_service_process_once(&ctx->service);
        (void)osThreadFlagsWait(
            IC_CARD_OS_FLAG_REQUEST | IC_CARD_OS_FLAG_ISR_EVENT,
            osFlagsWaitAny,
            ic_card_os_ms_to_ticks(IC_CARD_OS_WAIT_SLICE_MS));
    }
}

/**
 * @brief 装配UART7 HAL适配、公共路由、Service、消息队列和worker。
 * @return 全部资源建立成功返回IC_CARD_OK，否则回滚并返回具体错误。
 */
ic_card_status_t ic_card_service_os_init(void)
{
    ic_card_service_os_context_t *ctx = &g_ic_card_service_os;
    ic_card_uart7_hal_config_t hal_config;
    ic_card_service_config_t service_config;
    uart_dispatch_handler_t handler = {0};
    ic_card_status_t status;

    if (ctx->initialized) {
        return IC_CARD_ERR_STATE;
    }
    (void)memset(ctx, 0, sizeof(*ctx));
    ctx->dispatch_handle = UART_DISPATCH_HANDLE_INVALID;
    ctx->request_queue = osMessageQueueNew(
        IC_CARD_OS_QUEUE_DEPTH,
        sizeof(ic_card_request_t),
        NULL);
    if (ctx->request_queue == NULL) {
        return IC_CARD_ERR_IO;
    }

    ic_uart7_config(&hal_config);
    status = ic_uart7_bind(
        &ctx->adapter,
        &ctx->device,
        &hal_config,
        &ctx->port);
    if (status != IC_CARD_OK) {
        ic_card_os_rollback(ctx);
        return status;
    }

    /* 先注册路由再启动RX DMA，防止初始化窗口内到达的数据无人认领。 */
    handler.tx_complete = ic_card_os_dispatch_tx;
    handler.rx_event = ic_card_os_dispatch_rx;
    handler.error = ic_card_os_dispatch_error;
    handler.user_ctx = ctx;
    if (!uart_dispatch_register(&handler, &ctx->dispatch_handle)) {
        ic_card_os_rollback(ctx);
        return IC_CARD_ERR_IO;
    }
    ctx->dispatch_registered = true;

    status = ic_bsp_init(&ctx->device, &ctx->port);
    if (status != IC_CARD_OK) {
        ic_card_os_rollback(ctx);
        return status;
    }

    (void)memset(&service_config, 0, sizeof(service_config));
    service_config.device = &ctx->device;
    service_config.now_ms = ic_card_os_now_ms;
    service_config.notify_worker = ic_card_os_notify_worker;
    service_config.notify_ctx = ctx;
    status = ic_card_service_init(&ctx->service, &service_config);
    if (status != IC_CARD_OK) {
        ic_card_os_rollback(ctx);
        return status;
    }

    ctx->worker_thread = osThreadNew(
        ic_card_os_worker_entry,
        ctx,
        &g_ic_card_worker_attr);
    if (ctx->worker_thread == NULL) {
        ic_card_os_rollback(ctx);
        return IC_CARD_ERR_IO;
    }
    ctx->initialized = true;
    return IC_CARD_OK;
}

/**
 * @brief 把请求按值复制到CMSIS队列并唤醒worker。
 * @param request 待提交请求。
 * @param queue_timeout_ms 等待OS队列空位的最长毫秒数。
 * @return 队列提交状态。
 * @warning 请求中的回调上下文仍由调用者负责保持到最终回调完成。
 */
ic_card_status_t ic_card_service_os_submit(
    const ic_card_request_t *request,
    uint32_t queue_timeout_ms)
{
    osStatus_t os_status;

    if (request == NULL) {
        return IC_CARD_ERR_PARAM;
    }
    if (!g_ic_card_service_os.initialized) {
        return IC_CARD_ERR_NOT_INIT;
    }
    os_status = osMessageQueuePut(
        g_ic_card_service_os.request_queue,
        request,
        0U,
        ic_card_os_ms_to_ticks(queue_timeout_ms));
    if (os_status != osOK) {
        return ((os_status == osErrorResource) || (os_status == osErrorTimeout)) ?
            IC_CARD_ERR_QUEUE_FULL : IC_CARD_ERR_IO;
    }
    (void)osThreadFlagsSet(
        g_ic_card_service_os.worker_thread,
        IC_CARD_OS_FLAG_REQUEST);
    return IC_CARD_OK;
}

/**
 * @brief 手动推进一次OS队列和平台无关Service。
 * @note 供确定性测试或调试使用；正常FreeRTOS运行由worker自动调用。
 */
void ic_card_service_os_process_once(void)
{
    if (g_ic_card_service_os.initialized) {
        ic_card_os_drain_queue(&g_ic_card_service_os);
        ic_card_service_process_once(&g_ic_card_service_os.service);
    }
}

/**
 * @brief 获取IC卡Service统计快照。
 * @param stats 接收统计值的输出对象。
 * @return 获取结果。
 */
ic_card_status_t ic_card_service_os_get_stats(ic_card_service_stats_t *stats)
{
    if (!g_ic_card_service_os.initialized) {
        return IC_CARD_ERR_NOT_INIT;
    }
    return ic_card_service_get_stats(&g_ic_card_service_os.service, stats);
}
#endif
