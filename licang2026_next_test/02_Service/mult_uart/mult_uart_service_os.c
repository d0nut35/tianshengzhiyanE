/**
 * @file    mult_uart_service_os.c
 * @brief   mult_uart Service 的CMSIS-RTOS2/FreeRTOS worker接入实现。
 *
 * 应用任务只向OS队列提交请求；唯一worker串行访问平台无关Service。
 * UART ISR经公共uart_dispatch路由到本模块，ISR只登记事件并唤醒worker，
 * 不在中断中执行abort、协议解析或用户完成回调。
 */

#include "mult_uart_service_os.h"

#include <string.h>

#include "cmsis_os.h"
#include "uart_dispatch.h"

#define MULT_UART_SERVICE_OS_FLAG_REQUEST    (1UL << 0)
#define MULT_UART_SERVICE_OS_FLAG_EVENT      (1UL << 1)
#define MULT_UART_SERVICE_OS_WAIT_TICKS      1U
#define MULT_UART_SERVICE_OS_QUEUE_DEPTH     4U
#define MULT_UART_SERVICE_OS_WORKER_STACK    (512U * 4U)

/**
 * @brief OS队列真正复制的消息。
 *
 * 参考F4版本只把mult_uart_request_t放入OS队列，其中tx_data仍指向调用者
 * 内存；worker稍后再复制时，调用者栈缓冲可能已经失效。F7移植改为在
 * submit返回前复制TX字节，彻底兑现异步接口的数据所有权契约。
 */
typedef struct {
    uint32_t request_id;
    mult_uart_operation_t operation;
    mult_uart_channel_t channel;
    uint8_t tx_data[MULT_UART_SERVICE_TX_MAX];
    size_t tx_len;
    size_t rx_capacity;
    uint32_t io_timeout_ms;
    mult_uart_done_fn_t done_cb;
    void *user_ctx;
} mult_uart_service_os_request_t;

/**
 * @brief RTOS接入层拥有的一整套默认单例资源。
 *
 * 定义留在.c中，防止HAL/CMSIS具体类型沿公共头文件泄漏到Device层。
 */
struct mult_uart_service_os {
    bool initialized;
#if MULT_UART_SERVICE_OS_DIAGNOSTICS_ENABLE
    volatile uint32_t worker_loop_count;
    volatile uint32_t os_submit_count;
    volatile uint32_t os_dequeue_count;
    volatile uint32_t notify_error_count;
#endif
    mult_uart_bus_t bus;
    mult_uart_service_t service;
    osMessageQueueId_t request_queue;
    osThreadId_t worker_thread;
    uart_dispatch_handle_t dispatch_handle;
    bool dispatch_registered;
};

/**
 * @brief mult_uart唯一worker任务入口，定义位于文件末尾。
 * @param argument OS装配对象。
 */
static void mult_uart_service_os_worker_entry(void *argument);

/**
 * @brief Core事件的ISR唤醒出口，定义位于下方。
 * @param user_ctx OS装配对象。
 * @warning 在ISR路径执行，只允许发送线程标志，不得阻塞。
 */
static void mult_uart_service_os_notify(void *user_ctx);

static mult_uart_service_os_t g_mult_uart_service_os;

static const osThreadAttr_t g_mult_uart_worker_attr = {
    .name = "multUart",
    .stack_size = MULT_UART_SERVICE_OS_WORKER_STACK,
    /* 串口总线状态机必须先于普通App/测试任务处理请求和DMA完成事件。 */
    .priority = (osPriority_t)osPriorityAboveNormal,
};

/**
 * @brief 将毫秒等待时间向上换算为CMSIS-RTOS2 tick。
 * @param timeout_ms 毫秒等待时间或osWaitForever。
 * @return 饱和后的tick数量。
 */
static uint32_t mult_uart_service_os_ms_to_ticks(uint32_t timeout_ms)
{
    uint64_t ticks;
    uint32_t freq;

    if ((timeout_ms == 0U) || (timeout_ms == osWaitForever)) {
        return timeout_ms;
    }

    freq = osKernelGetTickFreq();
    ticks = ((uint64_t)timeout_ms * (uint64_t)freq + 999ULL) / 1000ULL;
    if (ticks == 0ULL) {
        ticks = 1ULL;
    }
    if (ticks > (uint64_t)UINT32_MAX) {
        ticks = UINT32_MAX;
    }
    return (uint32_t)ticks;
}

/**
 * @brief 把RTOS tick换算成Core使用的毫秒时间基准。
 * @param time_ctx 未使用的时间上下文。
 * @return 当前毫秒数，溢出时饱和到UINT32_MAX。
 */
static uint32_t mult_uart_service_os_now_ms(void *time_ctx)
{
    uint64_t ms;
    uint32_t tick;
    uint32_t freq;

    (void)time_ctx;
    tick = osKernelGetTickCount();
    freq = osKernelGetTickFreq();
    if (freq == 0U) {
        return 0U;
    }

    ms = ((uint64_t)tick * 1000ULL) / (uint64_t)freq;
    return (ms > (uint64_t)UINT32_MAX) ? UINT32_MAX : (uint32_t)ms;
}

/**
 * @brief 把CMSIS-RTOS2状态映射为mult_uart统一错误码。
 * @param status CMSIS-RTOS2返回值。
 * @return 对应的mult_uart状态。
 * @note OS错误类型在此边界收敛，不泄漏到平台无关接口。
 */
static mult_uart_status_t mult_uart_service_os_map_status(osStatus_t status)
{
    if (status == osOK) {
        return MULT_UART_OK;
    }
    if ((status == osErrorTimeout) || (status == osErrorResource)) {
        return MULT_UART_ERR_QUEUE_FULL;
    }
    if (status == osErrorParameter) {
        return MULT_UART_ERR_PARAM;
    }
    return MULT_UART_ERR_IO;
}

/**
 * @brief 判断操作是否需要TX数据。
 * @param operation 事务操作类型。
 * @return WRITE或WRITE_READ返回true。
 */
static bool mult_uart_service_os_op_needs_tx(mult_uart_operation_t operation)
{
    return (operation == MULT_UART_OP_WRITE) ||
           (operation == MULT_UART_OP_WRITE_READ);
}

/**
 * @brief 判断操作是否需要RX缓冲区。
 * @param operation 事务操作类型。
 * @return READ或WRITE_READ返回true。
 */
static bool mult_uart_service_os_op_needs_rx(mult_uart_operation_t operation)
{
    return (operation == MULT_UART_OP_READ) ||
           (operation == MULT_UART_OP_WRITE_READ);
}

/**
 * @brief 在复制TX前完成尺寸与指针校验。
 *
 * 平台无关Service仍会再次校验完整请求；这里的重复检查是OS所有权边界
 * 所必需的，防止以非法长度读取调用者缓冲区。
 */
static mult_uart_status_t mult_uart_service_os_make_queue_item(
    const mult_uart_request_t *request,
    mult_uart_service_os_request_t *item)
{
    bool needs_tx;
    bool needs_rx;

    if ((request == NULL) || (item == NULL) ||
        ((uint32_t)request->operation > (uint32_t)MULT_UART_OP_WRITE_READ) ||
        ((uint32_t)request->channel >= MULT_UART_CHANNEL_COUNT)) {
        return MULT_UART_ERR_PARAM;
    }

    needs_tx = mult_uart_service_os_op_needs_tx(request->operation);
    needs_rx = mult_uart_service_os_op_needs_rx(request->operation);

    if (needs_tx) {
        if ((request->tx_data == NULL) || (request->tx_len == 0U)) {
            return MULT_UART_ERR_PARAM;
        }
        if (request->tx_len > MULT_UART_SERVICE_TX_MAX) {
            return MULT_UART_ERR_OVERFLOW;
        }
    } else if ((request->tx_data != NULL) || (request->tx_len != 0U)) {
        return MULT_UART_ERR_PARAM;
    }

    if (needs_rx) {
        if ((request->rx_capacity == 0U) ||
            (request->rx_capacity > MULT_UART_SERVICE_RX_MAX)) {
            return MULT_UART_ERR_OVERFLOW;
        }
    } else if (request->rx_capacity != 0U) {
        return MULT_UART_ERR_PARAM;
    }

    (void)memset(item, 0, sizeof(*item));
    item->request_id = request->request_id;
    item->operation = request->operation;
    item->channel = request->channel;
    item->tx_len = request->tx_len;
    item->rx_capacity = request->rx_capacity;
    item->io_timeout_ms = request->io_timeout_ms;
    item->done_cb = request->done_cb;
    item->user_ctx = request->user_ctx;
    if (request->tx_len > 0U) {
        (void)memcpy(item->tx_data, request->tx_data, request->tx_len);
    }
    return MULT_UART_OK;
}

/**
 * @brief 把拥有TX副本的OS消息转换为平台无关Service请求。
 * @param item OS队列消息。
 * @param request 接收转换结果的Service请求。
 * @warning request->tx_data指向item内部数组，只能在当前搬运调用链中使用；
 *          Service提交函数会在返回前再次复制TX数据。
 */
static void mult_uart_service_os_make_service_request(
    const mult_uart_service_os_request_t *item,
    mult_uart_request_t *request)
{
    (void)memset(request, 0, sizeof(*request));
    request->request_id = item->request_id;
    request->operation = item->operation;
    request->channel = item->channel;
    request->tx_data = (item->tx_len > 0U) ? item->tx_data : NULL;
    request->tx_len = item->tx_len;
    request->rx_capacity = item->rx_capacity;
    request->io_timeout_ms = item->io_timeout_ms;
    request->done_cb = item->done_cb;
    request->user_ctx = item->user_ctx;
}

/**
 * @brief 为无法搬入Service队列的OS请求构造失败完成事件。
 * @param request 被拒绝的请求。
 * @param status 拒绝原因。
 * @note 统一走完成回调可避免已经进入OS队列的请求无声丢失。
 */
static void mult_uart_service_os_complete_rejected(
    const mult_uart_request_t *request,
    mult_uart_status_t status)
{
    mult_uart_completion_t completion;

    if ((request == NULL) || (request->done_cb == NULL)) {
        return;
    }

    completion.request_id = request->request_id;
    completion.status = status;
    completion.operation = request->operation;
    completion.channel = request->channel;
    completion.rx_data = NULL;
    completion.rx_len = 0U;
    request->done_cb(request->user_ctx, &completion);
}

/**
 * @brief 把OS请求队列搬入平台无关Service静态队列。
 * @param ctx OS适配上下文。
 */
static void mult_uart_service_os_drain_request_queue(
    mult_uart_service_os_t *ctx)
{
    mult_uart_service_os_request_t item;
    mult_uart_request_t request;
    mult_uart_status_t status;

    while (ctx->service.queue_count < MULT_UART_SERVICE_QUEUE_DEPTH) {
        if (osMessageQueueGet(ctx->request_queue, &item, NULL, 0U) != osOK) {
            return;
        }
#if MULT_UART_SERVICE_OS_DIAGNOSTICS_ENABLE
        ctx->os_dequeue_count++;
#endif

        mult_uart_service_os_make_service_request(&item, &request);
        status = mult_uart_service_submit(&ctx->service, &request);
        if (status != MULT_UART_OK) {
            mult_uart_service_os_complete_rejected(&request, status);
        }
    }
}

/**
 * @brief 从Core ISR事件回调唤醒worker。
 * @param user_ctx OS适配上下文。
 * @warning ISR中只设置线程标志，协议和队列操作全部留在worker上下文。
 */
static void mult_uart_service_os_notify(void *user_ctx)
{
    mult_uart_service_os_t *ctx = (mult_uart_service_os_t *)user_ctx;
    uint32_t flags_result;

    if ((ctx != NULL) && (ctx->worker_thread != NULL)) {
        flags_result = osThreadFlagsSet(
            ctx->worker_thread,
            MULT_UART_SERVICE_OS_FLAG_EVENT);
#if MULT_UART_SERVICE_OS_DIAGNOSTICS_ENABLE
        if ((flags_result & osFlagsError) != 0U) {
            ctx->notify_error_count++;
        }
#else
        (void)flags_result;
#endif
    }
}

/**
 * @brief 把公共路由TX完成事件转交给UART7复用BSP。
 * @param user_ctx OS适配上下文。
 * @param huart 产生事件的UART句柄。
 * @return 事件属于本适配器时返回true。
 */
static bool mult_uart_service_os_dispatch_tx(
    void *user_ctx,
    UART_HandleTypeDef *huart)
{
    mult_uart_service_os_t *ctx = (mult_uart_service_os_t *)user_ctx;
    return (ctx != NULL) && mult_uart_handle_tx(
        &ctx->bus,
        huart);
}

/**
 * @brief 把公共路由ReceiveToIdle事件转交给UART7复用BSP。
 * @param user_ctx OS适配上下文。
 * @param huart 产生事件的UART句柄。
 * @param rx_len 本次DMA有效字节数。
 * @return 事件属于本适配器时返回true。
 */
static bool mult_uart_service_os_dispatch_rx(
    void *user_ctx,
    UART_HandleTypeDef *huart,
    uint16_t rx_len)
{
    mult_uart_service_os_t *ctx = (mult_uart_service_os_t *)user_ctx;
    return (ctx != NULL) && mult_uart_handle_rx(
        &ctx->bus,
        huart,
        rx_len);
}

/**
 * @brief 把公共路由UART错误事件转交给UART7复用BSP。
 * @param user_ctx OS适配上下文。
 * @param huart 产生错误的UART句柄。
 * @return 事件属于本适配器时返回true。
 */
static bool mult_uart_service_os_dispatch_error(
    void *user_ctx,
    UART_HandleTypeDef *huart)
{
    mult_uart_service_os_t *ctx = (mult_uart_service_os_t *)user_ctx;
    return (ctx != NULL) && mult_uart_handle_error(
        &ctx->bus,
        huart);
}

/**
 * @brief 测试构建中按初始化逆序撤销复用Service OS资源。
 * @param ctx OS适配上下文。
 * @note 正式任务的启动失败不可恢复，由MX_FREERTOS_Init调用者进入Error_Handler。
 */
#if !LICANG_RELEASE_MINIMAL
static void mult_uart_service_os_rollback(mult_uart_service_os_t *ctx)
{
    if (ctx->dispatch_registered) {
        (void)uart_dispatch_unregister(ctx->dispatch_handle);
        ctx->dispatch_registered = false;
    }
    if (ctx->service.initialized) {
        if (ctx->service.state == MULT_UART_SERVICE_STATE_RUNNING) {
            (void)mult_uart_service_stop(&ctx->service);
        }
        (void)mult_uart_service_deinit(&ctx->service);
    }
    if (ctx->bus.initialized) {
        (void)mult_uart_deinit(&ctx->bus);
    }
    if (ctx->request_queue != NULL) {
        (void)osMessageQueueDelete(ctx->request_queue);
    }
    (void)memset(ctx, 0, sizeof(*ctx));
}
#endif

/**
 * @brief 装配UART7复用BSP、Service、路由、队列和worker。
 * @return 全部资源建立成功返回MULT_UART_OK，否则返回具体错误；测试构建会回滚。
 */
mult_uart_status_t mult_uart_service_os_init(void)
{
    mult_uart_service_os_t *ctx = &g_mult_uart_service_os;
    mult_uart_service_config_t service_config = {0};
    uart_dispatch_handler_t dispatch_handler = {0};
    mult_uart_status_t status;

    if (ctx->initialized) {
        return MULT_UART_ERR_STATE;
    }

    (void)memset(ctx, 0, sizeof(*ctx));
    ctx->dispatch_handle = UART_DISPATCH_HANDLE_INVALID;
    ctx->request_queue = osMessageQueueNew(
        MULT_UART_SERVICE_OS_QUEUE_DEPTH,
        sizeof(mult_uart_service_os_request_t),
        NULL);
    if (ctx->request_queue == NULL) {
        return MULT_UART_ERR_IO;
    }

    status = mult_uart_init(&ctx->bus);
    if (status != MULT_UART_OK) {
#if !LICANG_RELEASE_MINIMAL
        mult_uart_service_os_rollback(ctx);
#endif
        return status;
    }

    service_config.bus = &ctx->bus;
    service_config.now_ms = mult_uart_service_os_now_ms;
    service_config.notify_cb = mult_uart_service_os_notify;
    service_config.notify_ctx = ctx;
    status = mult_uart_service_init(&ctx->service, &service_config);
    if (status != MULT_UART_OK) {
#if !LICANG_RELEASE_MINIMAL
        mult_uart_service_os_rollback(ctx);
#endif
        return status;
    }

    status = mult_uart_service_start(&ctx->service);
    if (status != MULT_UART_OK) {
#if !LICANG_RELEASE_MINIMAL
        mult_uart_service_os_rollback(ctx);
#endif
        return status;
    }

    dispatch_handler.tx_complete = mult_uart_service_os_dispatch_tx;
    dispatch_handler.rx_event = mult_uart_service_os_dispatch_rx;
    dispatch_handler.error = mult_uart_service_os_dispatch_error;
    dispatch_handler.user_ctx = ctx;
    if (!uart_dispatch_register(&dispatch_handler, &ctx->dispatch_handle)) {
#if !LICANG_RELEASE_MINIMAL
        mult_uart_service_os_rollback(ctx);
#endif
        return MULT_UART_ERR_IO;
    }
    ctx->dispatch_registered = true;

    ctx->worker_thread = osThreadNew(
        mult_uart_service_os_worker_entry,
        ctx,
        &g_mult_uart_worker_attr);
    if (ctx->worker_thread == NULL) {
#if !LICANG_RELEASE_MINIMAL
        mult_uart_service_os_rollback(ctx);
#endif
        return MULT_UART_ERR_IO;
    }

    ctx->initialized = true;
    return MULT_UART_OK;
}

/**
 * @brief 在调用返回前复制TX数据并提交到CMSIS队列。
 * @param request 待提交请求。
 * @param queue_timeout_ms 等待OS队列空位的最长毫秒数。
 * @return 队列提交状态。
 */
mult_uart_status_t mult_uart_service_os_submit(
    const mult_uart_request_t *request,
    uint32_t queue_timeout_ms)
{
    mult_uart_service_os_t *ctx = &g_mult_uart_service_os;
    mult_uart_service_os_request_t item;
    mult_uart_status_t status;
    osStatus_t os_status;

    if (!ctx->initialized || (ctx->request_queue == NULL)) {
        return (request == NULL) ? MULT_UART_ERR_PARAM :
            MULT_UART_ERR_NOT_INIT;
    }

    status = mult_uart_service_os_make_queue_item(request, &item);
    if (status != MULT_UART_OK) {
        return status;
    }

    os_status = osMessageQueuePut(
        ctx->request_queue,
        &item,
        0U,
        mult_uart_service_os_ms_to_ticks(queue_timeout_ms));
    if (os_status != osOK) {
        return mult_uart_service_os_map_status(os_status);
    }
#if MULT_UART_SERVICE_OS_DIAGNOSTICS_ENABLE
    ctx->os_submit_count++;
#endif

    if (ctx->worker_thread != NULL) {
        uint32_t flags_result = osThreadFlagsSet(
            ctx->worker_thread,
            MULT_UART_SERVICE_OS_FLAG_REQUEST);
#if MULT_UART_SERVICE_OS_DIAGNOSTICS_ENABLE
        if ((flags_result & osFlagsError) != 0U) {
            ctx->notify_error_count++;
        }
#else
        (void)flags_result;
#endif
    }
    return MULT_UART_OK;
}

/**
 * @brief 手动搬运OS队列并推进一次平台无关Service。
 * @note 供确定性测试使用；正常FreeRTOS运行由worker自动调用。
 */
void mult_uart_service_os_process_once(void)
{
    mult_uart_service_os_t *ctx = &g_mult_uart_service_os;

    if (ctx->initialized) {
        mult_uart_service_os_drain_request_queue(ctx);
        mult_uart_service_process_once(&ctx->service);
    }
}

/**
 * @brief 获取复用Service统计快照。
 * @param stats 接收统计值的输出对象。
 * @return 获取结果。
 */
#if MULT_UART_SERVICE_OS_DIAGNOSTICS_ENABLE
mult_uart_status_t mult_uart_service_os_get_stats(
    mult_uart_service_stats_t *stats)
{
    if (!g_mult_uart_service_os.initialized) {
        return MULT_UART_ERR_NOT_INIT;
    }
    return mult_uart_service_get_stats(
        &g_mult_uart_service_os.service,
        stats);
}

/** @copydoc mult_uart_service_os_get_diagnostics() */
mult_uart_status_t mult_uart_service_os_get_diagnostics(
    mult_uart_service_os_diagnostics_t *diagnostics)
{
    mult_uart_service_os_t *ctx = &g_mult_uart_service_os;

    if (diagnostics == NULL) {
        return MULT_UART_ERR_PARAM;
    }
    if (!ctx->initialized) {
        return MULT_UART_ERR_NOT_INIT;
    }

    (void)memset(diagnostics, 0, sizeof(*diagnostics));
    diagnostics->worker_loop_count = ctx->worker_loop_count;
    diagnostics->os_submit_count = ctx->os_submit_count;
    diagnostics->os_dequeue_count = ctx->os_dequeue_count;
    diagnostics->os_queue_count = osMessageQueueGetCount(ctx->request_queue);
    diagnostics->notify_error_count = ctx->notify_error_count;
    diagnostics->service_submit_count = ctx->service.stats.submitted;
    diagnostics->service_complete_count = ctx->service.stats.completed;
    diagnostics->uart_error_count = ctx->bus.uart_error_count;
    diagnostics->last_uart_error = ctx->bus.last_uart_error;
    return MULT_UART_OK;
}

/**
 * @brief 获取默认复用Service OS单例。
 * @return 单例地址；仅用于系统装配和诊断，不转移所有权。
 */
mult_uart_service_os_t *mult_uart_service_os_get_default(void)
{
    return &g_mult_uart_service_os;
}
#endif

/**
 * @brief mult_uart唯一worker任务入口。
 * @param argument OS适配上下文；当前使用默认单例，因此函数内无需解引用。
 * @note 单worker保证公共UART链路始终只有一笔在途事务。
 */
static void mult_uart_service_os_worker_entry(void *argument)
{
    (void)argument;

    /* 单worker保证公共UART链路始终只有一笔在途事务。 */
    for (;;) {
#if MULT_UART_SERVICE_OS_DIAGNOSTICS_ENABLE
        g_mult_uart_service_os.worker_loop_count++;
#endif
        mult_uart_service_os_process_once();
        (void)osThreadFlagsWait(
            MULT_UART_SERVICE_OS_FLAG_REQUEST |
                MULT_UART_SERVICE_OS_FLAG_EVENT,
            osFlagsWaitAny,
            MULT_UART_SERVICE_OS_WAIT_TICKS);
    }
}
