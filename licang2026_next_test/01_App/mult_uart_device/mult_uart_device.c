/**
 * @file    mult_uart_device.c
 * @brief   mult_uart 上层设备事务框架实现。
 *
 * 本文件是 P5 的第一块应用层代码。它不碰 UART/GPIO/DMA，也不自己切通道；
 * 它只把“业务设备”翻译成 Service OS 能执行的请求，并负责把完成回调再翻译
 * 回设备维度。这样后续真实设备协议模块可以围绕 device_id 编写，而不用知道
 * 复用器 A/B 真值、UART7 DMA 或 worker task 的细节。
 */

#include "mult_uart_device.h"

#include <string.h>

#include "mux_service.h"

typedef struct {
    bool in_use;
    uint32_t request_id;
    mult_uart_device_id_t device_id;
    mult_uart_operation_t operation;
    mult_uart_device_done_fn_t done_cb;
    void *user_ctx;
} mult_uart_device_slot_t;

typedef struct {
    bool initialized;
    uint32_t next_request_id;
    mult_uart_device_config_t configs[MULT_UART_DEVICE_COUNT];
    mult_uart_device_slot_t slots[MULT_UART_DEVICE_COUNT];
#if MULT_UART_DEVICE_DIAGNOSTICS_ENABLE
    mult_uart_device_stats_t stats;
#endif
} mult_uart_device_manager_t;

static mult_uart_device_manager_t g_mult_uart_device_manager;

/**
 * @brief 判断 device_id 是否处于本层固定管理范围。
 */
static bool mult_uart_device_id_is_valid(mult_uart_device_id_t device_id)
{
    return ((uint32_t)device_id < MULT_UART_DEVICE_COUNT);
}

/**
 * @brief 生成默认 device->channel 配置。
 *
 * 第一版默认一一映射，后续真实产品若 device 顺序和复用器通道不同，只需要
 * 在 init 时传入配置表，不需要改协议代码。
 */
static void mult_uart_device_make_default_config(
    mult_uart_device_config_t *configs)
{
    size_t i;

    for (i = 0U; i < MULT_UART_DEVICE_COUNT; ++i) {
        configs[i].enabled = true;
        configs[i].channel = (mult_uart_channel_t)i;
        configs[i].default_io_timeout_ms =
            MULT_UART_DEVICE_DEFAULT_IO_MS;
    }
}

/**
 * @brief 复制并校验用户提供的配置表。
 */
static mult_uart_status_t mult_uart_device_apply_config(
    mult_uart_device_manager_t *manager,
    const mult_uart_device_config_t *configs,
    size_t config_count)
{
    size_t i;

    mult_uart_device_make_default_config(manager->configs);
    if (configs == NULL) {
        return MULT_UART_OK;
    }

    if (config_count > MULT_UART_DEVICE_COUNT) {
        return MULT_UART_ERR_PARAM;
    }

    for (i = 0U; i < config_count; ++i) {
        if ((uint32_t)configs[i].channel >= MULT_UART_CHANNEL_COUNT) {
            return MULT_UART_ERR_PARAM;
        }
        manager->configs[i] = configs[i];
    }

    return MULT_UART_OK;
}

/**
 * @brief 分配一条 pending slot。
 *
 * 第一版采用“每个 device 一个 pending slot”的规则。这样同一设备不会出现
 * 两笔未完成请求交错返回，避免协议层还没建立帧序号时难以归属回复。
 */
static mult_uart_status_t mult_uart_device_alloc_slot(
    mult_uart_device_manager_t *manager,
    const mult_uart_device_transfer_t *transfer,
    mult_uart_device_slot_t **slot_out)
{
    mult_uart_device_slot_t *slot;
    uint32_t request_id;

    if (!mult_uart_device_id_is_valid(transfer->device_id)) {
        return MULT_UART_ERR_PARAM;
    }

    slot = &manager->slots[(uint32_t)transfer->device_id];
    if (slot->in_use) {
        return MULT_UART_ERR_BUSY;
    }

    request_id = ++manager->next_request_id;
    if (request_id == 0U) {
        request_id = ++manager->next_request_id;
    }

    (void)memset(slot, 0, sizeof(*slot));
    slot->in_use = true;
    slot->request_id = request_id;
    slot->device_id = transfer->device_id;
    slot->operation = transfer->operation;
    slot->done_cb = transfer->done_cb;
    slot->user_ctx = transfer->user_ctx;
    *slot_out = slot;
    return MULT_UART_OK;
}

/**
 * @brief 释放 pending slot。
 */
static void mult_uart_device_free_slot(mult_uart_device_slot_t *slot)
{
    if (slot != NULL) {
        (void)memset(slot, 0, sizeof(*slot));
    }
}

/**
 * @brief 把底层 Service completion 翻译成 device completion。
 *
 * 注意这里仍在 Service worker task 上下文中执行。真实设备协议若需要较长解析，
 * 后续可以再转发到协议任务；第一版先直接回调，保持链路最短、最容易调试。
 */
static void mult_uart_device_service_done(
    void *user_ctx,
    const mult_uart_completion_t *completion)
{
#if MULT_UART_DEVICE_DIAGNOSTICS_ENABLE
    mult_uart_device_manager_t *manager = &g_mult_uart_device_manager;
#endif
    mult_uart_device_slot_t *slot =
        (mult_uart_device_slot_t *)user_ctx;
    mult_uart_device_completion_t device_completion;
    mult_uart_device_done_fn_t done_cb;
    void *done_ctx;

    if ((slot == NULL) || (completion == NULL) || !slot->in_use) {
        return;
    }

    device_completion.request_id = completion->request_id;
    device_completion.device_id = slot->device_id;
    device_completion.status = completion->status;
    device_completion.operation = completion->operation;
    device_completion.rx_data = completion->rx_data;
    device_completion.rx_len = completion->rx_len;

    done_cb = slot->done_cb;
    done_ctx = slot->user_ctx;
    mult_uart_device_free_slot(slot);
#if MULT_UART_DEVICE_DIAGNOSTICS_ENABLE
    manager->stats.completed++;
#endif

    if (done_cb != NULL) {
        done_cb(done_ctx, &device_completion);
    }
}

/**
 * @brief 把 device transfer 填成底层 Service request。
 */
static void mult_uart_device_make_request(
    const mult_uart_device_manager_t *manager,
    const mult_uart_device_transfer_t *transfer,
    mult_uart_device_slot_t *slot,
    mult_uart_request_t *request)
{
    const mult_uart_device_config_t *config =
        &manager->configs[(uint32_t)transfer->device_id];

    (void)memset(request, 0, sizeof(*request));
    request->request_id = slot->request_id;
    request->operation = transfer->operation;
    request->channel = config->channel;
    request->tx_data = transfer->tx_data;
    request->tx_len = transfer->tx_len;
    request->rx_capacity = transfer->rx_capacity;
    request->io_timeout_ms =
        (transfer->io_timeout_ms != 0U) ?
            transfer->io_timeout_ms : config->default_io_timeout_ms;
    request->done_cb = mult_uart_device_service_done;
    request->user_ctx = slot;
}

/**
 * @brief 初始化Device管理器并加载设备到通道的映射。
 * @param configs 可选配置表；传NULL时生成默认映射。
 * @param config_count 配置元素个数。
 * @return 配置合法时返回MULT_UART_OK，否则返回参数或重复初始化错误。
 */
mult_uart_status_t mult_uart_device_init(
    const mult_uart_device_config_t *configs,
    size_t config_count)
{
    mult_uart_device_manager_t *manager = &g_mult_uart_device_manager;
    mult_uart_status_t status;

    if (manager->initialized) {
        return MULT_UART_ERR_STATE;
    }

    (void)memset(manager, 0, sizeof(*manager));
    status = mult_uart_device_apply_config(manager, configs, config_count);
    if (status != MULT_UART_OK) {
        (void)memset(manager, 0, sizeof(*manager));
        return status;
    }

    manager->initialized = true;
    return MULT_UART_OK;
}

/**
 * @brief 反初始化Device管理器。
 * @return 没有pending事务时返回MULT_UART_OK，否则返回BUSY或未初始化错误。
 * @note 拒绝带未完成事务反初始化，避免下层回调访问已清空的slot。
 */
#if MULT_UART_DEVICE_TEST_API_ENABLE
mult_uart_status_t mult_uart_device_deinit(void)
{
    mult_uart_device_manager_t *manager = &g_mult_uart_device_manager;
    size_t i;

    if (!manager->initialized) {
        return MULT_UART_ERR_NOT_INIT;
    }

    for (i = 0U; i < MULT_UART_DEVICE_COUNT; ++i) {
        if (manager->slots[i].in_use) {
            return MULT_UART_ERR_BUSY;
        }
    }

    (void)memset(manager, 0, sizeof(*manager));
    return MULT_UART_OK;
}
#endif

/**
 * @brief 校验并提交一笔设备级串口事务。
 * @param transfer 设备、操作、收发长度、超时和回调信息。
 * @return 请求成功入队返回MULT_UART_OK，否则返回具体校验或队列错误。
 * @note 同一设备同时只允许一笔pending事务，底层完成后才释放slot。
 */
mult_uart_status_t mult_uart_device_submit(
    const mult_uart_device_transfer_t *transfer)
{
    mult_uart_device_manager_t *manager = &g_mult_uart_device_manager;
    mult_uart_device_slot_t *slot;
    mult_uart_request_t request;
    mult_uart_status_t status;
    const mult_uart_device_config_t *config;
    uint32_t queue_timeout_ms;

    if ((transfer == NULL) || !manager->initialized) {
        return (transfer == NULL) ? MULT_UART_ERR_PARAM :
            MULT_UART_ERR_NOT_INIT;
    }

    if (!mult_uart_device_id_is_valid(transfer->device_id)) {
#if MULT_UART_DEVICE_DIAGNOSTICS_ENABLE
        manager->stats.invalid_request++;
#endif
        return MULT_UART_ERR_PARAM;
    }

    config = &manager->configs[(uint32_t)transfer->device_id];
    if (!config->enabled) {
#if MULT_UART_DEVICE_DIAGNOSTICS_ENABLE
        manager->stats.invalid_request++;
#endif
        return MULT_UART_ERR_STATE;
    }

    status = mult_uart_device_alloc_slot(manager, transfer, &slot);
    if (status != MULT_UART_OK) {
#if MULT_UART_DEVICE_DIAGNOSTICS_ENABLE
        if (status == MULT_UART_ERR_BUSY) {
            manager->stats.busy++;
        } else {
            manager->stats.invalid_request++;
        }
#endif
        return status;
    }

    mult_uart_device_make_request(manager, transfer, slot, &request);
    queue_timeout_ms =
        (transfer->queue_timeout_ms != 0U) ?
            transfer->queue_timeout_ms : MULT_UART_DEVICE_QUEUE_TIMEOUT_MS;

    status = mux_service_submit(&request, queue_timeout_ms);
    if (status != MULT_UART_OK) {
        mult_uart_device_free_slot(slot);
#if MULT_UART_DEVICE_DIAGNOSTICS_ENABLE
        manager->stats.submit_error++;
#endif
        return status;
    }

#if MULT_UART_DEVICE_DIAGNOSTICS_ENABLE
    manager->stats.submitted++;
#endif
    return MULT_UART_OK;
}

/**
 * @brief 获取Device层统计快照。
 * @param stats 输出统计对象。
 * @return 获取成功返回MULT_UART_OK，否则返回参数或未初始化错误。
 */
#if MULT_UART_DEVICE_DIAGNOSTICS_ENABLE
mult_uart_status_t mult_uart_device_get_stats(
    mult_uart_device_stats_t *stats)
{
    mult_uart_device_manager_t *manager = &g_mult_uart_device_manager;

    if (stats == NULL) {
        return MULT_UART_ERR_PARAM;
    }
    if (!manager->initialized) {
        return MULT_UART_ERR_NOT_INIT;
    }

    *stats = manager->stats;
    return MULT_UART_OK;
}
#endif
