/**
 * @file    mux_bsp.c
 * @brief   四通道TTL UART复用器与STM32 UART7 HAL实现。
 */

#include "mux_bsp.h"

#include <limits.h>

#include "main.h"
#include "usart.h"

#define MULT_UART_SWITCH_SETTLE_US 5U

/** 将HAL状态收敛为复用模块状态。 */
static mult_uart_status_t mult_uart_map_hal(HAL_StatusTypeDef status)
{
    if (status == HAL_OK) return MULT_UART_OK;
    if (status == HAL_BUSY) return MULT_UART_ERR_BUSY;
    if (status == HAL_TIMEOUT) return MULT_UART_ERR_TIMEOUT;
    return MULT_UART_ERR_IO;
}

/** 一次BSRR写同时提交PD9/PD10，避免出现中间通道。 */
static void mult_uart_write_select(bool a_high, bool b_high)
{
    uint32_t set_mask = (a_high ? M_A_Pin : 0U) |
                        (b_high ? M_B_Pin : 0U);
    uint32_t reset_mask = (a_high ? 0U : M_A_Pin) |
                          (b_high ? 0U : M_B_Pin);

    M_A_GPIO_Port->BSRR = set_mask | (reset_mask << 16U);
}

/** EN/INH低有效，传true表示接通复用器。 */
static void mult_uart_write_enable(bool enabled)
{
    HAL_GPIO_WritePin(
        M_EN_GPIO_Port,
        M_EN_Pin,
        enabled ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

/** 通道切换后的短稳定等待。 */
static void mult_uart_delay_us(uint32_t delay_us)
{
    uint32_t cycles_per_us = SystemCoreClock / 1000000U;
    uint32_t spin;

    if (cycles_per_us == 0U) cycles_per_us = 1U;
    while (delay_us-- > 0U) {
        for (spin = 0U; spin < cycles_per_us; ++spin) __NOP();
    }
}

/** 启动固定UART7异步发送。 */
static mult_uart_status_t mult_uart_start_tx(
    const uint8_t *data,
    size_t len)
{
    if (len > UINT16_MAX) return MULT_UART_ERR_PARAM;
    return mult_uart_map_hal(HAL_UART_Transmit_IT(
        &huart7,
        data,
        (uint16_t)len));
}

/** 清旧状态后启动UART7 ReceiveToIdle DMA，并关闭半传输中断。 */
static mult_uart_status_t mult_uart_start_rx(
    uint8_t *data,
    size_t capacity)
{
    HAL_StatusTypeDef status;

    if (capacity > UINT16_MAX) return MULT_UART_ERR_PARAM;
    __HAL_UART_CLEAR_FLAG(
        &huart7,
        UART_CLEAR_OREF | UART_CLEAR_NEF | UART_CLEAR_PEF |
            UART_CLEAR_FEF | UART_CLEAR_IDLEF);
    __HAL_UART_SEND_REQ(&huart7, UART_RXDATA_FLUSH_REQUEST);
    huart7.ErrorCode = HAL_UART_ERROR_NONE;

    status = HAL_UARTEx_ReceiveToIdle_DMA(
        &huart7,
        data,
        (uint16_t)capacity);
    if (status != HAL_OK) return mult_uart_map_hal(status);
    __HAL_DMA_DISABLE_IT(huart7.hdmarx, DMA_IT_HT);
    return MULT_UART_OK;
}

/** 同步终止UART7收发，并清除可能污染下一事务的旧中断。 */
static mult_uart_status_t mult_uart_abort_io(void)
{
    HAL_StatusTypeDef status = HAL_UART_Abort(&huart7);

    if (status != HAL_OK) return mult_uart_map_hal(status);
    __HAL_UART_CLEAR_PEFLAG(&huart7);
    HAL_NVIC_ClearPendingIRQ(UART7_IRQn);
    HAL_NVIC_ClearPendingIRQ(DMA1_Stream3_IRQn);
    return MULT_UART_OK;
}

/**
 * @brief 判断枚举值是否对应硬件支持的4个通道之一。
 * @param channel 待检查通道。
 * @return 通道合法返回true。
 */
static bool mult_uart_channel_is_valid(mult_uart_channel_t channel)
{
    return ((uint32_t)channel < MULT_UART_CHANNEL_COUNT);
}

/**
 * @brief 判断TX/RX任一方向是否仍有DMA操作在途。
 * @param bus 复用总线对象。
 * @return 任一方向活动时返回true。
 */
static bool mult_uart_io_is_active(const mult_uart_bus_t *bus)
{
    return bus->tx_active || bus->rx_active;
}

/**
 * @brief 根据独立的 TX/RX 活动标志刷新总线状态。
 * @param bus 复用总线对象。
 * @note ERROR 必须由 abort 显式恢复，不能被普通完成事件覆盖。
 */
static void mult_uart_sync_active_state(mult_uart_bus_t *bus)
{
    if (bus->state == MULT_UART_STATE_ERROR) {
        return;
    }

    bus->state = mult_uart_io_is_active(bus) ?
        MULT_UART_STATE_ACTIVE : MULT_UART_STATE_IDLE;
}

/**
 * @brief 将 bus 恢复为确定的 UNINIT 状态。
 * @param bus 复用总线对象。
 * @note 用于首次初始化、初始化失败回滚和成功反初始化。
 */
static void mult_uart_reset_bus(mult_uart_bus_t *bus)
{
    bus->initialized = false;
    bus->enabled = false;
    bus->current_channel = (mult_uart_channel_t)MULT_UART_CHANNEL_INVALID;
    bus->state = MULT_UART_STATE_UNINIT;
    bus->tx_active = false;
    bus->rx_active = false;
    bus->aborting = false;
    bus->rx_capacity = 0U;

    bus->event_cb = NULL;
    bus->event_ctx = NULL;
#if !LICANG_RELEASE_MINIMAL
    bus->uart_error_count = 0U;
    bus->last_uart_error = 0U;
#endif
}

/**
 * @brief 检查只能在静止状态执行的控制操作。
 * @param bus 复用总线对象。
 * @return IDLE 返回 OK；DMA 活动/abort 中返回 BUSY；ERROR 返回 STATE。
 */
static mult_uart_status_t mult_uart_require_idle(const mult_uart_bus_t *bus)
{
    if (!bus->initialized) {
        return MULT_UART_ERR_NOT_INIT;
    }

    if ((bus->state == MULT_UART_STATE_ACTIVE) ||
        (bus->state == MULT_UART_STATE_ABORTING) ||
        mult_uart_io_is_active(bus)) {
        return MULT_UART_ERR_BUSY;
    }

    if (bus->state != MULT_UART_STATE_IDLE) {
        return MULT_UART_ERR_STATE;
    }

    return MULT_UART_OK;
}

/**
 * @brief 在 ISR 上下文构造并同步投递一个 Core 事件。
 * @param bus 复用总线对象。
 * @param type 事件类型。
 * @param status 统一状态码。
 * @param rx_len RX事件的有效长度，其余事件传0。
 * @param port_error 平台原始错误码，其余事件传0。
 * @warning event 是栈对象，回调不得保存其地址，也不得执行阻塞操作。
 */
static void mult_uart_emit_event(
    mult_uart_bus_t *bus,
    mult_uart_event_type_t type,
    mult_uart_status_t status,
    size_t rx_len,
    uint32_t port_error)
{
    mult_uart_event_t event;

    if (bus->event_cb == NULL) {
        return;
    }

    event.type = type;
    event.status = status;
    event.rx_len = rx_len;
    event.port_error = port_error;
    bus->event_cb(bus->event_ctx, &event);
}

/**
 * @brief 初始化复用总线并建立安全的默认硬件状态。
 * @param bus 待初始化总线对象。
 * @param config EN极性、切换策略和稳定时间配置。
 * @param port 已绑定硬件上下文的平台能力。
 * @return 初始化结果。
 *
 * managed EN 模式会先输出禁用电平；只有全部平台检查和初始写入成功后，
 * 才把对象标记为 initialized，避免上层看到半初始化状态。
 */
mult_uart_status_t mult_uart_init(mult_uart_bus_t *bus)
{
    if (bus == NULL) return MULT_UART_ERR_PARAM;

    if (bus->initialized) {
        return MULT_UART_ERR_STATE;
    }

    if ((huart7.Instance != UART7) || (huart7.hdmarx == NULL) ||
        (M_A_GPIO_Port != M_B_GPIO_Port)) {
        mult_uart_reset_bus(bus);
        return MULT_UART_ERR_IO;
    }

    mult_uart_reset_bus(bus);
    mult_uart_write_enable(false);
    bus->state = MULT_UART_STATE_IDLE;
    bus->initialized = true;
    return MULT_UART_OK;
}

/**
 * @brief 在总线静止时绑定或解绑 ISR 事件出口。
 * @param bus 总线对象。
 * @param event_cb ISR事件回调；传NULL表示解绑。
 * @param event_ctx 原样传给event_cb的上下文。
 * @return 绑定结果。
 * @note 允许传入 NULL 以支持只验证 A/B/EN 的阶段。
 */
mult_uart_status_t mult_uart_bind_event(
    mult_uart_bus_t *bus,
    mult_uart_event_cb_t event_cb,
    void *event_ctx)
{
    mult_uart_status_t status;

    if (bus == NULL) {
        return MULT_UART_ERR_PARAM;
    }

    status = mult_uart_require_idle(bus);
    if (status != MULT_UART_OK) {
        return status;
    }

    bus->event_cb = event_cb;
    bus->event_ctx = event_ctx;
    return MULT_UART_OK;
}

/**
 * @brief 连接当前已选通的逻辑通道。
 * @param bus 总线对象。
 * @return 使能结果。
 * @note managed EN 模式必须先成功 select，防止默认 A/B 电平误接通道 0。
 */
mult_uart_status_t mult_uart_enable(mult_uart_bus_t *bus)
{
    mult_uart_status_t status;

    if (bus == NULL) {
        return MULT_UART_ERR_PARAM;
    }

    status = mult_uart_require_idle(bus);
    if (status != MULT_UART_OK) {
        return status;
    }

    if (bus->enabled) {
        return MULT_UART_OK;
    }

    if (!mult_uart_channel_is_valid(bus->current_channel)) {
        return MULT_UART_ERR_STATE;
    }

    mult_uart_write_enable(true);
    bus->enabled = true;
    return MULT_UART_OK;
}

/**
 * @brief 在总线静止时断开复用模块。
 * @param bus 总线对象。
 * @return 禁用结果。
 * @note EN 不受软件管理时无法保证物理断开，因此返回 UNSUPPORTED。
 */
mult_uart_status_t mult_uart_disable(mult_uart_bus_t *bus)
{
    mult_uart_status_t status;

    if (bus == NULL) {
        return MULT_UART_ERR_PARAM;
    }

    status = mult_uart_require_idle(bus);
    if (status != MULT_UART_OK) {
        return status;
    }

    if (!bus->enabled) {
        return MULT_UART_OK;
    }

    mult_uart_write_enable(false);
    bus->enabled = false;
    return MULT_UART_OK;
}

/**
 * @brief 根据 channel 的 bit0/bit1 写 A/B，并执行可选的先断后切。
 * @param bus 总线对象。
 * @param channel 目标逻辑通道。
 * @return 切换结果。
 *
 * 所有硬件步骤成功后才更新通道缓存。若切换失败，managed EN 会保持或
 * 尽力进入禁用状态，避免把不确定的 A/B 组合暴露给串口线路。
 */
mult_uart_status_t mult_uart_select(
    mult_uart_bus_t *bus,
    mult_uart_channel_t channel)
{
    mult_uart_status_t status;
    bool temporarily_disabled = false;
    bool a_high;
    bool b_high;

    if (bus == NULL) {
        return MULT_UART_ERR_PARAM;
    }

    status = mult_uart_require_idle(bus);
    if (status != MULT_UART_OK) {
        return status;
    }

    if (!mult_uart_channel_is_valid(channel)) {
        return MULT_UART_ERR_PARAM;
    }

    if (bus->current_channel == channel) {
        return MULT_UART_OK;
    }

    if (bus->enabled) {
        mult_uart_write_enable(false);
        bus->enabled = false;
        temporarily_disabled = true;
    }

    a_high = (((uint32_t)channel & 0x01U) != 0U);
    b_high = (((uint32_t)channel & 0x02U) != 0U);
    mult_uart_write_select(a_high, b_high);
    mult_uart_delay_us(MULT_UART_SWITCH_SETTLE_US);

    if (temporarily_disabled) {
        mult_uart_write_enable(true);
        bus->enabled = true;
    }

    bus->current_channel = channel;
    return MULT_UART_OK;
}

/**
 * @brief 读取最近一次成功选通的通道。
 * @param bus 总线对象。
 * @param channel 接收当前通道的输出对象；未选通过时输出INVALID。
 * @return 查询结果。
 */
mult_uart_status_t mult_uart_get_channel(
    const mult_uart_bus_t *bus,
    mult_uart_channel_t *channel)
{
    if ((bus == NULL) || (channel == NULL)) {
        return MULT_UART_ERR_PARAM;
    }

    if (!bus->initialized) {
        *channel = (mult_uart_channel_t)MULT_UART_CHANNEL_INVALID;
        return MULT_UART_ERR_NOT_INIT;
    }

    *channel = bus->current_channel;
    return MULT_UART_OK;
}

/**
 * @brief 启动公共 UART 的异步 DMA 发送。
 * @param bus 总线对象。
 * @param data 待发送数据。
 * @param len 数据长度。
 * @return DMA启动结果。
 *
 * TX 与 RX 使用独立活动标志，因此允许在 RX DMA 已挂起时发送请求。
 * 调用 port 前先标记 tx_active，用于封闭极短 DMA 的完成竞态。
 */
mult_uart_status_t mult_uart_start_tx_dma(
    mult_uart_bus_t *bus,
    const uint8_t *data,
    size_t len)
{
    mult_uart_status_t status;

    if ((bus == NULL) || (data == NULL) || (len == 0U)) {
        return MULT_UART_ERR_PARAM;
    }

    if (!bus->initialized) {
        return MULT_UART_ERR_NOT_INIT;
    }

    if ((bus->state == MULT_UART_STATE_ERROR) ||
        (bus->state == MULT_UART_STATE_UNINIT)) {
        return MULT_UART_ERR_STATE;
    }

    if (bus->aborting || bus->tx_active) {
        return MULT_UART_ERR_BUSY;
    }

    if (!bus->enabled ||
        !mult_uart_channel_is_valid(bus->current_channel) ||
        (bus->event_cb == NULL)) {
        return MULT_UART_ERR_STATE;
    }

    /* 先标记 TX 活动，避免极短 DMA 在 start 返回前完成造成竞态。 */
    bus->tx_active = true;
    mult_uart_sync_active_state(bus);
    status = mult_uart_start_tx(data, len);
    if ((status != MULT_UART_OK) &&
        bus->tx_active) {
        bus->tx_active = false;
        mult_uart_sync_active_state(bus);
    }

    return status;
}

/**
 * @brief 启动公共 UART 的异步 DMA/Idle 接收。
 * @param bus 总线对象。
 * @param data 接收缓冲区。
 * @param capacity 缓冲区容量。
 * @return DMA接收启动结果。
 *
 * 保存 capacity 供 ISR 校验实际长度；WRITE_READ 应先调用本函数再启动
 * TX，以免设备紧随请求返回时丢失响应首字节。
 */
mult_uart_status_t mult_uart_start_rx_dma(
    mult_uart_bus_t *bus,
    uint8_t *data,
    size_t capacity)
{
    mult_uart_status_t status;

    if ((bus == NULL) || (data == NULL) || (capacity == 0U)) {
        return MULT_UART_ERR_PARAM;
    }

    if (!bus->initialized) {
        return MULT_UART_ERR_NOT_INIT;
    }

    if ((bus->state == MULT_UART_STATE_ERROR) ||
        (bus->state == MULT_UART_STATE_UNINIT)) {
        return MULT_UART_ERR_STATE;
    }

    if (bus->aborting || bus->rx_active) {
        return MULT_UART_ERR_BUSY;
    }

    if (!bus->enabled ||
        !mult_uart_channel_is_valid(bus->current_channel) ||
        (bus->event_cb == NULL)) {
        return MULT_UART_ERR_STATE;
    }

    bus->rx_capacity = capacity;
    bus->rx_active = true;
    mult_uart_sync_active_state(bus);
    status = mult_uart_start_rx(data, capacity);
    if ((status != MULT_UART_OK) &&
        bus->rx_active) {
        bus->rx_active = false;
        bus->rx_capacity = 0U;
        mult_uart_sync_active_state(bus);
    }

    return status;
}

/**
 * @brief 中止两个方向的 DMA，并从 ACTIVE/ERROR 恢复到 IDLE。
 * @param bus 总线对象。
 * @return 同步中止结果。
 *
 * aborting 在 port 执行期间屏蔽同步到达的完成 ISR。平台 abort 必须在
 * 返回前停 DMA并清除挂起中断，防止旧 ISR 被下一次传输误认领。
 */
mult_uart_status_t mult_uart_abort(mult_uart_bus_t *bus)
{
    mult_uart_status_t status;

    if (bus == NULL) {
        return MULT_UART_ERR_PARAM;
    }

    if (!bus->initialized) {
        return MULT_UART_ERR_NOT_INIT;
    }

    if ((bus->state == MULT_UART_STATE_IDLE) &&
        !mult_uart_io_is_active(bus)) {
        return MULT_UART_OK;
    }

    bus->aborting = true;
    bus->state = MULT_UART_STATE_ABORTING;
    bus->tx_active = false;
    bus->rx_active = false;
    bus->rx_capacity = 0U;
    bus->current_channel =
        (mult_uart_channel_t)MULT_UART_CHANNEL_INVALID;
    status = mult_uart_abort_io();
    bus->aborting = false;
    if (status == MULT_UART_OK) {
        bus->state = MULT_UART_STATE_IDLE;
    } else {
        bus->state = MULT_UART_STATE_ERROR;
    }

    return status;
}

/**
 * @brief TX DMA完成ISR入口，只结束TX方向并保留仍活动的RX。
 * @param bus 总线对象。
 * @warning 仅由绑定公共UART的HAL适配器调用。
 */
void mult_uart_on_tx_complete_isr(mult_uart_bus_t *bus)
{
    if ((bus == NULL) || !bus->initialized ||
        bus->aborting || !bus->tx_active) {
        return;
    }

    bus->tx_active = false;
    mult_uart_sync_active_state(bus);
    mult_uart_emit_event(
        bus,
        MULT_UART_EVENT_TX_COMPLETE,
        MULT_UART_OK,
        0U,
        0U);
}

/**
 * @brief RX DMA/Idle 完成 ISR 入口。
 * @param bus 总线对象。
 * @param rx_len 本次接收有效字节数。
 * @note 超出启动 capacity 的长度被视为 adapter/硬件错误并上报 OVERFLOW。
 */
void mult_uart_on_rx_complete_isr(
    mult_uart_bus_t *bus,
    size_t rx_len)
{
    if ((bus == NULL) || !bus->initialized ||
        bus->aborting || !bus->rx_active) {
        return;
    }

    if (rx_len > bus->rx_capacity) {
        bus->rx_active = false;
        bus->rx_capacity = 0U;
        bus->state = MULT_UART_STATE_ERROR;
        bus->current_channel =
            (mult_uart_channel_t)MULT_UART_CHANNEL_INVALID;
        mult_uart_emit_event(
            bus,
            MULT_UART_EVENT_ERROR,
            MULT_UART_ERR_OVERFLOW,
            rx_len,
            0U);
        return;
    }

    bus->rx_active = false;
    bus->rx_capacity = 0U;
    mult_uart_sync_active_state(bus);
    mult_uart_emit_event(
        bus,
        MULT_UART_EVENT_RX_COMPLETE,
        MULT_UART_OK,
        rx_len,
        0U);
}

/**
 * @brief UART/DMA 错误 ISR 入口。
 * @param bus 总线对象。
 * @param port_error 平台原始错误码或错误位。
 * @note 任一方向出错后整体进入 ERROR，由任务上下文调用 abort 恢复。
 */
void mult_uart_on_error_isr(
    mult_uart_bus_t *bus,
    uint32_t port_error)
{
    if ((bus == NULL) || !bus->initialized || bus->aborting ||
        !mult_uart_io_is_active(bus)) {
        return;
    }

    bus->tx_active = false;
    bus->rx_active = false;
    bus->rx_capacity = 0U;
    bus->state = MULT_UART_STATE_ERROR;
    bus->current_channel =
        (mult_uart_channel_t)MULT_UART_CHANNEL_INVALID;
    mult_uart_emit_event(
        bus,
        MULT_UART_EVENT_ERROR,
        MULT_UART_ERR_IO,
        0U,
        port_error);
}

bool mult_uart_handle_tx(
    mult_uart_bus_t *bus,
    UART_HandleTypeDef *huart)
{
    if ((bus == NULL) || (huart != &huart7)) return false;
    mult_uart_on_tx_complete_isr(bus);
    return true;
}

bool mult_uart_handle_rx(
    mult_uart_bus_t *bus,
    UART_HandleTypeDef *huart,
    uint16_t rx_len)
{
    if ((bus == NULL) || (huart != &huart7)) return false;
    mult_uart_on_rx_complete_isr(bus, (size_t)rx_len);
    return true;
}

bool mult_uart_handle_error(
    mult_uart_bus_t *bus,
    UART_HandleTypeDef *huart)
{
    if ((bus == NULL) || (huart != &huart7)) return false;
#if !LICANG_RELEASE_MINIMAL
    bus->last_uart_error = huart->ErrorCode;
    bus->uart_error_count++;
#endif
    mult_uart_on_error_isr(bus, huart->ErrorCode);
    return true;
}

/**
 * @brief 在无在途 DMA 时反初始化总线。
 * @param bus 总线对象。
 * @return 反初始化结果。
 * @note managed EN 若仍使能，会先物理断开；失败时保留对象供诊断/恢复。
 */
#if MULT_UART_BSP_TEST_API_ENABLE
mult_uart_status_t mult_uart_deinit(mult_uart_bus_t *bus)
{
    if (bus == NULL) {
        return MULT_UART_ERR_PARAM;
    }

    if (!bus->initialized) {
        return MULT_UART_ERR_NOT_INIT;
    }

    if ((bus->state == MULT_UART_STATE_ACTIVE) ||
        (bus->state == MULT_UART_STATE_ABORTING) ||
        mult_uart_io_is_active(bus)) {
        return MULT_UART_ERR_BUSY;
    }

    if (bus->state != MULT_UART_STATE_IDLE) {
        return MULT_UART_ERR_STATE;
    }

    if (bus->enabled) mult_uart_write_enable(false);

    mult_uart_reset_bus(bus);
    return MULT_UART_OK;
}
#endif
