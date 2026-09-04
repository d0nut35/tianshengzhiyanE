/**
 * @file    ic_card_uart7_hal.c
 * @brief   IC卡读写器直连UART7的中断发送/ReceiveToIdle DMA硬件绑定。
 */

#include "ic_card_uart7_hal.h"

#include <stddef.h>

#include "usart.h"

/**
 * @brief 将HAL返回值收敛为平台无关IC卡状态。
 * @param status HAL接口返回值。
 * @return 对应的ic_card状态码。
 */
static ic_card_status_t ic_card_map_hal_status(HAL_StatusTypeDef status)
{
    if (status == HAL_OK) {
        return IC_CARD_OK;
    }
    if (status == HAL_BUSY) {
        return IC_CARD_ERR_BUSY;
    }
    return IC_CARD_ERR_IO;
}

/**
 * @brief 通过绑定的UART启动异步中断发送。
 * @param ctx HAL适配器上下文。
 * @param data 待发送数据。
 * @param len 数据长度。
 * @return HAL启动结果的统一映射。
 * @warning data在完成回调前必须有效；该生命周期由Core的tx_buffer保证。
 */
static ic_card_status_t ic_card_hal_tx_start(
    void *ctx,
    const uint8_t *data,
    size_t len)
{
    ic_card_uart7_hal_t *adapter = (ic_card_uart7_hal_t *)ctx;

    if ((adapter == NULL) || (adapter->uart == NULL) || (data == NULL) ||
        (len == 0U) || (len > UINT16_MAX)) {
        return IC_CARD_ERR_PARAM;
    }
    return ic_card_map_hal_status(HAL_UART_Transmit_IT(
        adapter->uart,
        data,
        (uint16_t)len));
}

/**
 * @brief 启动Normal DMA + ReceiveToIdle接收。
 * @param ctx HAL适配器上下文。
 * @param data 接收缓冲区。
 * @param capacity 缓冲区容量。
 * @return HAL启动结果的统一映射。
 * @note 关闭半传输中断，避免同一缓冲区被HT和IDLE重复提交；真正帧边界由Core判断。
 */
static ic_card_status_t ic_card_hal_rx_start(
    void *ctx,
    uint8_t *data,
    size_t capacity)
{
    ic_card_uart7_hal_t *adapter = (ic_card_uart7_hal_t *)ctx;
    HAL_StatusTypeDef hal_status;

    if ((adapter == NULL) || (adapter->uart == NULL) || (data == NULL) ||
        (capacity == 0U) || (capacity > UINT16_MAX)) {
        return IC_CARD_ERR_PARAM;
    }
    hal_status = HAL_UARTEx_ReceiveToIdle_DMA(
        adapter->uart,
        data,
        (uint16_t)capacity);
    if ((hal_status == HAL_OK) && (adapter->uart->hdmarx != NULL)) {
        __HAL_DMA_DISABLE_IT(adapter->uart->hdmarx, DMA_IT_HT);
    }
    return ic_card_map_hal_status(hal_status);
}

/**
 * @brief 同步停止UART7 TX/RX并清除相关待处理中断。
 * @param ctx HAL适配器上下文。
 * @return HAL abort结果的统一映射。
 * @note 防止上一笔事务的迟到中断被下一笔读取错误认领。
 */
static ic_card_status_t ic_card_hal_abort(void *ctx)
{
    ic_card_uart7_hal_t *adapter = (ic_card_uart7_hal_t *)ctx;
    HAL_StatusTypeDef hal_status;

    if ((adapter == NULL) || (adapter->uart == NULL)) {
        return IC_CARD_ERR_PARAM;
    }
    hal_status = HAL_UART_Abort(adapter->uart);
    __HAL_UART_CLEAR_OREFLAG(adapter->uart);
    HAL_NVIC_ClearPendingIRQ(UART7_IRQn);
    HAL_NVIC_ClearPendingIRQ(DMA1_Stream3_IRQn);
    return ic_card_map_hal_status(hal_status);
}

/**
 * @brief 使用CubeMX生成的huart7填充默认适配配置。
 * @param config 接收配置的输出对象。
 */
void ic_card_uart7_hal_make_config(ic_card_uart7_hal_config_t *config)
{
    if (config != NULL) {
        config->uart = &huart7;
    }
}

/**
 * @brief 绑定Core对象与UART7硬件并输出port能力。
 * @param adapter HAL适配器对象。
 * @param device IC卡Core对象。
 * @param config UART7资源配置。
 * @param port 接收port函数表的输出对象。
 * @return 绑定结果。
 */
ic_card_status_t ic_card_uart7_hal_bind(
    ic_card_uart7_hal_t *adapter,
    ic_card_t *device,
    const ic_card_uart7_hal_config_t *config,
    ic_card_port_t *port)
{
    static const ic_card_port_t template_port = {
        ic_card_hal_tx_start,
        ic_card_hal_rx_start,
        ic_card_hal_abort,
        NULL,
    };

    if ((adapter == NULL) || (device == NULL) || (config == NULL) ||
        (config->uart == NULL) || (port == NULL)) {
        return IC_CARD_ERR_PARAM;
    }
    adapter->device = device;
    adapter->uart = config->uart;
    *port = template_port;
    port->ctx = adapter;
    return IC_CARD_OK;
}

/**
 * @brief 过滤并转发UART7 TX完成回调。
 * @param adapter HAL适配器对象。
 * @param huart 产生回调的UART句柄。
 * @return 句柄匹配并已转发时返回true。
 */
bool ic_card_uart7_hal_handle_tx_complete(
    ic_card_uart7_hal_t *adapter,
    UART_HandleTypeDef *huart)
{
    if ((adapter == NULL) || (huart != adapter->uart)) {
        return false;
    }
    ic_card_on_tx_complete_isr(adapter->device);
    return true;
}

/**
 * @brief 过滤并转发UART7 ReceiveToIdle回调。
 * @param adapter HAL适配器对象。
 * @param huart 产生回调的UART句柄。
 * @param rx_len 本次有效字节数。
 * @return 句柄匹配并已转发时返回true。
 */
bool ic_card_uart7_hal_handle_rx_event(
    ic_card_uart7_hal_t *adapter,
    UART_HandleTypeDef *huart,
    uint16_t rx_len)
{
    if ((adapter == NULL) || (huart != adapter->uart)) {
        return false;
    }
    ic_card_on_rx_event_isr(adapter->device, rx_len);
    return true;
}

/**
 * @brief 过滤并转发UART7错误回调。
 * @param adapter HAL适配器对象。
 * @param huart 产生错误的UART句柄。
 * @return 句柄匹配并已转发时返回true。
 */
bool ic_card_uart7_hal_handle_error(
    ic_card_uart7_hal_t *adapter,
    UART_HandleTypeDef *huart)
{
    if ((adapter == NULL) || (huart != adapter->uart)) {
        return false;
    }
    ic_card_on_error_isr(adapter->device);
    return true;
}
