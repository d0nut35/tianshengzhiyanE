/**
 * @file    lsc16_stm32_hal.c
 * @brief   F7 UART8中断发送和ReceiveToIdle DMA的LSC16适配实现。
 *
 * 本文件不定义HAL_UART_*全局回调；回调唯一入口属于uart_dispatch，
 * 本适配器只提供先核对huart再认领事件的handler。
 */

#include "lsc16_stm32_hal.h"

#include <string.h>

#include "usart.h"

/**
 * @brief 将HAL状态收敛成平台无关LSC16错误。
 * @param status HAL接口返回值。
 * @return 对应的lsc16状态码。
 */
static lsc16_status_t lsc16_map_hal_status(HAL_StatusTypeDef status)
{
    if (status == HAL_OK) {
        return LSC16_OK;
    }
    if (status == HAL_BUSY) {
        return LSC16_ERR_BUSY;
    }
    return LSC16_ERR_IO;
}

/**
 * @brief 通过绑定的UART8启动异步中断发送。
 * @param ctx HAL适配器上下文。
 * @param data 待发送数据。
 * @param len 数据长度。
 * @return HAL启动结果的统一映射。
 * @warning HAL会持续读取Core的tx_buffer直到TX complete，其生命周期由
 *          tx_busy保护。
 */
static lsc16_status_t lsc16_hal_tx_start(
    void *ctx,
    const uint8_t *data,
    size_t len)
{
    lsc16_stm32_hal_t *adapter = (lsc16_stm32_hal_t *)ctx;

    if ((adapter == NULL) || !adapter->bound || (data == NULL) ||
        (len == 0U) || (len > UINT16_MAX)) {
        return LSC16_ERR_PARAM;
    }
    return lsc16_map_hal_status(HAL_UART_Transmit_IT(
        adapter->config.uart,
        data,
        (uint16_t)len));
}

/**
 * @brief 使用ReceiveToIdle Normal DMA接收不定长回报。
 * @param ctx HAL适配器上下文。
 * @param data 接收缓冲区。
 * @param capacity 缓冲区容量。
 * @return HAL启动结果的统一映射。
 */
static lsc16_status_t lsc16_hal_rx_start(
    void *ctx,
    uint8_t *data,
    size_t capacity)
{
    lsc16_stm32_hal_t *adapter = (lsc16_stm32_hal_t *)ctx;
    HAL_StatusTypeDef hal_status;

    if ((adapter == NULL) || !adapter->bound || (data == NULL) ||
        (capacity == 0U) || (capacity > UINT16_MAX)) {
        return LSC16_ERR_PARAM;
    }
    hal_status = HAL_UARTEx_ReceiveToIdle_DMA(
        adapter->config.uart,
        data,
        (uint16_t)capacity);
    if ((hal_status == HAL_OK) && (adapter->config.uart->hdmarx != NULL)) {
        /* 只使用IDLE/TC作为批次边界，禁止HT造成同一缓冲区重复发布。 */
        __HAL_DMA_DISABLE_IT(adapter->config.uart->hdmarx, DMA_IT_HT);
    }
    return lsc16_map_hal_status(hal_status);
}

/**
 * @brief 同步停止UART8收发。
 * @param ctx HAL适配器上下文。
 * @return HAL abort结果的统一映射。
 * @note 错误恢复只在普通上下文调用。
 */
static lsc16_status_t lsc16_hal_abort(void *ctx)
{
    lsc16_stm32_hal_t *adapter = (lsc16_stm32_hal_t *)ctx;

    if ((adapter == NULL) || !adapter->bound) {
        return LSC16_ERR_PARAM;
    }
    return lsc16_map_hal_status(HAL_UART_Abort(adapter->config.uart));
}

/**
 * @brief 使用CubeMX生成的huart8填充默认适配配置。
 * @param config 接收配置的输出对象。
 */
void lsc16_stm32_hal_make_uart8_config(lsc16_stm32_hal_config_t *config)
{
    if (config != NULL) {
        config->uart = &huart8;
    }
}

/**
 * @brief 绑定Core对象与UART8硬件并输出port能力。
 * @param adapter HAL适配器对象。
 * @param device LSC16 Core对象。
 * @param config UART8资源配置。
 * @param port 接收port函数表的输出对象。
 * @return 绑定结果。
 */
lsc16_status_t lsc16_stm32_hal_bind(
    lsc16_stm32_hal_t *adapter,
    lsc16_t *device,
    const lsc16_stm32_hal_config_t *config,
    lsc16_port_t *port)
{
    if ((adapter == NULL) || (device == NULL) || (config == NULL) ||
        (config->uart == NULL) || (port == NULL)) {
        return LSC16_ERR_PARAM;
    }
    (void)memset(adapter, 0, sizeof(*adapter));
    adapter->device = device;
    adapter->config = *config;
    adapter->bound = true;

    port->tx_start = lsc16_hal_tx_start;
    port->rx_start = lsc16_hal_rx_start;
    port->abort = lsc16_hal_abort;
    port->ctx = adapter;
    return LSC16_OK;
}

/**
 * @brief 过滤并转发UART8 TX完成回调。
 * @param adapter HAL适配器对象。
 * @param huart 产生回调的UART句柄。
 * @return 句柄匹配并已转发时返回true；否则让路由器继续查询。
 */
bool lsc16_stm32_hal_handle_tx_complete(
    lsc16_stm32_hal_t *adapter,
    UART_HandleTypeDef *huart)
{
    if ((adapter == NULL) || !adapter->bound ||
        (huart != adapter->config.uart)) {
        return false;
    }
    lsc16_on_tx_complete_isr(adapter->device);
    return true;
}

/**
 * @brief 过滤并转发UART8 ReceiveToIdle回调。
 * @param adapter HAL适配器对象。
 * @param huart 产生回调的UART句柄。
 * @param rx_len 本次有效字节数。
 * @return 句柄匹配并已转发时返回true。
 */
bool lsc16_stm32_hal_handle_rx_event(
    lsc16_stm32_hal_t *adapter,
    UART_HandleTypeDef *huart,
    uint16_t rx_len)
{
    if ((adapter == NULL) || !adapter->bound ||
        (huart != adapter->config.uart)) {
        return false;
    }
    lsc16_on_rx_event_isr(adapter->device, rx_len);
    return true;
}

/**
 * @brief 过滤并转发UART8错误回调。
 * @param adapter HAL适配器对象。
 * @param huart 产生错误的UART句柄。
 * @return 句柄匹配并已转发时返回true。
 */
bool lsc16_stm32_hal_handle_error(
    lsc16_stm32_hal_t *adapter,
    UART_HandleTypeDef *huart)
{
    if ((adapter == NULL) || !adapter->bound ||
        (huart != adapter->config.uart)) {
        return false;
    }
    lsc16_on_error_isr(adapter->device);
    return true;
}
