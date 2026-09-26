/** @file zdt_turntable_uart7_hal.c @brief ZDT UART7直连HAL适配实现。 */

#include "zdt_turntable_uart7_hal.h"

#include <limits.h>
#include <string.h>

#include "usart.h"

/**
 * @brief 将STM32 HAL状态收敛为平台无关ZDT状态。
 * @param status HAL接口返回值。
 * @return 对应的ZDT状态码。
 */
static zdt_turntable_status_t zdt_map_hal(HAL_StatusTypeDef status)
{
    if (status == HAL_OK) {
        return ZDT_TURNTABLE_OK;
    }
    if (status == HAL_BUSY) {
        return ZDT_TURNTABLE_ERR_BUSY;
    }
    if (status == HAL_TIMEOUT) {
        return ZDT_TURNTABLE_ERR_TIMEOUT;
    }
    return ZDT_TURNTABLE_ERR_IO;
}

/**
 * @brief 通过UART7启动一帧异步中断发送。
 * @param ctx HAL适配器上下文。
 * @param data Service active.frame中的发送数据。
 * @param len 有效字节数。
 * @return 异步发送启动结果。
 * @warning data在TX完成或abort前必须有效；该生命周期由Service保证。
 */
static zdt_turntable_status_t zdt_hal_tx(
    void *ctx, const uint8_t *data, size_t len)
{
    zdt_turntable_uart7_hal_t *adapter =
        (zdt_turntable_uart7_hal_t *)ctx;
    if ((adapter == NULL) || !adapter->bound || (data == NULL) ||
        (len == 0U) || (len > UINT16_MAX)) {
        return ZDT_TURNTABLE_ERR_PARAM;
    }
    return zdt_map_hal(HAL_UART_Transmit_IT(
        adapter->uart, data, (uint16_t)len));
}

/**
 * @brief 通过UART7启动ReceiveToIdle DMA接收。
 * @param ctx HAL适配器上下文。
 * @param data Service长期持有的接收缓冲区。
 * @param capacity 缓冲区容量。
 * @return DMA启动结果。
 * @note 禁用半传输中断，避免固定小响应被HT和IDLE重复提交。
 */
static zdt_turntable_status_t zdt_hal_rx(
    void *ctx, uint8_t *data, size_t capacity)
{
    zdt_turntable_uart7_hal_t *adapter =
        (zdt_turntable_uart7_hal_t *)ctx;
    HAL_StatusTypeDef status;

    if ((adapter == NULL) || !adapter->bound || (data == NULL) ||
        (capacity == 0U) || (capacity > UINT16_MAX)) {
        return ZDT_TURNTABLE_ERR_PARAM;
    }
    status = HAL_UARTEx_ReceiveToIdle_DMA(
        adapter->uart, data, (uint16_t)capacity);
    if (status == HAL_OK) {
        __HAL_DMA_DISABLE_IT(adapter->uart->hdmarx, DMA_IT_HT);
    }
    return zdt_map_hal(status);
}

/**
 * @brief 同步终止UART7收发并清理相关待处理中断。
 * @param ctx HAL适配器上下文。
 * @return HAL abort结果。
 * @note 用于完成、超时和错误恢复，防止迟到事件污染下一笔事务。
 */
static zdt_turntable_status_t zdt_hal_abort(void *ctx)
{
    zdt_turntable_uart7_hal_t *adapter =
        (zdt_turntable_uart7_hal_t *)ctx;
    HAL_StatusTypeDef status;

    if ((adapter == NULL) || !adapter->bound) {
        return ZDT_TURNTABLE_ERR_NOT_INIT;
    }
    status = HAL_UART_Abort(adapter->uart);
    if (status == HAL_OK) {
        /* 清除UART/RX DMA遗留状态后，下一笔事务会重新建立RX窗口。 */
        __HAL_UART_CLEAR_PEFLAG(adapter->uart);
        HAL_NVIC_ClearPendingIRQ(UART7_IRQn);
        HAL_NVIC_ClearPendingIRQ(DMA1_Stream3_IRQn);
    }
    return zdt_map_hal(status);
}

/** @copydoc zdt_turntable_uart7_hal_bind() */
zdt_turntable_status_t zdt_turntable_uart7_hal_bind(
    zdt_turntable_uart7_hal_t *adapter,
    zdt_turntable_service_t *service,
    zdt_turntable_port_t *port)
{
    if ((adapter == NULL) || (service == NULL) || (port == NULL)) {
        return ZDT_TURNTABLE_ERR_PARAM;
    }
    (void)memset(adapter, 0, sizeof(*adapter));
    adapter->uart = &huart7;
    adapter->service = service;
    adapter->bound = true;
    port->tx_start = zdt_hal_tx;
    port->rx_start = zdt_hal_rx;
    port->abort = zdt_hal_abort;
    port->ctx = adapter;
    return ZDT_TURNTABLE_OK;
}

/**
 * @brief 判断公共UART事件是否属于当前ZDT直连适配器。
 * @param adapter 适配器对象。
 * @param huart 事件携带的UART句柄。
 * @return 适配器完整绑定且句柄相同时返回true。
 */
static bool zdt_owns(
    const zdt_turntable_uart7_hal_t *adapter,
    const UART_HandleTypeDef *huart)
{
    return (adapter != NULL) && adapter->bound &&
           (adapter->service != NULL) && (adapter->uart == huart);
}

/** @copydoc zdt_turntable_uart7_hal_handle_tx_complete() */
bool zdt_turntable_uart7_hal_handle_tx_complete(
    zdt_turntable_uart7_hal_t *adapter, UART_HandleTypeDef *huart)
{
    if (!zdt_owns(adapter, huart)) {
        return false;
    }
    zdt_turntable_service_on_tx_complete_isr(adapter->service);
    return true;
}

/** @copydoc zdt_turntable_uart7_hal_handle_rx_event() */
bool zdt_turntable_uart7_hal_handle_rx_event(
    zdt_turntable_uart7_hal_t *adapter,
    UART_HandleTypeDef *huart,
    uint16_t rx_len)
{
    if (!zdt_owns(adapter, huart)) {
        return false;
    }
    zdt_turntable_service_on_rx_event_isr(adapter->service, rx_len);
    return true;
}

/** @copydoc zdt_turntable_uart7_hal_handle_error() */
bool zdt_turntable_uart7_hal_handle_error(
    zdt_turntable_uart7_hal_t *adapter, UART_HandleTypeDef *huart)
{
    if (!zdt_owns(adapter, huart)) {
        return false;
    }
    zdt_turntable_service_on_error_isr(adapter->service);
    return true;
}
