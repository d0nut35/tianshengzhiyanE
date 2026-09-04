/**
 * @file    debug_uart1.c
 * @brief   USART1调试命令接收和纯文本发送实现。
 */

#include "debug_uart1.h"

#include <string.h>

#include "uart_dispatch.h"
#include "usart.h"

/**
 * @brief 重新挂接USART1 ReceiveToIdle Normal DMA。
 * @param debug 调试串口对象。
 * @return DMA启动成功返回true。
 * @note 关闭HT中断，避免串口助手的一条短命令被误拆成两次回调。
 */
static bool debug_uart1_restart_rx(debug_uart1_t *debug)
{
    HAL_StatusTypeDef status = HAL_UARTEx_ReceiveToIdle_DMA(
        &huart1,
        debug->dma_rx_buffer,
        sizeof(debug->dma_rx_buffer));

    if ((status == HAL_OK) && (huart1.hdmarx != NULL)) {
        __HAL_DMA_DISABLE_IT(huart1.hdmarx, DMA_IT_HT);
    }
    return (status == HAL_OK);
}

/**
 * @brief 过滤USART1发送完成事件。
 * @param ctx 调试串口对象。
 * @param huart 产生回调的UART句柄。
 * @return 事件属于USART1时返回true。
 * @note 当前测试使用阻塞TX，本handler只防止公共路由继续误投递该句柄。
 */
static bool debug_uart1_dispatch_tx(void *ctx, UART_HandleTypeDef *huart)
{
    (void)ctx;
    return (huart == &huart1);
}

/**
 * @brief 复制一条USART1调试命令并立即续接DMA。
 * @param ctx 调试串口对象。
 * @param huart 产生回调的UART句柄。
 * @param rx_len 本次有效字节数。
 * @return 事件属于USART1时返回true。
 * @warning ISR中只复制短命令；上一条尚未消费时丢弃新数据而不覆盖旧命令。
 */
static bool debug_uart1_dispatch_rx(
    void *ctx,
    UART_HandleTypeDef *huart,
    uint16_t rx_len)
{
    debug_uart1_t *debug = (debug_uart1_t *)ctx;

    if ((debug == NULL) || (huart != &huart1)) {
        return false;
    }
    if (rx_len > sizeof(debug->dma_rx_buffer)) {
        rx_len = sizeof(debug->dma_rx_buffer);
        ++debug->overflow_count;
    }
    if (debug->message_ready) {
        ++debug->overflow_count;
    } else if (rx_len > 0U) {
        (void)memcpy(debug->message, debug->dma_rx_buffer, rx_len);
        debug->message_len = rx_len;
        debug->message_ready = true;
    }
    if (!debug_uart1_restart_rx(debug)) {
        ++debug->error_count;
    }
    return true;
}

/**
 * @brief 处理USART1错误并重新建立调试命令接收。
 * @param ctx 调试串口对象。
 * @param huart 产生错误的UART句柄。
 * @return 事件属于USART1时返回true。
 * @warning ISR中不执行阻塞发送或字符串格式化。
 */
static bool debug_uart1_dispatch_error(void *ctx, UART_HandleTypeDef *huart)
{
    debug_uart1_t *debug = (debug_uart1_t *)ctx;

    if ((debug == NULL) || (huart != &huart1)) {
        return false;
    }
    ++debug->error_count;
    (void)HAL_UART_AbortReceive(&huart1);
    __HAL_UART_CLEAR_OREFLAG(&huart1);
    (void)debug_uart1_restart_rx(debug);
    return true;
}

/**
 * @brief 初始化测试专用USART1接收并注册公共UART路由。
 * @param debug 待初始化对象。
 * @return 初始化成功返回true。
 */
bool debug_uart1_init(debug_uart1_t *debug)
{
    uart_dispatch_handler_t handler = {0};

    if ((debug == NULL) || debug->initialized) {
        return false;
    }
    (void)memset(debug, 0, sizeof(*debug));
    debug->dispatch_handle = UART_DISPATCH_HANDLE_INVALID;
    handler.tx_complete = debug_uart1_dispatch_tx;
    handler.rx_event = debug_uart1_dispatch_rx;
    handler.error = debug_uart1_dispatch_error;
    handler.user_ctx = debug;
    if (!uart_dispatch_register(&handler, &debug->dispatch_handle)) {
        return false;
    }
    debug->dispatch_registered = true;
    if (!debug_uart1_restart_rx(debug)) {
        (void)uart_dispatch_unregister(debug->dispatch_handle);
        (void)memset(debug, 0, sizeof(*debug));
        return false;
    }
    debug->initialized = true;
    return true;
}

/**
 * @brief 停止USART1接收并撤销公共路由注册。
 * @param debug 调试串口对象。
 */
void debug_uart1_deinit(debug_uart1_t *debug)
{
    if ((debug == NULL) || !debug->initialized) {
        return;
    }

    /* 先停止 DMA，再注销路由，避免注销后的迟到中断落入无人处理状态。 */
    (void)HAL_UART_AbortReceive(&huart1);
    if (debug->dispatch_registered) {
        (void)uart_dispatch_unregister(debug->dispatch_handle);
    }
    (void)memset(debug, 0, sizeof(*debug));
}

/**
 * @brief 在普通上下文取走一条完整调试命令。
 * @param debug 调试串口对象。
 * @param data 接收命令副本的输出缓冲区。
 * @param capacity 输出缓冲区容量。
 * @param len 接收有效长度的输出对象。
 * @return 取到新命令返回true。
 * @note 复制和清pending标志时短暂关中断，避免与ISR并发覆盖。
 */
bool debug_uart1_take_message(
    debug_uart1_t *debug,
    uint8_t *data,
    size_t capacity,
    size_t *len)
{
    uint32_t primask;
    uint16_t message_len;

    if ((debug == NULL) || (data == NULL) || (len == NULL) ||
        !debug->initialized || (capacity == 0U)) {
        return false;
    }
    primask = __get_PRIMASK();
    __disable_irq();
    if (!debug->message_ready) {
        if (primask == 0U) {
            __enable_irq();
        }
        return false;
    }
    message_len = debug->message_len;
    if (message_len > capacity) {
        message_len = (uint16_t)capacity;
    }
    (void)memcpy(data, debug->message, message_len);
    debug->message_len = 0U;
    debug->message_ready = false;
    if (primask == 0U) {
        __enable_irq();
    }
    *len = message_len;
    return true;
}

/**
 * @brief 通过USART1阻塞发送一段测试结果。
 * @param debug 调试串口对象。
 * @param data 待发送数据。
 * @param len 数据长度。
 * @return HAL发送成功返回true。
 * @warning 仅允许在测试任务或主循环调用，禁止在ISR中使用。
 */
bool debug_uart1_write(debug_uart1_t *debug, const uint8_t *data, size_t len)
{
    if ((debug == NULL) || !debug->initialized || (data == NULL) ||
        (len == 0U) || (len > UINT16_MAX)) {
        return false;
    }
    return (HAL_UART_Transmit(&huart1, (uint8_t *)(uintptr_t)data,
        (uint16_t)len, 100U) == HAL_OK);
}

/**
 * @brief 通过USART1发送NUL结尾文本。
 * @param debug 调试串口对象。
 * @param text 待发送文本。
 * @return 发送成功返回true。
 */
bool debug_uart1_write_text(debug_uart1_t *debug, const char *text)
{
    return (text != NULL) && debug_uart1_write(
        debug,
        (const uint8_t *)text,
        strlen(text));
}
