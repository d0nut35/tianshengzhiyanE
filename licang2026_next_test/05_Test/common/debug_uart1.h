/**
 * @file    debug_uart1.h
 * @brief   USART1调试端口的DMA空闲接收与阻塞文本发送封装。
 *
 * 该模块仅给05_Test使用：PA9/PA10接CH340，接收电脑命令并返回可读文本。
 * RX回调经公共uart_dispatch认领；短文本TX使用阻塞HAL发送，因此无需TX DMA。
 */

#ifndef DEBUG_UART1_H
#define DEBUG_UART1_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "stm32f7xx_hal.h"

#define DEBUG_UART1_RX_BUFFER_SIZE 64U

typedef struct {
    bool initialized;
    uint8_t dma_rx_buffer[DEBUG_UART1_RX_BUFFER_SIZE];
    uint8_t message[DEBUG_UART1_RX_BUFFER_SIZE];
    volatile uint16_t message_len;
    volatile bool message_ready;
    volatile uint32_t overflow_count;
    volatile uint32_t error_count;
    uint8_t dispatch_handle;
    bool dispatch_registered;
} debug_uart1_t;

/**
 * @brief 注册USART1路由并启动ReceiveToIdle DMA。
 * @param debug 调试端口实例。
 * @return true表示初始化成功，false表示参数、路由或DMA启动失败。
 */
bool debug_uart1_init(debug_uart1_t *debug);

/**
 * @brief 停止 USART1 的 DMA 接收并注销公共回调路由。
 *
 * 测试初始化中途失败或切换测试模式时必须成对释放资源，否则旧 handler
 * 仍可能认领后续 USART1 中断。
 * @param debug 调试端口实例。
 */
void debug_uart1_deinit(debug_uart1_t *debug);

/**
 * @brief 原子取走最近一条电脑调试命令。
 * @param debug 调试端口实例。
 * @param data 输出缓冲区。
 * @param capacity 输出缓冲区容量。
 * @param len 输出实际消息长度。
 * @return true表示取得消息；false表示参数无效或当前没有完整消息。
 * @note 上一条消息未取走时到达的新消息会计入overflow并丢弃。
 */
bool debug_uart1_take_message(
    debug_uart1_t *debug,
    uint8_t *data,
    size_t capacity,
    size_t *len);

/**
 * @brief 通过USART1阻塞发送原始字节。
 * @param debug 调试端口实例。
 * @param data 待发送数据。
 * @param len 数据长度。
 * @return true表示HAL发送完成，否则返回false。
 * @warning 只允许在任务或裸机主循环普通上下文调用。
 */
bool debug_uart1_write(debug_uart1_t *debug, const uint8_t *data, size_t len);

/**
 * @brief 通过USART1阻塞发送零结尾文本。
 * @param debug 调试端口实例。
 * @param text 零结尾字符串。
 * @return true表示HAL发送完成，否则返回false。
 * @warning 只允许在任务或裸机主循环普通上下文调用。
 */
bool debug_uart1_write_text(debug_uart1_t *debug, const char *text);

#ifdef __cplusplus
}
#endif

#endif /* DEBUG_UART1_H */
