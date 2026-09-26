/**
 * @file    zdt_turntable_uart7_hal.h
 * @brief   ZDT转盘电机UART7直连STM32F7 HAL适配器。
 *
 * 本适配器只用于当前独立直连测试。以后接UART7复用通道2时保留Core、
 * Service和Device，只替换本transport，不能复制第二套ZDT协议。
 */

#ifndef ZDT_TURNTABLE_UART7_HAL_H
#define ZDT_TURNTABLE_UART7_HAL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#include "stm32f7xx_hal.h"
#include "zdt_turntable_service.h"

/**
 * @brief UART7直连适配器状态。
 * @note 本对象只保存硬件和Service指针；DMA缓冲区由Service持有。
 */
typedef struct {
    UART_HandleTypeDef *uart;
    zdt_turntable_service_t *service;
    bool bound;
} zdt_turntable_uart7_hal_t;

/**
 * @brief 绑定UART7并生成Service port函数表。
 * @param adapter 待初始化适配器。
 * @param service 接收ISR事件的事务Service。
 * @param port 输出给Service配置的传输函数表。
 * @return 绑定结果；本函数不启动DMA，也不注册全局HAL回调。
 * @warning 直连测试期间UART7必须由ZDT独占，不能同时运行复用器或IC卡测试。
 */
zdt_turntable_status_t zdt_turntable_uart7_hal_bind(
    zdt_turntable_uart7_hal_t *adapter,
    zdt_turntable_service_t *service,
    zdt_turntable_port_t *port);

/**
 * @brief 过滤并认领UART7 DMA发送完成事件。
 * @param adapter 已绑定适配器。
 * @param huart 产生回调的UART句柄。
 * @return 句柄属于本适配器且事件已转发时返回true。
 * @warning 仅由公共uart_dispatch在HAL回调上下文调用。
 */
bool zdt_turntable_uart7_hal_handle_tx_complete(
    zdt_turntable_uart7_hal_t *adapter, UART_HandleTypeDef *huart);

/**
 * @brief 过滤并认领UART7 ReceiveToIdle事件。
 * @param adapter 已绑定适配器。
 * @param huart 产生回调的UART句柄。
 * @param rx_len DMA缓冲区有效字节数。
 * @return 句柄属于本适配器且事件已转发时返回true。
 * @warning 仅发布事件，协议解析不在ISR中执行。
 */
bool zdt_turntable_uart7_hal_handle_rx_event(
    zdt_turntable_uart7_hal_t *adapter,
    UART_HandleTypeDef *huart,
    uint16_t rx_len);

/**
 * @brief 过滤并认领UART7错误事件。
 * @param adapter 已绑定适配器。
 * @param huart 产生回调的UART句柄。
 * @return 句柄属于本适配器且事件已转发时返回true。
 * @warning 仅由公共uart_dispatch在HAL回调上下文调用。
 */
bool zdt_turntable_uart7_hal_handle_error(
    zdt_turntable_uart7_hal_t *adapter, UART_HandleTypeDef *huart);

#ifdef __cplusplus
}
#endif

#endif /* ZDT_TURNTABLE_UART7_HAL_H */
