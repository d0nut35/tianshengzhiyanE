/**
 * @file    ic_card_uart7_hal.h
 * @brief   IC卡读写器直连UART7的STM32F7 HAL适配器。
 *
 * 本适配器只用于“不经过复用板”的独立验证。正式接入复用通道1时保留
 * ic_card_core协议Core，另接mult_uart transport，不能同时启动两种UART7所有者。
 */

#ifndef IC_CARD_UART7_HAL_H
#define IC_CARD_UART7_HAL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#include "ic_card_core.h"
#include "stm32f7xx_hal.h"

typedef struct {
    UART_HandleTypeDef *uart;
} ic_card_uart7_hal_config_t;

typedef struct {
    ic_card_t *device;
    UART_HandleTypeDef *uart;
} ic_card_uart7_hal_t;

/**
 * @brief 使用CubeMX生成的huart7填充默认直连配置。
 * @param config 输出板级配置。
 * @note 只填写资源描述，不初始化UART、DMA或GPIO。
 */
void ic_card_uart7_hal_make_config(ic_card_uart7_hal_config_t *config);

/**
 * @brief 绑定HAL资源并生成可注入协议Core的port函数表。
 * @param adapter HAL适配器实例。
 * @param device IC卡协议Core实例。
 * @param config UART7板级配置。
 * @param port 输出平台无关端口能力。
 * @return IC_CARD_OK表示成功，否则返回参数或状态错误。
 * @pre MX_DMA_Init()和MX_UART7_Init()已经完成。
 */
ic_card_status_t ic_card_uart7_hal_bind(
    ic_card_uart7_hal_t *adapter,
    ic_card_t *device,
    const ic_card_uart7_hal_config_t *config,
    ic_card_port_t *port);

/**
 * @brief 认领并转发匹配UART的DMA发送完成事件。
 * @param adapter HAL适配器实例。
 * @param huart 产生事件的HAL UART句柄。
 * @return true表示句柄匹配并已处理；false表示应继续路由。
 * @warning 由公共uart_dispatch在ISR上下文调用。
 */
bool ic_card_uart7_hal_handle_tx_complete(
    ic_card_uart7_hal_t *adapter,
    UART_HandleTypeDef *huart);

/**
 * @brief 认领并转发匹配UART的ReceiveToIdle事件。
 * @param adapter HAL适配器实例。
 * @param huart 产生事件的HAL UART句柄。
 * @param rx_len 本次DMA缓冲区有效长度。
 * @return true表示句柄匹配并已处理；false表示应继续路由。
 * @warning 由公共uart_dispatch在ISR上下文调用。
 */
bool ic_card_uart7_hal_handle_rx_event(
    ic_card_uart7_hal_t *adapter,
    UART_HandleTypeDef *huart,
    uint16_t rx_len);

/**
 * @brief 认领并转发匹配UART的错误事件。
 * @param adapter HAL适配器实例。
 * @param huart 产生事件的HAL UART句柄。
 * @return true表示句柄匹配并已处理；false表示应继续路由。
 * @warning 由公共uart_dispatch在ISR上下文调用。
 */
bool ic_card_uart7_hal_handle_error(
    ic_card_uart7_hal_t *adapter,
    UART_HandleTypeDef *huart);

#ifdef __cplusplus
}
#endif

#endif /* IC_CARD_UART7_HAL_H */
