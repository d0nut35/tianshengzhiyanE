/**
 * @file    lsc16_stm32_hal.h
 * @brief   LSC16 Core到STM32F7 UART HAL/DMA的板级适配器。
 */

#ifndef LSC16_STM32_HAL_H
#define LSC16_STM32_HAL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>

#include "lsc16_core.h"
#include "stm32f7xx_hal.h"

typedef struct {
    UART_HandleTypeDef *uart;
} lsc16_stm32_hal_config_t;

typedef struct {
    bool bound;
    lsc16_t *device;
    lsc16_stm32_hal_config_t config;
} lsc16_stm32_hal_t;

/**
 * @brief 使用CubeMX生成的huart8填充默认板级配置。
 * @param config 输出板级配置。
 * @note 只填写资源描述，不初始化UART或DMA。
 */
void lsc16_stm32_hal_make_uart8_config(lsc16_stm32_hal_config_t *config);

/**
 * @brief 绑定HAL资源并生成可注入LSC16 Core的port函数表。
 * @param adapter HAL适配器实例。
 * @param device LSC16协议Core实例。
 * @param config UART8板级配置。
 * @param port 输出平台无关端口能力。
 * @return LSC16_OK表示成功，否则返回参数或状态错误。
 * @pre MX_DMA_Init()和MX_UART8_Init()已经完成。
 */
lsc16_status_t lsc16_stm32_hal_bind(
    lsc16_stm32_hal_t *adapter,
    lsc16_t *device,
    const lsc16_stm32_hal_config_t *config,
    lsc16_port_t *port);

/**
 * @brief 认领并转发匹配UART的DMA发送完成事件。
 * @param adapter HAL适配器实例。
 * @param huart 产生事件的HAL UART句柄。
 * @return true表示句柄匹配并已处理；false表示应继续路由。
 * @warning 由公共uart_dispatch在ISR上下文调用。
 */
bool lsc16_stm32_hal_handle_tx_complete(
    lsc16_stm32_hal_t *adapter,
    UART_HandleTypeDef *huart);

/**
 * @brief 认领并转发匹配UART的ReceiveToIdle事件。
 * @param adapter HAL适配器实例。
 * @param huart 产生事件的HAL UART句柄。
 * @param rx_len 本次DMA缓冲区有效长度。
 * @return true表示句柄匹配并已处理；false表示应继续路由。
 * @warning 由公共uart_dispatch在ISR上下文调用。
 */
bool lsc16_stm32_hal_handle_rx_event(
    lsc16_stm32_hal_t *adapter,
    UART_HandleTypeDef *huart,
    uint16_t rx_len);

/**
 * @brief 认领并转发匹配UART的错误事件。
 * @param adapter HAL适配器实例。
 * @param huart 产生事件的HAL UART句柄。
 * @return true表示句柄匹配并已处理；false表示应继续路由。
 * @warning 由公共uart_dispatch在ISR上下文调用。
 */
bool lsc16_stm32_hal_handle_error(
    lsc16_stm32_hal_t *adapter,
    UART_HandleTypeDef *huart);

#ifdef __cplusplus
}
#endif

#endif /* LSC16_STM32_HAL_H */
