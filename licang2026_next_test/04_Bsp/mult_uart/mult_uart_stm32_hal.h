/**
 * @file    mult_uart_stm32_hal.h
 * @brief   mult_uart Core 的 STM32F7 HAL 适配层
 * @note    本层只负责把 Core port 操作翻译为 GPIO/UART/DMA HAL 调用。
 */

#ifndef MULT_UART_STM32_HAL_H
#define MULT_UART_STM32_HAL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#include "mult_uart_core.h"
#include "stm32f7xx_hal.h"

/** @brief 一条 STM32 UART 复用总线所需的具体硬件资源。 */
typedef struct {
    UART_HandleTypeDef *huart;
    GPIO_TypeDef *select_port;
    uint16_t select_a_pin;
    uint16_t select_b_pin;
    GPIO_TypeDef *enable_port;
    uint16_t enable_pin;
    IRQn_Type uart_irqn;
    IRQn_Type rx_dma_irqn;
} mult_uart_stm32_hal_config_t;

/**
 * @brief HAL adapter 运行上下文
 *
 * bus 用于把 HAL ISR 回调路由回对应 Core 对象；config 保存由 system
 * assembly 注入的 UART、GPIO 和 IRQ 资源。对象应使用静态存储期。
 */
typedef struct {
    mult_uart_bus_t *bus;
    mult_uart_stm32_hal_config_t config;
    bool bound;
    volatile uint32_t uart_error_count;
    volatile uint32_t last_uart_error;
} mult_uart_stm32_hal_t;

/**
 * @brief 生成F7正式硬件绑定：UART7 + PD9(A)/PD10(B)/PD11(EN/INH)。
 * @param config 输出板级配置。
 * @note 仅填写资源描述，不初始化 GPIO、UART 或 DMA。
 */
void mult_uart_stm32_hal_make_uart7_config(
    mult_uart_stm32_hal_config_t *config);

/**
 * @brief 校验并保存硬件资源，同时生成可注入 Core 的 port。
 * @param adapter HAL适配器实例。
 * @param bus 复用总线Core实例。
 * @param config UART、GPIO和IRQ板级配置。
 * @param port 输出平台无关端口能力。
 * @return MULT_UART_OK表示成功，否则返回参数或状态错误。
 * @pre MX_GPIO_Init()、MX_DMA_Init() 和 MX_UART7_Init() 已完成。
 */
mult_uart_status_t mult_uart_stm32_hal_bind(
    mult_uart_stm32_hal_t *adapter,
    mult_uart_bus_t *bus,
    const mult_uart_stm32_hal_config_t *config,
    mult_uart_port_t *port);

/**
 * @brief 将匹配UART的HAL TX完成回调转交给Core。
 * @param adapter HAL适配器实例。
 * @param huart 产生事件的HAL UART句柄。
 * @return true表示句柄匹配并已处理；false表示继续路由。
 * @warning 由公共uart_dispatch在ISR上下文调用。
 */
bool mult_uart_stm32_hal_handle_tx_complete(
    mult_uart_stm32_hal_t *adapter,
    UART_HandleTypeDef *huart);

/**
 * @brief 将匹配UART的HAL ReceiveToIdle事件转交给Core。
 * @param adapter HAL适配器实例。
 * @param huart 产生事件的HAL UART句柄。
 * @param rx_len 本次DMA缓冲区有效长度。
 * @return true表示句柄匹配并已处理；false表示继续路由。
 * @warning 由公共uart_dispatch在ISR上下文调用。
 */
bool mult_uart_stm32_hal_handle_rx_event(
    mult_uart_stm32_hal_t *adapter,
    UART_HandleTypeDef *huart,
    uint16_t rx_len);

/**
 * @brief 将匹配UART的HAL错误及原始ErrorCode转交给Core。
 * @param adapter HAL适配器实例。
 * @param huart 产生事件的HAL UART句柄。
 * @return true表示句柄匹配并已处理；false表示继续路由。
 * @warning 由公共uart_dispatch在ISR上下文调用。
 */
bool mult_uart_stm32_hal_handle_error(
    mult_uart_stm32_hal_t *adapter,
    UART_HandleTypeDef *huart);

#ifdef __cplusplus
}
#endif

#endif /* MULT_UART_STM32_HAL_H */
