/**
 * @file    ble_adaption.h
 * @brief   BLE 平台适配层（STM32 HAL USART1 + DMA 空闲接收）
 * @author  haoyu
 * @note    - 本层含 HAL，向上只暴露 init / ISR 入口 / 任务处理入口
 *          - UART 句柄等板级接线在 .c；解析器实例本层持有
 *          - ISR 只拷帧+入队+重启；解析换算放任务；OS 队列归 Service
 *          - 协议量→物理量换算（deg→rad）在本层完成后上抛
 */

#ifndef BLE_ADAPTION_H
#define BLE_ADAPTION_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "ble.h"

/* 原始字节块回调：ISR 上下文，返回前须拷贝入队 */
typedef void (*ble_rx_cb_t)(const uint8_t *data, uint16_t len);

/* 一帧导航命令上抛回调：任务上下文，已换算为物理量，由装配根注入 */
typedef void (*ble_nav_out_t)(ble_nav_mode_t mode,
                              float x_mm, float y_mm, float yaw_rad,
                              float v_mm_s, float w_rad_s);

/**
 * @brief  实例化适配层：建解析器、注入回调、启动 DMA 空闲接收
 * @param  on_rx_raw 收到原始字节块回调（供 ISR 入队），不可为空
 * @param  on_nav    解析出一帧导航命令回调（已换算），不可为空
 * @retval BLE_OK / BLE_ERR_PARAM / BLE_ERR
 */
ble_status_t ble_adp_init(ble_rx_cb_t on_rx_raw, ble_nav_out_t on_nav);

/**
 * @brief  UART 接收完成 ISR 入口（由 it_dispatch 按句柄路由调用）
 * @param  size 本次接收字节数
 * @retval BLE_OK / BLE_ERR_PARAM / BLE_ERR_INIT
 */
ble_status_t ble_adp_rx_isr(uint16_t size);

/**
 * @brief  UART 错误 ISR 入口：重启接收，不动解析器
 * @retval BLE_OK / BLE_ERR_INIT / BLE_ERR
 */
ble_status_t ble_adp_err_isr(void);

/**
 * @brief  任务侧处理出队的原始字节块：喂解析器→换算→上抛
 * @param  data 字节缓冲
 * @param  len  字节长度
 * @retval BLE_OK / BLE_ERR_PARAM / BLE_ERR_INIT
 */
ble_status_t ble_adp_process(const uint8_t *data, uint16_t len);

#ifdef __cplusplus
}
#endif

#endif /* BLE_ADAPTION_H */
