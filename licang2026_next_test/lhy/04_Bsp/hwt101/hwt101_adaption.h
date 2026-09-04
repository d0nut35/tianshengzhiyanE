/**
 * @file    hwt101_adaption.h
 * @brief   HWT101 平台适配层（STM32 HAL UART + DMA 双缓冲，懒扫描）
 * @author  haoyu
 * @note    - 本层含 HAL，持有 DMA 双缓冲与最新值缓存
 *          - 中断(rx_isr)只记长度 + 切缓冲 + 重启 DMA，
 *            不解析（最短中断）
 *          - 解析在 read(任务上下文) 里对最近收完的缓冲调
 *            hwt101_parse
 */

#ifndef HWT101_ADAPTION_H
#define HWT101_ADAPTION_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "hwt101.h"

/**
 * @brief  实例化：配置 huart2 DMA 双缓冲接收资源，清空缓存
 * @retval HWT101_OK / HWT101_ERR_PARAM / HWT101_ERR
 */
hwt101_status_t hwt101_adp_init(void);

/**
 * @brief  启动 UART DMA 接收
 * @retval HWT101_OK / HWT101_ERR_INIT / HWT101_ERR
 */
hwt101_status_t hwt101_adp_start(void);

/**
 * @brief  读最新角速度 / 角度（任务上下文调用，内部懒扫描 + 解析）
 * @param  gyro_dps 输出角速度 Wz，单位 度/秒（附带量，本段存在才写）
 * @param  yaw_deg  输出偏航角，单位 度
 * @note   扫描最近收完的缓冲调 hwt101_parse，以 yaw 帧为准；
 *         本次无有效 yaw 帧返回 ERR_RES 且不写输出，
 *         由上层处理（沿用上次 / 报 IMU 故障）
 * @retval HWT101_OK / HWT101_ERR_PARAM / HWT101_ERR_INIT /
 *         HWT101_ERR_RES(无新帧)
 */
hwt101_status_t hwt101_adp_read(float *gyro_dps, float *yaw_deg);

/**
 * @brief  软件标定：把当前航向标定为指定角度（任务上下文调用）
 * @param  yaw_deg 期望的当前航向，单位 度
 * @note   读一帧原始 yaw 算偏置，此后 read 输出均含该偏置；
 *         无新帧返回 ERR_RES，由上层重试
 * @retval HWT101_OK / HWT101_ERR_INIT / HWT101_ERR_RES(无新帧)
 */
hwt101_status_t hwt101_adp_set_yaw(float yaw_deg);

/**
 * @brief  向 HWT101 写一个配置寄存器（阻塞发送 FF AA reg lo hi）
 * @param  reg 寄存器地址（HWT101_REG_*）
 * @param  lo  数据低字节
 * @param  hi  数据高字节
 * @note   任务上下文调用；不依赖 adp_init，帧间等待由调用方保证
 * @retval HWT101_OK / HWT101_ERR_TMO / HWT101_ERR
 */
hwt101_status_t hwt101_adp_write_reg(uint8_t reg, uint8_t lo, uint8_t hi);

/**
 * @brief  UART 接收事件 ISR 入口（由 it_dispatch 按句柄路由调用）
 * @param  size 本次接收字节数
 * @note   仅记录有效长度 + 切到另一缓冲 + 重启 DMA，不解析
 * @retval HWT101_OK / HWT101_ERR_INIT / HWT101_ERR
 */
hwt101_status_t hwt101_adp_rx_isr(uint16_t size);

/**
 * @brief  UART 错误 ISR 入口：重启接收
 * @retval HWT101_OK / HWT101_ERR_INIT / HWT101_ERR
 */
hwt101_status_t hwt101_adp_err_isr(void);

#ifdef __cplusplus
}
#endif

#endif /* HWT101_ADAPTION_H */
