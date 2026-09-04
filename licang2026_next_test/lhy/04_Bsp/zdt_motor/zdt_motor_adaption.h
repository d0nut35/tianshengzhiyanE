/**
 * @file    zdt_motor_adaption.h
 * @brief   ZDT 电机平台适配层（STM32 HAL UART + DMA）
 * @author  haoyu
 * @note    - 本层含 HAL，向上只暴露对象句柄与 ISR 入口
 *          - 板级接线（地址/方向/UART）在 .c 配置表中
 *          - 收到一帧经 zdt_rx_cb_t 回调上抛，OS 入队由 Service 做
 */

#ifndef ZDT_MOTOR_ADAPTION_H
#define ZDT_MOTOR_ADAPTION_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "zdt_motor.h"

#define ZDT_ADP_MOTOR_NUM   4U    /* 板载电机数（四麦轮） */

/* 收到一帧回调：ISR 上下文，返回前需用完/拷贝 data */
typedef void (*zdt_rx_cb_t)(zdt_motor_t *motor,
                            const uint8_t *data,
                            uint16_t len);

/**
 * @brief  实例化板载电机与电机组，注入 HAL 收发端口
 * @param  on_frame 收到一帧的回调，不可为空
 * @retval ZDT_OK / ZDT_ERR_PARAM / ZDT_ERR
 */
zdt_status_t zdt_adp_init(zdt_rx_cb_t on_frame);

/**
 * @brief  启动各电机 UART 接收（使能/上报为 TX，经 handler 串行下发）
 * @retval ZDT_OK / ZDT_ERR_INIT / ZDT_ERR
 */
zdt_status_t zdt_adp_start(void);

/**
 * @brief  取电机组句柄（用于组速度控制）
 * @retval 电机组指针；未初始化返回 NULL
 */
zdt_group_t *zdt_adp_group(void);

/**
 * @brief  取单电机句柄（用于读位置 / 设速度）
 * @param  idx 电机下标，0..ZDT_ADP_MOTOR_NUM-1
 * @retval 电机指针；越界或未初始化返回 NULL
 */
zdt_motor_t *zdt_adp_motor(uint8_t idx);

/**
 * @brief  UART 接收完成 ISR 入口（由 it_dispatch 按句柄路由调用）
 * @param  idx  电机下标
 * @param  size 本次接收字节数
 * @retval ZDT_OK / ZDT_ERR_PARAM / ZDT_ERR_INIT
 */
zdt_status_t zdt_adp_rx_isr(uint8_t idx, uint16_t size);

/**
 * @brief  UART 错误 ISR 入口：重启该电机接收
 * @param  idx 电机下标
 * @retval ZDT_OK / ZDT_ERR_PARAM / ZDT_ERR_INIT
 */
zdt_status_t zdt_adp_err_isr(uint8_t idx);

#ifdef __cplusplus
}
#endif

#endif /* ZDT_MOTOR_ADAPTION_H */
