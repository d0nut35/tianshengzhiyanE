/**
 * @file    it_dispatch.h
 * @brief   中断适配层：独占 HAL UART ISR 回调，按句柄路由到各模块
 * @author  haoyu
 * @note    - 全工程唯一定义 HAL_UARTEx_RxEventCallback / HAL_UART_ErrorCallback
 *          - 本层无对外函数：回调由 HAL 中断直接调用（弱符号覆盖）
 *          - 接线表（huart→模块）集中在 it_dispatch.c，与 system_assembly 同源
 */

#ifndef IT_DISPATCH_H
#define IT_DISPATCH_H

#ifdef __cplusplus
extern "C" {
#endif

/* 本层不暴露接口；存在仅为登记中断适配职责并保持目录一致。 */

#ifdef __cplusplus
}
#endif

#endif /* IT_DISPATCH_H */
