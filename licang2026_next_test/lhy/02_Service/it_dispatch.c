/**
 * @file    it_dispatch.c
 * @brief   中断适配层：HAL UART 回调按句柄路由（电机 huart3~6 + 陀螺仪 huart2）
 * @author  haoyu
 * @note    - huart3/4/5/6 = 电机 1/2/3/4 回传，路由到 zdt_adp_rx_isr(idx, size)
 *          - huart2 = 陀螺仪(hwt101)；huart1 = BLE 待适配层落地后接入
 *          - idx 顺序须与 zdt_motor_adaption.c 的 g_cfg 一致
 *          - RX 正常路径高频，此处不打日志；仅异常分支记一笔
 */

#include "it_dispatch.h"

#include "main.h"
#include "usart.h"

#include "zdt_motor_adaption.h"
#include "hwt101_adaption.h"
#include "ble_adaption.h"

/* ===== 调试日志：0=不编译进固件，1=经 RTT 输出 ===== */
#ifndef IT_DISP_LOG_EN
#define IT_DISP_LOG_EN   1
#endif
#if IT_DISP_LOG_EN
#include "cat_log.h"
#define IT_LOGW(fmt, ...)  LOGW("[it] " fmt, ##__VA_ARGS__)
#else
#define IT_LOGW(fmt, ...)  do {} while (0)
#endif

/**
 * @brief  UART 空闲 / 接收事件回调（HAL 弱符号唯一实现）
 * @param  huart 触发的 UART 句柄
 * @param  size  本次收到的字节数
 * @note   ISR 上下文：仅路由，不阻塞、不打高频日志
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size)
{
    if (size == 0U) {
        return; /* 空闲无数据，忽略 */
    }
    if (huart == &huart3) {
        (void)zdt_adp_rx_isr(0U, size);
    } else if (huart == &huart4) {
        (void)zdt_adp_rx_isr(1U, size);
    } else if (huart == &huart5) {
        (void)zdt_adp_rx_isr(2U, size);
    } else if (huart == &huart6) {
        (void)zdt_adp_rx_isr(3U, size);
    } else if (huart == &huart2) {
        /* 陀螺仪：记长度+切缓冲+重启 */
        (void)hwt101_adp_rx_isr(size);
    } else if (huart == &huart1) {
        (void)ble_adp_rx_isr(size);         /* BLE：拷帧+入队+重启 */
    } else {
        /* 其它 UART 暂未接入 */
    }
}

/**
 * @brief  UART 错误回调（HAL 弱符号唯一实现）
 * @param  huart 出错的 UART 句柄
 * @note   重启对应电机接收以从错误中恢复
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart3) {
        (void)zdt_adp_err_isr(0U);
    } else if (huart == &huart4) {
        (void)zdt_adp_err_isr(1U);
    } else if (huart == &huart5) {
        (void)zdt_adp_err_isr(2U);
    } else if (huart == &huart6) {
        (void)zdt_adp_err_isr(3U);
    } else if (huart == &huart2) {
        (void)hwt101_adp_err_isr();         /* 陀螺仪：重启接收 */
    } else if (huart == &huart1) {
        (void)ble_adp_err_isr();            /* BLE：重启接收 */
    } else {
        IT_LOGW("error on unmapped uart");
    }
}
