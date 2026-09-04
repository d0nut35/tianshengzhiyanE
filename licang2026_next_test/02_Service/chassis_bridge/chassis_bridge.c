/**
 * @file    chassis_bridge.c
 * @brief   lhy底盘子系统接入桥实现。
 *
 * handler只运行在HAL回调上下文：比较句柄、转交lhy适配层ISR入口，
 * 不解析帧、不阻塞、不打日志。协议解析与DMA重启由lhy适配层自行完成。
 */

#include "chassis_bridge.h"

#include <stdbool.h>
#include <string.h>

#include "app_main.h"
#include "hwt101_adaption.h"
#include "uart_dispatch.h"
#include "usart.h"
#include "zdt_motor_adaption.h"

typedef struct {
    bool initialized;
    bool booted;
    bool dispatch_registered;
    uart_dispatch_handle_t dispatch_handle;
} chassis_bridge_context_t;

static chassis_bridge_context_t g_chassis_bridge;

/* 电机槽位→回传UART；顺序须与lhy zdt_motor_adaption.c的g_cfg一致。 */
static UART_HandleTypeDef *const g_motor_uarts[ZDT_ADP_MOTOR_NUM] = {
    &huart3,
    &huart4,
    &huart5,
    &huart6,
};

/**
 * @brief 按UART句柄查找电机槽位。
 * @param huart 事件携带的UART句柄。
 * @param idx 输出电机槽位。
 * @return 句柄属于某台电机回传UART时返回true。
 */
static bool chassis_bridge_motor_idx(
    const UART_HandleTypeDef *huart,
    uint8_t *idx)
{
    uint8_t i;

    for (i = 0U; i < ZDT_ADP_MOTOR_NUM; ++i) {
        if (g_motor_uarts[i] == huart) {
            *idx = i;
            return true;
        }
    }
    return false;
}

/**
 * @brief 把ReceiveToIdle事件转交给lhy电机或陀螺仪适配层。
 * @param ctx 未使用。
 * @param huart 产生事件的UART句柄。
 * @param rx_len 本次DMA有效字节数。
 * @return 句柄属于底盘UART时返回true。
 * @note 与lhy原it_dispatch一致：rx_len为0时认领事件但不处理。
 */
static bool chassis_bridge_dispatch_rx(
    void *ctx,
    UART_HandleTypeDef *huart,
    uint16_t rx_len)
{
    uint8_t idx;

    (void)ctx;
    if (huart == &huart2) {
        if (rx_len != 0U) {
            (void)hwt101_adp_rx_isr(rx_len);
        }
        return true;
    }
    if (chassis_bridge_motor_idx(huart, &idx)) {
        if (rx_len != 0U) {
            (void)zdt_adp_rx_isr(idx, rx_len);
        }
        return true;
    }
    return false;
}

/**
 * @brief 把UART错误事件转交给lhy适配层重启接收。
 * @param ctx 未使用。
 * @param huart 产生错误的UART句柄。
 * @return 句柄属于底盘UART时返回true。
 */
static bool chassis_bridge_dispatch_error(void *ctx, UART_HandleTypeDef *huart)
{
    uint8_t idx;

    (void)ctx;
    if (huart == &huart2) {
        (void)hwt101_adp_err_isr();
        return true;
    }
    if (chassis_bridge_motor_idx(huart, &idx)) {
        (void)zdt_adp_err_isr(idx);
        return true;
    }
    return false;
}

/** @copydoc chassis_bridge_init() */
chassis_bridge_status_t chassis_bridge_init(void)
{
    chassis_bridge_context_t *ctx = &g_chassis_bridge;
    uart_dispatch_handler_t handler = {0};

    if (ctx->initialized) {
        return CHASSIS_BRIDGE_ERR_STATE;
    }
    (void)memset(ctx, 0, sizeof(*ctx));
    ctx->dispatch_handle = UART_DISPATCH_HANDLE_INVALID;

    /* 只登记RX与错误；huart3的DMA发送完成事件lhy不使用。 */
    handler.rx_event = chassis_bridge_dispatch_rx;
    handler.error = chassis_bridge_dispatch_error;
    handler.user_ctx = ctx;
    if (!uart_dispatch_register(&handler, &ctx->dispatch_handle)) {
        return CHASSIS_BRIDGE_ERR_IO;
    }
    ctx->dispatch_registered = true;
    ctx->initialized = true;
    return CHASSIS_BRIDGE_OK;
}

/** @copydoc chassis_bridge_boot() */
chassis_bridge_status_t chassis_bridge_boot(void)
{
    chassis_bridge_context_t *ctx = &g_chassis_bridge;

    if (!ctx->initialized || ctx->booted) {
        return CHASSIS_BRIDGE_ERR_STATE;
    }
    if (app_init() != APP_OK) {
        return CHASSIS_BRIDGE_ERR_APP;
    }
    ctx->booted = true;
    return CHASSIS_BRIDGE_OK;
}
