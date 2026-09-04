/**
 * @file    uart_dispatch.h
 * @brief   STM32 HAL UART全局回调的唯一注册与分发入口。
 *
 * HAL的weak callback是全工程唯一符号。各设备模块不能分别定义
 * HAL_UART_TxCpltCallback()等函数，否则后续加入UART8舵控、IC或二维码
 * 时会产生重复符号或错误认领。本模块拥有全局入口，各适配器只注册
 * “先检查UART句柄、匹配后处理”的轻量handler。
 *
 * 本模块只路由“DMA发送完成、空闲接收、UART错误”三类底层事件，
 * 不解析IC卡、二维码、ZDT或舵控协议。协议帧边界、校验和业务状态
 * 必须由对应Device/Core处理，避免公共UART基础设施与具体设备耦合。
 */

#ifndef UART_DISPATCH_H
#define UART_DISPATCH_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#include "stm32f7xx_hal.h"

/** 静态注册表容量；不在ISR中动态分配内存。 */
#define UART_DISPATCH_MAX_HANDLERS       6U

/** 用于表示“尚未成功注册”，与有效槽位0~5不重叠。 */
#define UART_DISPATCH_HANDLE_INVALID     0xFFU

/** 注册成功后返回的槽位索引，仅用于后续反注册。 */
typedef uint8_t uart_dispatch_handle_t;

/**
 * @brief DMA发送完成事件处理函数。
 * @return true表示句柄匹配且已认领事件，路由器停止继续分发；
 *         false表示不属于本模块，继续询问后续注册者。
 */
typedef bool (*uart_dispatch_tx_fn_t)(
    void *user_ctx,
    UART_HandleTypeDef *huart);

/**
 * @brief ReceiveToIdle接收事件处理函数。
 * @param rx_len HAL报告的本次接收数据长度；本层原样转交，不解析数据。
 * @return 语义与tx_complete相同：true表示已认领，false表示继续分发。
 */
typedef bool (*uart_dispatch_rx_fn_t)(
    void *user_ctx,
    UART_HandleTypeDef *huart,
    uint16_t rx_len);

/**
 * @brief UART错误事件处理函数。
 * @return 语义与tx_complete相同：true表示已认领，false表示继续分发。
 */
typedef bool (*uart_dispatch_error_fn_t)(
    void *user_ctx,
    UART_HandleTypeDef *huart);

/**
 * @brief 一个UART消费者的回调集合。
 *
 * 未使用的事件回调可以为NULL。user_ctx由注册者拥有，路由器
 * 不会复制或释放它，其生命周期必须覆盖整个注册期间。
 */
typedef struct {
    uart_dispatch_tx_fn_t tx_complete;
    uart_dispatch_rx_fn_t rx_event;
    uart_dispatch_error_fn_t error;
    void *user_ctx;
} uart_dispatch_handler_t;

/**
 * @brief 注册一个UART事件消费者。
 * @param handler 待复制到静态注册表的回调集合。
 * @param handle 成功时返回槽位句柄；失败时写入UART_DISPATCH_HANDLE_INVALID。
 * @return true表示注册成功；false表示参数无效或注册表已满。
 * @pre 应在对应UART DMA开始前完成注册；当前设计在osKernelStart前注册。
 * @note 至少提供一个handler；成功后handle用于初始化失败回滚或反注册。
 */
bool uart_dispatch_register(
    const uart_dispatch_handler_t *handler,
    uart_dispatch_handle_t *handle);

/**
 * @brief 反注册先前返回的handler槽位。
 * @param handle uart_dispatch_register()成功返回的句柄。
 * @return true表示槽位已撤销；false表示句柄越界或槽位未使用。
 * @pre 调用前必须确保该UART没有在途DMA/IRQ，不能与ISR并发修改注册表。
 */
bool uart_dispatch_unregister(uart_dispatch_handle_t handle);

#ifdef __cplusplus
}
#endif

#endif /* UART_DISPATCH_H */
