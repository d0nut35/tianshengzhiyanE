/**
 * @file    uart_dispatch.c
 * @brief   STM32 HAL UART weak callback公共路由实现。
 *
 * 路由器只运行在HAL回调上下文中，因此不执行协议解析、阻塞等待、
 * 超时重试或业务状态转移。它只依次询问注册者，并在某个handler
 * 返回true后立即停止，以保证同一UART事件只被一个模块认领。
 */

#include "uart_dispatch.h"

#include <string.h>

typedef struct {
    /* false表示槽位空闲；true时handler的内容才可访问。 */
    bool in_use;

    /* 按值保存函数表；user_ctx仍是注册者拥有的指针。 */
    uart_dispatch_handler_t handler;
} uart_dispatch_slot_t;

/*
 * 固定长度静态表，避免回调路径依赖malloc/FreeRTOS heap。
 * C语言静态存储期对象会自动清零，所以上电时所有in_use默认为false。
 */
static uart_dispatch_slot_t g_uart_dispatch_slots[UART_DISPATCH_MAX_HANDLERS];

/**
 * @brief 校验路由handler至少实现一种UART事件。
 * @param handler 待检查handler。
 * @return handler有效时返回true。
 */
static bool uart_dispatch_handler_is_valid(
    const uart_dispatch_handler_t *handler)
{
    return (handler != NULL) &&
           ((handler->tx_complete != NULL) ||
            (handler->rx_event != NULL) ||
            (handler->error != NULL));
}

/**
 * @brief 把一个模块handler注册到固定路由表。
 * @param handler 待注册回调集合。
 * @param handle 接收注册句柄的输出对象。
 * @return 注册成功返回true，参数无效或表满返回false。
 * @pre 应在对应UART开始DMA前调用，避免与ISR并发修改路由表。
 */
bool uart_dispatch_register(
    const uart_dispatch_handler_t *handler,
    uart_dispatch_handle_t *handle)
{
    uint32_t i;

    /* 先拒绝无法工作的handler，同时要求调用者接收句柄。 */
    if (!uart_dispatch_handler_is_valid(handler) || (handle == NULL)) {
        return false;
    }

    /* 预先写入失败值，使调用者在任何失败路径上都不会误用旧句柄。 */
    *handle = UART_DISPATCH_HANDLE_INVALID;

    /* 首次适配：找到第一个空闲槽位即完成注册。 */
    for (i = 0U; i < UART_DISPATCH_MAX_HANDLERS; ++i) {
        if (!g_uart_dispatch_slots[i].in_use) {
            /*
             * 注册发生在osKernelStart和UART DMA启动前。先复制完整handler，
             * 最后发布in_use，避免ISR看到半初始化的函数表。
             */
            g_uart_dispatch_slots[i].handler = *handler;
            g_uart_dispatch_slots[i].in_use = true;
            *handle = (uart_dispatch_handle_t)i;
            return true;
        }
    }

    /* 遍历后仍未返回，说明固定注册表已满。 */
    return false;
}

/**
 * @brief 从固定路由表撤销一个模块handler。
 * @param handle 注册时获得的句柄。
 * @return 槽位存在并成功清除时返回true。
 * @pre 必须先停止该模块UART/DMA并确保不会再进入其ISR回调。
 */
bool uart_dispatch_unregister(uart_dispatch_handle_t handle)
{
    uart_dispatch_slot_t *slot;

    /* 先校验范围，避免越界访问全局注册表。 */
    if ((uint32_t)handle >= UART_DISPATCH_MAX_HANDLERS) {
        return false;
    }

    slot = &g_uart_dispatch_slots[handle];
    if (!slot->in_use) {
        return false;
    }

    /* 调用者保证无并发ISR；先撤销可见性，再清空旧函数指针。 */
    slot->in_use = false;
    (void)memset(&slot->handler, 0, sizeof(slot->handler));
    return true;
}

/**
 * @brief 全工程唯一HAL UART发送完成回调入口。
 * @param huart 产生事件的UART句柄。
 * @note 按注册顺序查询handler，第一个返回true的消费者认领事件。
 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    uint32_t i;

    /*
     * HAL在DMA发送完成后进入此全局回调。路由器依次询问已注册模块：
     * 1. 跳过空槽位和不关心TX事件的模块；
     * 2. 调用handler，由handler比较huart是否为自己绑定的UART；
     * 3. handler返回true表示已处理，立即return，防止重复通知。
     *
     * 此路径仍处在中断/HAL回调上下文，handler必须足够轻量。
     */
    for (i = 0U; i < UART_DISPATCH_MAX_HANDLERS; ++i) {
        if (g_uart_dispatch_slots[i].in_use &&
            (g_uart_dispatch_slots[i].handler.tx_complete != NULL) &&
            g_uart_dispatch_slots[i].handler.tx_complete(
                g_uart_dispatch_slots[i].handler.user_ctx,
                huart)) {
            return;
        }
    }
}

/**
 * @brief 全工程唯一HAL ReceiveToIdle回调入口。
 * @param huart 产生事件的UART句柄。
 * @param Size 本次DMA接收有效字节数。
 */
void HAL_UARTEx_RxEventCallback(
    UART_HandleTypeDef *huart,
    uint16_t size)
{
    uint32_t i;

    /*
     * ReceiveToIdle DMA产生接收事件后进入这里。size是HAL提供的接收长度，
     * 路由器不读取DMA缓冲区、不判断帧头、不做校验，只将huart和size
     * 转交给认领该UART的适配器。适配器负责搬运/发布数据，协议Core
     * 在非ISR上下文中再完成真正解析。
     */
    for (i = 0U; i < UART_DISPATCH_MAX_HANDLERS; ++i) {
        if (g_uart_dispatch_slots[i].in_use &&
            (g_uart_dispatch_slots[i].handler.rx_event != NULL) &&
            g_uart_dispatch_slots[i].handler.rx_event(
                g_uart_dispatch_slots[i].handler.user_ctx,
                huart,
                size)) {
            return;
        }
    }
}

/**
 * @brief 全工程唯一HAL UART错误回调入口。
 * @param huart 产生错误的UART句柄。
 * @warning handler仅记录错误并通知任务，不能在ISR内执行阻塞恢复。
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    uint32_t i;

    /*
     * 奇偶校验、帧错误、噪声或溢出等UART错误统一从此分发。
     * 路由层不擅自清状态或重启DMA，因为只有所属适配器知道自己的
     * RX缓冲区、状态机和恢复顺序。返回true后停止继续分发。
     */
    for (i = 0U; i < UART_DISPATCH_MAX_HANDLERS; ++i) {
        if (g_uart_dispatch_slots[i].in_use &&
            (g_uart_dispatch_slots[i].handler.error != NULL) &&
            g_uart_dispatch_slots[i].handler.error(
                g_uart_dispatch_slots[i].handler.user_ctx,
                huart)) {
            return;
        }
    }
}
