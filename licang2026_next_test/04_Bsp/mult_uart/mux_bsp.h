/**
 * @file    mux_bsp.h
 * @brief   四通道TTL UART复用器与STM32 UART7 HAL BSP。
 */

#ifndef MUX_BSP_H
#define MUX_BSP_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "stm32f7xx_hal.h"

#ifndef LICANG_RELEASE_MINIMAL
#define LICANG_RELEASE_MINIMAL 0
#endif

/* 正式任务上电后常驻运行，不保留仅供测试重复装配使用的反初始化入口。 */
#define MULT_UART_BSP_TEST_API_ENABLE (!LICANG_RELEASE_MINIMAL)

/* 硬件固定为 1 路公共 UART 分时连接 4 路 TTL UART。 */
#define MULT_UART_CHANNEL_COUNT       4U
#define MULT_UART_CHANNEL_INVALID     0xFFU

/**
 * @brief 模块统一状态码
 *
 * BSP和上层Service使用该状态类型，HAL错误在BSP内收敛。
 */
typedef enum {
    MULT_UART_OK = 0,
    MULT_UART_ERR_PARAM,
    MULT_UART_ERR_NOT_INIT,
    MULT_UART_ERR_STATE,
    MULT_UART_ERR_BUSY,
    MULT_UART_ERR_QUEUE_FULL,
    MULT_UART_ERR_TIMEOUT,
    MULT_UART_ERR_IO,
    MULT_UART_ERR_OVERFLOW,
    MULT_UART_ERR_UNSUPPORTED,
    MULT_UART_ERR_CANCELLED,
} mult_uart_status_t;

/**
 * @brief 复用器逻辑通道
 *
 * 硬件资料规定选择真值为 B:A：
 *   00 -> TX0/RX0
 *   01 -> TX1/RX1
 *   10 -> TX2/RX2
 *   11 -> TX3/RX3
 *
 * 因此枚举值的 bit0 对应 A，bit1 对应 B。
 */
typedef enum {
    MULT_UART_CHANNEL_0 = 0U,
    MULT_UART_CHANNEL_1 = 1U,
    MULT_UART_CHANNEL_2 = 2U,
    MULT_UART_CHANNEL_3 = 3U,
} mult_uart_channel_t;

/**
 * @brief 物理总线生命周期状态
 *
 * UART 为全双工外设，TX/RX 是否活动分别由 bus 中的 tx_active/rx_active
 * 表示；ACTIVE 可同时包含 TX 和 RX，不能再用互斥的 TX_BUSY/RX_BUSY 表示。
 */
typedef enum {
    MULT_UART_STATE_UNINIT = 0,
    MULT_UART_STATE_IDLE,
    MULT_UART_STATE_ACTIVE,
    MULT_UART_STATE_ABORTING,
    MULT_UART_STATE_ERROR,
} mult_uart_state_t;

/**
 * @brief 底层异步事件类型
 *
 * DMA完成和UART错误由公共uart_dispatch转交BSP，再通知Service。
 */
typedef enum {
    MULT_UART_EVENT_TX_COMPLETE = 0,
    MULT_UART_EVENT_RX_COMPLETE,
    MULT_UART_EVENT_ERROR,
} mult_uart_event_type_t;

/** @brief ISR 上抛给 Service 的通用事件。 */
typedef struct {
    mult_uart_event_type_t type;
    mult_uart_status_t status;
    size_t rx_len;             /* 仅 RX_COMPLETE 有效。 */
    uint32_t port_error;       /* 平台诊断值；0 表示未提供。 */
} mult_uart_event_t;

/**
 * @brief Core 事件回调
 *
 * @warning 该回调由BSP的HAL事件入口调用，因此运行于ISR上下文。
 *          后续 Service 只能在回调内执行 FromISR 通知，禁止阻塞和解析协议。
 * @note    event 指针只在回调执行期间有效。
 */
typedef void (*mult_uart_event_cb_t)(
    void *user_ctx,
    const mult_uart_event_t *event);

/**
 * @brief 固定UART7与四通道复用器的运行对象。
 *
 * 对象公开是为了静态分配，不代表上层可以直接修改成员。除 system
 * assembly 创建实例外，其他模块必须通过本头文件 API 访问。
 *
 * @warning bus 必须使用静态存储期（自动清零），或由调用者在首次 init 前
 *          显式清零。禁止把未初始化的栈对象直接传给 mult_uart_init()。
 */
typedef struct {
    volatile bool initialized;
    volatile bool enabled;
    volatile mult_uart_channel_t current_channel;
    volatile mult_uart_state_t state;
    volatile bool tx_active;
    volatile bool rx_active;
    volatile bool aborting;
    volatile size_t rx_capacity;
    mult_uart_event_cb_t event_cb;
    void *event_ctx;
#if !LICANG_RELEASE_MINIMAL
    volatile uint32_t uart_error_count;
    volatile uint32_t last_uart_error;
#endif
} mult_uart_bus_t;

/**
 * @brief 初始化物理复用总线对象
 *
 * @param bus    调用者静态分配的总线对象。
 * @retval MULT_UART_OK或参数/硬件资源错误。
 * @pre MX_GPIO_Init()、MX_DMA_Init()和MX_UART7_Init()已完成。
 */
mult_uart_status_t mult_uart_init(mult_uart_bus_t *bus);

/**
 * @brief 绑定 DMA/错误事件回调
 *
 * @note 允许在最小 SELECT 验证阶段传 NULL；开始 DMA 前必须绑定有效回调。
 *       仅允许在总线 IDLE 时修改绑定。
 */
mult_uart_status_t mult_uart_bind_event(
    mult_uart_bus_t *bus,
    mult_uart_event_cb_t event_cb,
    void *event_ctx);

/**
 * @brief 低电平使能复用模块；必须先成功选择有效通道。
 */
mult_uart_status_t mult_uart_enable(mult_uart_bus_t *bus);

/**
 * @brief 禁用复用模块
 *
 * @return 操作结果。
 */
mult_uart_status_t mult_uart_disable(mult_uart_bus_t *bus);

/**
 * @brief 选择逻辑通道
 *
 * @note 只允许在 IDLE 状态调用。Core 按通道值生成 A/B：
 *       A=(channel & 0x01)，B=(channel & 0x02)。
 *       当前已使能时固定执行“禁用->写A/B->等待5us->恢复使能”。
 */
mult_uart_status_t mult_uart_select(
    mult_uart_bus_t *bus,
    mult_uart_channel_t channel);

/**
 * @brief 查询当前逻辑通道。
 * @param bus 复用总线对象。
 * @param channel 输出当前通道；尚未成功选择时输出INVALID。
 * @return MULT_UART_OK表示读取成功，否则返回参数或未初始化错误。
 */
mult_uart_status_t mult_uart_get_channel(
    const mult_uart_bus_t *bus,
    mult_uart_channel_t *channel);

/**
 * @brief 在当前通道启动 DMA 发送
 *
 * @warning data 在收到 TX_COMPLETE/ERROR 事件或 abort 完成前必须保持有效。
 *          后续 Service 通过内部 job 持有并保证该生命周期。
 */
mult_uart_status_t mult_uart_start_tx_dma(
    mult_uart_bus_t *bus,
    const uint8_t *data,
    size_t len);

/**
 * @brief 在当前通道启动 DMA 接收
 *
 * @warning data 在收到 RX_COMPLETE/ERROR 事件或 abort 完成前必须保持有效。
 */
mult_uart_status_t mult_uart_start_rx_dma(
    mult_uart_bus_t *bus,
    uint8_t *data,
    size_t capacity);

/**
 * @brief 中止当前 TX/RX DMA 并使总线恢复到可重新调度的状态。
 * @note IDLE 时为幂等空操作并保留当前通道；实际中止或错误恢复成功后通道失效。
 */
mult_uart_status_t mult_uart_abort(mult_uart_bus_t *bus);

/**
 * @brief 平台 UART TX DMA 完成 ISR 入口
 * @warning 仅由具体平台 ISR adapter 调用。
 */
void mult_uart_on_tx_complete_isr(mult_uart_bus_t *bus);

/**
 * @brief 平台 UART RX/Idle DMA 完成 ISR 入口
 * @param rx_len 本次 DMA 缓冲区中的有效字节数。
 * @warning 仅由具体平台 ISR adapter 调用。
 */
void mult_uart_on_rx_complete_isr(
    mult_uart_bus_t *bus,
    size_t rx_len);

/**
 * @brief 平台 UART 错误 ISR 入口
 * @param port_error 平台原始诊断值或错误位掩码；0 表示未提供。
 * @warning 仅由具体平台 ISR adapter 调用。
 */
void mult_uart_on_error_isr(
    mult_uart_bus_t *bus,
    uint32_t port_error);

/** 公共uart_dispatch入口；只认领固定UART7。 */
bool mult_uart_handle_tx(
    mult_uart_bus_t *bus,
    UART_HandleTypeDef *huart);
bool mult_uart_handle_rx(
    mult_uart_bus_t *bus,
    UART_HandleTypeDef *huart,
    uint16_t rx_len);
bool mult_uart_handle_error(
    mult_uart_bus_t *bus,
    UART_HandleTypeDef *huart);

/**
 * @brief 反初始化复用总线对象。
 * @param bus 复用总线对象。
 * @return MULT_UART_OK表示成功；总线忙时返回状态错误。
 * @note 调用前应先由Service停止调度并完成abort。
 */
#if MULT_UART_BSP_TEST_API_ENABLE
mult_uart_status_t mult_uart_deinit(mult_uart_bus_t *bus);
#endif

#ifdef __cplusplus
}
#endif

#endif /* MUX_BSP_H */
