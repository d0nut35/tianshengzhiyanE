/**
 * @file    mult_uart_core.h
 * @brief   四通道 TTL UART 复用总线核心接口（平台与 OS 无关）
 * @note    本层只描述复用器、DMA 端口能力和 ISR 事件，不包含 HAL/RTOS 类型。
 */

#ifndef MULT_UART_CORE_H
#define MULT_UART_CORE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* 硬件固定为 1 路公共 UART 分时连接 4 路 TTL UART。 */
#define MULT_UART_CHANNEL_COUNT       4U
#define MULT_UART_CHANNEL_INVALID     0xFFU

/**
 * @brief 模块统一状态码
 *
 * Core、STM32 HAL port 和后续 Service 均使用该状态类型；具体平台错误
 * 由适配层映射，不能把具体平台或 OS 的状态类型直接抛给上层。
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
 * DMA 完成和 UART 错误由具体平台 ISR adapter 转调对应的
 * mult_uart_on_xxx_isr()，再由 Core 以本枚举通知 Service。
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
 * @warning 该回调由 mult_uart_on_xxx_isr() 调用，因此运行于 ISR 上下文。
 *          后续 Service 只能在回调内执行 FromISR 通知，禁止阻塞和解析协议。
 * @note    event 指针只在回调执行期间有效。
 */
typedef void (*mult_uart_event_cb_t)(
    void *user_ctx,
    const mult_uart_event_t *event);

/**
 * @brief 平台端口操作
 *
 * 谁需要，谁定义接口：本 Core 声明自己需要 GPIO、DMA 和短延时能力；
 * 具体平台 adapter 负责实现，system assembly 负责把实现注入 Core。
 */
typedef struct {
    /** 设置 A/B 电平；平台实现应尽量缩短两个 GPIO 写操作的间隔。 */
    mult_uart_status_t (*write_select)(
        void *ctx,
        bool a_high,
        bool b_high);

    /** 写 EN 实际引脚电平；level_high=true 表示输出高电平。 */
    mult_uart_status_t (*write_enable)(
        void *ctx,
        bool level_high);

    /** 启动 UART DMA 发送；成功仅表示 DMA 已启动。 */
    mult_uart_status_t (*start_tx_dma)(
        void *ctx,
        const uint8_t *data,
        size_t len);

    /** 启动 UART DMA 接收；实际长度由 RX_COMPLETE 事件返回。 */
    mult_uart_status_t (*start_rx_dma)(
        void *ctx,
        uint8_t *data,
        size_t capacity);

    /**
     * 中止当前 UART TX/RX DMA，用于超时、停止和错误恢复。
     * 返回前必须同步停止 DMA，并清除 UART/DMA 标志及待处理 IRQ，确保旧完成
     * 中断不会在下一次 start 后到达。
     */
    mult_uart_status_t (*abort_dma)(void *ctx);

    /** 复用器切换后的短延时；settle_us 为 0 时不会调用。 */
    void (*delay_us)(void *ctx, uint32_t delay_us);
} mult_uart_port_ops_t;

/** @brief 已绑定 context 的平台端口实例。 */
typedef struct {
    const mult_uart_port_ops_t *ops;
    void *ctx;
} mult_uart_port_t;

/**
 * @brief 复用总线配置
 *
 * 模块 EN 为低电平使能且板载下拉。若 EN 悬空或直接接地，设置
 * manage_enable=false；若 EN 接 MCU GPIO，建议设置为 true，并在切换通道
 * 时使用 break_before_switch，避免 A/B 分步变化时短暂连到中间通道。
 */
typedef struct {
    bool manage_enable;
    bool enable_active_low;
    bool break_before_switch;
    uint32_t switch_settle_us;
} mult_uart_config_t;

/**
 * @brief 一个对象对应“一个物理 UART + 一块四通道复用模块”
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
    mult_uart_config_t config;
    mult_uart_port_t port;
    mult_uart_event_cb_t event_cb;
    void *event_ctx;
} mult_uart_bus_t;

/**
 * @brief 初始化物理复用总线对象
 *
 * @param bus    调用者静态分配的总线对象。
 * @param config 模块配置，初始化期间复制。
 * @param port   已绑定具体硬件 context 的端口，初始化期间复制。
 * @retval MULT_UART_OK 或参数/端口错误。
 *
 * @note manage_enable=true 时，初始化应先把 EN 置为禁用电平；
 *       manage_enable=false 时，Core 假设 EN 已由硬件下拉或接地使能。
 */
mult_uart_status_t mult_uart_init(
    mult_uart_bus_t *bus,
    const mult_uart_config_t *config,
    const mult_uart_port_t *port);

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
 * @brief 使能复用模块；EN 低有效由配置转换为实际引脚电平。
 * @note manage_enable=true 时，必须先成功选择有效通道。
 */
mult_uart_status_t mult_uart_enable(mult_uart_bus_t *bus);

/**
 * @brief 禁用复用模块
 *
 * @retval manage_enable=false 时返回 MULT_UART_ERR_UNSUPPORTED。
 */
mult_uart_status_t mult_uart_disable(mult_uart_bus_t *bus);

/**
 * @brief 选择逻辑通道
 *
 * @note 只允许在 IDLE 状态调用。Core 按通道值生成 A/B：
 *       A=(channel & 0x01)，B=(channel & 0x02)。
 *       break_before_switch=true 且当前已使能时，执行“禁用->写 A/B
 *       ->稳定等待->恢复使能”。
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

/**
 * @brief 反初始化复用总线对象。
 * @param bus 复用总线对象。
 * @return MULT_UART_OK表示成功；总线忙时返回状态错误。
 * @note 调用前应先由Service停止调度并完成abort。
 */
mult_uart_status_t mult_uart_deinit(mult_uart_bus_t *bus);

#ifdef __cplusplus
}
#endif

#endif /* MULT_UART_CORE_H */
