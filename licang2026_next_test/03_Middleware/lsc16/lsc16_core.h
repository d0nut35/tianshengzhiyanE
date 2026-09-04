/**
 * @file    lsc16_core.h
 * @brief   LSC系列16路舵机控制板的平台无关协议Core。
 *
 * 本层只理解“55 55 Length Cmd Parameters”协议、命令参数和接收状态机。
 * 它不知道STM32、UART8、DMA或FreeRTOS；具体收发由port注入，因此既可绑定
 * F7 HAL adapter，也可在PC上用fake port验证帧字节。
 */

#ifndef LSC16_CORE_H
#define LSC16_CORE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define LSC16_ACTION_GROUP_ALL          0xFFU
#define LSC16_REPEAT_FOREVER            0U
#define LSC16_SERVO_COUNT_MAX           16U
#define LSC16_SERVO_ID_MAX              15U
#define LSC16_SERVO_POSITION_MIN        500U
#define LSC16_SERVO_POSITION_MAX        2500U
#define LSC16_FRAME_SIZE_MAX            55U
#define LSC16_DMA_RX_BUFFER_SIZE        64U
#define LSC16_RX_RING_BUFFER_SIZE       128U

/** LSC16各层统一返回值，平台HAL错误在adapter内收敛为本枚举。 */
typedef enum {
    LSC16_OK = 0,
    LSC16_ERR_PARAM,
    LSC16_ERR_NOT_INIT,
    LSC16_ERR_STATE,
    LSC16_ERR_BUSY,
    LSC16_ERR_OVERFLOW,
    LSC16_ERR_IO,
    LSC16_ERR_QUEUE_FULL,
    LSC16_ERR_UNSUPPORTED,
} lsc16_status_t;

/** HAL回调到Core的轻量事件，只允许用于唤醒普通上下文。 */
typedef enum {
    LSC16_ISR_EVENT_TX_COMPLETE = 0,
    LSC16_ISR_EVENT_RX_READY,
    LSC16_ISR_EVENT_ERROR,
} lsc16_isr_event_t;

/** 控制板主动回传事件位，可在一次process中合并多个事件。 */
typedef enum {
    LSC16_REPORT_EVENT_NONE              = 0U,
    LSC16_REPORT_EVENT_ACTION_STARTED    = (1UL << 0),
    LSC16_REPORT_EVENT_ACTION_STOPPED    = (1UL << 1),
    LSC16_REPORT_EVENT_ACTION_COMPLETED  = (1UL << 2),
    LSC16_REPORT_EVENT_BATTERY_UPDATED   = (1UL << 3),
    LSC16_REPORT_EVENT_INVALID_FRAME     = (1UL << 4),
} lsc16_report_event_t;

/** 一个舵机的目标脉宽；id范围0~15，position范围500~2500。 */
typedef struct {
    uint8_t id;
    uint16_t position;
} lsc16_servo_target_t;

typedef struct {
    uint8_t command;
    uint8_t action_group;
    uint16_t repeat_count;
    uint16_t battery_mv;
} lsc16_report_t;

typedef lsc16_status_t (*lsc16_tx_start_fn_t)(
    void *ctx,
    const uint8_t *data,
    size_t len);

typedef lsc16_status_t (*lsc16_rx_start_fn_t)(
    void *ctx,
    uint8_t *data,
    size_t capacity);

typedef lsc16_status_t (*lsc16_abort_fn_t)(void *ctx);

typedef struct {
    lsc16_tx_start_fn_t tx_start;
    lsc16_rx_start_fn_t rx_start;
    lsc16_abort_fn_t abort;
    void *ctx;
} lsc16_port_t;

typedef void (*lsc16_isr_notify_fn_t)(
    void *user_ctx,
    lsc16_isr_event_t event);

typedef struct {
    bool initialized;
    volatile bool tx_busy;
    lsc16_port_t port;
    lsc16_isr_notify_fn_t notify_cb;
    void *notify_ctx;

    /* DMA在发送完成前持续读取tx_buffer，故缓冲区必须由Core长期持有。 */
    uint8_t tx_buffer[LSC16_FRAME_SIZE_MAX];
    uint8_t dma_rx_buffer[LSC16_DMA_RX_BUFFER_SIZE];

    /* ISR只推进head，任务/主循环只推进tail，形成单生产者单消费者队列。 */
    uint8_t rx_ring[LSC16_RX_RING_BUFFER_SIZE];
    volatile uint16_t rx_head;
    volatile uint16_t rx_tail;
    volatile uint32_t rx_overflow_count;
    volatile uint32_t uart_error_count;

    uint8_t rx_frame[LSC16_DMA_RX_BUFFER_SIZE];
    uint16_t rx_frame_count;
    uint32_t report_events;
    lsc16_report_t last_report;
} lsc16_t;

/**
 * @brief 初始化协议对象并立即启动一次异步接收。
 * @param device LSC16协议Core实例。
 * @param port 已绑定具体传输上下文的端口能力。
 * @return LSC16_OK表示成功，否则返回参数或底层I/O错误。
 */
lsc16_status_t lsc16_init(lsc16_t *device, const lsc16_port_t *port);

/**
 * @brief 终止底层收发并解除协议对象。
 * @param device LSC16协议Core实例。
 * @return LSC16_OK表示成功，否则返回参数、状态或I/O错误。
 */
lsc16_status_t lsc16_deinit(lsc16_t *device);

/**
 * @brief 绑定ISR轻量通知出口。
 * @param device LSC16协议Core实例。
 * @param notify_cb 事件通知函数，可为NULL以解除绑定。
 * @param user_ctx 原样传给notify_cb的上下文。
 * @return LSC16_OK表示成功，否则返回参数或未初始化错误。
 */
lsc16_status_t lsc16_bind_isr_notify(
    lsc16_t *device,
    lsc16_isr_notify_fn_t notify_cb,
    void *user_ctx);

/**
 * @brief 按0x03协议一次移动1至16个舵机。
 * @param device LSC16协议Core实例。
 * @param targets 舵机ID与目标脉宽数组。
 * @param servo_count 数组元素数，范围1至16。
 * @param move_time_ms 所有目标共用的动作时间。
 * @return LSC16_OK仅表示异步发送已启动，不表示舵机已经到位。
 */
lsc16_status_t lsc16_move_servos(
    lsc16_t *device,
    const lsc16_servo_target_t *targets,
    uint8_t servo_count,
    uint16_t move_time_ms);

/**
 * @brief 按0x06协议运行舵控板中已保存的动作组。
 * @param device LSC16协议Core实例。
 * @param action_group 动作组编号。
 * @param repeat_count 重复次数；0表示循环执行。
 * @return LSC16_OK仅表示异步发送已启动。
 */
lsc16_status_t lsc16_run_action_group(
    lsc16_t *device,
    uint8_t action_group,
    uint16_t repeat_count);

/**
 * @brief 发送0x07命令停止当前动作组。
 * @param device LSC16协议Core实例。
 * @return LSC16_OK仅表示异步发送已启动。
 * @note UART发送完成不等于舵控板已经执行停止或已经回报停止状态。
 */
lsc16_status_t lsc16_stop_action_group(lsc16_t *device);

/**
 * @brief 设置一个动作组的速度倍率。
 * @param device LSC16协议Core实例。
 * @param action_group 动作组编号。
 * @param speed_percent 速度百分比，合法范围由协议约束。
 * @return LSC16_OK仅表示异步发送已启动。
 */
lsc16_status_t lsc16_set_action_group_speed(
    lsc16_t *device,
    uint8_t action_group,
    uint16_t speed_percent);

/**
 * @brief 发送0x0F电池电压查询。
 * @param device LSC16协议Core实例。
 * @return LSC16_OK仅表示异步发送已启动。
 * @note 电压结果通过LSC16_REPORT_EVENT_BATTERY_UPDATED异步交付。
 */
lsc16_status_t lsc16_request_battery_voltage(lsc16_t *device);

/**
 * @brief 消费ISR缓存并解析完整回传帧。
 * @param device LSC16协议Core实例。
 * @note 必须在任务或裸机主循环中调用，禁止在ISR中解析协议。
 */
void lsc16_process(lsc16_t *device);

/**
 * @brief 终止异常传输并重新建立接收窗口。
 * @param device LSC16协议Core实例。
 * @return LSC16_OK表示恢复成功，否则返回参数、状态或I/O错误。
 */
lsc16_status_t lsc16_recover(lsc16_t *device);

/**
 * @brief 查询是否仍有一帧处于异步发送中。
 * @param device LSC16协议Core实例。
 * @return true表示发送尚未完成；对象无效时返回false。
 */
bool lsc16_is_tx_busy(const lsc16_t *device);

/**
 * @brief 原子取走并清除待处理主动回报事件位。
 * @param device LSC16协议Core实例。
 * @return 本次取得的lsc16_report_event_t位组合。
 */
uint32_t lsc16_take_report_events(lsc16_t *device);

/**
 * @brief 复制最近一次已解析的主动回报。
 * @param device LSC16协议Core实例。
 * @param report 输出回报快照。
 * @return LSC16_OK表示复制成功，否则返回参数或未初始化错误。
 */
lsc16_status_t lsc16_get_last_report(
    const lsc16_t *device,
    lsc16_report_t *report);

/**
 * @brief UART发送完成ISR入口。
 * @param device LSC16协议Core实例。
 * @warning 仅允许HAL adapter经公共uart_dispatch调用。
 */
void lsc16_on_tx_complete_isr(lsc16_t *device);

/**
 * @brief ReceiveToIdle事件ISR入口。
 * @param device LSC16协议Core实例。
 * @param rx_len 本次DMA缓冲区有效字节数。
 * @warning 仅搬运字节并通知worker，禁止在ISR内解析协议。
 */
void lsc16_on_rx_event_isr(lsc16_t *device, uint16_t rx_len);

/**
 * @brief UART错误ISR入口。
 * @param device LSC16协议Core实例。
 * @warning 仅记录错误并通知普通上下文完成恢复。
 */
void lsc16_on_error_isr(lsc16_t *device);

#ifdef __cplusplus
}
#endif

#endif /* LSC16_CORE_H */
