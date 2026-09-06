/**
 * @file    ic_bsp.h
 * @brief   M2940B-HA读卡器协议与STM32 HAL板级接口。
 *
 * 正式Mission只使用协议构帧、验帧和数据提取，并通过mux_submit()访问
 * UART7通道1。直连UART7接口只供独立测试，正式最小构建不会编译。
 */

#ifndef IC_BSP_H
#define IC_BSP_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifndef LICANG_RELEASE_MINIMAL
#define LICANG_RELEASE_MINIMAL 0
#endif

#if !LICANG_RELEASE_MINIMAL
#include "stm32f7xx_hal.h"
#endif

#define IC_CARD_BLOCK_DATA_SIZE              16U
#define IC_CARD_COMMAND_FRAME_SIZE            8U
#define IC_CARD_FRAME_SIZE_MAX               32U
#define IC_CARD_DMA_RX_BUFFER_SIZE            32U
#define IC_CARD_RX_RING_BUFFER_SIZE           128U

/** Core、Service和平台适配层共用的错误类型。 */
typedef enum {
    IC_CARD_OK = 0,
    IC_CARD_ERR_PARAM,
    IC_CARD_ERR_NOT_INIT,
    IC_CARD_ERR_STATE,
    IC_CARD_ERR_BUSY,
    IC_CARD_ERR_OVERFLOW,
    IC_CARD_ERR_IO,
    IC_CARD_ERR_TIMEOUT,
    IC_CARD_ERR_QUEUE_FULL,
    IC_CARD_ERR_PROTOCOL,
    IC_CARD_ERR_CARD,
    IC_CARD_ERR_UNSUPPORTED,
} ic_card_status_t;

/** 厂家协议命令类型。 */
typedef enum {
    IC_CARD_PACKET_CARD_OPERATION = 0x01U,
    IC_CARD_PACKET_QUERY          = 0x02U,
    IC_CARD_PACKET_SETTING        = 0x03U,
    IC_CARD_PACKET_OTHER          = 0x04U,
} ic_card_packet_type_t;

/** 当前项目使用的厂家命令号。 */
typedef enum {
    IC_CARD_CMD_READ_UID          = 0xA1U,
    IC_CARD_CMD_READ_BLOCK_KEY_A  = 0xA3U,
    IC_CARD_CMD_QUERY_ADDRESS     = 0xB0U,
    IC_CARD_CMD_QUERY_WORK_MODE   = 0xB1U,
    IC_CARD_CMD_QUERY_BEEPER      = 0xB2U,
    IC_CARD_CMD_QUERY_AUTO_READ   = 0xB8U,
} ic_card_command_t;

/** ISR只发布事件，不在中断内解析协议。 */
typedef enum {
    IC_CARD_ISR_EVENT_TX_COMPLETE = 0,
    IC_CARD_ISR_EVENT_RX_READY,
    IC_CARD_ISR_EVENT_ERROR,
} ic_card_isr_event_t;

/**
 * @brief 已通过长度和校验检查的读卡器响应。
 * @note payload不包含前5字节和末尾校验；对象按值保存，不引用DMA缓冲区。
 */
typedef struct {
    uint8_t packet_type;
    uint8_t command;
    uint8_t address;
    uint8_t device_status;
    uint8_t payload[IC_CARD_FRAME_SIZE_MAX - 6U];
    uint8_t payload_len;
    uint8_t raw[IC_CARD_FRAME_SIZE_MAX];
    uint8_t raw_len;
} ic_card_response_t;

typedef ic_card_status_t (*ic_card_tx_start_fn_t)(
    void *ctx,
    const uint8_t *data,
    size_t len);

typedef ic_card_status_t (*ic_card_rx_start_fn_t)(
    void *ctx,
    uint8_t *data,
    size_t capacity);

typedef ic_card_status_t (*ic_card_abort_fn_t)(void *ctx);

/** 由上层注入的异步串口能力。 */
typedef struct {
    ic_card_tx_start_fn_t tx_start;
    ic_card_rx_start_fn_t rx_start;
    ic_card_abort_fn_t abort;
    void *ctx;
} ic_card_port_t;

typedef void (*ic_card_isr_notify_fn_t)(
    void *user_ctx,
    ic_card_isr_event_t event);

/**
 * @brief 一个读卡器协议实例。
 *
 * tx_buffer由Core长期持有，保证DMA发送期间数据不失效；RX采用ISR写head、
 * 任务写tail的单生产者单消费者环形缓冲，协议解析始终在普通上下文执行。
 */
typedef struct {
    bool initialized;
    volatile bool tx_busy;
    ic_card_port_t port;
    ic_card_isr_notify_fn_t notify_cb;
    void *notify_ctx;
    uint8_t tx_buffer[IC_CARD_FRAME_SIZE_MAX];
    uint8_t dma_rx_buffer[IC_CARD_DMA_RX_BUFFER_SIZE];
    uint8_t rx_ring[IC_CARD_RX_RING_BUFFER_SIZE];
    volatile uint16_t rx_head;
    volatile uint16_t rx_tail;
    volatile uint32_t rx_overflow_count;
    volatile uint32_t uart_error_count;
    uint8_t parse_buffer[IC_CARD_FRAME_SIZE_MAX];
    uint8_t parse_count;
    uint8_t expected_len;
    uint32_t response_sequence;
    uint32_t invalid_frame_count;
    ic_card_response_t last_response;
} ic_card_t;

/**
 * @brief 计算厂家规定的“逐字节异或后按位取反”校验值。
 * @param data 待计算字节序列。
 * @param len_without_checksum 不含校验字节的数据长度。
 * @return 计算得到的1字节校验值。
 */
uint8_t ic_checksum(const uint8_t *data, size_t len_without_checksum);

/**
 * @brief 构造使用内部Key A读取数据块的8字节A3命令。
 * @param address 读卡器地址。
 * @param block Mifare数据块号。
 * @param led_beep_prompt 是否请求成功时蜂鸣/亮灯。
 * @param frame 接收完整命令帧的缓冲区。
 * @param capacity frame容量，至少为IC_CARD_COMMAND_FRAME_SIZE。
 * @param frame_len 返回有效帧长。
 * @return 构帧结果。
 * @note 本函数不访问UART，适用于直连Core和串口复用事务共同调用。
 */
ic_card_status_t ic_read_frame(
    uint8_t address,
    uint8_t block,
    bool led_beep_prompt,
    uint8_t *frame,
    size_t capacity,
    size_t *frame_len);

/**
 * @brief 构造B0/B1/B2/B8查询命令。
 * @param command 支持的查询命令。
 * @param address 当前读卡器地址；B0会自动使用协议规定的地址0。
 * @param frame 接收完整命令帧的缓冲区。
 * @param capacity frame容量。
 * @param frame_len 返回有效帧长。
 * @return 构帧结果。
 */
ic_card_status_t ic_query_frame(
    ic_card_command_t command,
    uint8_t address,
    uint8_t *frame,
    size_t capacity,
    size_t *frame_len);

/**
 * @brief 校验并解析一帧已经完整收齐的读卡器响应。
 * @param frame 完整响应帧。
 * @param frame_len 响应字节数。
 * @param response 接收按值解析结果。
 * @return 长度、包类型和校验均合法时返回IC_CARD_OK。
 * @note response不保留frame指针，调用者可在返回后立即复用RX缓冲区。
 */
ic_card_status_t ic_parse_frame(
    const uint8_t *frame,
    size_t frame_len,
    ic_card_response_t *response);

/**
 * @brief 初始化协议对象并立即启动一次异步接收。
 * @param device 协议Core实例。
 * @param port 已绑定具体传输上下文的端口能力。
 * @return IC_CARD_OK表示成功，否则返回参数或底层I/O错误。
 */
ic_card_status_t ic_bsp_init(ic_card_t *device, const ic_card_port_t *port);

/**
 * @brief 终止底层收发并清空协议对象。
 * @param device 协议Core实例。
 * @return IC_CARD_OK表示成功，否则返回参数、状态或I/O错误。
 */
ic_card_status_t ic_bsp_deinit(ic_card_t *device);

/**
 * @brief 绑定ISR轻量通知出口。
 * @param device 协议Core实例。
 * @param notify_cb 事件通知函数，可为NULL以解除绑定。
 * @param user_ctx 原样传给notify_cb的上下文。
 * @return IC_CARD_OK表示成功，否则返回参数或未初始化错误。
 */
ic_card_status_t ic_bsp_bind_notify(
    ic_card_t *device,
    ic_card_isr_notify_fn_t notify_cb,
    void *user_ctx);

/**
 * @brief 发送A3命令，使用读卡器内部Key A读取一个16字节数据块。
 * @param device 协议Core实例。
 * @param address 读卡器设备地址。
 * @param block Mifare数据块号。
 * @param led_beep_prompt 是否请求读卡器使用灯光/蜂鸣提示。
 * @return IC_CARD_OK仅表示异步发送已启动，不表示读卡成功。
 */
ic_card_status_t ic_bsp_read(
    ic_card_t *device,
    uint8_t address,
    uint8_t block,
    bool led_beep_prompt);

/**
 * @brief 发送B0/B1/B2/B8等固定8字节查询命令。
 * @param device 协议Core实例。
 * @param command 支持的查询命令。
 * @param address 读卡器设备地址。
 * @return IC_CARD_OK仅表示异步发送已启动。
 */
ic_card_status_t ic_bsp_query(
    ic_card_t *device,
    ic_card_command_t command,
    uint8_t address);

/**
 * @brief 消费ISR缓存并解析完整响应。
 * @param device 协议Core实例。
 * @note 必须在任务或裸机主循环中调用，禁止在ISR内解析协议。
 */
void ic_bsp_process(ic_card_t *device);

/**
 * @brief 错误或超时后同步中止传输并重新建立接收窗口。
 * @param device 协议Core实例。
 * @return IC_CARD_OK表示恢复成功，否则返回参数、状态或I/O错误。
 */
ic_card_status_t ic_bsp_recover(ic_card_t *device);

/**
 * @brief 查询是否仍有一帧处于异步发送中。
 * @param device 协议Core实例。
 * @return true表示发送尚未完成；对象无效时返回false。
 */
bool ic_bsp_tx_busy(const ic_card_t *device);

/**
 * @brief 获取最近响应。
 * @param last_sequence 调用者保存的序号；有新响应时更新它并复制response。
 * @return true表示本次取得新响应，false表示没有更新。
 */
bool ic_bsp_take_response(
    const ic_card_t *device,
    uint32_t *last_sequence,
    ic_card_response_t *response);

/**
 * @brief 检查A3成功响应并复制16字节数据。
 * @param response 已通过基础帧校验的响应。
 * @param expected_address 期望的读卡器设备地址。
 * @param data 输出16字节块数据。
 * @return IC_CARD_OK表示响应类型、状态和长度均正确。
 */
ic_card_status_t ic_block_data(
    const ic_card_response_t *response,
    uint8_t expected_address,
    uint8_t data[IC_CARD_BLOCK_DATA_SIZE]);

/**
 * @brief UART发送完成ISR入口。
 * @param device 协议Core实例。
 * @warning 仅允许HAL adapter经公共uart_dispatch调用。
 */
void ic_bsp_tx_isr(ic_card_t *device);

/**
 * @brief ReceiveToIdle事件ISR入口。
 * @param device 协议Core实例。
 * @param rx_len 本次DMA缓冲区有效字节数。
 * @warning 仅搬运字节并通知worker，禁止在ISR内解析协议。
 */
void ic_bsp_rx_isr(ic_card_t *device, uint16_t rx_len);

/**
 * @brief UART错误ISR入口。
 * @param device 协议Core实例。
 * @warning 仅记录错误并通知普通上下文完成恢复。
 */
void ic_bsp_error_isr(ic_card_t *device);

#if !LICANG_RELEASE_MINIMAL
/** 直连UART7仅用于独立测试；正式Mission由mux_service独占UART7。 */
typedef struct {
    UART_HandleTypeDef *uart;
} ic_card_uart7_hal_config_t;

typedef struct {
    ic_card_t *device;
    UART_HandleTypeDef *uart;
} ic_card_uart7_hal_t;

void ic_uart7_config(ic_card_uart7_hal_config_t *config);
ic_card_status_t ic_uart7_bind(
    ic_card_uart7_hal_t *adapter,
    ic_card_t *device,
    const ic_card_uart7_hal_config_t *config,
    ic_card_port_t *port);
bool ic_uart7_tx_isr(
    ic_card_uart7_hal_t *adapter,
    UART_HandleTypeDef *huart);
bool ic_uart7_rx_isr(
    ic_card_uart7_hal_t *adapter,
    UART_HandleTypeDef *huart,
    uint16_t rx_len);
bool ic_uart7_error_isr(
    ic_card_uart7_hal_t *adapter,
    UART_HandleTypeDef *huart);
#endif

#ifdef __cplusplus
}
#endif

#endif /* IC_BSP_H */
