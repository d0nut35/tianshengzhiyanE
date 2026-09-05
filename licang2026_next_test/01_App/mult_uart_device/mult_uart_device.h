/**
 * @file    mult_uart_device.h
 * @brief   mult_uart 上层设备事务框架。
 *
 * 这一层是 P5 的第一版骨架：它不描述具体设备协议内容，而是先建立
 * “设备 ID -> 复用通道 -> Service OS 请求 -> 完成回调”的统一入口。
 * 后续真实设备模块应基于本层提交命令和解析回复，而不是直接访问 HAL、
 * DMA、GPIO 或底层 channel 切换细节。
 */

#ifndef MULT_UART_DEVICE_H
#define MULT_UART_DEVICE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "mux_service.h"

#ifndef LICANG_RELEASE_MINIMAL
#define LICANG_RELEASE_MINIMAL 0
#endif

/* 正式任务只保留事务接口；统计和重复初始化测试入口不进入镜像。 */
#define MULT_UART_DEVICE_DIAGNOSTICS_ENABLE (!LICANG_RELEASE_MINIMAL)
#define MULT_UART_DEVICE_TEST_API_ENABLE    (!LICANG_RELEASE_MINIMAL)

#define MULT_UART_DEVICE_COUNT               4U
#define MULT_UART_DEVICE_QUEUE_TIMEOUT_MS    0U
#define MULT_UART_DEVICE_DEFAULT_IO_MS       100U

typedef enum {
    MULT_UART_DEVICE_0 = 0,
    MULT_UART_DEVICE_1,
    MULT_UART_DEVICE_2,
    MULT_UART_DEVICE_3,
} mult_uart_device_id_t;

/**
 * @brief 单个外部设备的静态配置。
 *
 * channel 是该设备接在复用器上的物理通道；default_io_timeout_ms 是应用
 * 没有给单笔请求指定超时时采用的默认 I/O 超时。
 */
typedef struct {
    bool enabled;
    mult_uart_channel_t channel;
    uint32_t default_io_timeout_ms;
} mult_uart_device_config_t;

/**
 * @brief 设备事务完成信息。
 *
 * rx_data 的生命周期继承 Service 契约：只在完成回调执行期间有效。真实设备
 * 协议如果要缓存回复，应在回调里复制或立即解析。
 */
typedef struct {
    uint32_t request_id;
    mult_uart_device_id_t device_id;
    mult_uart_status_t status;
    mult_uart_operation_t operation;
    const uint8_t *rx_data;
    size_t rx_len;
} mult_uart_device_completion_t;

typedef void (*mult_uart_device_done_fn_t)(
    void *user_ctx,
    const mult_uart_device_completion_t *completion);

/**
 * @brief 上层提交的一笔设备事务。
 *
 * device_id 表示业务设备；本层根据配置表转换到真实 mult_uart_channel_t。
 * io_timeout_ms 为 0 时使用该设备默认 I/O 超时；queue_timeout_ms 为 0 时
 * 表示尝试立即放入 OS queue。
 */
typedef struct {
    mult_uart_device_id_t device_id;
    mult_uart_operation_t operation;
    const uint8_t *tx_data;
    size_t tx_len;
    size_t rx_capacity;
    uint32_t io_timeout_ms;
    uint32_t queue_timeout_ms;
    mult_uart_device_done_fn_t done_cb;
    void *user_ctx;
} mult_uart_device_transfer_t;

#if MULT_UART_DEVICE_DIAGNOSTICS_ENABLE
typedef struct {
    uint32_t submitted;
    uint32_t completed;
    uint32_t invalid_request;
    uint32_t busy;
    uint32_t submit_error;
} mult_uart_device_stats_t;
#endif

/**
 * @brief 初始化设备事务层。
 *
 * configs 为 NULL 时使用默认映射：device0~3 分别对应 channel0~3，默认
 * I/O 超时为 MULT_UART_DEVICE_DEFAULT_IO_MS。
 * @param configs 可选设备映射配置表；传NULL使用默认一一映射。
 * @param config_count configs元素数量；使用默认映射时必须为0。
 * @return 初始化成功返回MULT_UART_OK，否则返回参数或状态错误。
 */
mult_uart_status_t mult_uart_device_init(
    const mult_uart_device_config_t *configs,
    size_t config_count);

/**
 * @brief 清空设备事务层状态，回到未初始化。
 *
 * 若仍有 pending 事务，本函数返回 BUSY，避免把已经提交到底层 Service 的
 * 完成回调上下文提前释放。
 * @return 成功返回MULT_UART_OK；存在未完成事务时返回MULT_UART_ERR_BUSY。
 */
#if MULT_UART_DEVICE_TEST_API_ENABLE
mult_uart_status_t mult_uart_device_deinit(void);
#endif

/**
 * @brief 提交一笔设备事务。
 *
 * 第一版策略是“同一设备同一时间只允许一笔未完成事务”，避免真实协议层
 * 在还没收到上一条回复时又发下一条，导致回复归属混乱。
 * @param transfer 待提交事务及完成回调信息。
 * @return 请求成功入队返回MULT_UART_OK，否则返回参数、忙或队列错误。
 * @warning transfer中的TX数据会在下层入队时复制；完成回调中的RX指针仅在
 *          回调执行期间有效。
 */
mult_uart_status_t mult_uart_device_submit(
    const mult_uart_device_transfer_t *transfer);

/**
 * @brief 读取设备事务层统计快照。
 * @param stats 接收统计快照的输出对象。
 * @return 成功返回MULT_UART_OK，否则返回参数或未初始化错误。
 */
#if MULT_UART_DEVICE_DIAGNOSTICS_ENABLE
mult_uart_status_t mult_uart_device_get_stats(
    mult_uart_device_stats_t *stats);
#endif

#ifdef __cplusplus
}
#endif

#endif /* MULT_UART_DEVICE_H */
