/**
 * @file    mux_service.h
 * @brief   UART7四通道复用器异步事务接口。
 */

#ifndef MUX_SERVICE_H
#define MUX_SERVICE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stddef.h>
#include <stdint.h>

#include "mux_bsp.h"

#define MUX_QUEUE_DEPTH       4U
#define MUX_TX_MAX            128U
#define MUX_RX_MAX            128U
#define MUX_DEFAULT_IO_MS     100U

typedef enum {
    MULT_UART_OP_SELECT = 0,
    MULT_UART_OP_WRITE,
    MULT_UART_OP_READ,
    MULT_UART_OP_WRITE_READ,
} mult_uart_operation_t;

/** 设备编号固定映射到复用通道0至3。 */
typedef enum {
    MUX_DEVICE_0 = 0,
    MUX_DEVICE_1,
    MUX_DEVICE_2,
    MUX_DEVICE_3,
} mux_device_t;

/** RX指针只在完成回调执行期间有效。 */
typedef struct {
    uint32_t request_id;
    mux_device_t device;
    mult_uart_status_t status;
    mult_uart_operation_t operation;
    const uint8_t *rx_data;
    size_t rx_len;
} mux_completion_t;

typedef void (*mux_done_fn_t)(
    void *user_ctx,
    const mux_completion_t *completion);

/** submit返回前复制TX数据；超时为0时使用100ms默认值。 */
typedef struct {
    mux_device_t device;
    mult_uart_operation_t operation;
    const uint8_t *tx_data;
    size_t tx_len;
    size_t rx_capacity;
    uint32_t io_timeout_ms;
    uint32_t queue_timeout_ms;
    mux_done_fn_t done_cb;
    void *user_ctx;
} mux_transfer_t;

/** 创建消息队列、worker、UART7 BSP和公共回调路由。 */
mult_uart_status_t mux_init(void);

/** 提交异步事务；同一设备在上一笔完成前返回BUSY。 */
mult_uart_status_t mux_submit(const mux_transfer_t *transfer);

#ifdef __cplusplus
}
#endif

#endif /* MUX_SERVICE_H */
