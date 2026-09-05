/**
 * @file    mux_service.h
 * @brief   UART7四通道复用器的CMSIS-RTOS2事务Service。
 */

#ifndef MUX_SERVICE_H
#define MUX_SERVICE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stddef.h>
#include <stdint.h>

#include "mux_bsp.h"

#define MULT_UART_SERVICE_QUEUE_DEPTH    4U
#define MULT_UART_SERVICE_TX_MAX         128U
#define MULT_UART_SERVICE_RX_MAX         128U

typedef enum {
    MULT_UART_OP_SELECT = 0,
    MULT_UART_OP_WRITE,
    MULT_UART_OP_READ,
    MULT_UART_OP_WRITE_READ,
} mult_uart_operation_t;

/** 完成信息中的RX指针只在回调执行期间有效。 */
typedef struct {
    uint32_t request_id;
    mult_uart_status_t status;
    mult_uart_operation_t operation;
    mult_uart_channel_t channel;
    const uint8_t *rx_data;
    size_t rx_len;
} mult_uart_completion_t;

typedef void (*mult_uart_done_fn_t)(
    void *user_ctx,
    const mult_uart_completion_t *completion);

/** Service会在submit返回前复制TX数据。 */
typedef struct {
    uint32_t request_id;
    mult_uart_operation_t operation;
    mult_uart_channel_t channel;
    const uint8_t *tx_data;
    size_t tx_len;
    size_t rx_capacity;
    uint32_t io_timeout_ms;
    mult_uart_done_fn_t done_cb;
    void *user_ctx;
} mult_uart_request_t;

/** 创建消息队列、worker和UART7事件路由。 */
mult_uart_status_t mux_service_init(void);

/** 提交异步事务；queue_timeout_ms只控制等待队列空位的时间。 */
mult_uart_status_t mux_service_submit(
    const mult_uart_request_t *request,
    uint32_t queue_timeout_ms);

#ifdef __cplusplus
}
#endif

#endif /* MUX_SERVICE_H */
