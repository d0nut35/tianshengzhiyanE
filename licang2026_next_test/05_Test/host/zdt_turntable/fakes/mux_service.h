#ifndef TEST_FAKE_MUX_SERVICE_H
#define TEST_FAKE_MUX_SERVICE_H

#include <stddef.h>
#include <stdint.h>

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

typedef enum {
    MULT_UART_OP_SELECT = 0,
    MULT_UART_OP_WRITE,
    MULT_UART_OP_READ,
    MULT_UART_OP_WRITE_READ,
} mult_uart_operation_t;

typedef enum {
    MUX_DEVICE_0 = 0,
    MUX_DEVICE_1,
    MUX_DEVICE_2,
    MUX_DEVICE_3,
} mux_device_t;

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

mult_uart_status_t mux_submit(const mux_transfer_t *transfer);

#endif /* TEST_FAKE_MUX_SERVICE_H */
