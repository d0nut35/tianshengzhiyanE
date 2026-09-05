/** @file zdt_turntable_service.c @brief ZDT转盘复用事务Service实现。 */

#include "zdt_turntable_service.h"

#include <stdbool.h>
#include <string.h>

#include "mux_service.h"

typedef struct {
    volatile bool active;
    uint32_t request_id;
    uint8_t expected_address;
    uint8_t expected_function;
    zdt_turntable_done_fn_t done_cb;
    void *user_ctx;
} turn_service_t;

static turn_service_t g_turn;

/** 将复用器状态收敛为ZDT状态。 */
static __attribute__((noinline)) zdt_turntable_status_t turn_map_mux(
    mult_uart_status_t status)
{
    switch (status) {
    case MULT_UART_OK:             return ZDT_TURNTABLE_OK;
    case MULT_UART_ERR_BUSY:       return ZDT_TURNTABLE_ERR_BUSY;
    case MULT_UART_ERR_QUEUE_FULL: return ZDT_TURNTABLE_ERR_QUEUE_FULL;
    case MULT_UART_ERR_TIMEOUT:    return ZDT_TURNTABLE_ERR_TIMEOUT;
    case MULT_UART_ERR_PARAM:
    case MULT_UART_ERR_OVERFLOW:   return ZDT_TURNTABLE_ERR_PARAM;
    default:                       return ZDT_TURNTABLE_ERR_IO;
    }
}

/** 在mux worker普通上下文解析响应并发布原请求回调。 */
static void turn_mux_done(
    void *user_ctx,
    const mux_completion_t *completion)
{
    turn_service_t *service = &g_turn;
    zdt_turntable_response_t response;
    zdt_turntable_status_t status;
    zdt_turntable_done_fn_t done_cb;
    void *done_ctx;
    uint32_t request_id;

    (void)user_ctx;
    done_cb = service->done_cb;
    done_ctx = service->user_ctx;
    request_id = service->request_id;
    service->active = false;
    status = turn_map_mux(completion->status);
    if (status == ZDT_TURNTABLE_OK) {
        status = turn_parse(
            completion->rx_data,
            completion->rx_len,
            service->expected_address,
            service->expected_function,
            &response);
    }
    if (done_cb != NULL) {
        done_cb(
            done_ctx,
            request_id,
            status,
            ((status == ZDT_TURNTABLE_OK) ||
             (status == ZDT_TURNTABLE_ERR_DEVICE)) ? &response : NULL);
    }
}

zdt_turntable_status_t turn_service_submit(
    void *submit_ctx,
    const zdt_turntable_request_t *request)
{
    mux_transfer_t transfer;
    mult_uart_status_t status;

    (void)submit_ctx;
    if (request == NULL) return ZDT_TURNTABLE_ERR_PARAM;
    if (request->frame_len > ZDT_TURNTABLE_FRAME_MAX) {
        return ZDT_TURNTABLE_ERR_PARAM;
    }
    if (g_turn.active) return ZDT_TURNTABLE_ERR_BUSY;

    g_turn.request_id = request->request_id;
    g_turn.expected_address = request->expected_address;
    g_turn.expected_function = request->expected_function;
    g_turn.done_cb = request->done_cb;
    g_turn.user_ctx = request->user_ctx;
    g_turn.active = true;
    (void)memset(&transfer, 0, sizeof(transfer));
    transfer.device = MUX_DEVICE_2;
    transfer.operation = MULT_UART_OP_WRITE_READ;
    transfer.tx_data = request->frame;
    transfer.tx_len = request->frame_len;
    transfer.rx_capacity = ZDT_TURNTABLE_RESPONSE_MAX;
    transfer.io_timeout_ms = request->timeout_ms;
    transfer.done_cb = turn_mux_done;
    status = mux_submit(&transfer);
    if (status != MULT_UART_OK) g_turn.active = false;
    return turn_map_mux(status);
}
