/** @file test_ic_card_device_transport.c @brief IC正式mux读球接口测试。 */

#include <assert.h>
#include <stdio.h>
#include <string.h>

#include "ic_card_service.h"
#include "mux_service.h"

static mux_transfer_t g_transfer;
static uint8_t g_tx[IC_CARD_COMMAND_FRAME_SIZE];
static mult_uart_status_t g_submit_status = MULT_UART_OK;
static unsigned g_submit_count;
static unsigned g_done_count;
static uint32_t g_done_request_id;
static ic_card_status_t g_done_status;
static ic_result_t g_done_result;

mult_uart_status_t mux_submit(const mux_transfer_t *transfer)
{
    assert(transfer != NULL);
    ++g_submit_count;
    if (g_submit_status != MULT_UART_OK) return g_submit_status;
    g_transfer = *transfer;
    assert(transfer->tx_len <= sizeof(g_tx));
    (void)memcpy(g_tx, transfer->tx_data, transfer->tx_len);
    g_transfer.tx_data = g_tx;
    return MULT_UART_OK;
}

static void read_done(
    void *user_ctx,
    uint32_t request_id,
    ic_card_status_t status,
    const ic_result_t *result)
{
    assert(user_ctx == &g_done_count);
    ++g_done_count;
    g_done_request_id = request_id;
    g_done_status = status;
    if (result != NULL) g_done_result = *result;
}

static void complete(mult_uart_status_t status, uint8_t value)
{
    uint8_t frame[IC_CARD_BLOCK_DATA_SIZE + 6U] = {0};
    mux_completion_t completion = {0};

    frame[0] = IC_CARD_PACKET_CARD_OPERATION;
    frame[1] = (uint8_t)sizeof(frame);
    frame[2] = IC_CARD_CMD_READ_BLOCK_KEY_A;
    frame[3] = IC_ADDRESS;
    frame[4] = 0U;
    (void)memset(&frame[5], value, IC_CARD_BLOCK_DATA_SIZE);
    frame[sizeof(frame) - 1U] = ic_checksum(frame, sizeof(frame) - 1U);
    completion.request_id = 99U;
    completion.device = MUX_DEVICE_1;
    completion.operation = MULT_UART_OP_WRITE_READ;
    completion.status = status;
    completion.rx_data = frame;
    completion.rx_len = sizeof(frame);
    g_transfer.done_cb(g_transfer.user_ctx, &completion);
}

int main(void)
{
    uint32_t first_request_id;
    static const uint8_t expected[] = {
        IC_CARD_PACKET_CARD_OPERATION,
        IC_CARD_COMMAND_FRAME_SIZE,
        IC_CARD_CMD_READ_BLOCK_KEY_A,
        IC_ADDRESS,
        IC_DATA_BLOCK,
        1U,
        0U,
        0U,
    };

    assert(ic_read(true, read_done, &g_done_count) == IC_CARD_ERR_NOT_INIT);
    assert(ic_init() == IC_CARD_OK);
    assert(ic_init() == IC_CARD_ERR_STATE);

    assert(ic_read(true, read_done, &g_done_count) == IC_CARD_OK);
    assert(g_submit_count == 1U);
    assert(g_transfer.device == MUX_DEVICE_1);
    assert(g_transfer.operation == MULT_UART_OP_WRITE_READ);
    assert(g_transfer.tx_len == sizeof(expected));
    assert(g_transfer.rx_capacity == IC_CARD_FRAME_SIZE_MAX);
    assert(g_transfer.io_timeout_ms == IC_READ_TIMEOUT_MS);
    assert(g_transfer.queue_timeout_ms == 0U);
    assert(memcmp(g_transfer.tx_data, expected, sizeof(expected) - 1U) == 0);
    assert(g_transfer.tx_data[7] == ic_checksum(g_transfer.tx_data, 7U));
    assert(ic_read(false, read_done, &g_done_count) == IC_CARD_ERR_BUSY);

    complete(MULT_UART_OK, 0x23U);
    assert(g_done_count == 1U);
    assert(g_done_status == IC_CARD_OK);
    assert(g_done_request_id != 0U);
    assert(g_done_result.ball.kind == IC_BALL_TARGET);
    assert(g_done_result.ball.row == 2U);
    assert(g_done_result.ball.column == 3U);
    first_request_id = g_done_request_id;

    assert(ic_read(false, read_done, &g_done_count) == IC_CARD_OK);
    complete(MULT_UART_ERR_TIMEOUT, 0U);
    assert(g_done_count == 2U);
    assert(g_done_status == IC_CARD_ERR_TIMEOUT);
    assert(g_done_request_id != 0U);
    assert(g_done_request_id != first_request_id);

    g_submit_status = MULT_UART_ERR_QUEUE_FULL;
    assert(ic_read(false, read_done, &g_done_count) == IC_CARD_ERR_QUEUE_FULL);
    g_submit_status = MULT_UART_OK;
    assert(ic_read(false, read_done, &g_done_count) == IC_CARD_OK);
    complete(MULT_UART_OK, 0x14U);
    assert(g_done_count == 3U);
    assert(g_done_result.ball.row == 1U);
    assert(g_done_result.ball.column == 4U);

    puts("IC card mux interface tests passed");
    return 0;
}
