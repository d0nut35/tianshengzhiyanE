/** @file test_zdt_turntable_service.c @brief ZDT合并Service主机测试。 */
#include <stdio.h>
#include <string.h>
#include "zdt_turntable_service.h"
#include "mux_service.h"

static int failures;
#define CHECK(x) do { if (!(x)) { printf("FAIL line %d: %s\n", __LINE__, #x); ++failures; } } while (0)
static mux_transfer_t transfer;
static uint8_t tx[ZDT_TURNTABLE_FRAME_MAX];
static mult_uart_status_t submit_status = MULT_UART_OK;
static unsigned submit_count, done_count;
static uint32_t done_id, previous_id;
static zdt_turntable_status_t done_status;
static zdt_turntable_response_t response;

mult_uart_status_t mux_submit(const mux_transfer_t *request)
{
    CHECK(request != NULL);
    ++submit_count;
    if (submit_status != MULT_UART_OK) return submit_status;
    transfer = *request;
    CHECK(request->tx_len <= sizeof(tx));
    memcpy(tx, request->tx_data, request->tx_len);
    transfer.tx_data = tx;
    return MULT_UART_OK;
}
static void done(void *ctx, uint32_t id, zdt_turntable_status_t status,
    const zdt_turntable_response_t *reply)
{
    CHECK(ctx == &done_count);
    ++done_count; done_id = id; done_status = status;
    if (reply != NULL) response = *reply;
}
static void complete(mult_uart_status_t status, const uint8_t *data, size_t len)
{
    mux_completion_t result = {0};
    result.device = MUX_DEVICE_2;
    result.operation = MULT_UART_OP_WRITE_READ;
    result.status = status; result.rx_data = data; result.rx_len = len;
    transfer.done_cb(transfer.user_ctx, &result);
}
static void check_transfer(uint8_t function, size_t len)
{
    CHECK(transfer.device == MUX_DEVICE_2);
    CHECK(transfer.operation == MULT_UART_OP_WRITE_READ);
    CHECK(transfer.tx_len == len && transfer.tx_data[1] == function);
    CHECK(transfer.rx_capacity == ZDT_TURNTABLE_RESPONSE_MAX);
    CHECK(transfer.io_timeout_ms == 500U);
}
static void test_options_and_emm(void)
{
    const uint8_t options[] = {1U, 0x1AU, 0x03U, 0xB7U, 0x6BU};
    const uint8_t ack[] = {1U, 0xFDU, 0x02U, 0x6BU};
    zdt_turntable_position_command_t cmd = {
        ZDT_TURNTABLE_DIR_CW, ZDT_TURNTABLE_POS_RELATIVE_LAST_TARGET,
        60U, 0U, 0U, 100U, 50U
    };
    CHECK(turn_query_options(done, &done_count) == ZDT_TURNTABLE_OK);
    check_transfer(0x1AU, 3U);
    CHECK(turn_query_status(done, &done_count) == ZDT_TURNTABLE_ERR_BUSY);
    complete(MULT_UART_OK, options, sizeof(options));
    CHECK(done_count == 1U && done_status == ZDT_TURNTABLE_OK);
    CHECK(response.kind == ZDT_TURNTABLE_REPLY_OPTIONS);
    CHECK(response.data.options.closed_loop);
    CHECK(response.data.options.firmware == ZDT_TURNTABLE_FIRMWARE_EMM);
    CHECK(done_id != 0U); previous_id = done_id;

    CHECK(turn_move_emm(&cmd, done, &done_count) == ZDT_TURNTABLE_OK);
    check_transfer(0xFDU, 13U);
    CHECK(tx[3] == 0x02U && tx[4] == 0x58U);
    CHECK(tx[6] == 0U && tx[7] == 0U && tx[8] == 0U && tx[9] == 89U);
    complete(MULT_UART_OK, ack, sizeof(ack));
    CHECK(done_count == 2U && done_status == ZDT_TURNTABLE_OK);
    CHECK(response.kind == ZDT_TURNTABLE_REPLY_ACK && done_id != previous_id);
}
static void test_status_stop_and_errors(void)
{
    const uint8_t status[] = {1U, 0x3AU, 0x83U, 0x6BU};
    const uint8_t stop_ack[] = {1U, 0xFEU, 0x02U, 0x6BU};
    CHECK(turn_query_status(done, &done_count) == ZDT_TURNTABLE_OK);
    check_transfer(0x3AU, 3U); complete(MULT_UART_OK, status, sizeof(status));
    CHECK(response.kind == ZDT_TURNTABLE_REPLY_STATUS);
    CHECK(response.data.motor_status.enabled && response.data.motor_status.reached);
    CHECK(turn_stop(done, &done_count) == ZDT_TURNTABLE_OK);
    check_transfer(0xFEU, 5U); complete(MULT_UART_OK, stop_ack, sizeof(stop_ack));
    CHECK(response.kind == ZDT_TURNTABLE_REPLY_ACK);
    CHECK(turn_query_status(done, &done_count) == ZDT_TURNTABLE_OK);
    complete(MULT_UART_ERR_TIMEOUT, NULL, 0U);
    CHECK(done_status == ZDT_TURNTABLE_ERR_TIMEOUT);
    submit_status = MULT_UART_ERR_QUEUE_FULL;
    CHECK(turn_query_status(done, &done_count) == ZDT_TURNTABLE_ERR_QUEUE_FULL);
    submit_status = MULT_UART_OK;
    CHECK(turn_query_status(done, &done_count) == ZDT_TURNTABLE_OK);
    complete(MULT_UART_ERR_IO, NULL, 0U);
    CHECK(done_status == ZDT_TURNTABLE_ERR_IO);
}
int main(void)
{
    turn_config_t config = {1U, 500U, 3200U};
    CHECK(turn_query_status(done, &done_count) == ZDT_TURNTABLE_ERR_NOT_INIT);
    CHECK(turn_init(NULL) == ZDT_TURNTABLE_ERR_PARAM);
    CHECK(turn_init(&config) == ZDT_TURNTABLE_OK);
    CHECK(turn_init(&config) == ZDT_TURNTABLE_ERR_STATE);
    test_options_and_emm();
    test_status_stop_and_errors();
    if (failures != 0) return 1;
    puts("zdt_turntable_service tests passed");
    return 0;
}
