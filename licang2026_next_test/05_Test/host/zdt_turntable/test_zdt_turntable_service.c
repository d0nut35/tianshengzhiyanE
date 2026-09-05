/** @file test_zdt_turntable_service.c @brief ZDT复用Service主机测试。 */

#include <stdio.h>
#include <string.h>

#include "zdt_turntable_service.h"
#include "mux_service.h"

static int failures;
#define CHECK(x) do { if (!(x)) { printf("FAIL line %d: %s\n", __LINE__, #x); failures++; } } while (0)

static mux_transfer_t g_transfer;
static uint8_t g_tx[ZDT_TURNTABLE_FRAME_MAX];
static mult_uart_status_t g_submit_status = MULT_UART_OK;
static unsigned g_submit_count;
static unsigned g_done_count;
static uint32_t g_done_id;
static zdt_turntable_status_t g_done_status;
static zdt_turntable_response_t g_response;

mult_uart_status_t mux_submit(const mux_transfer_t *transfer)
{
    CHECK(transfer != NULL);
    ++g_submit_count;
    if (g_submit_status != MULT_UART_OK) return g_submit_status;
    g_transfer = *transfer;
    CHECK(transfer->tx_len <= sizeof(g_tx));
    (void)memcpy(g_tx, transfer->tx_data, transfer->tx_len);
    g_transfer.tx_data = g_tx;
    return MULT_UART_OK;
}

static void done(
    void *user_ctx,
    uint32_t request_id,
    zdt_turntable_status_t status,
    const zdt_turntable_response_t *response)
{
    CHECK(user_ctx == &g_done_count);
    ++g_done_count;
    g_done_id = request_id;
    g_done_status = status;
    if (response != NULL) g_response = *response;
}

static zdt_turntable_request_t make_request(uint32_t request_id)
{
    zdt_turntable_request_t request;

    (void)memset(&request, 0, sizeof(request));
    request.request_id = request_id;
    request.frame[0] = 1U;
    request.frame[1] = 0x3AU;
    request.frame[2] = ZDT_TURNTABLE_CHECK_BYTE;
    request.frame_len = 3U;
    request.expected_address = 1U;
    request.expected_function = 0x3AU;
    request.timeout_ms = 500U;
    request.done_cb = done;
    request.user_ctx = &g_done_count;
    return request;
}

static void complete(
    mult_uart_status_t status,
    const uint8_t *rx,
    size_t rx_len)
{
    mux_completion_t completion = {0};

    completion.request_id = 99U;
    completion.device = MUX_DEVICE_2;
    completion.operation = MULT_UART_OP_WRITE_READ;
    completion.status = status;
    completion.rx_data = rx;
    completion.rx_len = rx_len;
    g_transfer.done_cb(g_transfer.user_ctx, &completion);
}

static void test_success_and_busy(void)
{
    zdt_turntable_request_t request = make_request(7U);
    const uint8_t reply[] = {1U, 0x3AU, 0x83U, ZDT_TURNTABLE_CHECK_BYTE};

    CHECK(turn_service_submit(NULL, &request) == ZDT_TURNTABLE_OK);
    CHECK(g_submit_count == 1U);
    CHECK(g_transfer.device == MUX_DEVICE_2);
    CHECK(g_transfer.operation == MULT_UART_OP_WRITE_READ);
    CHECK(g_transfer.tx_len == request.frame_len);
    CHECK(g_transfer.rx_capacity == ZDT_TURNTABLE_RESPONSE_MAX);
    CHECK(g_transfer.io_timeout_ms == request.timeout_ms);
    CHECK(g_transfer.queue_timeout_ms == 0U);
    CHECK(memcmp(g_transfer.tx_data, request.frame, request.frame_len) == 0);
    CHECK(turn_service_submit(NULL, &request) == ZDT_TURNTABLE_ERR_BUSY);

    complete(MULT_UART_OK, reply, sizeof(reply));
    CHECK(g_done_count == 1U);
    CHECK(g_done_id == 7U);
    CHECK(g_done_status == ZDT_TURNTABLE_OK);
    CHECK(g_response.kind == ZDT_TURNTABLE_REPLY_STATUS);
}

static void test_error_mapping_and_recovery(void)
{
    zdt_turntable_request_t request = make_request(8U);

    CHECK(turn_service_submit(NULL, &request) == ZDT_TURNTABLE_OK);
    complete(MULT_UART_ERR_TIMEOUT, NULL, 0U);
    CHECK(g_done_count == 2U);
    CHECK(g_done_id == 8U);
    CHECK(g_done_status == ZDT_TURNTABLE_ERR_TIMEOUT);

    g_submit_status = MULT_UART_ERR_QUEUE_FULL;
    CHECK(turn_service_submit(NULL, &request) ==
          ZDT_TURNTABLE_ERR_QUEUE_FULL);
    g_submit_status = MULT_UART_OK;
    request.request_id = 9U;
    CHECK(turn_service_submit(NULL, &request) == ZDT_TURNTABLE_OK);
    complete(MULT_UART_ERR_IO, NULL, 0U);
    CHECK(g_done_count == 3U);
    CHECK(g_done_id == 9U);
    CHECK(g_done_status == ZDT_TURNTABLE_ERR_IO);
}

static void test_protocol_error(void)
{
    zdt_turntable_request_t request = make_request(10U);
    const uint8_t bad_reply[] = {1U, 0x36U, 0U, ZDT_TURNTABLE_CHECK_BYTE};

    CHECK(turn_service_submit(NULL, &request) == ZDT_TURNTABLE_OK);
    complete(MULT_UART_OK, bad_reply, sizeof(bad_reply));
    CHECK(g_done_count == 4U);
    CHECK(g_done_id == 10U);
    CHECK(g_done_status == ZDT_TURNTABLE_ERR_PROTOCOL);
}

int main(void)
{
    zdt_turntable_request_t bad = {0};

    CHECK(turn_service_submit(NULL, NULL) == ZDT_TURNTABLE_ERR_PARAM);
    bad.frame_len = ZDT_TURNTABLE_FRAME_MAX + 1U;
    CHECK(turn_service_submit(NULL, &bad) == ZDT_TURNTABLE_ERR_PARAM);
    test_success_and_busy();
    test_error_mapping_and_recovery();
    test_protocol_error();
    if (failures != 0) return 1;
    puts("zdt_turntable_service tests passed");
    return 0;
}
