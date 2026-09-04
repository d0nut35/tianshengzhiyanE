#include <assert.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "mult_uart_device.h"
#include "mult_uart_service_os.h"

typedef struct {
    mult_uart_request_t last_request;
    uint32_t last_queue_timeout_ms;
    size_t submit_count;
    mult_uart_status_t submit_status;
} fake_service_os_t;

typedef struct {
    mult_uart_device_completion_t completion;
    uint8_t rx_snapshot[MULT_UART_SERVICE_RX_MAX];
    size_t completion_count;
} fake_app_t;

static fake_service_os_t fake_service_os;

mult_uart_status_t mult_uart_service_os_submit(
    const mult_uart_request_t *request,
    uint32_t queue_timeout_ms)
{
    assert(request != NULL);
    fake_service_os.last_request = *request;
    fake_service_os.last_queue_timeout_ms = queue_timeout_ms;
    fake_service_os.submit_count++;
    return fake_service_os.submit_status;
}

static void app_done(
    void *user_ctx,
    const mult_uart_device_completion_t *completion)
{
    fake_app_t *app = (fake_app_t *)user_ctx;

    app->completion = *completion;
    if ((completion->rx_data != NULL) && (completion->rx_len > 0U)) {
        memcpy(app->rx_snapshot, completion->rx_data, completion->rx_len);
    }
    app->completion_count++;
}

static void reset_fake(void)
{
    memset(&fake_service_os, 0, sizeof(fake_service_os));
    fake_service_os.submit_status = MULT_UART_OK;
}

static void complete_last_request(
    mult_uart_status_t status,
    const uint8_t *rx_data,
    size_t rx_len)
{
    mult_uart_completion_t completion;

    completion.request_id = fake_service_os.last_request.request_id;
    completion.status = status;
    completion.operation = fake_service_os.last_request.operation;
    completion.channel = fake_service_os.last_request.channel;
    completion.rx_data = rx_data;
    completion.rx_len = rx_len;
    fake_service_os.last_request.done_cb(
        fake_service_os.last_request.user_ctx,
        &completion);
}

static void test_default_mapping_and_completion(void)
{
    fake_app_t app;
    const uint8_t tx[] = {'p', 'i', 'n', 'g'};
    const uint8_t rx[] = {'o', 'k'};
    mult_uart_device_transfer_t transfer = {
        MULT_UART_DEVICE_1,
        MULT_UART_OP_WRITE_READ,
        tx,
        sizeof(tx),
        16U,
        50U,
        7U,
        app_done,
        &app,
    };

    reset_fake();
    memset(&app, 0, sizeof(app));
    assert(mult_uart_device_init(NULL, 0U) == MULT_UART_OK);

    assert(mult_uart_device_submit(&transfer) == MULT_UART_OK);
    assert(fake_service_os.submit_count == 1U);
    assert(fake_service_os.last_request.channel == MULT_UART_CHANNEL_1);
    assert(fake_service_os.last_request.operation == MULT_UART_OP_WRITE_READ);
    assert(fake_service_os.last_request.tx_data == tx);
    assert(fake_service_os.last_request.tx_len == sizeof(tx));
    assert(fake_service_os.last_request.rx_capacity == 16U);
    assert(fake_service_os.last_request.io_timeout_ms == 50U);
    assert(fake_service_os.last_queue_timeout_ms == 7U);

    complete_last_request(MULT_UART_OK, rx, sizeof(rx));
    assert(app.completion_count == 1U);
    assert(app.completion.device_id == MULT_UART_DEVICE_1);
    assert(app.completion.status == MULT_UART_OK);
    assert(app.completion.rx_len == sizeof(rx));
    assert(memcmp(app.rx_snapshot, rx, sizeof(rx)) == 0);
    assert(mult_uart_device_deinit() == MULT_UART_OK);
}

static void test_same_device_busy_until_completion(void)
{
    const uint8_t tx[] = {'a'};
    mult_uart_device_transfer_t transfer = {
        MULT_UART_DEVICE_2,
        MULT_UART_OP_WRITE,
        tx,
        sizeof(tx),
        0U,
        10U,
        0U,
        NULL,
        NULL,
    };
    mult_uart_device_stats_t stats;

    reset_fake();
    assert(mult_uart_device_init(NULL, 0U) == MULT_UART_OK);
    assert(mult_uart_device_submit(&transfer) == MULT_UART_OK);
    assert(mult_uart_device_submit(&transfer) == MULT_UART_ERR_BUSY);
    assert(mult_uart_device_deinit() == MULT_UART_ERR_BUSY);
    assert(mult_uart_device_get_stats(&stats) == MULT_UART_OK);
    assert(stats.busy == 1U);

    complete_last_request(MULT_UART_OK, NULL, 0U);
    assert(mult_uart_device_submit(&transfer) == MULT_UART_OK);
    complete_last_request(MULT_UART_OK, NULL, 0U);
    assert(mult_uart_device_deinit() == MULT_UART_OK);
}

static void test_custom_config_and_disabled_device(void)
{
    mult_uart_device_config_t configs[MULT_UART_DEVICE_COUNT];
    const uint8_t tx[] = {'x'};
    mult_uart_device_transfer_t transfer = {
        MULT_UART_DEVICE_0,
        MULT_UART_OP_WRITE,
        tx,
        sizeof(tx),
        0U,
        0U,
        0U,
        NULL,
        NULL,
    };

    reset_fake();
    configs[0].enabled = true;
    configs[0].channel = MULT_UART_CHANNEL_3;
    configs[0].default_io_timeout_ms = 25U;
    configs[1].enabled = false;
    configs[1].channel = MULT_UART_CHANNEL_1;
    configs[1].default_io_timeout_ms = 25U;
    configs[2].enabled = true;
    configs[2].channel = MULT_UART_CHANNEL_2;
    configs[2].default_io_timeout_ms = 25U;
    configs[3].enabled = true;
    configs[3].channel = MULT_UART_CHANNEL_0;
    configs[3].default_io_timeout_ms = 25U;

    assert(mult_uart_device_init(configs, MULT_UART_DEVICE_COUNT) ==
           MULT_UART_OK);
    assert(mult_uart_device_submit(&transfer) == MULT_UART_OK);
    assert(fake_service_os.last_request.channel == MULT_UART_CHANNEL_3);
    assert(fake_service_os.last_request.io_timeout_ms == 25U);
    complete_last_request(MULT_UART_OK, NULL, 0U);

    transfer.device_id = MULT_UART_DEVICE_1;
    assert(mult_uart_device_submit(&transfer) == MULT_UART_ERR_STATE);
    assert(mult_uart_device_deinit() == MULT_UART_OK);
}

int main(void)
{
    test_default_mapping_and_completion();
    test_same_device_busy_until_completion();
    test_custom_config_and_disabled_device();
    puts("mult_uart_device tests passed");
    return 0;
}
