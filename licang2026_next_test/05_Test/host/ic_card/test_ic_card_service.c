#include <assert.h>
#include <stdio.h>
#include <string.h>

#include "ic_card_service.h"

typedef struct {
    uint8_t *rx;
    size_t rx_capacity;
    uint8_t tx[IC_CARD_FRAME_SIZE_MAX];
    size_t tx_len;
    uint32_t now;
    unsigned aborts;
    unsigned callback_count;
    ic_card_status_t callback_status;
} fixture_t;

static ic_card_status_t fake_tx(void *ctx, const uint8_t *data, size_t len)
{
    fixture_t *fixture = (fixture_t *)ctx;
    (void)memcpy(fixture->tx, data, len);
    fixture->tx_len = len;
    return IC_CARD_OK;
}

static ic_card_status_t fake_rx(void *ctx, uint8_t *data, size_t capacity)
{
    fixture_t *fixture = (fixture_t *)ctx;
    fixture->rx = data;
    fixture->rx_capacity = capacity;
    return IC_CARD_OK;
}

static ic_card_status_t fake_abort(void *ctx)
{
    ++((fixture_t *)ctx)->aborts;
    return IC_CARD_OK;
}

static uint32_t fake_now_ms(void *ctx)
{
    return ((fixture_t *)ctx)->now;
}

static void fake_done(
    void *ctx,
    uint32_t request_id,
    ic_card_status_t status,
    const ic_card_response_t *response)
{
    fixture_t *fixture = (fixture_t *)ctx;
    (void)request_id;
    (void)response;
    ++fixture->callback_count;
    fixture->callback_status = status;
}

int main(void)
{
    fixture_t fixture = {0};
    ic_card_t device = {0};
    ic_card_service_t service = {0};
    ic_card_port_t port = {fake_tx, fake_rx, fake_abort, &fixture};
    ic_card_service_config_t config = {
        &device, fake_now_ms, &fixture, NULL, NULL,
    };
    ic_card_request_t request = {0};
    uint8_t response_frame[22] = {
        0x01U, 0x16U, 0xA3U, 0x20U, 0x00U,
    };

    assert(ic_card_init(&device, &port) == IC_CARD_OK);
    assert(ic_card_service_init(&service, &config) == IC_CARD_OK);
    request.request_id = 7U;
    request.type = IC_CARD_REQUEST_READ_BLOCK;
    request.address = 0x20U;
    request.timeout_ms = 500U;
    request.data.read_block.block = 1U;
    request.done_cb = fake_done;
    request.user_ctx = &fixture;

    assert(ic_card_service_submit(&service, &request) == IC_CARD_OK);
    ic_card_service_process_once(&service);
    assert((fixture.tx_len == 8U) && service.active_valid);
    ic_card_on_tx_complete_isr(&device);
    (void)memset(&response_frame[5], 0x31, 16U);
    response_frame[21] = ic_card_checksum(response_frame, 21U);
    (void)memcpy(fixture.rx, response_frame, sizeof(response_frame));
    ic_card_on_rx_event_isr(&device, sizeof(response_frame));
    ic_card_service_process_once(&service);
    assert((fixture.callback_count == 1U) &&
           (fixture.callback_status == IC_CARD_OK));

    /* 第二笔不喂响应，用fake时间推进到截止点，验证abort和超时回调。 */
    request.request_id = 8U;
    assert(ic_card_service_submit(&service, &request) == IC_CARD_OK);
    ic_card_service_process_once(&service);
    fixture.now = 501U;
    ic_card_service_process_once(&service);
    assert((fixture.callback_count == 2U) &&
           (fixture.callback_status == IC_CARD_ERR_TIMEOUT));
    assert(fixture.aborts == 1U);
    puts("IC card service fake tests passed");
    return 0;
}
