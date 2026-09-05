/** @file test_ic_card_device_transport.c @brief IC Device可注入transport测试。 */

#include <assert.h>
#include <stdio.h>
#include <string.h>

#include "ic_card_device.h"

typedef struct {
    unsigned submitted;
    ic_card_request_t request;
} fake_transport_t;

static bool read_done;
static ic_card_ball_result_t read_result;

static ic_card_status_t fake_submit(
    void *ctx,
    const ic_card_request_t *request,
    uint32_t queue_timeout_ms)
{
    fake_transport_t *fake = (fake_transport_t *)ctx;
    assert(queue_timeout_ms == 0U);
    fake->request = *request;
    fake->submitted++;
    return IC_CARD_OK;
}

static void fake_read_done(
    void *ctx,
    uint32_t request_id,
    ic_card_status_t status,
    const ic_card_ball_result_t *result)
{
    (void)ctx;
    assert(request_id != 0U);
    assert(status == IC_CARD_OK);
    assert(result != NULL);
    read_result = *result;
    read_done = true;
}

int main(void)
{
    fake_transport_t fake = {0};
    ic_card_response_t response = {0};

    assert(ic_card_device_init_with_transport(fake_submit, &fake) == IC_CARD_OK);
    assert(ic_card_device_read_competition_ball(
        true, fake_read_done, NULL) == IC_CARD_OK);
    assert(fake.submitted == 1U);
    assert(fake.request.type == IC_CARD_REQUEST_READ_BLOCK);
    assert(fake.request.data.read_block.block == IC_CARD_DEVICE_DATA_BLOCK);

    response.packet_type = IC_CARD_PACKET_CARD_OPERATION;
    response.command = IC_CARD_CMD_READ_BLOCK_KEY_A;
    response.address = IC_CARD_DEVICE_ADDRESS;
    response.payload_len = IC_CARD_BLOCK_DATA_SIZE;
    memset(response.payload, 0x23, IC_CARD_BLOCK_DATA_SIZE);
    fake.request.done_cb(
        fake.request.user_ctx,
        fake.request.request_id,
        IC_CARD_OK,
        &response);
    assert(read_done);
    assert(read_result.ball.row == 2U && read_result.ball.column == 3U);
    puts("IC card injected transport tests passed");
    return 0;
}
