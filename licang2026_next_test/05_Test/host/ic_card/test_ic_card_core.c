#include <assert.h>
#include <stdio.h>
#include <string.h>

#include "ic_bsp.h"

typedef struct {
    uint8_t tx[IC_CARD_FRAME_SIZE_MAX];
    size_t tx_len;
    uint8_t *rx;
    size_t rx_capacity;
    unsigned rx_starts;
} fake_port_t;

static ic_card_status_t fake_tx(void *ctx, const uint8_t *data, size_t len)
{
    fake_port_t *fake = (fake_port_t *)ctx;
    (void)memcpy(fake->tx, data, len);
    fake->tx_len = len;
    return IC_CARD_OK;
}

static ic_card_status_t fake_rx(void *ctx, uint8_t *data, size_t capacity)
{
    fake_port_t *fake = (fake_port_t *)ctx;
    fake->rx = data;
    fake->rx_capacity = capacity;
    ++fake->rx_starts;
    return IC_CARD_OK;
}

static ic_card_status_t fake_abort(void *ctx)
{
    (void)ctx;
    return IC_CARD_OK;
}

/** 模拟一次可被IDLE切开的DMA接收事件，验证Core不依赖物理分包边界。 */
static void fake_feed(
    ic_card_t *device,
    fake_port_t *fake,
    const uint8_t *data,
    size_t len)
{
    assert(len <= fake->rx_capacity);
    (void)memcpy(fake->rx, data, len);
    ic_bsp_rx_isr(device, (uint16_t)len);
    ic_bsp_process(device);
}

int main(void)
{
    ic_card_t device = {0};
    fake_port_t fake = {0};
    ic_card_port_t port = {fake_tx, fake_rx, fake_abort, &fake};
    ic_card_response_t response;
    uint8_t block[IC_CARD_BLOCK_DATA_SIZE];
    uint32_t sequence = 0U;
    uint8_t success[22] = {
        0x01U, 0x16U, 0xA3U, 0x20U, 0x00U,
        0x23U, 0x23U, 0x23U, 0x23U, 0x23U, 0x23U, 0x23U, 0x23U,
        0x23U, 0x23U, 0x23U, 0x23U, 0x23U, 0x23U, 0x23U, 0x23U, 0x00U,
    };
    uint8_t bad[22];
    const uint8_t expected_read[] = {
        0x01U, 0x08U, 0xA3U, 0x20U, 0x01U, 0x01U, 0x00U, 0x75U,
    };
    const uint8_t expected_query[] = {
        0x02U, 0x08U, 0xB0U, 0x00U, 0x00U, 0x00U, 0x00U, 0x45U,
    };
    uint8_t pure_frame[IC_CARD_COMMAND_FRAME_SIZE];
    size_t pure_len = 0U;

    assert(ic_bsp_init(&device, &port) == IC_CARD_OK);
    assert(fake.rx_starts == 1U);
    assert(ic_bsp_read(&device, 0x20U, 0x01U, true) == IC_CARD_OK);
    assert(fake.tx_len == sizeof(expected_read));
    assert(memcmp(fake.tx, expected_read, sizeof(expected_read)) == 0);
    ic_bsp_tx_isr(&device);

    success[21] = ic_checksum(success, 21U);
    fake_feed(&device, &fake, success, 7U);
    fake_feed(&device, &fake, &success[7], 15U);
    assert(ic_bsp_take_response(&device, &sequence, &response));
    assert(ic_block_data(&response, 0x20U, block) == IC_CARD_OK);
    assert(block[0] == 0x23U);

    (void)memcpy(bad, success, sizeof(bad));
    bad[21] ^= 0x01U;
    fake_feed(&device, &fake, bad, sizeof(bad));
    assert(!ic_bsp_take_response(&device, &sequence, &response));
    assert(device.invalid_frame_count == 1U);

    assert(ic_bsp_query(
        &device, IC_CARD_CMD_QUERY_ADDRESS, 0x20U) == IC_CARD_OK);
    assert(memcmp(fake.tx, expected_query, sizeof(expected_query)) == 0);

    /* 复用transport调用的纯构帧/整帧解析必须与直连Core完全一致。 */
    assert(ic_read_frame(
        0x20U, 0x01U, true, pure_frame, sizeof(pure_frame), &pure_len) ==
        IC_CARD_OK);
    assert(pure_len == sizeof(expected_read));
    assert(memcmp(pure_frame, expected_read, pure_len) == 0);
    assert(ic_query_frame(
        IC_CARD_CMD_QUERY_ADDRESS,
        0x20U,
        pure_frame,
        sizeof(pure_frame),
        &pure_len) == IC_CARD_OK);
    assert(memcmp(pure_frame, expected_query, pure_len) == 0);
    assert(ic_parse_frame(
        success, sizeof(success), &response) == IC_CARD_OK);
    assert(response.command == IC_CARD_CMD_READ_BLOCK_KEY_A);
    assert(ic_parse_frame(
        bad, sizeof(bad), &response) == IC_CARD_ERR_PROTOCOL);
    puts("IC card core fake tests passed");
    return 0;
}
