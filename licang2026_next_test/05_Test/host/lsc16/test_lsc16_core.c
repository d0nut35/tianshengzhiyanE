/**
 * @file    test_lsc16_core.c
 * @brief   LSC16协议Core的PC fake测试。
 */

#include <assert.h>
#include <stdio.h>
#include <string.h>

#include "lsc16_core.h"

typedef struct {
    uint8_t tx[LSC16_FRAME_SIZE_MAX];
    size_t tx_len;
    uint8_t *rx;
    size_t rx_capacity;
    uint32_t abort_count;
} fake_port_t;

static lsc16_status_t fake_tx(void *ctx, const uint8_t *data, size_t len)
{
    fake_port_t *fake = (fake_port_t *)ctx;
    assert(len <= sizeof(fake->tx));
    memcpy(fake->tx, data, len);
    fake->tx_len = len;
    return LSC16_OK;
}

static lsc16_status_t fake_rx(void *ctx, uint8_t *data, size_t capacity)
{
    fake_port_t *fake = (fake_port_t *)ctx;
    fake->rx = data;
    fake->rx_capacity = capacity;
    return LSC16_OK;
}

static lsc16_status_t fake_abort(void *ctx)
{
    ++((fake_port_t *)ctx)->abort_count;
    return LSC16_OK;
}

static void test_servo_frame(void)
{
    static const uint8_t expected[] = {
        0x55U, 0x55U, 0x08U, 0x03U, 0x01U,
        0xE8U, 0x03U, 0x01U, 0xD0U, 0x07U,
    };
    fake_port_t fake = {0};
    lsc16_port_t port = {fake_tx, fake_rx, fake_abort, &fake};
    lsc16_servo_target_t target = {1U, 2000U};
    lsc16_t device;

    assert(lsc16_init(&device, &port) == LSC16_OK);
    assert(fake.rx_capacity == LSC16_DMA_RX_BUFFER_SIZE);
    assert(lsc16_move_servos(&device, &target, 1U, 1000U) == LSC16_OK);
    assert(fake.tx_len == sizeof(expected));
    assert(memcmp(fake.tx, expected, sizeof(expected)) == 0);
    assert(lsc16_move_servos(&device, &target, 1U, 1000U) == LSC16_ERR_BUSY);
    lsc16_on_tx_complete_isr(&device);
    assert(!lsc16_is_tx_busy(&device));
}

static void test_action_frame_and_reports(void)
{
    static const uint8_t run_expected[] = {
        0x55U, 0x55U, 0x05U, 0x06U, 0x08U, 0x01U, 0x00U,
    };
    static const uint8_t completed[] = {
        0x55U, 0x55U, 0x05U, 0x08U, 0x08U, 0x01U, 0x00U,
    };
    fake_port_t fake = {0};
    lsc16_port_t port = {fake_tx, fake_rx, fake_abort, &fake};
    lsc16_report_t report;
    lsc16_t device;
    uint32_t events;

    assert(lsc16_init(&device, &port) == LSC16_OK);
    assert(lsc16_run_action_group(&device, 8U, 1U) == LSC16_OK);
    assert(fake.tx_len == sizeof(run_expected));
    assert(memcmp(fake.tx, run_expected, sizeof(run_expected)) == 0);

    memcpy(fake.rx, run_expected, sizeof(run_expected));
    lsc16_on_rx_event_isr(&device, (uint16_t)sizeof(run_expected));
    lsc16_process(&device);
    events = lsc16_take_report_events(&device);
    assert((events & LSC16_REPORT_EVENT_ACTION_STARTED) != 0U);
    assert(lsc16_get_last_report(&device, &report) == LSC16_OK);
    assert((report.action_group == 8U) && (report.repeat_count == 1U));

    memcpy(fake.rx, completed, sizeof(completed));
    lsc16_on_rx_event_isr(&device, (uint16_t)sizeof(completed));
    lsc16_process(&device);
    events = lsc16_take_report_events(&device);
    assert((events & LSC16_REPORT_EVENT_ACTION_COMPLETED) != 0U);
}

static void test_validation(void)
{
    fake_port_t fake = {0};
    lsc16_port_t port = {fake_tx, fake_rx, fake_abort, &fake};
    lsc16_servo_target_t target = {16U, 1500U};
    lsc16_t device;

    assert(lsc16_init(&device, &port) == LSC16_OK);
    assert(lsc16_move_servos(&device, &target, 1U, 1000U) == LSC16_ERR_PARAM);
    target.id = 1U;
    target.position = 499U;
    assert(lsc16_move_servos(&device, &target, 1U, 1000U) == LSC16_ERR_PARAM);
}

int main(void)
{
    test_servo_frame();
    test_action_frame_and_reports();
    test_validation();
    puts("LSC16 core fake tests passed");
    return 0;
}
