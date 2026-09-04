/**
 * @file    test_lsc16_service.c
 * @brief   LSC16 Service串行队列和异步完成的PC fake测试。
 */

#include <assert.h>
#include <stdio.h>
#include <string.h>

#include "lsc16_service.h"

typedef struct {
    uint32_t tx_count;
    uint8_t frames[2][LSC16_FRAME_SIZE_MAX];
    size_t lengths[2];
    uint8_t *rx;
} fake_port_t;

typedef struct {
    uint32_t count;
    uint32_t ids[2];
    lsc16_status_t status[2];
} completion_log_t;

static lsc16_status_t fake_tx(void *ctx, const uint8_t *data, size_t len)
{
    fake_port_t *fake = (fake_port_t *)ctx;
    assert(fake->tx_count < 2U);
    memcpy(fake->frames[fake->tx_count], data, len);
    fake->lengths[fake->tx_count] = len;
    ++fake->tx_count;
    return LSC16_OK;
}

static lsc16_status_t fake_rx(void *ctx, uint8_t *data, size_t capacity)
{
    (void)capacity;
    ((fake_port_t *)ctx)->rx = data;
    return LSC16_OK;
}

static lsc16_status_t fake_abort(void *ctx)
{
    (void)ctx;
    return LSC16_OK;
}

static void done(void *ctx, uint32_t request_id, lsc16_status_t status)
{
    completion_log_t *log = (completion_log_t *)ctx;
    log->ids[log->count] = request_id;
    log->status[log->count] = status;
    ++log->count;
}

int main(void)
{
    fake_port_t fake = {0};
    completion_log_t log = {0};
    lsc16_port_t port = {fake_tx, fake_rx, fake_abort, &fake};
    lsc16_service_config_t config = {0};
    lsc16_request_t move = {0};
    lsc16_request_t action = {0};
    lsc16_service_t service;
    lsc16_t device;

    assert(lsc16_init(&device, &port) == LSC16_OK);
    config.device = &device;
    assert(lsc16_service_init(&service, &config) == LSC16_OK);

    move.request_id = 10U;
    move.type = LSC16_REQUEST_MOVE_SERVOS;
    move.data.move.count = 1U;
    move.data.move.move_time_ms = 1000U;
    move.data.move.targets[0].id = 1U;
    move.data.move.targets[0].position = 1500U;
    move.done_cb = done;
    move.user_ctx = &log;
    action.request_id = 11U;
    action.type = LSC16_REQUEST_RUN_ACTION_GROUP;
    action.data.action_run.action_group = 8U;
    action.data.action_run.repeat_count = 1U;
    action.done_cb = done;
    action.user_ctx = &log;

    assert(lsc16_service_submit(&service, &move) == LSC16_OK);
    assert(lsc16_service_submit(&service, &action) == LSC16_OK);
    lsc16_service_process_once(&service);
    assert(fake.tx_count == 1U);
    lsc16_on_tx_complete_isr(&device);
    lsc16_service_process_once(&service);
    assert((log.count == 1U) && (log.ids[0] == 10U));
    assert(fake.tx_count == 2U);
    lsc16_on_tx_complete_isr(&device);
    lsc16_service_process_once(&service);
    assert((log.count == 2U) && (log.ids[1] == 11U));

    puts("LSC16 service fake tests passed");
    return 0;
}
