/** @file zdt_turntable_service.c @brief ZDT复用事务和电机语义实现。 */
#include "zdt_turntable_service.h"
#include <stdbool.h>
#include <string.h>
#include "mux_service.h"

typedef struct {
    bool initialized;
    bool active;
    bool firmware_known;
    bool closed_loop;
    bool scaled_input;
    zdt_turntable_firmware_t firmware;
    turn_config_t config;
    uint32_t next_id;
    uint32_t request_id;
    uint8_t expected_function;
    turn_done_fn_t done_cb;
    void *done_ctx;
} turn_state_t;
static turn_state_t g_turn;

/** 把复用器结果转换为转盘状态。 */
static zdt_turntable_status_t turn_map(mult_uart_status_t s)
{
    switch (s) {
    case MULT_UART_OK:
        return ZDT_TURNTABLE_OK;
    case MULT_UART_ERR_BUSY:
        return ZDT_TURNTABLE_ERR_BUSY;
    case MULT_UART_ERR_QUEUE_FULL:
        return ZDT_TURNTABLE_ERR_QUEUE_FULL;
    case MULT_UART_ERR_TIMEOUT:
        return ZDT_TURNTABLE_ERR_TIMEOUT;
    case MULT_UART_ERR_PARAM:
    case MULT_UART_ERR_OVERFLOW:
        return ZDT_TURNTABLE_ERR_PARAM;
    default:
        return ZDT_TURNTABLE_ERR_IO;
    }
}

/** 生成非零、单调递增的业务请求ID。 */
static uint32_t turn_id(void)
{
    if (++g_turn.next_id == 0U) {
        ++g_turn.next_id;
    }
    return g_turn.next_id;
}

/** 在mux worker上下文解析响应并回调业务层。 */
static void turn_mux_done(void *ctx, const mux_completion_t *completion)
{
    zdt_turntable_response_t response;
    zdt_turntable_status_t status;
    turn_done_fn_t cb = g_turn.done_cb;
    void *user = g_turn.done_ctx;
    uint32_t id = g_turn.request_id;

    (void)ctx;
    if (!g_turn.active || (completion == NULL)) {
        return;
    }
    g_turn.active = false;
    status = turn_map(completion->status);
    if (status == ZDT_TURNTABLE_OK) {
        status = turn_parse(completion->rx_data, completion->rx_len,
            g_turn.config.address, g_turn.expected_function, &response);
    }
    if (g_turn.expected_function == 0x1AU) {
        if ((status == ZDT_TURNTABLE_OK) &&
            (response.kind == ZDT_TURNTABLE_REPLY_OPTIONS)) {
            g_turn.firmware = response.data.options.firmware;
            g_turn.firmware_known = true;
            g_turn.closed_loop = response.data.options.closed_loop;
            g_turn.scaled_input = response.data.options.scaled_input;
        }
    }
    if (cb != NULL) {
        cb(user, id, status,
           ((status == ZDT_TURNTABLE_OK) ||
            (status == ZDT_TURNTABLE_ERR_DEVICE)) ? &response : NULL);
    }
}

/** 封装固定复用通道2的单笔写读事务。 */
static zdt_turntable_status_t turn_submit(
    uint8_t function,
    const uint8_t *frame,
    size_t len,
    turn_done_fn_t cb,
    void *ctx)
{
    mux_transfer_t transfer = {0};
    mult_uart_status_t status;
    if (!g_turn.initialized) {
        return ZDT_TURNTABLE_ERR_NOT_INIT;
    }
    if ((frame == NULL) || (len == 0U) || (len > ZDT_TURNTABLE_FRAME_MAX)) {
        return ZDT_TURNTABLE_ERR_PARAM;
    }
    if (g_turn.active) {
        return ZDT_TURNTABLE_ERR_BUSY;
    }
    g_turn.request_id = turn_id();
    g_turn.expected_function = function;
    g_turn.done_cb = cb;
    g_turn.done_ctx = ctx;
    g_turn.active = true;
    transfer.device = MUX_DEVICE_2;
    transfer.operation = MULT_UART_OP_WRITE_READ;
    transfer.tx_data = frame;
    transfer.tx_len = len;
    transfer.rx_capacity = ZDT_TURNTABLE_RESPONSE_MAX;
    transfer.io_timeout_ms = g_turn.config.timeout_ms;
    transfer.done_cb = turn_mux_done;
    status = mux_submit(&transfer);
    if (status != MULT_UART_OK) {
        g_turn.active = false;
    }
    return turn_map(status);
}

zdt_turntable_status_t turn_init(const turn_config_t *config)
{
    if ((config == NULL) || (config->address == 0U) ||
        (config->timeout_ms == 0U) ||
        (config->emm_pulses_per_revolution == 0U)) {
        return ZDT_TURNTABLE_ERR_PARAM;
    }
    if (g_turn.initialized) {
        return ZDT_TURNTABLE_ERR_STATE;
    }
    (void)memset(&g_turn, 0, sizeof(g_turn));
    g_turn.config = *config;
    g_turn.initialized = true;
    return ZDT_TURNTABLE_OK;
}
zdt_turntable_status_t turn_query_options(
    turn_done_fn_t cb,
    void *ctx)
{
    uint8_t frame[3];
    size_t len;
    zdt_turntable_status_t s;

    if (!g_turn.initialized) {
        return ZDT_TURNTABLE_ERR_NOT_INIT;
    }
    s = turn_options_frame(g_turn.config.address, frame, sizeof(frame), &len);
    if (s != ZDT_TURNTABLE_OK) {
        return s;
    }
    s = turn_submit(0x1AU, frame, len, cb, ctx);
    return s;
}
zdt_turntable_status_t turn_query_status(
    turn_done_fn_t cb,
    void *ctx)
{
    uint8_t frame[3];
    size_t len;
    zdt_turntable_status_t s;

    if (!g_turn.initialized) {
        return ZDT_TURNTABLE_ERR_NOT_INIT;
    }
    s = turn_status_frame(g_turn.config.address, frame, sizeof(frame), &len);
    return (s == ZDT_TURNTABLE_OK) ?
        turn_submit(0x3AU, frame, len, cb, ctx) : s;
}

zdt_turntable_status_t turn_move_emm(
    const zdt_turntable_position_command_t *cmd,
    turn_done_fn_t cb,
    void *ctx)
{
    uint8_t frame[ZDT_TURNTABLE_FRAME_MAX];
    size_t len;
    uint64_t numerator;
    uint64_t pulses;
    zdt_turntable_position_command_t adjusted;
    zdt_turntable_status_t s;

    if (!g_turn.initialized) {
        return ZDT_TURNTABLE_ERR_NOT_INIT;
    }
    if ((cmd == NULL) || !g_turn.firmware_known || !g_turn.closed_loop ||
        (g_turn.firmware != ZDT_TURNTABLE_FIRMWARE_EMM)) {
        return (cmd == NULL) ?
            ZDT_TURNTABLE_ERR_PARAM : ZDT_TURNTABLE_ERR_STATE;
    }
    adjusted = *cmd;
    if (g_turn.scaled_input) {
        if (adjusted.speed > 300U) {
            return ZDT_TURNTABLE_ERR_PARAM;
        }
        adjusted.speed = (uint16_t)(adjusted.speed * 10U);
    }
    numerator = (uint64_t)cmd->angle_0p1deg *
                g_turn.config.emm_pulses_per_revolution;
    pulses = numerator / 3600U;
    if ((numerator % 3600U) >= 1800U) {
        ++pulses;
    }
    if (pulses > UINT32_MAX) {
        return ZDT_TURNTABLE_ERR_PARAM;
    }
    s = turn_emm_frame(g_turn.config.address, &adjusted, (uint32_t)pulses,
        frame, sizeof(frame), &len);
    return (s == ZDT_TURNTABLE_OK) ?
        turn_submit(0xFDU, frame, len, cb, ctx) : s;
}

zdt_turntable_status_t turn_stop(turn_done_fn_t cb, void *ctx)
{
    uint8_t frame[5];
    size_t len;
    zdt_turntable_status_t s;

    if (!g_turn.initialized) {
        return ZDT_TURNTABLE_ERR_NOT_INIT;
    }
    s = turn_stop_frame(g_turn.config.address, frame, sizeof(frame), &len);
    return (s == ZDT_TURNTABLE_OK) ?
        turn_submit(0xFEU, frame, len, cb, ctx) : s;
}
#if !LICANG_RELEASE_MINIMAL
zdt_turntable_status_t turn_query(uint8_t f, turn_done_fn_t cb, void *ctx)
{
    uint8_t frame[3]; size_t len; zdt_turntable_status_t s;
    if (!g_turn.initialized) return ZDT_TURNTABLE_ERR_NOT_INIT;
    if (f == 0x1FU) s = turn_version_frame(g_turn.config.address, frame, sizeof(frame), &len);
    else if (f == 0x3AU) s = turn_status_frame(g_turn.config.address, frame, sizeof(frame), &len);
    else if (f == 0x36U) s = turn_position_frame(g_turn.config.address, frame, sizeof(frame), &len);
    else return ZDT_TURNTABLE_ERR_UNSUPPORTED;
    return (s == ZDT_TURNTABLE_OK) ? turn_submit(f, frame, len, cb, ctx) : s;
}
zdt_turntable_status_t turn_move(const zdt_turntable_position_command_t *cmd,
    turn_done_fn_t cb, void *ctx)
{
    uint8_t frame[ZDT_TURNTABLE_FRAME_MAX];
    size_t len;
    zdt_turntable_position_command_t adjusted;
    zdt_turntable_status_t s;

    if ((cmd == NULL) || !g_turn.initialized || !g_turn.firmware_known || !g_turn.closed_loop)
        return (cmd == NULL) ? ZDT_TURNTABLE_ERR_PARAM : ZDT_TURNTABLE_ERR_STATE;
    if (g_turn.firmware == ZDT_TURNTABLE_FIRMWARE_EMM) return turn_move_emm(cmd, cb, ctx);
    adjusted = *cmd;
    if (g_turn.scaled_input) {
        if (adjusted.angle_0p1deg > (UINT32_MAX / 10U)) {
            return ZDT_TURNTABLE_ERR_PARAM;
        }
        adjusted.angle_0p1deg *= 10U;
    }
    s = turn_x_frame(g_turn.config.address, &adjusted,
        frame, sizeof(frame), &len);
    return (s == ZDT_TURNTABLE_OK) ? turn_submit(0xFDU, frame, len, cb, ctx) : s;
}
#endif
