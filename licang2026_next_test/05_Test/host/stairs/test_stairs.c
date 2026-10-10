/**
 * @file test_stairs.c
 * @brief Compile the actual stair loop and sensor publisher with host fixtures.
 * @note Host assertions do not validate motor wiring or real line geometry.
 */
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include "../../../../lhy/04_Bsp/lsensor/lsensor_handler.c"
#include "../../../../lhy/01_App/app_stairs.c"

test_gpio_t test_gpio;
static uint32_t now_tick;
static uint32_t tick_hz = 1000U;
static float cmd_vy;
static float cmd_wz;
static unsigned drives;
static unsigned stops;
static unsigned events;
static unsigned finished;
static unsigned step;
static unsigned polls;
static int pose_y;
static uint8_t end_on;
static uint8_t fail_send;
static uint8_t fail_pose;
static uint8_t fail_line;
static route_side_t side;
static stair_ctx_t *active_ctx;

typedef enum {
    RUN_OK, STOP_WAIT, STOP_MOVE, STOP_SETTLE, LOST_START,
    BAD_SENSOR, BAD_SEND, BAD_POSE, BAD_END
} test_case_t;
static test_case_t run_case;

/* Publish through the production IDR decoder, never assign the cached mask. */
static void sample_mask(uint8_t mask)
{
    uint8_t i; /* Sensor table index. */
    test_gpio.IDR = 0U;
    for (i = 0U; i < LSENSOR_COUNT; i++) {
        if ((mask & (1U << i)) != 0U) {
            test_gpio.IDR |= s_cfg[i].pin;
        }
    }
    lsh_sample();
}

uint32_t osKernelGetTickCount(void) { return now_tick; }
uint32_t osKernelGetTickFreq(void) { return tick_hz; }
osStatus_t osDelayUntil(uint32_t ticks) { now_tick = ticks; return osOK; }
osThreadId_t osThreadNew(osThreadFunc_t fn, void *arg,
                        const osThreadAttr_t *attr)
{
    (void)fn; (void)arg; (void)attr;
    return (osThreadId_t)&test_gpio;
}
route_side_t route_side(void) { return side; }
float route_lat_sign(void)
{
    return (side == ROUTE_SIDE_MIRROR) ? -1.0f : 1.0f;
}

csvc_status_t csvc_free(float vx, float vy, float wz)
{
    assert(vx == 0.0f);
    if (fail_send != 0U) {
        fail_send = 0U;
        return CSVC_ERR;
    }
    if (active_ctx != NULL) {
        assert(active_ctx->cam_ready != 0U);
        assert(active_ctx->paused == 0U);
        assert(active_ctx->settling == 0U);
        assert(active_ctx->stopped == 0U);
        assert(vy == route_lat_sign() * ST_VY_MMS);
    }
    assert(fabsf(wz) <= ST_W_MAX);
    cmd_vy = vy;
    cmd_wz = wz;
    drives++;
    return CSVC_OK;
}

align_status_t align_stop(void)
{
    cmd_vy = 0.0f;
    cmd_wz = 0.0f;
    now_tick += util_ms_ticks(200U);
    stops++;
    return ALIGN_OK;
}

csvc_status_t csvc_get_pose(map_point_t *pos, float *yaw)
{
    if (fail_pose != 0U) { return CSVC_ERR; }
    pos->x_mm = 0;
    pos->y_mm = (int16_t)pose_y;
    *yaw = 0.0f;
    return CSVC_OK;
}

align_status_t align_on_line(uint8_t id, uint8_t *on_line)
{
    assert(id == ((side == ROUTE_SIDE_MIRROR) ? 6U : 1U));
    if (fail_line != 0U) { return ALIGN_ERR; }
    *on_line = end_on;
    return ALIGN_OK;
}

app_status_t link_post(chassis_command_type_t type, uint16_t id, uint8_t ok)
{
    assert(id == 17U);
    assert(ok == 1U);
    if (type == CHASSIS_CMD_STAIR_LOW || type == CHASSIS_CMD_STAIR_HIGH ||
        type == CHASSIS_CMD_STAIR_MID) {
        events++;
        step = 0U;
        end_on = 1U;
        pose_y = (type == CHASSIS_CMD_STAIR_LOW) ? 3000 : 2600;
    } else if (type == CHASSIS_CMD_STAIRS_FINISHED) {
        assert(cmd_vy == 0.0f && cmd_wz == 0.0f);
        finished++;
    }
    return APP_OK;
}

/* Use the real hook; the default STOP fixture performs the documented stop. */
static void dispatch(link_hook_t hook, void *ctx, mission_command_type_t type)
{
    chassis_mission_command_t cmd = {0}; /* Fake inbound command. */
    cmd.type = type;
    cmd.request_id = 17U;
    if (hook(&cmd, ctx) == 0U) {
        assert(type == MISSION_CMD_STOP);
        (void)align_stop();
    }
}

uint8_t link_poll(link_hook_t hook, void *ctx, uint32_t tmo)
{
    stair_ctx_t *sc = ctx; /* Actual production context during this poll. */
    assert(++polls < 500U);
    active_ctx = sc;
    now_tick += (tmo == osWaitForever) ? 1U : tmo;
    if (sc->cam_ready == 0U) {
        assert(cmd_vy == 0.0f && cmd_wz == 0.0f);
        dispatch(hook, ctx, (run_case == STOP_WAIT) ?
                 MISSION_CMD_STOP : MISSION_CMD_CAM_READY);
        return 1U;
    }
    if (sc->settling != 0U) {
        assert(cmd_vy == 0.0f && cmd_wz == 0.0f);
        if (run_case == STOP_SETTLE) {
            dispatch(hook, ctx, MISSION_CMD_STOP);
        }
        return 0U;
    }
    step++;
    if (step == 1U) {
        sample_mask((run_case == LOST_START) ? 0x1EU : 0x0CU);
        fail_pose = (run_case == BAD_POSE);
        fail_line = (run_case == BAD_END && events == 3U);
    } else if (step == 2U) {
        sample_mask(0x0EU); /* 2 leaves line: clockwise correction. */
        s_ready = (run_case == BAD_SENSOR) ? 0U : 1U;
        fail_send = (run_case == BAD_SEND);
        if (run_case == STOP_MOVE) { dispatch(hook, ctx, MISSION_CMD_STOP); }
    } else if (step == 3U) {
        assert(cmd_wz == -1.5f);
        dispatch(hook, ctx, MISSION_CMD_STAIR_STOP);
    } else if (step == 4U) {
        assert(sc->paused != 0U);
        assert(cmd_vy == 0.0f && cmd_wz == 0.0f);
    } else if (step == 5U) {
        sample_mask(0x04U); /* 4 enters line: counterclockwise on resume. */
        dispatch(hook, ctx, MISSION_CMD_STAIR_RESUME);
        assert(cmd_vy == 0.0f && cmd_wz == 0.0f);
    } else if (step == 6U) {
        assert(cmd_wz == 1.5f);
        sample_mask(0x1EU); /* All lost: preserve last nonzero correction. */
    } else if (step == 7U) {
        assert(cmd_wz == 1.5f);
        sample_mask(0x0CU); /* Reacquisition zeros rotation, not history. */
    } else if (step == 8U) {
        assert(step == 8U && cmd_wz == 0.0f);
        pose_y = (events == 1U) ? 2700 : 2300;
        end_on = 0U;
        sample_mask(0x1EU); /* Layer boundary must take precedence over loss. */
    } else {
        assert(events == 3U && step <= 12U);
        assert(cmd_wz == 1.5f); /* Keep driving until the fifth high sample. */
    }
    return 0U;
}

static void test_snapshot(void)
{
    uint8_t mask; /* All possible six-sensor samples. */
    uint8_t i;    /* Sensor identifier. */
    assert(lsh_get_mask() == LSH_MASK_INVALID);
    assert(lsh_init() == LSH_OK);
    for (mask = 0U; mask < 64U; mask++) {
        sample_mask(mask);
        assert(lsh_get_mask() == mask);
        test_gpio.IDR ^= 0xFFFFU;
        assert(lsh_get_mask() == mask); /* GPIO change is invisible until publish. */
        for (i = 1U; i <= LSENSOR_COUNT; i++) {
            assert(lsh_get_level((lsensor_id_t)i) == ((mask >> (i - 1U)) & 1U));
        }
    }
}

static void test_control(void)
{
    static const float expected[15] = { /* Independently enumerated truth table. */
        0.0f, -1.5f, 1.5f, 0.0f, -1.5f, -3.0f, 0.0f, -1.5f,
        1.5f, 0.0f, 3.0f, 1.5f, 0.0f, -1.5f, 1.5f
    };
    stair_ctx_t sc = {0}; /* Isolated controller history. */
    uint8_t mask;         /* Four-bit pattern ordered 2,3,4,5. */
    unsigned before;      /* Failed reads must not issue a drive command. */
    for (side = ROUTE_SIDE_DEFAULT; side <= ROUTE_SIDE_MIRROR; side++) {
        sc.vy_mms = route_lat_sign() * ST_VY_MMS;
        for (mask = 0U; mask < 15U; mask++) {
            sample_mask((uint8_t)(mask << 1U));
            assert(stair_drive(&sc) == APP_OK);
            assert(cmd_wz == expected[mask] && cmd_vy == sc.vy_mms);
        }
    }
    sample_mask(0x02U); /* Original user example: only 2 is high. */
    assert(stair_drive(&sc) == APP_OK && cmd_wz < 0.0f);
    sample_mask(0x0CU);
    assert(stair_drive(&sc) == APP_OK && cmd_wz == 0.0f);
    sample_mask(0x1EU);
    assert(stair_drive(&sc) == APP_OK && cmd_wz == -1.5f);
    sc.last_wz = 0.0f;
    before = drives;
    assert(stair_drive(&sc) == APP_ERR && drives == before);
    s_ready = 0U;
    assert(stair_drive(&sc) == APP_ERR && drives == before);
    s_ready = 1U;
    sample_mask(0x02U);
    fail_send = 1U;
    assert(stair_drive(&sc) == APP_ERR && sc.last_wz == 0.0f);
}

static void test_sweep(test_case_t which, route_side_t selected, uint32_t hz)
{
    app_status_t ret; /* Production state-machine completion status. */
    run_case = which;
    side = selected;
    tick_hz = hz;
    now_tick = UINT32_MAX - 30U; /* Include unsigned tick wraparound. */
    cmd_vy = cmd_wz = 0.0f;
    drives = stops = events = finished = step = polls = 0U;
    end_on = s_ready = 1U;
    fail_pose = fail_send = fail_line = 0U;
    active_ctx = NULL;
    sample_mask(0x0CU);
    ret = stairs_sweep(17U);
    active_ctx = NULL;
    assert(cmd_vy == 0.0f && cmd_wz == 0.0f);
    assert(stops > 0U);
    if (which == RUN_OK) {
        assert(ret == APP_OK && events == 3U && finished == 1U);
        assert(drives == 19U); /* Four more end-line confirmation periods. */
    } else {
        assert(ret == APP_ERR && finished == 0U);
    }
}

static void test_end_filter(void)
{
    stair_ctx_t sc = {0}; /* Counter belongs to this sweep, not static state. */
    uint8_t done;         /* Actual production boundary result. */
    unsigned i;          /* Consecutive sample index. */
    chassis_mission_command_t cmd = {0}; /* Pause resets the confirmation. */

    for (side = ROUTE_SIDE_DEFAULT; side <= ROUTE_SIDE_MIRROR; side++) {
        sc.end_id = side == ROUTE_SIDE_MIRROR ? 6U : 1U;
        sc.req_id = 17U;
        sc.high_cnt = 0U;
        fail_line = 0U;
        end_on = 0U;
        for (i = 1U; i <= 4U; i++) {
            assert(layer_done(&g_layers[2], &sc, &done) == APP_OK);
            assert(!done && sc.high_cnt == i);
        }
        end_on = 1U;
        assert(layer_done(&g_layers[2], &sc, &done) == APP_OK);
        assert(!done && sc.high_cnt == 0U);
        end_on = 0U;
        for (i = 1U; i <= 5U; i++) {
            assert(layer_done(&g_layers[2], &sc, &done) == APP_OK);
            assert(done == (i == 5U));
        }
        assert(layer_done(&g_layers[2], &sc, &done) == APP_OK);
        assert(done && sc.high_cnt == 5U); /* Saturate instead of overflow. */
        cmd.type = MISSION_CMD_STAIR_STOP;
        assert(stair_hook(&cmd, &sc) == 1U && sc.high_cnt == 0U);
        for (i = 1U; i <= 5U; i++) {
            assert(layer_done(&g_layers[2], &sc, &done) == APP_OK);
            assert(done == (i == 5U));
        }
        fail_line = 1U;
        assert(layer_done(&g_layers[2], &sc, &done) == APP_ERR);
        fail_line = 0U;
    }
}

int main(void)
{
    test_case_t which; /* Failure and state-transition scenarios. */
    test_snapshot();
    test_control();
    test_end_filter();
    for (which = RUN_OK; which <= BAD_END; which++) {
        test_sweep(which, ROUTE_SIDE_DEFAULT, 1000U);
        test_sweep(which, ROUTE_SIDE_MIRROR, 1000U);
    }
    test_sweep(RUN_OK, ROUTE_SIDE_DEFAULT, 100U);
    test_sweep(RUN_OK, ROUTE_SIDE_MIRROR, 100U);
    puts("PASS: 64 snapshots, 16 patterns, both sides, loss/history, pause/resume,");
    puts("      layer boundaries, STOP in all waits, IO failures, tick rates/wrap.");
    puts("      End-line five highs, low reset, pause reset, saturation, both sides.");
    return 0;
}
