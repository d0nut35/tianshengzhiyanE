/**
 * @file test_depot_line.c
 * @brief Exercise production depot calibration and its app-task failure gate.
 * @note Host fixtures do not validate physical sensor wiring or stopping distance.
 */
#include <assert.h>
#include <setjmp.h>
#include <stdio.h>
#include "../../../../lhy/01_App/app_main.c"

static uint32_t tick;
static uint32_t hz;
static uint32_t begin;
static unsigned reads;
static unsigned stops;
static unsigned moves;
static unsigned writes;
static unsigned homes;
static unsigned ready_ok;
static unsigned ready_fail;
static unsigned stop_sent;
static unsigned integration;
static float velocity;
static map_point_t pose;
static route_side_t side;
static jmp_buf done;
volatile mission_color_t g_mission_side;

typedef enum {
    NORMAL, INITIAL_HIGH, BOUNCE, TIMEOUT, SENSOR_FAIL, MOVE_FAIL,
    READ_FAIL, WRITE_FAIL, STOP_SEEK, STOP_RETURN, STOP_SETTLE, STOP_FAIL
} scenario_t;
static scenario_t scenario;

uint32_t osKernelGetTickCount(void) { return tick; }
uint32_t osKernelGetTickFreq(void) { return hz; }
osStatus_t osDelay(uint32_t ticks)
{
    tick += ticks;
    if ((writes != 0U) && (velocity != 0.0f)) {
        pose.y_mm++;
    }
    if (integration && ticks == APP_HOME_WAIT_MS) { longjmp(done, 1); }
    return osOK;
}
osThreadId_t osThreadNew(osThreadFunc_t fn, void *arg, const osThreadAttr_t *attr)
{
    (void)fn; (void)arg; (void)attr;
    return (osThreadId_t)&tick;
}
route_side_t route_side(void) { return side; }
float route_lat_sign(void) { return side == ROUTE_SIDE_MIRROR ? -1.0f : 1.0f; }
int16_t route_side_x(int16_t x) { return x; }
app_status_t route_set_side(route_side_t value) { side = value; return APP_OK; }
csvc_status_t csvc_free(float vx, float vy, float wz)
{
    assert(vx == 0.0f && wz == 0.0f);
    if (scenario == MOVE_FAIL) { return CSVC_ERR; }
    assert(!stop_sent);
    assert(vy == (writes ? -100.0f : 100.0f) * route_lat_sign());
    velocity = vy;
    moves++;
    return CSVC_OK;
}
align_status_t align_stop(void)
{
    velocity = 0.0f;
    stops++;
    tick += util_ms_ticks(200U);
    return scenario == STOP_FAIL ? ALIGN_ERR : ALIGN_OK;
}
align_status_t align_on_line(uint8_t id, uint8_t *on_line)
{
    static const uint8_t bounce[] = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0};
    assert(id == (side == ROUTE_SIDE_MIRROR ? 1U : 6U));
    if (scenario == SENSOR_FAIL) { return ALIGN_ERR; }
    if (scenario == BOUNCE) {
        assert(reads < sizeof(bounce));
        *on_line = bounce[reads];
    } else {
        *on_line = scenario == TIMEOUT || (scenario != INITIAL_HIGH && reads < 2U);
    }
    reads++;
    return ALIGN_OK;
}
csvc_status_t csvc_get_pose(map_point_t *pos, float *yaw)
{
    if (scenario == READ_FAIL) { return CSVC_ERR; }
    *pos = pose;
    *yaw = 179.5f;
    return CSVC_OK;
}
csvc_status_t csvc_set_pose(map_point_t pos, float yaw)
{
    if (!reads) { return CSVC_OK; } /* Boot calibration in app_task. */
    assert(stops > 0U && velocity == 0.0f);
    assert(pos.x_mm == 403 && pos.y_mm == 2144 && yaw == 179.5f);
    if (scenario == WRITE_FAIL) { return CSVC_ERR; }
    writes++;
    pose = pos;
    return CSVC_OK;
}
uint8_t link_poll(link_hook_t hook, void *ctx, uint32_t tmo)
{
    chassis_mission_command_t cmd = {0};
    if (integration && hook == NULL && tmo != 0U) { longjmp(done, 1); }
    if (integration && hook == depot_hook) {
        cmd.type = MISSION_CMD_DEPOT_OK;
        cmd.request_id = 17U;
        return hook(&cmd, ctx);
    }
    if (!stop_sent &&
        ((scenario == STOP_SEEK && reads >= 2U) ||
         (scenario == STOP_RETURN && writes && moves >= 2U) ||
         (scenario == STOP_SETTLE && stops))) {
        cmd.type = MISSION_CMD_STOP;
        assert(hook != NULL && hook(&cmd, ctx) == 0U);
        stop_sent = 1U;
        (void)align_stop();
        return 1U;
    }
    return 0U;
}
app_status_t link_post(chassis_command_type_t type, uint16_t id, uint8_t ok)
{
    (void)id;
    if (type == CHASSIS_CMD_DEPOT_1_READY) {
        if (ok) {
            assert(writes == 1U && pose.y_mm >= 2208 && velocity == 0.0f);
            ready_ok++;
        } else { ready_fail++; }
    }
    return APP_OK;
}
app_status_t link_wait(mission_command_type_t type, uint16_t *id, uint32_t tmo)
{
    (void)tmo;
    *id = 17U;
    return type == MISSION_CMD_GO_DEPOT_1 ? APP_OK : APP_ERR;
}
void link_handshake(void) {}
app_status_t route_go(route_id_t id)
{
    if (id == ROUTE_HOME) { homes++; }
    return APP_OK;
}
app_status_t stairs_sweep(uint16_t id) { (void)id; return APP_OK; }
csvc_status_t csvc_arc(float v, float r, bool insitu)
{ (void)v; (void)r; (void)insitu; return CSVC_OK; }
csvc_status_t csvc_init(void) { return CSVC_OK; }
hwt101_status_t hwt101_adp_boot_cfg(void) { return HWT101_OK; }
zdt_status_t system_assembly_init(void) { return ZDT_OK; }

static void reset_case(scenario_t value, route_side_t field, uint32_t frequency)
{
    scenario = value;
    side = field;
    hz = frequency;
    tick = UINT32_MAX - 50U; /* Every case crosses the tick wrap. */
    begin = tick;
    reads = stops = moves = writes = homes = ready_ok = ready_fail = 0U;
    stop_sent = integration = 0U;
    velocity = 0.0f;
    pose.x_mm = 403;
    pose.y_mm = 2190;
    g_app_up = 1U;
    g_mission_side = side == ROUTE_SIDE_MIRROR ? MISSION_COLOR_BLUE : MISSION_COLOR_RED;
}

int main(void)
{
    volatile unsigned field;
    unsigned rate;
    volatile unsigned item;
    volatile unsigned count = 0U;
    const uint32_t rates[] = {1000U, 100U};
    for (field = 0U; field < 2U; field++) {
        for (rate = 0U; rate < 2U; rate++) {
            for (item = NORMAL; item <= STOP_FAIL; item++) {
                reset_case((scenario_t)item, (route_side_t)field, rates[rate]);
                if (item <= BOUNCE) {
                    assert(depot_locate() == APP_OK);
                    assert(writes == 1U && pose.y_mm >= 2208);
                    assert(reads == (item == BOUNCE ? 11U : item == INITIAL_HIGH ? 5U : 7U));
                    assert(moves == (item == INITIAL_HIGH ? 1U : 2U));
                } else {
                    assert(depot_locate() == APP_ERR);
                    if (item != STOP_RETURN) { assert(writes == 0U); }
                    if (item == TIMEOUT) {
                        assert(tick - begin == util_ms_ticks(20200U));
                    }
                }
                assert(velocity == 0.0f && stops > 0U);
                count++;
            }
        }
        for (item = NORMAL; item <= STOP_FAIL; item++) {
            reset_case((scenario_t)item, (route_side_t)field, 1000U);
            integration = 1U;
            if (setjmp(done) == 0) { app_task(NULL); assert(0); }
            if (scenario <= BOUNCE) {
                assert(homes == 1U && ready_ok == 1U && ready_fail == 0U);
            } else {
                assert(homes == 0U && ready_ok == 0U && ready_fail == 1U);
            }
            count++;
        }
    }
    printf("Depot line: %u host cases passed (red/blue, debounce, timeout, STOP, failures, app gate).\n", count);
    return 0;
}
