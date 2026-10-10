/**
 * @file test_depot_line.c
 * @brief Exercise production depot calibration and its app-task failure gate.
 * @note Host fixtures do not validate physical sensor wiring or stopping distance.
 */
#include <assert.h>
#include <math.h>
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
static float angular;
static float expect_vy;
static uint8_t gray_mask;
static unsigned gray_reads;
static unsigned gray_fail_at;
static unsigned send_fail_at;
static unsigned pose_fail_at;
static unsigned stop_at;
static uint8_t shift_test;
static uint8_t gray_script;
static int shift_dir;
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
    if (shift_test && velocity != 0.0f) {
        pose.y_mm += (int16_t)shift_dir;
    } else if ((writes != 0U) && (velocity != 0.0f)) {
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
    assert(vx == 0.0f && fabsf(wz) <= APP_DEPOT_W_MAX);
    if (scenario == MOVE_FAIL || (send_fail_at && moves >= send_fail_at)) {
        return CSVC_ERR;
    }
    assert(!stop_sent);
    if (shift_test) {
        assert(vy == expect_vy);
        if (gray_script) {
            assert(wz == ((gray_reads == 2U || gray_reads == 3U) ? -1.5f : 0.0f));
        }
    } else {
        assert(wz == 0.0f);
        assert(vy == (writes ? -100.0f : 100.0f) * route_lat_sign());
    }
    velocity = vy;
    angular = wz;
    moves++;
    return CSVC_OK;
}
align_status_t align_stop(void)
{
    velocity = 0.0f;
    angular = 0.0f;
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
    if (scenario == READ_FAIL || (pose_fail_at && moves >= pose_fail_at)) {
        return CSVC_ERR;
    }
    *pos = pose;
    *yaw = 179.5f;
    return CSVC_OK;
}
/* 线尾校准返程从全高开始；控制测试可注入压线和全丢线序列。 */
uint8_t lsh_get_mask(void)
{
    gray_reads++;
    if (gray_fail_at && gray_reads >= gray_fail_at) {
        return LSH_MASK_INVALID;
    }
    if (gray_script) {
        if (gray_reads == 1U || gray_reads == 3U) { return 0x1EU; }
        if (gray_reads == 2U) { return 0x02U; }
        return 0x0CU;
    }
    return gray_mask;
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
         (scenario == STOP_SETTLE && stops) ||
         (shift_test && stop_at && moves >= stop_at))) {
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
    angular = 0.0f;
    shift_test = gray_script = 0U;
    gray_mask = 0x1EU;
    gray_reads = gray_fail_at = send_fail_at = pose_fail_at = stop_at = 0U;
    pose.x_mm = 403;
    pose.y_mm = 2190;
    g_app_up = 1U;
    g_mission_side = side == ROUTE_SIDE_MIRROR ? MISSION_COLOR_BLUE : MISSION_COLOR_RED;
}

/** @brief 仓库双向运行逐项验证电平组合、纠偏记忆和下发失败不污染历史。 */
static void test_gray(void)
{
    static const float expect[16] = { /* 四位按 2/3/4/5 从低位到高位。 */
        0.0f, -1.5f, 1.5f, 0.0f, -1.5f, -3.0f, 0.0f, -1.5f,
        1.5f, 0.0f, 3.0f, 1.5f, 0.0f, -1.5f, 1.5f, 0.0f
    };
    unsigned mask;      /* 四路组合索引。 */
    unsigned field;     /* 红蓝侧。 */
    int direction;      /* 两个平移方向。 */
    float last;         /* 当前点位横移的纠偏历史。 */
    unsigned before;    /* 无效读取不能下发新的速度。 */
    for (field = 0U; field < 2U; field++) {
        for (direction = -1; direction <= 1; direction += 2) {
            reset_case(NORMAL, (route_side_t)field, 1000U);
            shift_test = 1U;
            expect_vy = (float)direction * route_lat_sign() * APP_DEPOT_VY_MMS;
            for (mask = 0U; mask < 16U; mask++) {
                last = 0.0f;
                gray_mask = (uint8_t)((mask << 1U) | 0x21U);
                assert(depot_drive(expect_vy, &last) == APP_OK);
                assert(angular == expect[mask] && velocity == expect_vy);
            }
            gray_mask = 0x02U;
            assert(depot_drive(expect_vy, &last) == APP_OK && angular == -1.5f);
            gray_mask = 0x0CU;
            assert(depot_drive(expect_vy, &last) == APP_OK && angular == 0.0f);
            gray_mask = 0x1EU;
            assert(depot_drive(expect_vy, &last) == APP_OK && angular == -1.5f);
            gray_mask = 0x10U;
            assert(depot_drive(expect_vy, &last) == APP_OK && angular == 1.5f);
            gray_mask = 0x1EU;
            assert(depot_drive(expect_vy, &last) == APP_OK && angular == 1.5f);
            before = moves;
            gray_mask = LSH_MASK_INVALID;
            assert(depot_drive(expect_vy, &last) == APP_ERR && moves == before);
            assert(depot_drive(expect_vy, NULL) == APP_ERR && moves == before);
            gray_mask = 0x02U;
            scenario = MOVE_FAIL;
            assert(depot_drive(expect_vy, &last) == APP_ERR && last == 1.5f);
        }
    }
}

/** @brief 运行真实 depot_shift，覆盖全高起步、全高持续和中途错误/STOP。 */
static void test_shift(unsigned fault, route_side_t field, int dir, uint32_t rate)
{
    app_status_t result; /* 横移最终状态。 */
    int16_t target;      /* 本次模拟目标里程计 Y。 */
    reset_case(NORMAL, field, rate);
    shift_test = gray_script = 1U;
    shift_dir = dir;
    expect_vy = (float)dir * route_lat_sign() * APP_DEPOT_VY_MMS;
    pose.y_mm = 2400;
    target = (int16_t)(2400 + dir * 65);
    if (fault == 1U) { gray_fail_at = 3U; }
    if (fault == 2U) { send_fail_at = 2U; }
    if (fault == 3U) { pose_fail_at = 2U; }
    if (fault == 4U) { stop_at = 2U; }
    if (fault == 5U) { gray_script = 0U; } /* 全程全高也不因电平停车。 */
    if (fault == 6U) { target = pose.y_mm; }
    if (fault == 7U) { gray_fail_at = 1U; }
    if (fault == 8U) { scenario = READ_FAIL; }
    result = depot_shift(target);
    if (fault == 0U || fault == 5U || fault == 6U) {
        assert(result == APP_OK);
        assert(dir > 0 ? pose.y_mm >= target : pose.y_mm <= target);
        if (fault == 6U) { assert(moves == 0U && gray_reads == 0U); }
        else { assert(moves >= 3U && stops == 1U); }
    } else {
        assert(result == APP_ERR && stops > 0U);
        assert(dir > 0 ? pose.y_mm < target : pose.y_mm > target);
    }
    assert(velocity == 0.0f && angular == 0.0f);
    if (fault == 4U) { assert(stop_sent && moves == 2U); }
}

int main(void)
{
    volatile unsigned field;
    unsigned rate;
    volatile unsigned item;
    volatile unsigned count = 0U;
    int direction; /* 仓库点位往返方向。 */
    const uint32_t rates[] = {1000U, 100U};
    for (field = 0U; field < 2U; field++) {
        for (rate = 0U; rate < 2U; rate++) {
            for (item = NORMAL; item <= STOP_FAIL; item++) {
                reset_case((scenario_t)item, (route_side_t)field, rates[rate]);
                if (item <= BOUNCE) {
                    assert(depot_locate() == APP_OK);
                    assert(writes == 1U && pose.y_mm >= 2208);
                    assert(reads == (item == BOUNCE ? 11U : item == INITIAL_HIGH ? 5U : 7U));
                    assert(moves >= (item == INITIAL_HIGH ? 1U : 2U));
                    assert(gray_reads > 0U); /* 校准后返回 D1 已接入闭环。 */
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
    test_gray();
    for (field = 0U; field < 2U; field++) {
        for (rate = 0U; rate < 2U; rate++) {
            for (direction = -1; direction <= 1; direction += 2) {
                for (item = 0U; item < 9U; item++) {
                    test_shift(item, (route_side_t)field, direction, rates[rate]);
                }
            }
        }
    }
    puts("Depot P: 64 direction/pattern cases and 72 shift scenarios passed.");
    return 0;
}
