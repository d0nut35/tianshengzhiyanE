/* 实际命令/停车函数；设备替身仅记录调用及注入失败，不证明实机效果。 */
#include <assert.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include "mission_app.h"
#include "mission_config.h"
enum { MISSION_TEST_MODE_IDLE, MISSION_TEST_MODE_PATH, MISSION_TEST_MODE_TARGET };
enum { MISSION_TEST_STAGE_PLATFORM, MISSION_TEST_STAGE_DEPOT, MISSION_TEST_STAGE_DONE };
typedef unsigned mission_command_type_t;
typedef unsigned chassis_command_type_t;
enum { MISSION_CMD_GO_DEPOT_1 = 10, MISSION_CMD_DEPOT_OK = 14 };
enum { CHASSIS_CMD_DEPOT_1_READY = 20, CHASSIS_CMD_HOME_READY = 24, CHASSIS_CMD_STOPPED };
enum { MISSION_FAULT_CHASSIS = 7, DEBUG_UART1_RX_BUFFER_SIZE = 64 };
typedef struct { mission_state_t state; uint16_t request_id; } mission_context_t;
static struct { unsigned mode, expected, depot_position; bool stop_requested; char text[192]; } g_wireless_test;
volatile mission_color_t g_mission_side;
static uint32_t now;
static unsigned calls[32], count, fail_at, fail_pre, pauses, stop_pause, faults, stopped;
static char last_log[192];
static uint32_t osKernelGetTickCount(void) { return now; }
static unsigned mission_ms_to_ticks(unsigned ms) { if (ms == 500) ++pauses; return ms; }
static void osDelay(unsigned ms) { now += ms; }
static void mission_test_write(const char *s) { snprintf(last_log, sizeof(last_log), "%s", s); }
static const char *mission_test_color_name(mission_color_t c) { return c == MISSION_COLOR_BLUE ? "BLUE" : "RED"; }
static void mission_enter_state(mission_context_t *c, mission_state_t s, unsigned timeout)
{ (void)timeout; c->state = s; }
static void mission_fail(mission_context_t *c, unsigned f)
{ assert(f == MISSION_FAULT_CHASSIS); ++faults; c->state = MISSION_STATE_FAULT; }
static bool mission_test_take_command(char *s, unsigned size)
{ if (stop_pause && pauses == stop_pause && !g_wireless_test.stop_requested) { snprintf(s, size, "STOP"); return true; } return false; }
static bool mission_test_handle_aux_command(mission_context_t *c, const char *s)
{ assert(strcmp(s, "STOP") == 0); g_wireless_test.stop_requested = true; ++c->request_id; return true; }
static bool mission_test_wait_chassis_event(mission_context_t *c, unsigned evt, uint16_t id)
{ assert(evt == CHASSIS_CMD_STOPPED && id == c->request_id); ++stopped; c->state = MISSION_STATE_STOPPED; return true; }
static bool mission_test_skip_platform(mission_context_t *c)
{ (void)c; assert(count == 0); calls[count++] = 1; return fail_pre != 1; }
static bool mission_test_skip_stairs(mission_context_t *c)
{ (void)c; assert(count == 1); calls[count++] = 2; return fail_pre != 2; }
static bool mission_test_skip_small_disc(mission_context_t *c)
{ (void)c; assert(count == 2); calls[count++] = 3; return fail_pre != 3; }
static bool mission_test_send_wait(mission_context_t *c, unsigned cmd, unsigned evt, mission_state_t state)
{
    assert(cmd >= MISSION_CMD_GO_DEPOT_1 && cmd <= MISSION_CMD_DEPOT_OK);
    assert(evt == cmd + 10 && state == MISSION_STATE_WAIT_DEPOT_1 && count < 32);
    calls[count++] = cmd; ++c->request_id;
    return count != fail_at;
}
#include "depot_path_under_test.inc"
static void reset(mission_context_t *c, bool blue)
{
    memset(c, 0, sizeof(*c)); memset(&g_wireless_test, 0, sizeof(g_wireless_test));
    count = fail_at = fail_pre = pauses = stop_pause = faults = stopped = 0; now = 0;
    g_mission_side = blue ? MISSION_COLOR_BLUE : MISSION_COLOR_RED;
}
int main(void)
{
    const unsigned red[] = {1,2,3,10,11,12,13,12,11,10,13,12,11,10,13,12,11,10,14};
    const unsigned blue[] = {1,2,3,10,11,12,13,10,11,12,13,10,11,12,13,10,11,12,13,10,14};
    mission_context_t c;
    for (unsigned side = 0; side < 2; ++side) {
        const unsigned *expected = side ? blue : red;
        unsigned length = side ? sizeof(blue)/sizeof(*blue) : sizeof(red)/sizeof(*red);
        reset(&c, side);
        assert(!mission_test_depot_path_command(&c, "ROUTE DEPOT")); assert(count == 0);
        assert(mission_test_depot_path_command(&c, "ROUTE DEPOT PATH"));
        assert(count == length && memcmp(calls, expected, length * sizeof(*expected)) == 0);
        assert(pauses == length - 4 && now == pauses * 500);
        assert(c.state == MISSION_STATE_COMPLETE && g_wireless_test.expected == MISSION_TEST_STAGE_DONE);
        assert(strcmp(last_log, "[PATH] DONE HOME\r\n") == 0);
        mission_test_depot_path_command(&c, "ROUTE DEPOT PATH"); assert(count == length);
        /* 每段失败都不再发下一命令，失败点不能记成已到位。 */
        for (unsigned fail = 4; fail <= length; ++fail) {
            reset(&c, side); fail_at = fail;
            mission_test_depot_path_command(&c, "ROUTE DEPOT PATH");
            assert(count == fail && faults == 1 && c.state == MISSION_STATE_FAULT);
            assert(g_wireless_test.expected != MISSION_TEST_STAGE_DONE);
            if (fail > 4) assert(g_wireless_test.depot_position == expected[fail - 2] - 9);
            else assert(g_wireless_test.depot_position == 0);
        }
        /* 每一个500ms停车点都可STOP，不能继续横移/回家。 */
        for (unsigned stop = 1; stop <= length - 4; ++stop) {
            reset(&c, side); stop_pause = stop;
            mission_test_depot_path_command(&c, "ROUTE DEPOT PATH");
            assert(count == stop + 3 && faults == 0 && stopped == 1);
            assert(c.state == MISSION_STATE_STOPPED && g_wireless_test.stop_requested);
        }
    }
    for (unsigned stage = 1; stage <= 3; ++stage) {
        reset(&c, false); fail_pre = stage;
        mission_test_depot_path_command(&c, "ROUTE DEPOT PATH"); assert(count == stage && faults == 1);
    }
    reset(&c, false); g_wireless_test.mode = MISSION_TEST_MODE_TARGET;
    mission_test_depot_path_command(&c, "ROUTE DEPOT PATH"); assert(count == 0);
    reset(&c, false); g_wireless_test.expected = MISSION_TEST_STAGE_DEPOT;
    mission_test_depot_path_command(&c, "ROUTE DEPOT PATH"); assert(count == 0);
    reset(&c, false); assert(mission_test_pause(&c));
    assert(now == MISSION_CHASSIS_ROUTE_TEST_PAUSE_MS); /* 原有测试等待不变。 */
    puts("Depot pure path red/blue, 500ms, STOP and failures passed (host only)");
}
