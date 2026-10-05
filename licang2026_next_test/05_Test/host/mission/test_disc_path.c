/* 直接调用无线生产命令函数，验证两步衔接、错误顺序和停止，不模拟实机。 */
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "mission_app.h"
#include "mission_config.h"
enum { MISSION_TEST_MODE_IDLE, MISSION_TEST_MODE_PATH, MISSION_TEST_MODE_TARGET };
enum { MISSION_TEST_STAGE_PLATFORM, MISSION_TEST_STAGE_STAIRS,
       MISSION_TEST_STAGE_SMALL_DISC, MISSION_TEST_STAGE_DEPOT };
enum { MISSION_CMD_GO_SMALL_DISC = 1, MISSION_CMD_SMALL_DISC_START };
enum { CHASSIS_CMD_SMALL_DISC_READY = 1, CHASSIS_CMD_SMALL_DISC_FINISHED };
enum { MISSION_FAULT_CHASSIS = 7 };
typedef struct { uint16_t request_id; mission_state_t state; uint32_t deadline; } mission_context_t;
static struct { unsigned mode, expected; bool small_disc_at_start, stop_requested; } g_wireless_test;
static unsigned platform, stairs, arrivals, starts, finished, faults;
static bool arrive_ok, run_ok, stop_on_run;
static char reply[128];
static void mission_test_write(const char *s) { snprintf(reply, sizeof(reply), "%s", s); }
static void mission_enter_state(mission_context_t *c, mission_state_t s, uint32_t t)
{ c->state = s; c->deadline = t; }
static bool mission_test_skip_platform(mission_context_t *c) { (void)c; ++platform; return true; }
static bool mission_test_skip_stairs(mission_context_t *c) { (void)c; ++stairs; return true; }
static bool mission_test_send_wait(mission_context_t *c, unsigned cmd, unsigned evt, mission_state_t s)
{
    assert(cmd == MISSION_CMD_GO_SMALL_DISC && evt == CHASSIS_CMD_SMALL_DISC_READY);
    c->request_id = 42; c->state = s; ++arrivals; return arrive_ok;
}
static bool mission_send_chassis(unsigned cmd, uint16_t id)
{ assert(cmd == MISSION_CMD_SMALL_DISC_START && id == 42); ++starts; return true; }
static bool mission_test_wait_chassis_event(mission_context_t *c, unsigned evt, uint16_t id)
{
    (void)c; assert(evt == CHASSIS_CMD_SMALL_DISC_FINISHED && id == 42); ++finished;
    if (stop_on_run) g_wireless_test.stop_requested = true;
    return run_ok;
}
static void mission_fail(mission_context_t *c, unsigned f)
{ assert(f == MISSION_FAULT_CHASSIS); c->state = MISSION_STATE_FAULT; ++faults; }
#include "disc_path_under_test.inc"
static void reset(mission_context_t *c)
{
    memset(c, 0, sizeof(*c)); memset(&g_wireless_test, 0, sizeof(g_wireless_test));
    platform = stairs = arrivals = starts = finished = faults = 0;
    arrive_ok = run_ok = true; stop_on_run = false; reply[0] = 0;
}
int main(void)
{
    mission_context_t c;
    reset(&c);
    assert(!mission_test_disc_path_command(&c, "DEPOT"));
    assert(mission_test_disc_path_command(&c, "DISC_RUN"));
    assert(starts == 0 && strcmp(reply, "ERR WAIT DISC_READY\r\n") == 0);
    /* 起点/阶梯后/小圆盘前均可到位，已完成前段不重走。 */
    for (unsigned stage = 0; stage < 3; ++stage) {
        reset(&c); g_wireless_test.expected = stage;
        assert(mission_test_disc_path_command(&c, "DISC_READY"));
        assert(platform == (stage == 0) && stairs == (stage != 2));
        assert(arrivals == 1 && starts == 0 && g_wireless_test.small_disc_at_start);
        assert(c.deadline == 0 && g_wireless_test.expected == MISSION_TEST_STAGE_SMALL_DISC);
        assert(mission_test_disc_path_command(&c, "DISC_READY"));
        assert(arrivals == 1); /* 重复到位命令不重复发GO。 */
        assert(mission_test_disc_path_command(&c, "DISC_RUN"));
        assert(starts == 1 && finished == 1 && arrivals == 1);
        assert(!g_wireless_test.small_disc_at_start && c.deadline == 0);
        assert(g_wireless_test.expected == MISSION_TEST_STAGE_DEPOT);
        assert(mission_test_disc_path_command(&c, "DISC_RUN"));
        assert(starts == 1); /* 绕完不能再从错误位置启动第二圈。 */
    }
    reset(&c); g_wireless_test.mode = MISSION_TEST_MODE_TARGET;
    mission_test_disc_path_command(&c, "DISC_READY"); assert(arrivals == 0);
    reset(&c); g_wireless_test.expected = MISSION_TEST_STAGE_DEPOT;
    mission_test_disc_path_command(&c, "DISC_READY"); assert(arrivals == 0);
    reset(&c); arrive_ok = false;
    mission_test_disc_path_command(&c, "DISC_READY");
    assert(faults == 1 && !g_wireless_test.small_disc_at_start && starts == 0);
    reset(&c); mission_test_disc_path_command(&c, "DISC_READY"); run_ok = false;
    mission_test_disc_path_command(&c, "DISC_RUN"); assert(faults == 1);
    reset(&c); mission_test_disc_path_command(&c, "DISC_READY");
    run_ok = false; stop_on_run = true; mission_test_disc_path_command(&c, "DISC_RUN");
    assert(faults == 0 && g_wireless_test.stop_requested);
    puts("Wireless disc path command regression passed (host logic only)");
    return 0;
}
