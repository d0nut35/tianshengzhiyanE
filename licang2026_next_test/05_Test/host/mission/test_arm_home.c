/* 实际Mission回报及等待函数：发送10不算到位，异常不能保留旧姿态。 */
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include "mission_app.h"
#include "mission_config.h"
#undef MISSION_CHASSIS_ROUTE_TEST_ENABLED
#define MISSION_CHASSIS_ROUTE_TEST_ENABLED TEST_MODE
#define DEPOT_TRACE(...) ((void)0)
#define PLATFORM_TRACE(...) ((void)0)
#define DEBUG_UART1_RX_BUFFER_SIZE 64
#define osFlagsError 0x80000000U
#define osFlagsWaitAny 0
enum { LSC16_OK, LSC16_FAIL };
enum { MISSION_FLAG_ARM_OK = 1, MISSION_FLAG_ARM_FAIL = 2 };
enum { LSC16_REPORT_EVENT_ACTION_STARTED = 1, LSC16_REPORT_EVENT_ACTION_STOPPED = 2,
       LSC16_REPORT_EVENT_ACTION_COMPLETED = 4, LSC16_REPORT_EVENT_INVALID_FRAME = 8 };
typedef int lsc16_status_t;
typedef struct { uint8_t action_group; } lsc16_report_t;
typedef struct {
    mission_state_t state;
    uint8_t active_arm_group, platform_attempts;
    bool arm_home_ready;
    uint32_t arm_last_action_report, deadline_tick;
    void *task;
} mission_context_t;
static struct { bool stop_requested; } g_wireless_test;
static unsigned sent, stopped, flags, wait_events;
static uint32_t now;
static bool tx_ok = true, wait_timeout;
static mission_context_t *waiting;
static void mission_arm_report(void *ctx, uint32_t events, const lsc16_report_t *report);
static uint32_t osKernelGetTickCount(void) { return now; }
static uint32_t osThreadFlagsSet(void *task, uint32_t value)
{ assert(task); flags |= value; return flags; }
static uint32_t osThreadFlagsClear(uint32_t value) { flags &= ~value; return flags; }
static uint32_t mission_ms_to_ticks(uint32_t ms) { return ms; }
static uint32_t osThreadFlagsWait(uint32_t value, unsigned options, uint32_t ticks)
{
    (void)value; (void)options; (void)ticks;
    if (wait_timeout) { now = waiting->deadline_tick; return osFlagsError; }
    lsc16_report_t report = {waiting->active_arm_group};
    mission_arm_report(waiting, wait_events, &report);
    return flags;
}
static int arm_run(uint8_t group, uint16_t repeat,
                   void (*done)(void *, uint32_t, lsc16_status_t), void *ctx)
{
    (void)group; (void)done; assert(repeat == 1);
    ++sent; waiting = ctx;
    return tx_ok ? LSC16_OK : LSC16_FAIL;
}
static int arm_stop(void *done, void *ctx)
{ (void)done; (void)ctx; ++stopped; return LSC16_OK; }
static void mission_enter_state(mission_context_t *c, mission_state_t state, uint32_t ms)
{ c->state = state; c->deadline_tick = ms ? now + ms : 0; }
static bool mission_test_take_command(char *cmd, unsigned size)
{ (void)cmd; (void)size; return false; }
static bool mission_test_handle_aux_command(mission_context_t *c, const char *cmd)
{ (void)c; (void)cmd; return false; }
static void mission_test_write(const char *text) { (void)text; }
#include "arm_home_under_test.inc"

int main(void)
{
    mission_context_t c = {0}; c.task = &c;
    lsc16_report_t home = {10}, pick = {22};
    assert(mission_start_arm(&c, 10, MISSION_STATE_DEPOT_PREPARE));
    assert(!c.arm_home_ready && sent == 1);
    mission_arm_tx_done(&c, 1, LSC16_OK); assert(!c.arm_home_ready);
    mission_arm_report(&c, LSC16_REPORT_EVENT_ACTION_STARTED, &home);
    assert(!c.arm_home_ready);
    mission_arm_report(&c, LSC16_REPORT_EVENT_ACTION_COMPLETED, &home);
    assert(c.arm_home_ready);
    assert(mission_test_run_arm_group(&c, 10)); assert(sent == 1);

    wait_events = LSC16_REPORT_EVENT_ACTION_COMPLETED;
    assert(mission_test_run_arm_group(&c, 22)); assert(!c.arm_home_ready && sent == 2);
    mission_arm_report(&c, LSC16_REPORT_EVENT_ACTION_COMPLETED, &home);
    assert(!c.arm_home_ready); /* 不匹配当前动作的旧回报不能恢复10。 */
    assert(mission_test_run_arm_group(&c, 10)); assert(c.arm_home_ready && sent == 3);
    assert(mission_test_run_arm_group(&c, 10)); assert(sent == 3);
    mission_arm_report(&c, LSC16_REPORT_EVENT_ACTION_STARTED, &pick);
    assert(!c.arm_home_ready);
    assert(mission_test_run_arm_group(&c, 10)); assert(sent == 4);

    assert(mission_start_arm(&c, 10, MISSION_STATE_DEPOT_PREPARE));
    flags = 0;
    mission_arm_report(&c, LSC16_REPORT_EVENT_ACTION_COMPLETED |
                           LSC16_REPORT_EVENT_ACTION_STOPPED, &home);
    assert(!c.arm_home_ready && flags == MISSION_FLAG_ARM_FAIL);
    c.state = MISSION_STATE_STOPPED;
    mission_arm_report(&c, LSC16_REPORT_EVENT_ACTION_COMPLETED, &home);
    assert(!c.arm_home_ready);
    c.state = MISSION_STATE_FAULT;
    mission_arm_report(&c, LSC16_REPORT_EVENT_ACTION_COMPLETED, &home);
    assert(!c.arm_home_ready);

    c.arm_home_ready = true; tx_ok = false;
    assert(!mission_start_arm(&c, 22, MISSION_STATE_DEPOT_PICK));
    assert(!c.arm_home_ready);
    c.arm_home_ready = true;
    mission_arm_tx_done(&c, 1, LSC16_FAIL); assert(!c.arm_home_ready);
    tx_ok = true; wait_events = LSC16_REPORT_EVENT_ACTION_STOPPED;
    assert(!mission_test_run_arm_group(&c, 10)); assert(!c.arm_home_ready);
    wait_timeout = true;
    assert(!mission_test_run_arm_group(&c, 10));
    assert(!c.arm_home_ready && stopped == 1);
    puts("Arm home completion/skip regression passed (host logic only)");
    return 0;
}
