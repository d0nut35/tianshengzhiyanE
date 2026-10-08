#include <assert.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include "mission_app.h"
#include "mission_config.h"
#include "zdt_turntable_core.h"
#undef MISSION_CHASSIS_ROUTE_TEST_ENABLED
#define MISSION_CHASSIS_ROUTE_TEST_ENABLED 0
#if TEST_TRACE
static void trace(const char *format, ...)
{
    char text[128]; va_list args; va_start(args,format);
    int len = vsnprintf(text,sizeof(text),format,args); va_end(args);
    assert(len >= 0 && (size_t)len < sizeof(text));
}
#define DEPOT_TRACE(...) trace(__VA_ARGS__)
#else
#define DEPOT_TRACE(...) ((void)0)
#endif
typedef struct {
    mission_state_t state; uint8_t current_slot; void *command_queue;
    struct { zdt_turntable_response_t zdt_response; zdt_turntable_status_t zdt_status;
             bool zdt_has_response; } storage;
} mission_context_t;
enum { osOK, MISSION_FLAG_ZDT_DONE };
static unsigned moves, gate_at, fault, stop_at;
static uint32_t now;
static zdt_turntable_direction_t expected_direction;
static uint32_t osKernelGetTickCount(void) { return now; }
static uint32_t mission_ms_to_ticks(uint32_t ms) { return ms; }
static void osDelay(uint32_t ticks) { now += ticks; }
static void osThreadFlagsClear(unsigned flag) { assert(flag == MISSION_FLAG_ZDT_DONE); }
static int osMessageQueueGet(void *q, mission_user_command_t *cmd, void *p, unsigned t)
{ (void)q; (void)p; (void)t; if (stop_at && moves >= stop_at) { *cmd = MISSION_USER_COMMAND_STOP; stop_at = 0; return osOK; } return -1; }
static void mission_handle_command(mission_context_t *c, mission_user_command_t cmd)
{ assert(cmd == MISSION_USER_COMMAND_STOP); c->state = MISSION_STATE_STOPPING; }
static void mission_zdt_done(void) {}
static bool mission_wait_zdt(mission_context_t *c) { (void)c; return fault != 2; }
static zdt_turntable_status_t mission_submit_slot_motion(mission_context_t *c, uint32_t angle,
    uint16_t speed, zdt_turntable_direction_t direction)
{
    assert(direction == expected_direction);
    assert(angle == (moves == 0 ? (direction == ZDT_TURNTABLE_DIR_CW ?
        MISSION_ZDT_COARSE_ANGLE_0P1DEG : MISSION_ZDT_REVERSE_COARSE_ANGLE_0P1DEG) : MISSION_ZDT_FINE_ANGLE_0P1DEG));
    assert(speed == 400); ++moves;
    c->storage.zdt_response.kind = ZDT_TURNTABLE_REPLY_ACK;
    c->storage.zdt_status = ZDT_TURNTABLE_OK; c->storage.zdt_has_response = true;
    return fault == 1 ? ZDT_TURNTABLE_ERR_BUSY : ZDT_TURNTABLE_OK;
}
static zdt_turntable_status_t turn_query_status(void (*done)(void), mission_context_t *c)
{
    assert(done == mission_zdt_done);
    zdt_turntable_response_t *r = &c->storage.zdt_response;
    memset(r,0,sizeof(*r)); r->kind = fault == 4 ? ZDT_TURNTABLE_REPLY_ACK : ZDT_TURNTABLE_REPLY_STATUS;
    r->data.motor_status.enabled = fault != 5;
    r->data.motor_status.stalled = fault == 6;
    r->data.motor_status.stall_protected = fault == 7;
    r->data.motor_status.power_loss_latched = fault == 8;
    r->data.motor_status.reached = fault != 9;
    c->storage.zdt_has_response = true; c->storage.zdt_status = ZDT_TURNTABLE_OK;
    return fault == 3 ? ZDT_TURNTABLE_ERR_IO : ZDT_TURNTABLE_OK;
}
static bool mission_gate_is_stably_high(void) { return gate_at && moves >= gate_at; }
#include "slot_under_test.inc"
static mission_context_t reset(unsigned gate, unsigned fail)
{
    mission_context_t c = {0}; c.state = MISSION_STATE_DEPOT_SEEK;
    moves = now = stop_at = 0; gate_at = gate; fault = fail; return c;
}
int main(void)
{
    assert(MISSION_ZDT_FINE_MAX_STEPS == 8);
    for (unsigned direction = 0; direction < 2; ++direction) {
        expected_direction = (zdt_turntable_direction_t)direction;
        for (unsigned require = 0; require < 2; ++require) {
            for (unsigned gate = 1; gate <= 9; ++gate) {
                mission_context_t c = reset(gate,0); uint8_t fine = 99;
                assert(mission_advance_slot(&c,expected_direction,&fine,require));
                assert(moves == gate && fine == gate - 1); /* 到PB0即停，不多转一步。 */
            }
            mission_context_t c = reset(0,0); uint8_t fine = 99;
            assert(mission_advance_slot(&c,expected_direction,&fine,require) == !require);
            assert(moves == 9); /* 一次粗转加八次微调，不能有第九次微调。 */
            if (!require) assert(fine == 8);
            for (unsigned fail = 1; fail <= 9; ++fail) {
                c = reset(1,fail);
                assert(!mission_advance_slot(&c,expected_direction,NULL,require));
                assert(moves == 1); /* PB0可选也不能跳过到位/电机/通信/超时保护。 */
            }
            c = reset(0,0); stop_at = 1;
            assert(!mission_advance_slot(&c,expected_direction,NULL,require));
            assert(c.state == MISSION_STATE_STOPPING && moves == 1);
        }
    }
    puts("Slot calibration limit, optional depot PB0 and motor/STOP protection passed (host only)");
}
