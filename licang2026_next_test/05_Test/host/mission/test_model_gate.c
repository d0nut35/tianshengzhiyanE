#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "mission_app.h"
#include "mission_config.h"
#include "nano_vision_core.h"
enum { MISSION_VISION_IDLE, MISSION_VISION_MODEL_QUERYING };
enum { MULT_UART_OP_WRITE_READ, MISSION_CMD_MISSION_READY, MISSION_FAULT_QUEUE };
typedef struct {
    unsigned phase;
    bool inflight, completion_pending;
    uint8_t next_sequence, tx[32];
} mission_vision_t;
typedef struct {
    mission_state_t state;
    bool chassis_ready, arm_home_ready, block_model_ready;
    uint32_t model_next_query_tick;
    uint16_t request_id;
    mission_vision_t vision;
} mission_context_t;
volatile mission_color_t g_mission_side;
static uint32_t now;
static unsigned sends, queries;
static uint32_t osKernelGetTickCount(void) { return now; }
static uint32_t mission_ms_to_ticks(uint32_t ms) { return ms; }
static uint8_t mission_next_vision_sequence(mission_vision_t *v) { return ++v->next_sequence; }
static bool mission_send_chassis(unsigned cmd, uint16_t id)
{ assert(cmd == MISSION_CMD_MISSION_READY && id == 7); ++sends; return true; }
static void mission_enter_state(mission_context_t *c, mission_state_t state, uint32_t timeout)
{ (void)timeout; c->state = state; }
static void mission_fail(mission_context_t *c, unsigned fault)
{ (void)fault; c->state = MISSION_STATE_FAULT; }
static nano_vision_status_t mission_submit_vision_transfer(mission_context_t *c, unsigned op, size_t len, uint32_t timeout)
{
    nano_vision_frame_t f;
    assert(op == MULT_UART_OP_WRITE_READ && timeout == 200);
    assert(nano_vision_decode_frame(c->vision.tx,len,&f) == NANO_VISION_OK);
    assert(f.type == NANO_VISION_MSG_MODEL_QUERY && f.payload[0] == 7);
    ++queries; return NANO_VISION_OK;
}
#include "model_gate_under_test.inc"
int main(void)
{
    for (unsigned bits = 0; bits < 8; ++bits) {
        mission_context_t c = {0};
        c.state = MISSION_STATE_WAIT_CHASSIS_READY; c.request_id = 7;
        c.arm_home_ready = (bits & 1) != 0;
        c.chassis_ready = (bits & 2) != 0;
        c.block_model_ready = (bits & 4) != 0;
        sends = 0; g_mission_side = MISSION_COLOR_RED;
        mission_try_ready(&c);
        assert(sends == (bits == 7));
    }
    mission_context_t c = {0}; c.state = MISSION_STATE_WAIT_HOME;
    now = 0; queries = 0;
    mission_model_process(&c); assert(queries == 1 && c.vision.phase == MISSION_VISION_MODEL_QUERYING);
    c.vision.phase = MISSION_VISION_IDLE; now = 999;
    mission_model_process(&c); assert(queries == 1);
    now = 1000; c.vision.completion_pending = true;
    mission_model_process(&c); assert(queries == 1);
    c.vision.completion_pending = false;
    mission_model_process(&c); assert(queries == 2);
    c.vision.phase = MISSION_VISION_IDLE; now = 2000; c.block_model_ready = true;
    mission_model_process(&c); assert(queries == 2);
    c.block_model_ready = false; c.state = MISSION_STATE_STOPPED;
    mission_model_process(&c); assert(queries == 2);
    puts("Model startup gate passed (host logic only)");
}
