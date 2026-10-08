#include <assert.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include "mission_app.h"
#include "mission_config.h"
#include "nano_vision_core.h"
#include "zdt_turntable_core.h"
#undef MISSION_CHASSIS_ROUTE_TEST_ENABLED
#define MISSION_CHASSIS_ROUTE_TEST_ENABLED 0
#if TEST_TRACE
static void diagnostic(const char *format, ...)
{
    char text[128]; va_list args; va_start(args,format);
    int len = vsnprintf(text,sizeof(text),format,args); va_end(args);
    assert(len >= 0 && (size_t)len < sizeof(text));
}
#define DEPOT_TRACE(...) diagnostic(__VA_ARGS__)
#else
#define DEPOT_TRACE(...) ((void)0)
#endif
enum { MISSION_VISION_IDLE, MISSION_VISION_MODEL_QUERYING };
enum { MULT_UART_OP_WRITE_READ, MISSION_CMD_MISSION_READY, MISSION_FAULT_QUEUE, MISSION_FAULT_VISION,
       MISSION_FLAG_ZDT_DONE };
typedef struct {
    unsigned phase;
    bool inflight, completion_pending;
    uint8_t next_sequence, tx[32];
    uint8_t mail_data[32];
    uint16_t mail_len;
    nano_vision_status_t mail_status;
} mission_vision_t;
typedef struct {
    mission_state_t state;
    bool chassis_ready, arm_home_ready, block_model_ready;
    uint32_t model_next_query_tick;
    uint16_t request_id;
    mission_vision_t vision;
    struct {
        zdt_turntable_response_t zdt_response;
        zdt_turntable_status_t zdt_status;
        bool zdt_has_response;
    } storage;
} mission_context_t;
volatile mission_color_t g_mission_side;
static uint32_t now;
static unsigned sends, queries;
static nano_vision_status_t submit_status = NANO_VISION_OK;
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
    ++queries; return submit_status;
}
static nano_vision_status_t mission_map_vision_status(nano_vision_status_t s) { return s; }
static zdt_turntable_status_t zdt_submit = ZDT_TURNTABLE_OK;
static bool zdt_completed = true;
static unsigned zdt_waits;
static zdt_turntable_response_t zdt_reply;
static void mission_zdt_done(void) {}
static void osThreadFlagsClear(unsigned flag) { assert(flag == MISSION_FLAG_ZDT_DONE); }
static zdt_turntable_status_t turn_query_options(void (*done)(void), mission_context_t *c)
{
    assert(done == mission_zdt_done);
    if (zdt_submit == ZDT_TURNTABLE_OK) {
        c->storage.zdt_response = zdt_reply;
        c->storage.zdt_status = ZDT_TURNTABLE_OK;
        c->storage.zdt_has_response = true;
    }
    return zdt_submit;
}
static bool mission_wait_zdt(mission_context_t *c) { (void)c; ++zdt_waits; return zdt_completed; }
#include "model_gate_under_test.inc"
#include "model_reply_under_test.inc"
static void response(mission_context_t *c, uint8_t sequence, nano_vision_model_state_t state)
{
    nano_vision_model_report_t report = {state, state == NANO_VISION_MODEL_ERROR ?
        NANO_VISION_REASON_MODEL_ERROR : NANO_VISION_REASON_NONE};
    size_t len;
    assert(nano_vision_build_model_state_frame(sequence,&report,c->vision.mail_data,32,&len) == NANO_VISION_OK);
    c->vision.mail_len = (uint16_t)len;
    c->vision.mail_status = NANO_VISION_OK;
    c->vision.phase = MISSION_VISION_MODEL_QUERYING;
    c->vision.completion_pending = true;
}
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
    /* 模型可以在STM32上电前就READY：仍须本次主动查询及两项硬件就绪。 */
    memset(&c,0,sizeof(c)); c.state = MISSION_STATE_WAIT_CHASSIS_READY; c.request_id = 7;
    c.arm_home_ready = c.chassis_ready = true; g_mission_side = MISSION_COLOR_RED;
    sends = 0;
    for (unsigned second = 0; second < 21; ++second) {
        now = second * 1000; unsigned queried = queries;
        mission_model_process(&c); assert(queries == queried + 1);
        response(&c,c.vision.next_sequence,NANO_VISION_MODEL_LOADING); model_reply(&c);
        assert(!c.block_model_ready && sends == 0 && c.vision.phase == MISSION_VISION_IDLE);
        assert(c.state == MISSION_STATE_WAIT_CHASSIS_READY);
    }
    now = 21000; mission_model_process(&c);
    response(&c,c.vision.next_sequence,NANO_VISION_MODEL_READY); model_reply(&c);
    assert(c.block_model_ready && c.state == MISSION_STATE_READY && sends == 1);
    assert(!c.vision.completion_pending);
    model_reply(&c); assert(sends == 1); /* 已消费的邮箱不可重复推进。 */
    memset(&c,0,sizeof(c)); c.state = MISSION_STATE_WAIT_HOME; c.request_id = 7; sends = 0;
    now = 0; mission_model_process(&c);
    response(&c,c.vision.next_sequence,NANO_VISION_MODEL_READY); model_reply(&c);
    assert(c.block_model_ready && sends == 0 && c.state == MISSION_STATE_WAIT_HOME);
    c.state = MISSION_STATE_WAIT_CHASSIS_READY; c.arm_home_ready = c.chassis_ready = true;
    mission_try_ready(&c); assert(sends == 1);
    /* 无回复可一直重试，不把模型加载当成识别超时；占用及完成邮箱都不可覆盖。 */
    memset(&c,0,sizeof(c)); c.state = MISSION_STATE_WAIT_HOME; now = 0;
    mission_model_process(&c); unsigned before = queries;
    c.vision.completion_pending = true; c.vision.mail_status = NANO_VISION_ERR_TIMEOUT;
    model_reply(&c); assert(c.state == MISSION_STATE_WAIT_HOME && !c.block_model_ready);
    now = 999; mission_model_process(&c); assert(queries == before);
    now = 1000; c.vision.inflight = true; mission_model_process(&c); assert(queries == before);
    c.vision.inflight = false; submit_status = NANO_VISION_ERR_BUSY;
    mission_model_process(&c); assert(queries == before + 1 && c.vision.phase == MISSION_VISION_IDLE);
    submit_status = NANO_VISION_OK;
    /* 记录当前故障策略：错序号/坏CRC/半包会锁存故障，日志必须与关闭时行为一致。 */
    for (unsigned error = 0; error < 4; ++error) {
        memset(&c,0,sizeof(c)); c.state = MISSION_STATE_WAIT_CHASSIS_READY; c.vision.next_sequence = 7;
        response(&c,error == 0 ? 6 : 7,NANO_VISION_MODEL_READY);
        if (error == 1) c.vision.mail_data[7] ^= 1;
        if (error == 2) c.vision.mail_len = 5;
        if (error == 3) c.vision.mail_status = NANO_VISION_ERR_IO;
        model_reply(&c); assert(c.state == MISSION_STATE_FAULT && !c.block_model_ready);
    }
    memset(&c,0,sizeof(c)); c.state = MISSION_STATE_WAIT_CHASSIS_READY; c.vision.next_sequence = 7;
    response(&c,7,NANO_VISION_MODEL_ERROR); model_reply(&c);
    assert(c.state == MISSION_STATE_FAULT && !c.block_model_ready);
    memset(&c,0,sizeof(c)); c.state = MISSION_STATE_STOPPED; c.vision.next_sequence = 7; sends = 0;
    response(&c,7,NANO_VISION_MODEL_READY); model_reply(&c);
    assert(c.state == MISSION_STATE_STOPPED && sends == 0 && !c.block_model_ready);
    /* 正式独有的转盘启动检查：日志开启不能改变失败条件或重复提交。 */
    for (unsigned error = 0; error < 6; ++error) {
        memset(&c,0,sizeof(c)); memset(&zdt_reply,0,sizeof(zdt_reply));
        zdt_reply.kind = ZDT_TURNTABLE_REPLY_OPTIONS;
        zdt_reply.data.options.closed_loop = true;
        zdt_reply.data.options.firmware = ZDT_TURNTABLE_FIRMWARE_EMM;
        zdt_submit = error == 1 ? ZDT_TURNTABLE_ERR_BUSY : ZDT_TURNTABLE_OK;
        zdt_completed = error != 2; zdt_waits = 0;
        if (error == 3) zdt_reply.kind = ZDT_TURNTABLE_REPLY_STATUS;
        if (error == 4) zdt_reply.data.options.closed_loop = false;
        if (error == 5) zdt_reply.data.options.firmware = ZDT_TURNTABLE_FIRMWARE_X;
        assert(mission_prepare_zdt(&c) == (error == 0));
        assert(zdt_waits == (error == 1 ? 0 : 1));
    }
    puts("Model startup gate passed (host logic only)");
}
