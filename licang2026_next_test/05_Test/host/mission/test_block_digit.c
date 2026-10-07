/* 抽取生产函数验证跳站、时序和协议隔离；不证明机械动作与真实路线安全。 */
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "mission_app.h"
#include "mission_config.h"
#include "nano_vision_core.h"
typedef unsigned mission_command_type_t;
typedef unsigned chassis_command_type_t;
enum { MISSION_CMD_GO_DEPOT_1 = 1, MISSION_CMD_GO_DEPOT_2,
    MISSION_CMD_GO_DEPOT_3, MISSION_CMD_GO_DEPOT_4, MISSION_CMD_DEPOT_OK };
enum { CHASSIS_CMD_DEPOT_1_READY = 1, CHASSIS_CMD_DEPOT_2_READY,
    CHASSIS_CMD_DEPOT_3_READY, CHASSIS_CMD_DEPOT_4_READY, CHASSIS_CMD_HOME_READY };
enum { MISSION_VISION_IDLE, MISSION_VISION_STARTING, MISSION_VISION_LISTENING,
    MISSION_VISION_ACKING, MISSION_VISION_STOPPING };
enum { MULT_UART_OP_WRITE, MISSION_FAULT_VISION, MISSION_FAULT_ARM,
    MISSION_TEST_STAGE_DONE, DEBUG_UART1_RX_BUFFER_SIZE = 128 };
typedef struct {
    unsigned phase;
    uint16_t session_id, mail_len;
    uint8_t next_sequence, tx[32], mail_data[32];
} mission_vision_t;
typedef struct {
    mission_state_t state;
    mission_vision_t vision;
    nano_vision_block_result_t block_result;
    bool block_result_received;
    bool arm_home_ready;
    uint32_t deadline_tick;
} mission_context_t;
static struct { bool stop_requested; unsigned depot_position, expected; char text[224]; } g_wireless_test;
static unsigned points[32], moves, groups[8], arm_count, reads, stage;
static unsigned found_at[3], faults, acknowledgements, fail_group;
static uint32_t now, stop_at, settle_total, found_pause_total;
static unsigned delays_200, delays_1000;
static uint32_t osKernelGetTickCount(void) { return now; }
static uint32_t mission_ms_to_ticks(uint32_t ms) { return ms; }
static void osDelay(uint32_t ticks) { now += ticks; }
static void mission_test_write(const char *text) { (void)text; }
static bool mission_test_take_command(char *c, size_t len)
{ (void)c; (void)len; if (stop_at && now >= stop_at) g_wireless_test.stop_requested = true; return false; }
static bool mission_test_handle_aux_command(mission_context_t *c, const char *cmd)
{ (void)c; (void)cmd; return true; }
static void mission_fail(mission_context_t *c, unsigned fault)
{ (void)fault; ++faults; c->state = MISSION_STATE_FAULT; }
static void mission_enter_state(mission_context_t *c, mission_state_t state, uint32_t ms)
{ c->state = state; c->deadline_tick = now + ms; }
static uint8_t mission_next_vision_sequence(mission_vision_t *v) { return ++v->next_sequence; }
static nano_vision_status_t mission_submit_vision_transfer(mission_context_t *c, unsigned op, size_t len, uint32_t timeout)
{
    nano_vision_frame_t f; nano_vision_event_ack_t ack;
    assert(op == MULT_UART_OP_WRITE && timeout == 200);
    assert(nano_vision_decode_frame(c->vision.tx,len,&f) == NANO_VISION_OK);
    assert(nano_vision_parse_event_ack(&f,&ack) == NANO_VISION_OK);
    assert(ack.session_id == c->vision.session_id);
    ++acknowledgements; return NANO_VISION_OK;
}
static bool mission_test_send_wait(mission_context_t *c, unsigned cmd, unsigned evt, mission_state_t s)
{ (void)c; (void)s; assert(cmd == evt); points[moves++] = cmd; return true; }
static bool mission_test_run_arm_group(mission_context_t *c, uint8_t group)
{
    if (group == 10 && c->arm_home_ready) return true;
    if (group == 28) assert(c->arm_home_ready);
    if (group == 29 || group == 30) {
        assert(arm_count > 0 && groups[arm_count - 1] == (unsigned)(group - 1));
        assert(g_wireless_test.depot_position == 4);
    }
    c->arm_home_ready = false;
    groups[arm_count++] = group;
    if (group == fail_group) return false;
    c->arm_home_ready = group == 10;
    if (group >= 28 && group <= 30) stage = group - 28;
    return true;
}
static bool mission_test_read_block_digit(mission_context_t *c, nano_vision_block_result_t *r)
{
    assert(c->state != MISSION_STATE_FAULT);
    assert(g_wireless_test.depot_position >= 1 && g_wireless_test.depot_position <= 3);
    assert(delays_200 == reads + 1); /* 每次读之前已停车并等200ms。 */
    ++reads;
    memset(r,0,sizeof(*r));
    r->status = (g_wireless_test.depot_position == found_at[stage]) ? NANO_VISION_BLOCK_DIGIT : NANO_VISION_BLOCK_NO_VALID;
    r->digit = r->status == NANO_VISION_BLOCK_DIGIT ? (uint8_t)(stage + 1) : 0;
    r->reason = r->digit ? NANO_VISION_REASON_CONFIRMED : NANO_VISION_REASON_LOW_SCORE;
    if (found_at[stage] == 99) { r->status = NANO_VISION_BLOCK_FAULT; r->reason = NANO_VISION_REASON_CAMERA_ERROR; }
    r->frames = 20; r->elapsed_ms = 1500;
    return true;
}
/* 延时函数也使用生产源码；计数包装仅记录测试证据。 */
#define mission_test_block_delay production_block_delay
#include "block_delay_under_test.inc"
#undef mission_test_block_delay
static bool mission_test_block_delay(mission_context_t *c, uint32_t ms)
{
    uint32_t start = now;
    bool ok = production_block_delay(c,ms);
    if (ms == 200) { ++delays_200; settle_total += now - start; }
    if (ms == 1000) { ++delays_1000; found_pause_total += now - start; }
    return ok;
}
#include "block_digit_under_test.inc"
static void reset(mission_context_t *c)
{
    memset(c,0,sizeof(*c)); memset(&g_wireless_test,0,sizeof(g_wireless_test));
    g_wireless_test.depot_position = 1;
    c->arm_home_ready = true;
    moves = arm_count = reads = stage = faults = acknowledgements = fail_group = 0;
    now = stop_at = settle_total = found_pause_total = delays_200 = delays_1000 = 0;
}
static void copy_frame(mission_context_t *c, size_t len) { c->vision.mail_len = (uint16_t)len; }
int main(void)
{
    mission_context_t c; reset(&c);
    found_at[0] = 2; found_at[1] = 3; found_at[2] = 1;
    assert(mission_test_run_block_digits(&c));
    const unsigned expected_moves[] = {2,4,3,4,3,2,1,4,5};
    const unsigned expected_groups[] = {28,29,30,10};
    assert(moves == 9 && memcmp(points,expected_moves,sizeof(expected_moves)) == 0);
    assert(arm_count == 4 && memcmp(groups,expected_groups,sizeof(expected_groups)) == 0);
    assert(reads == 6 && delays_200 == 6 && settle_total == 1200);
    assert(delays_1000 == 3 && found_pause_total == 3000);
    assert(c.state == MISSION_STATE_COMPLETE);
    reset(&c); found_at[0] = found_at[1] = found_at[2] = 0;
    assert(mission_test_run_block_digits(&c)); assert(reads == 9 && delays_1000 == 0);
    reset(&c); found_at[0] = 99;
    assert(!mission_test_run_block_digits(&c)); assert(faults == 1 && moves == 0 && arm_count == 1);
    reset(&c); found_at[0] = 2; stop_at = 50;
    assert(!mission_test_run_block_digits(&c)); assert(reads == 0 && moves == 0);
    reset(&c); fail_group = 28;
    assert(!mission_test_run_block_digits(&c)); assert(reads == 0 && faults == 1);
    /* 首层没有10完成缓存时仍须执行10；失败禁止执行28或开启会话。 */
    reset(&c); c.arm_home_ready = false; fail_group = 10;
    assert(!mission_test_run_block_digits(&c));
    assert(faults == 1 && reads == 0 && moves == 0 && arm_count == 1 && groups[0] == 10);
    /* 直接切29/30失败时不得采样下一层，层间不能插入10。 */
    reset(&c); found_at[0] = found_at[1] = found_at[2] = 1; fail_group = 29;
    assert(!mission_test_run_block_digits(&c));
    assert(faults == 1 && reads == 1 && arm_count == 2 && groups[1] == 29);
    reset(&c); fail_group = 30;
    assert(!mission_test_run_block_digits(&c));
    assert(faults == 1 && reads == 4 && arm_count == 3 && groups[2] == 30);
    /* 最后回10失败时停在D4，禁止回家；全部NO_VALID仍完整扫描九个点。 */
    reset(&c); found_at[0] = found_at[1] = found_at[2] = 0; fail_group = 10;
    assert(!mission_test_run_block_digits(&c));
    assert(faults == 1 && reads == 9 && arm_count == 4 && groups[3] == 10);
    assert(points[moves - 1] == MISSION_CMD_GO_DEPOT_4);

    /* READY匹配本点后才开始8秒通信保护；旧会话、旧帧和运动状态均不能确认。 */
    reset(&c); c.state = MISSION_STATE_BLOCK_WAIT_DIGIT;
    c.vision.phase = MISSION_VISION_STARTING; c.vision.session_id = 7;
    nano_vision_session_t ready = {6,NANO_VISION_SCENE_BLOCK_DIGIT,NANO_VISION_COLOR_ANY};
    size_t len;
    assert(nano_vision_build_session_ready_frame(1,&ready,c.vision.mail_data,32,&len) == NANO_VISION_OK);
    copy_frame(&c,len); mission_handle_block_result(&c); assert(c.vision.phase == MISSION_VISION_STARTING);
    ready.session_id = 7;
    assert(nano_vision_build_session_ready_frame(1,&ready,c.vision.mail_data,32,&len) == NANO_VISION_OK);
    copy_frame(&c,len); mission_handle_block_result(&c);
    assert(c.vision.phase == MISSION_VISION_LISTENING && c.deadline_tick == 8000);
    nano_vision_block_result_t r = {6,NANO_VISION_BLOCK_DIGIT,3,90,NANO_VISION_REASON_CONFIRMED,99,20,3,192};
    assert(nano_vision_build_block_result_frame(1,&r,c.vision.mail_data,32,&len) == NANO_VISION_OK);
    copy_frame(&c,len); mission_handle_block_result(&c); assert(!c.block_result_received);
    r.session_id = 7; r.age_ms = 121;
    assert(nano_vision_build_block_result_frame(1,&r,c.vision.mail_data,32,&len) == NANO_VISION_OK);
    copy_frame(&c,len); mission_handle_block_result(&c); assert(!c.block_result_received);
    r.age_ms = 20;
    assert(nano_vision_build_block_result_frame(1,&r,c.vision.mail_data,32,&len) == NANO_VISION_OK);
    copy_frame(&c,len); c.state = MISSION_STATE_WAIT_DEPOT_1;
    mission_handle_block_result(&c); assert(!c.block_result_received);
    c.state = MISSION_STATE_BLOCK_WAIT_DIGIT;
    mission_handle_block_result(&c); assert(c.block_result_received && acknowledgements == 1);
    assert(c.vision.phase == MISSION_VISION_ACKING && c.block_result.digit == 3);
    reset(&c); c.state = MISSION_STATE_BLOCK_WAIT_DIGIT;
    c.vision.phase = MISSION_VISION_STARTING; c.vision.session_id = 7;
    r.status = NANO_VISION_BLOCK_FAULT; r.digit = 0; r.reason = NANO_VISION_REASON_CAMERA_ERROR;
    assert(nano_vision_build_block_result_frame(1,&r,c.vision.mail_data,32,&len) == NANO_VISION_OK);
    copy_frame(&c,len); mission_handle_block_result(&c);
    assert(c.block_result_received && c.block_result.status == NANO_VISION_BLOCK_FAULT);
    puts("Block layer route and session isolation passed (host logic only)");
}
