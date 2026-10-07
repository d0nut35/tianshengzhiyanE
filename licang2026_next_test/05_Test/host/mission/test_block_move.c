/* 实际搬运函数、回报门禁及协议Core；仅验证主机逻辑，不证明实物夹持。 */
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include "mission_app.h"
#include "mission_config.h"
#include "nano_vision_core.h"
#include "ball_manifest_core.h"
#include "lsc16_core.h"
#undef MISSION_CHASSIS_ROUTE_TEST_ENABLED
#define MISSION_CHASSIS_ROUTE_TEST_ENABLED TEST_MODE
#undef MISSION_DEPOT_TRACE_ENABLED
#define MISSION_DEPOT_TRACE_ENABLED TEST_TRACE
#include "block_move_protocol.inc"
#define CHASSIS_MISSION_REQUEST_ID_INVALID 0U
#include "stair_trace_disabled.h"
#define PLATFORM_TRACE(...) ((void)0)
#if TEST_TRACE
static void capture(const char *format, ...)
{
    char text[128]; va_list args; va_start(args, format);
    int length = vsnprintf(text, sizeof(text), format, args);
    va_end(args); assert(length >= 0 && length < (int)sizeof(text));
}
#define DEPOT_TRACE(...) capture(__VA_ARGS__)
#define BLOCK_TRACE(...) capture(__VA_ARGS__)
#else
#define DEPOT_TRACE(...) ((void)0)
#define BLOCK_TRACE(...) ((void)0)
#endif
enum { MISSION_FLAG_ARM_OK = 4, MISSION_FLAG_ARM_FAIL = 8 };
enum { MISSION_VISION_IDLE, MISSION_VISION_STARTING, MISSION_VISION_LISTENING,
    MISSION_VISION_ACKING, MISSION_VISION_STOPPING, MISSION_VISION_MODEL_QUERYING };
typedef enum { MISSION_VISION_SCENE_PLATFORM = 1, MISSION_VISION_SCENE_STAIR,
    MISSION_VISION_SCENE_SMALL_DISC, MISSION_VISION_SCENE_DEPOT_DIGIT,
    MISSION_VISION_SCENE_BLOCK_DIGIT } mission_vision_scene_t;
typedef enum { MISSION_FAULT_NONE, MISSION_FAULT_TIMEOUT, MISSION_FAULT_CHASSIS,
    MISSION_FAULT_ARM, MISSION_FAULT_VISION, MISSION_FAULT_STORAGE,
    MISSION_FAULT_QUEUE, MISSION_FAULT_PROTOCOL } mission_fault_t;
typedef enum { MULT_UART_OP_READ, MULT_UART_OP_WRITE, MULT_UART_OP_WRITE_READ } mult_uart_operation_t;
typedef unsigned mult_uart_status_t;
typedef enum { ZDT_TURNTABLE_DIR_CW, ZDT_TURNTABLE_DIR_CCW } zdt_turntable_direction_t;
typedef struct {
    unsigned phase; bool inflight, completion_pending, stop_requested;
    uint16_t session_id, next_session_id, mail_len;
    uint8_t next_sequence, tx[32], mail_data[32];
    nano_vision_scene_t scene; mult_uart_status_t mail_status;
} mission_vision_t;
typedef struct {
    mission_state_t state; mission_stair_layer_t stair_layer;
    uint32_t deadline_tick; uint16_t request_id;
    mission_vision_t vision; nano_vision_block_result_t block_result;
    bool block_result_received, arm_home_ready, block_model_ready;
    uint8_t block_stage, block_step, block_point, block_digit, block_found_mask, block_placed_mask;
    uint8_t active_arm_group, platform_attempts, fault_code, current_slot;
    uint8_t depot_position, depot_digit, depot_column, depot_target_slot, depot_first_digit;
    bool depot_preparing; uint32_t arm_last_action_report; void *task;
    ball_manifest_t manifest;
} mission_context_t;
#if TEST_MODE
static struct { bool block_move, stop_requested; unsigned depot_position, expected; } g_wireless_test;
static mission_context_t *pump_context;
static bool chassis_queued;
static chassis_mission_event_t queued_event;
static mission_state_t stop_at_state;
enum { CHASSIS_MISSION_FLAG_EVENT = 1, MISSION_FLAG_VISION_DONE = 16,
       MISSION_ALL_FLAGS = 31, osFlagsWaitAny = 0, osOK = 0,
       MISSION_TEST_STAGE_DONE = 6, DEBUG_UART1_RX_BUFFER_SIZE = 128 };
#define osFlagsError 0x80000000U
static int mission_event_queue;
static uint32_t osThreadFlagsWait(unsigned mask, unsigned options, uint32_t ticks);
static int osMessageQueueGet(int queue, chassis_mission_event_t *event, void *priority, uint32_t ticks);
static bool mission_test_take_command(char *command, size_t capacity);
static bool mission_test_handle_aux_command(mission_context_t *c, const char *command);
static void mission_handle_chassis(mission_context_t *c, const chassis_mission_event_t *event);
static void mission_handle_arm(mission_context_t *c, bool success);
static void mission_handle_vision(mission_context_t *c);
static void mission_test_write(const char *text) { (void)text; }
#endif
volatile mission_color_t g_mission_side = MISSION_COLOR_RED;
static unsigned groups[32], group_count, commands[32], move_count, flags;
static unsigned sessions, arm_stops, vision_stops, depot_calls, ack_count;
static uint32_t now;
static bool arm_ok, queue_ok, io_ok;
static uint32_t osKernelGetTickCount(void) { return now; }
static uint32_t mission_ms_to_ticks(uint32_t ms) { return ms; }
static uint32_t osThreadFlagsSet(void *task, uint32_t value)
{ assert(task); flags |= value; return flags; }
static void mission_enter_state(mission_context_t *c, mission_state_t state, uint32_t ms)
{ c->state = state; c->deadline_tick = ms ? now + mission_ms_to_ticks(ms) : 0; }
static lsc16_status_t arm_run(uint8_t group, uint16_t repeats,
    void (*done)(void *, uint32_t, lsc16_status_t), void *c)
{
    (void)done; (void)c; assert(repeats == 1 && group_count < 32);
    if (!arm_ok) return LSC16_ERR_IO;
    groups[group_count++] = group; return LSC16_OK;
}
static lsc16_status_t arm_stop(void *done, void *c)
{ (void)done; (void)c; ++arm_stops; return LSC16_OK; }
static void turn_stop(void *done, void *c) { (void)done; (void)c; }
static bool mission_send_chassis(mission_command_type_t command, uint16_t id)
{
    assert(id && move_count < 32);
    if (!queue_ok) return false;
#if TEST_MODE
    if (pump_context) {
        queued_event.request_id = id; queued_event.is_ready = 1;
        queued_event.type = command == MISSION_CMD_DEPOT_OK ? CHASSIS_CMD_HOME_READY :
            (command == MISSION_CMD_STOP ? CHASSIS_CMD_STOPPED :
                (uint8_t)(CHASSIS_CMD_DEPOT_1_READY + command - MISSION_CMD_GO_DEPOT_1));
        chassis_queued = true;
    }
#endif
    commands[move_count++] = command; return true;
}
static uint8_t mission_next_vision_sequence(mission_vision_t *v) { return ++v->next_sequence; }
static nano_vision_status_t mission_map_vision_status(mult_uart_status_t status)
{ return (nano_vision_status_t)status; }
static nano_vision_status_t mission_submit_vision_transfer(mission_context_t *c,
    mult_uart_operation_t operation, size_t len, uint32_t timeout)
{
    (void)operation; (void)timeout;
    assert(!c->vision.inflight && !c->vision.completion_pending);
    if (!io_ok) return NANO_VISION_ERR_IO;
    if (len) {
        nano_vision_frame_t f;
        assert(nano_vision_decode_frame(c->vision.tx, len, &f) == NANO_VISION_OK);
        if (c->vision.phase == MISSION_VISION_STARTING) {
            static const unsigned order[3][3] = {{1,2,3},{3,2,1},{3,2,1}};
            assert(c->depot_position == order[c->block_stage][c->block_step]);
            ++sessions;
        }
        if (c->vision.phase == MISSION_VISION_ACKING) ++ack_count;
        if (c->vision.phase == MISSION_VISION_STOPPING) ++vision_stops;
    }
    c->vision.inflight = true; return NANO_VISION_OK;
}
static void mission_start_run(mission_context_t *c) { (void)c; assert(false); }
static void mission_start_stair_layer(mission_context_t *c) { (void)c; assert(false); }
static void mission_start_stair_exit(mission_context_t *c) { (void)c; assert(false); }
static void mission_start_small_disc_exit(mission_context_t *c) { (void)c; assert(false); }
static uint8_t mission_stair_grasp_group(mission_stair_layer_t layer)
{ (void)layer; assert(false); return 0; }
#if !TEST_MODE
static void mission_depot_prepare(mission_context_t *c)
{ assert(c->depot_position == 1 && c->arm_home_ready); ++depot_calls; c->state = MISSION_STATE_DEPOT_SEEK; }
static void mission_depot_start_digit(mission_context_t *c) { (void)c; assert(false); }
static void mission_depot_next_ball(mission_context_t *c) { (void)c; assert(false); }
static void mission_depot_digit_done(mission_context_t *c) { (void)c; assert(false); }
static void mission_depot_arm_done(mission_context_t *c) { (void)c; assert(false); }
static bool mission_advance_slot(mission_context_t *c, zdt_turntable_direction_t dir, uint8_t *fine)
{ (void)c; (void)dir; (void)fine; assert(false); return false; }
static void mission_try_ready(mission_context_t *c) { (void)c; assert(false); }
#if TEST_TRACE
static bool mission_stair_trace_active(const mission_context_t *c) { (void)c; return false; }
#endif
#endif
#include "block_move_declarations.inc"
#include "block_move_under_test.inc"

#if TEST_MODE
/* 执行真实无线事件循环：替身仅产生设备回报，不替代抓放决策。 */
static uint32_t osThreadFlagsWait(unsigned mask, unsigned options, uint32_t ticks)
{
    (void)mask; (void)options; now += ticks;
    mission_context_t *c = pump_context; assert(c);
    if (c->vision.inflight) {
        size_t len = 0;
        if (c->vision.phase == MISSION_VISION_STARTING) {
            nano_vision_session_t r = {c->vision.session_id, NANO_VISION_SCENE_BLOCK_DIGIT, NANO_VISION_COLOR_ANY};
            assert(nano_vision_build_session_ready_frame(1, &r, c->vision.mail_data, 32, &len) == NANO_VISION_OK);
        } else if (c->vision.phase == MISSION_VISION_LISTENING) {
            nano_vision_block_result_t r = {c->vision.session_id, NANO_VISION_BLOCK_DIGIT,
                (uint8_t)(c->block_stage + 1), 90, NANO_VISION_REASON_CONFIRMED, 21, 20, 3, 192};
            assert(nano_vision_build_block_result_frame(1, &r, c->vision.mail_data, 32, &len) == NANO_VISION_OK);
        } else if (c->vision.phase == MISSION_VISION_STOPPING) {
            assert(nano_vision_build_session_stopped_frame(1, c->vision.session_id,
                c->vision.mail_data, 32, &len) == NANO_VISION_OK);
        }
        c->vision.mail_len = (uint16_t)len; c->vision.mail_status = NANO_VISION_OK;
        c->vision.inflight = false; c->vision.completion_pending = true;
        flags |= MISSION_FLAG_VISION_DONE;
    }
    if (c->state == MISSION_STATE_BLOCK_PREPARE || c->state == MISSION_STATE_BLOCK_WAIT_POSE ||
        c->state == MISSION_STATE_BLOCK_WAIT_GRASP || c->state == MISSION_STATE_BLOCK_WAIT_PLACE ||
        c->state == MISSION_STATE_BLOCK_FINISH) {
        lsc16_report_t r = {0}; r.action_group = c->active_arm_group;
        mission_arm_report(c, LSC16_REPORT_EVENT_ACTION_COMPLETED, &r);
    }
    if (chassis_queued) flags |= CHASSIS_MISSION_FLAG_EVENT;
    unsigned result = flags; flags = 0;
    return result ? result : osFlagsError;
}
static int osMessageQueueGet(int queue, chassis_mission_event_t *event, void *priority, uint32_t ticks)
{
    (void)queue; (void)priority; (void)ticks;
    if (!chassis_queued) return 1;
    *event = queued_event; chassis_queued = false; return osOK;
}
static bool mission_test_take_command(char *command, size_t capacity)
{
    assert(capacity >= 5);
    if (stop_at_state != MISSION_STATE_BOOT && pump_context->state == stop_at_state &&
        !g_wireless_test.stop_requested) { strcpy(command, "STOP"); return true; }
    return false;
}
static bool mission_test_handle_aux_command(mission_context_t *c, const char *command)
{
    assert(strcmp(command, "STOP") == 0);
    mission_handle_command(c, MISSION_USER_COMMAND_STOP);
    g_wireless_test.stop_requested = true; return true;
}
static void mission_handle_chassis(mission_context_t *c, const chassis_mission_event_t *event)
{ block_handle_chassis(c, event); }
static void mission_handle_arm(mission_context_t *c, bool success) { block_handle_arm(c, success); }
static void mission_handle_vision(mission_context_t *c)
{
    block_complete_io(c);
    if (c->state == MISSION_STATE_BLOCK_WAIT_DIGIT &&
        (c->vision.phase == MISSION_VISION_STARTING || c->vision.phase == MISSION_VISION_LISTENING))
        mission_handle_block_result(c);
}
#endif

static void reset(mission_context_t *c)
{
    memset(c, 0, sizeof(*c)); c->task = c; c->request_id = 42;
    c->current_slot = 8; c->arm_home_ready = true; c->state = MISSION_STATE_WAIT_DEPOT_1;
    assert(ball_manifest_append(&c->manifest, BALL_MANIFEST_REGION_TURNTABLE,
        BALL_MANIFEST_COLOR_RED, 0x23, 2, 3, 2) == BALL_MANIFEST_OK);
    assert(ball_manifest_append_read_failed(&c->manifest, BALL_MANIFEST_REGION_STAIR,
        BALL_MANIFEST_COLOR_RED, 5) == BALL_MANIFEST_OK);
    group_count = move_count = flags = sessions = arm_stops = vision_stops = depot_calls = ack_count = 0;
    now = 100; arm_ok = queue_ok = io_ok = true;
#if TEST_MODE
    memset(&g_wireless_test, 0, sizeof(g_wireless_test)); g_wireless_test.block_move = true;
    pump_context = NULL; chassis_queued = false; stop_at_state = MISSION_STATE_BOOT;
#endif
}
static void begin(mission_context_t *c)
{
#if TEST_MODE
    mission_block_begin(c);
#else
    chassis_mission_event_t event = {c->request_id, CHASSIS_CMD_DEPOT_1_READY, 1};
    block_handle_chassis(c, &event);
#endif
}
static void finish_arm(mission_context_t *c)
{
    unsigned moves_before = move_count, sessions_before = sessions;
    lsc16_report_t r = {0}; r.action_group = (uint8_t)(c->active_arm_group + 1);
    mission_arm_report(c, LSC16_REPORT_EVENT_ACTION_COMPLETED, &r);
    assert(flags == 0); /* 错组号不能推进，UART发送成功也不能当动作完成。 */
    mission_arm_tx_done(c, 1, LSC16_OK);
    assert(move_count == moves_before && sessions == sessions_before && flags == 0);
    r.action_group = c->active_arm_group;
    mission_arm_report(c, LSC16_REPORT_EVENT_ACTION_COMPLETED, &r);
    assert(flags == MISSION_FLAG_ARM_OK); flags = 0;
    block_handle_arm(c, true);
}
static void reached(mission_context_t *c)
{
    mission_state_t state = c->state;
    uint8_t point = c->block_point;
    uint8_t type = state == MISSION_STATE_BLOCK_WAIT_HOME ? CHASSIS_CMD_HOME_READY :
        (uint8_t)(CHASSIS_CMD_DEPOT_1_READY + point - 1);
    chassis_mission_event_t e = {(uint16_t)(c->request_id - 1), type, 1};
    block_handle_chassis(c, &e); assert(c->state == state);
    e.request_id = c->request_id; e.type = CHASSIS_CMD_PLATFORM_READY;
    block_handle_chassis(c, &e); assert(c->state == state);
    e.type = type; block_handle_chassis(c, &e);
}
static void ready(mission_context_t *c)
{
    nano_vision_session_t r = {c->vision.session_id, NANO_VISION_SCENE_BLOCK_DIGIT, NANO_VISION_COLOR_ANY};
    size_t len;
    assert(nano_vision_build_session_ready_frame(1, &r, c->vision.mail_data, 32, &len) == NANO_VISION_OK);
    c->vision.mail_len = (uint16_t)len; c->vision.inflight = false;
    mission_handle_block_result(c);
    assert(c->vision.phase == MISSION_VISION_LISTENING && c->deadline_tick == now + 8000);
}
static void result(mission_context_t *c, unsigned digit, bool bad_ack)
{
    nano_vision_block_result_t r = {c->vision.session_id,
        digit ? NANO_VISION_BLOCK_DIGIT : NANO_VISION_BLOCK_NO_VALID,
        (uint8_t)digit, digit ? 90 : 0,
        digit ? NANO_VISION_REASON_CONFIRMED : NANO_VISION_REASON_NO_CANDIDATE,
        21, 20, digit ? 3 : 20, digit ? 192 : 1500};
    size_t len; unsigned before = group_count;
    assert(nano_vision_build_block_result_frame(1, &r, c->vision.mail_data, 32, &len) == NANO_VISION_OK);
    c->vision.mail_len = (uint16_t)len;
    mission_handle_block_result(c);
    assert(c->vision.phase == MISSION_VISION_ACKING && group_count == before);
    mission_vision_process(c); assert(group_count == before); /* ACK仍在途。 */
    c->vision.inflight = false; c->vision.completion_pending = true;
    mission_vision_process(c); assert(group_count == before); /* 回调邮箱尚未消费。 */
    c->vision.mail_status = bad_ack ? NANO_VISION_ERR_IO : NANO_VISION_OK;
    block_complete_io(c); mission_vision_process(c);
    if (bad_ack) assert(c->state == MISSION_STATE_FAULT && group_count == before);
}
static void run(mission_context_t *c, const unsigned digits[3], const unsigned found[3])
{
    unsigned bounds = 0;
    while (c->state != MISSION_STATE_COMPLETE && c->state != MISSION_STATE_DEPOT_SEEK) {
        assert(++bounds < 100);
        switch (c->state) {
        case MISSION_STATE_BLOCK_PREPARE:
        case MISSION_STATE_BLOCK_WAIT_POSE:
        case MISSION_STATE_BLOCK_WAIT_GRASP:
        case MISSION_STATE_BLOCK_WAIT_PLACE:
        case MISSION_STATE_BLOCK_FINISH:
            if (c->state == MISSION_STATE_BLOCK_WAIT_PLACE)
                assert((c->block_placed_mask & (1U << (2 - c->block_stage))) == 0);
            finish_arm(c); break;
        case MISSION_STATE_BLOCK_WAIT_POSITION:
        case MISSION_STATE_BLOCK_WAIT_D4:
        case MISSION_STATE_BLOCK_RETURN_DEPOT:
        case MISSION_STATE_BLOCK_WAIT_HOME:
            assert(c->vision.phase == MISSION_VISION_IDLE);
            reached(c); break;
        case MISSION_STATE_BLOCK_SETTLE: {
            unsigned before = sessions;
            now = c->deadline_tick - 1; mission_check_timeout(c); assert(sessions == before);
            ++now; mission_check_timeout(c); assert(sessions == before + 1);
            ready(c); break;
        }
        case MISSION_STATE_BLOCK_WAIT_DIGIT:
            result(c, found[c->block_stage] == c->depot_position ? digits[c->block_stage] : 0, false);
            break;
        default: assert(false);
        }
    }
}
static void verify_run(const unsigned digits[3], const unsigned found[3], bool home_cache)
{
    mission_context_t c; reset(&c); c.arm_home_ready = home_cache;
    ball_manifest_t manifest = c.manifest;
    begin(&c); run(&c, digits, found);
    assert(memcmp(&manifest, &c.manifest, sizeof(manifest)) == 0 && c.current_slot == 8);
    unsigned expected[16], n = 0, mask = 0;
    if (!home_cache) expected[n++] = 10;
    for (unsigned stage = 0; stage < 3; ++stage) {
        expected[n++] = 28 + stage;
        if (digits[stage]) {
            static const unsigned grabs[] = {31, 33, 35}, places[] = {36, 34, 32};
            expected[n++] = grabs[stage]; expected[n++] = places[digits[stage] - 1];
            mask |= 1U << (2 - stage);
        }
    }
    expected[n++] = 10;
    assert(group_count == n && memcmp(expected, groups, n * sizeof(unsigned)) == 0);
    assert(c.block_found_mask == mask && c.block_placed_mask == mask);
    assert(c.depot_position == 1 && c.arm_home_ready && c.vision.phase == MISSION_VISION_IDLE);
#if TEST_MODE
    assert(depot_calls == 0 && c.state == MISSION_STATE_COMPLETE);
    assert(commands[move_count - 2] == MISSION_CMD_GO_DEPOT_1);
    assert(commands[move_count - 1] == MISSION_CMD_DEPOT_OK);
#else
    assert(depot_calls == 1 && commands[move_count - 1] == MISSION_CMD_GO_DEPOT_1);
    chassis_mission_event_t repeat = {c.request_id, CHASSIS_CMD_DEPOT_1_READY, 1};
    block_handle_chassis(&c, &repeat); assert(depot_calls == 1);
#endif
    assert(arm_stops == 0 && vision_stops == 0 && ack_count == sessions);
}
int main(void)
{
    /* 单块覆盖9种源层/目标层，另两层完整扫描；数字均为业务期望。 */
    for (unsigned source = 0; source < 3; ++source) {
        for (unsigned target = 1; target <= 3; ++target) {
            unsigned digits[3] = {0}, found[3] = {0};
            digits[source] = target; found[source] = source + 1;
            verify_run(digits, found, true);
        }
    }
    const unsigned permutations[][3] = {{1,2,3},{1,3,2},{2,1,3},{2,3,1},{3,1,2},{3,2,1}};
    const unsigned found[] = {2,3,1}, empty[] = {0,0,0};
    for (unsigned i = 0; i < 6; ++i) verify_run(permutations[i], found, i != 0);
    verify_run(empty, empty, true);
    /* READY后超过原4秒仍接收新鲜结果；8秒边界无结果则停车。 */
    mission_context_t c; reset(&c); begin(&c); finish_arm(&c);
    now = c.deadline_tick; mission_check_timeout(&c); ready(&c);
    now += 5000; mission_check_timeout(&c);
    assert(c.state == MISSION_STATE_BLOCK_WAIT_DIGIT && arm_stops == 0);
    result(&c, 2, false);
    assert(c.state == MISSION_STATE_BLOCK_WAIT_GRASP && c.active_arm_group == 31);
    reset(&c); begin(&c); finish_arm(&c);
    now = c.deadline_tick; mission_check_timeout(&c); ready(&c);
    now = c.deadline_tick - 1; mission_check_timeout(&c);
    assert(c.state == MISSION_STATE_BLOCK_WAIT_DIGIT && arm_stops == 0);
    ++now; mission_check_timeout(&c);
    assert(c.state == MISSION_STATE_FAULT && c.fault_code == MISSION_FAULT_TIMEOUT);
    assert(arm_stops == 1 && c.block_found_mask == 0 && c.block_placed_mask == 0);
    /* 已确认数字也不能在ACK失败后夹取。 */
    reset(&c); begin(&c); finish_arm(&c);
    now = c.deadline_tick; mission_check_timeout(&c); ready(&c);
    result(&c, 2, true); assert(c.block_found_mask == 0 && c.block_placed_mask == 0 && arm_stops == 1);
    /* 相机/模型可在READY之前报告FAULT；ACK消费后必须停车，不能当空层跳过。 */
    reset(&c); begin(&c); finish_arm(&c); now = c.deadline_tick; mission_check_timeout(&c);
    c.vision.inflight = false;
    nano_vision_block_result_t camera_fault = {c.vision.session_id, NANO_VISION_BLOCK_FAULT,
        0, 0, NANO_VISION_REASON_CAMERA_ERROR, 0, 0, 0, 0};
    size_t fault_length;
    assert(nano_vision_build_block_result_frame(1, &camera_fault, c.vision.mail_data, 32, &fault_length) == NANO_VISION_OK);
    c.vision.mail_len = (uint16_t)fault_length;
    mission_handle_block_result(&c);
    assert(c.vision.phase == MISSION_VISION_ACKING && c.block_result_received);
    c.vision.inflight = false; c.vision.completion_pending = true; c.vision.mail_status = NANO_VISION_OK;
    block_complete_io(&c); mission_vision_process(&c);
    assert(c.state == MISSION_STATE_FAULT && group_count == 1 && move_count == 1);
    assert(c.block_placed_mask == 0 && arm_stops == 1 && commands[0] == MISSION_CMD_STOP);
    /* STOP和期限保护覆盖每种等待，迟到回报不继续、不自动松夹或收臂。 */
    const mission_state_t waits[] = {MISSION_STATE_BLOCK_PREPARE, MISSION_STATE_BLOCK_WAIT_POSE,
        MISSION_STATE_BLOCK_WAIT_POSITION, MISSION_STATE_BLOCK_WAIT_DIGIT,
        MISSION_STATE_BLOCK_WAIT_GRASP, MISSION_STATE_BLOCK_WAIT_D4,
        MISSION_STATE_BLOCK_WAIT_PLACE, MISSION_STATE_BLOCK_FINISH,
        MISSION_STATE_BLOCK_RETURN_DEPOT, MISSION_STATE_BLOCK_WAIT_HOME};
    for (unsigned i = 0; i < sizeof(waits)/sizeof(waits[0]); ++i) {
        reset(&c); c.block_stage = 1; c.active_arm_group = 33;
        mission_enter_state(&c, waits[i], 30000);
        mission_handle_command(&c, MISSION_USER_COMMAND_STOP);
        assert(c.state == MISSION_STATE_STOPPING && arm_stops == 1 && c.active_arm_group == 0);
        lsc16_report_t r = {0}; r.action_group = 33;
        mission_arm_report(&c, LSC16_REPORT_EVENT_ACTION_COMPLETED, &r);
        assert(flags == 0); block_handle_arm(&c, true); assert(c.block_placed_mask == 0);
        chassis_mission_event_t e = {c.request_id, CHASSIS_CMD_STOPPED, 1};
        block_handle_chassis(&c, &e); assert(c.state == MISSION_STATE_STOPPED);
        reset(&c); c.block_stage = 1; mission_enter_state(&c, waits[i], 30000);
        now = c.deadline_tick; mission_check_timeout(&c);
        assert(c.state == MISSION_STATE_FAULT && c.fault_code == MISSION_FAULT_TIMEOUT);
        assert(arm_stops == 1 && c.block_placed_mask == 0);
    }
    reset(&c); arm_ok = false; begin(&c); assert(c.state == MISSION_STATE_FAULT && group_count == 0);
    reset(&c); begin(&c); finish_arm(&c); now = c.deadline_tick; io_ok = false;
    mission_check_timeout(&c); assert(c.state == MISSION_STATE_FAULT && sessions == 0);
    reset(&c); begin(&c); finish_arm(&c); now = c.deadline_tick; mission_check_timeout(&c); ready(&c);
    result(&c, 3, false); assert(c.state == MISSION_STATE_BLOCK_WAIT_GRASP && move_count == 0);
    queue_ok = false; finish_arm(&c); assert(c.state == MISSION_STATE_FAULT && c.block_placed_mask == 0);
    /* 到D4前不放置；实际放置动作停止/失败也不能标记完成或切下一层。 */
    reset(&c); begin(&c); finish_arm(&c); now = c.deadline_tick; mission_check_timeout(&c); ready(&c);
    result(&c, 1, false); finish_arm(&c);
    assert(c.state == MISSION_STATE_BLOCK_WAIT_D4 && groups[group_count - 1] == 31);
    reached(&c); assert(c.state == MISSION_STATE_BLOCK_WAIT_PLACE && groups[group_count - 1] == 36);
    lsc16_report_t stopped = {0}; stopped.action_group = 36;
    mission_arm_report(&c, LSC16_REPORT_EVENT_ACTION_STOPPED, &stopped);
    assert(flags == MISSION_FLAG_ARM_FAIL); flags = 0;
    block_handle_arm(&c, false);
    assert(c.state == MISSION_STATE_FAULT && c.block_placed_mask == 0 && group_count == 3);
    /* 匹配请求的失败到位回报必须停车，不能运行放置。 */
    reset(&c); begin(&c); finish_arm(&c); now = c.deadline_tick; mission_check_timeout(&c); ready(&c);
    result(&c, 3, false); finish_arm(&c);
    chassis_mission_event_t failed = {c.request_id, CHASSIS_CMD_DEPOT_4_READY, 0};
    block_handle_chassis(&c, &failed);
    assert(c.state == MISSION_STATE_FAULT && group_count == 2 && c.block_placed_mask == 0);
#if TEST_MODE
    reset(&c); pump_context = &c;
    assert(mission_test_run_block_move(&c));
    assert(c.state == MISSION_STATE_COMPLETE && c.depot_position == 1 && c.block_placed_mask == 7);
    assert(commands[move_count - 2] == MISSION_CMD_GO_DEPOT_1 && commands[move_count - 1] == MISSION_CMD_DEPOT_OK);
    assert(ack_count == 3 && depot_calls == 0 && !chassis_queued);
    /* STOP正好遇到结果/动作回调时，循环仍须关闭会话、不夹取或标记放置。 */
    const mission_state_t stop_states[] = {MISSION_STATE_BLOCK_WAIT_DIGIT, MISSION_STATE_BLOCK_WAIT_GRASP,
        MISSION_STATE_BLOCK_WAIT_D4, MISSION_STATE_BLOCK_WAIT_PLACE};
    for (unsigned i = 0; i < sizeof(stop_states)/sizeof(stop_states[0]); ++i) {
        reset(&c); pump_context = &c; stop_at_state = stop_states[i];
        assert(!mission_test_run_block_move(&c));
        assert(c.state == MISSION_STATE_STOPPING && c.vision.phase == MISSION_VISION_IDLE);
        assert(c.block_placed_mask == 0 && arm_stops == 1);
        assert(chassis_queued && queued_event.type == CHASSIS_CMD_STOPPED);
        if (stop_states[i] == MISSION_STATE_BLOCK_WAIT_DIGIT) assert(group_count == 1 && vision_stops == 1);
    }
#endif
    puts("Block move mappings, ACK/action/position gates, STOP and handoff passed (host only)");
    return 0;
}
