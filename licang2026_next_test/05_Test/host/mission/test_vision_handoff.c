/* 直接测试正式视觉启动/推进函数；设备替身只记录提交，不伪造实机结论。 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include <limits.h>
#include "mission_app.h"
#include "mission_config.h"
#include "nano_vision_core.h"
#undef MISSION_CHASSIS_ROUTE_TEST_ENABLED
#define MISSION_CHASSIS_ROUTE_TEST_ENABLED 0
#undef MISSION_DEPOT_TRACE_ENABLED
#define MISSION_DEPOT_TRACE_ENABLED TEST_TRACE
#if TEST_TRACE
static unsigned log_lines;
static char log_text[128];
static void capture_log(const char *format, ...)
{
    va_list args; va_start(args, format);
    int length = vsnprintf(log_text, sizeof(log_text), format, args);
    va_end(args);
    if (length < 0 || length >= (int)sizeof(log_text)) abort();
    ++log_lines;
}
#define DEPOT_TRACE(...) capture_log(__VA_ARGS__)
#else
#define DEPOT_TRACE(...) ((void)0)
#endif
#define CHECK(c) do { if (!(c)) { fprintf(stderr, "FAIL line %u: %s\n", __LINE__, #c); exit(1); } } while (0)
enum { MISSION_VISION_IDLE, MISSION_VISION_STARTING, MISSION_VISION_LISTENING,
       MISSION_VISION_ACKING, MISSION_VISION_STOPPING };
typedef enum { MISSION_VISION_SCENE_PLATFORM = 1, MISSION_VISION_SCENE_STAIR,
               MISSION_VISION_SCENE_SMALL_DISC, MISSION_VISION_SCENE_DEPOT_DIGIT,
               MISSION_VISION_SCENE_BLOCK_DIGIT } mission_vision_scene_t;
enum { MULT_UART_OP_READ, MULT_UART_OP_WRITE_READ };
enum { MISSION_FAULT_ARM, MISSION_FAULT_PROTOCOL, MISSION_FAULT_QUEUE, MISSION_FAULT_VISION };
enum { MISSION_CMD_CAM_READY, CHASSIS_CMD_STAIR_PAUSE };
typedef struct { unsigned type; } chassis_mission_event_t;
typedef struct {
    unsigned phase;
    bool inflight, completion_pending;
    uint16_t next_session_id, session_id;
    uint8_t next_sequence;
    nano_vision_scene_t scene;
    uint8_t tx[NANO_VISION_FRAME_MAX];
} mission_vision_t;
typedef struct {
    mission_vision_t vision;
    mission_state_t state;
    uint32_t deadline_tick;
    mission_stair_layer_t stair_layer;
    uint8_t depot_digit;
    uint8_t stair_balls, active_arm_group;
    uint32_t arm_last_action_report;
    uint16_t request_id;
} mission_context_t;
volatile mission_color_t g_mission_side = MISSION_COLOR_RED;
static unsigned submits, resets, advances, arms, cam_ready;
#if TEST_TRACE
static uint32_t tick;
static uint32_t osKernelGetTickCount(void) { return tick; }
static uint32_t osKernelGetTickFreq(void) { return 1000; }
#endif
#include "stair_diag_under_test.inc"
static uint32_t last_timeout;
static uint8_t mission_next_vision_sequence(mission_vision_t *v) { return ++v->next_sequence; }
static void mission_reset_vision(mission_context_t *c) { STAIR_SESSION(c, false); ++resets; c->vision.phase = MISSION_VISION_IDLE; }
static void mission_enter_state(mission_context_t *c, mission_state_t state, uint32_t timeout)
{ c->deadline_tick = timeout; c->state = state; }
static nano_vision_status_t mission_submit_vision_transfer(mission_context_t *c, unsigned op, size_t len, uint32_t timeout)
{ (void)op; (void)len; ++submits; last_timeout = timeout; c->vision.inflight = true; return NANO_VISION_OK; }
static void mission_depot_digit_done(mission_context_t *c) { (void)c; ++advances; }
static uint8_t mission_stair_grasp_group(mission_stair_layer_t layer);
static bool mission_send_chassis(unsigned command, uint16_t id)
{ (void)id; CHECK(command == MISSION_CMD_CAM_READY); ++cam_ready; return true; }
static bool mission_start_arm(mission_context_t *c, uint8_t group, mission_state_t state)
{ ++arms; c->active_arm_group = group; c->state = state; return true; }
static void mission_fail(mission_context_t *c, unsigned fault) { (void)fault; c->state = MISSION_STATE_FAULT; }
#include "vision_under_test.inc"

int main(void)
{
    mission_context_t c = {0};
    /* ACK回调刚到，主循环还没检查失败/成功：不得直接推进第二次数字会话。 */
    c.state = MISSION_STATE_DEPOT_WAIT_DIGIT;
    c.vision.phase = MISSION_VISION_ACKING;
    c.vision.completion_pending = true;
    mission_vision_process(&c);
    CHECK(advances == 0 && resets == 0 && submits == 0);
    /* 数字帧尚未解析时，不得提交下一次READ并覆盖单份回报mailbox。 */
    c.vision.phase = MISSION_VISION_LISTENING;
    mission_vision_process(&c);
    CHECK(submits == 0);
    /* Mission消费成功ACK后才放行；在途期间同样不得推进。 */
    c.vision.phase = MISSION_VISION_ACKING;
    c.vision.completion_pending = false; c.vision.inflight = true;
    mission_vision_process(&c); CHECK(advances == 0);
    c.vision.inflight = false;
    mission_vision_process(&c); CHECK(advances == 1 && resets == 1);
    /* C100启动没有总期限；每笔短读释放总线，超时后继续等同一READY。 */
    memset(&c, 0, sizeof(c));
    CHECK(mission_start_vision(&c, MISSION_VISION_SCENE_DEPOT_DIGIT,
                              MISSION_STAIR_NONE, MISSION_STATE_DEPOT_WAIT_DIGIT));
    CHECK(last_timeout == MISSION_VISION_TIMEOUT_MS && c.deadline_tick == 0);
    for (unsigned i = 0; i < 40; ++i) {
        c.vision.inflight = false;
        c.vision.completion_pending = false;
        unsigned before = submits;
        mission_vision_process(&c);
        CHECK(submits == before + 1 && last_timeout == MISSION_VISION_READ_TIMEOUT_MS);
        CHECK(c.vision.phase == MISSION_VISION_STARTING && c.vision.session_id == 1);
    }
    memset(&c, 0, sizeof(c));
    CHECK(mission_start_vision(&c, MISSION_VISION_SCENE_BLOCK_DIGIT,
                              MISSION_STAIR_NONE, MISSION_STATE_BLOCK_WAIT_DIGIT));
    CHECK(c.deadline_tick == MISSION_BLOCK_PREPARE_TIMEOUT_MS);
    c.vision.inflight = false;
    unsigned before_block = submits;
    mission_vision_process(&c);
    CHECK(submits == before_block + 1 && last_timeout == MISSION_VISION_READ_TIMEOUT_MS);
    memset(&c, 0, sizeof(c));
    CHECK(mission_start_vision(&c, MISSION_VISION_SCENE_PLATFORM,
                              MISSION_STAIR_NONE, MISSION_STATE_PLATFORM_WAIT_VISION));
    CHECK(last_timeout == MISSION_VISION_TIMEOUT_MS);
    for (unsigned layer = MISSION_STAIR_LOW; layer <= MISSION_STAIR_MID; ++layer) {
        /* 先ACK后停车：ACK只结束会话，停车回报前不得抓取。 */
        memset(&c, 0, sizeof(c));
        c.stair_layer = (mission_stair_layer_t)layer;
        c.state = MISSION_STATE_STAIR_WAIT_PAUSE; c.vision.phase = MISSION_VISION_ACKING;
        unsigned before_arms = arms;
        mission_vision_process(&c); CHECK(arms == before_arms);
        chassis_mission_event_t pause = { CHASSIS_CMD_STAIR_PAUSE };
        handle_pause(&c, &pause);
        CHECK(arms == before_arms + 1 && c.active_arm_group == 13 + layer);
        CHECK(c.state == MISSION_STATE_STAIR_WAIT_GRASP);
        /* 先停车后ACK：等待回调被消费后才抓取，同一回报不重复动作。 */
        c.state = MISSION_STATE_STAIR_WAIT_PAUSE; c.vision.phase = MISSION_VISION_ACKING;
        c.vision.completion_pending = true;
        before_arms = arms;
        handle_pause(&c, &pause); CHECK(c.state == MISSION_STATE_STAIR_WAIT_ACK);
        mission_vision_process(&c); CHECK(arms == before_arms);
        c.vision.completion_pending = false;
        mission_vision_process(&c);
        CHECK(arms == before_arms + 1 && c.active_arm_group == 13 + layer);
        handle_pause(&c, &pause); CHECK(arms == before_arms + 1);
    }
    memset(&c, 0, sizeof(c));
    mission_start_stair_layer(&c); CHECK(c.state == MISSION_STATE_STAIR_WAIT_LAYER);
    c.stair_layer = MISSION_STAIR_HIGH;
    mission_start_stair_layer(&c);
    CHECK(c.state == MISSION_STATE_STAIR_WAIT_VISION_START);
    CHECK(c.vision.scene == NANO_VISION_SCENE_STAIR_HIGH);
    c.stair_balls = 2;
    unsigned before_cam = cam_ready, before_submits = submits;
    mission_start_stair_layer(&c);
    CHECK(cam_ready == before_cam + 1 && submits == before_submits);
    CHECK(c.state == MISSION_STATE_STAIR_SCANNING);
#if TEST_TRACE
    STAIR_RESET(); c.vision.session_id = 7; c.vision.scene = NANO_VISION_SCENE_STAIR_HIGH;
    STAIR_SESSION(&c, true);
    nano_vision_event_t event = {0};
    STAIR_REJECT(&c, NANO_VISION_ERR_TIMEOUT, &event); CHECK(g_stair_diag.rejected[0] == 1);
    STAIR_REJECT(&c, NANO_VISION_OK, &event); CHECK(g_stair_diag.rejected[1] == 1);
    event.session_id = 7;
    STAIR_REJECT(&c, NANO_VISION_OK, &event); CHECK(g_stair_diag.rejected[2] == 1);
    event.observation.scene = c.vision.scene;
    STAIR_REJECT(&c, NANO_VISION_OK, &event); CHECK(g_stair_diag.rejected[3] == 1);
    event.observation.status = NANO_VISION_OBS_VALID;
    STAIR_REJECT(&c, NANO_VISION_OK, &event); CHECK(g_stair_diag.rejected[4] == 1);
    event.observation.color = NANO_VISION_COLOR_RED;
    event.observation.age_ms = 121;
    STAIR_REJECT(&c, NANO_VISION_OK, &event); CHECK(g_stair_diag.rejected[5] == 1);
    STAIR_STATE_REJECT(); STAIR_ACCEPT();
    CHECK(g_stair_diag.accepted == 1 && g_stair_events == 1);
    STAIR_SESSION(&c, false);
    c.vision.session_id = 8; STAIR_SESSION(&c, true);
    CHECK(g_stair_diag.accepted == 0 && g_stair_closed.sid == 7);
    unsigned before_logs = log_lines;
    STAIR_POLL(&c); CHECK(log_lines == before_logs + 2);
    CHECK(strstr(log_text, "SID=7") != NULL);
    STAIR_POLL(&c); CHECK(log_lines == before_logs + 2);
    tick = 1000; STAIR_POLL(&c); CHECK(log_lines == before_logs + 4);
    /* 即使计数达到32位上限，日志也不会超出正式128字节缓冲。 */
    g_stair_diag.accepted = UINT32_MAX;
    g_stair_diag.last_rejected.session_id = UINT16_MAX;
    g_stair_diag.last_rejected.observation.frame_id = UINT16_MAX;
    g_stair_diag.last_rejected.observation.age_ms = UINT16_MAX;
    for (unsigned i = 0; i < 7; ++i) g_stair_diag.rejected[i] = UINT32_MAX;
    mission_stair_summary(&g_stair_diag, true);
    c.arm_last_action_report = 0x0863; c.active_arm_group = 13;
    unsigned before_raw_arms = arms;
    STAIR_POLL(&c);
    CHECK(strstr(log_text, "GROUP=99") != NULL && strstr(log_text, "WAIT=13") != NULL);
    CHECK(arms == before_raw_arms);
#else
    unsigned side_effect = 0;
    STAIR_TRACE(&c, "%u", ++side_effect);
    CHECK(side_effect == 0);
#endif
    puts("Vision handoff regression passed (host logic only)");
    return 0;
}
