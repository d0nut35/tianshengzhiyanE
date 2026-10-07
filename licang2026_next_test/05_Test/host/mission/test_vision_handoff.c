/* 直接测试正式视觉启动/推进函数；设备替身只记录提交，不伪造实机结论。 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "mission_app.h"
#include "mission_config.h"
#include "nano_vision_core.h"
#undef MISSION_CHASSIS_ROUTE_TEST_ENABLED
#define MISSION_CHASSIS_ROUTE_TEST_ENABLED 0
#define DEPOT_TRACE(...) ((void)0)
#define CHECK(c) do { if (!(c)) { fprintf(stderr, "FAIL line %u: %s\n", __LINE__, #c); exit(1); } } while (0)
enum { MISSION_VISION_IDLE, MISSION_VISION_STARTING, MISSION_VISION_LISTENING,
       MISSION_VISION_ACKING, MISSION_VISION_STOPPING };
typedef enum { MISSION_VISION_SCENE_PLATFORM = 1, MISSION_VISION_SCENE_STAIR,
               MISSION_VISION_SCENE_SMALL_DISC, MISSION_VISION_SCENE_DEPOT_DIGIT,
               MISSION_VISION_SCENE_BLOCK_DIGIT } mission_vision_scene_t;
enum { MULT_UART_OP_READ, MULT_UART_OP_WRITE_READ };
enum { MISSION_FAULT_ARM, MISSION_FAULT_PROTOCOL };
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
} mission_context_t;
volatile mission_color_t g_mission_side = MISSION_COLOR_RED;
static unsigned submits, resets, advances;
static uint32_t last_timeout;
static uint8_t mission_next_vision_sequence(mission_vision_t *v) { return ++v->next_sequence; }
static void mission_reset_vision(mission_context_t *c) { ++resets; c->vision.phase = MISSION_VISION_IDLE; }
static void mission_enter_state(mission_context_t *c, mission_state_t state, uint32_t timeout)
{ c->deadline_tick = timeout; c->state = state; }
static nano_vision_status_t mission_submit_vision_transfer(mission_context_t *c, unsigned op, size_t len, uint32_t timeout)
{ (void)op; (void)len; ++submits; last_timeout = timeout; c->vision.inflight = true; return NANO_VISION_OK; }
static void mission_depot_digit_done(mission_context_t *c) { (void)c; ++advances; }
static uint8_t mission_stair_grasp_group(mission_stair_layer_t layer) { (void)layer; return 14; }
static bool mission_start_arm(mission_context_t *c, uint8_t group, mission_state_t state)
{ (void)group; c->state = state; return true; }
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
    puts("Vision handoff regression passed (host logic only)");
    return 0;
}
