/* 实际Mission建档与存球推进+真实manifest，验证第三球不再被正式2球规则拦截。 */
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "mission_app.h"
#include "mission_config.h"
#include "ball_manifest_core.h"
#undef MISSION_CHASSIS_ROUTE_TEST_ENABLED
#define MISSION_CHASSIS_ROUTE_TEST_ENABLED TEST_MODE
#define TURN_TRACE(...) ((void)0)
#define PLATFORM_TRACE(...) ((void)0)
#include "stair_trace_disabled.h"
enum { MISSION_STORAGE_REGION_PLATFORM, MISSION_STORAGE_REGION_STAIR,
       MISSION_STORAGE_REGION_SMALL_DISC };
typedef unsigned mission_storage_region_t;
enum { MISSION_CMD_STAIR_RESUME, MISSION_CMD_SMALL_DISC_RESUME };
enum { MISSION_VISION_SCENE_STAIR, MISSION_VISION_SCENE_SMALL_DISC };
enum { MISSION_FAULT_QUEUE, MISSION_FAULT_VISION, MISSION_FAULT_ARM };
volatile mission_color_t g_mission_side = MISSION_COLOR_RED;
typedef struct {
    mission_state_t state; mission_stair_layer_t stair_layer;
    unsigned stair_balls, platform_balls, platform_attempts, small_disc_balls;
    uint8_t storage_slot, active_arm_group;
    uint16_t request_id;
    bool platform_read_ok, arm_home_ready;
    ball_manifest_t manifest;
    struct { struct { uint8_t code, row, column; } ic_ball; } storage;
} mission_context_t;
static unsigned resumes, visions;
static bool mission_send_chassis(unsigned command, unsigned id)
{ (void)command; (void)id; ++resumes; return true; }
static bool mission_start_vision(mission_context_t *ctx, unsigned scene,
    mission_stair_layer_t layer, mission_state_t state)
{ (void)scene; (void)layer; ++visions; ctx->state = state; return true; }
static void mission_enter_state(mission_context_t *ctx, mission_state_t state, unsigned ms)
{ (void)ms; ctx->state = state; }
static void mission_fail(mission_context_t *ctx, unsigned fault)
{ (void)ctx; (void)fault; assert(0); }
static bool mission_start_arm(mission_context_t *ctx, unsigned group, mission_state_t state)
{ (void)ctx; (void)group; (void)state; assert(0); return false; }
static void mission_finish_platform(mission_context_t *ctx)
{ (void)ctx; assert(0); }
#include "stair_capacity_under_test.inc"
int main(void)
{
    for (unsigned side = 0; side < 2; ++side) {
        for (unsigned failed = 0; failed < 2; ++failed) {
            mission_context_t c = {0};
            g_mission_side = side ? MISSION_COLOR_BLUE : MISSION_COLOR_RED;
            ball_manifest_init(&c.manifest);
#if TEST_MODE
            ball_manifest_init_stair_test(&c.manifest, MISSION_STAIR_TEST_BALL_COUNT);
#endif
            resumes = visions = 0;
            for (unsigned i = 0; i < MISSION_STAIR_BALL_LIMIT; ++i) {
                c.storage.ic_ball.code = 0x12; c.storage.ic_ball.row = 1;
                c.storage.ic_ball.column = 2;
                c.stair_layer = i < 3 ? MISSION_STAIR_LOW :
                    (i < 6 ? MISSION_STAIR_HIGH : MISSION_STAIR_MID);
                assert(mission_record_ball(&c, MISSION_STORAGE_REGION_STAIR, !failed));
                c.state = MISSION_STATE_STAIR_WAIT_STORAGE;
                mission_handle_storage(&c);
                assert(c.storage_slot == i + 1 && c.manifest.count == i + 1);
                assert(ball_manifest_validate(&c.manifest) == BALL_MANIFEST_OK);
                ball_manifest_record_t r;
                assert(ball_manifest_get(&c.manifest, i, &r) == BALL_MANIFEST_OK);
                assert(r.storage_slot == i && r.region == BALL_MANIFEST_REGION_STAIR);
                assert(r.state == (failed ? BALL_MANIFEST_STATE_READ_FAILED : BALL_MANIFEST_STATE_STORED));
                assert(c.state == (i + 1 == MISSION_STAIR_BALL_LIMIT ?
                    MISSION_STATE_STAIR_WAIT_RESUME : MISSION_STATE_STAIR_WAIT_VISION_RESUME));
            }
            assert(resumes == 1 && visions == MISSION_STAIR_BALL_LIMIT - 1);
            assert(!mission_record_ball(&c, MISSION_STORAGE_REGION_STAIR, !failed));
            assert(c.manifest.count == MISSION_STAIR_BALL_LIMIT);
        }
    }
    puts("Actual Mission stair records: formal 2 / wireless 8, read failures and slot history passed (host only)");
}
