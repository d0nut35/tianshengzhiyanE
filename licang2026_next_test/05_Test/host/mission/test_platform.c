/* 实际圆盘读卡/转槽/计数函数回归；不替代机械抓取验证。 */
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "mission_app.h"
#include "mission_config.h"
#undef MISSION_CHASSIS_ROUTE_TEST_ENABLED
#define MISSION_CHASSIS_ROUTE_TEST_ENABLED TEST_MODE
#define DEPOT_TRACE(...) ((void)0)
#include "stair_trace_disabled.h"
#define PLATFORM_TRACE(...) ((void)0)
enum { IC_CARD_OK, IC_CARD_ERR_BUSY, IC_READ_TIMEOUT_MS = 1000, MISSION_FLAG_IC_DONE = 1 };
enum { LSC16_OK, LSC16_FAIL };
enum { MISSION_FAULT_ARM, MISSION_FAULT_VISION, MISSION_FAULT_QUEUE };
enum { MISSION_CMD_STAIR_RESUME, MISSION_CMD_SMALL_DISC_RESUME, MISSION_CMD_GO_STAIRS };
enum { MISSION_VISION_SCENE_PLATFORM, MISSION_VISION_SCENE_STAIR, MISSION_VISION_SCENE_SMALL_DISC };
typedef enum { MISSION_STORAGE_REGION_PLATFORM, MISSION_STORAGE_REGION_STAIR, MISSION_STORAGE_REGION_SMALL_DISC } mission_storage_region_t;
enum { ZDT_TURNTABLE_DIR_CW, ZDT_TURNTABLE_DIR_CCW };
typedef struct {
    mission_state_t state;
    mission_stair_layer_t stair_layer;
    uint8_t platform_balls, platform_attempts, stair_balls, small_disc_balls;
    uint8_t storage_slot, current_slot, active_arm_group;
    uint16_t request_id;
    bool platform_read_ok, arm_home_ready;
    struct { unsigned ic_status; } storage;
} mission_context_t;
static unsigned reads, records, turns, last_arm, visions;
static unsigned arm_commands, stairs_commands;
static bool read_success, move_success = true, arm_success = true;
static unsigned osThreadFlagsClear(unsigned f) { return f; }
static unsigned mission_ms_to_ticks(unsigned ms) { return ms; }
static void osDelay(unsigned ms) { (void)ms; }
static void mission_ic_done(void) {}
static void mission_arm_tx_done(void) {}
static unsigned ic_read(bool prompt, void (*done)(void), mission_context_t *c)
{ (void)prompt; (void)done; ++reads; c->storage.ic_status = read_success ? IC_CARD_OK : IC_CARD_ERR_BUSY; return IC_CARD_OK; }
static bool mission_wait_device(unsigned flag, unsigned ms) { (void)flag; (void)ms; return true; }
static bool mission_record_ball(mission_context_t *c, mission_storage_region_t r, bool ok)
{ (void)c; (void)r; (void)ok; ++records; return true; }
static bool mission_advance_slot(mission_context_t *c, unsigned dir, uint8_t *fine)
{ (void)c; (void)dir; (void)fine; ++turns; return move_success; }
static unsigned arm_run(uint8_t group, unsigned count, void (*done)(void), mission_context_t *c)
{ (void)count; (void)done; (void)c; assert(group != 17); ++arm_commands; last_arm = group; return arm_success ? LSC16_OK : LSC16_FAIL; }
static void mission_enter_state(mission_context_t *c, mission_state_t state, unsigned ms)
{ (void)ms; c->state = state; }
static void mission_fail(mission_context_t *c, unsigned fault) { (void)fault; c->state = MISSION_STATE_FAULT; }
static bool mission_start_vision(mission_context_t *c, unsigned scene, mission_stair_layer_t layer, mission_state_t state)
{ (void)scene; (void)layer; ++visions; c->state = state; return true; }
static bool mission_send_chassis(unsigned cmd, uint16_t id)
{ (void)id; if (cmd == MISSION_CMD_GO_STAIRS) ++stairs_commands; return true; }
#if !MISSION_CHASSIS_ROUTE_TEST_ENABLED
static uint16_t mission_next_request_id(mission_context_t *c) { return ++c->request_id; }
#endif
#include "platform_under_test.inc"
static void attempt(mission_context_t *c, bool ok)
{
    read_success = ok;
    assert(mission_start_arm(c, 12, MISSION_STATE_PLATFORM_WAIT_GRASP));
    unsigned previous_reads = reads, previous_turns = turns;
    mission_platform_prepare_storage(c);
    assert(reads == previous_reads && turns == previous_turns && !c->arm_home_ready);
    assert(c->active_arm_group == ((c->platform_balls == 4) ? 10 : 11));
    assert(c->state == ((c->platform_balls == 4) ? MISSION_STATE_PLATFORM_WAIT_AVOID : MISSION_STATE_PLATFORM_WAIT_RETURN));
    /* 模拟收到所等动作的完成回报，再进入实际读卡/转槽函数。 */
    c->arm_home_ready = c->active_arm_group == 10;
    c->state = MISSION_STATE_PLATFORM_WAIT_STORAGE;
    if (mission_store_ball(c, MISSION_STORAGE_REGION_PLATFORM)) mission_handle_storage(c);
    else c->state = MISSION_STATE_FAULT;
}
static void assert_finished(const mission_context_t *c, unsigned previous_stairs)
{
#if MISSION_CHASSIS_ROUTE_TEST_ENABLED
    assert(c->state == MISSION_STATE_COMPLETE && stairs_commands == previous_stairs);
#else
    assert(c->state == MISSION_STATE_WAIT_STAIRS && stairs_commands == previous_stairs + 1);
#endif
}
int main(void)
{
    mission_context_t c = {0};
    attempt(&c, false);
    assert(reads == 5 && records == 0 && turns == 0 && c.platform_balls == 0 && c.storage_slot == 0);
    assert(c.platform_attempts == 1 && c.state == MISSION_STATE_PLATFORM_SETTLE && visions == 0);
    for (unsigned i = 0; i < 4; ++i) attempt(&c, true);
    assert(c.platform_balls == 4 && c.platform_attempts == 5 && c.current_slot == 4);
    attempt(&c, false);
    assert(c.platform_balls == 4 && c.current_slot == 4 && last_arm == 11);
    assert(c.state == MISSION_STATE_PLATFORM_WAIT_POSE); /* 10后失败必须回11。 */
    unsigned previous_arms = arm_commands, previous_stairs = stairs_commands;
    attempt(&c, true);
    assert(c.platform_balls == 5 && c.platform_attempts == 7 && turns == 5 && records == 5);
    assert(last_arm == 10 && arm_commands == previous_arms + 2); /* 12→10，不重复10。 */
    assert_finished(&c, previous_stairs);
    memset(&c, 0, sizeof(c));
    unsigned before = turns;
    for (unsigned i = 0; i < 7; ++i) attempt(&c, false);
    assert(c.platform_balls == 0 && turns == before && c.platform_attempts == 7);
    assert(c.state == MISSION_STATE_PLATFORM_WAIT_DEPARTURE_POSE);
    memset(&c, 0, sizeof(c));
    previous_stairs = stairs_commands;
    for (unsigned i = 0; i < 5; ++i) attempt(&c, true);
    assert(c.platform_attempts == 5); assert_finished(&c, previous_stairs);
    /* 已收4球，最后球连续读卡失败至第7次：不占槽，已回10就直接结束。 */
    memset(&c, 0, sizeof(c));
    for (unsigned i = 0; i < 4; ++i) attempt(&c, true);
    attempt(&c, false); attempt(&c, false);
    previous_arms = arm_commands; previous_stairs = stairs_commands; before = turns;
    attempt(&c, false);
    assert(c.platform_balls == 4 && c.platform_attempts == 7 && turns == before);
    assert(arm_commands == previous_arms + 2); assert_finished(&c, previous_stairs);
    /* 非圆盘仍保留读卡失败记录并推进，不引入7次限制。 */
    read_success = false;
    before = turns;
    assert(mission_store_ball(&c, MISSION_STORAGE_REGION_STAIR));
    assert(mission_store_ball(&c, MISSION_STORAGE_REGION_SMALL_DISC));
    assert(turns == before + 2);
    /* PB0可选不能忽略通信/电机失败，也不能在失败时更新槽号。 */
    move_success = false;
    uint8_t prior_slot = c.current_slot;
    assert(!mission_store_ball(&c, MISSION_STORAGE_REGION_STAIR));
    assert(!mission_store_ball(&c, MISSION_STORAGE_REGION_SMALL_DISC));
    assert(c.current_slot == prior_slot);
    memset(&c, 0, sizeof(c)); move_success = false;
    attempt(&c, true);
    assert(c.state == MISSION_STATE_FAULT && c.platform_balls == 0 && c.current_slot == 0);
    /* 正式/无线小圆盘底盘退出后只提交10；仍等待动作回报，失败必须停车。 */
    memset(&c, 0, sizeof(c));
    previous_arms = arm_commands;
    mission_start_small_disc_exit(&c);
    assert(arm_commands == previous_arms + 1 && last_arm == 10);
    assert(c.state == MISSION_STATE_SMALL_DISC_WAIT_SAFE && !c.arm_home_ready);
    arm_success = false;
    mission_start_small_disc_exit(&c);
    assert(c.state == MISSION_STATE_FAULT);
    before = c.platform_attempts;
    assert(!mission_start_arm(&c, 12, MISSION_STATE_PLATFORM_WAIT_GRASP));
    assert(c.platform_attempts == before);
    puts("Platform retry regression passed (host logic only)");
    return 0;
}
