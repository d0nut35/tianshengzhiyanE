/* 实际圆盘读卡/转槽/计数函数回归；不替代机械抓取验证。 */
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "mission_app.h"
#include "mission_config.h"
#define DEPOT_TRACE(...) ((void)0)
#define PLATFORM_TRACE(...) ((void)0)
enum { IC_CARD_OK, IC_CARD_ERR_BUSY, IC_READ_TIMEOUT_MS = 1000, MISSION_FLAG_IC_DONE = 1 };
enum { LSC16_OK, LSC16_FAIL };
enum { MISSION_FAULT_ARM, MISSION_FAULT_VISION, MISSION_FAULT_QUEUE };
enum { MISSION_CMD_STAIR_RESUME, MISSION_CMD_SMALL_DISC_RESUME };
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
{ (void)count; (void)done; (void)c; last_arm = group; return arm_success ? LSC16_OK : LSC16_FAIL; }
static void mission_enter_state(mission_context_t *c, mission_state_t state, unsigned ms)
{ (void)ms; c->state = state; }
static void mission_fail(mission_context_t *c, unsigned fault) { (void)fault; c->state = MISSION_STATE_FAULT; }
static bool mission_start_vision(mission_context_t *c, unsigned scene, mission_stair_layer_t layer, mission_state_t state)
{ (void)scene; (void)layer; ++visions; c->state = state; return true; }
static bool mission_send_chassis(unsigned cmd, uint16_t id) { (void)cmd; (void)id; return true; }
#include "platform_under_test.inc"
static void attempt(mission_context_t *c, bool ok)
{
    read_success = ok;
    assert(mission_start_arm(c, 12, MISSION_STATE_PLATFORM_WAIT_GRASP));
    c->active_arm_group = (c->platform_balls == 4) ? 17 : 11;
    c->state = MISSION_STATE_PLATFORM_WAIT_STORAGE;
    if (mission_store_ball(c, MISSION_STORAGE_REGION_PLATFORM)) mission_handle_storage(c);
    else c->state = MISSION_STATE_FAULT;
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
    assert(c.state == MISSION_STATE_PLATFORM_WAIT_POSE); /* 17后失败必须回11。 */
    attempt(&c, true);
    assert(c.platform_balls == 5 && c.platform_attempts == 7 && turns == 5 && records == 5);
    assert(last_arm == 10 && c.state == MISSION_STATE_PLATFORM_WAIT_DEPARTURE_POSE);
    memset(&c, 0, sizeof(c));
    unsigned before = turns;
    for (unsigned i = 0; i < 7; ++i) attempt(&c, false);
    assert(c.platform_balls == 0 && turns == before && c.platform_attempts == 7);
    assert(c.state == MISSION_STATE_PLATFORM_WAIT_DEPARTURE_POSE);
    memset(&c, 0, sizeof(c));
    for (unsigned i = 0; i < 5; ++i) attempt(&c, true);
    assert(c.platform_attempts == 5 && c.state == MISSION_STATE_PLATFORM_WAIT_DEPARTURE_POSE);
    /* 非圆盘仍保留读卡失败记录并推进，不引入7次限制。 */
    read_success = false;
    before = turns;
    assert(mission_store_ball(&c, MISSION_STORAGE_REGION_STAIR));
    assert(mission_store_ball(&c, MISSION_STORAGE_REGION_SMALL_DISC));
    assert(turns == before + 2);
    memset(&c, 0, sizeof(c)); move_success = false;
    attempt(&c, true);
    assert(c.state == MISSION_STATE_FAULT && c.platform_balls == 0 && c.current_slot == 0);
    arm_success = false;
    before = c.platform_attempts;
    assert(!mission_start_arm(&c, 12, MISSION_STATE_PLATFORM_WAIT_GRASP));
    assert(c.platform_attempts == before);
    puts("Platform retry regression passed (host logic only)");
    return 0;
}
