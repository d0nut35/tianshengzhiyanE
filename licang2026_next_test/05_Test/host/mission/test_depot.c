/* 正式仓库控制逻辑回归：实际源码函数+实际manifest，替身不模拟硬件可靠性。 */
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "mission_app.h"
#include "mission_config.h"
#include "ball_manifest_core.h"
#undef MISSION_CHASSIS_ROUTE_TEST_ENABLED
#define MISSION_CHASSIS_ROUTE_TEST_ENABLED 0
/* 主机测试不接USART1，诊断打印不改变状态转换断言。 */
#define DEPOT_TRACE(...) ((void)0)
#define PLATFORM_TRACE(...) ((void)0)

typedef uint8_t mission_command_type_t;
enum { MISSION_CMD_GO_DEPOT_1 = 12, MISSION_CMD_DEPOT_OK = 16 };
enum { MISSION_FAULT_STORAGE = 5, MISSION_FAULT_ARM = 3,
       MISSION_FAULT_VISION = 4, MISSION_FAULT_QUEUE = 6, MISSION_FAULT_TIMEOUT = 1 };
enum { MISSION_VISION_SCENE_PLATFORM = 1, MISSION_VISION_SCENE_DEPOT_DIGIT = 4 };
typedef enum { ZDT_TURNTABLE_DIR_CW, ZDT_TURNTABLE_DIR_CCW } zdt_turntable_direction_t;
typedef struct {
    mission_state_t state;
    uint32_t deadline_tick;
    uint8_t current_slot, depot_target_slot, depot_position, depot_column;
    uint8_t depot_first_digit, depot_digit, depot_columns_used, depot_sequence, depot_row;
    uint16_t depot_abnormal_mask, request_id;
    bool depot_preparing, arm_home_ready;
    uint8_t active_arm_group;
    ball_manifest_t manifest;
} mission_context_t;

static uint32_t now;
static unsigned groups[128], group_count, moves, sessions, stops;
static uint8_t last_command;
static bool move_ok = true, arm_ok = true;
static uint32_t osKernelGetTickCount(void) { return now; }
static void mission_enter_state(mission_context_t *c, mission_state_t state, uint32_t ms)
{ c->state = state; c->deadline_tick = ms ? now + ms : 0; }
static void mission_fail(mission_context_t *c, unsigned fault)
{ (void)fault; c->state = MISSION_STATE_FAULT; }
static bool mission_start_arm(mission_context_t *c, uint8_t group, mission_state_t state)
{
    if (!arm_ok) return false;
    assert(group_count < 128);
    groups[group_count++] = group;
    c->active_arm_group = group;
    c->arm_home_ready = false;
    mission_enter_state(c, state, 30000);
    return true;
}
static bool mission_start_vision(mission_context_t *c, unsigned scene,
                                mission_stair_layer_t layer, mission_state_t state)
{ (void)scene; (void)layer; ++sessions; mission_enter_state(c, state, 30000); return true; }
static bool mission_stop_vision(mission_context_t *c) { (void)c; ++stops; return true; }
static uint16_t mission_next_request_id(mission_context_t *c) { return ++c->request_id; }
static bool mission_send_chassis(mission_command_type_t cmd, uint16_t id)
{ assert(id != 0); last_command = cmd; return true; }
static bool mission_advance_slot(mission_context_t *c, zdt_turntable_direction_t direction, uint8_t *fine)
{ (void)c; (void)direction; (void)fine; ++moves; return move_ok; }
static void mission_start_run(mission_context_t *c) { (void)c; assert(false); }
static void mission_depot_next_ball(mission_context_t *ctx);
static void mission_depot_start_digit(mission_context_t *ctx);
#include "depot_under_test.inc"

static void reset(mission_context_t *c)
{
    memset(c, 0, sizeof(*c)); now = 100; group_count = moves = sessions = stops = 0;
    last_command = 0; move_ok = arm_ok = true;
}
static void seek(mission_context_t *c)
{
    unsigned bound = 0;
    while (c->state == MISSION_STATE_DEPOT_SEEK) {
        assert(++bound <= 12); now = c->deadline_tick; mission_check_timeout(c);
    }
}
static void add_ball(mission_context_t *c, uint8_t row, uint8_t col, uint8_t slot)
{
    assert(ball_manifest_append(&c->manifest, BALL_MANIFEST_REGION_TURNTABLE,
        BALL_MANIFEST_COLOR_RED, (uint8_t)((row << 4) | col), row, col, slot) == BALL_MANIFEST_OK);
}
static void digit(mission_context_t *c, uint8_t value)
{ c->depot_digit = value; mission_depot_digit_done(c); }

/* 替身交付完成回报，实际姿态回报校验另由test_arm_home覆盖。 */
static void finish_arm(mission_context_t *c)
{
    c->arm_home_ready = c->active_arm_group == MISSION_HOME_ACTION_GROUP;
    mission_depot_arm_done(c);
}
static void until_return(mission_context_t *c)
{
    unsigned bound = 0;
    while (c->state != MISSION_STATE_DEPOT_RETURN) {
        assert(++bound < 10);
        if (c->state == MISSION_STATE_DEPOT_SEEK) seek(c);
        else finish_arm(c);
    }
}

int main(void)
{
    mission_context_t c;
    /* 按当前位置而非固定12槽排序：12→9→6→3，层序为2→1→3。 */
    reset(&c); c.current_slot = 11; c.depot_column = 2; c.depot_position = 1;
    add_ball(&c, 3, 2, 2); add_ball(&c, 2, 2, 8); add_ball(&c, 1, 2, 5);
    const uint8_t nearest[] = {1, 2, 0};
    unsigned total_steps = 0;
    for (unsigned i = 0; i < sizeof(nearest); ++i) {
        if (i == 0) mission_depot_next_ball(&c);
        assert(c.depot_sequence == nearest[i]);
        total_steps += mission_depot_ccw_steps(c.current_slot, c.depot_target_slot);
        until_return(&c);
        assert(c.manifest.records[nearest[i]].state == BALL_MANIFEST_STATE_STORED);
        finish_arm(&c);
        assert(c.manifest.records[nearest[i]].state == BALL_MANIFEST_STATE_PLACED);
    }
    assert(total_steps == 9 && moves == 9 && c.depot_position == 2);
    /* 后续列保留真实槽号；当前槽零步优先，CCW可跨1→12且不改方向。 */
    reset(&c); c.current_slot = 0; c.depot_column = 1;
    add_ball(&c, 3, 1, 10); add_ball(&c, 1, 1, 0); add_ball(&c, 2, 1, 11);
    mission_depot_next_ball(&c); assert(c.depot_sequence == 1);
    assert(mission_depot_ccw_steps(0, 0) == 0);
    assert(mission_depot_ccw_steps(0, 11) == 1);
    assert(ball_manifest_mark_placed(&c.manifest, 1) == BALL_MANIFEST_OK);
    mission_depot_next_ball(&c); assert(c.depot_sequence == 2);
    c.depot_abnormal_mask = 1U << 2;
    mission_depot_next_ball(&c); assert(c.depot_sequence == 0);
    /* 阶段1稳定期间不启动视觉；200ms到期只启动一次。 */
    reset(&c); mission_enter_state(&c, MISSION_STATE_PLATFORM_SETTLE, MISSION_PLATFORM_SETTLE_MS);
    now += 199; mission_check_timeout(&c); assert(sessions == 0);
    ++now; mission_check_timeout(&c); assert(sessions == 1 && c.state == MISSION_STATE_PLATFORM_WAIT_VISION);
    /* 抓取数不能推定为9；空盘、部分抓取、九球和已在12槽均实际走到12槽。 */
    const uint8_t starts[] = {0, 3, 9, 11};
    for (unsigned i = 0; i < sizeof(starts); ++i) {
        reset(&c); c.current_slot = starts[i]; mission_depot_prepare(&c);
        assert(groups[0] == 10 && sessions == 0 && moves == 0);
        /* 发送10不算到位，尚未完成时主循环不能转盘。 */
        now += 1; mission_check_timeout(&c); assert(moves == 0);
        finish_arm(&c); seek(&c);
        assert(c.current_slot == 11 && moves == 11U - starts[i]);
        assert(group_count == 1 && sessions == 1);
    }
    /* 小圆盘退出已完成10：入仓不重发，转到12后直接开始数字会话。 */
    reset(&c); c.current_slot = 9; c.arm_home_ready = true;
    mission_depot_prepare(&c); assert(group_count == 0);
    seek(&c); assert(moves == 2 && sessions == 1 && group_count == 0);
    /* 重复目标与读卡失败留车，正常球按CCW距离；回10前不能提前PLACED。 */
    reset(&c); c.current_slot = 4;
    add_ball(&c, 1, 2, 0); add_ball(&c, 3, 2, 1); add_ball(&c, 3, 2, 2);
    assert(ball_manifest_append_read_failed(&c.manifest, BALL_MANIFEST_REGION_TURNTABLE,
        BALL_MANIFEST_COLOR_RED, 3) == BALL_MANIFEST_OK);
    mission_depot_prepare(&c); assert(c.depot_abnormal_mask == 12);
    finish_arm(&c); seek(&c);
    digit(&c, 2); assert(sessions == 2); digit(&c, 2);
    assert(c.depot_sequence == 1 && c.depot_row == 3);
    unsigned start = group_count;
    until_return(&c);
    assert(c.manifest.records[1].state == BALL_MANIFEST_STATE_STORED);
    const unsigned row3[] = {22, 23, 24, 10};
    assert(group_count == start + sizeof(row3) / sizeof(row3[0]));
    assert(memcmp(groups + start, row3, sizeof(row3)) == 0);
    finish_arm(&c);
    assert(c.manifest.records[1].state == BALL_MANIFEST_STATE_PLACED && c.depot_sequence == 0);
    until_return(&c); finish_arm(&c);
    assert(c.manifest.records[0].state == BALL_MANIFEST_STATE_PLACED);
    assert(c.manifest.records[2].state == BALL_MANIFEST_STATE_STORED);
    assert(c.manifest.records[3].state == BALL_MANIFEST_STATE_READ_FAILED);
    assert(c.depot_position == 2 && last_command == MISSION_CMD_GO_DEPOT_1 + 1);
    /* 数字不一致/重复列都拒绝放球。 */
    reset(&c); c.depot_position = 1; c.depot_first_digit = 1; digit(&c, 2);
    assert(c.state == MISSION_STATE_FAULT && group_count == 0);
    reset(&c); c.depot_first_digit = 2; c.depot_columns_used = 1U << 2; digit(&c, 2);
    assert(c.state == MISSION_STATE_FAULT);
    /* 电机失败不伪造槽号、不收臂继续；下发机械臂失败不标记PLACED。 */
    reset(&c); c.current_slot = 9; mission_depot_prepare(&c); finish_arm(&c);
    move_ok = false; seek(&c); assert(c.state == MISSION_STATE_FAULT && c.current_slot == 9);
    reset(&c); add_ball(&c, 2, 1, 0); c.depot_column = 1; arm_ok = false;
    mission_depot_next_ball(&c);
    assert(c.state == MISSION_STATE_FAULT && c.manifest.records[0].state == BALL_MANIFEST_STATE_STORED);
    /* 二层在D4放置后不运行24；成功收臂才开始1秒计时。 */
    reset(&c); add_ball(&c, 2, 4, 0); c.depot_column = 4; c.depot_position = 4;
    mission_depot_next_ball(&c); until_return(&c);
    const unsigned row2[] = {10, 22, 25, 10};
    assert(group_count == 4 && memcmp(groups, row2, sizeof(row2)) == 0);
    assert(c.manifest.records[0].state == BALL_MANIFEST_STATE_STORED);
    finish_arm(&c);
    assert(c.state == MISSION_STATE_DEPOT_DWELL && c.deadline_tick == now + 1000);
    /* D4无球也等待完整1秒；随后直接DEPOT_OK，不返D1。 */
    reset(&c); c.depot_position = 4; c.depot_column = 4; mission_depot_next_ball(&c);
    now += 999; mission_check_timeout(&c); assert(last_command == 0);
    ++now; mission_check_timeout(&c);
    assert(last_command == MISSION_CMD_DEPOT_OK && c.state == MISSION_STATE_DEPOT_WAIT_HOME);
    reset(&c); mission_enter_state(&c, MISSION_STATE_DEPOT_WAIT_DIGIT, 3000);
    now += 3000; mission_check_timeout(&c);
    assert(stops == 1 && c.state == MISSION_STATE_DEPOT_DIGIT_STOP);
    puts("Mission depot regression passed (host logic only)");
    return 0;
}
