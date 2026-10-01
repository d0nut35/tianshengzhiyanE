/* 无线生产放置函数回归；设备替身只核对顺序，不证明机械安全。 */
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "mission_app.h"
#include "mission_config.h"
#include "ball_manifest_core.h"
typedef uint8_t mission_command_type_t;
typedef uint8_t chassis_command_type_t;
enum { MISSION_CMD_GO_DEPOT_2 = 1, MISSION_CMD_GO_DEPOT_3, MISSION_CMD_GO_DEPOT_4 };
enum { CHASSIS_CMD_DEPOT_2_READY = 1, CHASSIS_CMD_DEPOT_3_READY, CHASSIS_CMD_DEPOT_4_READY };
typedef struct { unsigned unused; } mission_context_t;
typedef struct { struct { uint8_t row, column; } ball; bool placed; } mission_test_ball_t;
static struct {
    mission_test_ball_t manual_balls[BALL_MANIFEST_CAPACITY];
    uint8_t current_slot, depot_position;
    char text[128];
} g_wireless_test;
static unsigned groups[100], group_count, steps_total, placed_count;
static uint8_t placed_order[BALL_MANIFEST_CAPACITY], digit_calls;
static unsigned fail_group;
static bool seek_ok;
static void mission_test_write(const char *s) { (void)s; }
static void mission_enter_state(mission_context_t *c, mission_state_t s, uint32_t t)
{ (void)c; (void)s; (void)t; }
static bool mission_test_send_wait(mission_context_t *c, mission_command_type_t cmd,
                                  chassis_command_type_t evt, mission_state_t s)
{ (void)c; (void)s; assert(cmd == evt); return true; }
static bool mission_test_read_depot_digit(mission_context_t *c, uint8_t *digit)
{ (void)c; *digit = (uint8_t)(digit_calls++ / 2 + 1); return true; }
static bool mission_test_run_arm_group(mission_context_t *c, uint8_t group)
{
    (void)c; assert(group_count < 100); groups[group_count++] = group;
    return group_count != fail_group;
}
static bool mission_test_seek_ball_slot(mission_context_t *c, uint8_t target)
{
    (void)c;
    if (!seek_ok) return false;
    steps_total += (g_wireless_test.current_slot + 12U - target) % 12U;
    g_wireless_test.current_slot = target;
    return true;
}
static void mission_test_print_ball(mission_context_t *c, uint8_t i)
{ (void)c; assert(placed_count < BALL_MANIFEST_CAPACITY); placed_order[placed_count++] = i; }
#include "depot_wireless_under_test.inc"
static void reset(void)
{
    memset(&g_wireless_test, 0, sizeof(g_wireless_test));
    const uint8_t rows[] = {1, 2, 3, 3, 1, 1, 2, 3, 2};
    const uint8_t cols[] = {2, 2, 1, 2, 3, 1, 3, 3, 1};
    for (unsigned i = 0; i < BALL_MANIFEST_CAPACITY; ++i) {
        g_wireless_test.manual_balls[i].ball.row = rows[i];
        g_wireless_test.manual_balls[i].ball.column = cols[i];
    }
    g_wireless_test.current_slot = 12;
    group_count = steps_total = placed_count = digit_calls = fail_group = 0;
    seek_ok = true;
}
int main(void)
{
    mission_context_t c = {0};
    reset(); assert(mission_test_run_depot_balls(&c));
    const uint8_t expected[] = {8, 5, 2, 1, 0, 3, 7, 6, 4};
    assert(placed_count == 9 && memcmp(placed_order, expected, sizeof(expected)) == 0);
    assert(steps_total == 31 && g_wireless_test.current_slot == 5);
    /* 2层→1层→3层，只有3层插入24，均回10再标记放置。 */
    const unsigned first_column[] = {27,10,22,25,10, 27,10,22,26,10, 27,10,22,23,24,10};
    assert(memcmp(groups, first_column, sizeof(first_column)) == 0);
    reset(); seek_ok = false;
    assert(!mission_test_run_depot_balls(&c));
    assert(g_wireless_test.current_slot == 12 && placed_count == 0);
    reset(); fail_group = 5; /* 第一球回10失败，不能提前placed。 */
    assert(!mission_test_run_depot_balls(&c));
    assert(!g_wireless_test.manual_balls[8].placed && placed_count == 0);
    puts("Wireless depot nearest-slot regression passed (host logic only)");
    return 0;
}
