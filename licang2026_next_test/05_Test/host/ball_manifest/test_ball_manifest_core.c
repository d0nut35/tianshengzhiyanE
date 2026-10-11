#include <assert.h>
#include <stdio.h>
#include <string.h>

#include "ball_manifest_core.h"

static void append_ball(
    ball_manifest_t *manifest,
    ball_manifest_region_t region,
    uint8_t index,
    uint8_t code)
{
    assert(ball_manifest_append(
        manifest,
        region,
        BALL_MANIFEST_COLOR_RED,
        code,
        (uint8_t)(code >> 4),
        (uint8_t)(code & 0x0FU),
        index) == BALL_MANIFEST_OK);
}

static void test_rule_counts(void)
{
    assert(BALL_MANIFEST_CAPACITY == 9U);
    assert(ball_manifest_region_expected(BALL_MANIFEST_REGION_TURNTABLE) == 5U);
    assert(ball_manifest_region_expected(BALL_MANIFEST_REGION_STAIR) == 2U);
    assert(ball_manifest_region_expected(BALL_MANIFEST_REGION_PILLAR) == 2U);
    assert(ball_manifest_region_expected(BALL_MANIFEST_REGION_INVALID) == 0U);
}

static void test_regions_complete_at_exact_success_count(void)
{
    ball_manifest_t manifest;
    uint8_t i;

    ball_manifest_init(&manifest);
    for (i = 0U; i < 5U; ++i) {
        append_ball(&manifest, BALL_MANIFEST_REGION_TURNTABLE, i, 0x11U);
    }
    assert(ball_manifest_region_is_complete(
        &manifest, BALL_MANIFEST_REGION_TURNTABLE));
    assert(!ball_manifest_region_is_complete(
        &manifest, BALL_MANIFEST_REGION_STAIR));
    assert(ball_manifest_append(
        &manifest, BALL_MANIFEST_REGION_TURNTABLE,
        BALL_MANIFEST_COLOR_RED, 0x12U, 1U, 2U, 5U) ==
        BALL_MANIFEST_ERR_REGION_COMPLETE);
    assert(manifest.count == 5U);

    append_ball(&manifest, BALL_MANIFEST_REGION_STAIR, 5U, 0x21U);
    append_ball(&manifest, BALL_MANIFEST_REGION_STAIR, 6U, 0x22U);
    append_ball(&manifest, BALL_MANIFEST_REGION_PILLAR, 7U, 0x31U);
    append_ball(&manifest, BALL_MANIFEST_REGION_PILLAR, 8U, 0x32U);
    assert(manifest.count == BALL_MANIFEST_CAPACITY);
    assert(ball_manifest_region_is_complete(
        &manifest, BALL_MANIFEST_REGION_STAIR));
    assert(ball_manifest_region_is_complete(
        &manifest, BALL_MANIFEST_REGION_PILLAR));
    assert(ball_manifest_validate(&manifest) == BALL_MANIFEST_OK);
}

static void test_duplicate_target_is_preserved(void)
{
    ball_manifest_t manifest;
    ball_manifest_record_t first;
    ball_manifest_record_t second;

    ball_manifest_init(&manifest);
    append_ball(&manifest, BALL_MANIFEST_REGION_TURNTABLE, 0U, 0x23U);
    append_ball(&manifest, BALL_MANIFEST_REGION_TURNTABLE, 1U, 0x23U);
    assert(ball_manifest_get(&manifest, 0U, &first) == BALL_MANIFEST_OK);
    assert(ball_manifest_get(&manifest, 1U, &second) == BALL_MANIFEST_OK);
    assert(first.ic_code == 0x23U);
    assert(second.ic_code == 0x23U);
    assert(first.sequence == 0U);
    assert(second.sequence == 1U);
}

static void test_rejected_append_does_not_modify_history(void)
{
    ball_manifest_t manifest;
    ball_manifest_record_t before;
    ball_manifest_record_t after;

    ball_manifest_init(&manifest);
    append_ball(&manifest, BALL_MANIFEST_REGION_TURNTABLE, 0U, 0x14U);
    assert(ball_manifest_get(&manifest, 0U, &before) == BALL_MANIFEST_OK);
    assert(ball_manifest_append(
        &manifest, BALL_MANIFEST_REGION_STAIR,
        BALL_MANIFEST_COLOR_RED, 0x21U, 2U, 1U, 0U) ==
        BALL_MANIFEST_ERR_STORAGE_SLOT_DUPLICATE);
    assert(manifest.count == 1U);
    assert(ball_manifest_get(&manifest, 0U, &after) == BALL_MANIFEST_OK);
    assert(before.sequence == after.sequence);
    assert(before.ic_code == after.ic_code);
    assert(before.checksum == after.checksum);
}

static void test_corruption_is_detected(void)
{
    ball_manifest_t manifest;

    ball_manifest_init(&manifest);
    append_ball(&manifest, BALL_MANIFEST_REGION_TURNTABLE, 0U, 0x11U);
    manifest.records[0].target_column = 4U;
    assert(ball_manifest_validate(&manifest) == BALL_MANIFEST_ERR_CORRUPT);
    assert(ball_manifest_append(
        &manifest, BALL_MANIFEST_REGION_STAIR,
        BALL_MANIFEST_COLOR_RED, 0x22U, 2U, 2U, 1U) ==
        BALL_MANIFEST_ERR_CORRUPT);
    assert(manifest.count == 1U);
}

static void test_mark_placed_updates_only_lifecycle(void)
{
    ball_manifest_t manifest;
    ball_manifest_record_t record;

    ball_manifest_init(&manifest);
    append_ball(&manifest, BALL_MANIFEST_REGION_TURNTABLE, 0U, 0x34U);
    assert(ball_manifest_mark_placed(&manifest, 0U) == BALL_MANIFEST_OK);
    assert(ball_manifest_get(&manifest, 0U, &record) == BALL_MANIFEST_OK);
    assert(record.state == BALL_MANIFEST_STATE_PLACED);
    assert(record.ic_code == 0x34U);
    assert(record.storage_slot == 0U);
    assert(manifest.count == 1U);
}

static void test_fallback_history_and_conflicts(void)
{
    ball_manifest_t m, before;
    ball_manifest_record_t r;
    ball_manifest_init(&m);
    assert(ball_manifest_append_read_failed(&m, BALL_MANIFEST_REGION_TURNTABLE,
        BALL_MANIFEST_COLOR_RED, 0) == BALL_MANIFEST_OK);
    assert(ball_manifest_mark_placed(&m, 0) == BALL_MANIFEST_ERR_TARGET_UNKNOWN);
    append_ball(&m, BALL_MANIFEST_REGION_TURNTABLE, 1, 0x11);
    assert(ball_manifest_plan_fallback(&m, 0) == BALL_MANIFEST_OK);
    assert(ball_manifest_get(&m, 0, &r) == BALL_MANIFEST_OK);
    assert(r.ic_code == 0 && r.target_row == 0 && r.target_column == 0);
    assert(r.fallback_row != 1 || r.fallback_column != 1);
    before = m;
    assert(ball_manifest_plan_fallback(&m, 9999) == BALL_MANIFEST_OK);
    assert(memcmp(&before, &m, sizeof(m)) == 0);
    assert(ball_manifest_mark_placed(&m, 0) == BALL_MANIFEST_OK);
    assert(m.records[0].state == BALL_MANIFEST_STATE_PLACED && m.records[0].ic_code == 0);
    assert(ball_manifest_validate(&m) == BALL_MANIFEST_OK);
    /* 已固定保底位置被后追加真实目标占用：报冲突，不能重抽或半写入。 */
    append_ball(&m, BALL_MANIFEST_REGION_STAIR, 2,
        (uint8_t)((r.fallback_row << 4) | r.fallback_column));
    before = m;
    assert(ball_manifest_plan_fallback(&m, 42) == BALL_MANIFEST_ERR_PARAM);
    assert(memcmp(&before, &m, sizeof(m)) == 0);
    m.records[0].fallback_row = 0;
    assert(ball_manifest_validate(&m) == BALL_MANIFEST_ERR_CORRUPT);
    /* 多种随机种子下，全未知九槽均覆盖九格，原始读卡字段始终为零。 */
    for (uint32_t seed = 0; seed < 100; ++seed) {
        ball_manifest_init(&m);
        for (uint8_t i = 0; i < 9; ++i) {
            assert(ball_manifest_append_read_failed(&m,
                i < 5 ? BALL_MANIFEST_REGION_TURNTABLE :
                (i < 7 ? BALL_MANIFEST_REGION_STAIR : BALL_MANIFEST_REGION_PILLAR),
                BALL_MANIFEST_COLOR_BLUE, i) == BALL_MANIFEST_OK);
        }
        assert(ball_manifest_plan_fallback(&m, seed) == BALL_MANIFEST_OK);
        unsigned used = 0;
        for (uint8_t i = 0; i < 9; ++i) {
            assert(ball_manifest_get(&m, i, &r) == BALL_MANIFEST_OK);
            assert(r.fallback_row >= 1 && r.fallback_row <= 3);
            assert(r.fallback_column >= 1 && r.fallback_column <= 3);
            unsigned bit = 1U << ((r.fallback_row - 1) * 3 + r.fallback_column - 1);
            assert(!(used & bit)); used |= bit;
            assert(r.ic_code == 0 && r.target_row == 0 && r.target_column == 0);
        }
        assert(used == 0x1FF);
    }
}

int main(void)
{
    test_rule_counts();
    test_regions_complete_at_exact_success_count();
    test_duplicate_target_is_preserved();
    test_rejected_append_does_not_modify_history();
    test_corruption_is_detected();
    test_mark_placed_updates_only_lifecycle();
    test_fallback_history_and_conflicts();
    puts("ball_manifest_core tests passed");
    return 0;
}
