/**
 * @file    ball_manifest_core.c
 * @brief   2026立体仓库本方球整场只追加档案表实现。
 */

#include "ball_manifest_core.h"

#include <string.h>

#define BALL_MANIFEST_REGION_COUNT 4U

static bool ball_manifest_region_valid(ball_manifest_region_t region)
{
    return (region == BALL_MANIFEST_REGION_TURNTABLE) ||
           (region == BALL_MANIFEST_REGION_STAIR) ||
           (region == BALL_MANIFEST_REGION_PILLAR);
}

static bool ball_manifest_color_valid(ball_manifest_color_t color)
{
    return (color == BALL_MANIFEST_COLOR_RED) ||
           (color == BALL_MANIFEST_COLOR_BLUE);
}

#if BALL_MANIFEST_RUNTIME_GUARDS_ENABLE
static bool ball_manifest_storage_slot_used(
    const ball_manifest_t *manifest,
    uint8_t storage_slot)
{
    uint8_t i;

    if (storage_slot == BALL_MANIFEST_STORAGE_SLOT_UNKNOWN) return false;
    for (i = 0U; i < manifest->count; ++i) {
        if (manifest->records[i].storage_slot == storage_slot) return true;
    }
    return false;
}

static uint16_t ball_manifest_crc16_update(uint16_t crc, uint8_t value)
{
    uint8_t bit;

    crc ^= (uint16_t)value << 8;
    for (bit = 0U; bit < 8U; ++bit) {
        crc = (crc & 0x8000U) != 0U ?
            (uint16_t)((crc << 1) ^ 0x1021U) : (uint16_t)(crc << 1);
    }
    return crc;
}

static uint16_t ball_manifest_record_checksum(
    const ball_manifest_record_t *record)
{
    uint16_t crc = 0xFFFFU;

    crc = ball_manifest_crc16_update(crc, record->sequence);
    crc = ball_manifest_crc16_update(crc, (uint8_t)record->region);
    crc = ball_manifest_crc16_update(crc, (uint8_t)record->color);
    crc = ball_manifest_crc16_update(crc, record->ic_code);
    crc = ball_manifest_crc16_update(crc, record->target_row);
    crc = ball_manifest_crc16_update(crc, record->target_column);
    crc = ball_manifest_crc16_update(crc, record->storage_slot);
    crc = ball_manifest_crc16_update(crc, (uint8_t)record->state);
    return crc;
}
#endif

void ball_manifest_init(ball_manifest_t *manifest)
{
    if (manifest == NULL) return;
    (void)memset(manifest, 0, sizeof(*manifest));
}

uint8_t ball_manifest_region_expected(ball_manifest_region_t region)
{
    if (region == BALL_MANIFEST_REGION_TURNTABLE) return 5U;
    if (region == BALL_MANIFEST_REGION_STAIR) return 2U;
    if (region == BALL_MANIFEST_REGION_PILLAR) return 2U;
    return 0U;
}

uint8_t ball_manifest_region_count(
    const ball_manifest_t *manifest,
    ball_manifest_region_t region)
{
#if BALL_MANIFEST_RUNTIME_GUARDS_ENABLE
    if ((manifest == NULL) || !ball_manifest_region_valid(region)) return 0U;
    return manifest->region_counts[(uint8_t)region];
#else
    uint8_t count = 0U;
    uint8_t i;

    if ((manifest == NULL) || !ball_manifest_region_valid(region)) return 0U;
    for (i = 0U; i < manifest->count; ++i) {
        if (manifest->records[i].region == region) ++count;
    }
    return count;
#endif
}

bool ball_manifest_region_is_complete(
    const ball_manifest_t *manifest,
    ball_manifest_region_t region)
{
    uint8_t expected;

    if (manifest == NULL) return false;
    expected = ball_manifest_region_expected(region);
    return (expected > 0U) &&
           (ball_manifest_region_count(manifest, region) >= expected);
}

ball_manifest_status_t ball_manifest_append(
    ball_manifest_t *manifest,
    ball_manifest_region_t region,
    ball_manifest_color_t color,
    uint8_t ic_code,
    uint8_t target_row,
    uint8_t target_column,
    uint8_t storage_slot)
{
    ball_manifest_record_t record;

    if ((manifest == NULL) || !ball_manifest_region_valid(region) ||
        !ball_manifest_color_valid(color) ||
        (target_row < 1U) || (target_row > 3U) ||
        (target_column < 1U) || (target_column > 4U) ||
        (ic_code != (uint8_t)((target_row << 4) | target_column))) {
        return BALL_MANIFEST_ERR_PARAM;
    }
#if BALL_MANIFEST_RUNTIME_GUARDS_ENABLE
    if (ball_manifest_validate(manifest) != BALL_MANIFEST_OK) {
        return BALL_MANIFEST_ERR_CORRUPT;
    }
#endif
    if (manifest->count >= BALL_MANIFEST_CAPACITY) {
        return BALL_MANIFEST_ERR_FULL;
    }
    if (ball_manifest_region_is_complete(manifest, region)) {
        return BALL_MANIFEST_ERR_REGION_COMPLETE;
    }
#if BALL_MANIFEST_RUNTIME_GUARDS_ENABLE
    if (ball_manifest_storage_slot_used(manifest, storage_slot)) {
        return BALL_MANIFEST_ERR_STORAGE_SLOT_DUPLICATE;
    }
#endif

    (void)memset(&record, 0, sizeof(record));
    record.sequence = manifest->count;
    record.region = region;
    record.color = color;
    record.ic_code = ic_code;
    record.target_row = target_row;
    record.target_column = target_column;
    record.storage_slot = storage_slot;
    record.state = BALL_MANIFEST_STATE_STORED;
    manifest->records[manifest->count] = record;
#if BALL_MANIFEST_RUNTIME_GUARDS_ENABLE
    manifest->records[manifest->count].checksum =
        ball_manifest_record_checksum(&record);
    manifest->records[manifest->count].committed = true;
    ++manifest->region_counts[(uint8_t)region];
#endif
    ++manifest->count;
    return BALL_MANIFEST_OK;
}

ball_manifest_status_t ball_manifest_append_read_failed(
    ball_manifest_t *manifest,
    ball_manifest_region_t region,
    ball_manifest_color_t color,
    uint8_t storage_slot)
{
    ball_manifest_record_t record;

    if ((manifest == NULL) || !ball_manifest_region_valid(region) ||
        !ball_manifest_color_valid(color) ||
        (storage_slot == BALL_MANIFEST_STORAGE_SLOT_UNKNOWN)) {
        return BALL_MANIFEST_ERR_PARAM;
    }
#if BALL_MANIFEST_RUNTIME_GUARDS_ENABLE
    if (ball_manifest_validate(manifest) != BALL_MANIFEST_OK) {
        return BALL_MANIFEST_ERR_CORRUPT;
    }
#endif
    if (manifest->count >= BALL_MANIFEST_CAPACITY) {
        return BALL_MANIFEST_ERR_FULL;
    }
    if (ball_manifest_region_is_complete(manifest, region)) {
        return BALL_MANIFEST_ERR_REGION_COMPLETE;
    }
#if BALL_MANIFEST_RUNTIME_GUARDS_ENABLE
    if (ball_manifest_storage_slot_used(manifest, storage_slot)) {
        return BALL_MANIFEST_ERR_STORAGE_SLOT_DUPLICATE;
    }
#endif

    (void)memset(&record, 0, sizeof(record));
    record.sequence = manifest->count;
    record.region = region;
    record.color = color;
    record.storage_slot = storage_slot;
    record.state = BALL_MANIFEST_STATE_READ_FAILED;
    manifest->records[manifest->count] = record;
#if BALL_MANIFEST_RUNTIME_GUARDS_ENABLE
    manifest->records[manifest->count].checksum =
        ball_manifest_record_checksum(&record);
    manifest->records[manifest->count].committed = true;
    ++manifest->region_counts[(uint8_t)region];
#endif
    ++manifest->count;
    return BALL_MANIFEST_OK;
}

ball_manifest_status_t ball_manifest_mark_placed(
    ball_manifest_t *manifest,
    uint8_t sequence)
{
    ball_manifest_record_t *record;

    if ((manifest == NULL) || (sequence >= manifest->count)) {
        return BALL_MANIFEST_ERR_PARAM;
    }
#if BALL_MANIFEST_RUNTIME_GUARDS_ENABLE
    if (ball_manifest_validate(manifest) != BALL_MANIFEST_OK) {
        return BALL_MANIFEST_ERR_CORRUPT;
    }
#endif
    record = &manifest->records[sequence];
    if (record->state == BALL_MANIFEST_STATE_READ_FAILED) {
        return BALL_MANIFEST_ERR_TARGET_UNKNOWN;
    }
#if BALL_MANIFEST_RUNTIME_GUARDS_ENABLE
    record->committed = false;
#endif
    record->state = BALL_MANIFEST_STATE_PLACED;
#if BALL_MANIFEST_RUNTIME_GUARDS_ENABLE
    record->checksum = ball_manifest_record_checksum(record);
    record->committed = true;
#endif
    return BALL_MANIFEST_OK;
}

ball_manifest_status_t ball_manifest_get(
    const ball_manifest_t *manifest,
    uint8_t sequence,
    ball_manifest_record_t *record)
{
    if ((manifest == NULL) || (record == NULL) ||
        (sequence >= manifest->count)) {
        return BALL_MANIFEST_ERR_PARAM;
    }
#if BALL_MANIFEST_RUNTIME_GUARDS_ENABLE
    if (ball_manifest_validate(manifest) != BALL_MANIFEST_OK) {
        return BALL_MANIFEST_ERR_CORRUPT;
    }
#endif
    *record = manifest->records[sequence];
    return BALL_MANIFEST_OK;
}

ball_manifest_status_t ball_manifest_validate(
    const ball_manifest_t *manifest)
{
#if BALL_MANIFEST_RUNTIME_GUARDS_ENABLE
    uint8_t counts[BALL_MANIFEST_REGION_COUNT] = {0U};
    uint8_t i;

    if ((manifest == NULL) || (manifest->count > BALL_MANIFEST_CAPACITY)) {
        return BALL_MANIFEST_ERR_PARAM;
    }
    for (i = 0U; i < manifest->count; ++i) {
        const ball_manifest_record_t *record = &manifest->records[i];
        if (!record->committed || (record->sequence != i) ||
            !ball_manifest_region_valid(record->region) ||
            !ball_manifest_color_valid(record->color) ||
            ((record->state != BALL_MANIFEST_STATE_STORED) &&
             (record->state != BALL_MANIFEST_STATE_PLACED) &&
             (record->state != BALL_MANIFEST_STATE_READ_FAILED)) ||
            ((record->state == BALL_MANIFEST_STATE_READ_FAILED) &&
             ((record->ic_code != 0U) || (record->target_row != 0U) ||
              (record->target_column != 0U))) ||
            (record->checksum != ball_manifest_record_checksum(record))) {
            return BALL_MANIFEST_ERR_CORRUPT;
        }
        ++counts[(uint8_t)record->region];
    }
    for (i = 0U; i < BALL_MANIFEST_REGION_COUNT; ++i) {
        if (counts[i] != manifest->region_counts[i]) {
            return BALL_MANIFEST_ERR_CORRUPT;
        }
    }
    return BALL_MANIFEST_OK;
#else
    return ((manifest == NULL) || (manifest->count > BALL_MANIFEST_CAPACITY)) ?
        BALL_MANIFEST_ERR_PARAM : BALL_MANIFEST_OK;
#endif
}
