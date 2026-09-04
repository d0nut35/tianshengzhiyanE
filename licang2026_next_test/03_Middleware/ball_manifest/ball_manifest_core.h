/**
 * @file    ball_manifest_core.h
 * @brief   2026立体仓库本方球整场只追加档案表。
 */

#ifndef BALL_MANIFEST_CORE_H
#define BALL_MANIFEST_CORE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#ifndef LICANG_RELEASE_MINIMAL
#define LICANG_RELEASE_MINIMAL 0
#endif

#define BALL_MANIFEST_RUNTIME_GUARDS_ENABLE (!LICANG_RELEASE_MINIMAL)

/** 本方颜色在全场最多9个球，档案保存到本轮比赛结束。 */
#define BALL_MANIFEST_CAPACITY             9U
#define BALL_MANIFEST_STORAGE_SLOT_UNKNOWN 0xFFU

typedef enum {
    BALL_MANIFEST_REGION_INVALID = 0,
    BALL_MANIFEST_REGION_TURNTABLE,
    BALL_MANIFEST_REGION_STAIR,
    BALL_MANIFEST_REGION_PILLAR,
} ball_manifest_region_t;

typedef enum {
    BALL_MANIFEST_COLOR_INVALID = 0,
    BALL_MANIFEST_COLOR_RED,
    BALL_MANIFEST_COLOR_BLUE,
} ball_manifest_color_t;

typedef enum {
    BALL_MANIFEST_STATE_INVALID = 0,
    BALL_MANIFEST_STATE_STORED,
    BALL_MANIFEST_STATE_PLACED,
    /** 已抓取但IC读取失败；物理槽位保留，目标位置未知。 */
    BALL_MANIFEST_STATE_READ_FAILED,
} ball_manifest_state_t;

typedef enum {
    BALL_MANIFEST_OK = 0,
    BALL_MANIFEST_ERR_PARAM,
    BALL_MANIFEST_ERR_FULL,
    BALL_MANIFEST_ERR_REGION_COMPLETE,
    BALL_MANIFEST_ERR_STORAGE_SLOT_DUPLICATE,
    BALL_MANIFEST_ERR_CORRUPT,
    BALL_MANIFEST_ERR_TARGET_UNKNOWN,
} ball_manifest_status_t;

typedef struct {
    uint8_t sequence;
    ball_manifest_region_t region;
    ball_manifest_color_t color;
    uint8_t ic_code;
    uint8_t target_row;
    uint8_t target_column;
    uint8_t storage_slot;
    ball_manifest_state_t state;
#if BALL_MANIFEST_RUNTIME_GUARDS_ENABLE
    uint16_t checksum;
    bool committed;
#endif
} ball_manifest_record_t;

typedef struct {
    ball_manifest_record_t records[BALL_MANIFEST_CAPACITY];
    uint8_t count;
#if BALL_MANIFEST_RUNTIME_GUARDS_ENABLE
    uint8_t region_counts[4];
#endif
} ball_manifest_t;

/** 清空并开始一轮新的球档案；只能由上层在安全空闲状态显式调用。 */
void ball_manifest_init(ball_manifest_t *manifest);

/** 返回2026规则下本方颜色在指定区域应抓取的球数：圆盘5、阶梯2、立柱2。 */
uint8_t ball_manifest_region_expected(ball_manifest_region_t region);

/** 查询某区域已成功提交的球数。非法参数返回0。 */
uint8_t ball_manifest_region_count(
    const ball_manifest_t *manifest,
    ball_manifest_region_t region);

/** 仅当成功提交数达到规则数量时，区域才算完成。 */
bool ball_manifest_region_is_complete(
    const ball_manifest_t *manifest,
    ball_manifest_region_t region);

/**
 * 追加一个已抓取、已读卡的球记录。
 *
 * 提交采用“先写完整记录、最后置committed、再增加计数”的顺序。目标IC编码
 * 允许重复，因为正式规则没有承诺9个目标位置一定互不重复；物理存储槽位若
 * 已知则禁止重复。失败不会修改既有记录或计数。
 */
ball_manifest_status_t ball_manifest_append(
    ball_manifest_t *manifest,
    ball_manifest_region_t region,
    ball_manifest_color_t color,
    uint8_t ic_code,
    uint8_t target_row,
    uint8_t target_column,
    uint8_t storage_slot);

/** IC重试耗尽后登记已占用槽位；目标编码、行、列保持未知。 */
ball_manifest_status_t ball_manifest_append_read_failed(
    ball_manifest_t *manifest,
    ball_manifest_region_t region,
    ball_manifest_color_t color,
    uint8_t storage_slot);

/** 把指定记录更新为已入库；不允许修改球的身份、来源或IC目标信息。 */
ball_manifest_status_t ball_manifest_mark_placed(
    ball_manifest_t *manifest,
    uint8_t sequence);

/** 复制读取记录，避免向调用者暴露可修改内部数组的指针。 */
ball_manifest_status_t ball_manifest_get(
    const ball_manifest_t *manifest,
    uint8_t sequence,
    ball_manifest_record_t *record);

/** 校验提交标记、顺序、区域计数和每条记录校验值。 */
ball_manifest_status_t ball_manifest_validate(
    const ball_manifest_t *manifest);

#ifdef __cplusplus
}
#endif

#endif /* BALL_MANIFEST_CORE_H */
