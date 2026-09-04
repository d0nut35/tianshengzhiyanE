/**
 * @file    ic_ball_rule_2026.h
 * @brief   2026初步赛事规则下的IC球数据解释接口。
 *
 * 规则解释属于App领域逻辑：输入是协议Core已提取出的16字节块数据，输出
 * 是比赛业务需要的球编号、行和列。后续正式规则变化时可以替换本文件，
 * 而不触碰读卡协议、UART HAL适配或Service队列。
 */

#ifndef IC_BALL_RULE_2026_H
#define IC_BALL_RULE_2026_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#include "ic_card_core.h"

typedef enum {
    IC_CARD_BALL_INVALID = 0,
    IC_CARD_BALL_TARGET,
} ic_card_ball_kind_t;

typedef struct {
    ic_card_ball_kind_t kind;
    uint8_t code;
    uint8_t row;
    uint8_t column;
} ic_card_ball_info_t;

/**
 * @brief 按2026初步规则解释比赛球数据。
 * @return true表示16字节内容一致且编码落在1~3行、1~4列；否则为false。
 * @note 2026初步规则已取消干扰球，因此0x14按“第1行第4列”处理。
 */
bool ic_ball_rule_2026_decode(
    const uint8_t data[IC_CARD_BLOCK_DATA_SIZE],
    ic_card_ball_info_t *ball);

#ifdef __cplusplus
}
#endif

#endif /* IC_BALL_RULE_2026_H */
