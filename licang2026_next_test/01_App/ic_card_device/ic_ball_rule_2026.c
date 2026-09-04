/**
 * @file    ic_ball_rule_2026.c
 * @brief   2026初步赛事规则下的IC球数据解释实现。
 */

#include "ic_ball_rule_2026.h"

#include <string.h>

/**
 * @brief 校验16字节比赛数据并转换为球的行列语义。
 * @param data 协议Core已经提取出的16字节数据块。
 * @param ball 接收解析结果的输出对象。
 * @return 数据一致且行列编码有效时返回true，否则返回false。
 * @note 失败前会先清空ball，调用方不能继续使用上一次的有效结果。
 */
bool ic_ball_rule_2026_decode(
    const uint8_t data[IC_CARD_BLOCK_DATA_SIZE],
    ic_card_ball_info_t *ball)
{
    uint8_t i;
    uint8_t row;
    uint8_t column;

    if ((data == NULL) || (ball == NULL)) {
        return false;
    }

    /* 先清空输出，保证失败路径不会遗留上一次有效球号。 */
    (void)memset(ball, 0, sizeof(*ball));
    for (i = 1U; i < IC_CARD_BLOCK_DATA_SIZE; ++i) {
        if (data[i] != data[0]) {
            return false;
        }
    }

    row = (uint8_t)(data[0] >> 4);
    column = (uint8_t)(data[0] & 0x0FU);
    if ((row < 1U) || (row > 3U) || (column < 1U) || (column > 4U)) {
        return false;
    }

    ball->kind = IC_CARD_BALL_TARGET;
    ball->code = data[0];
    ball->row = row;
    ball->column = column;
    return true;
}
