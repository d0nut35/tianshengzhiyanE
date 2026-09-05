/**
 * @file    test_ic_ball_rule_2026.c
 * @brief   2026初步赛事球号解释规则的PC单元测试。
 */

#include <assert.h>
#include <stdio.h>
#include <string.h>

#include "ic_card_service.h"

int main(void)
{
    ic_ball_t ball;
    uint8_t block[IC_CARD_BLOCK_DATA_SIZE];

    (void)memset(block, 0x23, sizeof(block));
    assert(ic_decode_ball(block, &ball));
    assert((ball.code == 0x23U) && (ball.row == 2U) && (ball.column == 3U));

    /* 2026初步规则已取消干扰球：0x14解释为第1行第4列。 */
    (void)memset(block, 0x14, sizeof(block));
    assert(ic_decode_ball(block, &ball));
    assert((ball.row == 1U) && (ball.column == 4U));

    /* 16字节不一致或行列越界时必须拒绝，不能留下旧结果。 */
    block[15] = 0x13U;
    assert(!ic_decode_ball(block, &ball));
    assert(ball.kind == IC_BALL_INVALID);
    (void)memset(block, 0x45, sizeof(block));
    assert(!ic_decode_ball(block, &ball));

    puts("IC ball 2026 rule tests passed");
    return 0;
}
