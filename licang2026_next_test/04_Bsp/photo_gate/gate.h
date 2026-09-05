/** @file gate.h @brief 通过STM32 HAL读取PB0光电门原始电平。 */

#ifndef GATE_H
#define GATE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>

#include "main.h"

/**
 * @brief 读取PB0当前原始逻辑电平。
 * @return PB0为高电平返回true，为低电平返回false。
 * @note 本接口不解释遮挡或到位语义；有效极性由上层按实机标定使用。
 */
static inline bool gate_read(void)
{
    return HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0) == GPIO_PIN_SET;
}

#ifdef __cplusplus
}
#endif

#endif /* GATE_H */
