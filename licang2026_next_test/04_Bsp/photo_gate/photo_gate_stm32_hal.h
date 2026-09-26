/** @file photo_gate_stm32_hal.h @brief 光电门PB0的STM32 HAL读取适配。 */

#ifndef PHOTO_GATE_STM32_HAL_H
#define PHOTO_GATE_STM32_HAL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>

/**
 * @brief 读取PB0当前原始逻辑电平。
 * @return PB0为高电平返回true，为低电平返回false。
 * @note 本接口不解释遮挡/到位语义；有效极性需由实机实验标定。
 */
bool photo_gate_stm32_hal_read_raw(void);

#ifdef __cplusplus
}
#endif

#endif /* PHOTO_GATE_STM32_HAL_H */
