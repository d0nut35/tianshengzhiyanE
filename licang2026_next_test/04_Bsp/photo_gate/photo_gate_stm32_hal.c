/** @file photo_gate_stm32_hal.c @brief 光电门PB0的STM32 HAL读取实现。 */

#include "photo_gate_stm32_hal.h"

#include "photo_gate_board_config.h"

/** @copydoc photo_gate_stm32_hal_read_raw() */
bool photo_gate_stm32_hal_read_raw(void)
{
    return HAL_GPIO_ReadPin(PHOTO_GATE_GPIO_PORT, PHOTO_GATE_GPIO_PIN) ==
        GPIO_PIN_SET;
}
