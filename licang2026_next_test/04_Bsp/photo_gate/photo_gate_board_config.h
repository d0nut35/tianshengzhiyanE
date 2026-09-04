/** @file photo_gate_board_config.h @brief F7光电门PB0板级映射。 */

#ifndef PHOTO_GATE_BOARD_CONFIG_H
#define PHOTO_GATE_BOARD_CONFIG_H

#include "main.h"

/** CubeMX当前把光电门OUT配置为PB0上拉输入。 */
#define PHOTO_GATE_GPIO_PORT GPIOB
#define PHOTO_GATE_GPIO_PIN  GPIO_PIN_0

#endif /* PHOTO_GATE_BOARD_CONFIG_H */
