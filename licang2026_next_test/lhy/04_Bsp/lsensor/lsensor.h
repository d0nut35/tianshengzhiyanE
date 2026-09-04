/**
 * @file    lsensor.h
 * @brief   灰度传感器对象定义
 * @note    - 检测点坐标相对车体坐标系，单位为 mm
 *          - 低电平表示检测到线，高电平表示未检测到线
 */

#ifndef LSENSOR_H
#define LSENSOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* 单个灰度传感器对象 */
typedef struct {
    uint8_t           is_inited;     /* 是否已初始化 */
    volatile uint8_t  is_on_line;    /* 当前原始电平，0=检测到线 */
    volatile uint8_t  was_on_line;   /* 上次原始电平，0=检测到线 */
    int16_t           x_mm;          /* 检测点 X 坐标，mm */
    int16_t           y_mm;          /* 检测点 Y 坐标，mm */
} lsensor_t;

#ifdef __cplusplus
}
#endif

#endif /* LSENSOR_H */
