/**
 * @file    app_util.h
 * @brief   App/Service 共用的 tick 换算与角度归一化内联函数
 * @note    - 无状态、无副作用，仅依赖 CMSIS-OS2 tick 频率
 *          - 只能在任务上下文取 tick 频率，ISR 中不要调用
 */

#ifndef APP_UTIL_H
#define APP_UTIL_H

#include <stdint.h>

#include "cmsis_os2.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief  将毫秒转换成当前 CMSIS-OS tick 数
 * @param  ms 毫秒数
 * @retval tick 数，最小为 1
 */
static inline uint32_t util_ms_ticks(uint32_t ms)
{
    uint32_t hz = osKernelGetTickFreq(); /* OS 每秒 tick 数 */
    uint32_t ticks;                      /* 换算后的 tick 数 */

    if (hz == 0U) {
        return 1U;
    }
    ticks = ((hz * ms) + 999U) / 1000U;
    return (ticks == 0U) ? 1U : ticks;
}

/**
 * @brief  将角度归一化到 [-180, 180)
 * @param  angle 待归一化角度，deg
 * @retval 归一化角度，deg
 */
static inline float util_ang_norm(float angle)
{
    while (angle >= 180.0f) {
        angle -= 360.0f;
    }
    while (angle < -180.0f) {
        angle += 360.0f;
    }
    return angle;
}

#ifdef __cplusplus
}
#endif

#endif /* APP_UTIL_H */
