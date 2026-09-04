/**
 * @file    lsensor_handler.c
 * @brief   固定六路灰度传感器周期采样与对象缓存
 * @note    - CMSIS-OS2 任务按固定周期采样
 *          - 六路 GPIO 均位于 GPIOC，一次读取 IDR 后统一更新
 */

#include "lsensor_handler.h"

#include <stddef.h>

#include "cmsis_os2.h"
#include "main.h"

#include "lsensor.h"
#include "lsensor_config.h"

#define LSH_STACK_BYTES   512U   /* 采样任务栈大小，bytes */

/* 固定传感器坐标与对应 GPIO 位 */
typedef struct {
    int16_t   x_mm;   /* 检测点 X 坐标，mm */
    int16_t   y_mm;   /* 检测点 Y 坐标，mm */
    uint16_t  pin;    /* GPIOC IDR 位掩码 */
} lsh_cfg_t;

static const lsh_cfg_t s_cfg[LSENSOR_COUNT] = {
    {LSENSOR_1_X_MM, LSENSOR_1_Y_MM, L_SENSOR_1_Pin},
    {LSENSOR_2_X_MM, LSENSOR_2_Y_MM, L_SENSOR_2_Pin},
    {LSENSOR_3_X_MM, LSENSOR_3_Y_MM, L_SENSOR_3_Pin},
    {LSENSOR_4_X_MM, LSENSOR_4_Y_MM, L_SENSOR_4_Pin},
    {LSENSOR_5_X_MM, LSENSOR_5_Y_MM, L_SENSOR_5_Pin},
    {LSENSOR_6_X_MM, LSENSOR_6_Y_MM, L_SENSOR_6_Pin}
};

static lsensor_t      s_sensor[LSENSOR_COUNT]; /* Handler 持有的对象组 */
static osThreadId_t   s_task = NULL;           /* 周期采样任务句柄 */
static volatile uint8_t s_ready = 0U;           /* 对象缓存可读标志 */

static const osThreadAttr_t s_task_attr = {
    .name       = "lsensor",
    .stack_size = LSH_STACK_BYTES,
    .priority   = osPriorityBelowNormal
};

/** @brief 一次读取 IDR 快照并更新全部传感器对象 */
static void lsh_sample(void)
{
    uint32_t idr = GPIOC->IDR; /* 同一时刻的六路输入电平快照 */
    uint8_t  i = 0U;           /* 传感器对象下标 */

    for (i = 0U; i < LSENSOR_COUNT; i++) {
        uint8_t level; /* 当前 GPIO 原始电平 */

        level = ((idr & (uint32_t)s_cfg[i].pin) != 0U) ? 1U : 0U;
        s_sensor[i].was_on_line = s_sensor[i].is_on_line;
        s_sensor[i].is_on_line = level;
    }
}

/** @brief 将毫秒采样周期转换为当前 CMSIS-OS tick 数 */
static uint32_t lsh_scan_ticks(void)
{
    uint32_t tick_hz = osKernelGetTickFreq(); /* OS 每秒 tick 数 */
    uint32_t ticks;                           /* 采样周期 tick 数 */

    ticks = ((tick_hz * LSENSOR_SCAN_MS) + 999U) / 1000U;
    return (ticks == 0U) ? 1U : ticks;
}

/** @brief 周期采样任务 */
static void lsh_task(void *arg)
{
    uint32_t period = lsh_scan_ticks();   /* 采样周期 tick 数 */
    uint32_t wake = osKernelGetTickCount(); /* 下一次唤醒时刻 */

    (void)arg;
    for (;;) {
        lsh_sample();
        wake += period;
        if (osDelayUntil(wake) != osOK) {
            wake = osKernelGetTickCount();
        }
    }
}

lsh_status_t lsh_init(void)
{
    uint8_t i = 0U; /* 传感器对象下标 */

    if (s_task != NULL) {
        return LSH_OK;
    }

    /* GPIO 已由 CubeMX 初始化，本层只装配对象并启动周期采样。 */
    for (i = 0U; i < LSENSOR_COUNT; i++) {
        s_sensor[i].is_inited = 1U;
        s_sensor[i].is_on_line = 0U;
        s_sensor[i].was_on_line = 0U;
        s_sensor[i].x_mm = s_cfg[i].x_mm;
        s_sensor[i].y_mm = s_cfg[i].y_mm;
    }
    lsh_sample();
    s_ready = 1U;

    s_task = osThreadNew(lsh_task, NULL, &s_task_attr);
    if (s_task == NULL) {
        s_ready = 0U;
        for (i = 0U; i < LSENSOR_COUNT; i++) {
            s_sensor[i].is_inited = 0U;
        }
        return LSH_ERR_RES;
    }
    return LSH_OK;
}

lsensor_level_t lsh_get_level(lsensor_id_t id)
{
    uint8_t idx; /* 物理序号转换后的数组下标 */

    if (s_ready == 0U) {
        return LSENSOR_LEVEL_INVALID;
    }
    if ((id < LSENSOR_ID_1) || (id > LSENSOR_ID_6)) {
        return LSENSOR_LEVEL_INVALID;
    }

    idx = (uint8_t)((uint32_t)id - (uint32_t)LSENSOR_ID_1);
    return (s_sensor[idx].is_on_line != 0U) ?
           LSENSOR_LEVEL_HIGH : LSENSOR_LEVEL_LOW;
}
