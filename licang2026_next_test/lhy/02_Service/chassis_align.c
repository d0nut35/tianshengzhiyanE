/**
 * @file    chassis_align.c
 * @brief   白线对齐动作原语：扫边缘、回中、找线、软件标定航向
 * @note    - 底盘命令 wz 与 IMU 原始 yaw 同向，回中按最短误差选方向
 *          - IMU 帧与世界帧反号，标定时对世界航向取负
 *          - 扫描与回中均带超时，超时前统一停车再返回失败
 */

#include "chassis_align.h"

#include <stddef.h>

#include "cmsis_os2.h"

#include "app_util.h"
#include "chassis_service.h"
#include "hwt101_adaption.h"
#include "lsensor/lsensor_handler.h"

/* ===== 调试日志：0=不编译进固件，1=经 RTT 输出 ===== */
#ifndef ALIGN_LOG_EN
#define ALIGN_LOG_EN   0
#endif
#if ALIGN_LOG_EN
#include "cat_log.h"
#define AL_LOGI(fmt, ...)  LOGI("[align] " fmt, ##__VA_ARGS__)
#define AL_LOGE(fmt, ...)  LOGE("[align] " fmt, ##__VA_ARGS__)
#else
#define AL_LOGI(fmt, ...)  do {} while (0)
#define AL_LOGE(fmt, ...)  do {} while (0)
#endif

/* 白线对齐参数 */
#define AL_CW_W_DEG      (-5.0f)  /* 顺时针扫描角速度，deg/s */
#define AL_CCW_W_DEG     5.0f     /* 逆时针扫描角速度，deg/s */
#define AL_TURN_W_DEG    8.0f     /* 回到中值的角速度，deg/s */
#define AL_YAW_TOL_DEG   0.8f     /* 回中角度容差，deg */
#define AL_MID_YAW_OFS   (-3.0f)  /* 中值补偿，沿 IMU yaw 正向，deg */
#define AL_SEEK_VX_MMS   20.0f    /* 找线时的右移速度，mm/s */
#define AL_SCAN_MS       5U       /* 灰度与 IMU 轮询周期，ms */
#define AL_STOP_MS       80U      /* 停车后机械稳定时间，ms */
#define AL_SETTLE_MS     20U      /* 找线停车后等控制任务执行，ms */
#define AL_IMU_TMO_MS    200U     /* 等待有效 IMU 角度超时，ms */
#define AL_SWEEP_TMO_MS  6000U    /* 单向找线边缘超时，ms */
#define AL_TURN_TMO_MS   6000U    /* 回到中值角度超时，ms */

static align_status_t seek_edge(lsensor_id_t id, float wz, float *yaw_deg);
static align_status_t turn_mid(float target);
static align_status_t imu_calib(float yaw_deg);

/** @copydoc align_stop */
align_status_t align_stop(void)
{
    if (csvc_free(0.0f, 0.0f, 0.0f) != CSVC_OK) {
        return ALIGN_ERR;
    }
    osDelay(AL_STOP_MS);
    return ALIGN_OK;
}

/** @copydoc align_yaw_read */
align_status_t align_yaw_read(float *yaw_deg)
{
    float gyro = 0.0f;                       /* 附带的 Z 轴角速度 */
    uint32_t start = osKernelGetTickCount(); /* 超时起点 */
    uint32_t tmo = util_ms_ticks(AL_IMU_TMO_MS); /* 超时 tick 数 */

    if (yaw_deg == NULL) {
        return ALIGN_ERR;
    }
    while ((osKernelGetTickCount() - start) < tmo) {
        if (hwt101_adp_read(&gyro, yaw_deg) == HWT101_OK) {
            return ALIGN_OK;
        }
        osDelay(AL_SCAN_MS);
    }
    return ALIGN_ERR;
}

/**
 * @brief  原地旋转至指定灰度传感器由压线变为高电平
 * @param  id      灰度传感器编号
 * @param  wz      旋转角速度，deg/s
 * @param  yaw_deg 高电平触发瞬间的 IMU 原始航向
 * @retval ALIGN_OK / ALIGN_ERR
 */
static align_status_t seek_edge(lsensor_id_t id, float wz, float *yaw_deg)
{
    lsensor_level_t level;                   /* 当前传感器电平 */
    float gyro = 0.0f;                       /* 附带的 Z 轴角速度 */
    float edge_yaw = 0.0f;                   /* 高电平触发瞬间的航向 */
    uint32_t start = osKernelGetTickCount(); /* 扫描超时起点 */
    uint32_t tmo = util_ms_ticks(AL_SWEEP_TMO_MS); /* 超时 tick 数 */
    uint8_t yaw_valid = 0U;                  /* 触发角度有效标志 */

    if (yaw_deg == NULL) {
        return ALIGN_ERR;
    }
    level = lsh_get_level(id);
    if (level != LSENSOR_LEVEL_LOW) {
        return ALIGN_ERR;
    }
    if (csvc_free(0.0f, 0.0f, wz) != CSVC_OK) {
        return ALIGN_ERR;
    }
    while ((osKernelGetTickCount() - start) < tmo) {
        level = lsh_get_level(id);
        if (level == LSENSOR_LEVEL_HIGH) {
            if (hwt101_adp_read(&gyro, &edge_yaw) == HWT101_OK) {
                *yaw_deg = edge_yaw;
                yaw_valid = 1U;
            }
            if (align_stop() != ALIGN_OK) {
                return ALIGN_ERR;
            }
            if (yaw_valid != 0U) {
                return ALIGN_OK;
            }
            return align_yaw_read(yaw_deg);
        }
        if (level == LSENSOR_LEVEL_INVALID) {
            break;
        }
        osDelay(AL_SCAN_MS);
    }
    (void)align_stop();
    return ALIGN_ERR;
}

/**
 * @brief  以定速原地旋转到两个白线边缘角度的中值
 * @param  target IMU 原始目标航向，deg
 * @retval ALIGN_OK / ALIGN_ERR
 */
static align_status_t turn_mid(float target)
{
    float yaw = 0.0f;                        /* 当前 IMU 原始航向 */
    float err;                               /* 最短航向误差 */
    float prev_err;                          /* 上次航向误差 */
    float wz;                                /* 底盘旋转指令 */
    uint32_t start = osKernelGetTickCount(); /* 回中超时起点 */
    uint32_t tmo = util_ms_ticks(AL_TURN_TMO_MS); /* 超时 tick 数 */

    if (align_yaw_read(&yaw) != ALIGN_OK) {
        return ALIGN_ERR;
    }
    err = util_ang_norm(target - yaw);
    if ((err >= -AL_YAW_TOL_DEG) && (err <= AL_YAW_TOL_DEG)) {
        return align_stop();
    }

    /* 底盘命令与 IMU 原始 yaw 同向，按最短误差选择旋转方向。 */
    wz = (err > 0.0f) ? AL_TURN_W_DEG : -AL_TURN_W_DEG;
    prev_err = err;
    if (csvc_free(0.0f, 0.0f, wz) != CSVC_OK) {
        return ALIGN_ERR;
    }
    while ((osKernelGetTickCount() - start) < tmo) {
        if (align_yaw_read(&yaw) != ALIGN_OK) {
            break;
        }
        err = util_ang_norm(target - yaw);
        if (((err >= -AL_YAW_TOL_DEG) && (err <= AL_YAW_TOL_DEG)) ||
            ((err * prev_err) <= 0.0f)) {
            return align_stop();
        }
        prev_err = err;
        osDelay(AL_SCAN_MS);
    }
    (void)align_stop();
    return ALIGN_ERR;
}

/**
 * @brief  软件标定：把当前 IMU 航向标定为指定角度（带重试）
 * @param  yaw_deg IMU 帧目标航向，deg
 * @retval ALIGN_OK / ALIGN_ERR
 */
static align_status_t imu_calib(float yaw_deg)
{
    uint32_t start = osKernelGetTickCount(); /* 超时起点 */
    uint32_t tmo = util_ms_ticks(AL_IMU_TMO_MS); /* 超时 tick 数 */

    while ((osKernelGetTickCount() - start) < tmo) {
        if (hwt101_adp_set_yaw(yaw_deg) == HWT101_OK) {
            return ALIGN_OK;
        }
        osDelay(AL_SCAN_MS);
    }
    return ALIGN_ERR;
}

/** @copydoc align_seek_line */
align_status_t align_seek_line(void)
{
    lsensor_level_t sen_2;  /* 2 号循迹传感器电平 */
    lsensor_level_t sen_5;  /* 5 号循迹传感器电平 */
    uint8_t found = 0U;     /* 双传感器同时压线标志 */

    if (csvc_free(AL_SEEK_VX_MMS, 0.0f, 0.0f) != CSVC_OK) {
        AL_LOGE("right move fail");
    } else {
        for (;;) {
            sen_2 = lsh_get_level(LSENSOR_ID_2);
            sen_5 = lsh_get_level(LSENSOR_ID_5);
            if ((sen_2 == LSENSOR_LEVEL_LOW) &&
                (sen_5 == LSENSOR_LEVEL_LOW)) {
                found = 1U;
                break;
            }
            if ((sen_2 == LSENSOR_LEVEL_INVALID) ||
                (sen_5 == LSENSOR_LEVEL_INVALID)) {
                AL_LOGE("lsensor invalid");
                break;
            }
            osDelay(AL_SCAN_MS);
        }
    }
    if (csvc_free(0.0f, 0.0f, 0.0f) != CSVC_OK) {
        AL_LOGE("stop fail");
        return ALIGN_ERR;
    }
    osDelay(AL_SETTLE_MS);
    return (found != 0U) ? ALIGN_OK : ALIGN_ERR;
}

/** @copydoc align_white_line */
align_status_t align_white_line(map_point_t pos, float yaw_deg)
{
    float cw_yaw = 0.0f;   /* 5 号离线时的顺时针边缘角度 */
    float ccw_yaw = 0.0f;  /* 2 号离线时的逆时针边缘角度 */
    float mid_yaw;         /* 两个边缘的环形角度中值 */

    if (seek_edge(LSENSOR_ID_5, AL_CW_W_DEG, &cw_yaw) != ALIGN_OK) {
        AL_LOGE("sensor 5 edge fail");
        return ALIGN_ERR;
    }
    if (seek_edge(LSENSOR_ID_2, AL_CCW_W_DEG, &ccw_yaw) != ALIGN_OK) {
        AL_LOGE("sensor 2 edge fail");
        return ALIGN_ERR;
    }
    mid_yaw = util_ang_norm(cw_yaw +
                            (0.5f * util_ang_norm(ccw_yaw - cw_yaw)) +
                            AL_MID_YAW_OFS);
    AL_LOGI("line yaw cw=%d ccw=%d mid=%d",
            (int)cw_yaw, (int)ccw_yaw, (int)mid_yaw);
    if (turn_mid(mid_yaw) != ALIGN_OK) {
        AL_LOGE("turn mid fail");
        return ALIGN_ERR;
    }
    /* IMU 帧与世界帧反号（chassis 侧取负对齐地图系） */
    if (imu_calib(-yaw_deg) != ALIGN_OK) {
        AL_LOGE("imu calib fail");
        return ALIGN_ERR;
    }
    if (csvc_set_pose(pos, yaw_deg) != CSVC_OK) {
        AL_LOGE("set pose fail");
        return ALIGN_ERR;
    }
    return ALIGN_OK;
}
