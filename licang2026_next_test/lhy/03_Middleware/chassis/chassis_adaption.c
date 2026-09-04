/**
 * @file    chassis_adaption.c
 * @brief   底盘场景适配层（本机几何/标定 + 单位换算，对上唯一入口）
 * @author  haoyu
 * @note    - 角度对上收 deg、内部转 rad；odom 的 yaw 取负对齐里程计地图系
 *          - 轮速 rps→电机：×CHASSIS_RPS_RAW，按 idx0=lf,1=rf,2=lr,3=rr 下发
 *          - 里程计自调 mh_get_pos 差分；首帧/重定位仅取基准不积分
 */

#include "chassis_adaption.h"

#include <stddef.h> /* NULL */
#include <math.h>   /* isfinite：yaw 脏值兜底 */

/* ---- 本机几何标定（换车改这里，取自原可用版本实测值） ---- */
#define CHASSIS_HALF_BASE_MM   128.94f   /* 半轮距：257.88/2，mm */
#define CHASSIS_HALF_SHAFT_MM  130.575f  /* 半轴距：261.15/2，mm */
#define CHASSIS_WHEEL_DIA_MM   75.0f     /* 轮直径，mm */
#define CHASSIS_CORRECTION     1.0f      /* 轮速标定系数 */
#define CHASSIS_ENC_PPR        65535.0f  /* 编码器每圈脉冲 */

/* ---- 轮速换算与下发 ---- */
#define CHASSIS_RPS_RAW        600.0f    /* rps→电机速度档（rpm×10）系数 */
#define CHASSIS_ACCEL          0xFFU     /* 速度帧加速度档 */
#define CHASSIS_WHEEL_NUM      4U        /* 同步驱动轮数 */

/* ---- 轮↔总线 idx 映射（依原可用版本接线：addr1=lf,2=rf,3=lr,4=rr） ---- */
#define CHASSIS_IDX_LF         0U        /* 左前轮 idx */
#define CHASSIS_IDX_RF         1U        /* 右前轮 idx */
#define CHASSIS_IDX_LR         2U        /* 左后轮 idx */
#define CHASSIS_IDX_RR         3U        /* 右后轮 idx */

#define CHASSIS_DEG2RAD        0.01745329252f /* deg→rad */
#define CHASSIS_INITED         1U        /* 标志位置位值 */

static mecanum_t        s_chassis;                  /* 底盘纯逻辑实例 */
static motor_handler_t *s_mh = NULL;                /* 电机 handler 句柄 */
static int64_t          s_last[CHASSIS_WHEEL_NUM];  /* 上次四轮绝对脉冲 */
static float            s_last_yaw = 0.0f;          /* 上次有效 yaw，deg */
static uint8_t          s_pos_on = 0U;              /* 脉冲基准是否已取 */
static uint8_t          s_inited = 0U;              /* 适配层是否已初始化 */

/**
 * @brief  轮速回调：四轮 rps→电机速度档，按 idx 一帧广播下发
 * @param  ctx 未用（handler 取文件级 s_mh）
 * @param  lf_rps/lr_rps/rf_rps/rr_rps 四轮转速，rps
 * @retval CHASSIS_OK / CHASSIS_ERR_INIT / CHASSIS_ERR
 */
static chassis_status_t adp_set_motors(void *ctx, float lf_rps, float lr_rps,
                                       float rf_rps, float rr_rps)
{
    int16_t sp[CHASSIS_WHEEL_NUM]; /* 按 idx 排布的速度档 */

    (void)ctx;
    if (s_mh == NULL) {
        return CHASSIS_ERR_INIT;
    }
    sp[CHASSIS_IDX_LF] = (int16_t)(lf_rps * CHASSIS_RPS_RAW);
    sp[CHASSIS_IDX_RF] = (int16_t)(rf_rps * CHASSIS_RPS_RAW);
    sp[CHASSIS_IDX_LR] = (int16_t)(lr_rps * CHASSIS_RPS_RAW);
    sp[CHASSIS_IDX_RR] = (int16_t)(rr_rps * CHASSIS_RPS_RAW);
    if (mh_speed(s_mh, sp, CHASSIS_WHEEL_NUM, CHASSIS_ACCEL) != ZDT_OK) {
        return CHASSIS_ERR;
    }
    return CHASSIS_OK;
}

chassis_status_t chassis_init(motor_handler_t *mh)
{
    chassis_status_t ret = CHASSIS_ERR; /* 实例化结果 */
    mecanum_geom_t geom;               /* 本机几何 */
    uint8_t i = 0U;                    /* 轮下标 */

    if (mh == NULL) {
        return CHASSIS_ERR_PARAM;
    }
    s_mh = mh;
    geom.half_base_mm  = CHASSIS_HALF_BASE_MM;
    geom.half_shaft_mm = CHASSIS_HALF_SHAFT_MM;
    geom.wheel_dia_mm  = CHASSIS_WHEEL_DIA_MM;
    geom.correction    = CHASSIS_CORRECTION;
    geom.enc_ppr       = CHASSIS_ENC_PPR;
    ret = mecanum_inst(&s_chassis, &geom, adp_set_motors, NULL);
    if (ret != CHASSIS_OK) {
        return ret;
    }
    for (i = 0U; i < CHASSIS_WHEEL_NUM; i++) {
        s_last[i] = 0;
    }
    s_pos_on = 0U;
    s_inited = CHASSIS_INITED;
    return CHASSIS_OK;
}

chassis_status_t chassis_set_vel(float vx, float vy, float wz)
{
    if (s_inited != CHASSIS_INITED) {
        return CHASSIS_ERR_INIT;
    }
    return mecanum_set_vel(&s_chassis, vx, vy, wz * CHASSIS_DEG2RAD);
}

chassis_status_t chassis_set_radius(float linear, float r, bool insitu)
{
    if (s_inited != CHASSIS_INITED) {
        return CHASSIS_ERR_INIT;
    }
    return mecanum_set_radius(&s_chassis, linear, r, insitu);
}

chassis_status_t chassis_nav(map_point_t target, float yaw_deg, float v, float w,
                             uint8_t *finished)
{
    if (s_inited != CHASSIS_INITED) {
        return CHASSIS_ERR_INIT;
    }
    /* 目标点/航向均为地图系，直接转 rad（不取负） */
    return mecanum_navigate(&s_chassis, (float)target.x_mm, (float)target.y_mm,
                            yaw_deg * CHASSIS_DEG2RAD, v, w * CHASSIS_DEG2RAD,
                            finished);
}

chassis_status_t chassis_follow(const map_point_t *path, uint16_t len,
                                float cruise_v, float yaw_deg, uint8_t *finished)
{
    if (s_inited != CHASSIS_INITED) {
        return CHASSIS_ERR_INIT;
    }
    return mecanum_follow(&s_chassis, path, len, cruise_v,
                          yaw_deg * CHASSIS_DEG2RAD, finished);
}

chassis_status_t chassis_odom(float yaw_deg, float dt_s)
{
    mecanum_delta_t delta;           /* 四轮脉冲增量 */
    int64_t pos[CHASSIS_WHEEL_NUM];  /* 四轮当前绝对脉冲 */
    uint8_t i = 0U;                  /* 轮下标 */

    if (s_inited != CHASSIS_INITED) {
        return CHASSIS_ERR_INIT;
    }
    /* yaw 兜底：非有限值（NaN/Inf）沿用上次，防脏值污染积分 */
    if (!isfinite(yaw_deg)) {
        yaw_deg = s_last_yaw;
    } else {
        s_last_yaw = yaw_deg;
    }
    for (i = 0U; i < CHASSIS_WHEEL_NUM; i++) {
        if (mh_get_pos(s_mh, i, &pos[i]) != ZDT_OK) {
            return CHASSIS_ERR;
        }
    }
    /* 首帧/重定位后仅取基准，本周期不积分 */
    if (s_pos_on != CHASSIS_INITED) {
        for (i = 0U; i < CHASSIS_WHEEL_NUM; i++) {
            s_last[i] = pos[i];
        }
        s_pos_on = CHASSIS_INITED;
        return CHASSIS_OK;
    }
    /* 差分成增量，按 idx 映射轮位 */
    delta.lf = (int32_t)(pos[CHASSIS_IDX_LF] - s_last[CHASSIS_IDX_LF]);
    delta.rf = (int32_t)(pos[CHASSIS_IDX_RF] - s_last[CHASSIS_IDX_RF]);
    delta.lr = (int32_t)(pos[CHASSIS_IDX_LR] - s_last[CHASSIS_IDX_LR]);
    delta.rr = (int32_t)(pos[CHASSIS_IDX_RR] - s_last[CHASSIS_IDX_RR]);
    for (i = 0U; i < CHASSIS_WHEEL_NUM; i++) {
        s_last[i] = pos[i];
    }
    /* hwt101 原始 yaw 取负对齐地图系，再转 rad */
    return mecanum_odom(&s_chassis, delta, -yaw_deg * CHASSIS_DEG2RAD, dt_s);
}

chassis_status_t chassis_get_odom(chassis_odom_t *out)
{
    if (s_inited != CHASSIS_INITED) {
        return CHASSIS_ERR_INIT;
    }
    return mecanum_get_odom(&s_chassis, out);
}

chassis_status_t chassis_set_pose(map_point_t pos, float yaw_deg)
{
    if (s_inited != CHASSIS_INITED) {
        return CHASSIS_ERR_INIT;
    }
    s_pos_on = 0U; /* 重定位后脉冲基准失效，下帧重取 */
    return mecanum_set_pose(&s_chassis, (float)pos.x_mm, (float)pos.y_mm,
                            yaw_deg * CHASSIS_DEG2RAD);
}

chassis_status_t chassis_reset_odom(void)
{
    if (s_inited != CHASSIS_INITED) {
        return CHASSIS_ERR_INIT;
    }
    s_pos_on = 0U; /* 脉冲基准一并重置 */
    return mecanum_reset_odom(&s_chassis);
}

chassis_status_t chassis_stop(void)
{
    if (s_inited != CHASSIS_INITED) {
        return CHASSIS_ERR_INIT;
    }
    return mecanum_stop(&s_chassis);
}
