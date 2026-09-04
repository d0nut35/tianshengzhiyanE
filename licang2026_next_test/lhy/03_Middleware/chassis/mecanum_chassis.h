/**
 * @file    mecanum_chassis.h
 * @brief   麦克纳姆底盘运动学/里程计/路径跟随（纯逻辑，平台无关）
 * @author  haoyu
 * @note    - 仅依赖标准库与同层 map.h；不碰 HAL/RTOS/电机协议
 *          - 四轮转速经 pf 回调以 rps 交出，rps→电机帧由适配层实现
 *          - 几何参数由适配层注入；导航整定为本层 .c 私有宏，不外泄
 *          - 轮序固定 lf,lr,rf,rr；角度一律 rad、长度一律 mm
 */

#ifndef MECANUM_CHASSIS_H
#define MECANUM_CHASSIS_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

#include "map.h"     /* map_point_t：路径点（地图系 mm） */

/* 状态码：OK==0，纯逻辑与适配层共用，便于逐层映射 */
typedef enum {
    CHASSIS_OK        = 0,  /* 成功 */
    CHASSIS_ERR       = 1,  /* 通用错误 */
    CHASSIS_ERR_PARAM = 2,  /* 参数非法（空指针/范围越界） */
    CHASSIS_ERR_INIT  = 3,  /* 未初始化或回调未注入 */
} chassis_status_t;

/* 里程计快照（地图系 mm / rad），供上层上报 */
typedef struct {
    float x_mm;     /* 地图系 x，单位 mm */
    float y_mm;     /* 地图系 y，单位 mm */
    float yaw_rad;  /* 航向角，单位 rad */
    float vx;       /* 车体系 x 速度，单位 mm/s */
    float vy;       /* 车体系 y 速度，单位 mm/s */
    float w;        /* 角速度，单位 rad/s */
} chassis_odom_t;

/* 麦轮几何参数（本机标定，由适配层注入） */
typedef struct {
    float half_base_mm;   /* 半轮距：左右轮距一半，单位 mm */
    float half_shaft_mm;  /* 半轴距：前后轴距一半，单位 mm */
    float wheel_dia_mm;   /* 轮直径，单位 mm */
    float correction;     /* 轮速标定系数，无量纲 */
    float enc_ppr;        /* 编码器每圈脉冲数 */
} mecanum_geom_t;

/* 一个控制周期内的四轮编码器脉冲增量 */
typedef struct {
    int32_t lf;  /* 左前轮脉冲增量 */
    int32_t lr;  /* 左后轮脉冲增量 */
    int32_t rf;  /* 右前轮脉冲增量 */
    int32_t rr;  /* 右后轮脉冲增量 */
} mecanum_delta_t;

typedef struct mecanum_chassis mecanum_t;

/* 轮速下发回调：四轮转速以 rps 交出（rps→电机帧、轮序映射由注入方实现） */
typedef chassis_status_t (*chassis_motors_fn_t)(void *ctx,
                                                float lf_rps,
                                                float lr_rps,
                                                float rf_rps,
                                                float rr_rps);

/* 麦轮底盘对象：字段为内部状态，外部经 API 访问，勿直接改写 */
struct mecanum_chassis {
    uint8_t is_inited;    /* 是否已实例化 */
    uint8_t odom_inited;  /* 里程计是否已收到首帧 yaw */
    mecanum_geom_t geom;  /* 几何参数 */
    chassis_odom_t odom;  /* 里程计状态 */
    float last_yaw;       /* 上周期真实 yaw，单位 rad */
    float nav_lin_cmd;    /* 导航线速度平滑指令，单位 mm/s */
    float nav_ang_cmd;    /* 导航角速度平滑指令，单位 rad/s */
    uint8_t  path_active; /* 路径跟随是否激活 */
    uint16_t path_index;  /* 当前路径点下标 */
    uint32_t path_sig;    /* 当前路径签名，用于识别新路径 */
    uint16_t path_look;   /* 上次前瞻点下标（只前不退） */
    float    path_dir;    /* 运动方向角滤波状态，单位 rad */
    float    path_yaw_i;  /* 路径航向 PID 积分累积，单位 rad*s */
    float    path_yaw_e;  /* 路径航向 PID 上次误差，单位 rad */
    uint8_t  path_pid_on; /* 路径航向 PID 是否已初始化 */
    chassis_motors_fn_t pf;  /* 轮速下发回调 */
    void *motor_ctx;      /* 回调上下文，透传给 pf */
};

/**
 * @brief  实例化底盘对象并注入几何参数与轮速回调
 * @param  self 底盘对象
 * @param  geom 几何参数，字段须为正
 * @param  pf   轮速下发回调，不可为空
 * @param  ctx  回调上下文，透传，可为空
 * @retval CHASSIS_OK / CHASSIS_ERR_PARAM
 */
chassis_status_t mecanum_inst(mecanum_t *self,
                              const mecanum_geom_t *geom,
                              chassis_motors_fn_t pf,
                              void *ctx);

/**
 * @brief  按车体系速度驱动底盘（运动学逆解→四轮 rps）
 * @param  self 底盘对象
 * @param  vx   车体系 x 速度，单位 mm/s
 * @param  vy   车体系 y 速度，单位 mm/s
 * @param  wz   角速度，单位 rad/s
 * @retval CHASSIS_OK / CHASSIS_ERR_PARAM / CHASSIS_ERR_INIT
 */
chassis_status_t mecanum_set_vel(mecanum_t *self, float vx, float vy, float wz);

/**
 * @brief  按线速度与转弯半径驱动底盘（画圆/原地自转）
 * @param  self   底盘对象
 * @param  linear 线速度，单位 mm/s
 * @param  r      转弯半径，单位 mm
 * @param  insitu true=原地自转，false=带半径转弯
 * @retval CHASSIS_OK / CHASSIS_ERR_PARAM / CHASSIS_ERR_INIT
 */
chassis_status_t mecanum_set_radius(mecanum_t *self, float linear, float r,
                                    bool insitu);

/**
 * @brief  立即停车（四轮速度清零）
 * @param  self 底盘对象
 * @retval CHASSIS_OK / CHASSIS_ERR_PARAM / CHASSIS_ERR_INIT
 */
chassis_status_t mecanum_stop(mecanum_t *self);

/**
 * @brief  用四轮编码器增量与真实 yaw 更新里程计（一次=一个周期）
 * @param  self    底盘对象
 * @param  delta   本周期四轮脉冲增量
 * @param  yaw_rad 外部真实航向角，单位 rad
 * @param  dt_s    控制周期，单位 s，须 >0
 * @retval CHASSIS_OK / CHASSIS_ERR_PARAM / CHASSIS_ERR_INIT
 */
chassis_status_t mecanum_odom(mecanum_t *self, mecanum_delta_t delta,
                              float yaw_rad, float dt_s);

/**
 * @brief  执行一个非阻塞点到点导航周期（位置+航向闭环）
 * @param  self  底盘对象
 * @param  tx_mm 目标地图系 x，单位 mm
 * @param  ty_mm 目标地图系 y，单位 mm
 * @param  t_yaw 目标航向，单位 rad
 * @param  t_v   本周期线速度上限，单位 mm/s
 * @param  t_w   本周期角速度上限，单位 rad/s
 * @param  finished 输出：1=到位（位置+航向双容差），0=未到，可为空
 * @retval CHASSIS_OK / CHASSIS_ERR_PARAM / CHASSIS_ERR_INIT
 */
chassis_status_t mecanum_navigate(mecanum_t *self, float tx_mm, float ty_mm,
                                  float t_yaw, float t_v, float t_w,
                                  uint8_t *finished);

/**
 * @brief  执行一个非阻塞路径跟随周期（前瞻巡线 + 航向 PID）
 * @param  self     底盘对象
 * @param  path     路径点数组，地图系 mm，与里程计同坐标
 * @param  len      路径点数，须 >0
 * @param  cruise_v 巡航线速度，单位 mm/s
 * @param  t_yaw    跟随目标航向，单位 rad
 * @param  finished 输出：1=已到终点，0=未到，不可为空
 * @retval CHASSIS_OK / CHASSIS_ERR_PARAM / CHASSIS_ERR_INIT
 */
chassis_status_t mecanum_follow(mecanum_t *self, const map_point_t *path,
                                uint16_t len, float cruise_v, float t_yaw,
                                uint8_t *finished);

/**
 * @brief  读取里程计快照（供上层上报/显示）
 * @param  self 底盘对象
 * @param  out  输出里程计状态，不可为空
 * @retval CHASSIS_OK / CHASSIS_ERR_PARAM / CHASSIS_ERR_INIT
 */
chassis_status_t mecanum_get_odom(const mecanum_t *self, chassis_odom_t *out);

/**
 * @brief  里程计清零（位姿/速度归零，下一帧 yaw 重新对齐）
 * @param  self 底盘对象
 * @retval CHASSIS_OK / CHASSIS_ERR_PARAM / CHASSIS_ERR_INIT
 */
chassis_status_t mecanum_reset_odom(mecanum_t *self);

/**
 * @brief  设置里程计初始位姿（重定位用）
 * @param  self    底盘对象
 * @param  x_mm    地图系 x，单位 mm
 * @param  y_mm    地图系 y，单位 mm
 * @param  yaw_rad 航向角，单位 rad
 * @retval CHASSIS_OK / CHASSIS_ERR_PARAM / CHASSIS_ERR_INIT
 */
chassis_status_t mecanum_set_pose(mecanum_t *self, float x_mm, float y_mm,
                                  float yaw_rad);

#ifdef __cplusplus
}
#endif

#endif /* MECANUM_CHASSIS_H */
