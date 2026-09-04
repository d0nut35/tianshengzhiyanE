/**
 * @file    chassis_adaption.h
 * @brief   底盘场景适配层（本机几何/整定 + 单位换算，对上唯一入口）
 * @author  haoyu
 * @note    - 上层命令唯一合法入口，负责参数校验；角度对上一律收 deg，内部转 rad
 *          - 直接持电机 handler：下发调 mh_speed，里程计调 mh_get_pos 自取脉冲
 *          - 轮速 rps→电机帧（×系数 + 轮序映射）在本层 .c 完成
 *          - 本机几何/标定常量见本层 .c，换车只改一处
 */

#ifndef CHASSIS_ADAPTION_H
#define CHASSIS_ADAPTION_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

#include "mecanum_chassis.h"     /* chassis_status_t / chassis_odom_t / map_point_t */
#include "zdt_motor_handler.h"   /* motor_handler_t：下发/读位置的总线句柄 */

/**
 * @brief  初始化底盘：载入本机几何，持 handler 句柄并实例化纯逻辑
 * @param  mh 已启动的电机 handler（system_assembly 建好并注入 OS），不可为空
 * @retval CHASSIS_OK / CHASSIS_ERR_PARAM / CHASSIS_ERR_INIT
 */
chassis_status_t chassis_init(motor_handler_t *mh);

/**
 * @brief  车体系速度控制（自由运动模式）
 * @param  vx 车体系 x 速度，单位 mm/s
 * @param  vy 车体系 y 速度，单位 mm/s
 * @param  wz 角速度，单位 deg/s
 * @retval CHASSIS_OK / CHASSIS_ERR_PARAM / CHASSIS_ERR_INIT
 */
chassis_status_t chassis_set_vel(float vx, float vy, float wz);

/**
 * @brief  线速度+转弯半径控制（画圆/原地自转）
 * @param  linear 线速度，单位 mm/s
 * @param  r      转弯半径，单位 mm
 * @param  insitu true=原地自转
 * @retval CHASSIS_OK / CHASSIS_ERR_PARAM / CHASSIS_ERR_INIT
 */
chassis_status_t chassis_set_radius(float linear, float r, bool insitu);

/**
 * @brief  点到点导航一个周期（航向/角速度入参 deg，内部转 rad）
 * @param  target  目标点（地图系 mm）
 * @param  yaw_deg 目标航向，单位 deg
 * @param  v       线速度上限，单位 mm/s
 * @param  w       角速度上限，单位 deg/s
 * @param  finished 输出：1=到位，0=未到，可为空
 * @retval CHASSIS_OK / CHASSIS_ERR_PARAM / CHASSIS_ERR_INIT
 */
chassis_status_t chassis_nav(map_point_t target, float yaw_deg,
                             float v, float w, uint8_t *finished);

/**
 * @brief  路径跟随一个周期（航向入参 deg，内部转 rad）
 * @param  path     路径点数组（地图系 mm）
 * @param  len      路径点数，须 >0
 * @param  cruise_v 巡航线速度，单位 mm/s
 * @param  yaw_deg  跟随目标航向，单位 deg
 * @param  finished 输出：1=到终点，不可为空
 * @retval CHASSIS_OK / CHASSIS_ERR_PARAM / CHASSIS_ERR_INIT
 */
chassis_status_t chassis_follow(const map_point_t *path, uint16_t len,
                                float cruise_v, float yaw_deg,
                                uint8_t *finished);

/**
 * @brief  里程计更新：内部 mh_get_pos 读四轮脉冲并差分，yaw deg→rad
 * @param  yaw_deg 真实航向，单位 deg
 * @param  dt_s    控制周期，单位 s，须 >0
 * @retval CHASSIS_OK / CHASSIS_ERR_PARAM / CHASSIS_ERR_INIT
 */
chassis_status_t chassis_odom(float yaw_deg, float dt_s);

/**
 * @brief  读取里程计快照（位姿/速度，供上报）
 * @param  out 输出里程计状态，不可为空
 * @retval CHASSIS_OK / CHASSIS_ERR_PARAM / CHASSIS_ERR_INIT
 */
chassis_status_t chassis_get_odom(chassis_odom_t *out);

/**
 * @brief  设置里程计初始位姿（重定位用，航向入参 deg）
 * @param  pos     初始位置（地图系 mm）
 * @param  yaw_deg 航向角，单位 deg
 * @retval CHASSIS_OK / CHASSIS_ERR_PARAM / CHASSIS_ERR_INIT
 */
chassis_status_t chassis_set_pose(map_point_t pos, float yaw_deg);

/**
 * @brief  里程计清零（位姿/速度归零）
 * @retval CHASSIS_OK / CHASSIS_ERR_INIT
 */
chassis_status_t chassis_reset_odom(void);

/**
 * @brief  立即停车
 * @retval CHASSIS_OK / CHASSIS_ERR_INIT
 */
chassis_status_t chassis_stop(void);

#ifdef __cplusplus
}
#endif

#endif /* CHASSIS_ADAPTION_H */
