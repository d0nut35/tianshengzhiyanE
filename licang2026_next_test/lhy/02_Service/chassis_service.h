/**
 * @file    chassis_service.h
 * @brief   底盘服务：命令队列 + 20ms 控制任务 + 导航仲裁（Service 层）
 * @author  haoyu
 * @note    - csvc = chassis service；对 App 暴露命令投递，内部起任务驱动 chassis
 *          - 角度一律 deg、目标用 map_point_t；坐标为里程计/世界系 mm
 *          - init 内接入 chassis_init(system_motor_handler()) + map_adp_init
 */

#ifndef CHASSIS_SERVICE_H
#define CHASSIS_SERVICE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

#include "map.h"     /* map_point_t：目标点/路径点（mm） */

/* 导航子模式（csvc_nav 的 mode 入参） */
#define CSVC_NAV_LINE   0x01U   /* 直线到点导航 */
#define CSVC_NAV_PATH   0x02U   /* A* 规划 + 路径跟随 */

/* 状态码：OK==0 */
typedef enum {
    CSVC_OK        = 0,  /* 成功 */
    CSVC_ERR       = 1,  /* 通用错误 */
    CSVC_ERR_PARAM = 2,  /* 参数非法 */
    CSVC_ERR_INIT  = 3,  /* 未初始化 */
    CSVC_ERR_RES   = 4,  /* 资源不可用（队列/任务建失败） */
    CSVC_BUSY      = 5,  /* 上一条导航未完成，拒绝新导航 */
} csvc_status_t;

/**
 * @brief  初始化底盘服务：接入 chassis/map、建命令队列与位姿锁、起三线程
 * @retval CSVC_OK / CSVC_ERR_INIT / CSVC_ERR_RES
 */
csvc_status_t csvc_init(void);

/**
 * @brief  重定位里程计初始位姿（上电对齐世界系，持位姿锁）
 * @param  pos     世界系坐标，mm
 * @param  yaw_deg 航向，deg
 * @retval CSVC_OK / CSVC_ERR_INIT / CSVC_ERR
 * @note   替代直接调 chassis_set_pose，避免与里程计线程并发写位姿
 */
csvc_status_t csvc_set_pose(map_point_t pos, float yaw_deg);

/**
 * @brief  投递自由速度命令（车体系）
 * @param  vx 车体系 x 速度，mm/s
 * @param  vy 车体系 y 速度，mm/s
 * @param  wz 角速度，deg/s
 * @retval CSVC_OK / CSVC_ERR_INIT / CSVC_ERR
 */
csvc_status_t csvc_free(float vx, float vy, float wz);

/**
 * @brief  投递画圆/定半径命令
 * @param  linear 线速度，mm/s
 * @param  r      转弯半径，mm
 * @param  insitu true=原地自转
 * @retval CSVC_OK / CSVC_ERR_INIT / CSVC_ERR
 */
csvc_status_t csvc_arc(float linear, float r, bool insitu);

/**
 * @brief  投递导航命令（直线到点或 A* 路径跟随）
 * @param  mode     CSVC_NAV_LINE / CSVC_NAV_PATH
 * @param  target   目标点（mm，里程计/世界系）
 * @param  yaw_deg  目标航向，deg
 * @param  v        线速度上限，mm/s
 * @param  w        角速度上限，deg/s（PATH 模式忽略）
 * @param  finished 完成标志输出，不可为空（0=进行中，1=已到）
 * @retval CSVC_OK / CSVC_BUSY / CSVC_ERR_PARAM / CSVC_ERR_INIT / CSVC_ERR
 */
csvc_status_t csvc_nav(uint8_t mode, map_point_t target, float yaw_deg,
                       float v, float w, uint8_t *finished);

#ifdef __cplusplus
}
#endif

#endif /* CHASSIS_SERVICE_H */
