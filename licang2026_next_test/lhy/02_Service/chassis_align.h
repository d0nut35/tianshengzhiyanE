/**
 * @file    chassis_align.h
 * @brief   白线对齐与航向标定：灰度扫边 + IMU 回中 + 位姿标定
 * @note    - 全部为阻塞接口，只能在任务上下文调用
 *          - 每个动作退出前均已投递停车命令，失败时底盘保持静止
 *          - 依赖 csvc 已初始化、lsensor 与 hwt101 已启动
 */

#ifndef CHASSIS_ALIGN_H
#define CHASSIS_ALIGN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "map.h"   /* map_point_t：标定坐标（mm） */

/* 状态码：OK==0 */
typedef enum {
    ALIGN_OK  = 0,  /* 对齐完成 */
    ALIGN_ERR = 1,  /* 传感器无效、超时或命令下发失败（已停车） */
} align_status_t;

/**
 * @brief  投递停车命令并等待底盘机械稳定
 * @retval ALIGN_OK / ALIGN_ERR
 */
align_status_t align_stop(void);

/**
 * @brief  在限定时间内读取一帧有效 IMU 原始航向
 * @param  yaw_deg 输出原始航向，deg
 * @retval ALIGN_OK / ALIGN_ERR
 */
align_status_t align_yaw_read(float *yaw_deg);

/**
 * @brief  右移找线：2、5 号同时压线后停车
 * @retval ALIGN_OK=找到线且已停稳 / ALIGN_ERR
 * @note   无论是否找到线均投递停车命令
 */
align_status_t align_seek_line(void);

/**
 * @brief  扫取白线两侧边缘、回中并按指定位姿软件标定航向
 * @param  pos     对齐完成后的世界系标定坐标
 * @param  yaw_deg 对齐完成后的世界系航向，deg
 * @retval ALIGN_OK / ALIGN_ERR
 */
align_status_t align_white_line(map_point_t pos, float yaw_deg);

#ifdef __cplusplus
}
#endif

#endif /* CHASSIS_ALIGN_H */
