/**
 * @file    app_route.h
 * @brief   任务点编排：按点表执行导航 → 找线 → 位姿标定
 * @note    - 所有场地坐标集中在 app_route.c 顶部的 g_route 表中（默认图）
 *          - 场地侧由 route_set_side 选择：镜像图关于 x=1250 对称换算
 *          - 阻塞接口，只能在底盘任务上下文调用
 */

#ifndef APP_ROUTE_H
#define APP_ROUTE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#include "app_main.h"   /* app_status_t */

/* 任务点编号，与 g_route 表下标一一对应 */
typedef enum {
    ROUTE_PLATFORM = 0,  /* 圆盘工作位 */
    ROUTE_STAIRS,        /* 阶梯起始工作位，即低层起点 */
    ROUTE_CYL_PRE,       /* 圆柱前置点，直线切入用 */
    ROUTE_CYL,           /* 圆柱绕圈起点 */
    ROUTE_DEPOT,         /* 立体仓库 1 号工作位，2~4 号由 app_main 横移到达 */
    ROUTE_HOME,          /* 起点 */
    ROUTE_NUM            /* 任务点总数 */
} route_id_t;

/* 场地侧：g_route 为默认图，镜像图为其关于 x=1250 的对称像 */
typedef enum {
    ROUTE_SIDE_DEFAULT = 0,  /* 默认地图，点表原值 */
    ROUTE_SIDE_MIRROR,       /* 镜像地图：x→2500-x，航向→180°-θ，y 不变 */
} route_side_t;

/**
 * @brief  选择场地侧：重载该侧 A* 障碍，之后 route_go 按该侧换算点表
 * @param  side 场地侧
 * @retval APP_OK / APP_ERR=入参非法或障碍重载失败
 * @note   需在底盘静止、无导航进行时调用；上电默认 ROUTE_SIDE_DEFAULT
 */
app_status_t route_set_side(route_side_t side);

/**
 * @brief  读取当前场地侧
 * @retval ROUTE_SIDE_DEFAULT / ROUTE_SIDE_MIRROR
 */
route_side_t route_side(void);

/**
 * @brief  把默认图 x 换算到当前场地侧
 * @param  x_mm 默认图 x，mm
 * @retval 当前侧 x，mm（默认侧原值返回）
 */
int16_t route_side_x(int16_t x_mm);

/**
 * @brief  车体系横向量（vy、画圆线速度）的场地侧符号
 * @retval 默认侧 +1.0 / 镜像侧 -1.0
 * @note   镜像后机器人在同一地图方向上左右互换，横向速度需取反
 */
float route_lat_sign(void);

/**
 * @brief  执行一个任务点：导航到点，按表决定是否找线与标定位姿
 * @param  id 任务点编号
 * @retval APP_OK / APP_ERR=编号非法、导航失败或对齐失败
 */
app_status_t route_go(route_id_t id);

#ifdef __cplusplus
}
#endif

#endif /* APP_ROUTE_H */
