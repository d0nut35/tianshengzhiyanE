/**
 * @file    app_route.h
 * @brief   任务点编排：按点表执行导航 → 找线 → 位姿标定
 * @note    - 所有场地坐标集中在 app_route.c 顶部的 g_route 表中
 *          - 阻塞接口，只能在底盘任务上下文调用
 */

#ifndef APP_ROUTE_H
#define APP_ROUTE_H

#ifdef __cplusplus
extern "C" {
#endif

#include "app_main.h"   /* app_status_t */

/* 任务点编号，与 g_route 表下标一一对应 */
typedef enum {
    ROUTE_PLATFORM = 0,  /* 圆盘工作位 */
    ROUTE_STAIRS,        /* 阶梯起始工作位，即低层起点 */
    ROUTE_DEPOT,         /* 立体仓库工作位 */
    ROUTE_HOME,          /* 起点 */
    ROUTE_NUM            /* 任务点总数 */
} route_id_t;

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
