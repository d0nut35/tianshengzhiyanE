/**
 * @file    app_route.c
 * @brief   任务点表与统一执行流程：导航 → 找线 → 位姿标定
 * @note    - g_route 集中全部场地坐标与速度，实机标定只改本文件的表
 *          - FIX_NONE 只导航；FIX_IMU 按当前 IMU 航向标定；FIX_LINE 扫白线回中
 *          - IMU 帧与世界帧反号（chassis 侧取负对齐地图系）
 */

#include "app_route.h"

#include <stdint.h>

#include "cmsis_os2.h"

#include "chassis_align.h"
#include "chassis_service.h"
#include "map.h"        /* map_point_t：导航与标定坐标 */

/* ===== 调试日志：0=不编译进固件，1=经 RTT 输出 ===== */
#ifndef APP_LOG_EN
#define APP_LOG_EN   0
#endif
#if APP_LOG_EN
#include "cat_log.h"
#define RT_LOGE(fmt, ...)  LOGE("[route] " fmt, ##__VA_ARGS__)
#else
#define RT_LOGE(fmt, ...)  do {} while (0)
#endif

#define RT_NAV_POLL_MS  10U   /* 等待到点的轮询周期，ms */

/* 到点后的标定方式 */
typedef enum {
    FIX_NONE = 0,  /* 只导航，不找线不标定 */
    FIX_IMU,       /* 找线后按当前 IMU 航向标定位姿 */
    FIX_LINE,      /* 找线后扫白线双边回中再标定 */
} route_fix_t;

/* 一个任务点：导航参数 + 标定参数 */
typedef struct {
    uint8_t     nav_mode;  /* CSVC_NAV_LINE / CSVC_NAV_PATH */
    map_point_t nav_pt;    /* 导航目标点，世界系 mm */
    float       yaw_deg;   /* 到点航向，兼作 FIX_LINE 的标定航向，deg */
    float       v_mms;     /* 线速度上限，mm/s */
    float       w_dps;     /* 角速度上限，deg/s */
    map_point_t fix_pt;    /* 标定坐标，世界系 mm；FIX_NONE 时不用 */
    route_fix_t fix;       /* 标定方式 */
} route_pt_t;

/* 场地坐标待实机标定；表序与 route_id_t 严格一致 */
static const route_pt_t g_route[ROUTE_NUM] = {
    /* PLATFORM：圆盘工作位，找线后按 IMU 航向标定 */
    { CSVC_NAV_PATH, { 500, 4300}, 180.0f, 500.0f, 30.0f,
      { 450, 4300}, FIX_IMU  },
    /* STAIRS：阶梯起始工作位，即低层起点 */
    { CSVC_NAV_PATH, {2050, 2950},   0.0f, 500.0f, 60.0f,
      {2130, 2950}, FIX_LINE },
    /* DEPOT：立体仓库工作位 */
    { CSVC_NAV_PATH, { 480, 2500}, 180.0f, 500.0f, 60.0f,
      { 380, 2500}, FIX_LINE },
    /* HOME：起点，直线返回不做找线对齐 */
    { CSVC_NAV_LINE, {1200,  350}, 180.0f, 500.0f, 30.0f,
      {   0,    0}, FIX_NONE },
};

static uint8_t g_nav_fin = 1U;  /* 导航完成标志，1=空闲 */

static app_status_t nav_wait(const route_pt_t *pt);
static app_status_t pose_fix(const route_pt_t *pt);

/**
 * @brief  发起导航并阻塞等待到点
 * @param  pt 任务点表项
 * @retval APP_OK / APP_ERR=命令被拒
 */
static app_status_t nav_wait(const route_pt_t *pt)
{
    g_nav_fin = 0U;
    if (csvc_nav(pt->nav_mode, pt->nav_pt, pt->yaw_deg, pt->v_mms,
                 pt->w_dps, &g_nav_fin) != CSVC_OK) {
        g_nav_fin = 1U; /* 失败恢复空闲态 */
        return APP_ERR;
    }
    /* g_nav_fin 由 csvc 控制任务置 1，文件级变量全程有效 */
    while (g_nav_fin == 0U) {
        osDelay(RT_NAV_POLL_MS);
    }
    return APP_OK;
}

/**
 * @brief  按表指定方式找线并标定位姿
 * @param  pt 任务点表项
 * @retval APP_OK / APP_ERR
 */
static app_status_t pose_fix(const route_pt_t *pt)
{
    float yaw = 0.0f; /* IMU 原始航向，deg */

    if (pt->fix == FIX_NONE) {
        return APP_OK;
    }
    if (align_seek_line() != ALIGN_OK) {
        return APP_ERR;
    }
    if (pt->fix == FIX_LINE) {
        return (align_white_line(pt->fix_pt, pt->yaw_deg) == ALIGN_OK)
               ? APP_OK : APP_ERR;
    }
    if (align_yaw_read(&yaw) != ALIGN_OK) {
        return APP_ERR;
    }
    return (csvc_set_pose(pt->fix_pt, -yaw) == CSVC_OK) ? APP_OK : APP_ERR;
}

/** @copydoc route_go */
app_status_t route_go(route_id_t id)
{
    const route_pt_t *pt; /* 当前任务点表项 */

    if (id >= ROUTE_NUM) {
        return APP_ERR;
    }
    pt = &g_route[id];
    if (nav_wait(pt) != APP_OK) {
        RT_LOGE("route %d nav fail", (int)id);
        return APP_ERR;
    }
    if (pose_fix(pt) != APP_OK) {
        RT_LOGE("route %d fix fail", (int)id);
        return APP_ERR;
    }
    return APP_OK;
}
