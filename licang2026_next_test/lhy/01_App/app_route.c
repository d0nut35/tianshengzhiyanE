/**
 * @file    app_route.c
 * @brief   任务点表与统一执行流程：导航 → 找线 → 位姿标定
 * @note    - g_route 集中全部场地坐标与速度（默认图），实机标定只改本文件的表
 *          - 镜像侧由 route_go 现场换算：x→RT_FIELD_W_MM-x，航向→180°-θ
 *          - FIX_NONE 只导航；FIX_IMU 按当前 IMU 航向标定；FIX_LINE 扫白线回中
 *          - IMU 帧与世界帧反号（chassis 侧取负对齐地图系）
 */

#include "app_route.h"

#include <stdint.h>

#include "cmsis_os2.h"

#include "app_util.h"   /* util_ang_norm：镜像航向归一化 */
#include "chassis_align.h"
#include "chassis_service.h"
#include "map.h"        /* map_point_t：导航与标定坐标 */
#include "map_cfg.h"    /* MAP_X_MAX_MM：镜像轴取场地 x 量程一半 */

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
/* 场地 x 量程，与 A* 地图同源；镜像轴为其一半 x=1250，mm */
#define RT_FIELD_W_MM   ((int16_t)MAP_X_MAX_MM)

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

/* 场地坐标待实机标定；表序与 route_id_t 严格一致；均为默认图坐标，
 * 镜像侧由 route_go 关于 x=RT_FIELD_W_MM/2 换算，不另建表 */
static const route_pt_t g_route[ROUTE_NUM] = {
    /* PLATFORM：圆盘工作位，找线后按 IMU 航向标定 */
    { CSVC_NAV_PATH, { 440, 4300}, 180.0f, 500.0f, 30.0f,
      { 473, 4300}, FIX_IMU  },
    /* STAIRS：阶梯起始工作位，即低层起点 */
    { CSVC_NAV_PATH, {2100, 2920},   0.0f, 500.0f, 60.0f,
      {2107, 2950}, FIX_LINE },
    /* [lyx] CYL_PRE：圆柱前置点，先直线到此再平推入位，避免斜穿 */
    { CSVC_NAV_LINE, {1700, 2595}, 180.0f, 500.0f, 60.0f,
      {   0,    0}, FIX_NONE },
    /* [lyx] CYL：圆柱绕圈起点，到点后由 app_main 定半径绕一圈 */
    { CSVC_NAV_LINE, {1475, 2595}, 180.0f, 500.0f, 60.0f,
      {   0,    0}, FIX_NONE },
    /* DEPOT：立体仓库 1 号工作位，找线标定后作为 2~4 号位横移基准 */
    { CSVC_NAV_PATH, { 440, 2286}, 180.0f, 500.0f, 60.0f,
      { 403, 2208}, FIX_LINE },
    /* HOME：起点，直线返回不做找线对齐 */
    { CSVC_NAV_LINE, {1270,  350}, 180.0f, 500.0f, 30.0f,
      {   0,    0}, FIX_NONE },
};

static uint8_t      g_nav_fin = 1U;  /* 导航完成标志，1=空闲 */
static route_side_t g_side = ROUTE_SIDE_DEFAULT; /* 场地侧，仅底盘任务读写 */

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
    route_pt_t pt; /* 当前任务点，已按场地侧换算 */

    if (id >= ROUTE_NUM) {
        return APP_ERR;
    }
    pt = g_route[id];
    pt.nav_pt.x_mm = route_side_x(pt.nav_pt.x_mm);
    pt.fix_pt.x_mm = route_side_x(pt.fix_pt.x_mm);
    if (g_side == ROUTE_SIDE_MIRROR) {
        /* 镜像后左右互换：航向 θ→180°-θ，y 不变 */
        pt.yaw_deg = util_ang_norm(180.0f - pt.yaw_deg);
    }
    if (nav_wait(&pt) != APP_OK) {
        RT_LOGE("route %d nav fail", (int)id);
        return APP_ERR;
    }
    if (pose_fix(&pt) != APP_OK) {
        RT_LOGE("route %d fix fail", (int)id);
        return APP_ERR;
    }
    return APP_OK;
}

/** @copydoc route_set_side */
app_status_t route_set_side(route_side_t side)
{
    if ((side != ROUTE_SIDE_DEFAULT) && (side != ROUTE_SIDE_MIRROR)) {
        return APP_ERR;
    }
    /* 障碍与点表必须同侧：先重载 A* 障碍，成功才切点表侧 */
    if (csvc_load_field((side == ROUTE_SIDE_MIRROR) ? 1U : 0U) != CSVC_OK) {
        RT_LOGE("field load fail");
        return APP_ERR;
    }
    g_side = side;
    return APP_OK;
}

/** @copydoc route_side */
route_side_t route_side(void)
{
    return g_side;
}

/** @copydoc route_side_x */
int16_t route_side_x(int16_t x_mm)
{
    if (g_side != ROUTE_SIDE_MIRROR) {
        return x_mm;
    }
    return (int16_t)(RT_FIELD_W_MM - x_mm);
}

/** @copydoc route_lat_sign */
float route_lat_sign(void)
{
    return (g_side == ROUTE_SIDE_MIRROR) ? -1.0f : 1.0f;
}
