/**
 * @file    app_route.c
 * @brief   任务点表与统一执行流程：导航 → 找线 → 位姿标定
 * @note    - g_route 为默认图点表，蓝方圆盘、阶梯、起点使用下方独立配置
 *          - 其余镜像侧点按 x→RT_FIELD_W_MM-x、航向→180°-θ 换算
 *          - FIX_NONE 只导航；FIX_IMU 按当前 IMU 航向标定；FIX_LINE 扫白线回中
 *          - IMU 帧与世界帧反号（chassis 侧取负对齐地图系）
 */

#include "app_route.h"
#include "../../01_App/mission/mission_red_backup_config.h"

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
 * 蓝方圆盘、阶梯、起点使用下方独立配置，其余镜像侧点由 route_go 关于 x=RT_FIELD_W_MM/2 换算 */
static const route_pt_t g_route[ROUTE_NUM] = {
    /* PLATFORM：圆盘工作位，找线后按 IMU 航向标定 */
    { CSVC_NAV_PATH, { 525, 4300}, 180.0f, 500.0f, 30.0f,
      { 473, 4300}, FIX_IMU  },
    /* STAIRS：阶梯起始工作位，即低层起点 */
    { CSVC_NAV_PATH, {2100, 2920},   0.0f, 500.0f, 60.0f,
      {2107, 2950}, FIX_LINE },
    /* [lyx] CYL_PRE：圆柱前置点，先直线到此再平推入位，避免斜穿 */
    { CSVC_NAV_LINE, {1550, 2163}, 180.0f, 500.0f, 60.0f,
      {   0,    0}, FIX_NONE },
    /* [lyx] CYL：绕桩预定位点，到点后由 app_main 两段找线，再按APP_CYL_R_MM设定的半径绕行 */
    { CSVC_NAV_LINE, {1550, 2163}, 180.0f, 500.0f, 60.0f,
      {   0,    0}, FIX_NONE },
    /* DEPOT：立体仓库 1 号工作位，找线标定后作为 2~4 号位横移基准 */
    /* [lyx] 当前入口预停点Y为2276 mm；后续找线基准点保持不变。 */
    { CSVC_NAV_PATH, { 440, 2276}, 180.0f, 500.0f, 60.0f,
      { 403, 2208}, FIX_LINE },
    /* HOME：起点，直线返回不做找线对齐 */
    { CSVC_NAV_LINE, {1215,  440}, 180.0f, 500.0f, 30.0f,
      {   0,    0}, FIX_NONE },
};

/* 蓝方 PLATFORM：直接填写蓝方世界系导航点、到点航向与标定点，不再镜像。
 * 初值沿用原镜像结果；找线后仍按当前 IMU 航向标定。 */
static const route_pt_t g_blue_platform = {
    CSVC_NAV_PATH, {2000, 4320},   0.0f, 500.0f, 30.0f,
    {2027, 4300}, FIX_IMU
};

/* 蓝方 STAIRS：直接填写蓝方世界系导航点、航向与标定点，不再镜像。
 * 初值沿用原镜像结果；实机调参只改此项，不影响红方阶梯。 */
static const route_pt_t g_blue_stair = {
    CSVC_NAV_PATH, { 400, 2950}, -180.0f, 500.0f, 60.0f,
    { 393, 2950}, FIX_LINE
};

/* 蓝方 HOME：直接填写蓝方世界系起点与到点航向，不再镜像。
 * 初值沿用原镜像结果；直线返回，不找线、不标定位姿。 */
static const route_pt_t g_blue_home = {
    CSVC_NAV_LINE, {1220,  440},   0.0f, 500.0f, 30.0f,
    {   0,    0}, FIX_NONE
};

static uint8_t      g_nav_fin = 1U;  /* 导航完成标志，1=空闲 */
static route_side_t g_side = ROUTE_SIDE_DEFAULT; /* 场地侧，仅底盘任务读写 */

static app_status_t nav_wait(const route_pt_t *pt);
static app_status_t pose_fix(const route_pt_t *pt, route_id_t id);

/**
 * @brief  发起导航并阻塞等待到点
 * @param  pt 任务点表项
 * @param  id 路线区域，用于选择白线标定补偿
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
static app_status_t pose_fix(const route_pt_t *pt, route_id_t id)
{
    float yaw = 0.0f; /* IMU 原始航向，deg */
    align_site_t site;

    if (pt->fix == FIX_NONE) {
        return APP_OK;
    }
    if (align_seek_line() != ALIGN_OK) {
        return APP_ERR;
    }
    if (pt->fix == FIX_LINE) {
        if (id == ROUTE_STAIRS) {
            site = (g_side == ROUTE_SIDE_MIRROR) ? AL_BLUE_STAIR : AL_RED_STAIR;
        } else if (id == ROUTE_DEPOT) {
            site = (g_side == ROUTE_SIDE_MIRROR) ? AL_BLUE_DEPOT : AL_RED_DEPOT;
        } else {
            return APP_ERR;
        }
        return (align_white_line(pt->fix_pt, pt->yaw_deg, site) == ALIGN_OK)
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
    if ((g_side == ROUTE_SIDE_MIRROR) && (id == ROUTE_PLATFORM)) {
        pt = g_blue_platform;
    } else if ((g_side == ROUTE_SIDE_MIRROR) && (id == ROUTE_STAIRS)) {
        pt = g_blue_stair;
    } else if ((g_side == ROUTE_SIDE_MIRROR) && (id == ROUTE_HOME)) {
        pt = g_blue_home;
    } else {
        pt = g_route[id];
        pt.nav_pt.x_mm = route_side_x(pt.nav_pt.x_mm);
        pt.fix_pt.x_mm = route_side_x(pt.fix_pt.x_mm);
        if (g_side == ROUTE_SIDE_MIRROR) {
            /* 镜像后左右互换：航向 θ→180°-θ，y 不变 */
            pt.yaw_deg = util_ang_norm(180.0f - pt.yaw_deg);
        }
    }
    if (nav_wait(&pt) != APP_OK) {
        RT_LOGE("route %d nav fail", (int)id);
        return APP_ERR;
    }
    if (pose_fix(&pt, id) != APP_OK) {
        RT_LOGE("route %d fix fail", (int)id);
        return APP_ERR;
    }
    return APP_OK;
}

/**
 * @brief 使用独立参数执行红方备用圆盘机、仓库或回家路线。
 * @param id 仅支持PLATFORM、DEPOT、HOME。
 * @param guard 底盘提供的STOP/总超时检查，返回0即中止。
 * @retval APP_OK=导航及需要的校准完成并停车；APP_ERR=中止或失败。
 * @note 单底盘任务串行使用；任何出口都解除校准保护回调，不改正式点表。
 */
app_status_t route_red_backup_go(route_id_t id, uint8_t (*guard)(void))
{
    static const route_pt_t platform = {
        CSVC_NAV_PATH, {RED_BACKUP_PLATFORM_X_MM, RED_BACKUP_PLATFORM_Y_MM},
        RED_BACKUP_PLATFORM_YAW_DEG, RED_BACKUP_PLATFORM_V_MMS, RED_BACKUP_PLATFORM_W_DPS,
        {RED_BACKUP_PLATFORM_FIX_X_MM, RED_BACKUP_PLATFORM_FIX_Y_MM}, FIX_IMU };
    static const route_pt_t depot = {
        CSVC_NAV_PATH, {RED_BACKUP_DEPOT_X_MM, RED_BACKUP_DEPOT_Y_MM},
        RED_BACKUP_DEPOT_YAW_DEG, RED_BACKUP_DEPOT_V_MMS, RED_BACKUP_DEPOT_W_DPS,
        {RED_BACKUP_DEPOT_FIX_X_MM, RED_BACKUP_DEPOT_FIX_Y_MM}, FIX_LINE };
    static const route_pt_t home = {
        CSVC_NAV_LINE, {RED_BACKUP_HOME_X_MM, RED_BACKUP_HOME_Y_MM},
        RED_BACKUP_HOME_YAW_DEG, RED_BACKUP_HOME_V_MMS, RED_BACKUP_HOME_W_DPS,
        {0, 0}, FIX_NONE };
    const route_pt_t *pt;
    float yaw;
    app_status_t result = APP_ERR;
    if (g_side != ROUTE_SIDE_DEFAULT || guard == NULL) return APP_ERR;
    if (id == ROUTE_PLATFORM) pt = &platform;
    else if (id == ROUTE_DEPOT) pt = &depot;
    else if (id == ROUTE_HOME) pt = &home;
    else return APP_ERR;
    align_set_guard(guard);
    if (!guard()) goto done;
    g_nav_fin = 0U;
    if (csvc_nav(pt->nav_mode, pt->nav_pt, pt->yaw_deg, pt->v_mms,
                 pt->w_dps, &g_nav_fin) != CSVC_OK) goto done;
    while (g_nav_fin == 0U) {
        if (!guard()) goto done;
        osDelay(util_ms_ticks(RT_NAV_POLL_MS));
    }
    /* STOP也可能把导航标记结束，必须先核对锁存再进入校准。 */
    if (!guard()) goto done;
    if (pt->fix != FIX_NONE) {
        if (align_seek_line() != ALIGN_OK || !guard()) goto done;
        if (pt->fix == FIX_LINE) {
            if (align_white_line(pt->fix_pt, pt->yaw_deg, AL_RED_BACKUP_DEPOT) != ALIGN_OK) goto done;
        } else {
            if (align_yaw_read(&yaw) != ALIGN_OK || !guard() ||
                csvc_set_pose(pt->fix_pt, -yaw) != CSVC_OK) goto done;
        }
    }
    if (guard()) result = APP_OK;
done:
    if (align_stop() != ALIGN_OK) result = APP_ERR;
    if (!guard()) result = APP_ERR;
    align_set_guard(NULL);
    return result;
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
