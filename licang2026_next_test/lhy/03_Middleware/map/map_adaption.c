/**
 * @file    map_adaption.c
 * @brief   地图场景适配层（本场地障碍 + 世界系↔地图系偏移）
 * @author  haoyu
 * @note    - 纯计算无 HAL/OS；偏移集中在 to_map/to_world
 *          - find_path 仅组合：转地图系 → 跑 A* → 出参回转世界系
 *          - 障碍表为本届赛场布局，换场地改本表与 map_cfg.h
 */

#include "map_adaption.h"
#include "map_cfg.h"        /* MAP_ORIGIN_X/Y_MM */
#include <stddef.h>         /* NULL */

/* 本场地障碍形状 */
typedef enum {
    MAP_OBS_RECT = 0U,      /* 矩形：用 x1/y1/x2/y2 */
    MAP_OBS_CIRCLE,         /* 圆形：用 x1/y1 圆心 + r */
} map_obs_type_t;

/* 本场地一处障碍（地图系 mm） */
typedef struct {
    map_obs_type_t type;    /* 形状 */
    int16_t  x1_mm;         /* 矩形一角 x / 圆心 x */
    int16_t  y1_mm;         /* 矩形一角 y / 圆心 y */
    int16_t  x2_mm;         /* 矩形对角 x（圆形不用） */
    int16_t  y2_mm;         /* 矩形对角 y（圆形不用） */
    uint16_t r_mm;          /* 圆半径（矩形不用） */
} map_obs_t;

/* 本届赛场障碍布局，换场地只改这张表（坐标按图纸近似，实测后修正） */
static const map_obs_t g_field_obs[] = {
    { MAP_OBS_CIRCLE, 1200, 2500,    0,    0, 200U },  /* 中间圆形 */
    { MAP_OBS_RECT,      0, 2100,  100, 2900,   0U },  /* 立体仓库，约 440x440 */
    { MAP_OBS_CIRCLE,    0, 4300,    0,    0, 250U },  /* 圆台 */
    { MAP_OBS_CIRCLE, 1200, 4300,    0,    0, 100U },  /* 圆柱障碍 */
    { MAP_OBS_RECT,   2400, 2100, 2500, 2900,   0U },  /* 立体仓库 2 */
};

/**
 * @brief  初始化地图：引擎清空 + 载入本场地障碍
 * @retval MAP_OK / MAP_ERR
 */
map_status_t map_adp_init(void)
{
    map_status_t ret = MAP_ERR;      /* 子步骤状态 */

    ret = map_init();
    if (MAP_OK != ret) {
        return ret;
    }

    return map_adp_load_field();
}

/**
 * @brief  载入本场地障碍布局（设障碍时引擎按车半径膨胀）
 * @retval MAP_OK / MAP_ERR_INIT / MAP_ERR_PARAM
 */
map_status_t map_adp_load_field(void)
{
    uint16_t i = 0U;                /* 障碍表下标 */
    uint16_t n = 0U;                /* 障碍表元素数 */
    map_status_t ret = MAP_ERR;      /* 单处障碍写入状态 */

    ret = map_clear();               /* 顺带守初始化（未初始化返回 INIT） */
    if (MAP_OK != ret) {
        return ret;
    }

    n = (uint16_t)(sizeof(g_field_obs) / sizeof(g_field_obs[0]));
    for (i = 0U; i < n; i++) {
        if (MAP_OBS_RECT == g_field_obs[i].type) {
            ret = map_set_obs_rect(g_field_obs[i].x1_mm, g_field_obs[i].y1_mm,
                                  g_field_obs[i].x2_mm, g_field_obs[i].y2_mm, 1U);
        } else {
            ret = map_set_obs_circle(g_field_obs[i].x1_mm, g_field_obs[i].y1_mm,
                                    g_field_obs[i].r_mm, 1U);
        }

        if (MAP_OK != ret) {
            return ret;
        }
    }

    return MAP_OK;
}

/**
 * @brief  世界系点转地图系点（合法世界系内不溢出，越界由引擎拦截）
 * @param  world 世界系坐标点
 * @retval 地图系坐标点
 */
map_point_t map_adp_to_map(map_point_t world)
{
    map_point_t out;                /* 地图系结果 */

    out.x_mm = (int16_t)(world.x_mm + MAP_ORIGIN_X_MM);
    out.y_mm = (int16_t)(world.y_mm + MAP_ORIGIN_Y_MM);

    return out;
}

/**
 * @brief  地图系点转世界系点
 * @param  map 地图系坐标点
 * @retval 世界系坐标点
 */
map_point_t map_adp_to_world(map_point_t map)
{
    map_point_t out;                /* 世界系结果 */

    out.x_mm = (int16_t)(map.x_mm - MAP_ORIGIN_X_MM);
    out.y_mm = (int16_t)(map.y_mm - MAP_ORIGIN_Y_MM);

    return out;
}

/**
 * @brief  世界系规划：经 to_map 转地图系跑 A*，再 to_world 回转
 * @param  sx_mm 起点 x  @param  sy_mm 起点 y
 * @param  gx_mm 终点 x  @param  gy_mm 终点 y
 * @param  path  输出世界系路径  @param  cap 缓存点数  @param  len 输出点数
 * @retval MAP_OK / MAP_ERR_PARAM / MAP_ERR_INIT / MAP_ERR_RES
 *         / MAP_ERR_NO_PATH / MAP_ERR_BLOCKED
 */
map_status_t map_adp_find_path(int16_t sx_mm, int16_t sy_mm,
                               int16_t gx_mm, int16_t gy_mm,
                               map_point_t *path, uint16_t cap, uint16_t *len)
{
    map_point_t s_w = { sx_mm, sy_mm };   /* 起点世界系 */
    map_point_t g_w = { gx_mm, gy_mm };   /* 终点世界系 */
    map_point_t s_m = { 0, 0 };           /* 起点地图系 */
    map_point_t g_m = { 0, 0 };           /* 终点地图系 */
    uint16_t i = 0U;                      /* 路径点下标 */
    map_status_t ret = MAP_ERR;            /* A* 状态 */

    if ((NULL == path) || (NULL == len) || (0U == cap)) {
        return MAP_ERR_PARAM;
    }

    s_m = map_adp_to_map(s_w);            /* 世界系→地图系，范围由引擎校验 */
    g_m = map_adp_to_map(g_w);
    // s_m = s_w;                               /* 世界系与地图系同系，直接用 */
    // g_m = g_w;

    ret = map_find_path(s_m.x_mm, s_m.y_mm, g_m.x_mm, g_m.y_mm, path, cap, len);
    if (MAP_OK != ret) {
        return ret;
    }

    for (i = 0U; i < *len; i++) {         /* 出参逐点回转世界系，供跟随器直接用 */
        path[i] = map_adp_to_world(path[i]);
    }

    return MAP_OK;
}
