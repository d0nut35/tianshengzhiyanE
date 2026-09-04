/**
 * @file    map_cfg.h
 * @brief   地图场景/机器人编译期常量（换场地、换车只改这一处）
 * @author  haoyu
 * @note    - 分辨率与场地尺寸决定静态数组大小，必须是编译期常量
 *          - 被 map.c（栅格尺寸）与 map_adaption.c（偏移/调参）共同 include
 *          - 坐标系：地图系原点在场地角，x 竖直向上为正、y 水平向左为正
 */

#ifndef MAP_CFG_H
#define MAP_CFG_H

#ifdef __cplusplus
extern "C" {
#endif

/* ---- 场地 / 机器人尺寸（决定栅格规模，编译期常量） ---- */
#define MAP_RES_MM          50U      /* 栅格分辨率，单位 mm */
#define MAP_X_MAX_MM        2500U    /* 地图系 x 量程，单位 mm */
#define MAP_Y_MAX_MM        5000U    /* 地图系 y 量程，单位 mm */
#define MAP_EXPAND_MM       150U     /* 障碍硬膨胀，取小车半径，单位 mm */

/* ---- 由上面派生的栅格规模（静态数组按此定尺寸） ---- */
#define MAP_GRID_W          ((MAP_X_MAX_MM / MAP_RES_MM) + 1U)  /* x 方向栅格数 */
#define MAP_GRID_H          ((MAP_Y_MAX_MM / MAP_RES_MM) + 1U)  /* y 方向栅格数 */
#define MAP_NODE_COUNT      (MAP_GRID_W * MAP_GRID_H)           /* 栅格总数 */

/* ---- A* 算法调参（贴边代价 / 视线剪枝 / 启发缓存） ---- */
#define MAP_SOFT_R_MM       250U     /* 软代价影响半径，单位 mm */
#define MAP_SOFT_COST_MAX   120U     /* 贴近障碍时叠加的最大软代价 */
#define MAP_SHORTCUT_RATIO  1100U    /* 剪枝阈值‰：直线代价≤原路径 1.1 倍则取直 */
#define MAP_USE_H_CACHE     1U       /* 1=缓存启发值（费 RAM 提速），0=每次现算 */

/* ---- 世界系→地图系原点偏移（原 CHASSIS_PATH_TARGET_OFFSET_X/Y） ---- */
#define MAP_ORIGIN_X_MM     0     /* 世界系原点落在地图系的 x，单位 mm */
#define MAP_ORIGIN_Y_MM     0      /* 世界系原点落在地图系的 y，单位 mm */

#ifdef __cplusplus
}
#endif

#endif /* MAP_CFG_H */
