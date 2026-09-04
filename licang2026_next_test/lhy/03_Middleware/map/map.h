/**
 * @file    map.h
 * @brief   通用栅格地图 + A* 路径引擎（纯算法，平台无关）
 * @author  haoyu
 * @note    - 仅依赖标准库；栅格尺寸由 map_cfg.h 编译期决定
 *          - 全程地图系 mm，原点在场地角，不含本场地障碍/世界系偏移
 *          - 设障碍会按 MAP_EXPAND_MM（车半径）硬膨胀；场景层见 map_adaption
 */

#ifndef MAP_H
#define MAP_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* 状态码：OK==0，适配层据此逐层映射 */
typedef enum {
    MAP_OK          = 0,    /* 成功 */
    MAP_ERR         = 1,    /* 通用错误 */
    MAP_ERR_PARAM   = 2,    /* 参数非法（空指针 / 坐标越界 / 容量为 0） */
    MAP_ERR_INIT    = 3,    /* 未初始化 */
    MAP_ERR_RES     = 4,    /* 资源不足（open 表满 / 路径缓存不够） */
    MAP_ERR_NO_PATH = 5,    /* 搜索结束仍无可行路径 */
    MAP_ERR_BLOCKED = 6,    /* 起点或终点落在障碍内 */
} map_status_t;

/* 地图系坐标点，单位 mm。
 * 默认本模块内部定义；工程级定义 USE_POINT_TYPES_H 即切换为 99_Utils 共享类型，
 * 本文件无需改动（架构文档 §11）。外部头须提供同名 map_point_t（x_mm/y_mm）。 */
#ifdef USE_POINT_TYPES_H
#include "point_types.h"        /* 全工程共享坐标类型，含 map_point_t */
#else
typedef struct {
    int16_t x_mm;               /* 竖直向上为正，范围 0..MAP_X_MAX_MM */
    int16_t y_mm;               /* 水平向左为正，范围 0..MAP_Y_MAX_MM */
} map_point_t;
#endif

/**
 * @brief  初始化引擎，清空障碍与 A* 工作区
 * @retval MAP_OK
 */
map_status_t map_init(void);

/**
 * @brief  清空所有障碍（保留初始化状态）
 * @retval MAP_OK / MAP_ERR_INIT
 */
map_status_t map_clear(void);

/**
 * @brief  设置单个栅格障碍状态（不膨胀）
 * @param  grid_x 栅格 x 下标
 * @param  grid_y 栅格 y 下标
 * @param  is_obs 0=可通行，非 0=障碍
 * @retval MAP_OK / MAP_ERR_PARAM / MAP_ERR_INIT
 */
map_status_t map_set_obs_grid(uint16_t grid_x, uint16_t grid_y, uint8_t is_obs);

/**
 * @brief  按 mm 坐标设单点障碍（设障碍时按车半径膨胀）
 * @param  x_mm   地图系 x，0..MAP_X_MAX_MM
 * @param  y_mm   地图系 y，0..MAP_Y_MAX_MM
 * @param  is_obs 0=可通行，非 0=障碍
 * @retval MAP_OK / MAP_ERR_PARAM / MAP_ERR_INIT
 */
map_status_t map_set_obs_mm(int16_t x_mm, int16_t y_mm, uint8_t is_obs);

/**
 * @brief  按 mm 坐标设矩形障碍（设障碍时按车半径膨胀）
 * @param  x1_mm  矩形一角 x
 * @param  y1_mm  矩形一角 y
 * @param  x2_mm  矩形对角 x
 * @param  y2_mm  矩形对角 y
 * @param  is_obs 0=可通行，非 0=障碍
 * @retval MAP_OK / MAP_ERR_PARAM / MAP_ERR_INIT
 */
map_status_t map_set_obs_rect(int16_t x1_mm, int16_t y1_mm,
                              int16_t x2_mm, int16_t y2_mm, uint8_t is_obs);

/**
 * @brief  按 mm 坐标设圆形障碍（设障碍时按车半径膨胀）
 * @param  cx_mm  圆心 x
 * @param  cy_mm  圆心 y
 * @param  r_mm   半径，单位 mm
 * @param  is_obs 0=可通行，非 0=障碍
 * @retval MAP_OK / MAP_ERR_PARAM / MAP_ERR_INIT
 */
map_status_t map_set_obs_circle(int16_t cx_mm, int16_t cy_mm,
                                uint16_t r_mm, uint8_t is_obs);

/**
 * @brief  读取单个栅格障碍状态
 * @param  grid_x 栅格 x 下标
 * @param  grid_y 栅格 y 下标
 * @retval 0=可通行；1=障碍或坐标非法/未初始化
 */
uint8_t map_is_obs(uint16_t grid_x, uint16_t grid_y);

/**
 * @brief  软代价 A* 规划 + 代价感知视线剪枝（地图系 mm）
 * @param  sx_mm 起点 x        @param  sy_mm 起点 y
 * @param  gx_mm 终点 x        @param  gy_mm 终点 y
 * @param  path  输出路径缓存  @param  cap   缓存可容点数
 * @param  len   输出路径点数（含所需点数，便于扩容重试）
 * @retval MAP_OK / MAP_ERR_PARAM / MAP_ERR_INIT / MAP_ERR_RES
 *         / MAP_ERR_NO_PATH / MAP_ERR_BLOCKED
 */
map_status_t map_find_path(int16_t sx_mm, int16_t sy_mm,
                           int16_t gx_mm, int16_t gy_mm,
                           map_point_t *path, uint16_t cap, uint16_t *len);

#ifdef __cplusplus
}
#endif

#endif /* MAP_H */
