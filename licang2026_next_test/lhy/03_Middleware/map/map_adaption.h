/**
 * @file    map_adaption.h
 * @brief   地图场景适配层（本场地障碍 + 世界系↔地图系偏移）
 * @author  haoyu
 * @note    - 纯计算无 HAL/OS；上层规划的唯一合法入口，负责参数校验
 *          - 入参/出参世界系 mm（带符号）；偏移封装在 to_map/to_world
 *          - 场景常量见 map_cfg.h，换场地只改 cfg 与本层障碍表
 */

#ifndef MAP_ADAPTION_H
#define MAP_ADAPTION_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "map.h"        /* map_status_t / map_point_t */

/**
 * @brief  初始化地图：引擎清空 + 载入本场地障碍
 * @retval MAP_OK / MAP_ERR
 */
map_status_t map_adp_init(void);

/**
 * @brief  载入本场地障碍布局（原 map_load_default_obstacles）
 * @retval MAP_OK / MAP_ERR_INIT / MAP_ERR_PARAM
 */
map_status_t map_adp_load_field(void);

/**
 * @brief  世界系规划：经 to_map 转地图系跑 A*，再 to_world 回转
 * @param  sx_mm 起点 x（世界系）  @param  sy_mm 起点 y
 * @param  gx_mm 终点 x（世界系）  @param  gy_mm 终点 y
 * @param  path  输出世界系路径    @param  cap   缓存可容点数
 * @param  len   输出路径点数
 * @retval MAP_OK / MAP_ERR_PARAM / MAP_ERR_INIT / MAP_ERR_RES
 *         / MAP_ERR_NO_PATH / MAP_ERR_BLOCKED
 */
map_status_t map_adp_find_path(int16_t sx_mm, int16_t sy_mm,
                               int16_t gx_mm, int16_t gy_mm,
                               map_point_t *path, uint16_t cap, uint16_t *len);

/**
 * @brief  世界系点转地图系点（供上层上报/显示用，偏移不外泄）
 * @param  world 世界系坐标点
 * @retval 地图系坐标点
 */
map_point_t map_adp_to_map(map_point_t world);

/**
 * @brief  地图系点转世界系点（find_path 出参回转，亦可上层复用）
 * @param  map 地图系坐标点
 * @retval 世界系坐标点
 */
map_point_t map_adp_to_world(map_point_t map);

#ifdef __cplusplus
}
#endif

#endif /* MAP_ADAPTION_H */
