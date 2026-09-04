/**
 * @file    map.c
 * @brief   通用栅格地图 + A* 路径引擎（纯算法，平台无关）
 * @author  haoyu
 * @note    - 仅依赖标准库与 map_cfg.h（编译期定栅格尺寸）
 *          - 全程地图系 mm，原点在场地角；设障碍按车半径硬膨胀
 *          - 本场地障碍/世界系偏移在 map_adaption，不在本层
 */

#include "map.h"
#include "map_cfg.h"

#include <stddef.h>
#include <stdint.h>

#define MAP_OBSTACLE_FREE        ((uint8_t)0U)         /* 栅格可通行 */
#define MAP_OBSTACLE_BLOCKED     ((uint8_t)1U)         /* 栅格被占 */
#define MAP_ASTAR_STATE_NONE     ((uint8_t)0U)         /* 未访问 */
#define MAP_ASTAR_STATE_OPEN     ((uint8_t)1U)         /* 在 open 表 */
#define MAP_ASTAR_STATE_CLOSED   ((uint8_t)2U)         /* 已扩展 */
#define MAP_ASTAR_PARENT_NONE    ((uint8_t)0xFFU)      /* 无父方向 */
#define MAP_ASTAR_STRAIGHT_COST  ((uint16_t)10U)       /* 直行单步代价 */
#define MAP_ASTAR_DIAGONAL_COST  ((uint16_t)14U)       /* 斜行单步代价≈10√2 */
#define MAP_GRID_MAX_X           ((uint16_t)(MAP_GRID_W - 1U))  /* x 最大下标 */
#define MAP_GRID_MAX_Y           ((uint16_t)(MAP_GRID_H - 1U))  /* y 最大下标 */
#define MAP_INDEX_INVALID        ((uint16_t)0xFFFFU)   /* 非法线性编号 */
#define MAP_ASTAR_COST_PER_MILLE ((uint32_t)1000U)     /* 剪枝比例基准‰ */

static uint8_t  g_mapIsInitialized = 0U;                       /* 初始化标志 */
static uint8_t  g_obstacleMap[MAP_GRID_H][MAP_GRID_W] = { 0U };    /* 障碍栅格 */
static uint8_t  g_astarState[MAP_GRID_H][MAP_GRID_W] = { 0U };     /* 节点状态 */
static uint16_t g_astarGCost[MAP_GRID_H][MAP_GRID_W] = { 0U };     /* 起点累计代价 */
static uint8_t  g_astarParentDir[MAP_GRID_H][MAP_GRID_W] = { 0U }; /* 父节点方向 */
static uint16_t g_astarOpenList[MAP_NODE_COUNT] = { 0U };          /* open 表/路径缓存 */
#if (MAP_USE_H_CACHE != 0U)
static uint16_t g_astarHCost[MAP_GRID_H][MAP_GRID_W] = { 0U };     /* 启发值缓存 */
#endif

/* 8 邻方向 x 偏移（与 g_astarDirCost 对齐） */
static const int8_t g_astarDirOffsetX[8] =
{
    0, 1, 1, 1, 0, -1, -1, -1
};

/* 8 邻方向 y 偏移 */
static const int8_t g_astarDirOffsetY[8] =
{
    1, 1, 0, -1, -1, -1, 0, 1
};

/* 8 邻单步代价：直行/斜行交替 */
static const uint16_t g_astarDirCost[8] =
{
    MAP_ASTAR_STRAIGHT_COST,
    MAP_ASTAR_DIAGONAL_COST,
    MAP_ASTAR_STRAIGHT_COST,
    MAP_ASTAR_DIAGONAL_COST,
    MAP_ASTAR_STRAIGHT_COST,
    MAP_ASTAR_DIAGONAL_COST,
    MAP_ASTAR_STRAIGHT_COST,
    MAP_ASTAR_DIAGONAL_COST
};

static void map_clear_buf(void);
static void map_astar_reset(void);
static uint8_t map_grid_is_valid(uint16_t grid_x, uint16_t grid_y);
static uint8_t map_mm_is_valid(int16_t x_mm, int16_t y_mm);
static map_status_t map_mm_to_grid(int16_t x_mm,
                                   int16_t y_mm,
                                   uint16_t *p_grid_x,
                                   uint16_t *p_grid_y);
static uint16_t map_grid_to_index(uint16_t grid_x, uint16_t grid_y);
static void map_index_to_grid(uint16_t index, uint16_t *p_grid_x, uint16_t *p_grid_y);
static uint8_t map_norm_obs(uint8_t is_obstacle);
static int32_t map_min_i32(int32_t value_a, int32_t value_b);
static int32_t map_max_i32(int32_t value_a, int32_t value_b);
static int32_t map_clip_i32(int32_t value, int32_t min_value, int32_t max_value);
static uint16_t map_floor_mm_to_grid(int32_t value_mm);
static uint16_t map_ceil_mm_to_grid(int32_t value_mm);
static map_status_t map_fill_rect(int32_t x1_mm,
                                  int32_t y1_mm,
                                  int32_t x2_mm,
                                  int32_t y2_mm,
                                  uint8_t is_obstacle);
static map_status_t map_fill_circle(int32_t center_x_mm,
                                    int32_t center_y_mm,
                                    uint16_t radius_mm,
                                    uint8_t is_obstacle);
static uint16_t map_abs_diff_u16(uint16_t value_a, uint16_t value_b);
static uint16_t map_astar_heuristic(uint16_t grid_x,
                                    uint16_t grid_y,
                                    uint16_t goal_x,
                                    uint16_t goal_y);
static uint8_t map_astar_can_step(uint16_t current_x,
                                  uint16_t current_y,
                                  int8_t step_x,
                                  int8_t step_y);
static uint16_t map_astar_pop_best(uint16_t goal_x,
                                   uint16_t goal_y,
                                   uint16_t *p_open_count);
static map_status_t map_astar_push_open(uint16_t grid_x,
                                        uint16_t grid_y,
                                        uint16_t *p_open_count);
static map_status_t map_astar_count_path(uint16_t start_x,
                                         uint16_t start_y,
                                         uint16_t goal_x,
                                         uint16_t goal_y,
                                         uint16_t *p_required_len);
static map_status_t map_astar_parent(uint16_t *p_grid_x, uint16_t *p_grid_y);
static uint16_t map_astar_soft(uint16_t grid_x, uint16_t grid_y);
static map_status_t map_astar_wr_raw(uint16_t start_x,
                                     uint16_t start_y,
                                     uint16_t goal_x,
                                     uint16_t goal_y,
                                     uint16_t path_len);
static map_status_t map_astar_line_cost(uint16_t start_x,
                                        uint16_t start_y,
                                        uint16_t end_x,
                                        uint16_t end_y,
                                        uint32_t *p_line_cost);
static uint32_t map_astar_seg_cost(uint16_t raw_start_index, uint16_t raw_end_index);
static uint8_t map_astar_can_cut(uint16_t raw_start_index, uint16_t raw_end_index);
static uint16_t map_astar_find_cut(uint16_t raw_anchor_index, uint16_t raw_path_len);
static map_status_t map_astar_cnt_prune(uint16_t raw_path_len, uint16_t *p_required_len);
static map_status_t map_astar_wr_prune(uint16_t raw_path_len,
                                       map_point_t *p_path,
                                       uint16_t path_len);

/**
 * @brief  清空障碍缓存（不检查初始化标志）
 */
static void map_clear_buf(void)
{
    uint16_t gridY = 0U; /* 当前清空的栅格 y 下标。 */
    uint16_t gridX = 0U; /* 当前清空的栅格 x 下标。 */

    for (gridY = 0U; gridY < MAP_GRID_H; gridY++)
    {
        for (gridX = 0U; gridX < MAP_GRID_W; gridX++)
        {
            g_obstacleMap[gridY][gridX] = MAP_OBSTACLE_FREE;
        }
    }
}

/**
 * @brief  重置 A* 搜索运行时工作区
 */
static void map_astar_reset(void)
{
    uint16_t gridY = 0U; /* 当前重置的栅格 y 下标。 */
    uint16_t gridX = 0U; /* 当前重置的栅格 x 下标。 */

    for (gridY = 0U; gridY < MAP_GRID_H; gridY++)
    {
        for (gridX = 0U; gridX < MAP_GRID_W; gridX++)
        {
            g_astarState[gridY][gridX] = MAP_ASTAR_STATE_NONE;
            g_astarGCost[gridY][gridX] = UINT16_MAX;
            g_astarParentDir[gridY][gridX] = MAP_ASTAR_PARENT_NONE;
        }
    }
}

/**
 * @brief  检查栅格坐标范围
 * @retval 1=合法 0=越界
 */
static uint8_t map_grid_is_valid(uint16_t grid_x, uint16_t grid_y)
{
    if ((grid_x >= MAP_GRID_W) || (grid_y >= MAP_GRID_H))
    {
        return 0U;
    }

    return 1U;
}

/**
 * @brief  检查毫米坐标范围
 * @retval 1=合法 0=越界
 */
static uint8_t map_mm_is_valid(int16_t x_mm, int16_t y_mm)
{
    if ((x_mm < 0) || (y_mm < 0))
    {
        return 0U;
    }

    if (((uint16_t)x_mm > MAP_X_MAX_MM) || ((uint16_t)y_mm > MAP_Y_MAX_MM))
    {
        return 0U;
    }

    return 1U;
}

/**
 * @brief  毫米坐标转最近栅格坐标
 * @param  x_mm/y_mm   毫米坐标
 * @param  p_grid_x/y  输出栅格下标
 * @retval MAP_OK / MAP_ERR_PARAM
 */
static map_status_t map_mm_to_grid(int16_t x_mm,
                                   int16_t y_mm,
                                   uint16_t *p_grid_x,
                                   uint16_t *p_grid_y)
{
    uint32_t roundedX = 0U; /* 四舍五入后的 x 坐标中间值。 */
    uint32_t roundedY = 0U; /* 四舍五入后的 y 坐标中间值。 */
//1.参数检查
    if ((NULL == p_grid_x) || (NULL == p_grid_y))
    {
        return MAP_ERR_PARAM;
    }

    if (0U == map_mm_is_valid(x_mm, y_mm))
    {
        return MAP_ERR_PARAM;
    }
//2.真正转换逻辑，并设有最大值
    roundedX = (uint32_t)x_mm + (MAP_RES_MM / 2U);
    roundedY = (uint32_t)y_mm + (MAP_RES_MM / 2U);
    *p_grid_x = (uint16_t)(roundedX / MAP_RES_MM);
    *p_grid_y = (uint16_t)(roundedY / MAP_RES_MM);

    if (*p_grid_x > MAP_GRID_MAX_X)
    {
        *p_grid_x = MAP_GRID_MAX_X;
    }

    if (*p_grid_y > MAP_GRID_MAX_Y)
    {
        *p_grid_y = MAP_GRID_MAX_Y;
    }

    return MAP_OK;
}

/**
 * @brief  栅格坐标转线性编号
 */
static uint16_t map_grid_to_index(uint16_t grid_x, uint16_t grid_y)
{
    uint32_t index = 0U; /* 线性编号计算中间值。 */

    index = ((uint32_t)grid_y * MAP_GRID_W) + grid_x;

    return (uint16_t)index;
}

/**
 * @brief  线性编号转栅格坐标
 */
static void map_index_to_grid(uint16_t index, uint16_t *p_grid_x, uint16_t *p_grid_y)
{
    uint16_t gridX = 0U; /* 由线性编号换算出的 x 下标。 */
    uint16_t gridY = 0U; /* 由线性编号换算出的 y 下标。 */

    gridX = (uint16_t)(index % MAP_GRID_W);
    gridY = (uint16_t)(index / MAP_GRID_W);

    if (NULL != p_grid_x)
    {
        *p_grid_x = gridX;
    }

    if (NULL != p_grid_y)
    {
        *p_grid_y = gridY;
    }
}

/**
 * @brief  障碍标志归一化为 0 或 1
 */
static uint8_t map_norm_obs(uint8_t is_obstacle)
{
    if (0U == is_obstacle)
    {
        return MAP_OBSTACLE_FREE;
    }

    return MAP_OBSTACLE_BLOCKED;
}

/**
 * @brief  取较小的 int32 值
 */
static int32_t map_min_i32(int32_t value_a, int32_t value_b)
{
    if (value_a < value_b)
    {
        return value_a;
    }

    return value_b;
}

/**
 * @brief  取较大的 int32 值
 */
static int32_t map_max_i32(int32_t value_a, int32_t value_b)
{
    if (value_a > value_b)
    {
        return value_a;
    }

    return value_b;
}

/**
 * @brief  将 int32 值裁剪到 [min,max]
 */
static int32_t map_clip_i32(int32_t value, int32_t min_value, int32_t max_value)
{
    if (value < min_value)
    {
        return min_value;
    }

    if (value > max_value)
    {
        return max_value;
    }

    return value;
}

/**
 * @brief  毫米下边界转栅格向下取整下标
 */
static uint16_t map_floor_mm_to_grid(int32_t value_mm)
{
    int32_t clippedValue = 0; /* 裁剪后的毫米坐标。 */

    clippedValue = map_clip_i32(value_mm, 0, (int32_t)MAP_X_MAX_MM);

    return (uint16_t)(clippedValue / (int32_t)MAP_RES_MM);
}

/**
 * @brief  毫米上边界转栅格向上取整下标
 */
static uint16_t map_ceil_mm_to_grid(int32_t value_mm)
{
    int32_t clippedValue = 0; /* 裁剪后的毫米坐标。 */
    int32_t roundedValue = 0; /* 向上取整前的毫米中间值。 */

    clippedValue = map_clip_i32(value_mm, 0, (int32_t)MAP_X_MAX_MM);
    roundedValue = clippedValue + (int32_t)MAP_RES_MM - 1;

    return (uint16_t)(roundedValue / (int32_t)MAP_RES_MM);
}

/**
 * @brief  设矩形障碍：按车半径膨胀并裁剪后写入
 * @param  x1_mm/y1_mm/x2_mm/y2_mm 矩形对角坐标
 * @param  is_obstacle 0=清除 非0=障碍
 * @retval MAP_OK / MAP_ERR_PARAM
 */
static map_status_t map_fill_rect(int32_t x1_mm,
                                  int32_t y1_mm,
                                  int32_t x2_mm,
                                  int32_t y2_mm,
                                  uint8_t is_obstacle)
{
    int32_t minX = 0; /* 矩形较小 x 坐标。 */
    int32_t maxX = 0; /* 矩形较大 x 坐标。 */
    int32_t minY = 0; /* 矩形较小 y 坐标。 */
    int32_t maxY = 0; /* 矩形较大 y 坐标。 */
    uint16_t startGridX = 0U; /* 矩形覆盖的起始栅格 x 下标。 */
    uint16_t endGridX = 0U; /* 矩形覆盖的结束栅格 x 下标。 */
    uint16_t startGridY = 0U; /* 矩形覆盖的起始栅格 y 下标。 */
    uint16_t endGridY = 0U; /* 矩形覆盖的结束栅格 y 下标。 */
    uint16_t gridX = 0U; /* 当前写入的栅格 x 下标。 */
    uint16_t gridY = 0U; /* 当前写入的栅格 y 下标。 */
    uint8_t obstacleValue = 0U; /* 归一化后的障碍标志。 */
    uint16_t expandMm = 0U; /* 障碍硬膨胀距离，单位 mm。 */

    obstacleValue = map_norm_obs(is_obstacle);
    if (MAP_OBSTACLE_BLOCKED == obstacleValue)
    {
        expandMm = (uint16_t)MAP_EXPAND_MM;
    }

    minX = map_min_i32(x1_mm, x2_mm);
    maxX = map_max_i32(x1_mm, x2_mm);
    minY = map_min_i32(y1_mm, y2_mm);
    maxY = map_max_i32(y1_mm, y2_mm);

    minX -= (int32_t)expandMm;
    maxX += (int32_t)expandMm;
    minY -= (int32_t)expandMm;
    maxY += (int32_t)expandMm;

    if ((maxX < 0) || (maxY < 0) ||
        (minX > (int32_t)MAP_X_MAX_MM) || (minY > (int32_t)MAP_Y_MAX_MM))
    {
        return MAP_ERR_PARAM;
    }

    minX = map_clip_i32(minX, 0, (int32_t)MAP_X_MAX_MM);
    maxX = map_clip_i32(maxX, 0, (int32_t)MAP_X_MAX_MM);
    minY = map_clip_i32(minY, 0, (int32_t)MAP_Y_MAX_MM);
    maxY = map_clip_i32(maxY, 0, (int32_t)MAP_Y_MAX_MM);

    startGridX = map_floor_mm_to_grid(minX);
    endGridX = map_ceil_mm_to_grid(maxX);
    startGridY = (uint16_t)(minY / (int32_t)MAP_RES_MM);
    endGridY = (uint16_t)((maxY + (int32_t)MAP_RES_MM - 1) / (int32_t)MAP_RES_MM);

    if (endGridX > MAP_GRID_MAX_X)
    {
        endGridX = MAP_GRID_MAX_X;
    }

    if (endGridY > MAP_GRID_MAX_Y)
    {
        endGridY = MAP_GRID_MAX_Y;
    }

    for (gridY = startGridY; gridY <= endGridY; gridY++)
    {
        for (gridX = startGridX; gridX <= endGridX; gridX++)
        {
            g_obstacleMap[gridY][gridX] = obstacleValue;
        }
    }

    return MAP_OK;
}

/**
 * @brief  设圆形障碍：按车半径膨胀并裁剪后写入
 * @param  center_x_mm/center_y_mm 圆心坐标
 * @param  radius_mm   圆半径
 * @param  is_obstacle 0=清除 非0=障碍
 * @retval MAP_OK / MAP_ERR_PARAM
 */
static map_status_t map_fill_circle(int32_t center_x_mm,
                                    int32_t center_y_mm,
                                    uint16_t radius_mm,
                                    uint8_t is_obstacle)
{
    int32_t minX = 0; /* 圆形包围盒较小 x 坐标。 */
    int32_t maxX = 0; /* 圆形包围盒较大 x 坐标。 */
    int32_t minY = 0; /* 圆形包围盒较小 y 坐标。 */
    int32_t maxY = 0; /* 圆形包围盒较大 y 坐标。 */
    int32_t gridPointX = 0; /* 当前栅格点 x 毫米坐标。 */
    int32_t gridPointY = 0; /* 当前栅格点 y 毫米坐标。 */
    int32_t diffX = 0; /* 当前栅格点相对圆心的 x 偏差。 */
    int32_t diffY = 0; /* 当前栅格点相对圆心的 y 偏差。 */
    int32_t distanceSq = 0; /* 当前栅格点到圆心距离平方。 */
    int32_t radiusSq = 0; /* 圆半径平方。 */
    int32_t expandedRadius = 0; /* 膨胀后的圆半径，单位 mm。 */
    uint16_t startGridX = 0U; /* 圆形包围盒起始栅格 x 下标。 */
    uint16_t endGridX = 0U; /* 圆形包围盒结束栅格 x 下标。 */
    uint16_t startGridY = 0U; /* 圆形包围盒起始栅格 y 下标。 */
    uint16_t endGridY = 0U; /* 圆形包围盒结束栅格 y 下标。 */
    uint16_t gridX = 0U; /* 当前写入的栅格 x 下标。 */
    uint16_t gridY = 0U; /* 当前写入的栅格 y 下标。 */
    uint8_t obstacleValue = 0U; /* 归一化后的障碍标志。 */

    obstacleValue = map_norm_obs(is_obstacle);
    expandedRadius = (int32_t)radius_mm;
    if (MAP_OBSTACLE_BLOCKED == obstacleValue)
    {
        expandedRadius += (int32_t)MAP_EXPAND_MM;
    }

    minX = center_x_mm - expandedRadius;
    maxX = center_x_mm + expandedRadius;
    minY = center_y_mm - expandedRadius;
    maxY = center_y_mm + expandedRadius;

    if ((maxX < 0) || (maxY < 0) ||
        (minX > (int32_t)MAP_X_MAX_MM) || (minY > (int32_t)MAP_Y_MAX_MM))
    {
        return MAP_ERR_PARAM;
    }

    minX = map_clip_i32(minX, 0, (int32_t)MAP_X_MAX_MM);
    maxX = map_clip_i32(maxX, 0, (int32_t)MAP_X_MAX_MM);
    minY = map_clip_i32(minY, 0, (int32_t)MAP_Y_MAX_MM);
    maxY = map_clip_i32(maxY, 0, (int32_t)MAP_Y_MAX_MM);

    startGridX = (uint16_t)(minX / (int32_t)MAP_RES_MM);
    endGridX = (uint16_t)((maxX + (int32_t)MAP_RES_MM - 1) / (int32_t)MAP_RES_MM);
    startGridY = (uint16_t)(minY / (int32_t)MAP_RES_MM);
    endGridY = (uint16_t)((maxY + (int32_t)MAP_RES_MM - 1) / (int32_t)MAP_RES_MM);

    if (endGridX > MAP_GRID_MAX_X)
    {
        endGridX = MAP_GRID_MAX_X;
    }

    if (endGridY > MAP_GRID_MAX_Y)
    {
        endGridY = MAP_GRID_MAX_Y;
    }

    radiusSq = expandedRadius * expandedRadius;
    for (gridY = startGridY; gridY <= endGridY; gridY++)
    {
        gridPointY = (int32_t)gridY * (int32_t)MAP_RES_MM;
        for (gridX = startGridX; gridX <= endGridX; gridX++)
        {
            gridPointX = (int32_t)gridX * (int32_t)MAP_RES_MM;
            diffX = gridPointX - center_x_mm;
            diffY = gridPointY - center_y_mm;
            distanceSq = (diffX * diffX) + (diffY * diffY);
            if (distanceSq <= radiusSq)
            {
                g_obstacleMap[gridY][gridX] = obstacleValue;
            }
        }
    }

    return MAP_OK;
}

/**
 * @brief  两个 uint16 值的绝对差
 */
static uint16_t map_abs_diff_u16(uint16_t value_a, uint16_t value_b)
{
    if (value_a >= value_b)
    {
        return (uint16_t)(value_a - value_b);
    }

    return (uint16_t)(value_b - value_a);
}

/**
 * @brief  八方向 octile 启发式距离
 */
static uint16_t map_astar_heuristic(uint16_t grid_x,
                                    uint16_t grid_y,
                                    uint16_t goal_x,
                                    uint16_t goal_y)
{
    uint16_t diffX = 0U; /* 当前点与目标点的 x 栅格差。 */
    uint16_t diffY = 0U; /* 当前点与目标点的 y 栅格差。 */
    uint16_t minDiff = 0U; /* x/y 差值中的较小值。 */
    uint16_t maxDiff = 0U; /* x/y 差值中的较大值。 */
    uint32_t cost = 0U; /* 启发式代价计算中间值。 */

    diffX = map_abs_diff_u16(grid_x, goal_x);
    diffY = map_abs_diff_u16(grid_y, goal_y);
    if (diffX < diffY)
    {
        minDiff = diffX;
        maxDiff = diffY;
    }
    else
    {
        minDiff = diffY;
        maxDiff = diffX;
    }

    cost = ((uint32_t)MAP_ASTAR_DIAGONAL_COST * minDiff) +
           ((uint32_t)MAP_ASTAR_STRAIGHT_COST * (maxDiff - minDiff));


    return (uint16_t)cost;

}

/**
 * @brief  检查 A* 单步移动是否允许（含斜向防穿角）
 * @retval 1=允许 0=阻挡
 */
static uint8_t map_astar_can_step(uint16_t current_x,
                                  uint16_t current_y,
                                  int8_t step_x,
                                  int8_t step_y)
{
    int32_t nextX = 0; /* 下一步栅格 x 下标中间值。 */
    int32_t nextY = 0; /* 下一步栅格 y 下标中间值。 */
    uint16_t nextGridX = 0U; /* 下一步栅格 x 下标。 */
    uint16_t nextGridY = 0U; /* 下一步栅格 y 下标。 */
    uint16_t sideGridX = 0U; /* 斜向防穿角检查的侧向 x 下标。 */
    uint16_t sideGridY = 0U; /* 斜向防穿角检查的侧向 y 下标。 */

    nextX = (int32_t)current_x + step_x;
    nextY = (int32_t)current_y + step_y;
    if ((nextX < 0) || (nextY < 0) ||
        (nextX >= (int32_t)MAP_GRID_W) || (nextY >= (int32_t)MAP_GRID_H))
    {
        return 0U;
    }

    nextGridX = (uint16_t)nextX;
    nextGridY = (uint16_t)nextY;
    if (MAP_OBSTACLE_BLOCKED == g_obstacleMap[nextGridY][nextGridX])
    {
        return 0U;
    }

    if ((0 != step_x) && (0 != step_y))
    {
        sideGridX = (uint16_t)((int32_t)current_x + step_x);
        sideGridY = current_y;
        if (MAP_OBSTACLE_BLOCKED == g_obstacleMap[sideGridY][sideGridX])
        {
            return 0U;
        }

        sideGridX = current_x;
        sideGridY = (uint16_t)((int32_t)current_y + step_y);
        if (MAP_OBSTACLE_BLOCKED == g_obstacleMap[sideGridY][sideGridX])
        {
            return 0U;
        }
    }

    return 1U;
}

/**
 * @brief  从 open 表取出 f 值最优节点
 * @param  goal_x/goal_y    目标栅格
 * @param  p_open_count     open 表节点数（输入输出）
 * @retval 最优节点线性编号或非法编号
 */
static uint16_t map_astar_pop_best(uint16_t goal_x,
                                   uint16_t goal_y,
                                   uint16_t *p_open_count)
{
    uint16_t openCount = 0U; /* 当前 open 表节点数量。 */
    uint16_t openIndex = 0U; /* 当前扫描的 open 表下标。 */
    uint16_t bestOpenIndex = 0U; /* 当前最优节点所在 open 表下标。 */
    uint16_t nodeIndex = 0U; /* 当前扫描节点的线性编号。 */
    uint16_t nodeGridX = 0U; /* 当前扫描节点的栅格 x 下标。 */
    uint16_t nodeGridY = 0U; /* 当前扫描节点的栅格 y 下标。 */
    uint16_t nodeHCost = 0U; /* 当前扫描节点的启发式代价。 */
    uint16_t bestHCost = UINT16_MAX; /* 当前最优节点的启发式代价。 */
    uint32_t nodeFCost = 0U; /* 当前扫描节点的总代价。 */
    uint32_t bestFCost = UINT32_MAX; /* 当前最优节点的总代价。 */
    uint16_t bestIndex = MAP_INDEX_INVALID; /* 当前最优节点的线性编号。 */

    if ((NULL == p_open_count) || (0U == *p_open_count))
    {
        return MAP_INDEX_INVALID;
    }

#if (MAP_USE_H_CACHE != 0U)
    (void)goal_x;
    (void)goal_y;
#endif

    openCount = *p_open_count;
    //对当前open表的所有点进行遍历找出代价和最小的点
    for (openIndex = 0U; openIndex < openCount; openIndex++)
    {
        nodeIndex = g_astarOpenList[openIndex];
        map_index_to_grid(nodeIndex, &nodeGridX, &nodeGridY);
#if (MAP_USE_H_CACHE != 0U)
        nodeHCost = g_astarHCost[nodeGridY][nodeGridX];
#else
        nodeHCost = map_astar_heuristic(nodeGridX, nodeGridY, goal_x, goal_y);
#endif
        nodeFCost = (uint32_t)g_astarGCost[nodeGridY][nodeGridX] + nodeHCost;

        if ((nodeFCost < bestFCost) ||
            ((nodeFCost == bestFCost) && (nodeHCost < bestHCost)))
        {
            bestFCost = nodeFCost;
            bestHCost = nodeHCost;
            bestOpenIndex = openIndex;
            bestIndex = nodeIndex;
        }
    }

    openCount--;
    g_astarOpenList[bestOpenIndex] = g_astarOpenList[openCount];
    *p_open_count = openCount;

    return bestIndex;
}

/**
 * @brief  将节点压入 open 表
 * @retval MAP_OK / MAP_ERR_PARAM / MAP_ERR_RES
 */
static map_status_t map_astar_push_open(uint16_t grid_x,
                                        uint16_t grid_y,
                                        uint16_t *p_open_count)
{
    uint16_t openCount = 0U; /* 当前 open 表节点数量。 */

    if (NULL == p_open_count)
    {
        return MAP_ERR_PARAM;
    }

    openCount = *p_open_count;
    if (openCount >= MAP_NODE_COUNT)
    {
        return MAP_ERR_RES;
    }

    g_astarOpenList[*p_open_count] = map_grid_to_index(grid_x, grid_y);
    *p_open_count = openCount + 1U;

    return MAP_OK;
}

/**
 * @brief  沿父节点链统计路径点数量
 * @retval MAP_OK / MAP_ERR_PARAM / MAP_ERR
 */
static map_status_t map_astar_count_path(uint16_t start_x,
                                         uint16_t start_y,
                                         uint16_t goal_x,
                                         uint16_t goal_y,
                                         uint16_t *p_required_len)
{
    uint16_t gridX = 0U; /* 当前回溯的栅格 x 下标。 */
    uint16_t gridY = 0U; /* 当前回溯的栅格 y 下标。 */
    uint16_t pathCount = 0U; /* 已统计的路径点数量。 */
    map_status_t status = MAP_ERR; /* 父节点回溯执行状态。 */

    if (NULL == p_required_len)
    {
        return MAP_ERR_PARAM;
    }

    gridX = goal_x;
    gridY = goal_y;
    pathCount = 1U;

    while ((gridX != start_x) || (gridY != start_y))
    {
        if (pathCount >= MAP_NODE_COUNT)
        {
            return MAP_ERR;
        }

        status = map_astar_parent(&gridX, &gridY);
        if (MAP_OK != status)
        {
            return status;
        }

        pathCount++;
    }

    *p_required_len = pathCount;

    return MAP_OK;
}

/**
 * @brief  将栅格坐标移动到其父节点坐标
 * @retval MAP_OK / MAP_ERR_PARAM / MAP_ERR_NO_PATH / MAP_ERR
 */
static map_status_t map_astar_parent(uint16_t *p_grid_x, uint16_t *p_grid_y)
{
    uint8_t parentDir = 0U; /* 当前节点的父方向。 */
    int32_t parentX = 0; /* 父节点 x 下标中间值。 */
    int32_t parentY = 0; /* 父节点 y 下标中间值。 */

    if ((NULL == p_grid_x) || (NULL == p_grid_y))
    {
        return MAP_ERR_PARAM;
    }

    if (0U == map_grid_is_valid(*p_grid_x, *p_grid_y))
    {
        return MAP_ERR_PARAM;
    }

    parentDir = g_astarParentDir[*p_grid_y][*p_grid_x];
    if ((MAP_ASTAR_PARENT_NONE == parentDir) || (parentDir >= 8U))
    {
        return MAP_ERR_NO_PATH;
    }

    parentX = (int32_t)(*p_grid_x) + g_astarDirOffsetX[parentDir];
    parentY = (int32_t)(*p_grid_y) + g_astarDirOffsetY[parentDir];
    if ((parentX < 0) || (parentY < 0) ||
        (parentX >= (int32_t)MAP_GRID_W) || (parentY >= (int32_t)MAP_GRID_H))
    {
        return MAP_ERR;
    }

    *p_grid_x = (uint16_t)parentX;
    *p_grid_y = (uint16_t)parentY;

    return MAP_OK;
}

/**
 * @brief  计算单个可通行栅格的障碍软代价
 * @retval 叠加到 A* 移动代价上的软代价
 */
static uint16_t map_astar_soft(uint16_t grid_x, uint16_t grid_y)
{
    uint16_t radiusMm = (uint16_t)MAP_SOFT_R_MM; /* 软代价影响半径，单位 mm。 */
    uint16_t maxCost = (uint16_t)MAP_SOFT_COST_MAX; /* 贴近障碍时的最大软代价。 */
    uint16_t radiusGrid = 0U; /* 软代价影响半径换算后的栅格数量。 */
    uint16_t startGridX = 0U; /* 搜索窗口起始 x 栅格下标。 */
    uint16_t endGridX = 0U; /* 搜索窗口结束 x 栅格下标。 */
    uint16_t startGridY = 0U; /* 搜索窗口起始 y 栅格下标。 */
    uint16_t endGridY = 0U; /* 搜索窗口结束 y 栅格下标。 */
    uint16_t scanGridX = 0U; /* 当前扫描的 x 栅格下标。 */
    uint16_t scanGridY = 0U; /* 当前扫描的 y 栅格下标。 */
    uint32_t radiusSq = 0U; /* 软代价影响半径平方。 */
    uint32_t bestDistanceSq = UINT32_MAX; /* 当前栅格到最近障碍的距离平方。 */
    uint32_t distanceSq = 0U; /* 当前扫描障碍点距离平方。 */
    uint32_t diffX = 0U; /* 当前扫描点相对目标点的 x 距离。 */
    uint32_t diffY = 0U; /* 当前扫描点相对目标点的 y 距离。 */
    uint32_t penalty = 0U; /* 根据距离换算出的软代价值。 */

    if ((0U == radiusMm) || (0U == maxCost))
    {
        return 0U;
    }

    radiusGrid = (uint16_t)((radiusMm + MAP_RES_MM - 1U) / MAP_RES_MM);
    if (grid_x > radiusGrid)
    {
        startGridX = (uint16_t)(grid_x - radiusGrid);
    }
    else
    {
        startGridX = 0U;
    }

    if (grid_y > radiusGrid)
    {
        startGridY = (uint16_t)(grid_y - radiusGrid);
    }
    else
    {
        startGridY = 0U;
    }

    endGridX = (uint16_t)(grid_x + radiusGrid);
    if (endGridX > MAP_GRID_MAX_X)
    {
        endGridX = MAP_GRID_MAX_X;
    }

    endGridY = (uint16_t)(grid_y + radiusGrid);
    if (endGridY > MAP_GRID_MAX_Y)
    {
        endGridY = MAP_GRID_MAX_Y;
    }

    radiusSq = (uint32_t)radiusMm * (uint32_t)radiusMm;
    for (scanGridY = startGridY; scanGridY <= endGridY; scanGridY++)
    {
        for (scanGridX = startGridX; scanGridX <= endGridX; scanGridX++)
        {
            if (MAP_OBSTACLE_BLOCKED != g_obstacleMap[scanGridY][scanGridX])
            {
                continue;
            }

            diffX = map_abs_diff_u16(grid_x, scanGridX);
            diffY = map_abs_diff_u16(grid_y, scanGridY);
            diffX *= MAP_RES_MM;
            diffY *= MAP_RES_MM;
            distanceSq = (diffX * diffX) + (diffY * diffY);
            if (distanceSq < bestDistanceSq)
            {
                bestDistanceSq = distanceSq;
            }
        }
    }

    if (bestDistanceSq >= radiusSq)
    {
        return 0U;
    }

    penalty = ((uint32_t)maxCost * (radiusSq - bestDistanceSq)) / radiusSq;
    if (penalty > maxCost)
    {
        penalty = maxCost;
    }

    return (uint16_t)penalty;
}

/**
 * @brief  将父节点链路原始路径写入复用缓冲区
 * @retval MAP_OK / MAP_ERR_PARAM / MAP_ERR
 */
static map_status_t map_astar_wr_raw(uint16_t start_x,
                                     uint16_t start_y,
                                     uint16_t goal_x,
                                     uint16_t goal_y,
                                     uint16_t path_len)
{
    uint16_t gridX = 0U; /* 当前回溯的栅格 x 下标。 */
    uint16_t gridY = 0U; /* 当前回溯的栅格 y 下标。 */
    uint16_t writeIndex = 0U; /* 当前写入路径索引缓存的位置。 */
    map_status_t status = MAP_ERR; /* 父节点回溯执行状态。 */

    if (0U == path_len)
    {
        return MAP_ERR_PARAM;
    }

    gridX = goal_x;
    gridY = goal_y;
    writeIndex = path_len;
    while (writeIndex > 0U)
    {
        writeIndex--;
        g_astarOpenList[writeIndex] = map_grid_to_index(gridX, gridY);
        if ((gridX == start_x) && (gridY == start_y))
        {
            return MAP_OK;
        }

        status = map_astar_parent(&gridX, &gridY);
        if (MAP_OK != status)
        {
            return status;
        }
    }

    return MAP_ERR;
}

/**
 * @brief  计算两栅格间不穿障碍直线段的代价
 * @retval MAP_OK / MAP_ERR_PARAM / MAP_ERR_BLOCKED
 */
static map_status_t map_astar_line_cost(uint16_t start_x,
                                        uint16_t start_y,
                                        uint16_t end_x,
                                        uint16_t end_y,
                                        uint32_t *p_line_cost)
{
    int32_t gridX = 0; /* 当前直线遍历的 x 栅格下标。 */
    int32_t gridY = 0; /* 当前直线遍历的 y 栅格下标。 */
    int32_t stepX = 0; /* 直线遍历的 x 步进方向。 */
    int32_t stepY = 0; /* 直线遍历的 y 步进方向。 */
    int32_t prevGridX = 0; /* 上一个直线遍历栅格 x 下标。 */
    int32_t prevGridY = 0; /* 上一个直线遍历栅格 y 下标。 */
    uint16_t absDeltaX = 0U; /* 起终点 x 栅格差绝对值。 */
    uint16_t absDeltaY = 0U; /* 起终点 y 栅格差绝对值。 */
    uint32_t crossX = 1U; /* 下一次跨越 x 栅格边界的比较分子。 */
    uint32_t crossY = 1U; /* 下一次跨越 y 栅格边界的比较分子。 */
    uint32_t compareX = 0U; /* x 边界比较值。 */
    uint32_t compareY = 0U; /* y 边界比较值。 */
    uint32_t lineCost = 0U; /* 当前累计的直线段总代价。 */
    uint8_t movedX = 0U; /* 本轮是否跨越了 x 栅格。 */
    uint8_t movedY = 0U; /* 本轮是否跨越了 y 栅格。 */

    if (NULL == p_line_cost)
    {
        return MAP_ERR_PARAM;
    }

    if ((0U == map_grid_is_valid(start_x, start_y)) || (0U == map_grid_is_valid(end_x, end_y)))
    {
        return MAP_ERR_PARAM;
    }

    if ((MAP_OBSTACLE_BLOCKED == g_obstacleMap[start_y][start_x]) ||
        (MAP_OBSTACLE_BLOCKED == g_obstacleMap[end_y][end_x]))
    {
        return MAP_ERR_BLOCKED;
    }

    absDeltaX = map_abs_diff_u16(start_x, end_x);
    absDeltaY = map_abs_diff_u16(start_y, end_y);
    stepX = (end_x > start_x) ? 1 : ((end_x < start_x) ? -1 : 0);
    stepY = (end_y > start_y) ? 1 : ((end_y < start_y) ? -1 : 0);
    gridX = (int32_t)start_x;
    gridY = (int32_t)start_y;

    while ((gridX != (int32_t)end_x) || (gridY != (int32_t)end_y))
    {
        prevGridX = gridX;
        prevGridY = gridY;
        movedX = 0U;
        movedY = 0U;

        if (0U == absDeltaX)
        {
            gridY += stepY;
            movedY = 1U;
        }
        else if (0U == absDeltaY)
        {
            gridX += stepX;
            movedX = 1U;
        }
        else
        {
            compareX = crossX * absDeltaY;
            compareY = crossY * absDeltaX;
            if (compareX < compareY)
            {
                gridX += stepX;
                crossX += 2U;
                movedX = 1U;
            }
            else if (compareY < compareX)
            {
                gridY += stepY;
                crossY += 2U;
                movedY = 1U;
            }
            else
            {
                if ((MAP_OBSTACLE_BLOCKED == g_obstacleMap[(uint16_t)gridY][(uint16_t)(gridX + stepX)]) ||
                    (MAP_OBSTACLE_BLOCKED == g_obstacleMap[(uint16_t)(gridY + stepY)][(uint16_t)gridX]))
                {
                    return MAP_ERR_BLOCKED;
                }

                gridX += stepX;
                gridY += stepY;
                crossX += 2U;
                crossY += 2U;
                movedX = 1U;
                movedY = 1U;
            }
        }

        if ((gridX < 0) || (gridY < 0) ||
            (gridX >= (int32_t)MAP_GRID_W) || (gridY >= (int32_t)MAP_GRID_H))
        {
            return MAP_ERR_PARAM;
        }

        if (MAP_OBSTACLE_BLOCKED == g_obstacleMap[(uint16_t)gridY][(uint16_t)gridX])
        {
            return MAP_ERR_BLOCKED;
        }

        if ((0U != movedX) && (0U != movedY) &&
            ((gridX != prevGridX) && (gridY != prevGridY)))
        {
            lineCost += MAP_ASTAR_DIAGONAL_COST;
        }
        else
        {
            lineCost += MAP_ASTAR_STRAIGHT_COST;
        }

        lineCost += map_astar_soft((uint16_t)gridX, (uint16_t)gridY);
    }

    *p_line_cost = lineCost;

    return MAP_OK;
}

/**
 * @brief  按 A* g 代价算原始路径一段的累计代价
 */
static uint32_t map_astar_seg_cost(uint16_t raw_start_index, uint16_t raw_end_index)
{
    uint16_t startGridX = 0U; /* 原始段起点 x 栅格下标。 */
    uint16_t startGridY = 0U; /* 原始段起点 y 栅格下标。 */
    uint16_t endGridX = 0U; /* 原始段终点 x 栅格下标。 */
    uint16_t endGridY = 0U; /* 原始段终点 y 栅格下标。 */

    map_index_to_grid(g_astarOpenList[raw_start_index], &startGridX, &startGridY);
    map_index_to_grid(g_astarOpenList[raw_end_index], &endGridX, &endGridY);

    return (uint32_t)g_astarGCost[endGridY][endGridX] - (uint32_t)g_astarGCost[startGridY][startGridX];
}

/**
 * @brief  判断原始路径两点能否用一条代价感知直线替换
 * @retval 1=可剪枝 0=不可
 */
static uint8_t map_astar_can_cut(uint16_t raw_start_index, uint16_t raw_end_index)
{
    uint16_t startGridX = 0U; /* 候选剪枝起点 x 栅格下标。 */
    uint16_t startGridY = 0U; /* 候选剪枝起点 y 栅格下标。 */
    uint16_t endGridX = 0U; /* 候选剪枝终点 x 栅格下标。 */
    uint16_t endGridY = 0U; /* 候选剪枝终点 y 栅格下标。 */
    uint32_t lineCost = 0U; /* 候选直线段的总代价。 */
    uint32_t rawCost = 0U; /* 原始路径对应段的总代价。 */
    map_status_t status = MAP_ERR; /* 直线段检查执行状态。 */

    if (raw_end_index <= (uint16_t)(raw_start_index + 1U))
    {
        return 1U;
    }

    map_index_to_grid(g_astarOpenList[raw_start_index], &startGridX, &startGridY);
    map_index_to_grid(g_astarOpenList[raw_end_index], &endGridX, &endGridY);
    status = map_astar_line_cost(startGridX, startGridY, endGridX, endGridY, &lineCost);
    if (MAP_OK != status)
    {
        return 0U;
    }

    rawCost = map_astar_seg_cost(raw_start_index, raw_end_index);
    if ((lineCost * MAP_ASTAR_COST_PER_MILLE) <=
        (rawCost * (uint32_t)MAP_SHORTCUT_RATIO))
    {
        return 1U;
    }

    return 0U;
}

/**
 * @brief  从锚点查找最远可剪枝目标点
 */
static uint16_t map_astar_find_cut(uint16_t raw_anchor_index, uint16_t raw_path_len)
{
    uint16_t targetIndex = 0U; /* 当前尝试的剪枝目标点下标。 */

    targetIndex = (uint16_t)(raw_path_len - 1U);
    while (targetIndex > (uint16_t)(raw_anchor_index + 1U))
    {
        if (0U != map_astar_can_cut(raw_anchor_index, targetIndex))
        {
            return targetIndex;
        }

        targetIndex--;
    }

    return (uint16_t)(raw_anchor_index + 1U);
}

/**
 * @brief  统计代价感知视线剪枝后的路径点数量
 * @retval MAP_OK / MAP_ERR_PARAM
 */
static map_status_t map_astar_cnt_prune(uint16_t raw_path_len, uint16_t *p_required_len)
{
    uint16_t anchorIndex = 0U; /* 当前剪枝锚点在原始路径中的下标。 */
    uint16_t targetIndex = 0U; /* 当前剪枝选中的目标点下标。 */
    uint16_t prunedCount = 0U; /* 已统计的剪枝后路径点数量。 */

    if ((NULL == p_required_len) || (0U == raw_path_len))
    {
        return MAP_ERR_PARAM;
    }

    prunedCount = 1U;
    while (anchorIndex < (uint16_t)(raw_path_len - 1U))
    {
        targetIndex = map_astar_find_cut(anchorIndex, raw_path_len);
        anchorIndex = targetIndex;
        prunedCount++;
    }

    *p_required_len = prunedCount;

    return MAP_OK;
}

/**
 * @brief  写出代价感知视线剪枝后的路径
 * @retval MAP_OK / MAP_ERR_PARAM / MAP_ERR
 */
static map_status_t map_astar_wr_prune(uint16_t raw_path_len,
                                       map_point_t *p_path,
                                       uint16_t path_len)
{
    uint16_t anchorIndex = 0U; /* 当前剪枝锚点在原始路径中的下标。 */
    uint16_t targetIndex = 0U; /* 当前剪枝选中的目标点下标。 */
    uint16_t writeIndex = 0U; /* 当前写入输出路径的位置。 */
    uint16_t pointGridX = 0U; /* 当前输出点 x 栅格下标。 */
    uint16_t pointGridY = 0U; /* 当前输出点 y 栅格下标。 */

    if ((NULL == p_path) || (0U == raw_path_len) || (0U == path_len))
    {
        return MAP_ERR_PARAM;
    }

    map_index_to_grid(g_astarOpenList[0], &pointGridX, &pointGridY);
    p_path[0].x_mm = (int16_t)(pointGridX * MAP_RES_MM);
    p_path[0].y_mm = (int16_t)(pointGridY * MAP_RES_MM);
    writeIndex = 1U;

    while (anchorIndex < (uint16_t)(raw_path_len - 1U))
    {
        if (writeIndex >= path_len)
        {
            return MAP_ERR;
        }

        targetIndex = map_astar_find_cut(anchorIndex, raw_path_len);
        map_index_to_grid(g_astarOpenList[targetIndex], &pointGridX, &pointGridY);
        p_path[writeIndex].x_mm = (int16_t)(pointGridX * MAP_RES_MM);
        p_path[writeIndex].y_mm = (int16_t)(pointGridY * MAP_RES_MM);
        anchorIndex = targetIndex;
        writeIndex++;
    }

    if (writeIndex != path_len)
    {
        return MAP_ERR;
    }

    return MAP_OK;
}

/**
 * @brief  初始化引擎，清空障碍与 A* 工作区
 * @retval MAP_OK
 */
map_status_t map_init(void)
{
    g_mapIsInitialized = 1U;
    map_clear_buf();
    map_astar_reset();

    return MAP_OK;
}

/**
 * @brief  清空所有障碍
 * @retval MAP_OK / MAP_ERR_INIT
 */
map_status_t map_clear(void)
{
    if (0U == g_mapIsInitialized)
    {
        return MAP_ERR_INIT;
    }

    map_clear_buf();

    return MAP_OK;
}

/**
 * @brief  设置单个栅格障碍状态（不膨胀）
 * @retval MAP_OK / MAP_ERR_PARAM / MAP_ERR_INIT
 */
map_status_t map_set_obs_grid(uint16_t grid_x,
                              uint16_t grid_y,
                              uint8_t is_obs)
{
    if (0U == g_mapIsInitialized)
    {
        return MAP_ERR_INIT;
    }

    if (0U == map_grid_is_valid(grid_x, grid_y))
    {
        return MAP_ERR_PARAM;
    }

    g_obstacleMap[grid_y][grid_x] = map_norm_obs(is_obs);

    return MAP_OK;
}

/**
 * @brief  按 mm 坐标设单点障碍（设障碍时按车半径膨胀）
 * @retval MAP_OK / MAP_ERR_PARAM / MAP_ERR_INIT
 */
map_status_t map_set_obs_mm(int16_t x_mm,
                            int16_t y_mm,
                            uint8_t is_obs)
{
    uint16_t gridX = 0U; /* 毫米坐标转换后的栅格 x 下标。 */
    uint16_t gridY = 0U; /* 毫米坐标转换后的栅格 y 下标。 */
    uint8_t obstacleValue = 0U; /* 归一化后的障碍标志。 */
    map_status_t status = MAP_ERR; /* 坐标转换执行状态。 */

    if (0U == g_mapIsInitialized)
    {
        return MAP_ERR_INIT;
    }

    status = map_mm_to_grid(x_mm, y_mm, &gridX, &gridY);
    if (MAP_OK != status)
    {
        return status;
    }

    obstacleValue = map_norm_obs(is_obs);
    if (MAP_OBSTACLE_BLOCKED == obstacleValue)
    {
        return map_fill_circle(x_mm, y_mm, 0U, obstacleValue);
    }

    return map_set_obs_grid(gridX, gridY, obstacleValue);
}

/**
 * @brief  按 mm 坐标设矩形障碍（设障碍时按车半径膨胀）
 * @retval MAP_OK / MAP_ERR_PARAM / MAP_ERR_INIT
 */
map_status_t map_set_obs_rect(int16_t x1_mm,
                              int16_t y1_mm,
                              int16_t x2_mm,
                              int16_t y2_mm,
                              uint8_t is_obs)
{
    if (0U == g_mapIsInitialized)
    {
        return MAP_ERR_INIT;
    }

    return map_fill_rect(x1_mm, y1_mm, x2_mm, y2_mm, is_obs);
}

/**
 * @brief  按 mm 坐标设圆形障碍（设障碍时按车半径膨胀）
 * @retval MAP_OK / MAP_ERR_PARAM / MAP_ERR_INIT
 */
map_status_t map_set_obs_circle(int16_t cx_mm,
                                int16_t cy_mm,
                                uint16_t r_mm,
                                uint8_t is_obs)
{
    if (0U == g_mapIsInitialized)
    {
        return MAP_ERR_INIT;
    }

    return map_fill_circle(cx_mm, cy_mm, r_mm, is_obs);
}

/**
 * @brief  读取单个栅格障碍状态
 * @retval 0=可通行；1=障碍或坐标非法/未初始化
 */
uint8_t map_is_obs(uint16_t grid_x, uint16_t grid_y)
{
    if (0U == g_mapIsInitialized)
    {
        return MAP_OBSTACLE_BLOCKED;
    }

    if (0U == map_grid_is_valid(grid_x, grid_y))
    {
        return MAP_OBSTACLE_BLOCKED;
    }

    return g_obstacleMap[grid_y][grid_x];
}

/**
 * @brief  软代价 A* 规划 + 代价感知视线剪枝（地图系 mm）
 * @param  sx_mm/sy_mm 起点  @param  gx_mm/gy_mm 终点
 * @param  p_path/path_capacity/p_path_len 输出路径/容量/点数
 * @retval MAP_OK / MAP_ERR_PARAM / MAP_ERR_INIT / MAP_ERR_RES
 *         / MAP_ERR_NO_PATH / MAP_ERR_BLOCKED
 */
map_status_t map_find_path(int16_t sx_mm,
                           int16_t sy_mm,
                           int16_t gx_mm,
                           int16_t gy_mm,
                           map_point_t *path,
                           uint16_t cap,
                           uint16_t *len)
{
    uint16_t startGridX = 0U; /* 起点栅格 x 下标。 */
    uint16_t startGridY = 0U; /* 起点栅格 y 下标。 */
    uint16_t goalGridX = 0U; /* 终点栅格 x 下标。 */
    uint16_t goalGridY = 0U; /* 终点栅格 y 下标。 */
    uint16_t currentGridX = 0U; /* 当前扩展节点的栅格 x 下标。 */
    uint16_t currentGridY = 0U; /* 当前扩展节点的栅格 y 下标。 */
    uint16_t nextGridX = 0U; /* 邻居节点的栅格 x 下标。 */
    uint16_t nextGridY = 0U; /* 邻居节点的栅格 y 下标。 */
    uint16_t openCount = 0U; /* 当前 open 表节点数量。 */
    uint16_t currentIndex = MAP_INDEX_INVALID; /* 当前扩展节点的线性编号。 */
    uint16_t rawPathLen = 0U; /* 回溯得到的原始路径点数量。 */
    uint16_t prunedPathLen = 0U; /* 代价感知剪枝后的路径点数量。 */
    uint16_t softCost = 0U; /* 邻居节点的障碍软代价。 */
    uint8_t dirIndex = 0U; /* 8 方向遍历下标。 */
    uint8_t parentDir = 0U; /* 邻居节点需要记录的父方向。 */
    int32_t nextX = 0; /* 邻居节点 x 下标中间值。 */
    int32_t nextY = 0; /* 邻居节点 y 下标中间值。 */
    uint32_t tentativeGCost = 0U; /* 从当前节点到邻居的候选 g 代价。 */
    map_status_t status = MAP_ERR; /* 子步骤执行状态。 */


//1.参数检查
    if ((NULL == path) || (NULL == len) || (0U == cap))
    {
        return MAP_ERR_PARAM;
    }

    *len = 0U;
    if (0U == g_mapIsInitialized)
    {
        return MAP_ERR_INIT;
    }

//2.毫米坐标转换
    status = map_mm_to_grid(sx_mm, sy_mm, &startGridX, &startGridY);
    if (MAP_OK != status)
    {
        return status;
    }
    status = map_mm_to_grid(gx_mm, gy_mm, &goalGridX, &goalGridY);
    if (MAP_OK != status)
    {
        return status;
    }

//3.判断坐标是否坐落于障碍中，是则返回
    if ((MAP_OBSTACLE_BLOCKED == g_obstacleMap[startGridY][startGridX]) ||
        (MAP_OBSTACLE_BLOCKED == g_obstacleMap[goalGridY][goalGridX]))
    {
        return MAP_ERR_BLOCKED;
    }

//4.重置工作区，后续优化，闲时重置，这个一定耗时很长
    map_astar_reset();

//5.开始计算,A*:f(x) = g(x) + h(x),g(x)是实际代价，h(x)是估计代价
    //清零起点代价
    g_astarGCost[startGridY][startGridX] = 0U;
    //起点父节点为无
    g_astarParentDir[startGridY][startGridX] = MAP_ASTAR_PARENT_NONE;
#if (MAP_USE_H_CACHE != 0U)
    g_astarHCost[startGridY][startGridX] = map_astar_heuristic(startGridX,
                                                              startGridY,
                                                              goalGridX,
                                                              goalGridY);
#endif
    //将起点加入OPEN表
    g_astarState[startGridY][startGridX] = MAP_ASTAR_STATE_OPEN;
    status = map_astar_push_open(startGridX, startGridY, &openCount);
    if (MAP_OK != status)
    {
        return status;
    }

    while (openCount > 0U)
    {

        currentIndex = map_astar_pop_best(goalGridX, goalGridY, &openCount);
        if (MAP_INDEX_INVALID == currentIndex)
        {
            return MAP_ERR;
        }

        map_index_to_grid(currentIndex, &currentGridX, &currentGridY);
        if ((currentGridX == goalGridX) && (currentGridY == goalGridY))
        {
            status = map_astar_count_path(startGridX,
                                          startGridY,
                                          goalGridX,
                                          goalGridY,
                                          &rawPathLen);
            if (MAP_OK != status)
            {
                return status;
            }

            status = map_astar_wr_raw(startGridX,
                                      startGridY,
                                      goalGridX,
                                      goalGridY,
                                      rawPathLen);
            if (MAP_OK != status)
            {
                return status;
            }

            status = map_astar_cnt_prune(rawPathLen, &prunedPathLen);
            if (MAP_OK != status)
            {
                return status;
            }

            *len = prunedPathLen;
            if (prunedPathLen > cap)
            {
                return MAP_ERR_RES;
            }

            return map_astar_wr_prune(rawPathLen, path, prunedPathLen);
        }

        g_astarState[currentGridY][currentGridX] = MAP_ASTAR_STATE_CLOSED;
        for (dirIndex = 0U; dirIndex < 8U; dirIndex++)
        {
            if (0U == map_astar_can_step(currentGridX,
                                         currentGridY,
                                         g_astarDirOffsetX[dirIndex],
                                         g_astarDirOffsetY[dirIndex]))
            {
                continue;
            }

            nextX = (int32_t)currentGridX + g_astarDirOffsetX[dirIndex];
            nextY = (int32_t)currentGridY + g_astarDirOffsetY[dirIndex];
            nextGridX = (uint16_t)nextX;
            nextGridY = (uint16_t)nextY;

            if (MAP_ASTAR_STATE_CLOSED == g_astarState[nextGridY][nextGridX])
            {
                continue;
            }

            softCost = map_astar_soft(nextGridX, nextGridY);
            tentativeGCost = (uint32_t)g_astarGCost[currentGridY][currentGridX] +
                             g_astarDirCost[dirIndex] +
                             softCost;
            if ((MAP_ASTAR_STATE_NONE == g_astarState[nextGridY][nextGridX]) ||
                (tentativeGCost < g_astarGCost[nextGridY][nextGridX]))
            {
                if (tentativeGCost > UINT16_MAX)
                {
                    return MAP_ERR_RES;
                }

                parentDir = (uint8_t)((dirIndex + 4U) & 0x07U);
                g_astarGCost[nextGridY][nextGridX] = (uint16_t)tentativeGCost;
                g_astarParentDir[nextGridY][nextGridX] = parentDir;

                if (MAP_ASTAR_STATE_NONE == g_astarState[nextGridY][nextGridX])
                {
#if (MAP_USE_H_CACHE != 0U)
                    g_astarHCost[nextGridY][nextGridX] = map_astar_heuristic(nextGridX,
                                                                            nextGridY,
                                                                            goalGridX,
                                                                            goalGridY);
#endif
                    g_astarState[nextGridY][nextGridX] = MAP_ASTAR_STATE_OPEN;
                    status = map_astar_push_open(nextGridX, nextGridY, &openCount);
                    if (MAP_OK != status)
                    {
                        return status;
                    }
                }
            }
        }
    }

    return MAP_ERR_NO_PATH;
}
