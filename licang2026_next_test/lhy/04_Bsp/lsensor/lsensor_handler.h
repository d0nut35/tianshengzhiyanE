/**
 * @file    lsensor_handler.h
 * @brief   固定六路灰度传感器对象组管理接口
 * @note    - Handler 内部持有全部 lsensor_t 对象
 *          - 内部任务周期读取 IDR 并更新对象缓存
 *          - 上层仅按板上序号 1~6 读取缓存电平
 */

#ifndef LSENSOR_HANDLER_H
#define LSENSOR_HANDLER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define LSENSOR_COUNT   6U   /* 板载灰度传感器数量 */

/* Handler 状态码，OK == 0 */
typedef enum {
    LSH_OK      = 0U,
    LSH_ERR_RES = 1U
} lsh_status_t;

/* 板上丝印序号，数值与物理传感器编号一致 */
typedef enum {
    LSENSOR_ID_1 = 1U,
    LSENSOR_ID_2,
    LSENSOR_ID_3,
    LSENSOR_ID_4,
    LSENSOR_ID_5,
    LSENSOR_ID_6
} lsensor_id_t;

/* 原始缓存电平：LOW=检测到线，HIGH=未检测到线；
 * INVALID=序号非法或 Handler 未初始化 */
typedef enum {
    LSENSOR_LEVEL_LOW     = 0U,
    LSENSOR_LEVEL_HIGH    = 1U,
    LSENSOR_LEVEL_INVALID = 0xFFU
} lsensor_level_t;

/**
 * @brief  初始化对象组并创建周期采样任务
 * @retval LSH_OK / LSH_ERR_RES
 * @note   重复调用已启动的 Handler 时直接返回 LSH_OK
 */
lsh_status_t lsh_init(void);

/**
 * @brief  读取指定传感器的最近一次缓存电平
 * @param  id 板上物理序号，LSENSOR_ID_1 ~ LSENSOR_ID_6
 * @retval LSENSOR_LEVEL_LOW / LSENSOR_LEVEL_HIGH /
 *         LSENSOR_LEVEL_INVALID
 */
lsensor_level_t lsh_get_level(lsensor_id_t id);

#ifdef __cplusplus
}
#endif

#endif /* LSENSOR_HANDLER_H */
