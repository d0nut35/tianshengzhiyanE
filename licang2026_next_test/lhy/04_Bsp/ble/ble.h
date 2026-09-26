/**
 * @file    ble.h
 * @brief   BLE 导航帧解析器（纯逻辑，平台无关）
 * @author  haoyu
 * @note    - 零 HAL / 零 RTOS：仅滑动窗口拆帧 + 字段解码，可 PC 单测
 *          - 12B 定长帧：55 cmd x2 y2 sgn yaw v2 w 6B（x/y/v 大端）
 *          - 输出协议原始量（mm/deg），deg→rad 换算在适配层
 */

#ifndef BLE_H
#define BLE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define BLE_BUF_SIZE    64U     /* 解析滑动窗口字节数，容多帧粘连 */

/* 状态码：OK==0，适配层据此逐层映射 */
typedef enum {
    BLE_OK        = 0,    /* 成功 */
    BLE_ERR       = 1,    /* 通用错误 */
    BLE_ERR_PARAM = 2,    /* 参数非法 */
    BLE_ERR_INIT  = 3,    /* 未初始化 */
} ble_status_t;

/* 导航模式，与协议命令字节对齐 */
typedef enum {
    BLE_NAV_LINE = 0x01U,   /* 直线导航 */
    BLE_NAV_PATH = 0x02U,   /* 路径导航（上层走 A* 寻路） */
} ble_nav_mode_t;

/* 解析输出：协议原始量，单位换算留给适配层 */
typedef struct {
    ble_nav_mode_t mode;        /* 导航模式 */
    int16_t        x_mm;        /* 目标 X，mm */
    int16_t        y_mm;        /* 目标 Y，mm */
    int16_t        yaw_deg;     /* 目标航向，deg，带符号 */
    uint16_t       v_max;       /* 最大线速度，mm/s */
    uint8_t        w_max_deg;   /* 最大角速度，deg/s */
} ble_nav_t;

/* 解析出一帧的回调：nav 仅在回调内有效，调用方需即时取用 */
typedef void (*ble_nav_cb_t)(void *ctx, const ble_nav_t *nav);

/* 解析器对象：滑动窗口 + 注入的整帧回调，由适配层持有一份 */
typedef struct {
    uint8_t      is_inited;         /* 是否已实例化 */
    uint16_t     len;               /* 窗口内有效字节数 */
    uint8_t      buf[BLE_BUF_SIZE]; /* 滑动窗口缓存 */
    ble_nav_cb_t pf_on_nav;         /* 解析出一帧的回调 */
    void        *cb_ctx;            /* 回调上下文 */
} ble_parser_t;

/**
 * @brief  实例化解析器并注入整帧回调
 * @param  p      解析器对象
 * @param  on_nav 解析出一帧的回调，不可为空
 * @param  ctx    回调上下文，可为空
 * @retval BLE_OK / BLE_ERR_PARAM
 */
ble_status_t ble_inst(ble_parser_t *p, ble_nav_cb_t on_nav, void *ctx);

/**
 * @brief  喂入接收字节流，逐帧拆解并回调（任务上下文，单线程访问）
 * @param  p    解析器对象
 * @param  data 接收字节缓冲
 * @param  len  字节长度
 * @retval BLE_OK / BLE_ERR_PARAM / BLE_ERR_INIT
 */
ble_status_t ble_feed(ble_parser_t *p, const uint8_t *data, uint16_t len);

#ifdef __cplusplus
}
#endif

#endif /* BLE_H */
