/**
 * @file    ble.c
 * @brief   BLE 导航帧解析器（纯逻辑，平台无关）
 * @author  haoyu
 * @note    - 滑动窗口拆 12B 定长帧：55 cmd x2 y2 sgn yaw v2 w 6B
 *          - 仅标准库依赖；解析出整帧经回调上抛，单线程访问
 */

#include "ble.h"

#include <stdbool.h>
#include <stddef.h>
#include <string.h>

#define BLE_HEAD        0x55U   /* 帧头 */
#define BLE_TAIL        0x6BU   /* 帧尾 */
#define BLE_FRAME_LEN   12U     /* 定长帧字节数 */
#define BLE_CMD_LINE    0x01U   /* 命令字：直线导航 */
#define BLE_CMD_PATH    0x02U   /* 命令字：路径导航 */
#define BLE_YAW_NEG     0x00U   /* yaw 符号：负 */
#define BLE_YAW_POS     0x01U   /* yaw 符号：正 */

#define BLE_OFF_CMD     1U      /* 命令字偏移 */
#define BLE_OFF_X       2U      /* 目标 X 偏移（大端 s16） */
#define BLE_OFF_Y       4U      /* 目标 Y 偏移（大端 s16） */
#define BLE_OFF_YAW_SGN 6U      /* yaw 符号偏移 */
#define BLE_OFF_YAW     7U      /* yaw 绝对值偏移（deg） */
#define BLE_OFF_V       8U      /* 最大线速度偏移（大端 u16） */
#define BLE_OFF_W       10U     /* 最大角速度偏移（deg/s） */
#define BLE_OFF_TAIL    11U     /* 帧尾偏移 */

/* ---- 内部工具 ---- */

/** @brief 读大端 int16 */
static int16_t rd_s16(const uint8_t *d)
{
    return (int16_t)(((uint16_t)d[0] << 8) | (uint16_t)d[1]);
}

/** @brief 读大端 uint16 */
static uint16_t rd_u16(const uint8_t *d)
{
    return (uint16_t)(((uint16_t)d[0] << 8) | (uint16_t)d[1]);
}

/** @brief 在窗口内找首个帧头位置，无则返回 len */
static uint16_t find_head(const ble_parser_t *p)
{
    uint16_t i = 0U; /* 帧头搜索下标 */

    for (i = 0U; i < p->len; i++) {
        if (p->buf[i] == BLE_HEAD) {
            return i;
        }
    }
    return p->len;
}

/** @brief 丢弃窗口头部 n 字节，余下前移 */
static void discard(ble_parser_t *p, uint16_t n)
{
    uint16_t remain = 0U; /* 丢弃后剩余字节数 */

    if (n == 0U) {
        return;
    }
    if (n >= p->len) {
        p->len = 0U;
        return;
    }
    remain = (uint16_t)(p->len - n);
    (void)memmove(p->buf, &p->buf[n], remain);
    p->len = remain;
}

/** @brief 命令字是否为支持的导航模式 */
static bool cmd_valid(uint8_t cmd)
{
    return ((cmd == BLE_CMD_LINE) || (cmd == BLE_CMD_PATH));
}

/** @brief 校验定长帧的头/命令/尾/yaw 符号字段 */
static bool frame_valid(const uint8_t *f)
{
    uint8_t sgn = f[BLE_OFF_YAW_SGN]; /* yaw 符号字节 */

    if ((f[0] != BLE_HEAD) || (!cmd_valid(f[BLE_OFF_CMD])) ||
        (f[BLE_OFF_TAIL] != BLE_TAIL)) {
        return false;
    }
    if ((sgn != BLE_YAW_NEG) && (sgn != BLE_YAW_POS)) {
        return false;
    }
    return true;
}

/** @brief 解码已校验的定长帧到导航命令（协议原始量） */
static void decode_frame(const uint8_t *f, ble_nav_t *nav)
{
    uint8_t sgn = f[BLE_OFF_YAW_SGN]; /* yaw 符号字节 */
    uint8_t deg = f[BLE_OFF_YAW];     /* yaw 绝对值，deg */

    nav->mode = (ble_nav_mode_t)f[BLE_OFF_CMD];
    nav->x_mm = rd_s16(&f[BLE_OFF_X]);
    nav->y_mm = rd_s16(&f[BLE_OFF_Y]);
    if (sgn == BLE_YAW_NEG) {
        nav->yaw_deg = (int16_t)(-(int16_t)deg); /* 负向航向 */
    } else {
        nav->yaw_deg = (int16_t)deg;
    }
    nav->v_max = rd_u16(&f[BLE_OFF_V]);
    nav->w_max_deg = f[BLE_OFF_W];
}

/** @brief 从窗口尽量拆出完整帧并回调，残缺则保留待续 */
static void consume(ble_parser_t *p)
{
    uint16_t head = 0U;   /* 窗口内帧头位置 */
    ble_nav_t nav = { BLE_NAV_LINE, 0, 0, 0, 0U, 0U }; /* 导航命令 */

    while (p->len > 0U) {
        head = find_head(p);
        if (head > 0U) {
            discard(p, head); /* 丢掉帧头前的杂字节 */
        }
        if (p->len < 2U) {
            return; /* 命令字还没到 */
        }
        if (!cmd_valid(p->buf[BLE_OFF_CMD])) {
            discard(p, 1U); /* 假头，跳过重找 */
            continue;
        }
        if (p->len < BLE_FRAME_LEN) {
            return; /* 整帧还没到 */
        }
        if (!frame_valid(p->buf)) {
            discard(p, 1U);
            continue;
        }
        decode_frame(p->buf, &nav);
        if (p->pf_on_nav != NULL) {
            p->pf_on_nav(p->cb_ctx, &nav);
        }
        discard(p, BLE_FRAME_LEN);
    }
}

/* ---- 对外接口 ---- */

/**
 * @brief  实例化解析器并注入整帧回调
 * @param  p/on_nav/ctx 对象 / 整帧回调（非空）/ 回调上下文
 * @retval BLE_OK / BLE_ERR_PARAM
 */
ble_status_t ble_inst(ble_parser_t *p, ble_nav_cb_t on_nav, void *ctx)
{
    if ((p == NULL) || (on_nav == NULL)) {
        return BLE_ERR_PARAM;
    }
    p->is_inited = 1U;
    p->len       = 0U;
    p->pf_on_nav = on_nav;
    p->cb_ctx    = ctx;
    return BLE_OK;
}

/**
 * @brief  喂入接收字节流，逐帧拆解并回调
 * @param  p/data/len 解析器 / 字节缓冲 / 字节长度
 * @retval BLE_OK / BLE_ERR_PARAM / BLE_ERR_INIT
 */
ble_status_t ble_feed(ble_parser_t *p, const uint8_t *data, uint16_t len)
{
    uint16_t i = 0U; /* 当前喂入字节下标 */

    if ((p == NULL) || (data == NULL)) {
        return BLE_ERR_PARAM;
    }
    if (p->is_inited == 0U) {
        return BLE_ERR_INIT;
    }
    for (i = 0U; i < len; i++) {
        if (p->len >= BLE_BUF_SIZE) {
            discard(p, 1U); /* 窗口满，丢最老字节滑动 */
        }
        p->buf[p->len] = data[i];
        p->len++;
        consume(p);
    }
    return BLE_OK;
}
