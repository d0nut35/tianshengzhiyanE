/**
 * @file    hwt101.c
 * @brief   HWT101 陀螺仪协议解析（纯函数，平台无关）
 * @author  haoyu
 * @note    - 全缓冲找包头 0x55，逐帧校验解码，兼容前导乱码
 *          - 同段多帧时各量取最后一帧（最新）；零状态、可单测
 */

#include "hwt101.h"

#include <stddef.h>     /* NULL */

#define HWT101_GYRO_FS   2000.0f   /* 角速度满量程，单位 度/秒 */
#define HWT101_ANGLE_FS   180.0f   /* 角度满量程，单位 度 */
#define HWT101_RAW_FS   32768.0f   /* int16 满量程 */

/**
 * @brief 校验一帧 11 字节子帧的校验和
 *        （前 10 字节累加 == 第 11 字节）
 */
static uint8_t csum_ok(const uint8_t *f)
{
    uint8_t s = 0U;     /* 累加和 */
    uint8_t i;          /* 字节下标 */

    for (i = 0U; i < (HWT101_FRAME_LEN - 1U); i++) {
        s = (uint8_t)(s + f[i]);
    }
    return (s == f[HWT101_FRAME_LEN - 1U]) ? 1U : 0U;
}

/** @brief 取子帧数据区 int16（YawL/WzL 在索引 6，高字节在索引 7） */
static int16_t raw_le16(const uint8_t *f)
{
    return (int16_t)(((uint16_t)f[7] << 8U) | (uint16_t)f[6]);
}

hwt101_status_t hwt101_parse(const uint8_t *data, uint16_t len,
                             hwt101_sample_t *out)
{
    uint16_t i = 0U;    /* 扫描下标 */
    int16_t  raw;       /* 解出的原始 int16 量 */

    if ((data == NULL) || (out == NULL)) {
        return HWT101_ERR_PARAM;
    }
    out->yaw_deg = 0.0f;
    out->gyro_dps = 0.0f;
    out->has_yaw = 0U;
    out->has_gyro = 0U;

    while (i < len) {
        /* 找包头，乱码逐字节跳 */
        if (data[i] != HWT101_FRAME_HEAD) {
            i++;
            continue;
        }
        /* 尾部不足一帧，收尾 */
        if ((uint16_t)(len - i) < HWT101_FRAME_LEN) {
            break;
        }
        /* 校验失败只前进 1 字节 */
        if (csum_ok(&data[i]) == 0U) {
            i++;
            continue;
        }
        raw = raw_le16(&data[i]);
        if (data[i + 1U] == HWT101_ID_GYRO) {
            out->gyro_dps = ((float)raw * HWT101_GYRO_FS)
                            / HWT101_RAW_FS;
            out->has_gyro = 1U;
        } else if (data[i + 1U] == HWT101_ID_ANGLE) {
            out->yaw_deg = ((float)raw * HWT101_ANGLE_FS)
                           / HWT101_RAW_FS;
            out->has_yaw = 1U;
        } else {
            /* 未知功能码 */
            i++;
            continue;
        }
        i = (uint16_t)(i + HWT101_FRAME_LEN);           /* 整帧消费 */
    }

    return ((out->has_yaw != 0U) || (out->has_gyro != 0U)) ?
           HWT101_OK : HWT101_ERR;
}
