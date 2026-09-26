/**
 * @file    hwt101.h
 * @brief   HWT101 陀螺仪协议解析（纯函数，平台无关）
 * @author  haoyu
 * @note    - 零 HAL / 零状态：只解析调用方给的字节，可在 PC 单测
 *          - DMA 双缓冲与最新值缓存归适配层，本层不持状态
 *          - 懒扫描方案：解析在任务上下文调用，中断不解析
 */

#ifndef HWT101_H
#define HWT101_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* HWT101 每个输出周期连发 2 个子包(各 11 字节)，共 22 字节：
 *   0x55 0x52 …  角速度(Wz)
 *   0x55 0x53 …  角度(Yaw)
 * parse 全缓冲扫描，一趟把两个子包都解出来。 */
#define HWT101_FRAME_LEN    11U     /* 单个子包(帧)字节数，
                                    * 非一次传输长度 */
#define HWT101_FRAME_HEAD   0x55U   /* 子包帧头 */
#define HWT101_ID_GYRO      0x52U   /* 角速度子包 ID */
#define HWT101_ID_ANGLE     0x53U   /* 角度子包 ID */

/* 配置写帧 FF AA REG DL DH；解锁后≥200ms、置零后≥500ms 再写下一帧 */
#define HWT101_REG_SAVE     0x00U   /* 保存配置，数据 0x0000 */
#define HWT101_REG_UNLOCK   0x69U   /* 解锁寄存器，数据 0xB588 */
#define HWT101_REG_CALIYAW  0x76U   /* Z 轴角度置零，数据 0x0000 */
#define HWT101_UNLOCK_DL    0x88U   /* 解锁键低字节 */
#define HWT101_UNLOCK_DH    0xB5U   /* 解锁键高字节 */

/* 状态码：OK==0，适配层据此逐层映射 */
typedef enum {
    HWT101_OK        = 0,    /* 成功 */
    HWT101_ERR       = 1,    /* 通用错误 / 本段无有效帧 */
    HWT101_ERR_PARAM = 2,    /* 参数非法 */
    HWT101_ERR_INIT  = 3,    /* 未初始化 */
    HWT101_ERR_RES   = 4,    /* 资源不可用 / 暂无有效数据 */
    HWT101_ERR_TMO   = 5,    /* 操作超时（适配层映射 HAL_TIMEOUT） */
} hwt101_status_t;

/* 一次解析的输出：本段里解出的最新角度 / 角速度 */
typedef struct {
    float   yaw_deg;    /* 偏航角，单位 度（has_yaw 为真时有效） */
    /* 角速度 Wz，单位 度/秒（has_gyro 为真时有效） */
    float   gyro_dps;
    uint8_t has_yaw;    /* 本段是否解出角度 */
    uint8_t has_gyro;   /* 本段是否解出角速度 */
} hwt101_sample_t;

/**
 * @brief  扫描一段原始字节，解出最新角度 / 角速度
 * @param  data 原始字节缓冲（DMA 收到的内容）
 * @param  len  有效字节数
 * @param  out  解析结果；同段多帧各量取最后一帧，未解出则对应 has_*=0
 * @note   找包头 0x55 → 校验 → 解码；纯函数、零平台依赖
 * @retval HWT101_OK(≥1 有效帧) / HWT101_ERR(无有效帧) /
 *         HWT101_ERR_PARAM
 */
hwt101_status_t hwt101_parse(const uint8_t *data, uint16_t len,
                             hwt101_sample_t *out);

#ifdef __cplusplus
}
#endif

#endif /* HWT101_H */
