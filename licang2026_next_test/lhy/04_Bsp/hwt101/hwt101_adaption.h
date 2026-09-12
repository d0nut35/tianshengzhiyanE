/**
 * @file    hwt101_adaption.h
 * @brief   固定 huart2 的 HWT101 单实例接口，不暴露 HAL 类型
 * @note    UART 配置由板级代码完成；接收为 2 x 64 字节 DMA 双缓冲。
 *          ISR 不解析；任务读取最近完成批次，允许重复读取。
 */

#ifndef HWT101_ADAPTION_H
#define HWT101_ADAPTION_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define HWT101_FRAME_LEN   11U     /* 一个输出子帧的字节数 */
#define HWT101_FRAME_HEAD  0x55U   /* 输出帧头 */
#define HWT101_ID_GYRO     0x52U   /* 角速度 Wz 子帧 */
#define HWT101_ID_ANGLE    0x53U   /* 航向 yaw 子帧 */
#define HWT101_REG_SAVE    0x00U   /* 保存配置，数据为 0 */
#define HWT101_REG_UNLOCK  0x69U   /* 解锁寄存器，数据为 0xB588 */
#define HWT101_REG_CALIYAW 0x76U   /* 硬件 Z 轴置零，数据为 0 */
#define HWT101_UNLOCK_DL   0x88U   /* 解锁键低字节 */
#define HWT101_UNLOCK_DH   0xB5U   /* 解锁键高字节 */

/* 保留上层使用的类型及数值，不增加硬件配置或注入接口。 */
typedef enum {
    HWT101_OK        = 0,   /* 成功 */
    HWT101_ERR       = 1,   /* 接收/发送失败，或初始化时接收仍活动 */
    HWT101_ERR_PARAM = 2,   /* 参数非法或 RX DMA 尚未绑定 */
    HWT101_ERR_INIT  = 3,   /* 未初始化 */
    HWT101_ERR_RES   = 4,   /* 最近批次无有效 yaw，或读取期间被覆盖 */
    HWT101_ERR_TMO   = 5,   /* 阻塞发送超时 */
} hwt101_status_t;

/**
 * @brief  检查 huart2 RX DMA 并清空状态；只能在接收停止时调用
 * @note   init/start 在任务中串行调用；不重配 UART 或创建 OS 资源。
 * @return HWT101_OK / HWT101_ERR_PARAM / HWT101_ERR
 */
hwt101_status_t hwt101_adp_init(void);

/**
 * @brief  启动接收；已启动时幂等，DMA 挂接失败后可再次调用重试
 * @return HWT101_OK / HWT101_ERR_INIT / HWT101_ERR
 */
hwt101_status_t hwt101_adp_start(void);

/**
 * @brief  任务中读取最近完成批次，重复读取命中缓存仍返回成功
 * @param  gyro_dps Wz，deg/s；该批无 gyro 时保留原值
 * @param  yaw_deg 带软件偏置的航向，范围 [-180, 180)，deg
 * @note   两指针均须非空；失败不写输出，多个任务无需外部互斥。
 * @return HWT101_OK / HWT101_ERR_PARAM / HWT101_ERR_INIT / HWT101_ERR_RES
 */
hwt101_status_t hwt101_adp_read(float *gyro_dps, float *yaw_deg);

/**
 * @brief  任务中将当前原始航向标定为指定角度，随后读取立即生效
 * @param  yaw_deg 有限角度，deg；支持多圈输入并归一化
 * @note   使用最近批次原始 yaw；失败不改变已有偏置。
 * @return HWT101_OK / HWT101_ERR_PARAM / HWT101_ERR_INIT / HWT101_ERR_RES
 */
hwt101_status_t hwt101_adp_set_yaw(float yaw_deg);

/**
 * @brief  任务中阻塞发送 FF AA reg lo hi，超时为 100 ms
 * @param  reg 寄存器；lo/hi 分别为数据低/高字节
 * @note   不依赖 init；解锁后至少等 200 ms、置零后至少等 500 ms，
 *         帧间等待由调用方负责。
 * @return HWT101_OK / HWT101_ERR_TMO / HWT101_ERR
 */
hwt101_status_t hwt101_adp_write_reg(uint8_t reg, uint8_t lo, uint8_t hi);

/**
 * @brief  上电配置：解锁 → Z 轴硬件置零 → 保存，内部按手册排帧间等待
 * @note   须在 hwt101_adp_start() 之前于任务上下文调用，阻塞约 720 ms；
 *         失败只影响上电零位，上层仍可用 set_yaw 软件标定。
 * @return HWT101_OK / HWT101_ERR_TMO / HWT101_ERR
 */
hwt101_status_t hwt101_adp_boot_cfg(void);

/**
 * @brief  接收 ISR 入口：记录长度、使旧缓存失效并切换 DMA 缓冲
 * @param  size 完成块长度，超过 64 字节按容量截断
 * @note   由统一 UART 路由过滤 huart2 后调用；不解析、不阻塞。
 * @return HWT101_OK / HWT101_ERR_INIT / HWT101_ERR
 */
hwt101_status_t hwt101_adp_rx_isr(uint16_t size);

/**
 * @brief  错误 ISR 入口：丢弃错误批次并在当前活动块重启 DMA
 * @return HWT101_OK / HWT101_ERR_INIT / HWT101_ERR
 */
hwt101_status_t hwt101_adp_err_isr(void);

#ifdef __cplusplus
}
#endif

#endif /* HWT101_ADAPTION_H */
