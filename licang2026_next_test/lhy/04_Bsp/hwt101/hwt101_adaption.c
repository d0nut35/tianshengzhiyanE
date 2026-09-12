/**
 * @file    hwt101_adaption.c
 * @brief   固定 huart2 的 HWT101 收发、协议解析与航向标定
 * @note    DMA 使用 2 x 64 字节缓冲，ISR 只切换接收资源。
 *          任务按接收序号发布原始值缓存；短临界区保护共享状态，
 *          扫描期间发生接收或错误事件时丢弃本次结果。
 */

#include "hwt101_adaption.h"

#include <math.h>
#include <stddef.h>
#include "cmsis_os2.h"
#include "usart.h"

#define HWT101_RX_LEN      64U          /* 单块 DMA 缓冲长度 */
#define HWT101_TX_TMO      100U         /* 寄存器发送超时，ms */
#define HWT101_UNLOCK_MS   210U         /* 解锁后到置零帧的间隔，ms */
#define HWT101_ZERO_MS     510U         /* 置零后到保存帧的间隔，ms */
#define HWT101_HAS_YAW     1U           /* 本批存在有效 yaw */
#define HWT101_HAS_GYRO    2U           /* 本批存在有效 Wz */
#define HWT101_YAW_SCALE   0.0054931640625f /* 180 / 32768 */
#define HWT101_GYRO_SCALE  0.06103515625f   /* 2000 / 32768 */

/* 缓存只存原始值，改变软件偏置无需重新扫描同一批数据。 */
typedef struct {
    float    yaw;     /* 原始航向，deg */
    float    gyro;    /* 原始角速度，deg/s */
    uint8_t  flags;   /* 本批有效量，禁止跨批拼接 */
} hwt_sample_t;

static uint8_t g_buf[2][HWT101_RX_LEN]; /* DMA 生命周期内始终有效 */
static volatile uint32_t g_rx_seq;     /* 每次接收/失效递增，允许回绕 */
static volatile uint16_t g_rx_len;     /* 最近完成块的长度 */
static volatile uint8_t g_active;      /* DMA 当前写入的块 */
static volatile uint8_t g_started;     /* DMA 已启动或正在挂接 */
static uint8_t g_inited;               /* 固定资源是否已检查 */
static uint8_t g_cached;               /* 当前批次是否已有解析缓存 */
static hwt_sample_t g_sample;          /* 任务在短临界区内读写 */
static float g_yaw_ofs;                /* 任务在短临界区内更新偏置 */

/**
 * @brief  在指定块挂接接收；失败使数据失效，允许 start 重试
 * @param  idx 已检查的缓冲下标，0/1
 * @param  size 刚完成块长度；启动或错误恢复传 0，使旧数据失效
 * @return HWT101_OK / HWT101_ERR
 * @note   发布下标早于 HAL 使能接收，避免立即到来的 IRQ 用错块。
 */
static hwt101_status_t arm_dma(uint8_t idx, uint16_t size)
{
    g_rx_len = (size <= HWT101_RX_LEN) ? size : HWT101_RX_LEN;
    g_cached = 0U;
    g_rx_seq++;
    g_active = idx;
    g_started = 1U;
    if (HAL_UARTEx_ReceiveToIdle_DMA(&huart2, g_buf[idx],
                                   HWT101_RX_LEN) != HAL_OK) {
        __HAL_UART_DISABLE_IT(&huart2, UART_IT_IDLE);
        g_started = 0U;
        g_rx_len = 0U;
        return HWT101_ERR;
    }
    __HAL_DMA_DISABLE_IT(huart2.hdmarx, DMA_IT_HT);
    return HWT101_OK;
}

/**
 * @brief  扫描完整子帧，各量取本批最后一帧；尾部残帧不跨批拼接
 * @param  data 完成块首地址，len 已限制在 DMA 缓冲容量内
 * @param  len 有效字节数
 * @param  out 内部解析结果，调用方须在发布前复核接收序号
 */
static void parse_buf(const uint8_t *data, size_t len, hwt_sample_t *out)
{
    size_t pos = 0U;       /* 子帧候选位置 */
    int16_t yaw = 0;       /* 最后一帧原始 yaw */
    int16_t gyro = 0;      /* 最后一帧原始 Wz */
    uint8_t flags = 0U;    /* 本批有效量 */
    uint8_t id;            /* 候选帧类型 */
    uint8_t sum;           /* 模 256 校验和 */
    int16_t raw;           /* 小端有符号测量值 */
    const uint8_t *frame;  /* 已确认足够长的候选帧 */

    while ((len - pos) >= HWT101_FRAME_LEN) {
        frame = &data[pos];
        id = frame[1];
        if ((frame[0] != HWT101_FRAME_HEAD) ||
            ((id != HWT101_ID_GYRO) && (id != HWT101_ID_ANGLE))) {
            pos++;
            continue;
        }
        sum = (uint8_t)(frame[0] + frame[1] + frame[2] + frame[3] +
                        frame[4] + frame[5] + frame[6] + frame[7] +
                        frame[8] + frame[9]);
        if (sum != frame[HWT101_FRAME_LEN - 1U]) {
            pos++;
            continue;
        }
        raw = (int16_t)((uint16_t)frame[6] | ((uint16_t)frame[7] << 8U));
        if (id == HWT101_ID_ANGLE) {
            yaw = raw;
            flags |= HWT101_HAS_YAW;
        } else {
            gyro = raw;
            flags |= HWT101_HAS_GYRO;
        }
        pos += HWT101_FRAME_LEN;
    }
    out->yaw = (float)yaw * HWT101_YAW_SCALE;
    out->gyro = (float)gyro * HWT101_GYRO_SCALE;
    out->flags = flags;
}

/**
 * @brief  获取当前批次的原始值；命中缓存时不再读取 DMA 数据
 * @param  out 调用方私有快照，只有返回 OK 时可作为有效 yaw 使用
 * @return HWT101_OK / HWT101_ERR_INIT / HWT101_ERR_RES
 * @note   多任务可在锁外重复解析，但只能发布序号仍匹配的结果。
 */
static hwt101_status_t read_raw(hwt_sample_t *out)
{
    uint32_t irq;       /* 调用前的中断屏蔽状态 */
    uint32_t seq;       /* 扫描开始时的接收序号 */
    uint16_t len;       /* 完成块有效长度 */
    uint8_t idx;        /* 完成块下标 */

    irq = __get_PRIMASK();
    __disable_irq();
    if (g_inited == 0U) {
        __set_PRIMASK(irq);
        return HWT101_ERR_INIT;
    }
    if (g_cached != 0U) {
        *out = g_sample;
        __set_PRIMASK(irq);
    } else {
        seq = g_rx_seq;
        len = g_rx_len;
        idx = (uint8_t)(g_active ^ 1U);
        __set_PRIMASK(irq);
        parse_buf(g_buf[idx], len, out);
        irq = __get_PRIMASK();
        __disable_irq();
        /* 下标切换两次会复原，必须比较完整序号后才能发布。 */
        if (seq != g_rx_seq) {
            __set_PRIMASK(irq);
            return HWT101_ERR_RES;
        }
        g_sample = *out;
        g_cached = 1U;
        __set_PRIMASK(irq);
    }
    return ((out->flags & HWT101_HAS_YAW) != 0U) ?
           HWT101_OK : HWT101_ERR_RES;
}

/**
 * @brief  归一化已在 [-360, 360) 内的角度，至多一次修正
 * @param  angle 原始 yaw 与归一化偏置之和，或标定余角
 * @return [-180, 180) 内的角度
 */
static float wrap_yaw(float angle)
{
    if (angle >= 180.0f) {
        angle -= 360.0f;
    } else if (angle < -180.0f) {
        angle += 360.0f;
    }
    return angle;
}

/** @copydoc hwt101_adp_init */
hwt101_status_t hwt101_adp_init(void)
{
    uint32_t irq; /* 保存调用前中断状态 */

    if (huart2.hdmarx == NULL) {
        return HWT101_ERR_PARAM;
    }
    irq = __get_PRIMASK();
    __disable_irq();
    /* 接收活动中不能重置缓冲归属，调用方必须在启动前初始化。 */
    if (g_started != 0U) {
        __set_PRIMASK(irq);
        return HWT101_ERR;
    }
    g_active = 0U;
    g_rx_len = 0U;
    g_rx_seq++;
    g_cached = 0U;
    g_yaw_ofs = 0.0f;
    g_inited = 1U;
    __set_PRIMASK(irq);
    return HWT101_OK;
}

/** @copydoc hwt101_adp_start */
hwt101_status_t hwt101_adp_start(void)
{
    if (g_inited == 0U) {
        return HWT101_ERR_INIT;
    }
    if (g_started != 0U) {
        return HWT101_OK;
    }
    return arm_dma(0U, 0U);
}

/** @copydoc hwt101_adp_read */
hwt101_status_t hwt101_adp_read(float *gyro_dps, float *yaw_deg)
{
    hwt_sample_t sample; /* 本次任务私有的原始值快照 */
    hwt101_status_t ret; /* 读取状态 */

    if ((gyro_dps == NULL) || (yaw_deg == NULL)) {
        return HWT101_ERR_PARAM;
    }
    ret = read_raw(&sample);
    if (ret != HWT101_OK) {
        return ret;
    }
    *yaw_deg = wrap_yaw(sample.yaw + g_yaw_ofs);
    if ((sample.flags & HWT101_HAS_GYRO) != 0U) {
        *gyro_dps = sample.gyro;
    }
    return HWT101_OK;
}

/** @copydoc hwt101_adp_set_yaw */
hwt101_status_t hwt101_adp_set_yaw(float yaw_deg)
{
    hwt_sample_t sample; /* 当前批次的原始航向 */
    hwt101_status_t ret; /* 原始读取状态 */
    uint32_t irq;        /* 保存调用前中断状态 */
    float offset;        /* 待发布的软件偏置 */
    float mag;           /* 多圈偏置的绝对值 */
    float step;          /* 不超过 mag 的 360 度二次幂倍数 */

    if (!isfinite(yaw_deg)) {
        return HWT101_ERR_PARAM;
    }
    ret = read_raw(&sample);
    if (ret != HWT101_OK) {
        return ret;
    }
    offset = yaw_deg - sample.yaw;
    /* 二分消去整圈，有限浮点输入最多约 120 轮，不引入取余库。 */
    if ((offset >= 360.0f) || (offset < -360.0f)) {
        mag = fabsf(offset);
        step = 360.0f;
        while (step <= (mag * 0.5f)) {
            step *= 2.0f;
        }
        do {
            if (mag >= step) {
                mag -= step;
            }
            step *= 0.5f;
        } while (step >= 360.0f);
        offset = (offset < 0.0f) ? -mag : mag;
    }
    irq = __get_PRIMASK();
    __disable_irq();
    g_yaw_ofs = wrap_yaw(offset);
    __set_PRIMASK(irq);
    return HWT101_OK;
}

/** @copydoc hwt101_adp_write_reg */
hwt101_status_t hwt101_adp_write_reg(uint8_t reg, uint8_t lo, uint8_t hi)
{
    uint8_t frame[5] = {0xFFU, 0xAAU, reg, lo, hi}; /* 固定寄存器写帧 */
    HAL_StatusTypeDef ret; /* HAL 发送状态 */

    ret = HAL_UART_Transmit(&huart2, frame, (uint16_t)sizeof(frame),
                            HWT101_TX_TMO);
    if (ret == HAL_TIMEOUT) {
        return HWT101_ERR_TMO;
    }
    return (ret == HAL_OK) ? HWT101_OK : HWT101_ERR;
}

/** @copydoc hwt101_adp_boot_cfg */
hwt101_status_t hwt101_adp_boot_cfg(void)
{
    hwt101_status_t ret; /* 首个失败帧的结果 */

    /* 帧间等待由手册规定，失败后仍需等完再发下一帧以保证器件状态确定。 */
    ret = hwt101_adp_write_reg(HWT101_REG_UNLOCK, HWT101_UNLOCK_DL,
                               HWT101_UNLOCK_DH);
    osDelay(HWT101_UNLOCK_MS);
    if (ret == HWT101_OK) {
        ret = hwt101_adp_write_reg(HWT101_REG_CALIYAW, 0x00U, 0x00U);
    }
    osDelay(HWT101_ZERO_MS);
    if (ret == HWT101_OK) {
        ret = hwt101_adp_write_reg(HWT101_REG_SAVE, 0x00U, 0x00U);
    }
    return ret;
}

/** @copydoc hwt101_adp_rx_isr */
hwt101_status_t hwt101_adp_rx_isr(uint16_t size)
{
    if (g_inited == 0U) {
        return HWT101_ERR_INIT;
    }
    return arm_dma((uint8_t)(g_active ^ 1U), size);
}

/** @copydoc hwt101_adp_err_isr */
hwt101_status_t hwt101_adp_err_isr(void)
{
    if (g_inited == 0U) {
        return HWT101_ERR_INIT;
    }
    /* 错误批次不参与读取；下一次正常完成事件重新发布长度。 */
    return arm_dma(g_active, 0U);
}
