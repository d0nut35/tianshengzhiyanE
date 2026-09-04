/**
 * @file    hwt101_adaption.c
 * @brief   HWT101 平台适配层（STM32 HAL UART2 + DMA 双缓冲，懒扫描）
 * @author  haoyu
 * @note    - 固定 huart2；ReceiveToIdle + ping-pong 双缓冲，
 *            关半传输中断
 *          - rx_isr(ISR) 只记长度 + 切缓冲 + 重启 DMA，
 *            不解析（最短中断）
 *          - read(任务) 扫描最近收完缓冲调 hwt101_parse；
 *            扫描期又收完则丢弃本次
 */

#include "hwt101_adaption.h"

#include "main.h"
#include "usart.h"

#define HWT101_RX_LEN   64U    /* 每块 DMA 接收缓冲字节数 */
#define HWT101_TX_TMO   100U   /* 寄存器写阻塞发送超时 ms */

/* HWT101 所在 UART（板级固定 huart2） */
static UART_HandleTypeDef *g_uart = &huart2;
static uint8_t  g_buf[2][HWT101_RX_LEN];      /* ping-pong 双缓冲 */
/* 每块最近收完有效长度(ISR写/任务读) */
static volatile uint16_t g_len[2] = {0U, 0U};
/* DMA 正写入的缓冲下标(ISR写/任务读) */
static volatile uint8_t  g_active = 0U;
static uint8_t g_inited = 0U;                 /* 是否已实例化 */
static uint8_t g_started = 0U;                /* 是否已启动接收 */
static float   g_yaw_ofs = 0.0f;              /* 软件航向偏置，deg */

/**
 * @brief  在指定缓冲上重启 ReceiveToIdle DMA，并关半传输中断
 * @param  idx 目标缓冲下标，0/1
 * @retval HWT101_OK / HWT101_ERR
 */
static hwt101_status_t arm_dma(uint8_t idx)
{
    if (HAL_UARTEx_ReceiveToIdle_DMA(g_uart,
                                     g_buf[idx],
                                     HWT101_RX_LEN) != HAL_OK) {
        __HAL_UART_DISABLE_IT(g_uart, UART_IT_IDLE);
        return HWT101_ERR;
    }
    /* 只在空闲/完成触发 */
    __HAL_DMA_DISABLE_IT(g_uart->hdmarx, DMA_IT_HT);
    /* 标记 DMA 当前写入块 */
    g_active = idx;
    return HWT101_OK;
}

/**
 * @brief  将角度归一化到 [-180, 180)
 * @param  angle 待归一化角度，deg
 * @retval 归一化角度，deg
 */
static float norm_deg(float angle)
{
    while (angle >= 180.0f) {
        angle -= 360.0f;
    }
    while (angle < -180.0f) {
        angle += 360.0f;
    }
    return angle;
}

hwt101_status_t hwt101_adp_init(void)
{
    g_uart = &huart2;
    /* 校验 UART 及其 RX DMA 绑定 */
    if ((g_uart == NULL) || (g_uart->hdmarx == NULL)) {
        return HWT101_ERR_PARAM;
    }
    g_active = 0U;
    g_len[0] = 0U;
    g_len[1] = 0U;
    g_yaw_ofs = 0.0f;
    g_started = 0U;
    g_inited = 1U;
    return HWT101_OK;
}

hwt101_status_t hwt101_adp_start(void)
{
    hwt101_status_t ret;     /* 启动结果 */

    if (g_inited == 0U) {
        return HWT101_ERR_INIT;
    }
    if (g_started == 1U) {
        return HWT101_OK;
    }
    /* 从干净状态起：清长度，从缓冲 0 开始接收 */
    g_active = 0U;
    g_len[0] = 0U;
    g_len[1] = 0U;
    ret = arm_dma(0U);
    if (ret != HWT101_OK) {
        return ret;
    }
    g_started = 1U;
    return HWT101_OK;
}

/**
 * @brief  懒扫描最近收完缓冲，解出原始（未加偏置）角速度 / 角度
 * @param  gyro_dps 输出角速度 Wz，度/秒（本段存在才写）
 * @param  yaw_deg  输出原始偏航角，度
 * @retval 同 hwt101_adp_read
 */
static hwt101_status_t read_raw(float *gyro_dps, float *yaw_deg)
{
    hwt101_sample_t s;      /* 本次解析结果 */
    uint8_t  finished;      /* 最近收完的缓冲下标 */
    uint16_t len;           /* 该缓冲有效长度 */

    if ((gyro_dps == NULL) || (yaw_deg == NULL)) {
        return HWT101_ERR_PARAM;
    }
    if (g_inited == 0U) {
        return HWT101_ERR_INIT;
    }
    /* DMA 正写 g_active，最近收完的是另一块 */
    finished = (uint8_t)(g_active ^ 1U);
    len = g_len[finished];
    if (len > HWT101_RX_LEN) {
        len = HWT101_RX_LEN;
    }
    if (hwt101_parse(g_buf[finished], len, &s) != HWT101_OK) {
        return HWT101_ERR_RES;          /* 本段无任何有效帧 */
    }
    /* 扫描期间又收完一块，DMA 可能已回头覆盖该块，丢弃本次 */
    if ((uint8_t)(g_active ^ 1U) != finished) {
        return HWT101_ERR_RES;
    }
    if (s.has_yaw == 0U) {
        /* 以 yaw 为准，无新 yaw 即无新数据 */
        return HWT101_ERR_RES;
    }
    *yaw_deg = s.yaw_deg;
    if (s.has_gyro != 0U) {
        *gyro_dps = s.gyro_dps;         /* gyro 附带量，存在才写 */
    }
    return HWT101_OK;
}

hwt101_status_t hwt101_adp_read(float *gyro_dps, float *yaw_deg)
{
    hwt101_status_t ret;    /* 原始读取结果 */

    ret = read_raw(gyro_dps, yaw_deg);
    if (ret == HWT101_OK) {
        *yaw_deg = norm_deg(*yaw_deg + g_yaw_ofs);
    }
    return ret;
}

hwt101_status_t hwt101_adp_set_yaw(float yaw_deg)
{
    float gyro = 0.0f;      /* 附带角速度，弃用 */
    float raw = 0.0f;       /* 当前原始 yaw，deg */
    hwt101_status_t ret;    /* 原始读取结果 */

    ret = read_raw(&gyro, &raw);
    if (ret != HWT101_OK) {
        return ret;
    }
    g_yaw_ofs = norm_deg(yaw_deg - raw);
    return HWT101_OK;
}

hwt101_status_t hwt101_adp_write_reg(uint8_t reg, uint8_t lo, uint8_t hi)
{
    uint8_t frame[5];           /* FF AA reg lo hi */
    HAL_StatusTypeDef ret;      /* HAL 发送结果 */

    frame[0] = 0xFFU;
    frame[1] = 0xAAU;
    frame[2] = reg;
    frame[3] = lo;
    frame[4] = hi;
    ret = HAL_UART_Transmit(g_uart, frame, (uint16_t)sizeof(frame),
                            HWT101_TX_TMO);
    if (ret == HAL_TIMEOUT) {
        return HWT101_ERR_TMO;
    }
    if (ret != HAL_OK) {
        return HWT101_ERR;
    }
    return HWT101_OK;
}

hwt101_status_t hwt101_adp_rx_isr(uint16_t size)
{
    uint8_t finished;       /* 刚收完的缓冲下标 */
    uint8_t next;           /* 下一块写入缓冲下标 */

    if (g_inited == 0U) {
        return HWT101_ERR_INIT;
    }
    /* 当前 DMA 写的就是刚收完的块，记录其有效长度 */
    finished = g_active;
    g_len[finished] = (size <= HWT101_RX_LEN) ? size : HWT101_RX_LEN;
    /* 切到另一块继续收，形成 ping-pong（arm_dma 内更新 g_active） */
    next = (uint8_t)(finished ^ 1U);
    return arm_dma(next);
}

hwt101_status_t hwt101_adp_err_isr(void)
{
    if (g_inited == 0U) {
        return HWT101_ERR_INIT;
    }
    /* 在当前活动缓冲上直接重启接收（沿用原版错误恢复方式） */
    return arm_dma(g_active);
}
