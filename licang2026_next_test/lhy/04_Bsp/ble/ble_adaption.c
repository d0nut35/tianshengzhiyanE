/**
 * @file    ble_adaption.c
 * @brief   BLE 平台适配层（STM32 HAL USART1 收发）
 * @author  haoyu
 * @note    - USART1 用 DMA 空闲接收；printf 按行缓冲发送
 *          - ISR 只拷帧上抛 + 重启；拆帧换算放任务侧 process
 *          - 协议量→物理量（deg→rad）在 on_frame 换算后上抛
 */

#include "ble_adaption.h"

#include <stdio.h>

#include "main.h"
#include "usart.h"

#define BLE_RX_BUF_SIZE  64U          /* USART1 DMA 接收缓冲字节数 */
#define BLE_TX_BUF_SIZE  96U          /* printf 单次缓冲字节数 */
#define BLE_TX_TIMEOUT   10U          /* USART1 阻塞发送超时 ms */
#define BLE_DEG2RAD      0.0174532925f /* 角度转弧度系数 */

static UART_HandleTypeDef *const g_uart = &huart1; /* BLE 收发 UART */

static ble_parser_t  g_parser;                  /* 解析器实例 */
static uint8_t       g_rx_buf[BLE_RX_BUF_SIZE]; /* DMA 接收缓冲 */
static uint8_t       g_tx_buf[BLE_TX_BUF_SIZE]; /* printf 行缓冲 */
static uint16_t      g_tx_len = 0U;             /* 当前已缓冲字节数 */
static ble_rx_cb_t   g_on_rx_raw = NULL;        /* 原始块回调 */
static ble_nav_out_t g_on_nav    = NULL;        /* 命令上抛回调 */
static uint8_t       g_inited    = 0U;          /* 是否已实例化 */

/* ---- 内部实现 ---- */

/**
 * @brief  启动 USART1 DMA 空闲接收并关半传输中断
 * @retval BLE_OK / BLE_ERR
 */
static ble_status_t start_rx(void)
{
    if (HAL_UARTEx_ReceiveToIdle_DMA(g_uart, g_rx_buf, BLE_RX_BUF_SIZE)
        != HAL_OK) {
        return BLE_ERR;
    }
    __HAL_DMA_DISABLE_IT(g_uart->hdmarx, DMA_IT_HT); /* 只在空闲/完成触发 */
    return BLE_OK;
}

/**
 * @brief  将 printf 缓冲行发送到 BLE USART1
 * @retval 0 或 EOF
 */
static int flush_tx(void)
{
    HAL_StatusTypeDef ret = HAL_OK; /* UART 发送结果 */

    if (g_tx_len == 0U) {
        return 0;
    }
    ret = HAL_UART_Transmit(g_uart, g_tx_buf, g_tx_len, BLE_TX_TIMEOUT);
    g_tx_len = 0U;
    return (ret == HAL_OK) ? 0 : EOF;
}

/**
 * @brief  解析器整帧回调：协议量换算为物理量后上抛
 * @param  ctx 未用
 * @param  nav 解析出的导航命令（原始量）
 */
static void on_frame(void *ctx, const ble_nav_t *nav)
{
    float yaw_rad = (float)nav->yaw_deg * BLE_DEG2RAD;   /* 航向 deg→rad */
    float w_rad   = (float)nav->w_max_deg * BLE_DEG2RAD; /* 角速度 deg→rad */

    (void)ctx;
    if (g_on_nav == NULL) {
        return;
    }
    g_on_nav(nav->mode, (float)nav->x_mm, (float)nav->y_mm, yaw_rad,
             (float)nav->v_max, w_rad);
}

/* ---- 对外接口 ---- */

/**
 * @brief  将标准输出重定向到 BLE USART1，按行批量发送
 * @param  ch     待输出字符
 * @param  stream 标准流，本实现未使用
 * @retval ch 或 EOF
 */
int fputc(int ch, FILE *stream)
{
    (void)stream;
    if (g_tx_len >= BLE_TX_BUF_SIZE) {
        if (flush_tx() == EOF) {
            return EOF;
        }
    }
    g_tx_buf[g_tx_len] = (uint8_t)ch;
    g_tx_len++;
    if ((ch == '\n') || (g_tx_len >= BLE_TX_BUF_SIZE)) {
        if (flush_tx() == EOF) {
            return EOF;
        }
    }
    return ch;
}

ble_status_t ble_adp_init(ble_rx_cb_t on_rx_raw, ble_nav_out_t on_nav)
{
    ble_status_t ret = BLE_ERR; /* 实例化 / 接收启动结果 */

    if ((on_rx_raw == NULL) || (on_nav == NULL)) {
        return BLE_ERR_PARAM;
    }
    ret = ble_inst(&g_parser, on_frame, NULL);
    if (ret != BLE_OK) {
        return ret;
    }
    g_on_rx_raw = on_rx_raw;
    g_on_nav    = on_nav;
    g_inited    = 1U;
    ret = start_rx();
    if (ret != BLE_OK) {
        g_inited = 0U; /* 接收没起来则回退未初始化 */
        return ret;
    }
    return BLE_OK;
}

ble_status_t ble_adp_rx_isr(uint16_t size)
{
    uint16_t n = size; /* 本次接收字节数（夹取到缓冲上限） */

    if (g_inited == 0U) {
        return BLE_ERR_INIT;
    }
    if (n > BLE_RX_BUF_SIZE) {
        n = BLE_RX_BUF_SIZE;
    }
    /* 先上抛（回调内拷贝入队），再重启接收，避免拷贝期被 DMA 覆盖 */
    if ((n > 0U) && (g_on_rx_raw != NULL)) {
        g_on_rx_raw(g_rx_buf, n);
    }
    return start_rx();
}

ble_status_t ble_adp_err_isr(void)
{
    if (g_inited == 0U) {
        return BLE_ERR_INIT;
    }
    return start_rx();
}

ble_status_t ble_adp_process(const uint8_t *data, uint16_t len)
{
    if (data == NULL) {
        return BLE_ERR_PARAM;
    }
    if (g_inited == 0U) {
        return BLE_ERR_INIT;
    }
    return ble_feed(&g_parser, data, len);
}
