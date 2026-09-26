/**
 * @file    zdt_motor_adaption.c
 * @brief   ZDT 电机平台适配层（STM32 HAL UART + DMA）
 * @author  haoyu
 * @note    - 命令统一走 huart3 TX 总线；电机1~4 回传各走 huart3/4/5/6 RX
 *          - 板级接线（地址/方向/UART）见 g_cfg 配置表
 *          - 仅 RX 在本层启动；使能/上报为 TX，经 handler 串行下发
 */

#include "zdt_motor_adaption.h"

#include "main.h"
#include "usart.h"

#include <string.h>

#define ZDT_ADP_TX_BUF_SIZE   128U   /* TX 稳定缓冲，≥最大多机帧 */
#define ZDT_ADP_RX_BUF_SIZE    36U   /* 每路 RX DMA 缓冲 */

/* 板级配置：每电机的回传 UART + 地址 + 默认方向（TX 统一 huart3） */
typedef struct {
    UART_HandleTypeDef *uart; /* 该电机回传 RX 所在 UART */
    uint8_t   addr;           /* 电机地址 */
    zdt_dir_t dir;            /* 默认正方向 */
} adp_cfg_t;

static const adp_cfg_t g_cfg[ZDT_ADP_MOTOR_NUM] = {
    { &huart3, 0x01U, ZDT_DIR_CW  },
    { &huart4, 0x02U, ZDT_DIR_CCW },
    { &huart5, 0x03U, ZDT_DIR_CW  },
    { &huart6, 0x04U, ZDT_DIR_CCW },
};

static zdt_motor_t g_motors[ZDT_ADP_MOTOR_NUM]; /* 电机实例 */
static zdt_group_t g_group;                     /* 电机组（总线） */
static zdt_rx_cb_t g_on_frame = NULL;           /* 收到一帧回调 */
static uint8_t     g_inited = 0U;               /* 是否已实例化 */

static uint8_t g_tx_buf[ZDT_ADP_TX_BUF_SIZE];   /* TX 稳定缓冲（DMA 源） */
/* 各路 RX 缓冲 */
static uint8_t g_rx_buf[ZDT_ADP_MOTOR_NUM][ZDT_ADP_RX_BUF_SIZE];

/* ---- 内部 port 实现 ---- */

/**
 * @brief  经 huart3 命令总线 DMA 发送一帧
 * @param  frame 帧缓冲
 * @param  size  帧长度
 * @retval ZDT_OK / ZDT_ERR_PARAM / ZDT_ERR_RES / ZDT_ERR
 */
static zdt_status_t adp_tx(const uint8_t *frame, uint16_t size)
{
    // 0.参数合法性检查：帧指针、长度与稳定缓冲容量
    if ((frame == NULL) || (size == 0U) || (size > ZDT_ADP_TX_BUF_SIZE)) {
        return ZDT_ERR_PARAM;
    }
    // 1.检查上一帧 DMA 是否已经发送完成
    /* 上一帧 DMA 未发完则丢弃本帧，避免拷贝破坏在飞数据 */
    if (huart3.gState != HAL_UART_STATE_READY) {
        return ZDT_ERR_RES;
    }
    // 2.拷贝到稳定缓冲并启动 DMA 发送
    (void)memcpy(g_tx_buf, frame, size); /* 拷到稳定缓冲，异步 DMA */
    if (HAL_UART_Transmit_DMA(&huart3, g_tx_buf, size) != HAL_OK) {
        return ZDT_ERR;
    }
    __HAL_DMA_DISABLE_IT(huart3.hdmatx, DMA_IT_HT); /* TX 不需要半传输中断 */
    return ZDT_OK;
}

/** @brief 单电机发送 port（电机参数无用，统一走命令总线） */
static zdt_status_t motor_send(zdt_motor_t *motor, const uint8_t *frame,
                               uint16_t size)
{
    (void)motor;
    return adp_tx(frame, size);
}

/** @brief 多机广播发送 port */
static zdt_status_t group_send(const uint8_t *frame, uint16_t size)
{
    return adp_tx(frame, size);
}

/**
 * @brief  重启某路 UART 的空闲中断 DMA 接收
 * @param  idx 电机下标
 * @retval ZDT_OK / ZDT_ERR_PARAM / ZDT_ERR
 */
static zdt_status_t restart_rx(uint8_t idx)
{
    UART_HandleTypeDef *u = NULL; /* 目标 UART */

    // 0.参数合法性检查：电机下标范围
    if (idx >= ZDT_ADP_MOTOR_NUM) {
        return ZDT_ERR_PARAM;
    }
    // 1.按电机配置重启空闲中断 DMA 接收
    u = g_cfg[idx].uart;
    if (HAL_UARTEx_ReceiveToIdle_DMA(u, g_rx_buf[idx], ZDT_ADP_RX_BUF_SIZE)
        != HAL_OK) {
        return ZDT_ERR;
    }
    __HAL_DMA_DISABLE_IT(u->hdmarx, DMA_IT_HT); /* 只在空闲/完成触发 */
    return ZDT_OK;
}

/** @brief 重启接收 port（按电机指针定位下标） */
static zdt_status_t motor_start_rx(zdt_motor_t *motor)
{
    uint8_t i = 0U; /* 槽位下标 */

    // 0.按电机对象查找接收通道
    for (i = 0U; i < ZDT_ADP_MOTOR_NUM; i++) {
        if (motor == &g_motors[i]) {
            return restart_rx(i);
        }
    }
    return ZDT_ERR_PARAM;
}

/**
 * @brief  实例化板载电机与电机组，注入 HAL 收发端口
 * @param  on_frame 收到一帧的回调，不可为空
 * @retval ZDT_OK / ZDT_ERR_PARAM / ZDT_ERR
 */
zdt_status_t zdt_adp_init(zdt_rx_cb_t on_frame)
{
    zdt_status_t ret = ZDT_ERR; /* 结果 */
    uint8_t i = 0U;            /* 槽位下标 */

    // 0.参数合法性检查：接收回调
    if (on_frame == NULL) {
        return ZDT_ERR_PARAM;
    }
    // 1.保存回调并初始化总线对象
    g_on_frame = on_frame;
    ret = zdt_group_inst(&g_group, group_send);
    if (ret != ZDT_OK) {
        return ret;
    }
    // 2.逐槽位实例化电机并绑定到总线
    for (i = 0U; i < ZDT_ADP_MOTOR_NUM; i++) {
        ret = zdt_motor_inst(&g_motors[i], g_cfg[i].addr, g_cfg[i].dir,
                             motor_send, motor_start_rx);
        if (ret != ZDT_OK) {
            return ret;
        }
        ret = zdt_group_bind(&g_group, i, &g_motors[i]);
        if (ret != ZDT_OK) {
            return ret;
        }
    }
    // 3.标记适配层已经完成初始化
    g_inited = 1U;
    return ZDT_OK;
}

/**
 * @brief  启动各电机 UART 接收（使能/上报为 TX，经 handler 串行下发）
 * @retval ZDT_OK / ZDT_ERR_INIT / ZDT_ERR
 */
zdt_status_t zdt_adp_start(void)
{
    zdt_status_t ret = ZDT_ERR; /* 结果 */
    uint8_t i = 0U;            /* 槽位下标 */

    // 0.检查适配层是否已经初始化
    if (g_inited == 0U) {
        return ZDT_ERR_INIT;
    }
    // 1.逐通道启动 DMA 接收
    /* 仅开各路 RX；使能/上报为 TX，由 handler 串行下发 */
    for (i = 0U; i < ZDT_ADP_MOTOR_NUM; i++) {
        ret = restart_rx(i);
        if (ret != ZDT_OK) {
            return ret;
        }
    }
    return ZDT_OK;
}

/**
 * @brief  取电机组句柄（用于组速度控制）
 * @retval 电机组指针；未初始化返回 NULL
 */
zdt_group_t* zdt_adp_group(void)
{
    if (g_inited == 0U) {
        return NULL;
    }
    return &g_group;
}

/**
 * @brief  取单电机句柄（用于读位置 / 设速度）
 * @param  idx 电机下标，0..ZDT_ADP_MOTOR_NUM-1
 * @retval 电机指针；越界或未初始化返回 NULL
 */
zdt_motor_t* zdt_adp_motor(uint8_t idx)
{
    if ((g_inited == 0U) || (idx >= ZDT_ADP_MOTOR_NUM)) {
        return NULL;
    }
    return &g_motors[idx];
}

/**
 * @brief  UART 接收完成 ISR 入口（由 it_dispatch 按句柄路由调用）
 * @param  idx  电机下标
 * @param  size 本次接收字节数
 * @retval ZDT_OK / ZDT_ERR_PARAM / ZDT_ERR_INIT
 */
zdt_status_t zdt_adp_rx_isr(uint8_t idx, uint16_t size)
{
    // 0.参数合法性检查：电机下标与初始化状态
    if (idx >= ZDT_ADP_MOTOR_NUM) {
        return ZDT_ERR_PARAM;
    }
    if (g_inited == 0U) {
        return ZDT_ERR_INIT;
    }
    // 1.上抛接收帧后重启 DMA 接收
    /* 先把帧上抛（回调内拷贝入队），再重启接收，避免拷贝期被 DMA 覆盖 */
    if (g_on_frame != NULL) {
        g_on_frame(&g_motors[idx], g_rx_buf[idx], size);
    }
    return restart_rx(idx);
}

/**
 * @brief  UART 错误 ISR 入口：重启该电机接收
 * @param  idx 电机下标
 * @retval ZDT_OK / ZDT_ERR_PARAM / ZDT_ERR_INIT
 */
zdt_status_t zdt_adp_err_isr(uint8_t idx)
{
    // 0.参数合法性检查：电机下标与初始化状态
    if (idx >= ZDT_ADP_MOTOR_NUM) {
        return ZDT_ERR_PARAM;
    }
    if (g_inited == 0U) {
        return ZDT_ERR_INIT;
    }
    // 1.错误后重启对应通道的 DMA 接收
    return restart_rx(idx);
}
