/**
 * @file    zdt.c
 * @brief   固定四轮 ZDT 电机 BSP 与 handler 合并实现。
 * @note    - 命令总线固定 huart3 TX，四路回传固定 huart3/4/5/6 RX
 *          - 热路径直接填帧，不再经过通用 builder/对象/port 多层调用
 *          - RX 仍由中断入队、任务解析，保护 int64 位置读写
 */

#include "zdt.h"

#include "main.h"
#include "usart.h"

#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "task.h"

#include <stddef.h>
#include <string.h>

#define ZDT_CHK            0x6BU    /* 固定校验字节 */
#define ZDT_SYNC_NOW       0x00U    /* 立即执行 */
#define ZDT_SYNC_BUF       0x01U    /* 缓存后同步执行 */

#define ZDT_CODE_ENABLE    0xF3U    /* 使能 */
#define ZDT_CODE_SPEED     0xF6U    /* 速度 */
#define ZDT_CODE_POS       0xFDU    /* 位置 */
#define ZDT_CODE_HOME      0x9AU    /* 回零 */
#define ZDT_CODE_REPORT    0x11U    /* 定时上报 */
#define ZDT_CODE_MULTI     0xAAU    /* 多机命令 */
#define ZDT_CODE_POS_RPT   0x36U    /* 实时位置 */

#define ZDT_AST_ENABLE     0xABU    /* 使能辅助码 */
#define ZDT_AST_REPORT     0x18U    /* 上报辅助码 */

#define ZDT_RET_ACK        0x02U    /* 命令接收正确 */
#define ZDT_RET_HOME_IDLE  0x12U    /* 回零时已在零点 */
#define ZDT_RET_PARAM_ERR  0xE2U    /* 参数错误 */
#define ZDT_RET_FMT_ERR    0xEEU    /* 格式错误 */
#define ZDT_RET_DONE       0x9FU    /* 动作完成 */

#define ZDT_POS_LEN        8U       /* 位置返回帧长 */
#define ZDT_ACK_LEN        4U       /* 应答返回帧长 */
#define ZDT_MULTI_HEAD     4U       /* 00 AA len_hi len_lo */
#define ZDT_SUB_MIN        3U       /* 子帧最小长度 */

#define ZDT_TX_BUF_SIZE    128U     /* TX DMA 稳定缓冲 */
#define ZDT_RX_BUF_SIZE    36U      /* 单路 RX DMA 缓冲 */
#define ZDT_MULTI_MAX      48U      /* 四轮多机命令最大长度 */

#define ZDT_MH_STACK_BYTES 2048U    /* TX/RX 工作线程栈 */
#define MH_TX_QUEUE_DEPTH  8U       /* TX 请求队列深度 */
#define MH_RX_QUEUE_DEPTH  16U      /* RX 帧队列深度 */

typedef enum {
    MH_REQ_SPEED = 0,
    MH_REQ_ENABLE,
    MH_REQ_POS,
    MH_REQ_HOME,
    MH_REQ_REPORT,
    MH_REQ_READ_POS,
} mh_req_kind_t;

typedef struct {
    mh_req_kind_t kind; /* 请求类型 */
    union {
        struct {
            int16_t speed[ZDT_ADP_MOTOR_NUM]; /* 固定四轮速度 */
            uint8_t n;                         /* 参与电机数 */
            uint8_t accel;                     /* 加速度档 */
        } speed;
        struct {
            uint8_t idx; /* 电机下标 */
            uint8_t on;  /* 1=使能 */
        } enable;
        struct {
            uint8_t        idx;   /* 电机下标 */
            zdt_pos_mode_t mode;  /* 位置模式 */
            int32_t        pulse; /* 目标脉冲 */
            uint16_t       rpm;   /* 速度幅值 */
            uint8_t        accel; /* 加速度档 */
        } pos;
        struct {
            uint8_t         idx;  /* 电机下标 */
            zdt_home_mode_t mode; /* 回零模式 */
        } home;
        struct {
            uint16_t period_ms; /* 上报周期 */
        } report;
    } data;
} mh_tx_req_t;

typedef struct {
    zdt_motor_t *motor;              /* 来源电机 */
    uint16_t     len;                /* 帧长度 */
    uint8_t      data[ZDT_RX_BUF_SIZE]; /* 帧副本 */
} mh_rx_evt_t;

static zdt_motor_t g_motor[ZDT_ADP_MOTOR_NUM]; /* 固定四轮对象 */
static zdt_group_t g_group;                    /* 固定四轮总线 */
static zdt_rx_cb_t g_rx_cb = NULL;             /* ISR 收帧回调 */
static uint8_t     g_adp_ok = 0U;              /* 适配层初始化标志 */

static uint8_t g_tx_buf[ZDT_TX_BUF_SIZE];      /* TX DMA 稳定缓冲 */
static uint8_t g_rx_buf[ZDT_ADP_MOTOR_NUM][ZDT_RX_BUF_SIZE]; /* RX 缓冲 */

static UART_HandleTypeDef *const g_rx_uart[ZDT_ADP_MOTOR_NUM] = {
    &huart3,
    &huart4,
    &huart5,
    &huart6,
};

static const uint8_t g_addr[ZDT_ADP_MOTOR_NUM] = {
    0x01U,
    0x02U,
    0x03U,
    0x04U,
};

static const zdt_dir_t g_dir[ZDT_ADP_MOTOR_NUM] = {
    ZDT_DIR_CW,
    ZDT_DIR_CCW,
    ZDT_DIR_CW,
    ZDT_DIR_CCW,
};

static const osThreadAttr_t g_mh_attr = {
    .name       = "mh_worker",
    .stack_size = ZDT_MH_STACK_BYTES,
    .priority   = osPriorityNormal,
};

static void zdt_u16_be(uint8_t *buf, uint16_t val)
{
    buf[0] = (uint8_t)(val >> 8);
    buf[1] = (uint8_t)val;
}

static void zdt_u32_be(uint8_t *buf, uint32_t val)
{
    buf[0] = (uint8_t)(val >> 24);
    buf[1] = (uint8_t)(val >> 16);
    buf[2] = (uint8_t)(val >> 8);
    buf[3] = (uint8_t)val;
}

static uint32_t zdt_rd_u32(const uint8_t *buf)
{
    return ((uint32_t)buf[0] << 24) |
           ((uint32_t)buf[1] << 16) |
           ((uint32_t)buf[2] << 8)  |
           ((uint32_t)buf[3]);
}

static uint16_t zdt_abs_i16(int16_t val)
{
    int32_t tmp = (int32_t)val; /* 防止取负时按 16 位溢出 */

    if (tmp < 0) {
        tmp = -tmp;
    }
    return (uint16_t)tmp;
}

static zdt_dir_t zdt_pick_dir(zdt_dir_t base, uint8_t neg)
{
    if (neg == 0U) {
        return base;
    }
    return (base == ZDT_DIR_CW) ? ZDT_DIR_CCW : ZDT_DIR_CW;
}

static zdt_status_t zdt_check_buf(uint8_t *buf, uint16_t cap,
                                  uint8_t addr, uint16_t need,
                                  uint16_t *len)
{
    if ((buf == NULL) || (len == NULL) || (addr == 0U) || (cap < need)) {
        return ZDT_ERR_PARAM;
    }
    return ZDT_OK;
}

static zdt_status_t zdt_tx(const uint8_t *frame, uint16_t size)
{
    if ((frame == NULL) || (size == 0U) || (size > ZDT_TX_BUF_SIZE)) {
        return ZDT_ERR_PARAM;
    }
    if (huart3.gState != HAL_UART_STATE_READY) {
        return ZDT_ERR_RES;
    }
    (void)memcpy(g_tx_buf, frame, size);
    if (HAL_UART_Transmit_DMA(&huart3, g_tx_buf, size) != HAL_OK) {
        return ZDT_ERR;
    }
    __HAL_DMA_DISABLE_IT(huart3.hdmatx, DMA_IT_HT);
    return ZDT_OK;
}

static zdt_status_t motor_send(zdt_motor_t *motor, const uint8_t *frame,
                               uint16_t size)
{
    (void)motor;
    return zdt_tx(frame, size);
}

static zdt_status_t group_send(const uint8_t *frame, uint16_t size)
{
    return zdt_tx(frame, size);
}

static zdt_status_t zdt_rx_start(uint8_t idx)
{
    UART_HandleTypeDef *uart = NULL; /* 目标 UART */

    if (idx >= ZDT_ADP_MOTOR_NUM) {
        return ZDT_ERR_PARAM;
    }
    uart = g_rx_uart[idx];
    if (HAL_UARTEx_ReceiveToIdle_DMA(uart, g_rx_buf[idx],
                                     ZDT_RX_BUF_SIZE) != HAL_OK) {
        return ZDT_ERR;
    }
    __HAL_DMA_DISABLE_IT(uart->hdmarx, DMA_IT_HT);
    return ZDT_OK;
}

static uint8_t zdt_motor_idx(const zdt_motor_t *motor)
{
    uint8_t i = 0U; /* 固定四轮下标 */

    for (i = 0U; i < ZDT_ADP_MOTOR_NUM; i++) {
        if (motor == &g_motor[i]) {
            return i;
        }
    }
    return ZDT_ADP_MOTOR_NUM;
}

static zdt_status_t motor_start_rx(zdt_motor_t *motor)
{
    return zdt_rx_start(zdt_motor_idx(motor));
}

static zdt_motor_t *mh_motor_at(uint8_t idx)
{
    if ((g_adp_ok == 0U) || (idx >= ZDT_ADP_MOTOR_NUM)) {
        return NULL;
    }
    return &g_motor[idx];
}

static void zdt_add_speed(uint8_t *buf, uint16_t *pos, uint8_t addr,
                          zdt_dir_t dir, uint16_t rpm, uint8_t accel)
{
    uint16_t i = *pos; /* 当前写入位置 */

    buf[i++] = addr;
    buf[i++] = ZDT_CODE_SPEED;
    buf[i++] = (uint8_t)dir;
    buf[i++] = (uint8_t)(rpm >> 8);
    buf[i++] = (uint8_t)rpm;
    buf[i++] = accel;
    buf[i++] = ZDT_SYNC_BUF;
    buf[i++] = ZDT_CHK;
    *pos = i;
}

static zdt_status_t zdt_send_speed(const int16_t *speed, uint8_t n,
                                   uint8_t accel)
{
    uint8_t frame[ZDT_MULTI_MAX]; /* 一帧四轮速度命令 */
    uint16_t pos = ZDT_MULTI_HEAD; /* 多机帧当前写入位置 */
    uint16_t total = 0U;           /* 多机整帧长度 */
    uint8_t i = 0U;                /* 电机下标 */
    uint16_t rpm = 0U;             /* 当前电机速度幅值 */
    zdt_dir_t dir = ZDT_DIR_CW;    /* 当前电机协议方向 */

    if ((speed == NULL) || (n == 0U) || (n > ZDT_ADP_MOTOR_NUM)) {
        return ZDT_ERR_PARAM;
    }
    frame[0] = ZDT_ADDR_BROADCAST;
    frame[1] = ZDT_CODE_MULTI;
    frame[2] = 0U;
    frame[3] = 0U;
    for (i = 0U; i < n; i++) {
        if ((speed[i] > (int16_t)ZDT_RPM_MAX) ||
            (speed[i] < -(int16_t)ZDT_RPM_MAX)) {
            return ZDT_ERR_PARAM;
        }
        g_motor[i].speed = speed[i];
        g_motor[i].accel = accel;
        rpm = zdt_abs_i16(speed[i]);
        dir = zdt_pick_dir(g_motor[i].dir, speed[i] < 0);
        zdt_add_speed(frame, &pos, g_motor[i].addr, dir, rpm, accel);
    }
    frame[pos++] = ZDT_CHK;
    total = pos;
    frame[2] = (uint8_t)(total >> 8);
    frame[3] = (uint8_t)total;
    return zdt_tx(frame, total);
}

static zdt_status_t zdt_send_enable(uint8_t idx, bool on)
{
    uint8_t frame[6]; /* 使能/失能命令 */
    zdt_status_t ret; /* 发送结果 */

    if (idx >= ZDT_ADP_MOTOR_NUM) {
        return ZDT_ERR_PARAM;
    }
    frame[0] = g_motor[idx].addr;
    frame[1] = ZDT_CODE_ENABLE;
    frame[2] = ZDT_AST_ENABLE;
    frame[3] = on ? 1U : 0U;
    frame[4] = ZDT_SYNC_NOW;
    frame[5] = ZDT_CHK;
    ret = zdt_tx(frame, (uint16_t)sizeof(frame));
    if (ret != ZDT_OK) {
        return ret;
    }
    g_motor[idx].is_enabled = on ? 1U : 0U;
    return ZDT_OK;
}

static zdt_status_t zdt_send_pos(uint8_t idx, zdt_pos_mode_t mode,
                                 int32_t pulse, uint16_t rpm, uint8_t accel)
{
    uint8_t frame[13];       /* 位置模式命令 */
    uint32_t mag = 0U;       /* 脉冲幅值 */
    zdt_dir_t dir;           /* 协议方向 */

    if ((idx >= ZDT_ADP_MOTOR_NUM) || (rpm > ZDT_RPM_MAX) ||
        ((uint8_t)mode > (uint8_t)ZDT_POS_REL_NOW)) {
        return ZDT_ERR_PARAM;
    }
    mag = (pulse >= 0) ? (uint32_t)pulse : (uint32_t)(-(int64_t)pulse);
    dir = zdt_pick_dir(g_motor[idx].dir, pulse < 0);
    frame[0] = g_motor[idx].addr;
    frame[1] = ZDT_CODE_POS;
    frame[2] = (uint8_t)dir;
    zdt_u16_be(&frame[3], rpm);
    frame[5] = accel;
    zdt_u32_be(&frame[6], mag);
    frame[10] = (uint8_t)mode;
    frame[11] = ZDT_SYNC_NOW;
    frame[12] = ZDT_CHK;
    return zdt_tx(frame, (uint16_t)sizeof(frame));
}

static zdt_status_t zdt_send_home(uint8_t idx, zdt_home_mode_t mode)
{
    uint8_t frame[5]; /* 回零命令 */

    if ((idx >= ZDT_ADP_MOTOR_NUM) ||
        ((uint8_t)mode > (uint8_t)ZDT_HOME_PWRLOSS)) {
        return ZDT_ERR_PARAM;
    }
    frame[0] = g_motor[idx].addr;
    frame[1] = ZDT_CODE_HOME;
    frame[2] = (uint8_t)mode;
    frame[3] = ZDT_SYNC_NOW;
    frame[4] = ZDT_CHK;
    return zdt_tx(frame, (uint16_t)sizeof(frame));
}

static zdt_status_t zdt_send_report(uint16_t period_ms)
{
    uint8_t frame[ZDT_MULTI_MAX]; /* 四轮定时上报配置 */
    uint16_t pos = ZDT_MULTI_HEAD; /* 多机帧当前写入位置 */
    uint16_t total = 0U;           /* 多机整帧长度 */
    uint8_t i = 0U;                /* 电机下标 */
    zdt_status_t ret;              /* 发送结果 */

    frame[0] = ZDT_ADDR_BROADCAST;
    frame[1] = ZDT_CODE_MULTI;
    frame[2] = 0U;
    frame[3] = 0U;
    for (i = 0U; i < ZDT_ADP_MOTOR_NUM; i++) {
        frame[pos++] = g_motor[i].addr;
        frame[pos++] = ZDT_CODE_REPORT;
        frame[pos++] = ZDT_AST_REPORT;
        frame[pos++] = ZDT_CODE_POS_RPT;
        frame[pos++] = (uint8_t)(period_ms >> 8);
        frame[pos++] = (uint8_t)period_ms;
        frame[pos++] = ZDT_CHK;
    }
    frame[pos++] = ZDT_CHK;
    total = pos;
    frame[2] = (uint8_t)(total >> 8);
    frame[3] = (uint8_t)total;
    ret = zdt_tx(frame, total);
    if (ret != ZDT_OK) {
        return ret;
    }
    if (g_group.is_inited != 0U) {
        g_group.report_ms = period_ms;
    }
    return ZDT_OK;
}

static zdt_status_t zdt_send_read_pos(void)
{
    uint8_t frame[20];       /* 四轮实时位置读取 */
    uint16_t pos = ZDT_MULTI_HEAD; /* 多机帧当前写入位置 */
    uint16_t total = 0U;     /* 多机整帧长度 */
    uint8_t i = 0U;          /* 电机下标 */

    frame[0] = ZDT_ADDR_BROADCAST;
    frame[1] = ZDT_CODE_MULTI;
    frame[2] = 0U;
    frame[3] = 0U;
    for (i = 0U; i < ZDT_ADP_MOTOR_NUM; i++) {
        frame[pos++] = g_motor[i].addr;
        frame[pos++] = ZDT_CODE_POS_RPT;
        frame[pos++] = ZDT_CHK;
    }
    frame[pos++] = ZDT_CHK;
    total = pos;
    frame[2] = (uint8_t)(total >> 8);
    frame[3] = (uint8_t)total;
    return zdt_tx(frame, total);
}

zdt_status_t zdt_cmd_enable(uint8_t *buf, uint16_t cap, uint8_t addr,
                            bool on, uint16_t *len)
{
    if (zdt_check_buf(buf, cap, addr, 6U, len) != ZDT_OK) {
        return ZDT_ERR_PARAM;
    }
    buf[0] = addr;
    buf[1] = ZDT_CODE_ENABLE;
    buf[2] = ZDT_AST_ENABLE;
    buf[3] = on ? 1U : 0U;
    buf[4] = ZDT_SYNC_NOW;
    buf[5] = ZDT_CHK;
    *len = 6U;
    return ZDT_OK;
}

zdt_status_t zdt_cmd_speed(uint8_t *buf, uint16_t cap, uint8_t addr,
                           const zdt_speed_t *spd, uint16_t *len)
{
    if ((spd == NULL) || (spd->rpm > ZDT_RPM_MAX) ||
        (zdt_check_buf(buf, cap, addr, 8U, len) != ZDT_OK)) {
        return ZDT_ERR_PARAM;
    }
    buf[0] = addr;
    buf[1] = ZDT_CODE_SPEED;
    buf[2] = (uint8_t)spd->dir;
    zdt_u16_be(&buf[3], spd->rpm);
    buf[5] = spd->accel;
    buf[6] = ZDT_SYNC_NOW;
    buf[7] = ZDT_CHK;
    *len = 8U;
    return ZDT_OK;
}

zdt_status_t zdt_cmd_pos(uint8_t *buf, uint16_t cap, uint8_t addr,
                         const zdt_pos_t *pos, uint16_t *len)
{
    if ((pos == NULL) || (pos->rpm > ZDT_RPM_MAX) ||
        ((uint8_t)pos->mode > (uint8_t)ZDT_POS_REL_NOW) ||
        (zdt_check_buf(buf, cap, addr, 13U, len) != ZDT_OK)) {
        return ZDT_ERR_PARAM;
    }
    buf[0] = addr;
    buf[1] = ZDT_CODE_POS;
    buf[2] = (uint8_t)pos->dir;
    zdt_u16_be(&buf[3], pos->rpm);
    buf[5] = pos->accel;
    zdt_u32_be(&buf[6], pos->pulse);
    buf[10] = (uint8_t)pos->mode;
    buf[11] = ZDT_SYNC_NOW;
    buf[12] = ZDT_CHK;
    *len = 13U;
    return ZDT_OK;
}

zdt_status_t zdt_cmd_home(uint8_t *buf, uint16_t cap, uint8_t addr,
                          zdt_home_mode_t mode, uint16_t *len)
{
    if (((uint8_t)mode > (uint8_t)ZDT_HOME_PWRLOSS) ||
        (zdt_check_buf(buf, cap, addr, 5U, len) != ZDT_OK)) {
        return ZDT_ERR_PARAM;
    }
    buf[0] = addr;
    buf[1] = ZDT_CODE_HOME;
    buf[2] = (uint8_t)mode;
    buf[3] = ZDT_SYNC_NOW;
    buf[4] = ZDT_CHK;
    *len = 5U;
    return ZDT_OK;
}

zdt_status_t zdt_cmd_report(uint8_t *buf, uint16_t cap, uint8_t addr,
                            uint16_t period_ms, uint16_t *len)
{
    if (zdt_check_buf(buf, cap, addr, 7U, len) != ZDT_OK) {
        return ZDT_ERR_PARAM;
    }
    buf[0] = addr;
    buf[1] = ZDT_CODE_REPORT;
    buf[2] = ZDT_AST_REPORT;
    buf[3] = ZDT_CODE_POS_RPT;
    zdt_u16_be(&buf[4], period_ms);
    buf[6] = ZDT_CHK;
    *len = 7U;
    return ZDT_OK;
}

zdt_status_t zdt_cmd_read_pos(uint8_t *buf, uint16_t cap, uint8_t addr,
                              uint16_t *len)
{
    if (zdt_check_buf(buf, cap, addr, 3U, len) != ZDT_OK) {
        return ZDT_ERR_PARAM;
    }
    buf[0] = addr;
    buf[1] = ZDT_CODE_POS_RPT;
    buf[2] = ZDT_CHK;
    *len = 3U;
    return ZDT_OK;
}

zdt_status_t zdt_multi_init(zdt_multi_t *mb, uint8_t *buf, uint16_t cap)
{
    if ((mb == NULL) || (buf == NULL) || (cap < (ZDT_MULTI_HEAD + 1U))) {
        return ZDT_ERR_PARAM;
    }
    mb->buf = buf;
    mb->cap = cap;
    mb->len = ZDT_MULTI_HEAD;
    buf[0] = ZDT_ADDR_BROADCAST;
    buf[1] = ZDT_CODE_MULTI;
    buf[2] = 0U;
    buf[3] = 0U;
    return ZDT_OK;
}

zdt_status_t zdt_multi_add_raw(zdt_multi_t *mb, const uint8_t *sub,
                               uint16_t sub_len)
{
    if ((mb == NULL) || (mb->buf == NULL) || (sub == NULL)) {
        return ZDT_ERR_PARAM;
    }
    if ((sub_len < ZDT_SUB_MIN) || (sub[sub_len - 1U] != ZDT_CHK) ||
        (sub[0] == ZDT_ADDR_BROADCAST)) {
        return ZDT_ERR_PARAM;
    }
    if ((uint16_t)(mb->len + sub_len + 1U) > mb->cap) {
        return ZDT_ERR_RES;
    }
    (void)memcpy(&mb->buf[mb->len], sub, sub_len);
    mb->len = (uint16_t)(mb->len + sub_len);
    return ZDT_OK;
}

zdt_status_t zdt_multi_add_speed(zdt_multi_t *mb, uint8_t addr,
                                 const zdt_speed_t *spd)
{
    uint8_t sub[8];        /* 速度子帧 */
    uint16_t sub_len = 0U; /* 子帧长度 */

    if ((spd == NULL) || (spd->rpm > ZDT_RPM_MAX)) {
        return ZDT_ERR_PARAM;
    }
    if (zdt_cmd_speed(sub, (uint16_t)sizeof(sub), addr, spd,
                      &sub_len) != ZDT_OK) {
        return ZDT_ERR_PARAM;
    }
    sub[6] = ZDT_SYNC_BUF;
    return zdt_multi_add_raw(mb, sub, sub_len);
}

zdt_status_t zdt_multi_done(zdt_multi_t *mb, uint16_t *len)
{
    uint16_t total = 0U; /* 多机整帧长度 */

    if ((mb == NULL) || (mb->buf == NULL) || (len == NULL)) {
        return ZDT_ERR_PARAM;
    }
    if ((uint16_t)(mb->len + 1U) > mb->cap) {
        return ZDT_ERR_RES;
    }
    mb->buf[mb->len++] = ZDT_CHK;
    total = mb->len;
    mb->buf[2] = (uint8_t)(total >> 8);
    mb->buf[3] = (uint8_t)total;
    *len = total;
    return ZDT_OK;
}

zdt_status_t zdt_decode(const uint8_t *data, uint16_t len, zdt_rx_t *out)
{
    if ((data == NULL) || (out == NULL)) {
        return ZDT_ERR_PARAM;
    }
    out->kind = ZDT_RX_NONE;
    out->addr = 0U;
    out->code = 0U;
    out->pulse = 0;

    if ((len >= ZDT_POS_LEN) && (data[1] == ZDT_CODE_POS_RPT) &&
        (data[2] <= 1U) && (data[7] == ZDT_CHK)) {
        uint32_t mag = zdt_rd_u32(&data[3]); /* 位置脉冲幅值 */

        out->kind = ZDT_RX_POS;
        out->addr = data[0];
        out->code = ZDT_CODE_POS_RPT;
        out->pulse = (data[2] == 0U) ? (int64_t)mag : -(int64_t)mag;
        return ZDT_OK;
    }
    if ((len >= ZDT_ACK_LEN) && (data[3] == ZDT_CHK)) {
        out->addr = data[0];
        out->code = data[1];
        switch (data[2]) {
        case ZDT_RET_ACK:
            out->kind = ZDT_RX_ACK;
            return ZDT_OK;
        case ZDT_RET_HOME_IDLE:
            out->kind = ZDT_RX_HOME_IDLE;
            return ZDT_OK;
        case ZDT_RET_PARAM_ERR:
            out->kind = ZDT_RX_PARAM_ERR;
            return ZDT_OK;
        case ZDT_RET_FMT_ERR:
            out->kind = ZDT_RX_FMT_ERR;
            return ZDT_OK;
        case ZDT_RET_DONE:
            out->kind = ZDT_RX_DONE;
            return ZDT_OK;
        default:
            break;
        }
    }
    return ZDT_ERR;
}

zdt_status_t zdt_motor_inst(zdt_motor_t *motor, uint8_t addr, zdt_dir_t dir,
                            zdt_send_fn_t pf_send, zdt_rx_fn_t pf_start_rx)
{
    if ((motor == NULL) || (pf_send == NULL) ||
        (pf_start_rx == NULL) || (addr == 0U)) {
        return ZDT_ERR_PARAM;
    }
    (void)memset(motor, 0, sizeof(*motor));
    motor->is_inited = 1U;
    motor->addr = addr;
    motor->dir = dir;
    motor->pf_send = pf_send;
    motor->pf_start_rx = pf_start_rx;
    return ZDT_OK;
}

zdt_status_t zdt_motor_enable(zdt_motor_t *motor)
{
    uint8_t frame[6];      /* 使能命令 */
    uint16_t len = 0U;     /* 命令长度 */
    zdt_status_t ret;      /* 发送结果 */

    if (motor == NULL) {
        return ZDT_ERR_PARAM;
    }
    if (motor->is_inited == 0U) {
        return ZDT_ERR_INIT;
    }
    if (motor->pf_send == NULL) {
        return ZDT_ERR_RES;
    }
    ret = zdt_cmd_enable(frame, (uint16_t)sizeof(frame), motor->addr,
                         true, &len);
    if (ret == ZDT_OK) {
        ret = motor->pf_send(motor, frame, len);
    }
    if (ret == ZDT_OK) {
        motor->is_enabled = 1U;
    }
    return ret;
}

zdt_status_t zdt_motor_disable(zdt_motor_t *motor)
{
    uint8_t frame[6];      /* 失能命令 */
    uint16_t len = 0U;     /* 命令长度 */
    zdt_status_t ret;      /* 发送结果 */

    if (motor == NULL) {
        return ZDT_ERR_PARAM;
    }
    if (motor->is_inited == 0U) {
        return ZDT_ERR_INIT;
    }
    if (motor->pf_send == NULL) {
        return ZDT_ERR_RES;
    }
    ret = zdt_cmd_enable(frame, (uint16_t)sizeof(frame), motor->addr,
                         false, &len);
    if (ret == ZDT_OK) {
        ret = motor->pf_send(motor, frame, len);
    }
    if (ret == ZDT_OK) {
        motor->is_enabled = 0U;
    }
    return ret;
}

zdt_status_t zdt_motor_set_speed(zdt_motor_t *motor,
                                 int16_t speed, uint8_t accel)
{
    if (motor == NULL) {
        return ZDT_ERR_PARAM;
    }
    if (motor->is_inited == 0U) {
        return ZDT_ERR_INIT;
    }
    if ((speed > (int16_t)ZDT_RPM_MAX) ||
        (speed < -(int16_t)ZDT_RPM_MAX)) {
        return ZDT_ERR_PARAM;
    }
    motor->speed = speed;
    motor->accel = accel;
    return ZDT_OK;
}

zdt_status_t zdt_motor_run_speed(zdt_motor_t *motor,
                                 int16_t speed, uint8_t accel)
{
    uint8_t frame[8];      /* 速度命令 */
    uint16_t len = 0U;     /* 命令长度 */
    zdt_speed_t spd;       /* 协议速度 */
    zdt_status_t ret;      /* 操作结果 */

    ret = zdt_motor_set_speed(motor, speed, accel);
    if (ret != ZDT_OK) {
        return ret;
    }
    if (motor->pf_send == NULL) {
        return ZDT_ERR_RES;
    }
    spd.dir = zdt_pick_dir(motor->dir, speed < 0);
    spd.rpm = zdt_abs_i16(speed);
    spd.accel = accel;
    ret = zdt_cmd_speed(frame, (uint16_t)sizeof(frame),
                        motor->addr, &spd, &len);
    if (ret != ZDT_OK) {
        return ret;
    }
    return motor->pf_send(motor, frame, len);
}

zdt_status_t zdt_motor_run_pos(zdt_motor_t *motor, zdt_pos_mode_t mode,
                               int32_t pulse, uint16_t rpm, uint8_t accel)
{
    uint8_t frame[13];     /* 位置命令 */
    uint16_t len = 0U;     /* 命令长度 */
    zdt_pos_t pos;         /* 协议位置 */
    zdt_status_t ret;      /* 操作结果 */

    if (motor == NULL) {
        return ZDT_ERR_PARAM;
    }
    if (motor->is_inited == 0U) {
        return ZDT_ERR_INIT;
    }
    if (motor->pf_send == NULL) {
        return ZDT_ERR_RES;
    }
    pos.dir = zdt_pick_dir(motor->dir, pulse < 0);
    pos.rpm = rpm;
    pos.accel = accel;
    pos.pulse = (pulse >= 0) ?
        (uint32_t)pulse : (uint32_t)(-(int64_t)pulse);
    pos.mode = mode;
    ret = zdt_cmd_pos(frame, (uint16_t)sizeof(frame), motor->addr,
                      &pos, &len);
    if (ret != ZDT_OK) {
        return ret;
    }
    return motor->pf_send(motor, frame, len);
}

zdt_status_t zdt_motor_home(zdt_motor_t *motor, zdt_home_mode_t mode)
{
    uint8_t frame[5];      /* 回零命令 */
    uint16_t len = 0U;     /* 命令长度 */
    zdt_status_t ret;      /* 操作结果 */

    if (motor == NULL) {
        return ZDT_ERR_PARAM;
    }
    if (motor->is_inited == 0U) {
        return ZDT_ERR_INIT;
    }
    if (motor->pf_send == NULL) {
        return ZDT_ERR_RES;
    }
    ret = zdt_cmd_home(frame, (uint16_t)sizeof(frame), motor->addr,
                       mode, &len);
    if (ret != ZDT_OK) {
        return ret;
    }
    return motor->pf_send(motor, frame, len);
}

zdt_status_t zdt_motor_parse(zdt_motor_t *motor, const uint8_t *data,
                             uint16_t len, zdt_rx_t *out)
{
    uint16_t off = 0U; /* 扫描偏移 */

    if ((motor == NULL) || (data == NULL) || (out == NULL)) {
        return ZDT_ERR_PARAM;
    }
    if (motor->is_inited == 0U) {
        return ZDT_ERR_INIT;
    }
    out->kind = ZDT_RX_NONE;
    for (off = 0U; (off + ZDT_ACK_LEN) <= len; off++) {
        if (data[off] != motor->addr) {
            continue;
        }
        if ((zdt_decode(&data[off], (uint16_t)(len - off), out) == ZDT_OK) &&
            (out->addr == motor->addr)) {
            if (out->kind == ZDT_RX_POS) {
                if (motor->dir == ZDT_DIR_CCW) {
                    out->pulse = -out->pulse;
                }
                motor->pos_pulse = out->pulse;
            }
            return ZDT_OK;
        }
    }
    out->kind = ZDT_RX_NONE;
    return ZDT_ERR;
}

zdt_status_t zdt_group_inst(zdt_group_t *group, zdt_grp_send_fn_t pf_send)
{
    uint8_t i = 0U; /* 槽位下标 */

    if ((group == NULL) || (pf_send == NULL)) {
        return ZDT_ERR_PARAM;
    }
    (void)memset(group, 0, sizeof(*group));
    group->is_inited = 1U;
    group->pf_send = pf_send;
    for (i = 0U; i < ZDT_GROUP_MAX; i++) {
        group->motors[i] = NULL;
    }
    return ZDT_OK;
}

zdt_status_t zdt_group_bind(zdt_group_t *group, uint16_t slot,
                            zdt_motor_t *motor)
{
    if ((group == NULL) || (motor == NULL)) {
        return ZDT_ERR_PARAM;
    }
    if (group->is_inited == 0U) {
        return ZDT_ERR_INIT;
    }
    if ((slot >= ZDT_GROUP_MAX) || (motor->is_inited == 0U)) {
        return ZDT_ERR_PARAM;
    }
    if (group->motors[slot] == NULL) {
        group->count++;
    }
    group->motors[slot] = motor;
    return ZDT_OK;
}

zdt_status_t zdt_group_speed(zdt_group_t *group)
{
    int16_t speed[ZDT_ADP_MOTOR_NUM]; /* 固定四轮速度 */
    uint8_t i = 0U;                   /* 电机下标 */

    if ((group == NULL) || (group->is_inited == 0U)) {
        return (group == NULL) ? ZDT_ERR_PARAM : ZDT_ERR_INIT;
    }
    for (i = 0U; i < ZDT_ADP_MOTOR_NUM; i++) {
        if (group->motors[i] == NULL) {
            return ZDT_ERR_PARAM;
        }
        speed[i] = group->motors[i]->speed;
    }
    return zdt_send_speed(speed, ZDT_ADP_MOTOR_NUM,
                          group->motors[0]->accel);
}

zdt_status_t zdt_group_report(zdt_group_t *group, uint16_t period_ms)
{
    if ((group == NULL) || (group->is_inited == 0U)) {
        return (group == NULL) ? ZDT_ERR_PARAM : ZDT_ERR_INIT;
    }
    return zdt_send_report(period_ms);
}

zdt_status_t zdt_group_read_pos(zdt_group_t *group)
{
    if ((group == NULL) || (group->is_inited == 0U)) {
        return (group == NULL) ? ZDT_ERR_PARAM : ZDT_ERR_INIT;
    }
    return zdt_send_read_pos();
}

static void mh_tx_gate(motor_handler_t *h)
{
    uint32_t now = osKernelGetTickCount(); /* 当前 tick */
    uint32_t dt = now - h->last_tx_tick;   /* 间隔，兼容回绕 */

    if (dt < h->gap_ms) {
        (void)osDelay((uint32_t)h->gap_ms - dt);
    }
}

static zdt_status_t mh_exec_req(const mh_tx_req_t *req)
{
    if (req == NULL) {
        return ZDT_ERR_PARAM;
    }
    switch (req->kind) {
    case MH_REQ_SPEED:
        return zdt_send_speed(req->data.speed.speed,
                              req->data.speed.n,
                              req->data.speed.accel);
    case MH_REQ_ENABLE:
        return zdt_send_enable(req->data.enable.idx,
                               req->data.enable.on != 0U);
    case MH_REQ_POS:
        return zdt_send_pos(req->data.pos.idx,
                            req->data.pos.mode,
                            req->data.pos.pulse,
                            req->data.pos.rpm,
                            req->data.pos.accel);
    case MH_REQ_HOME:
        return zdt_send_home(req->data.home.idx,
                             req->data.home.mode);
    case MH_REQ_REPORT:
        return zdt_send_report(req->data.report.period_ms);
    case MH_REQ_READ_POS:
        return zdt_send_read_pos();
    default:
        return ZDT_ERR_PARAM;
    }
}

static void mh_tx_thread(void *arg)
{
    motor_handler_t *h = (motor_handler_t *)arg; /* handler 对象 */
    mh_tx_req_t req;                             /* TX 请求 */

    for (;;) {
        if (osMessageQueueGet((osMessageQueueId_t)h->tx_queue, &req,
                              NULL, osWaitForever) != osOK) {
            continue;
        }
        mh_tx_gate(h);
        (void)mh_exec_req(&req);
        h->last_tx_tick = osKernelGetTickCount();
    }
}

static void mh_rx_thread(void *arg)
{
    motor_handler_t *h = (motor_handler_t *)arg; /* handler 对象 */
    mh_rx_evt_t evt;                             /* RX 帧 */
    zdt_rx_t rx;                                 /* 解析结果 */
    uint8_t idx = 0U;                            /* 电机下标 */

    for (;;) {
        if (osMessageQueueGet((osMessageQueueId_t)h->rx_queue, &evt,
                              NULL, osWaitForever) != osOK) {
            continue;
        }
        idx = zdt_motor_idx(evt.motor);
        if (idx >= ZDT_ADP_MOTOR_NUM) {
            continue;
        }
        taskENTER_CRITICAL();
        (void)zdt_motor_parse(evt.motor, evt.data, evt.len, &rx);
        taskEXIT_CRITICAL();
        if ((h->pf_on_evt != NULL) && (rx.kind != ZDT_RX_NONE)) {
            h->pf_on_evt(h->evt_ctx, idx, &rx);
        }
    }
}

zdt_status_t mh_inst(motor_handler_t *h, zdt_group_t *bus,
                     const mh_os_thread_t *thread,
                     const mh_os_queue_t *queue,
                     const mh_os_time_t *time,
                     const mh_os_lock_t *lock,
                     uint16_t gap_ms)
{
    (void)thread;
    (void)queue;
    (void)time;
    (void)lock;

    if ((h == NULL) || (bus == NULL)) {
        return ZDT_ERR_PARAM;
    }
    (void)memset(h, 0, sizeof(*h));
    h->is_inited = 1U;
    h->gap_ms = (gap_ms < MH_GAP_MS_MIN) ? MH_GAP_MS_MIN : gap_ms;
    h->bus = bus;
    return ZDT_OK;
}

zdt_status_t mh_on_event(motor_handler_t *h, mh_evt_cb_t pf, void *ctx)
{
    if (h == NULL) {
        return ZDT_ERR_PARAM;
    }
    if (h->is_inited == 0U) {
        return ZDT_ERR_INIT;
    }
    h->pf_on_evt = pf;
    h->evt_ctx = ctx;
    return ZDT_OK;
}

zdt_status_t mh_start(motor_handler_t *h)
{
    if (h == NULL) {
        return ZDT_ERR_PARAM;
    }
    if (h->is_inited == 0U) {
        return ZDT_ERR_INIT;
    }
    if (h->is_started != 0U) {
        return ZDT_OK;
    }
    h->tx_queue = osMessageQueueNew(MH_TX_QUEUE_DEPTH,
                                    (uint32_t)sizeof(mh_tx_req_t), NULL);
    if (h->tx_queue == NULL) {
        return ZDT_ERR_RES;
    }
    h->rx_queue = osMessageQueueNew(MH_RX_QUEUE_DEPTH,
                                    (uint32_t)sizeof(mh_rx_evt_t), NULL);
    if (h->rx_queue == NULL) {
        return ZDT_ERR_RES;
    }
    h->tx_thread = osThreadNew((osThreadFunc_t)mh_tx_thread, h, &g_mh_attr);
    if (h->tx_thread == NULL) {
        return ZDT_ERR_RES;
    }
    h->rx_thread = osThreadNew((osThreadFunc_t)mh_rx_thread, h, &g_mh_attr);
    if (h->rx_thread == NULL) {
        return ZDT_ERR_RES;
    }
    h->is_started = 1U;
    return ZDT_OK;
}

zdt_status_t mh_speed(motor_handler_t *h, const int16_t *speed,
                      uint8_t n, uint8_t accel)
{
    mh_tx_req_t req; /* 速度请求 */
    uint8_t i = 0U;  /* 拷贝下标 */

    if ((h == NULL) || (speed == NULL)) {
        return ZDT_ERR_PARAM;
    }
    if (h->is_started == 0U) {
        return ZDT_ERR_INIT;
    }
    if ((n == 0U) || (n > ZDT_ADP_MOTOR_NUM)) {
        return ZDT_ERR_PARAM;
    }
    req.kind = MH_REQ_SPEED;
    req.data.speed.n = n;
    req.data.speed.accel = accel;
    for (i = 0U; i < n; i++) {
        req.data.speed.speed[i] = speed[i];
    }
    if (osMessageQueuePut((osMessageQueueId_t)h->tx_queue, &req,
                          0U, 0U) != osOK) {
        return ZDT_ERR_RES;
    }
    return ZDT_OK;
}

zdt_status_t mh_enable(motor_handler_t *h, uint8_t idx, bool on)
{
    mh_tx_req_t req; /* 使能请求 */

    if (h == NULL) {
        return ZDT_ERR_PARAM;
    }
    if (h->is_started == 0U) {
        return ZDT_ERR_INIT;
    }
    if (idx >= ZDT_ADP_MOTOR_NUM) {
        return ZDT_ERR_PARAM;
    }
    req.kind = MH_REQ_ENABLE;
    req.data.enable.idx = idx;
    req.data.enable.on = on ? 1U : 0U;
    if (osMessageQueuePut((osMessageQueueId_t)h->tx_queue, &req,
                          0U, 0U) != osOK) {
        return ZDT_ERR_RES;
    }
    return ZDT_OK;
}

zdt_status_t mh_pos(motor_handler_t *h, uint8_t idx,
                    zdt_pos_mode_t mode, int32_t pulse,
                    uint16_t rpm, uint8_t accel)
{
    mh_tx_req_t req; /* 位置请求 */

    if (h == NULL) {
        return ZDT_ERR_PARAM;
    }
    if (h->is_started == 0U) {
        return ZDT_ERR_INIT;
    }
    if (idx >= ZDT_ADP_MOTOR_NUM) {
        return ZDT_ERR_PARAM;
    }
    req.kind = MH_REQ_POS;
    req.data.pos.idx = idx;
    req.data.pos.mode = mode;
    req.data.pos.pulse = pulse;
    req.data.pos.rpm = rpm;
    req.data.pos.accel = accel;
    if (osMessageQueuePut((osMessageQueueId_t)h->tx_queue, &req,
                          0U, 0U) != osOK) {
        return ZDT_ERR_RES;
    }
    return ZDT_OK;
}

zdt_status_t mh_home(motor_handler_t *h, uint8_t idx,
                     zdt_home_mode_t mode)
{
    mh_tx_req_t req; /* 回零请求 */

    if (h == NULL) {
        return ZDT_ERR_PARAM;
    }
    if (h->is_started == 0U) {
        return ZDT_ERR_INIT;
    }
    if (idx >= ZDT_ADP_MOTOR_NUM) {
        return ZDT_ERR_PARAM;
    }
    req.kind = MH_REQ_HOME;
    req.data.home.idx = idx;
    req.data.home.mode = mode;
    if (osMessageQueuePut((osMessageQueueId_t)h->tx_queue, &req,
                          0U, 0U) != osOK) {
        return ZDT_ERR_RES;
    }
    return ZDT_OK;
}

zdt_status_t mh_report(motor_handler_t *h, uint16_t period_ms)
{
    mh_tx_req_t req; /* 上报请求 */

    if (h == NULL) {
        return ZDT_ERR_PARAM;
    }
    if (h->is_started == 0U) {
        return ZDT_ERR_INIT;
    }
    req.kind = MH_REQ_REPORT;
    req.data.report.period_ms = period_ms;
    if (osMessageQueuePut((osMessageQueueId_t)h->tx_queue, &req,
                          0U, 0U) != osOK) {
        return ZDT_ERR_RES;
    }
    return ZDT_OK;
}

zdt_status_t mh_request_pos_all(motor_handler_t *h)
{
    mh_tx_req_t req; /* 读位置请求 */

    if (h == NULL) {
        return ZDT_ERR_PARAM;
    }
    if (h->is_started == 0U) {
        return ZDT_ERR_INIT;
    }
    req.kind = MH_REQ_READ_POS;
    if (osMessageQueuePut((osMessageQueueId_t)h->tx_queue, &req,
                          0U, 0U) != osOK) {
        return ZDT_ERR_RES;
    }
    return ZDT_OK;
}

zdt_status_t mh_get_pos(motor_handler_t *h, uint8_t idx, int64_t *out)
{
    zdt_motor_t *motor = NULL; /* 目标电机 */

    if ((h == NULL) || (out == NULL)) {
        return ZDT_ERR_PARAM;
    }
    if (h->is_inited == 0U) {
        return ZDT_ERR_INIT;
    }
    motor = mh_motor_at(idx);
    if (motor == NULL) {
        return ZDT_ERR_PARAM;
    }
    taskENTER_CRITICAL();
    *out = motor->pos_pulse;
    taskEXIT_CRITICAL();
    return ZDT_OK;
}

zdt_status_t mh_feed_rx(motor_handler_t *h, zdt_motor_t *motor,
                        const uint8_t *data, uint16_t len)
{
    mh_rx_evt_t evt;   /* 入队 RX 帧 */
    uint16_t cnt = 0U; /* 实际拷贝长度 */

    if ((h == NULL) || (motor == NULL) || (data == NULL) || (len == 0U)) {
        return ZDT_ERR_PARAM;
    }
    if (h->is_started == 0U) {
        return ZDT_ERR_INIT;
    }
    if (zdt_motor_idx(motor) >= ZDT_ADP_MOTOR_NUM) {
        return ZDT_ERR_PARAM;
    }
    cnt = (len > ZDT_RX_BUF_SIZE) ? ZDT_RX_BUF_SIZE : len;
    evt.motor = motor;
    evt.len = cnt;
    (void)memcpy(evt.data, data, cnt);
    if (osMessageQueuePut((osMessageQueueId_t)h->rx_queue, &evt,
                          0U, 0U) != osOK) {
        return ZDT_ERR_RES;
    }
    return ZDT_OK;
}

zdt_status_t zdt_adp_init(zdt_rx_cb_t on_frame)
{
    uint8_t i = 0U; /* 电机下标 */

    if (on_frame == NULL) {
        return ZDT_ERR_PARAM;
    }
    g_rx_cb = on_frame;
    (void)memset(&g_group, 0, sizeof(g_group));
    g_group.is_inited = 1U;
    g_group.count = ZDT_ADP_MOTOR_NUM;
    g_group.pf_send = group_send;
    for (i = 0U; i < ZDT_ADP_MOTOR_NUM; i++) {
        (void)memset(&g_motor[i], 0, sizeof(g_motor[i]));
        g_motor[i].is_inited = 1U;
        g_motor[i].addr = g_addr[i];
        g_motor[i].dir = g_dir[i];
        g_motor[i].pf_send = motor_send;
        g_motor[i].pf_start_rx = motor_start_rx;
        g_group.motors[i] = &g_motor[i];
    }
    g_adp_ok = 1U;
    return ZDT_OK;
}

zdt_status_t zdt_adp_start(void)
{
    zdt_status_t ret; /* 启动结果 */
    uint8_t i = 0U;   /* 电机下标 */

    if (g_adp_ok == 0U) {
        return ZDT_ERR_INIT;
    }
    for (i = 0U; i < ZDT_ADP_MOTOR_NUM; i++) {
        ret = zdt_rx_start(i);
        if (ret != ZDT_OK) {
            return ret;
        }
    }
    return ZDT_OK;
}

zdt_group_t *zdt_adp_group(void)
{
    if (g_adp_ok == 0U) {
        return NULL;
    }
    return &g_group;
}

zdt_motor_t *zdt_adp_motor(uint8_t idx)
{
    return mh_motor_at(idx);
}

zdt_status_t zdt_adp_rx_isr(uint8_t idx, uint16_t size)
{
    uint16_t len = size; /* 本次有效长度 */

    if (idx >= ZDT_ADP_MOTOR_NUM) {
        return ZDT_ERR_PARAM;
    }
    if (g_adp_ok == 0U) {
        return ZDT_ERR_INIT;
    }
    if (len > ZDT_RX_BUF_SIZE) {
        len = ZDT_RX_BUF_SIZE;
    }
    if ((g_rx_cb != NULL) && (len != 0U)) {
        g_rx_cb(&g_motor[idx], g_rx_buf[idx], len);
    }
    return zdt_rx_start(idx);
}

zdt_status_t zdt_adp_err_isr(uint8_t idx)
{
    if (idx >= ZDT_ADP_MOTOR_NUM) {
        return ZDT_ERR_PARAM;
    }
    if (g_adp_ok == 0U) {
        return ZDT_ERR_INIT;
    }
    return zdt_rx_start(idx);
}
