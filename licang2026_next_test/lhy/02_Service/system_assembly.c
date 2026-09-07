/**
 * @file    system_assembly.c
 * @brief   系统组装根：电机、陀螺仪与灰度传感器装配
 * @author  haoyu
 * @note    - 电机 Handler 固定使用 CMSIS-OS2，BSP 内部自建队列和线程
 *          - 灰度 Handler 按固定硬件直接使用 CMSIS-OS2 与 GPIO IDR
 *          - 装配顺序：lsensor → motor → hwt101 → 广播读握手放行
 *          - 上电握手：集齐四轮广播读回复才放行，杜绝裸跑，与顺序无关
 *          - 工程 tick = 1kHz，故 tick 数与 ms 一一对应
 */

#include "system_assembly.h"

#include "cmsis_os2.h"

#include "zdt.h"
#include "hwt101_adaption.h"
#include "lsensor/lsensor_handler.h"

/* ===== 调试日志：0=不编译进固件，1=经 RTT 输出 ===== */
#ifndef SYS_ASM_LOG_EN
#define SYS_ASM_LOG_EN   1
#endif
#if SYS_ASM_LOG_EN
#include "cat_log.h"
#define SYS_LOGI(fmt, ...)  LOGI("[sys] " fmt, ##__VA_ARGS__)
#define SYS_LOGW(fmt, ...)  LOGW("[sys] " fmt, ##__VA_ARGS__)
#define SYS_LOGE(fmt, ...)  LOGE("[sys] " fmt, ##__VA_ARGS__)
#else
#define SYS_LOGI(fmt, ...)  do {} while (0)
#define SYS_LOGW(fmt, ...)  do {} while (0)
#define SYS_LOGE(fmt, ...)  do {} while (0)
#endif

#define SYS_HANDSHAKE_POLL_MS 150U   /* 上电握手等齐超时 ms */
#define SYS_POS_FLAGS_ALL     ((1U << ZDT_ADP_MOTOR_NUM) - 1U) /* 四轮到位掩码 */
#define SYS_POS_FLAG(idx)     (1U << (idx))                    /* 单轮到位位 */

/* ---- 被注入的对象 ---- */
static motor_handler_t g_motor_handler;   /* 命令总线 handler */
static uint8_t         g_assembled = 0U;  /* 组装完成标志 */

/* ---- 上电握手 / 里程计同步：四轮到位事件标志（RX 钩子置位） ---- */
static osEventFlagsId_t g_pos_flags = NULL;            /* 每轮一位，广播读回复置位 */
static volatile uint8_t g_enabled[ZDT_ADP_MOTOR_NUM]; /* 收使能应答（诊断 + 重发收敛） */

/**
 * @brief  RX 蹦床：适配层 ISR 回调签名 → handler 入队接口
 * @param  motor 来源电机
 * @param  data  帧缓冲
 * @param  len   帧长度
 * @note   ISR 上下文，仅做一次非阻塞入队
 */
static void motor_rx_trampoline(zdt_motor_t *motor,
                                const uint8_t *data, uint16_t len)
{
    (void)mh_feed_rx(&g_motor_handler, motor, data, len);
}

/**
 * @brief  电机 RX 事件钩子：按帧类型置位就绪标志 + RTT 日志
 * @param  ctx 钩子上下文（本层未用）
 * @param  idx 来源电机槽位
 * @param  rx  解析结果（kind / code / pulse）
 * @note   RX 线程上下文，仅置标志 + 轻量日志，勿阻塞
 */
static void motor_evt_cb(void *ctx, uint8_t idx, const zdt_rx_t *rx)
{
    (void)ctx;
    // 0.空指针与越界保护
    if ((rx == NULL) || (idx >= ZDT_ADP_MOTOR_NUM)) {
        return;
    }
    // 1.按帧类型置位就绪标志（ACK 记使能、POS 置到位）/ 记录异常
    switch (rx->kind) {
    case ZDT_RX_ACK:
        g_enabled[idx] = 1U;    /* 命令应答：使能等已被电机接收 */
        break;
    case ZDT_RX_POS:
        /* 广播读回复：置位对应轮到位标志（握手 / 里程计线程等齐用） */
        (void)osEventFlagsSet(g_pos_flags, SYS_POS_FLAG(idx));
        break;
    case ZDT_RX_DONE:
        SYS_LOGI("m%u done code=0x%02X",
                 (unsigned)idx, (unsigned)rx->code);
        break;
    case ZDT_RX_PARAM_ERR:
    case ZDT_RX_FMT_ERR:
        SYS_LOGE("m%u motor err kind=%d",
                 (unsigned)idx, (int)rx->kind);
        break;
    default:
        break;
    }
}

zdt_status_t system_assembly_init(void)
{
    zdt_status_t ret = ZDT_ERR;  /* 步骤结果 */
    uint8_t      i = 0U;         /* 电机下标 */
    uint32_t     flags = 0U;     /* 等齐返回的标志位 */

    /* 0) 启动灰度传感器周期采样任务 */
    if (lsh_init() != LSH_OK) {
        SYS_LOGE("lsensor init fail");
        return ZDT_ERR_RES;
    }
    /* 1) 建四轮到位事件标志（RX 钩子置位，须先于起线程） */
    g_pos_flags = osEventFlagsNew(NULL);
    if (g_pos_flags == NULL) {
        SYS_LOGE("pos flags alloc fail");
        return ZDT_ERR_RES;
    }
    /* 2) 平台适配：建电机/组 + 注入 HAL 收发，RX 回调指向蹦床 */
    ret = zdt_adp_init(motor_rx_trampoline);
    if (ret != ZDT_OK) {
        SYS_LOGE("adp_init fail=%d", (int)ret);
        return ret;
    }
    /* 3) 实例化 handler：绑定固定四轮总线 */
    ret = mh_inst(&g_motor_handler, zdt_adp_group(),
                  NULL, NULL, NULL, NULL, MH_GAP_MS_MIN);
    if (ret != ZDT_OK) {
        SYS_LOGE("mh_inst fail=%d", (int)ret);
        return ret;
    }
    /* 4) 注册 RX 事件钩子：电机应答经它上抛并置位就绪标志 */
    ret = mh_on_event(&g_motor_handler, motor_evt_cb, NULL);
    if (ret != ZDT_OK) {
        SYS_LOGE("mh_on_event fail=%d", (int)ret);
        return ret;
    }
    /* 5) 起 TX/RX 线程与队列（钩子已就位） */
    ret = mh_start(&g_motor_handler);
    if (ret != ZDT_OK) {
        SYS_LOGE("mh_start fail=%d", (int)ret);
        return ret;
    }
    /* 6) 仅启动各路 RX（使能 / 上报作为 TX 经 handler 串行下发） */
    ret = zdt_adp_start();
    if (ret != ZDT_OK) {
        SYS_LOGE("adp_start fail=%d", (int)ret);
        return ret;
    }
    /* 7) 陀螺仪：装配 + 启动 DMA 接收（不阻塞，先于握手做完） */
    if (hwt101_adp_init() != HWT101_OK) {
        SYS_LOGE("hwt101 init fail");
        return ZDT_ERR;
    }
    if (hwt101_adp_start() != HWT101_OK) {
        SYS_LOGE("hwt101 start fail");
        return ZDT_ERR;
    }
    // /* 8) 关定时上报（兜住电机 flash 残留周期），改为里程计线程广播读采集 */
    // (void)mh_report(&g_motor_handler, 0U);
    /* 9) 上电握手：重发使能 + 广播读，集齐四轮回复才放行（§3.4）；
     *    电机晚于板子上电也会被纳管，闭环建立前不放行以杜绝裸跑
     */
    for (;;) {
        /* 对未应答的电机重发使能（幂等，晚上电也纳管） */
        for (i = 0U; i < ZDT_ADP_MOTOR_NUM; i++) {
            if (g_enabled[i] == 0U) {
                (void)mh_enable(&g_motor_handler, i, true);
            }
        }
        /* 清四位 → 一帧广播读 → 限时等齐四轮回复 */
        (void)osEventFlagsClear(g_pos_flags, SYS_POS_FLAGS_ALL);
        (void)mh_request_pos_all(&g_motor_handler);
        flags = osEventFlagsWait(g_pos_flags, SYS_POS_FLAGS_ALL,
                                 osFlagsWaitAll, SYS_HANDSHAKE_POLL_MS);
        if ((flags & osFlagsError) == 0U) {
            break; /* 四轮全部回复，闭环就绪，放行 */
        }
        SYS_LOGW("waiting handshake: pos=0x%X",
                 (unsigned)(osEventFlagsGet(g_pos_flags) & SYS_POS_FLAGS_ALL));
    }
    g_assembled = 1U;
    SYS_LOGI("all motors ready: %u motors (broadcast-read) + hwt101",
             (unsigned)ZDT_ADP_MOTOR_NUM);
    return ZDT_OK;
}

motor_handler_t *system_motor_handler(void)
{
    if (g_assembled == 0U) {
        return NULL;
    }
    return &g_motor_handler;
}

osEventFlagsId_t system_pos_flags(void)
{
    return g_pos_flags;
}
