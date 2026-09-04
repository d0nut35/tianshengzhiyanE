/**
 * @file    system_assembly.c
 * @brief   系统组装根：电机、陀螺仪与灰度传感器装配
 * @author  haoyu
 * @note    - 电机 Handler 的 OS 能力在本层注入
 *          - 灰度 Handler 按固定硬件直接使用 CMSIS-OS2 与 GPIO IDR
 *          - 装配顺序：lsensor → motor → hwt101 → 广播读握手放行
 *          - 上电握手：集齐四轮广播读回复才放行，杜绝裸跑，与顺序无关
 *          - 工程 tick = 1kHz，故 tick 数与 ms 一一对应
 */

#include "system_assembly.h"

#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "task.h"

#include "zdt_motor_adaption.h"
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

#define SYS_MH_STACK_BYTES    2048U  /* 收发线程栈，含日志 */
#define SYS_HANDSHAKE_POLL_MS 150U   /* 上电握手等齐超时 ms */
#define SYS_POS_FLAGS_ALL     ((1U << ZDT_ADP_MOTOR_NUM) - 1U) /* 四轮到位掩码 */
#define SYS_POS_FLAG(idx)     (1U << (idx))                    /* 单轮到位位 */

/* ---- 被注入的对象 ---- */
static motor_handler_t g_motor_handler;   /* 命令总线 handler */
static uint8_t         g_assembled = 0U;  /* 组装完成标志 */

/* ---- 上电握手 / 里程计同步：四轮到位事件标志（RX 钩子置位） ---- */
static osEventFlagsId_t g_pos_flags = NULL;            /* 每轮一位，广播读回复置位 */
static volatile uint8_t g_enabled[ZDT_ADP_MOTOR_NUM]; /* 收使能应答（诊断 + 重发收敛） */

/* ---- CMSIS-OS2：线程 ---- */

/* 收发线程共用属性（TX/RX 同优先级，解析负载轻） */
static const osThreadAttr_t g_mh_thread_attr = {
    .name       = "mh_worker",
    .stack_size = SYS_MH_STACK_BYTES,
    .priority   = osPriorityNormal,
};

/**
 * @brief  新建线程（注入给 handler）
 * @param  entry  线程体，签名 void(*)(void*)
 * @param  arg    传给线程体的参数
 * @param  handle 输出线程句柄
 * @retval ZDT_OK / ZDT_ERR_PARAM / ZDT_ERR_RES
 */
static zdt_status_t os_thread_new(void (*entry)(void *), void *arg,
                                  void **handle)
{
    osThreadId_t id = NULL; /* 新线程句柄 */

    if ((entry == NULL) || (handle == NULL)) {
        return ZDT_ERR_PARAM;
    }
    id = osThreadNew((osThreadFunc_t)entry, arg, &g_mh_thread_attr);
    if (id == NULL) {
        return ZDT_ERR_RES;
    }
    *handle = (void *)id;
    return ZDT_OK;
}

/* ---- CMSIS-OS2：消息队列 ---- */

/**
 * @brief  新建消息队列（注入给 handler）
 * @param  depth   队列深度
 * @param  item_sz 单条消息字节数
 * @param  handle  输出队列句柄
 * @retval ZDT_OK / ZDT_ERR_PARAM / ZDT_ERR_RES
 */
static zdt_status_t os_queue_new(uint32_t depth, uint32_t item_sz,
                                 void **handle)
{
    osMessageQueueId_t id = NULL; /* 新队列句柄 */

    if ((handle == NULL) || (depth == 0U) || (item_sz == 0U)) {
        return ZDT_ERR_PARAM;
    }
    id = osMessageQueueNew(depth, item_sz, NULL);
    if (id == NULL) {
        return ZDT_ERR_RES;
    }
    *handle = (void *)id;
    return ZDT_OK;
}

/**
 * @brief  入队一条消息（上层在 ISR / 任务均以 timeout=0 非阻塞投递）
 * @param  q          队列句柄
 * @param  item       消息源
 * @param  timeout_ms 超时 ms
 * @retval ZDT_OK / ZDT_ERR_PARAM / ZDT_ERR_RES
 */
static zdt_status_t os_queue_put(void *q, const void *item,
                                 uint32_t timeout_ms)
{
    if ((q == NULL) || (item == NULL)) {
        return ZDT_ERR_PARAM;
    }
    if (osMessageQueuePut((osMessageQueueId_t)q, item,
                          0U, timeout_ms) != osOK) {
        return ZDT_ERR_RES;
    }
    return ZDT_OK;
}

/**
 * @brief  出队一条消息
 * @param  q          队列句柄
 * @param  item       输出缓冲
 * @param  timeout_ms 超时 ms（MH_WAIT_FOREVER 即 osWaitForever）
 * @retval ZDT_OK / ZDT_ERR_PARAM / ZDT_ERR_RES（超时即重试）
 */
static zdt_status_t os_queue_get(void *q, void *item,
                                 uint32_t timeout_ms)
{
    if ((q == NULL) || (item == NULL)) {
        return ZDT_ERR_PARAM;
    }
    if (osMessageQueueGet((osMessageQueueId_t)q, item,
                          NULL, timeout_ms) != osOK) {
        return ZDT_ERR_RES;
    }
    return ZDT_OK;
}

/* ---- CMSIS-OS2：时间 ---- */

/** @brief 阻塞延时 ms（tick=1kHz，ms 即 tick 数） */
static zdt_status_t os_delay_ms(uint32_t ms)
{
    (void)osDelay(ms);
    return ZDT_OK;
}

/** @brief 取系统 tick（帧间隔门控用） */
static uint32_t os_get_tick(void)
{
    return osKernelGetTickCount();
}

/* ---- 临界区：保护 pos_pulse 的 int64 跨线程读写 ---- */

/** @brief 进入临界区（关中断，护极短的位置读写） */
static void os_lock_enter(void)
{
    taskENTER_CRITICAL();
}

/** @brief 退出临界区 */
static void os_lock_exit(void)
{
    taskEXIT_CRITICAL();
}

/* ---- 注入接口实例 ---- */
static const mh_os_thread_t g_os_thread = { os_thread_new };
static const mh_os_queue_t  g_os_queue  = { os_queue_new,
                                            os_queue_put,
                                            os_queue_get };
static const mh_os_time_t   g_os_time   = { os_delay_ms,
                                            os_get_tick };
static const mh_os_lock_t   g_os_lock   = { os_lock_enter,
                                            os_lock_exit };

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
    /* 3) 实例化 handler：绑定总线 + 注入 OS 接口 */
    ret = mh_inst(&g_motor_handler, zdt_adp_group(),
                  &g_os_thread, &g_os_queue, &g_os_time,
                  &g_os_lock, MH_GAP_MS_MIN);
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
