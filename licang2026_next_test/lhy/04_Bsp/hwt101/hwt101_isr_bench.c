/**
 * @file    hwt101_isr_bench.c
 * @brief   HWT101 中断耗时基准：ISR 内"解析" vs "入队"
 *          对照测量（可移除）
 * @author  haoyu
 * @note    - 三段分别用 DWT 计周期：懒扫描基线 / ISR 解析 / ISR 入队
 *          - parse、enqueue 按 IRQ 奇偶交替各测一半，
 *            互不暖 cache，贴近实际只跑其一
 *          - enqueue 策略的解析成本搬到排空任务，
 *            单独计为 tparse 供对照
 */

#include "hwt101_isr_bench.h"

#include "main.h"        /* CMSIS: DWT / CoreDebug + HAL */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include <string.h>

#include "hwt101.h"      /* 复用协议常量 FRAME_LEN / HEAD / ID_* */

#define BENCH_CPU_MHZ   216U    /* 内核主频，用于周期→us
                                * 换算，按实际改 */
#define BENCH_BUF_LEN    64U    /* 与 HWT101 DMA 缓冲等长 */
#define BENCH_Q_DEPTH     8U    /* 入队基准的队列深度 */
#define BENCH_GYRO_FS  2000.0f  /* 角速度满量程 dps */
#define BENCH_ANGLE_FS  180.0f  /* 角度满量程 deg */
#define BENCH_RAW_FS  32768.0f  /* int16 满量程 */

/* 一段中断耗时统计（单位：CPU 周期） */
typedef struct {
    uint32_t last;      /* 最近一次 */
    uint32_t min;       /* 最小值，最可信的中断段成本 */
    uint32_t max;       /* 最大值，含抢占/cache 抖动 */
    uint64_t sum;       /* 累加，用于求平均 */
    uint32_t cnt;       /* 样本数 */
} bench_stat_t;

/* 入队基准的消息：原始帧 + 长度 */
typedef struct {
    uint16_t len;                  /* 有效字节数 */
    uint8_t  data[BENCH_BUF_LEN];  /* 帧拷贝 */
} bench_msg_t;

static bench_stat_t g_stat_base;    /* 懒扫描 ISR 基线段 */
static bench_stat_t g_stat_parse;   /* ISR 内解析段 */
static bench_stat_t g_stat_enq;     /* ISR 内入队段 */
/* 排空任务里的解析段（入队策略的真实解析开销） */
static bench_stat_t g_stat_tparse;

static QueueHandle_t g_q = NULL;    /* 入队基准队列 */
static uint32_t g_enq_drop = 0U;    /* 队列满丢帧计数 */
static uint32_t g_drain_cnt = 0U;   /* 排空任务出队计数 */
/* IRQ 计数，用于 parse/enqueue 交替 */
static uint32_t g_round = 0U;

/* 解析结果(被 ISR 与任务共享，基准用途容忍竞态) */
static volatile float g_yaw_deg = 0.0f;
static volatile float g_gyro_dps = 0.0f;  /* 解析结果 */

/* 模拟懒扫描 ping-pong 的有效长度记录 */
static uint16_t g_pp_len[2] = {0U, 0U};
/* 模拟懒扫描的活动缓冲下标 */
static uint8_t  g_active = 0U;

/** @brief 读当前周期计数 */
static inline uint32_t cyc_now(void)
{
    return DWT->CYCCNT;
}

/** @brief 使能 DWT 周期计数器（幂等） */
static void dwt_init(void)
{
    /* 打开跟踪总开关 */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0U;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;            /* 启动周期计数 */
}

/** @brief 累计一段耗时样本 */
static void stat_add(bench_stat_t *s, uint32_t dt)
{
    s->last = dt;
    /* 首样本或更小则刷新下限 */
    if ((s->cnt == 0U) || (dt < s->min)) {
        s->min = dt;
    }
    if (dt > s->max) {
        s->max = dt;
    }
    s->sum += dt;
    s->cnt++;
}

/** @brief 校验一帧 11 字节子帧的校验和 */
static uint8_t bench_csum_ok(const uint8_t *f)
{
    uint8_t s = 0U;  /* 累加和 */
    uint8_t i;       /* 字节下标 */

    for (i = 0U; i < (HWT101_FRAME_LEN - 1U); i++) {
        s = (uint8_t)(s + f[i]);
    }
    return (s == f[HWT101_FRAME_LEN - 1U]) ? 1U : 0U;
}

/**
 * @brief  待测"解析"：找包头 0x55 逐帧校验解码（复刻原版懒扫描内核）
 * @param  buf 原始字节
 * @param  len 有效字节数
 */
static void bench_parse(const uint8_t *buf, uint16_t len)
{
    uint16_t i = 0U;  /* 扫描下标 */
    int16_t  raw;     /* 原始 int16 量 */

    while (i < len) {
        /* 前导乱码逐字节跳过 */
        if (buf[i] != HWT101_FRAME_HEAD) {
            i++;
            continue;
        }
        /* 尾部不足一帧 */
        if ((uint16_t)(len - i) < HWT101_FRAME_LEN) {
            break;
        }
        /* 校验失败仅前进 1 字节 */
        if (bench_csum_ok(&buf[i]) == 0U) {
            i++;
            continue;
        }
        raw = (int16_t)(((uint16_t)buf[i + 7U] << 8U)
                        | (uint16_t)buf[i + 6U]);
        if (buf[i + 1U] == HWT101_ID_GYRO) {
            g_gyro_dps = ((float)raw * BENCH_GYRO_FS) / BENCH_RAW_FS;
        } else if (buf[i + 1U] == HWT101_ID_ANGLE) {
            g_yaw_deg = ((float)raw * BENCH_ANGLE_FS) / BENCH_RAW_FS;
        } else {
            i++;                                      /* 未知功能码 */
            continue;
        }
        i = (uint16_t)(i + HWT101_FRAME_LEN);
    }
}

/**
 * @brief  待测"入队"：拷贝整帧并 FromISR 投递（zdt 电机同款模式）
 * @param  buf 原始字节
 * @param  len 有效字节数
 */
static void bench_enqueue(const uint8_t *buf, uint16_t len)
{
    /* static：ISR 内单发者，免 64B 栈帧 */
    static bench_msg_t msg;
    BaseType_t hpw = pdFALSE;          /* 是否唤醒更高优先级任务 */

    msg.len = (len <= BENCH_BUF_LEN) ? len : BENCH_BUF_LEN;
    (void)memcpy(msg.data, buf, msg.len);
    if (xQueueSendFromISR(g_q, &msg, &hpw) != pdTRUE) {
        g_enq_drop++;                  /* 队列满，记一次丢帧 */
    }
    portYIELD_FROM_ISR(hpw);
}

/**
 * @brief  排空任务：出队并解析，计入队策略真实搬到任务侧的解析开销
 * @param  arg 未用
 */
static void bench_drain_task(void *arg)
{
    bench_msg_t m;     /* 出队消息 */
    uint32_t    t0;    /* 计时起点 */

    (void)arg;
    for (;;) {
        if (xQueueReceive(g_q, &m, portMAX_DELAY) == pdTRUE) {
            t0 = cyc_now();
            bench_parse(m.data, m.len);
            stat_add(&g_stat_tparse, cyc_now() - t0);
            g_drain_cnt++;
        }
    }
}

void hwt101_bench_init(void)
{
    dwt_init();
    g_q = xQueueCreate(BENCH_Q_DEPTH, sizeof(bench_msg_t));
    /* 队列建失败则入队基准不可用，但不影响解析基准 */
    if (g_q == NULL) {
        return;
    }
    (void)xTaskCreate(bench_drain_task, "hwtbench", 256U, NULL,
                      tskIDLE_PRIORITY + 2U, NULL);
}

void hwt101_bench_on_rx(const uint8_t *buf, uint16_t size)
{
    uint16_t len;   /* 截断后的有效长度 */
    uint32_t t0;    /* 计时起点 */

    if (buf == NULL) {
        return;
    }
    len = (size <= BENCH_BUF_LEN) ? size : BENCH_BUF_LEN;

    /* 基线：懒扫描在 ISR 里仅做的事——记录本次有效长度
     * + 切换活动缓冲 */
    t0 = cyc_now();
    g_pp_len[g_active] = len;
    g_active ^= 1U;
    stat_add(&g_stat_base, cyc_now() - t0);

    /* parse / enqueue 按奇偶交替，各测一半，避免互相暖 cache */
    if ((g_round & 1U) == 0U) {
        t0 = cyc_now();
        bench_parse(buf, len);
        stat_add(&g_stat_parse, cyc_now() - t0);
    } else if (g_q != NULL) {
        t0 = cyc_now();
        bench_enqueue(buf, len);
        stat_add(&g_stat_enq, cyc_now() - t0);
    } else {
        /* 队列未就绪，跳过入队基准 */
    }
    g_round++;
}

/** @brief 打印一段统计：cnt + min/avg/max（周期与 us） */
static void bench_print(const char *tag, const bench_stat_t *s)
{
    uint32_t avg;   /* 平均周期 */
    uint32_t cnt = s->cnt;

    if (cnt == 0U) {
        printf("%-8s n=0\r\n", tag);
        return;
    }
    avg = (uint32_t)(s->sum / cnt);
    printf("%-8s n=%lu  min=%lu(%lu.%02luus) "
           "avg=%lu(%lu.%02luus) "
           "max=%lu(%lu.%02luus)\r\n",
           tag, (unsigned long)cnt,
           (unsigned long)s->min,
           (unsigned long)(s->min / BENCH_CPU_MHZ),
           (unsigned long)(((s->min % BENCH_CPU_MHZ) * 100U)
                           / BENCH_CPU_MHZ),
           (unsigned long)avg, (unsigned long)(avg / BENCH_CPU_MHZ),
           (unsigned long)(((avg % BENCH_CPU_MHZ) * 100U)
                           / BENCH_CPU_MHZ),
           (unsigned long)s->max,
           (unsigned long)(s->max / BENCH_CPU_MHZ),
           (unsigned long)(((s->max % BENCH_CPU_MHZ) * 100U)
                           / BENCH_CPU_MHZ));
}

void hwt101_bench_report(void)
{
    printf("---- hwt101 isr bench @%uMHz drop=%lu drain=%lu ----\r\n",
           (unsigned)BENCH_CPU_MHZ, (unsigned long)g_enq_drop,
           (unsigned long)g_drain_cnt);
    bench_print("base",   &g_stat_base);    /* 懒扫描 ISR 基线 */
    bench_print("parse",  &g_stat_parse);   /* ISR 内解析 */
    bench_print("enq",    &g_stat_enq);     /* ISR 内入队 */
    /* 入队策略：任务侧解析开销 */
    bench_print("tparse", &g_stat_tparse);
}
