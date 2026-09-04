/**
 * @file    hwt101_isr_bench.h
 * @brief   HWT101 中断耗时基准：对比 ISR 内"解析" vs "入队"（可移除）
 * @author  haoyu
 * @note    - 用 DWT->CYCCNT 计中断段周期数；min 最可信
 *            (滤抢占/cache 噪声)
 *          - 在 huart2 RxEvent 里调
 *            hwt101_bench_on_rx(buf,size) 即可采集
 *          - 仅供测量，定稿后删除；依赖 FreeRTOS + CMSIS DWT
 */

#ifndef HWT101_ISR_BENCH_H
#define HWT101_ISR_BENCH_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/** @brief 初始化：开 DWT 周期计数 + 建队列与排空任务 */
void hwt101_bench_init(void);

/** @brief 在 huart2 RxEvent ISR 内调用，喂入刚收完的缓冲与长度 */
void hwt101_bench_on_rx(const uint8_t *buf, uint16_t size);

/** @brief 打印四类段耗时统计(cnt/min/avg/max，周期与 us) */
void hwt101_bench_report(void);

#ifdef __cplusplus
}
#endif

#endif /* HWT101_ISR_BENCH_H */
