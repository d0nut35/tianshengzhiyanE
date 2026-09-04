/**
 * @file    zdt_motor_handler.h
 * @brief   ZDT 电机 handler：独占一条总线，TX/RX 双线程 + 仲裁
 * @author  haoyu
 * @note    - handler 拥有的是“一条总线”，不是“一个组”：
 *            同总线全部电机都归它管
 *          - TX 线程：唯一发送者，帧间强制 ≥gap_ms（电机要求约 3ms）防黏包
 *          - RX 线程：从帧队列取出 → zdt_motor_parse → 写 pos_pulse（单写者）
 *          - 中断只收 → mh_feed_rx 入 RX 队列；解析与发送分属两个线程
 *          - 线程/队列/时间/临界区经分离的 OS 接口注入，本层 OS 无关
 */

#ifndef ZDT_MOTOR_HANDLER_H
#define ZDT_MOTOR_HANDLER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>
#include "zdt_motor.h"

#define MH_GAP_MS_MIN     6U            /* 电机要求的最小帧间隔 ms */
#define MH_WAIT_FOREVER   0xFFFFFFFFU   /* pf_q_get 阻塞等待 */

/* ---- 注入的 OS 接口（按职责拆分，句柄一律 void*） ---- */

/* 线程 */
typedef struct {
    zdt_status_t (*pf_thread_new)(void (*entry)(void *), void *arg,
                                  void **handle);
} mh_os_thread_t;

/* 消息队列 */
typedef struct {
    zdt_status_t (*pf_q_new)(uint32_t depth, uint32_t item_sz, void **handle);
    zdt_status_t (*pf_q_put)(void *q, const void *item, uint32_t timeout_ms);
    zdt_status_t (*pf_q_get)(void *q, void *item, uint32_t timeout_ms);
} mh_os_queue_t;

/* 时间：延时 + tick（帧间隔门控用） */
typedef struct {
    zdt_status_t (*pf_delay_ms)(uint32_t ms);
    uint32_t     (*pf_get_tick)(void);
} mh_os_time_t;

/* 临界区：保护位置 int64 的跨线程读写 */
typedef struct {
    void (*pf_enter)(void);
    void (*pf_exit)(void);
} mh_os_lock_t;

/* ---- 上层事件钩子（RX 解析结果上抛） ---- */

/* 每解析出一帧回调上抛：RX 线程上下文，勿阻塞 / 勿持锁久 */
typedef void (*mh_evt_cb_t)(void *ctx, uint8_t idx,
                            const zdt_rx_t *rx);

/* 电机 handler 对象（一个实例 = 一条总线） */
typedef struct {
    uint8_t      is_inited;       /* 是否已实例化 */
    uint8_t      is_started;      /* 是否已起线程 */
    uint16_t     gap_ms;          /* 帧间最小间隔，≥MH_GAP_MS_MIN */
    zdt_group_t *bus;             /* 本总线电机注册表 + 广播发送口 */
    uint32_t     last_tx_tick;    /* 上次发送时刻，TX 线程用于间隔门控 */
    void        *tx_queue;        /* TX 请求队列：上层投递的控制命令 */
    void        *rx_queue;        /* RX 帧队列：中断喂入的原始返回帧 */
    void        *tx_thread;       /* TX 线程：串行下发 + 3ms 间隔 */
    void        *rx_thread;       /* RX 线程：取帧解析 + 写 pos_pulse */
    const mh_os_thread_t *os_thread; /* 注入：线程 */
    const mh_os_queue_t  *os_queue;  /* 注入：队列 */
    const mh_os_time_t   *os_time;   /* 注入：时间 */
    const mh_os_lock_t   *os_lock;   /* 注入：临界区 */
    mh_evt_cb_t           pf_on_evt; /* 注入：RX 事件钩子，可空 */
    void                 *evt_ctx;   /* 注入：钩子上下文 */
} motor_handler_t;

/**
 * @brief  实例化 handler：绑定总线注册表与 OS 接口
 * @param  h      handler 对象
 * @param  bus    已装配的总线电机表（来自 zdt_adp_group），含各电机句柄
 * @param  thread 线程接口
 * @param  queue  队列接口
 * @param  time   时间接口
 * @param  lock   临界区接口
 * @param  gap_ms 帧间最小间隔，小于 MH_GAP_MS_MIN 时取下限
 * @retval ZDT_OK / ZDT_ERR_PARAM
 */
zdt_status_t mh_inst(motor_handler_t *h, zdt_group_t *bus,
                     const mh_os_thread_t *thread,
                     const mh_os_queue_t *queue,
                     const mh_os_time_t *time,
                     const mh_os_lock_t *lock,
                     uint16_t gap_ms);

/**
 * @brief  注册 RX 事件钩子：每解析出一帧经它上抛上层
 * @param  h   handler 对象
 * @param  pf  钩子函数，可空（空则不回调）
 * @param  ctx 钩子上下文，回调时原样带回
 * @retval ZDT_OK / ZDT_ERR_PARAM / ZDT_ERR_INIT
 * @note   建议在 mh_start 前注册；回调在 RX 线程上下文，勿阻塞
 */
zdt_status_t mh_on_event(motor_handler_t *h, mh_evt_cb_t pf,
                         void *ctx);

/**
 * @brief  启动：建 TX/RX 两个队列与两个工作线程
 * @param  h handler 对象
 * @retval ZDT_OK / ZDT_ERR_INIT / ZDT_ERR_RES
 */
zdt_status_t mh_start(motor_handler_t *h);

/**
 * @brief  投递：同步广播总线前 n 个电机的速度（一帧多机命令）
 * @param  h     handler 对象
 * @param  speed 有符号速度数组，对应 idx 0..n-1
 * @param  n     参与同步的电机数，≤总线电机数
 * @param  accel 加速度档
 * @retval ZDT_OK / ZDT_ERR_PARAM / ZDT_ERR_INIT / ZDT_ERR_RES
 */
zdt_status_t mh_speed(motor_handler_t *h, const int16_t *speed,
                      uint8_t n, uint8_t accel);

/**
 * @brief  投递：使能 / 失能总线上某台电机
 * @param  h   handler 对象
 * @param  idx 电机下标
 * @param  on  true=使能
 * @retval ZDT_OK / ZDT_ERR_PARAM / ZDT_ERR_INIT / ZDT_ERR_RES
 */
zdt_status_t mh_enable(motor_handler_t *h, uint8_t idx, bool on);

/**
 * @brief  投递：单机位置模式
 * @param  h     handler 对象
 * @param  idx   电机下标
 * @param  mode  位置模式
 * @param  pulse 有符号目标脉冲
 * @param  rpm   速度幅值
 * @param  accel 加速度档
 * @retval ZDT_OK / ZDT_ERR_PARAM / ZDT_ERR_INIT / ZDT_ERR_RES
 */
zdt_status_t mh_pos(motor_handler_t *h, uint8_t idx, zdt_pos_mode_t mode,
                    int32_t pulse, uint16_t rpm, uint8_t accel);

/**
 * @brief  投递：单机回零
 * @param  h    handler 对象
 * @param  idx  电机下标
 * @param  mode 回零模式
 * @retval ZDT_OK / ZDT_ERR_PARAM / ZDT_ERR_INIT / ZDT_ERR_RES
 */
zdt_status_t mh_home(motor_handler_t *h, uint8_t idx, zdt_home_mode_t mode);

/**
 * @brief  投递：配置总线多机定时上报位置周期
 * @param  h         handler 对象
 * @param  period_ms 上报周期 ms，0 关闭
 * @retval ZDT_OK / ZDT_ERR_PARAM / ZDT_ERR_INIT / ZDT_ERR_RES
 */
zdt_status_t mh_report(motor_handler_t *h, uint16_t period_ms);

/**
 * @brief  投递：多机命令读总线全部电机实时位置（00 AA 一帧，仅入队）
 * @param  h handler 对象
 * @retval ZDT_OK / ZDT_ERR_PARAM / ZDT_ERR_INIT / ZDT_ERR_RES
 * @note   回复经各路独立 RX 到达，由 RX 线程逐帧解析更新 pos_pulse
 */
zdt_status_t mh_request_pos_all(motor_handler_t *h);

/**
 * @brief  线程安全读取某电机当前位置脉冲（里程计用）
 * @param  h   handler 对象
 * @param  idx 电机下标
 * @param  out 输出位置脉冲
 * @retval ZDT_OK / ZDT_ERR_PARAM / ZDT_ERR_INIT
 */
zdt_status_t mh_get_pos(motor_handler_t *h, uint8_t idx, int64_t *out);

/**
 * @brief  喂入一帧接收数据（适配层 rx 回调里调，ISR 上下文）
 * @param  h     handler 对象
 * @param  motor 来源电机
 * @param  data  帧缓冲
 * @param  len   帧长度
 * @retval ZDT_OK / ZDT_ERR_PARAM / ZDT_ERR_INIT / ZDT_ERR_RES
 */
zdt_status_t mh_feed_rx(motor_handler_t *h, zdt_motor_t *motor,
                        const uint8_t *data, uint16_t len);

#ifdef __cplusplus
}
#endif

#endif /* ZDT_MOTOR_HANDLER_H */
