/**
 * @file    zdt.h
 * @brief   固定四轮 ZDT 电机 BSP 与 handler 接口。
 * @note    - 本车硬件固定：huart3 发命令，huart3/4/5/6 收四轮回传
 *          - 对上保留原 zdt_cmd / zdt_motor / mh / zdt_adp 接口
 *          - 实现合并到 zdt.c，减少跨层函数与可移植抽象
 */

#ifndef ZDT_H
#define ZDT_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#define ZDT_ADP_MOTOR_NUM   4U          /* 本车固定四轮数量 */
#define ZDT_GROUP_MAX       10U         /* 兼容旧组对象槽位上限 */
#define ZDT_RPM_MAX         30000U      /* 协议速度幅值上限 */
#define ZDT_ACCEL_MAX       0xFFU       /* 加速度档上限 */
#define ZDT_FRAME_MAX       36U         /* 单帧/RX 缓冲兼容上限 */
#define ZDT_ADDR_BROADCAST  0x00U       /* 多机帧广播地址 */

#define MH_GAP_MS_MIN       6U          /* 电机帧间最小间隔 ms */
#define MH_WAIT_FOREVER     0xFFFFFFFFU /* 队列永久等待 */

typedef enum {
    ZDT_OK        = 0,
    ZDT_ERR       = 1,
    ZDT_ERR_PARAM = 2,
    ZDT_ERR_INIT  = 3,
    ZDT_ERR_RES   = 4,
} zdt_status_t;

typedef enum {
    ZDT_DIR_CW  = 0U,
    ZDT_DIR_CCW = 1U,
} zdt_dir_t;

typedef enum {
    ZDT_POS_REL_LAST = 0x00U,
    ZDT_POS_ABS      = 0x01U,
    ZDT_POS_REL_NOW  = 0x02U,
} zdt_pos_mode_t;

typedef enum {
    ZDT_HOME_NEAREST = 0x00U,
    ZDT_HOME_DIR     = 0x01U,
    ZDT_HOME_COLLIDE = 0x02U,
    ZDT_HOME_LIMIT   = 0x03U,
    ZDT_HOME_ABS     = 0x04U,
    ZDT_HOME_PWRLOSS = 0x05U,
} zdt_home_mode_t;

typedef struct {
    zdt_dir_t dir;      /* 协议方向 */
    uint16_t  rpm;      /* 速度幅值 */
    uint8_t   accel;    /* 加速度档 */
} zdt_speed_t;

typedef struct {
    zdt_dir_t      dir;   /* 协议方向 */
    uint16_t       rpm;   /* 速度幅值 */
    uint8_t        accel; /* 加速度档 */
    uint32_t       pulse; /* 目标脉冲幅值 */
    zdt_pos_mode_t mode;  /* 位置模式 */
} zdt_pos_t;

typedef struct {
    uint8_t *buf;      /* 外部缓冲 */
    uint16_t cap;      /* 缓冲容量 */
    uint16_t len;      /* 当前长度 */
} zdt_multi_t;

typedef enum {
    ZDT_RX_NONE = 0,
    ZDT_RX_POS,
    ZDT_RX_ACK,
    ZDT_RX_HOME_IDLE,
    ZDT_RX_PARAM_ERR,
    ZDT_RX_FMT_ERR,
    ZDT_RX_DONE,
} zdt_rx_kind_t;

typedef struct {
    zdt_rx_kind_t kind;  /* 返回帧类型 */
    uint8_t       addr;  /* 电机地址 */
    uint8_t       code;  /* 功能码 */
    int64_t       pulse; /* 位置脉冲 */
} zdt_rx_t;

typedef struct zdt_motor zdt_motor_t;
typedef struct zdt_group zdt_group_t;

typedef zdt_status_t (*zdt_send_fn_t)(zdt_motor_t *motor,
                                      const uint8_t *frame,
                                      uint16_t size);
typedef zdt_status_t (*zdt_rx_fn_t)(zdt_motor_t *motor);
typedef zdt_status_t (*zdt_grp_send_fn_t)(const uint8_t *frame,
                                          uint16_t size);

struct zdt_motor {
    uint8_t       is_inited;   /* 是否可用 */
    uint8_t       is_enabled;  /* 使能状态 */
    uint8_t       addr;        /* 协议地址 */
    zdt_dir_t     dir;         /* 本车默认正方向 */
    int16_t       speed;       /* 缓存速度 */
    uint8_t       accel;       /* 缓存加速度 */
    int64_t       pos_pulse;   /* 当前位置脉冲 */
    zdt_send_fn_t pf_send;     /* 兼容旧注入接口 */
    zdt_rx_fn_t   pf_start_rx; /* 兼容旧注入接口 */
};

struct zdt_group {
    uint8_t           is_inited;             /* 是否可用 */
    uint8_t           count;                 /* 已挂载数量 */
    uint16_t          report_ms;             /* 上报周期 */
    zdt_motor_t      *motors[ZDT_GROUP_MAX]; /* 兼容旧槽位 */
    zdt_grp_send_fn_t pf_send;               /* 兼容旧广播接口 */
};

typedef struct {
    zdt_status_t (*pf_thread_new)(void (*entry)(void *), void *arg,
                                  void **handle);
} mh_os_thread_t;

typedef struct {
    zdt_status_t (*pf_q_new)(uint32_t depth, uint32_t item_sz, void **handle);
    zdt_status_t (*pf_q_put)(void *q, const void *item, uint32_t timeout_ms);
    zdt_status_t (*pf_q_get)(void *q, void *item, uint32_t timeout_ms);
} mh_os_queue_t;

typedef struct {
    zdt_status_t (*pf_delay_ms)(uint32_t ms);
    uint32_t     (*pf_get_tick)(void);
} mh_os_time_t;

typedef struct {
    void (*pf_enter)(void);
    void (*pf_exit)(void);
} mh_os_lock_t;

typedef void (*mh_evt_cb_t)(void *ctx, uint8_t idx, const zdt_rx_t *rx);

typedef struct {
    uint8_t      is_inited;    /* 是否已实例化 */
    uint8_t      is_started;   /* 是否已启动 */
    uint16_t     gap_ms;       /* 帧间隔 */
    zdt_group_t *bus;          /* 固定四轮总线 */
    uint32_t     last_tx_tick; /* 上次发送 tick */
    void        *tx_queue;     /* TX 队列 */
    void        *rx_queue;     /* RX 队列 */
    void        *tx_thread;    /* TX 线程 */
    void        *rx_thread;    /* RX 线程 */
    mh_evt_cb_t  pf_on_evt;    /* RX 事件钩子 */
    void        *evt_ctx;      /* 钩子上下文 */
} motor_handler_t;

typedef void (*zdt_rx_cb_t)(zdt_motor_t *motor,
                            const uint8_t *data,
                            uint16_t len);

zdt_status_t zdt_cmd_enable(uint8_t *buf, uint16_t cap, uint8_t addr,
                            bool on, uint16_t *len);
zdt_status_t zdt_cmd_speed(uint8_t *buf, uint16_t cap, uint8_t addr,
                           const zdt_speed_t *spd, uint16_t *len);
zdt_status_t zdt_cmd_pos(uint8_t *buf, uint16_t cap, uint8_t addr,
                         const zdt_pos_t *pos, uint16_t *len);
zdt_status_t zdt_cmd_home(uint8_t *buf, uint16_t cap, uint8_t addr,
                          zdt_home_mode_t mode, uint16_t *len);
zdt_status_t zdt_cmd_report(uint8_t *buf, uint16_t cap, uint8_t addr,
                            uint16_t period_ms, uint16_t *len);
zdt_status_t zdt_cmd_read_pos(uint8_t *buf, uint16_t cap, uint8_t addr,
                              uint16_t *len);
zdt_status_t zdt_multi_init(zdt_multi_t *mb, uint8_t *buf, uint16_t cap);
zdt_status_t zdt_multi_add_speed(zdt_multi_t *mb, uint8_t addr,
                                 const zdt_speed_t *spd);
zdt_status_t zdt_multi_add_raw(zdt_multi_t *mb, const uint8_t *sub,
                               uint16_t sub_len);
zdt_status_t zdt_multi_done(zdt_multi_t *mb, uint16_t *len);
zdt_status_t zdt_decode(const uint8_t *data, uint16_t len, zdt_rx_t *out);

zdt_status_t zdt_motor_inst(zdt_motor_t *motor, uint8_t addr,
                            zdt_dir_t dir, zdt_send_fn_t pf_send,
                            zdt_rx_fn_t pf_start_rx);
zdt_status_t zdt_motor_enable(zdt_motor_t *motor);
zdt_status_t zdt_motor_disable(zdt_motor_t *motor);
zdt_status_t zdt_motor_set_speed(zdt_motor_t *motor,
                                 int16_t speed, uint8_t accel);
zdt_status_t zdt_motor_run_speed(zdt_motor_t *motor,
                                 int16_t speed, uint8_t accel);
zdt_status_t zdt_motor_run_pos(zdt_motor_t *motor,
                               zdt_pos_mode_t mode,
                               int32_t pulse,
                               uint16_t rpm,
                               uint8_t accel);
zdt_status_t zdt_motor_home(zdt_motor_t *motor, zdt_home_mode_t mode);
zdt_status_t zdt_motor_parse(zdt_motor_t *motor, const uint8_t *data,
                             uint16_t len, zdt_rx_t *out);
zdt_status_t zdt_group_inst(zdt_group_t *group, zdt_grp_send_fn_t pf_send);
zdt_status_t zdt_group_bind(zdt_group_t *group,
                            uint16_t slot,
                            zdt_motor_t *motor);
zdt_status_t zdt_group_speed(zdt_group_t *group);
zdt_status_t zdt_group_report(zdt_group_t *group, uint16_t period_ms);
zdt_status_t zdt_group_read_pos(zdt_group_t *group);

zdt_status_t mh_inst(motor_handler_t *h, zdt_group_t *bus,
                     const mh_os_thread_t *thread,
                     const mh_os_queue_t *queue,
                     const mh_os_time_t *time,
                     const mh_os_lock_t *lock,
                     uint16_t gap_ms);
zdt_status_t mh_on_event(motor_handler_t *h, mh_evt_cb_t pf, void *ctx);
zdt_status_t mh_start(motor_handler_t *h);
zdt_status_t mh_speed(motor_handler_t *h, const int16_t *speed,
                      uint8_t n, uint8_t accel);
zdt_status_t mh_enable(motor_handler_t *h, uint8_t idx, bool on);
zdt_status_t mh_pos(motor_handler_t *h, uint8_t idx,
                    zdt_pos_mode_t mode, int32_t pulse,
                    uint16_t rpm, uint8_t accel);
zdt_status_t mh_home(motor_handler_t *h, uint8_t idx,
                     zdt_home_mode_t mode);
zdt_status_t mh_report(motor_handler_t *h, uint16_t period_ms);
zdt_status_t mh_request_pos_all(motor_handler_t *h);
zdt_status_t mh_get_pos(motor_handler_t *h, uint8_t idx, int64_t *out);
zdt_status_t mh_feed_rx(motor_handler_t *h, zdt_motor_t *motor,
                        const uint8_t *data, uint16_t len);

zdt_status_t zdt_adp_init(zdt_rx_cb_t on_frame);
zdt_status_t zdt_adp_start(void);
zdt_group_t *zdt_adp_group(void);
zdt_motor_t *zdt_adp_motor(uint8_t idx);
zdt_status_t zdt_adp_rx_isr(uint8_t idx, uint16_t size);
zdt_status_t zdt_adp_err_isr(uint8_t idx);

#ifdef __cplusplus
}
#endif

#endif /* ZDT_H */
