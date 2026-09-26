/**
 * @file    zdt_motor_handler.c
 * @brief   ZDT 电机 handler 实现：TX/RX 双线程 + 总线仲裁（OS 注入）
 * @author  haoyu
 * @note    - TX 线程唯一发送者，帧间用 tick 门控强制 ≥gap_ms 防黏包
 *          - RX 线程取帧解析并在持锁下写 pos_pulse
 *          - 锁只护 pos_pulse；speed/accel/cmd 仅 TX 线程读写无需锁
 */

#include "zdt_motor_handler.h"

#include <string.h>

#define MH_TX_QUEUE_DEPTH   8U    /* TX 请求队列深度 */
#define MH_RX_QUEUE_DEPTH   16U   /* RX 帧队列深度（2ms×4 上报留余量） */
#define MH_RX_FRAME_MAX     36U   /* RX 帧拷贝上界，与适配层 DMA 缓冲一致 */

/* ---- 内部类型 ---- */

/* TX 请求类型 */
typedef enum {
    MH_REQ_SPEED = 0, /* 组速度 */
    MH_REQ_ENABLE,    /* 使能/失能 */
    MH_REQ_POS,       /* 单机位置 */
    MH_REQ_HOME,      /* 单机回零 */
    MH_REQ_REPORT,    /* 定时上报 */
    MH_REQ_READ_POS,  /* 多机读位置 */
} mh_req_kind_t;

/* TX 请求（标签联合，进 tx_queue） */
typedef struct {
    mh_req_kind_t kind; /* 请求类型 */
    union {
        /* 组速度请求：缓存多台电机速度后广播 */
        struct {
            int16_t speed[ZDT_GROUP_MAX]; /* 各电机有符号速度 */
            uint8_t motor_num;            /* 电机数 */
            uint8_t accel;                /* 加速度档 */
        } speed;
        /* 单机使能请求：控制指定电机使能状态 */
        struct {
            uint8_t motor_idx; /* 电机下标 */
            uint8_t enabled;   /* 1=使能 */
        } enable;
        /* 单机位置请求：按指定模式运行到目标脉冲 */
        struct {
            uint8_t        motor_idx; /* 电机下标 */
            zdt_pos_mode_t mode;      /* 位置模式 */
            int32_t        pulse;     /* 有符号目标脉冲 */
            uint16_t       rpm;       /* 速度幅值 */
            uint8_t        accel;     /* 加速度档 */
        } position;
        /* 单机回零请求：按指定模式触发回零 */
        struct {
            uint8_t         motor_idx; /* 电机下标 */
            zdt_home_mode_t mode;      /* 回零模式 */
        } home;
        /* 上报配置请求：设置定时上报周期 */
        struct {
            uint16_t period_ms; /* 上报周期 ms */
        } report;
    } data;
} mh_tx_req_t;

/* RX 帧事件（中断拷贝后进 rx_queue） */
typedef struct {
    zdt_motor_t *motor;            /* 来源电机 */
    uint16_t     len;              /* 帧长度 */
    uint8_t      data[MH_RX_FRAME_MAX]; /* 帧副本 */
} mh_rx_evt_t;

/* ---- 内部工具 ---- */

/** @brief 取总线槽位电机指针，越界/空返回 NULL */
static zdt_motor_t *motor_at(const motor_handler_t *h, uint8_t idx)
{
    if (idx >= ZDT_GROUP_MAX) {
        return NULL;
    }
    return h->bus->motors[idx];
}

/** @brief 反查电机在总线的槽位，未命中返回 ZDT_GROUP_MAX */
static uint8_t motor_index(const motor_handler_t *h,
                           const zdt_motor_t *m)
{
    uint8_t i = 0U; /* 槽位下标 */

    for (i = 0U; i < ZDT_GROUP_MAX; i++) {
        if (h->bus->motors[i] == m) {
            return i;
        }
    }
    return ZDT_GROUP_MAX;
}

/** @brief 距上次发送补足 gap_ms（tick 无符号减法兼容回绕） */
static void tx_gate(motor_handler_t *h)
{
    uint32_t now = h->os_time->pf_get_tick(); /* 当前 tick */
    uint32_t elapsed = now - h->last_tx_tick; /* 距上次发送已过 ms */

    if (elapsed < h->gap_ms) {
        (void)h->os_time->pf_delay_ms((uint32_t)h->gap_ms - elapsed);
    }
}

/**
 * @brief  按请求类型在总线上执行一次发送
 * @param  h   handler 对象
 * @param  req TX 请求
 * @retval 透传底层 zdt_* 的状态
 */
static zdt_status_t tx_dispatch(motor_handler_t *h, const mh_tx_req_t *req)
{
    zdt_motor_t *m = NULL; /* 目标电机 */
    uint8_t i = 0U;        /* 槽位下标 */

    switch (req->kind) {
    case MH_REQ_SPEED:
        /* 缓存各电机速度后发送一帧多机广播命令 */
        for (i = 0U; i < req->data.speed.motor_num; i++) {
            m = motor_at(h, i);
            if (m != NULL) {
                (void)zdt_motor_set_speed(m, req->data.speed.speed[i],
                                          req->data.speed.accel);
            }
        }
        return zdt_group_speed(h->bus);
    case MH_REQ_ENABLE:
        /* 控制指定电机立即使能或失能 */
        m = motor_at(h, req->data.enable.motor_idx);
        if (m == NULL) {
            return ZDT_ERR_PARAM;
        }
        if (req->data.enable.enabled != 0U) {
            return zdt_motor_enable(m);
        }
        return zdt_motor_disable(m);
    case MH_REQ_POS:
        /* 控制指定电机按位置模式运行 */
        m = motor_at(h, req->data.position.motor_idx);
        if (m == NULL) {
            return ZDT_ERR_PARAM;
        }
        return zdt_motor_run_pos(m, req->data.position.mode,
                                 req->data.position.pulse,
                                 req->data.position.rpm,
                                 req->data.position.accel);
    case MH_REQ_HOME:
        /* 控制指定电机按选定模式回零 */
        m = motor_at(h, req->data.home.motor_idx);
        if (m == NULL) {
            return ZDT_ERR_PARAM;
        }
        return zdt_motor_home(m, req->data.home.mode);
    case MH_REQ_REPORT:
        /* 同步配置总线内电机的位置上报周期 */
        return zdt_group_report(h->bus, req->data.report.period_ms);
    case MH_REQ_READ_POS:
        /* 一帧多机读(00 AA)，四轮同刻锁存并各自回复 */
        return zdt_group_read_pos(h->bus);
    default:
        /* 队列数据异常时拒绝执行 */
        return ZDT_ERR_PARAM;
    }
}

/** @brief TX 线程体：取请求 → 补够 gap → 串行下发 → 记 tick */
static void tx_thread_body(void *arg)
{
    motor_handler_t *h = (motor_handler_t *)arg; /* handler 对象 */
    mh_tx_req_t req;                             /* 取出的请求 */

    for (;;) {
        // 0.阻塞等待一条发送请求
        if (h->os_queue->pf_q_get(h->tx_queue, &req,
                                  MH_WAIT_FOREVER) != ZDT_OK) {
            continue;
        }
        // 1.补足帧间隔并串行下发
        tx_gate(h); /* 与上一帧间隔 ≥gap_ms */
        (void)tx_dispatch(h, &req);
        // 2.记录本次发送时刻
        h->last_tx_tick = h->os_time->pf_get_tick();
    }
}

/** @brief RX 线程体：取帧 → 持锁解析（位置帧更新 pos_pulse） */
static void rx_thread_body(void *arg)
{
    motor_handler_t *h = (motor_handler_t *)arg; /* handler 对象 */
    mh_rx_evt_t evt;                             /* 取出的帧事件 */
    zdt_rx_t rx;                                 /* 解析结果 */
    uint8_t idx = 0U;                            /* 来源电机槽位 */

    for (;;) {
        // 0.阻塞等待一条接收帧事件
        if (h->os_queue->pf_q_get(h->rx_queue, &evt,
                                  MH_WAIT_FOREVER) != ZDT_OK) {
            continue;
        }
        // 1.持锁解析并更新位置（锁只护 pos_pulse）
        h->os_lock->pf_enter();
        (void)zdt_motor_parse(evt.motor, evt.data, evt.len, &rx);
        h->os_lock->pf_exit();
        // 2.反查槽位，出锁后上抛事件（避免上层回调重入锁死锁）
        idx = motor_index(h, evt.motor);
        if ((h->pf_on_evt != NULL) && (rx.kind != ZDT_RX_NONE) &&
            (idx < ZDT_GROUP_MAX)) {
            h->pf_on_evt(h->evt_ctx, idx, &rx);
        }
    }
}

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
                     const mh_os_thread_t *thread, const mh_os_queue_t *queue,
                     const mh_os_time_t *time, const mh_os_lock_t *lock,
                      uint16_t gap_ms)
{
    // 0.参数合法性检查：handler、总线与 OS 接口
    if ((h == NULL) || (bus == NULL) || (thread == NULL) || (queue == NULL) ||
        (time == NULL) || (lock == NULL)) {
        return ZDT_ERR_PARAM;
    }
    // 1.初始化 handler 成员
    h->is_inited    = 1U;
    h->is_started   = 0U;
    h->gap_ms       = (gap_ms < MH_GAP_MS_MIN) ? MH_GAP_MS_MIN : gap_ms;
    h->bus          = bus;
    h->last_tx_tick = 0U;
    h->tx_queue     = NULL;
    h->rx_queue     = NULL;
    h->tx_thread    = NULL;
    h->rx_thread    = NULL;
    h->os_thread    = thread;
    h->os_queue     = queue;
    h->os_time      = time;
    h->os_lock      = lock;
    h->pf_on_evt    = NULL;
    h->evt_ctx      = NULL;
    return ZDT_OK;
}

/**
 * @brief  注册 RX 事件钩子：每解析出一帧经它上抛上层
 * @param  h   handler 对象
 * @param  pf  钩子函数，可空（空则不回调）
 * @param  ctx 钩子上下文，回调时原样带回
 * @retval ZDT_OK / ZDT_ERR_PARAM / ZDT_ERR_INIT
 * @note   建议在 mh_start 前注册；回调在 RX 线程上下文，勿阻塞
 */
zdt_status_t mh_on_event(motor_handler_t *h, mh_evt_cb_t pf,
                         void *ctx)
{
    // 0.参数合法性检查：handler 指针与实例化状态
    if (h == NULL) {
        return ZDT_ERR_PARAM;
    }
    if (h->is_inited == 0U) {
        return ZDT_ERR_INIT;
    }
    // 1.记录钩子与上下文（pf 可空，表示关闭上抛）
    h->pf_on_evt = pf;
    h->evt_ctx   = ctx;
    return ZDT_OK;
}

/**
 * @brief  启动：建 TX/RX 两个队列与两个工作线程
 * @param  h handler 对象
 * @retval ZDT_OK / ZDT_ERR_INIT / ZDT_ERR_RES
 */
zdt_status_t mh_start(motor_handler_t *h)
{
    zdt_status_t ret = ZDT_ERR; /* 结果 */

    // 0.参数合法性检查：handler 指针与实例化状态
    if (h == NULL) {
        return ZDT_ERR_PARAM;
    }
    if (h->is_inited == 0U) {
        return ZDT_ERR_INIT;
    }
    // 1.创建 TX 与 RX 队列
    ret = h->os_queue->pf_q_new(MH_TX_QUEUE_DEPTH,
                                (uint32_t)sizeof(mh_tx_req_t), &h->tx_queue);
    if (ret != ZDT_OK) {
        return ret;
    }
    ret = h->os_queue->pf_q_new(MH_RX_QUEUE_DEPTH,
                                (uint32_t)sizeof(mh_rx_evt_t), &h->rx_queue);
    if (ret != ZDT_OK) {
        return ret;
    }
    // 2.创建 TX 与 RX 工作线程
    ret = h->os_thread->pf_thread_new(tx_thread_body, h, &h->tx_thread);
    if (ret != ZDT_OK) {
        return ret;
    }
    ret = h->os_thread->pf_thread_new(rx_thread_body, h, &h->rx_thread);
    if (ret != ZDT_OK) {
        return ret;
    }
    // 3.标记 handler 已经启动
    h->is_started = 1U;
    return ZDT_OK;
}

/**
 * @brief  投递组速度请求（前 n 个电机同步广播）
 * @param  h     handler 对象
 * @param  speed 有符号速度数组
 * @param  n     电机数（1..ZDT_GROUP_MAX）
 * @param  accel 加速度档
 * @retval ZDT_OK / ZDT_ERR_PARAM / ZDT_ERR_INIT / ZDT_ERR_RES
 */
zdt_status_t mh_speed(motor_handler_t *h, const int16_t *speed,
                      uint8_t n, uint8_t accel)
{
    mh_tx_req_t req; /* 速度请求 */
    uint8_t i = 0U;  /* 拷贝下标 */

    // 0.参数合法性检查：handler、速度数组、启动状态与数量
    if ((h == NULL) || (speed == NULL)) {
        return ZDT_ERR_PARAM;
    }
    if (h->is_started == 0U) {
        return ZDT_ERR_INIT;
    }
    if ((n == 0U) || (n > ZDT_GROUP_MAX)) {
        return ZDT_ERR_PARAM;
    }
    // 1.填写组速度请求
    req.kind                 = MH_REQ_SPEED;
    req.data.speed.motor_num = n;
    req.data.speed.accel     = accel;
    for (i = 0U; i < n; i++) {
        req.data.speed.speed[i] = speed[i];
    }
    // 2.非阻塞投递发送请求
    return h->os_queue->pf_q_put(h->tx_queue, &req, 0U);
}

/**
 * @brief  投递使能/失能请求
 * @param  h   handler 对象
 * @param  idx 电机下标
 * @param  on  true=使能
 * @retval ZDT_OK / ZDT_ERR_PARAM / ZDT_ERR_INIT / ZDT_ERR_RES
 */
zdt_status_t mh_enable(motor_handler_t *h, uint8_t idx, bool on)
{
    mh_tx_req_t req; /* 使能请求 */

    // 0.参数合法性检查：handler、启动状态与电机下标
    if (h == NULL) {
        return ZDT_ERR_PARAM;
    }
    if (h->is_started == 0U) {
        return ZDT_ERR_INIT;
    }
    if (idx >= ZDT_GROUP_MAX) {
        return ZDT_ERR_PARAM;
    }
    // 1.填写使能请求
    req.kind                  = MH_REQ_ENABLE;
    req.data.enable.motor_idx = idx;
    req.data.enable.enabled   = on ? 1U : 0U;
    // 2.非阻塞投递发送请求
    return h->os_queue->pf_q_put(h->tx_queue, &req, 0U);
}

/**
 * @brief  投递单机位置请求
 * @param  h     handler 对象
 * @param  idx   电机下标
 * @param  mode  位置模式
 * @param  pulse 有符号目标脉冲
 * @param  rpm/accel 速度幅值 / 加速度
 * @retval ZDT_OK / ZDT_ERR_PARAM / ZDT_ERR_INIT / ZDT_ERR_RES
 */
zdt_status_t mh_pos(motor_handler_t *h, uint8_t idx, zdt_pos_mode_t mode,
                    int32_t pulse, uint16_t rpm, uint8_t accel)
{
    mh_tx_req_t req; /* 位置请求 */

    // 0.参数合法性检查：handler、启动状态与电机下标
    if (h == NULL) {
        return ZDT_ERR_PARAM;
    }
    if (h->is_started == 0U) {
        return ZDT_ERR_INIT;
    }
    if (idx >= ZDT_GROUP_MAX) {
        return ZDT_ERR_PARAM;
    }
    // 1.填写位置请求
    req.kind                    = MH_REQ_POS;
    req.data.position.motor_idx = idx;
    req.data.position.mode      = mode;
    req.data.position.pulse     = pulse;
    req.data.position.rpm       = rpm;
    req.data.position.accel     = accel;
    // 2.非阻塞投递发送请求
    return h->os_queue->pf_q_put(h->tx_queue, &req, 0U);
}

/**
 * @brief  投递单机回零请求
 * @param  h    handler 对象
 * @param  idx  电机下标
 * @param  mode 回零模式
 * @retval ZDT_OK / ZDT_ERR_PARAM / ZDT_ERR_INIT / ZDT_ERR_RES
 */
zdt_status_t mh_home(motor_handler_t *h, uint8_t idx, zdt_home_mode_t mode)
{
    mh_tx_req_t req; /* 回零请求 */

    // 0.参数合法性检查：handler、启动状态与电机下标
    if (h == NULL) {
        return ZDT_ERR_PARAM;
    }
    if (h->is_started == 0U) {
        return ZDT_ERR_INIT;
    }
    if (idx >= ZDT_GROUP_MAX) {
        return ZDT_ERR_PARAM;
    }
    // 1.填写回零请求
    req.kind                = MH_REQ_HOME;
    req.data.home.motor_idx = idx;
    req.data.home.mode      = mode;
    // 2.非阻塞投递发送请求
    return h->os_queue->pf_q_put(h->tx_queue, &req, 0U);
}

/**
 * @brief  投递定时上报配置请求
 * @param  h         handler 对象
 * @param  period_ms 上报周期 ms，0 关闭
 * @retval ZDT_OK / ZDT_ERR_PARAM / ZDT_ERR_INIT / ZDT_ERR_RES
 */
zdt_status_t mh_report(motor_handler_t *h, uint16_t period_ms)
{
    mh_tx_req_t req; /* 上报请求 */

    // 0.参数合法性检查：handler 指针与启动状态
    if (h == NULL) {
        return ZDT_ERR_PARAM;
    }
    if (h->is_started == 0U) {
        return ZDT_ERR_INIT;
    }
    // 1.填写上报请求
    req.kind                  = MH_REQ_REPORT;
    req.data.report.period_ms = period_ms;
    // 2.非阻塞投递发送请求
    return h->os_queue->pf_q_put(h->tx_queue, &req, 0U);
}

/**
 * @brief  投递多机读位置请求（00 AA 一帧逐台 0x36，仅入队）
 * @param  h handler 对象
 * @retval ZDT_OK / ZDT_ERR_PARAM / ZDT_ERR_INIT / ZDT_ERR_RES
 */
zdt_status_t mh_request_pos_all(motor_handler_t *h)
{
    mh_tx_req_t req; /* 多机读请求 */

    // 0.参数合法性检查：handler 指针与启动状态
    if (h == NULL) {
        return ZDT_ERR_PARAM;
    }
    if (h->is_started == 0U) {
        return ZDT_ERR_INIT;
    }
    // 1.填写多机读请求（仅类型标签）
    req.kind = MH_REQ_READ_POS;
    // 2.非阻塞投递发送请求
    return h->os_queue->pf_q_put(h->tx_queue, &req, 0U);
}

/**
 * @brief  线程安全读取某电机当前位置脉冲（里程计用）
 * @param  h   handler 对象
 * @param  idx 电机下标
 * @param  out 输出位置脉冲
 * @retval ZDT_OK / ZDT_ERR_PARAM / ZDT_ERR_INIT
 */
zdt_status_t mh_get_pos(motor_handler_t *h, uint8_t idx, int64_t *out)
{
    zdt_motor_t *m = NULL; /* 目标电机 */

    // 0.参数合法性检查：handler、输出指针、状态与下标
    if ((h == NULL) || (out == NULL)) {
        return ZDT_ERR_PARAM;
    }
    if (h->is_inited == 0U) {
        return ZDT_ERR_INIT;
    }
    m = motor_at(h, idx);
    if (m == NULL) {
        return ZDT_ERR_PARAM;
    }
    // 1.持锁读取位置脉冲
    h->os_lock->pf_enter();
    *out = m->pos_pulse;
    h->os_lock->pf_exit();
    return ZDT_OK;
}

/**
 * @brief  喂入一帧接收数据（适配层 rx 回调里调，ISR 上下文）
 * @param  h     handler 对象
 * @param  motor 来源电机
 * @param  data  帧缓冲
 * @param  len   帧长度
 * @retval ZDT_OK / ZDT_ERR_PARAM / ZDT_ERR_INIT / ZDT_ERR_RES
 */
zdt_status_t mh_feed_rx(motor_handler_t *h, zdt_motor_t *motor,
                        const uint8_t *data, uint16_t len)
{
    mh_rx_evt_t evt;     /* 帧事件 */
    uint16_t cnt = 0U;   /* 实拷贝字节数 */

    // 0.参数合法性检查：handler、电机、帧缓存与启动状态
    if ((h == NULL) || (motor == NULL) || (data == NULL)) {
        return ZDT_ERR_PARAM;
    }
    if (h->is_started == 0U) {
        return ZDT_ERR_INIT;
    }
    // 1.限制长度并拷贝接收帧
    cnt = (len > MH_RX_FRAME_MAX) ? MH_RX_FRAME_MAX : len;
    evt.motor = motor;
    evt.len   = cnt;
    (void)memcpy(evt.data, data, cnt);
    // 2.从 ISR 非阻塞投递接收事件
    /* ISR 上下文：超时 0 非阻塞入队 */
    return h->os_queue->pf_q_put(h->rx_queue, &evt, 0U);
}
