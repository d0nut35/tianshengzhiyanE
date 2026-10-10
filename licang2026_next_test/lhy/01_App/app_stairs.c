/**
 * @file    app_stairs.c
 * @brief   阶梯三层横移状态机：按里程计 y 分层，暂停/恢复与层事件解耦
 * @note    - 沿地图 -y 慢速横移，层边界待实机标定
 *          - 末层不看 y，以 1 号灰度连续 5 次离线（高电平）作为线尾停车条件
 *          - 镜像侧航向反向：车体系 vy 取反，线尾灰度换到对称位置的 6 号
 *          - 横移中每周期先分发命令再查边界；暂停期间不判边界
 *          - 切层时停车等待视觉就绪，再按 ST_SETTLE_MS 稳定后继续横移 [lyx]
 *          - 2/5 压线、3/4 在线外为基准，灰度 P 调节叠加车体系旋转
 */

#include "app_stairs.h"

#include "cmsis_os2.h"

#include "app_link.h"
#include "app_route.h"  /* route_side / route_lat_sign：场地侧 */
#include "app_util.h"
#include "chassis_align.h"
#include "chassis_service.h"
#include "lsensor/lsensor_handler.h"
#include "map.h"        /* map_point_t：里程计位姿 */

/* ===== 调试日志：0=不编译进固件，1=经 RTT 输出 ===== */
#ifndef APP_LOG_EN
#define APP_LOG_EN   0
#endif
#if APP_LOG_EN
#include "cat_log.h"
#define ST_LOGI(fmt, ...)  LOGI("[stair] " fmt, ##__VA_ARGS__)
#define ST_LOGE(fmt, ...)  LOGE("[stair] " fmt, ##__VA_ARGS__)
#else
#define ST_LOGI(fmt, ...)  do {} while (0)
#define ST_LOGE(fmt, ...)  do {} while (0)
#endif

/* 横移参数：默认侧航向 0°，车体系 vy 为负即沿地图 -y 前进；
 * 镜像侧航向 180°，经 route_lat_sign 取反后仍沿地图 -y */
#define ST_VY_MMS      (-100.0f) /* 横移速度，车体系 vy，mm/s（默认侧取值） */
#define ST_POLL_MS     10U      /* 横移中命令/位姿轮询周期，ms */
#define ST_END_HIGH_N  5U       /* 线尾确认所需连续高电平次数 */
#define ST_LINE_KP     3.0f     /* 单位归一化灰度偏差对应角速度，deg/s */
#define ST_W_MAX       3.0f     /* 纠偏角速度绝对值上限，deg/s */
#define ST_ERR_SCALE   0.5f     /* 两对差分之和 [-2,2] 归一化至 [-1,1] */
#define ST_LINE_MASK   0x1EU    /* 六路快照中 2~5 号对应的位 */
#define ST_SETTLE_MS   500U     /* 切层视觉就绪后的原地稳定时间，ms [lyx] */
#define ST_HIGH_Y_MM   2750     /* 低层结束、高层起点 y，mm [lyx] */
#define ST_MID_Y_MM    2400     /* 高层结束、中层起点 y，mm [lyx] */
#define ST_BLUE_HIGH_Y_MM 2780  /* 蓝方实际中层结束、高层起点 y，mm；底盘首段仍发LOW */
#define ST_BLUE_MID_Y_MM  2400  /* 蓝方实际高层结束、低层起点 y，mm；末段仍发MID */
#define ST_END_ID      1U       /* 线尾检测灰度板上序号，离线即中层结束 */
#define ST_END_ID_MIRROR 6U     /* 镜像侧线尾灰度：与 1 号关于车体 x 轴对称 */

/* 运行上下文：暴露给命令钩子，使暂停与层事件互不阻塞 */
typedef struct {
    uint16_t               req_id;    /* 阶梯阶段请求编号 */
    chassis_command_type_t layer_evt; /* 当前层事件 CHASSIS_CMD_STAIR_xxx */
    float                  vy_mms;    /* 本侧横移速度，车体系 vy，mm/s */
    float                  last_wz;   /* 最近一次非零纠偏，整次扫描内保留 */
    uint8_t                end_id;    /* 本侧线尾检测灰度序号 */
    uint8_t                high_cnt;  /* 线尾连续高电平计数，暂停后重新确认 */
    uint8_t                moving;    /* 1=低层已放行，全程横移不再清零 */
    uint8_t                paused;    /* 1=收到 STAIR_STOP 尚未恢复 */
    uint8_t                cam_ready; /* 1=本层事件已被 CAM_READY 确认 */
    uint8_t                settling;  /* 1=切层后原地稳定，恢复时暂不横移 [lyx] */
    uint8_t                stopped;   /* 全局 STOP 锁存，禁止下周期重新启动 */
} stair_ctx_t;

/* 层结束判定方式 */
typedef enum {
    END_BY_Y = 0,  /* 里程计 y 到达 end_y_mm */
    END_BY_LINE,   /* ST_END_ID 号灰度离线（高电平） */
} stair_end_t;

/* 阶梯三层：层起点事件 + 本层结束判定 */
typedef struct {
    chassis_command_type_t evt;      /* 层起点事件 */
    stair_end_t            end;      /* 结束判定方式 */
    int16_t                end_y_mm; /* END_BY_Y 的结束 y，mm */
} stair_layer_t;

/* 默认地图层配置。 */
static const stair_layer_t g_layers[] = {
    { CHASSIS_CMD_STAIR_LOW,  END_BY_Y,    ST_HIGH_Y_MM },
    { CHASSIS_CMD_STAIR_HIGH, END_BY_Y,    ST_MID_Y_MM  },
    { CHASSIS_CMD_STAIR_MID,  END_BY_LINE, 0            },
};

#define ST_LAYER_NUM  (sizeof(g_layers) / sizeof(g_layers[0]))

/* 蓝方独立坐标：底盘段号仍LOW/HIGH/MID，Mission映射为实际中/高/低；此处不得重复交换事件。 */
static const stair_layer_t g_layers_blue[ST_LAYER_NUM] = {
    { CHASSIS_CMD_STAIR_LOW,  END_BY_Y,    ST_BLUE_HIGH_Y_MM },
    { CHASSIS_CMD_STAIR_HIGH, END_BY_Y,    ST_BLUE_MID_Y_MM  },
    { CHASSIS_CMD_STAIR_MID,  END_BY_LINE, 0                 },
};

static uint8_t stair_hook(const chassis_mission_command_t *cmd, void *ctx);
static app_status_t layer_done(const stair_layer_t *lay, stair_ctx_t *sc,
                               uint8_t *done);

/**
 * @brief  按两对灰度差分下发平移和限幅 P 纠偏
 * @param  sc 本次扫描上下文，保存最近非零纠偏供全丢线时沿用
 * @retval APP_OK / APP_ERR=快照无效、丢线无历史方向或命令失败
 * @note   H 为高电平：e=(H5-H2+H3-H4)/2；负角速度顺时针。
 *         3/4 位于线外侧，其高电平差分与 2/5 采用相反符号。
 */
static app_status_t stair_drive(stair_ctx_t *sc)
{
    uint8_t levels = lsh_get_mask(); /* 同一次采样的电平快照 */
    int8_t err;                      /* 两对传感器的有符号差分和 */
    float wz;                        /* 限幅后的纠偏角速度，deg/s */

    if (levels == LSH_MASK_INVALID) {
        return APP_ERR;
    }
    if ((levels & ST_LINE_MASK) == ST_LINE_MASK) {
        if (sc->last_wz == 0.0f) {
            return APP_ERR;
        }
        wz = sc->last_wz;
    } else {
        err = (int8_t)((levels >> 4U) & 1U) -
              (int8_t)((levels >> 1U) & 1U) +
              (int8_t)((levels >> 2U) & 1U) -
              (int8_t)((levels >> 3U) & 1U);
        wz = ST_LINE_KP * ST_ERR_SCALE * (float)err;
        if (wz > ST_W_MAX) {
            wz = ST_W_MAX;
        } else if (wz < -ST_W_MAX) {
            wz = -ST_W_MAX;
        }
    }
    /* 车体传感器与旋转符号不随红蓝镜像变化，只有平移速度取反。 */
    if (csvc_free(0.0f, sc->vy_mms, wz) != CSVC_OK) {
        return APP_ERR;
    }
    if (wz != 0.0f) {
        sc->last_wz = wz;
    }
    return APP_OK;
}

/**
 * @brief  阶梯段命令钩子：暂停/恢复/放行非阻塞处理，其余交默认分发
 * @param  cmd 出队的 Mission 命令
 * @param  ctx 阶梯运行上下文
 * @retval 1=本命令已消费 / 0=交默认处理（STOP 与未知命令）
 * @note   低层等 CAM_READY 时收到 STOP 直接回 PAUSE（车已停）；
 *         RESUME 时本层事件尚未被 CAM_READY 确认则重发，因 Mission 在
 *         WAIT_PAUSE 丢弃层事件（边界与 STAIR_STOP 同时到达时会发生）
 */
static uint8_t stair_hook(const chassis_mission_command_t *cmd, void *ctx)
{
    stair_ctx_t *sc = (stair_ctx_t *)ctx; /* 阶梯运行上下文 */

    switch (cmd->type) {
        case MISSION_CMD_STOP:
            sc->stopped = 1U;
            return 0U; /* 默认分发负责停车和 STOPPED，本函数只锁存退出。 */
        case MISSION_CMD_CAM_READY:
            sc->cam_ready = 1U;
            break;
        case MISSION_CMD_STAIR_STOP:
            sc->high_cnt = 0U;
            if ((sc->moving != 0U) && (sc->paused == 0U)) {
                (void)align_stop();
            }
            sc->paused = 1U;
            (void)link_post(CHASSIS_CMD_STAIR_PAUSE, sc->req_id, 1U);
            break;
        case MISSION_CMD_STAIR_RESUME:
            if (sc->paused == 0U) {
                break;
            }
            sc->paused = 0U;
            /* 恢复只放行，由主循环先判边界、读取灰度再下发速度。 */
            (void)link_post(CHASSIS_CMD_STAIR_RESUME, sc->req_id, 1U);
            if (sc->cam_ready == 0U) {
                (void)link_post(sc->layer_evt, sc->req_id, 1U);
            }
            break;
        default:
            return 0U;
    }
    return 1U;
}

/**
 * @brief  判断当前层是否走到边界
 * @param  lay    当前层表项
 * @param  sc     本侧线尾检测灰度及连续确认计数
 * @param  done   输出 1=已到边界
 * @retval APP_OK / APP_ERR=位姿或灰度读取失败
 * @note   末层需连续 5 次高电平，低电平清零，不受里程计累计误差影响
 */
static app_status_t layer_done(const stair_layer_t *lay, stair_ctx_t *sc,
                               uint8_t *done)
{
    map_point_t pos = { 0, 0 }; /* 里程计坐标 */
    float       yaw = 0.0f;     /* 里程计航向，附带量 */
    uint8_t     on_line = 0U;   /* 线尾灰度压线标志 */

    if (lay->end == END_BY_LINE) {
        if (align_on_line(sc->end_id, &on_line) != ALIGN_OK) {
            return APP_ERR;
        }
        if (on_line != 0U) {
            sc->high_cnt = 0U;
        } else if (sc->high_cnt < ST_END_HIGH_N) {
            sc->high_cnt++;
        }
        *done = (sc->high_cnt >= ST_END_HIGH_N) ? 1U : 0U;
        return APP_OK;
    }
    if (csvc_get_pose(&pos, &yaw) != CSVC_OK) {
        return APP_ERR;
    }
    *done = (pos.y_mm <= lay->end_y_mm) ? 1U : 0U;
    return APP_OK;
}

/** @copydoc stairs_sweep */
app_status_t stairs_sweep(uint16_t req_id)
{
    stair_ctx_t  ctx;                        /* 运行上下文 */
    uint32_t     poll = util_ms_ticks(ST_POLL_MS); /* 轮询 tick 数 */
    uint32_t     settle_start;               /* 切层稳定计时起点 [lyx] */
    uint32_t     settle_ticks = util_ms_ticks(ST_SETTLE_MS); /* 稳定 tick [lyx] */
    uint32_t     drive_tick;                 /* 最近一次纠偏的 OS tick */
    uint8_t      done = 0U;                  /* 本层到边界标志 */
    uint8_t      i;                          /* 层索引 */
    const stair_layer_t *layers = (route_side() == ROUTE_SIDE_MIRROR)
                                  ? g_layers_blue : g_layers; /* 本侧层配置 */

    ctx.req_id = req_id;
    ctx.paused = 0U;
    ctx.moving = 0U;
    ctx.settling = 0U;
    ctx.stopped = 0U;
    ctx.last_wz = 0.0f;
    /* 镜像侧左右互换：vy 取反后仍沿地图 -y，线尾灰度换对称位保持停点 y */
    ctx.vy_mms = route_lat_sign() * ST_VY_MMS;
    ctx.end_id = (route_side() == ROUTE_SIDE_MIRROR) ? ST_END_ID_MIRROR
                                                     : ST_END_ID;
    for (i = 0U; i < ST_LAYER_NUM; i++) {
        ctx.high_cnt = 0U;
        if (ctx.moving != 0U) {
            /* [lyx] 高/中层切入前先停车，消除视觉会话切换期间的盲移。 */
            if (align_stop() != ALIGN_OK) {
                return APP_ERR;
            }
            ctx.settling = 1U;
        }
        ctx.layer_evt = layers[i].evt;
        ctx.cam_ready = 0U;
        (void)link_post(ctx.layer_evt, req_id, 1U);
        /* 每层都等视觉会话就绪；低层还要等机械臂先到识别姿态。 [lyx] */
        while (ctx.cam_ready == 0U) {
            (void)link_poll(stair_hook, &ctx, osWaitForever);
            if (ctx.stopped != 0U) {
                return APP_ERR;
            }
        }
        if (ctx.moving == 0U) {
            ctx.moving = 1U;
        } else {
            /* [lyx] READY 后按现有稳定时间等待，期间处理暂停/恢复命令。 */
            settle_start = osKernelGetTickCount();
            while ((osKernelGetTickCount() - settle_start) < settle_ticks) {
                (void)link_poll(stair_hook, &ctx, poll);
                if (ctx.stopped != 0U) {
                    return APP_ERR;
                }
            }
            ctx.settling = 0U;
        }
        drive_tick = osKernelGetTickCount() - poll;
        for (;;) {
            (void)link_poll(stair_hook, &ctx, poll);
            if (ctx.stopped != 0U) {
                return APP_ERR;
            }
            if (ctx.paused != 0U) {
                continue;
            }
            /* 命令提前唤醒时不重复计数同一周期，线尾仍每 10ms 确认一次。 */
            if ((layers[i].end == END_BY_LINE) &&
                ((osKernelGetTickCount() - drive_tick) < poll)) {
                continue;
            }
            if (layer_done(&layers[i], &ctx, &done) != APP_OK) {
                (void)align_stop();
                ST_LOGE("stair layer %u check fail", (unsigned)i);
                return APP_ERR;
            }
            if (done != 0U) {
                break;
            }
            if ((osKernelGetTickCount() - drive_tick) >= poll) {
                drive_tick = osKernelGetTickCount();
                if (stair_drive(&ctx) != APP_OK) {
                    (void)align_stop();
                    ST_LOGE("stair line control fail");
                    return APP_ERR;
                }
            }
        }
        ST_LOGI("stair layer %u done", (unsigned)i);
    }
    if (align_stop() != ALIGN_OK) {
        return APP_ERR;
    }
    (void)link_post(CHASSIS_CMD_STAIRS_FINISHED, req_id, 1U);
    return APP_OK;
}
