/**
 * @file    app_stairs.c
 * @brief   阶梯三层横移状态机：按里程计 y 分层，暂停/恢复与层事件解耦
 * @note    - 沿地图 +y 反向慢速横移，层边界待实机标定
 *          - 横移中每周期先分发命令再查位姿；暂停期间不判边界
 *          - 切层后 Mission 异步切换视觉，切换期间车仍在走，属识别盲区
 */

#include "app_stairs.h"

#include "cmsis_os2.h"

#include "app_link.h"
#include "app_util.h"
#include "chassis_align.h"
#include "chassis_service.h"
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

/* 横移参数：车体系 vy 为负即沿地图 +y 反向前进 */
#define ST_VY_MMS      (-70.0f) /* 横移速度，车体系 vy，mm/s */
#define ST_POLL_MS     10U      /* 横移中命令/位姿轮询周期，ms */
#define ST_HIGH_Y_MM   2700     /* 低层结束、高层起点 y，mm（暂定） */
#define ST_MID_Y_MM    2420     /* 高层结束、中层起点 y，mm（暂定） */
#define ST_END_Y_MM    2200     /* 中层结束 y，mm（暂定） */

/* 运行上下文：暴露给命令钩子，使暂停与层事件互不阻塞 */
typedef struct {
    uint16_t               req_id;    /* 阶梯阶段请求编号 */
    chassis_command_type_t layer_evt; /* 当前层事件 CHASSIS_CMD_STAIR_xxx */
    uint8_t                moving;    /* 1=低层已放行，全程横移不再清零 */
    uint8_t                paused;    /* 1=收到 STAIR_STOP 尚未恢复 */
    uint8_t                cam_ready; /* 1=本层事件已被 CAM_READY 确认 */
} stair_ctx_t;

/* 阶梯三层：层起点事件 + 本层结束 y 边界 */
typedef struct {
    chassis_command_type_t evt;      /* 层起点事件 */
    int16_t                end_y_mm; /* 本层结束 y，mm */
} stair_layer_t;

static const stair_layer_t g_layers[] = {
    { CHASSIS_CMD_STAIR_LOW,  ST_HIGH_Y_MM },
    { CHASSIS_CMD_STAIR_HIGH, ST_MID_Y_MM  },
    { CHASSIS_CMD_STAIR_MID,  ST_END_Y_MM  },
};

#define ST_LAYER_NUM  (sizeof(g_layers) / sizeof(g_layers[0]))

static uint8_t stair_hook(const chassis_mission_command_t *cmd, void *ctx);

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
        case MISSION_CMD_CAM_READY:
            sc->cam_ready = 1U;
            break;
        case MISSION_CMD_STAIR_STOP:
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
            if (sc->moving != 0U) {
                (void)csvc_free(0.0f, ST_VY_MMS, 0.0f);
            }
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

/** @copydoc stairs_sweep */
app_status_t stairs_sweep(uint16_t req_id)
{
    stair_ctx_t  ctx;                        /* 运行上下文 */
    map_point_t  pos = { 0, 0 };             /* 里程计坐标 */
    float        yaw = 0.0f;                 /* 里程计航向，附带量 */
    uint32_t     poll = util_ms_ticks(ST_POLL_MS); /* 轮询 tick 数 */
    uint8_t      i;                          /* 层索引 */

    ctx.req_id = req_id;
    ctx.paused = 0U;
    ctx.moving = 0U;
    for (i = 0U; i < ST_LAYER_NUM; i++) {
        ctx.layer_evt = g_layers[i].evt;
        ctx.cam_ready = 0U;
        (void)link_post(ctx.layer_evt, req_id, 1U);
        if (ctx.moving == 0U) {
            /* 仅低层起点等放行：机械臂需先到识别姿态 */
            while (ctx.cam_ready == 0U) {
                (void)link_poll(stair_hook, &ctx, osWaitForever);
            }
            ctx.moving = 1U;
            if ((ctx.paused == 0U) &&
                (csvc_free(0.0f, ST_VY_MMS, 0.0f) != CSVC_OK)) {
                ST_LOGE("stair move fail");
                return APP_ERR;
            }
        }
        for (;;) {
            (void)link_poll(stair_hook, &ctx, poll);
            if (ctx.paused != 0U) {
                continue;
            }
            if (csvc_get_pose(&pos, &yaw) != CSVC_OK) {
                (void)align_stop();
                ST_LOGE("stair pose fail");
                return APP_ERR;
            }
            if (pos.y_mm <= g_layers[i].end_y_mm) {
                break;
            }
        }
        ST_LOGI("stair layer %u done y=%d", (unsigned)i, (int)pos.y_mm);
    }
    if (align_stop() != ALIGN_OK) {
        return APP_ERR;
    }
    (void)link_post(CHASSIS_CMD_STAIRS_FINISHED, req_id, 1U);
    return APP_OK;
}
