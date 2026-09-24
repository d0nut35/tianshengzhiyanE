/**
 * @file    app_main.c
 * @brief   应用入口：上电时序 + 底盘任务主流程
 * @note    - 时序：hwt101 上电配置 → system_assembly_init → csvc_init → 位姿
 *          - 主流程经 app_link 收 Mission 阶段命令，到位后原样带回 request_id
 *          - 动作原语在 chassis_align，点表在 app_route，横移在 app_stairs
 */

#include "app_main.h"

#include "cmsis_os2.h"

#include "app_link.h"
#include "app_route.h"
#include "app_stairs.h"
#include "app_util.h" /* [lyx] 小圆盘绕行使用统一tick换算。 */
#include "chassis_align.h"
#include "chassis_service.h"
#include "hwt101_adaption.h"
#include "system_assembly.h"

/* ===== 调试日志：0=不编译进固件，1=经 RTT 输出 ===== */
#ifndef APP_LOG_EN
#define APP_LOG_EN   0
#endif
#if APP_LOG_EN
#include "cat_log.h"
#define APP_LOGI(fmt, ...)  LOGI("[app] " fmt, ##__VA_ARGS__)
#define APP_LOGE(fmt, ...)  LOGE("[app] " fmt, ##__VA_ARGS__)
#else
#define APP_LOGI(fmt, ...)  do {} while (0)
#define APP_LOGE(fmt, ...)  do {} while (0)
#endif

#define APP_TASK_STACK    2048U   /* 底盘任务栈字节 */
#define APP_BOOT_POLL_MS  10U     /* 等应用启动标志的轮询周期，ms */
#define APP_CSVC_WAIT_MS  1000U   /* 等底盘服务稳定，ms */

#define APP_HOME_WAIT_MS  3000U   /* 回原点后停留，ms（暂定） */
#define APP_IDLE_MS       1000U   /* 流程结束后的空转周期，ms */

/* 上电初始位姿（世界系，按场地标定） */
#define APP_START_X_MM    1200
#define APP_START_Y_MM    350
#define APP_START_YAW_DEG 0.0f

/* 阶梯末层以 1 号灰度离线停车，该处 y 按实机标定；x 与航向沿用里程计 */
#define APP_STAIR_END_Y_MM 2144

/* [lyx] 绕圆柱1.15圈：定半径画圆跑固定时长后停车，时长以 2πR/v 理论值起步、实机标定 */
#define APP_CYL_SETTLE_MS 1000U     /* 到绕圈起点后等底盘稳定，ms */
#define APP_CYL_V_MMS     180.0f    /* [lyx] 绕圈线速度，mm/s */
#define APP_CYL_R_MM      (-318.0f) /* [lyx] 底盘中心轨迹半径，符号定转向，mm */
#define APP_CYL_ARC_MS    12765U    /* [lyx] 180mm/s、半径318mm理论绕1.15圈 */
/* [lyx] 小圆盘绕行期间短周期接收视觉触发后的停车和恢复命令。 */
#define APP_CYL_POLL_MS   10U
/* [lyx] 绕完后沿车体 -x 退出圆柱障碍膨胀区，参数待实机标定。 */
#define APP_CYL_EXIT_VX_MMS (-100.0f) /* [lyx] 车体 -x 平移速度，mm/s */
#define APP_CYL_EXIT_MS     1800U     /* [lyx] 平移时长，理论位移约180mm */

/* 仓库横移：1 号位找线标定后按里程计 y 在 1~4 号位间开环横移，不再找线；
 * [lyx] 航向 180° 时车体系 vy 为负即沿地图 +y 前进，反向时取相反速度。 */
#define APP_DEPOT_VY_MMS  (-70.0f)  /* 正向横移速度，车体系 vy，mm/s */
#define APP_DEPOT_POLL_MS 10U       /* 横移中位姿轮询周期，ms */

static osThreadId_t g_task = NULL;  /* 底盘任务 */
static uint8_t      g_app_up = 0U;  /* 应用启动标志，1=资源就绪 */

/* 仓库点位表：Mission 命令 → 目标 y → 到位回执；1 号位由 route_go 到达 */
static const struct {
    mission_command_type_t cmd;  /* 期望的 Mission 仓库命令 */
    int16_t                y_mm; /* 该工作位地图 y，mm */
    chassis_command_type_t rsp;  /* 到位后回执的事件类型 */
} g_depot_tbl[] = {
    { MISSION_CMD_GO_DEPOT_1, 2208, CHASSIS_CMD_DEPOT_1_READY },
    { MISSION_CMD_GO_DEPOT_2, 2407, CHASSIS_CMD_DEPOT_2_READY },
    { MISSION_CMD_GO_DEPOT_3, 2608, CHASSIS_CMD_DEPOT_3_READY },
    { MISSION_CMD_GO_DEPOT_4, 2793, CHASSIS_CMD_DEPOT_4_READY },
};

#define DEPOT_TBL_NUM  (sizeof(g_depot_tbl) / sizeof(g_depot_tbl[0]))

static const osThreadAttr_t g_task_attr = {
    .name       = "chassis",
    .stack_size = APP_TASK_STACK,
    .priority   = osPriorityNormal,
};

/* [lyx] 小圆盘绕行上下文；暂停时间不计入完整一圈的运动时长。 */
typedef struct {
    uint16_t req_id;
    uint8_t  paused;
    uint8_t  failed;
} small_disc_ctx_t;

static void app_task(void *arg);
/* [lyx] 小圆盘命令钩子和可暂停绕行流程。 */
static uint8_t small_disc_hook(
    const chassis_mission_command_t *cmd,
    void *ctx);
static app_status_t small_disc_round(uint16_t req_id);
static app_status_t depot_shift(int16_t y_mm);
static uint8_t depot_hook(const chassis_mission_command_t *cmd, void *ctx);

/**
 * @brief  [lyx] 处理小圆盘绕行中的停车和恢复命令
 * @param  cmd Mission 命令
 * @param  ctx 小圆盘运行上下文
 * @retval 1=已消费小圆盘命令 / 0=交给链路默认处理
 */
static uint8_t small_disc_hook(
    const chassis_mission_command_t *cmd,
    void *ctx)
{
    small_disc_ctx_t *disc = (small_disc_ctx_t *)ctx;
    uint8_t ok = 1U;

    /* [lyx] 全局STOP交给链路停车，同时结束当前绕行或退出动作。 */
    if (cmd->type == MISSION_CMD_STOP) {
        disc->failed = 1U;
        return 0U;
    }
    if ((cmd->type != MISSION_CMD_SMALL_DISC_STOP) &&
        (cmd->type != MISSION_CMD_SMALL_DISC_RESUME)) {
        return 0U;
    }
    if (cmd->request_id != disc->req_id) {
        return 1U;
    }
    if (cmd->type == MISSION_CMD_SMALL_DISC_STOP) {
        if ((disc->paused == 0U) && (align_stop() != ALIGN_OK)) {
            ok = 0U;
        }
        if (ok != 0U) {
            disc->paused = 1U;
        }
        if (link_post(CHASSIS_CMD_SMALL_DISC_PAUSED,
                      disc->req_id, ok) != APP_OK) {
            ok = 0U;
        }
    } else {
        if ((disc->paused != 0U) &&
            (csvc_arc(APP_CYL_V_MMS, APP_CYL_R_MM, false) != CSVC_OK)) {
            ok = 0U;
        }
        if (ok != 0U) {
            disc->paused = 0U;
        }
        if (link_post(CHASSIS_CMD_SMALL_DISC_RESUMED,
                      disc->req_id, ok) != APP_OK) {
            ok = 0U;
        }
    }
    if (ok == 0U) {
        disc->failed = 1U;
    }
    return 1U;
}

/** [lyx] 绕行结束后沿车体 -x 平移，期间继续处理停车命令。 */
static app_status_t small_disc_exit(small_disc_ctx_t *ctx, uint32_t poll_ticks)
{
    uint32_t started = osKernelGetTickCount();

    if (csvc_free(APP_CYL_EXIT_VX_MMS, 0.0f, 0.0f) != CSVC_OK) {
        APP_LOGE("small disc exit fail");
        return APP_ERR;
    }
    while ((osKernelGetTickCount() - started) <
           util_ms_ticks(APP_CYL_EXIT_MS)) {
        (void)link_poll(small_disc_hook, ctx, poll_ticks);
        if ((ctx->paused != 0U) || (ctx->failed != 0U)) {
            (void)align_stop();
            return APP_ERR;
        }
    }
    return (align_stop() == ALIGN_OK) ? APP_OK : APP_ERR;
}

/**
 * @brief  [lyx] 绕小圆盘指定圈数，并允许视觉触发停车和恢复
 * @param  req_id 小圆盘阶段请求编号
 * @retval APP_OK / APP_ERR=画圆、停车、恢复或回执失败
 * @note   只累计底盘实际运动时间，抓球暂停后仍会补完剩余圆周。
 */
static app_status_t small_disc_round(uint16_t req_id)
{
    small_disc_ctx_t ctx;
    uint32_t elapsed = 0U;
    uint32_t arc_ticks = util_ms_ticks(APP_CYL_ARC_MS);
    uint32_t poll_ticks = util_ms_ticks(APP_CYL_POLL_MS);

    ctx.req_id = req_id;
    ctx.paused = 0U;
    ctx.failed = 0U;
    if (csvc_arc(APP_CYL_V_MMS, APP_CYL_R_MM, false) != CSVC_OK) {
        APP_LOGE("small disc arc fail");
        return APP_ERR;
    }
    while ((elapsed < arc_ticks) && (ctx.failed == 0U)) {
        uint32_t started = osKernelGetTickCount();
        uint8_t was_paused = ctx.paused;

        (void)link_poll(small_disc_hook, &ctx, poll_ticks);
        if (was_paused == 0U) {
            elapsed += osKernelGetTickCount() - started;
        }
    }
    if ((align_stop() != ALIGN_OK) || (ctx.failed != 0U)) {
        return APP_ERR;
    }
    return small_disc_exit(&ctx, poll_ticks);
}

/**
 * @brief  [lyx] 根据当前里程计 y 双向横移到指定仓库位置后停车
 * @param  y_mm 目标 y，mm
 * @retval APP_OK / APP_ERR=命令下发或位姿读取失败（已停车）
 * @note   只按里程计 y 判停，x 与航向不闭环，依赖 1 号位刚标定过的位姿
 */
static app_status_t depot_shift(int16_t y_mm)
{
    map_point_t pos = { 0, 0 };  /* 里程计坐标 */
    float       yaw = 0.0f;      /* 里程计航向，附带量 */
    float       vy_mms;          /* [lyx] 按目标方向选择的车体系横移速度 */

    if (csvc_get_pose(&pos, &yaw) != CSVC_OK) {
        return APP_ERR;
    }
    if (pos.y_mm == y_mm) {
        return APP_OK;
    }
    vy_mms = (pos.y_mm < y_mm) ? APP_DEPOT_VY_MMS : -APP_DEPOT_VY_MMS;
    if (csvc_free(0.0f, vy_mms, 0.0f) != CSVC_OK) {
        return APP_ERR;
    }
    do {
        osDelay(APP_DEPOT_POLL_MS);
        if (csvc_get_pose(&pos, &yaw) != CSVC_OK) {
            (void)align_stop();
            return APP_ERR;
        }
    } while (((vy_mms < 0.0f) && (pos.y_mm < y_mm)) ||
             ((vy_mms > 0.0f) && (pos.y_mm > y_mm)));
    return (align_stop() == ALIGN_OK) ? APP_OK : APP_ERR;
}

/* 仓库命令分发上下文 */
typedef struct {
    mission_command_type_t cmd; /* 捕获到的仓库命令类型 */
    uint16_t               id;  /* 请求编号，回执时原样带回 */
    uint8_t                got; /* 1=已捕获仓库命令 */
} depot_ctx_t;

/**
 * @brief  link_poll 钩子：捕获仓库命令 GO_DEPOT_1~4 与 DEPOT_OK
 * @param  cmd 出队的 Mission 命令
 * @param  ctx 仓库分发上下文 depot_ctx_t
 * @retval 1=命令已消费 / 0=交由 link_poll 走默认处理
 */
static uint8_t depot_hook(const chassis_mission_command_t *cmd, void *ctx)
{
    depot_ctx_t *depot = (depot_ctx_t *)ctx;

    /* GO_DEPOT_1~DEPOT_OK 在协议中连续排列，区间判断即覆盖全部仓库命令 */
    if ((cmd->type < MISSION_CMD_GO_DEPOT_1) ||
        (cmd->type > MISSION_CMD_DEPOT_OK)) {
        return 0U;
    }
    depot->cmd = cmd->type;
    depot->id  = cmd->request_id;
    depot->got = 1U;
    return 1U;
}

/**
 * @brief  应用主任务：按 Mission 命令串行执行底盘任务点
 * @param  arg 未用
 * @note   阶段失败以 is_ready=0 回执，由 Mission 决定停机，底盘不自行中止
 */
static void app_task(void *arg)
{
    uint16_t    id = CHASSIS_MISSION_REQUEST_ID_INVALID; /* 阶段请求编号 */
    uint8_t     ok;                   /* 阶段结果，1=成功 */
    map_point_t pos = { 0, 0 };       /* 阶梯线尾停车处里程计坐标 */
    float       yaw = 0.0f;           /* 阶梯线尾停车处航向，deg */
    uint8_t     i;                    /* 仓库点位表索引 */

    (void)arg;
    while (g_app_up == 0U) {
        osDelay(APP_BOOT_POLL_MS);
    }
    osDelay(APP_CSVC_WAIT_MS);
    link_handshake();

    if (link_wait(MISSION_CMD_GO_PLATFORM, &id, osWaitForever) == APP_OK) {
        ok = (route_go(ROUTE_PLATFORM) == APP_OK) ? 1U : 0U;
        (void)link_post(CHASSIS_CMD_PLATFORM_READY, id, ok);
    }
    if (link_wait(MISSION_CMD_GO_STAIRS, &id, osWaitForever) == APP_OK) {
        ok = (route_go(ROUTE_STAIRS) == APP_OK) ? 1U : 0U;
        (void)link_post(CHASSIS_CMD_STAIRS_READY, id, ok);
        /* 阶梯起始位即低层起点：层事件与 CAM_READY 放行由 sweep 统一处理 */
        if ((stairs_sweep(id) == APP_OK) &&
            (csvc_get_pose(&pos, &yaw) == CSVC_OK)) {
            /* 线尾停车处 y 已知，只修 y 消除横移段里程计累计误差 */
            APP_LOGI("stairs end y=%d -> %d", (int)pos.y_mm,
                     APP_STAIR_END_Y_MM);
            pos.y_mm = (int16_t)APP_STAIR_END_Y_MM;
            (void)csvc_set_pose(pos, yaw);
        }
    }
    /* [lyx] 阶梯结束后等待上层放行，再进入小圆盘识别和可暂停绕行流程。 */
    if (link_wait(MISSION_CMD_GO_SMALL_DISC, &id, osWaitForever) == APP_OK) {
        ok = ((route_go(ROUTE_CYL_PRE) == APP_OK) &&
              (route_go(ROUTE_CYL) == APP_OK)) ? 1U : 0U;
        if (ok != 0U) {
            osDelay(APP_CYL_SETTLE_MS);
        }
        (void)link_post(CHASSIS_CMD_SMALL_DISC_READY, id, ok);
        if ((ok != 0U) &&
            (link_wait(MISSION_CMD_SMALL_DISC_START,
                       &id, osWaitForever) == APP_OK)) {
            ok = (small_disc_round(id) == APP_OK) ? 1U : 0U;
            (void)link_post(CHASSIS_CMD_SMALL_DISC_FINISHED, id, ok);
        }
    }

    /* [lyx] 小圆盘退出姿态完成后，等待上层下发已有的仓库1号位命令。 */
    (void)link_wait(MISSION_CMD_GO_DEPOT_1, &id, osWaitForever);
    /* 仓库：1 号位找线标定作为横移基准，到位后由 Mission 逐点驱动横移；
     * 1 号位失败则位姿不可信，不盲横移直接回家 */
    ok = (route_go(ROUTE_DEPOT) == APP_OK) ? 1U : 0U;
    (void)link_post(CHASSIS_CMD_DEPOT_1_READY, id, ok);
    if (ok != 0U) {
        depot_ctx_t dctx; /* 仓库命令分发上下文 */

        /* 收到 DEPOT_OK 才退出，其余仓库命令查表横移并回执 */
        for (;;) {
            dctx.got = 0U;
            while (dctx.got == 0U) {
                (void)link_poll(depot_hook, &dctx, osWaitForever);
            }
            if (dctx.cmd == MISSION_CMD_DEPOT_OK) {
                /* [lyx] 回家到位回执沿用DEPOT_OK请求编号。 */
                id = dctx.id;
                break;
            }
            ok = 0U;
            for (i = 0U; i < DEPOT_TBL_NUM; i++) {
                if (g_depot_tbl[i].cmd == dctx.cmd) {
                    /* [lyx] 首次1号位由route_go标定，后续1~4号位统一双向横移。 */
                    ok = (depot_shift(g_depot_tbl[i].y_mm) == APP_OK)
                         ? 1U : 0U;
                    (void)link_post(g_depot_tbl[i].rsp, dctx.id, ok);
                    break;
                }
            }
            if (ok == 0U) {
                APP_LOGE("depot cmd %u fail", (unsigned)dctx.cmd);
            }
        }
    }
    /* [lyx] 回家后显式保持零速度，确认停车成功后再向Mission回执。 */
    ok = (route_go(ROUTE_HOME) == APP_OK) ? 1U : 0U;
    if (align_stop() != ALIGN_OK) {
        ok = 0U;
    }
    (void)link_post(CHASSIS_CMD_HOME_READY, id, ok);
    osDelay(APP_HOME_WAIT_MS);

    /* 当前 Mission 流程到此结束，任务保持低功耗等待。 */
    for (;;) {
        osDelay(APP_IDLE_MS);
    }
}

app_status_t app_init(void)
{
    map_point_t start; /* 上电初始位姿坐标 */

    /* 0) 陀螺仪上电配置（失败只记日志，后续仍可软件标定航向） */
    if (hwt101_adp_boot_cfg() != HWT101_OK) {
        APP_LOGE("imu cfg fail");
    }
    /* 1) 装配电机 + 陀螺仪子系统 */
    if (system_assembly_init() != ZDT_OK) {
        APP_LOGE("assembly init fail");
        return APP_ERR;
    }
    APP_LOGI("assembly init success");
    /* 2) 起底盘服务（接入 chassis + map + 20ms 控制任务） */
    if (csvc_init() != CSVC_OK) {
        APP_LOGE("chassis service init fail");
        return APP_ERR;
    }
    APP_LOGI("chassis service init success");
    /* 3) 里程计对齐世界系初始位姿 */
    start.x_mm = (int16_t)APP_START_X_MM;
    start.y_mm = (int16_t)APP_START_Y_MM;
    (void)csvc_set_pose(start, APP_START_YAW_DEG);
    /* 4) 起底盘任务 */
    g_task = osThreadNew(app_task, NULL, &g_task_attr);
    if (g_task == NULL) {
        APP_LOGE("app task fail");
        return APP_ERR;
    }
    g_app_up = 1U;
    APP_LOGI("app up");
    return APP_OK;
}
