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

/* 暂定配置：协议已有 MISSION_CMD_GO_DEPOT_1~4，但 Mission 尚未下发且无对应
 * READY 事件，仓库与回原点段暂以固定延时替代命令等待，补齐后换 link_wait。 */
#define APP_ARM_WAIT_MS   1000U  /* 等机械臂夹球完成，ms（暂定） */
#define APP_DEPOT_WAIT_MS 3000U   /* 每个仓库工作位停留占位，ms（暂定） */
#define APP_HOME_WAIT_MS  3000U   /* 回原点后停留，ms（暂定） */
#define APP_IDLE_MS       1000U   /* 流程结束后的空转周期，ms */

/* 上电初始位姿（世界系，按场地标定） */
#define APP_START_X_MM    1200
#define APP_START_Y_MM    350
#define APP_START_YAW_DEG 0.0f

/* 阶梯末层以 1 号灰度离线停车，该处 y 按实机标定；x 与航向沿用里程计 */
#define APP_STAIR_END_Y_MM 2144

/* 绕圆柱一圈：定半径画圆跑固定时长后停车，时长以 2πR/v 理论值起步、实机标定 */
#define APP_CYL_SETTLE_MS 1000U     /* 到绕圈起点后等底盘稳定，ms */
#define APP_CYL_V_MMS     200.0f    /* 绕圈线速度，mm/s */
#define APP_CYL_R_MM      (-350.0f) /* 绕圈半径，符号定转向，mm */
#define APP_CYL_ARC_MS    11000U    /* 一圈时长，2π·350/200≈11.0s（暂定） */

/* 仓库横移：1 号位找线标定后沿地图 +y 开环横移到 2~4 号位，不再找线；
 * 航向 180° 时车体系 vy 为负即沿地图 +y 前进 */
#define APP_DEPOT_VY_MMS  (-70.0f)  /* 横移速度，车体系 vy，mm/s */
#define APP_DEPOT_POLL_MS 10U       /* 横移中位姿轮询周期，ms */

static osThreadId_t g_task = NULL;  /* 底盘任务 */
static uint8_t      g_app_up = 0U;  /* 应用启动标志，1=资源就绪 */

/* 仓库 2~4 号工作位 y，按递增排列；1 号位坐标在 app_route 点表 */
static const int16_t g_depot_y[] = { 2403, 2598, 2793 };

#define APP_DEPOT_NUM  (sizeof(g_depot_y) / sizeof(g_depot_y[0]))

static const osThreadAttr_t g_task_attr = {
    .name       = "chassis",
    .stack_size = APP_TASK_STACK,
    .priority   = osPriorityNormal,
};

static void app_task(void *arg);
static app_status_t cyl_round(void);
static app_status_t depot_shift(int16_t y_mm);

/**
 * @brief  绕圆柱一圈：定半径画圆按固定时长跑完后停车
 * @retval APP_OK / APP_ERR=画圆或停车命令被拒
 * @note   需底盘已在绕圈起点静止；不按里程计判圈，只按 APP_CYL_ARC_MS 计时
 */
static app_status_t cyl_round(void)
{
    osDelay(APP_CYL_SETTLE_MS);
    if (csvc_arc(APP_CYL_V_MMS, APP_CYL_R_MM, false) != CSVC_OK) {
        APP_LOGE("cyl arc fail");
        return APP_ERR;
    }
    osDelay(APP_CYL_ARC_MS);
    return (align_stop() == ALIGN_OK) ? APP_OK : APP_ERR;
}

/**
 * @brief  沿地图 +y 开环横移到指定 y 后停车
 * @param  y_mm 目标 y，mm，需大于当前 y
 * @retval APP_OK / APP_ERR=命令下发或位姿读取失败（已停车）
 * @note   只按里程计 y 判停，x 与航向不闭环，依赖 1 号位刚标定过的位姿
 */
static app_status_t depot_shift(int16_t y_mm)
{
    map_point_t pos = { 0, 0 };  /* 里程计坐标 */
    float       yaw = 0.0f;      /* 里程计航向，附带量 */

    if (csvc_free(0.0f, APP_DEPOT_VY_MMS, 0.0f) != CSVC_OK) {
        return APP_ERR;
    }
    do {
        osDelay(APP_DEPOT_POLL_MS);
        if (csvc_get_pose(&pos, &yaw) != CSVC_OK) {
            (void)align_stop();
            return APP_ERR;
        }
    } while (pos.y_mm < y_mm);
    return (align_stop() == ALIGN_OK) ? APP_OK : APP_ERR;
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
    uint8_t     i;                    /* 仓库横移目标索引 */

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
    /* 阶梯后去圆柱：经前置点直线切入绕圈起点，再绕圆柱一圈 */
    if ((route_go(ROUTE_CYL_PRE) == APP_OK) &&
        (route_go(ROUTE_CYL) == APP_OK)) {
        (void)cyl_round();
    }

    /* 以下为暂定时序，见 APP_ARM_WAIT_MS 处说明 */
    osDelay(APP_ARM_WAIT_MS);
    /* 仓库：1 号位找线标定，再横移轮流到 2~4 号位，每点停留占位；
     * 1 号位失败则位姿不可信，不盲横移直接回家 */
    if (route_go(ROUTE_DEPOT) == APP_OK) {
        osDelay(APP_DEPOT_WAIT_MS);
        for (i = 0U; i < APP_DEPOT_NUM; i++) {
            if (depot_shift(g_depot_y[i]) != APP_OK) {
                APP_LOGE("depot shift %u fail", (unsigned)i);
                break;
            }
            osDelay(APP_DEPOT_WAIT_MS);
        }
    }
    (void)route_go(ROUTE_HOME);
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
