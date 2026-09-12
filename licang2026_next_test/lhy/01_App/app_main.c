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

/* 暂定配置：协议缺 MISSION_CMD_GO_DEPOT/GO_HOME 及对应 READY 事件，
 * 仓库与回原点段暂以固定延时替代命令等待，协议补齐后换成 link_wait。 */
#define APP_ARM_WAIT_MS   10000U  /* 等机械臂夹球完成，ms（暂定） */
#define APP_DEPOT_WAIT_MS 3000U   /* 仓库段动作占位，ms（暂定） */
#define APP_HOME_WAIT_MS  3000U   /* 回原点后停留，ms（暂定） */
#define APP_IDLE_MS       1000U   /* 流程结束后的空转周期，ms */

/* 上电初始位姿（世界系，按场地标定） */
#define APP_START_X_MM    1200
#define APP_START_Y_MM    350
#define APP_START_YAW_DEG 0.0f

static osThreadId_t g_task = NULL;  /* 底盘任务 */
static uint8_t      g_app_up = 0U;  /* 应用启动标志，1=资源就绪 */

static const osThreadAttr_t g_task_attr = {
    .name       = "chassis",
    .stack_size = APP_TASK_STACK,
    .priority   = osPriorityNormal,
};

static void app_task(void *arg);

/**
 * @brief  应用主任务：按 Mission 命令串行执行底盘任务点
 * @param  arg 未用
 * @note   阶段失败以 is_ready=0 回执，由 Mission 决定停机，底盘不自行中止
 */
static void app_task(void *arg)
{
    uint16_t id = CHASSIS_MISSION_REQUEST_ID_INVALID; /* 阶段请求编号 */
    uint8_t  ok;                                      /* 阶段结果，1=成功 */

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
        (void)stairs_sweep(id);
    }

    /* 以下两段为暂定时序，见 APP_ARM_WAIT_MS 处说明 */
    osDelay(APP_ARM_WAIT_MS);
    (void)route_go(ROUTE_DEPOT);
    osDelay(APP_DEPOT_WAIT_MS);
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
