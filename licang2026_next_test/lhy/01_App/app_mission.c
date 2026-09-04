/**
 * @file    app_mission.c
 * @brief   比赛任务编排示例：航点队列 + 到点动作（点到点抓取/放置）
 * @author  haoyu
 * @note    - app_mission_push() 把"航点+动作"塞进队列；编排任务串行取出执行
 *          - 每个航点：csvc_nav 开到点 → 轮询 finished 等到达 → 执行 grab/release
 *          - app_arm_* 是桩，请接你的夹爪实现（如 mh_pos 控 idx>=4 的夹爪电机）
 */

#include "app_mission.h"

#include "cmsis_os2.h"

#include "chassis_service.h"

/* ===== 调试日志：0=不编译进固件，1=经 RTT 输出 ===== */
#ifndef APP_M_LOG_EN
#define APP_M_LOG_EN   1
#endif
#if APP_M_LOG_EN
#include "cat_log.h"
#define M_LOGI(fmt, ...)  LOGI("[mission] " fmt, ##__VA_ARGS__)
#else
#define M_LOGI(fmt, ...)  do {} while (0)
#endif

#define M_QDEPTH    16U      /* 航点队列深度 */
#define M_STACK     2048U    /* 编排任务栈字节 */
#define M_NAV_V     800.0f   /* 导航线速度，mm/s */
#define M_NAV_W     90.0f    /* 导航角速度上限，deg/s */
#define M_POLL_MS   20U      /* 等到达轮询周期，ms */
#define M_ARM_MS    500U     /* 机械臂动作占位耗时，ms */

/* 航点 + 到点动作 */
typedef struct {
    map_point_t   pt;       /* 目标点，世界系 mm */
    float         yaw_deg;  /* 到点航向，deg */
    mission_act_t act;      /* 到点后动作 */
} waypoint_t;

static osMessageQueueId_t g_wp_q = NULL; /* 航点队列 */
static osThreadId_t       g_task = NULL; /* 编排任务 */

static const osThreadAttr_t g_attr = {
    .name       = "app_mission",
    .stack_size = M_STACK,
    .priority   = osPriorityNormal,
};

static void app_arm_grab(void);
static void app_arm_release(void);
static void mission_goto(const waypoint_t *wp);
static void mission_task(void *arg);

/**
 * @brief  机械臂抓取（桩）：接你的夹爪实现
 * @note   例：mh_pos(system_motor_handler(), 4, 模式, 闭合脉冲, rpm, accel) + 等到位
 */
static void app_arm_grab(void)
{
    M_LOGI("grab");
    (void)osDelay(M_ARM_MS); /* 占位：换成夹爪闭合 + 到位等待 */
}

/**
 * @brief  机械臂放置（桩）：接你的夹爪实现（张开夹爪）
 */
static void app_arm_release(void)
{
    M_LOGI("release");
    (void)osDelay(M_ARM_MS); /* 占位：换成夹爪张开 + 到位等待 */
}

/**
 * @brief  导航到一个航点：发 csvc_nav 后串行等到达
 * @param  wp 航点
 * @note   fin 是栈变量，csvc 任务会写 *fin；本函数等到 fin=1 才返回，期间栈有效
 */
static void mission_goto(const waypoint_t *wp)
{
    uint8_t fin = 0U; /* 到达标志，由 csvc 置 1 */

    if (csvc_nav(CSVC_NAV_LINE, wp->pt, wp->yaw_deg, M_NAV_V, M_NAV_W, &fin)
        != CSVC_OK) {
        M_LOGI("nav rejected");
        return;
    }
    while (fin == 0U) {
        (void)osDelay(M_POLL_MS);
    }
}

/**
 * @brief  编排任务：依次取航点，开到点后执行到点动作
 * @param  arg 未用
 */
static void mission_task(void *arg)
{
    waypoint_t wp; /* 当前航点 */

    (void)arg;
    for (;;) {
        if (osMessageQueueGet(g_wp_q, &wp, NULL, osWaitForever) != osOK) {
            continue;
        }
        mission_goto(&wp);              /* 1) 开到点 */
        switch (wp.act) {               /* 2) 到点动作 */
            case MISSION_ACT_GRAB:
                app_arm_grab();
                break;
            case MISSION_ACT_RELEASE:
                app_arm_release();
                break;
            default:
                break;
        }
        M_LOGI("waypoint done (%d,%d)", (int)wp.pt.x_mm, (int)wp.pt.y_mm);
    }
}

app_status_t app_mission_init(void)
{
    g_wp_q = osMessageQueueNew(M_QDEPTH, sizeof(waypoint_t), NULL);
    if (g_wp_q == NULL) {
        return APP_ERR;
    }
    g_task = osThreadNew(mission_task, NULL, &g_attr);
    if (g_task == NULL) {
        return APP_ERR;
    }
    return APP_OK;
}

app_status_t app_mission_push(map_point_t pt, float yaw_deg, mission_act_t act)
{
    waypoint_t wp; /* 待入队航点 */

    if (g_wp_q == NULL) {
        return APP_ERR;
    }
    wp.pt = pt;
    wp.yaw_deg = yaw_deg;
    wp.act = act;
    if (osMessageQueuePut(g_wp_q, &wp, 0U, 0U) != osOK) {
        return APP_ERR; /* 队列满 */
    }
    return APP_OK;
}
