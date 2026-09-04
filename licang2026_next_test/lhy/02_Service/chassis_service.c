/**
 * @file    chassis_service.c
 * @brief   底盘服务：命令队列 + 里程计/控制/监控三线程 + 导航仲裁
 * @author  haoyu
 * @note    - 里程计线程(20ms)：清四位→广播读→WaitAll 四轮→持锁写 chassis_odom；
 *            广播读下发与等待全程锁外，连续 10 次超时报故障
 *          - 控制线程(20ms,主角)：整段持锁——读位姿→按命令算控制→下发(~70µs)，
 *            故障置位则跳过运动；上报走 UART 留在锁外
 *          - 监控线程：等故障信号→持锁停车 + latch，竞赛先用简单停车
 *          - 位姿锁置本层(osMutex)，只护跨线程位姿读写；命令队列深度 1 覆盖式
 */

#include "chassis_service.h"

#include <stdio.h>
#include <string.h>

#include "cmsis_os2.h"

#include "chassis_adaption.h"
#include "system_assembly.h"
#include "hwt101_adaption.h"
#include "map_adaption.h"

/* ===== 调试日志：0=不编译进固件，1=经 RTT 输出 ===== */
#ifndef CSVC_LOG_EN
#define CSVC_LOG_EN   1
#endif
#if CSVC_LOG_EN
#include "cat_log.h"
#define CSVC_LOGW(fmt, ...)  LOGW("[csvc] " fmt, ##__VA_ARGS__)
#define CSVC_LOGE(fmt, ...)  LOGE("[csvc] " fmt, ##__VA_ARGS__)
#else
#define CSVC_LOGW(fmt, ...)  do {} while (0)
#define CSVC_LOGE(fmt, ...)  do {} while (0)
#endif

#define CSVC_PERIOD_MS    20U        /* 控制/采集周期 ms */
#define CSVC_ODOM_DT_S    0.02f      /* 里程计积分周期 s（=20ms） */
#define CSVC_RAD2DEG      57.29578f  /* rad→deg */
#define CSVC_PATH_MAX     128U       /* 路径点缓存上限 */
#define CSVC_TASK_STACK   2048U      /* 里程计/控制任务栈字节 */
#define CSVC_GUARD_STACK  1024U      /* 监控任务栈字节 */
#define CSVC_READ_TMO_MS  10U        /* 广播读等齐超时 ms（含 TX 3ms 门控+回帧，须 <周期） */
#define CSVC_FAIL_MAX     100U        /* 连续超时报故障阈值 */
#define CSVC_POS_FLAGS_ALL 0x0FU     /* 四轮到位掩码（与装配根同源） */
#define CSVC_FAULT_FLAG   0x01U      /* 故障信号位（里程计→监控） */

/* 内部命令模式 */
typedef enum {
    CMD_NONE = 0,  /* 无命令 */
    CMD_FREE,      /* 自由速度 */
    CMD_ARC,       /* 画圆 */
    CMD_LINE,      /* 直线导航 */
    CMD_PATH       /* 路径跟随 */
} csvc_cmd_mode_t;

/* 内部命令（投递到队列） */
typedef struct {
    csvc_cmd_mode_t mode;   /* 命令类型 */
    union {
        struct { float vx, vy, wz; } fr;              /* FREE，wz deg/s */
        struct { float linear, r; bool insitu; } arc; /* ARC */
        struct { map_point_t tgt; float yaw, v, w; uint8_t *fin; } line; /* LINE */
        struct { float yaw, v; uint16_t len; uint8_t *fin; } path;      /* PATH */
    } u;
} csvc_cmd_t;

static osMessageQueueId_t g_queue = NULL;    /* 命令队列（深度 1） */
static motor_handler_t   *g_mh = NULL;       /* 电机 handler（广播读用） */
static osMutexId_t        g_pose_lock = NULL; /* 位姿锁：护跨线程位姿读写 */
static osEventFlagsId_t   g_pos_flags = NULL; /* 四轮到位标志（装配根同源） */
static osEventFlagsId_t   g_fault = NULL;    /* 故障信号：里程计→监控 */
static volatile uint8_t   g_faulted = 0U;    /* 故障 latch：置位后控制不再下发 */
static osThreadId_t       g_odom_task = NULL;  /* 里程计线程句柄 */
static osThreadId_t       g_ctrl_task = NULL;  /* 控制线程句柄 */
static osThreadId_t       g_guard_task = NULL; /* 监控线程句柄 */
static uint8_t            g_inited = 0U;     /* 是否已初始化 */
static float              g_last_yaw = 0.0f; /* 上次有效 yaw，deg */
static uint8_t           *g_nav_fin = NULL;  /* 当前导航完成标志（仲裁用） */
static map_point_t        g_path[CSVC_PATH_MAX]; /* 路径缓存（世界系 mm） */

/* 里程计 / 控制线程同优先级（采集与控制都吃实时性） */
static const osThreadAttr_t g_odom_attr = {
    .name       = "csvc_odom",
    .stack_size = CSVC_TASK_STACK,
    .priority   = osPriorityHigh,
};
static const osThreadAttr_t g_ctrl_attr = {
    .name       = "csvc_ctrl",
    .stack_size = CSVC_TASK_STACK,
    .priority   = osPriorityHigh,
};
static const osThreadAttr_t g_guard_attr = {
    .name       = "csvc_guard",
    .stack_size = CSVC_GUARD_STACK,
    .priority   = osPriorityNormal,
};

static void csvc_report(const chassis_odom_t *odom);
static void csvc_cancel_nav(void);
static csvc_status_t csvc_post(const csvc_cmd_t *cmd);
static csvc_status_t csvc_plan(map_point_t target, uint16_t *out_len);
static void csvc_odom_task(void *arg);
static void csvc_ctrl_task(void *arg);
static void csvc_guard_task(void *arg);

/**
 * @brief  上报里程计快照给上位机（printf 重定向到上报 UART）
 * @param  odom 里程计快照
 */
static void csvc_report(const chassis_odom_t *odom)
{
    /* 6 元组：x,y,yaw(deg),vx,vy,w(deg/s)，与上位机网页约定 */
    printf("%d,%d,%d,%d,%d,%d\r\n",
           (int)odom->x_mm, (int)odom->y_mm,
           (int)(odom->yaw_rad * CSVC_RAD2DEG),
           (int)odom->vx, (int)odom->vy,
           (int)(odom->w * CSVC_RAD2DEG));
}

/**
 * @brief  取消进行中的导航（手动速度打断时把旧导航标记结束）
 */
static void csvc_cancel_nav(void)
{
    if (g_nav_fin != NULL) {
        *g_nav_fin = 1U;   /* 标记旧导航结束（被打断） */
        g_nav_fin = NULL;
    }
}

/**
 * @brief  覆盖式投递命令（深度 1 队列，先清空再放最新）
 * @param  cmd 待投递命令
 * @retval CSVC_OK / CSVC_ERR_INIT / CSVC_ERR
 */
static csvc_status_t csvc_post(const csvc_cmd_t *cmd)
{
    csvc_cmd_t drain; /* 清空旧命令的临时承接 */

    if ((g_inited == 0U) || (g_queue == NULL)) {
        return CSVC_ERR_INIT;
    }
    while (osMessageQueueGet(g_queue, &drain, NULL, 0U) == osOK) {
        /* 丢弃未消费的旧命令，只保留最新 */
    }
    if (osMessageQueuePut(g_queue, cmd, 0U, 0U) != osOK) {
        return CSVC_ERR;
    }
    return CSVC_OK;
}

/**
 * @brief  以里程计当前位姿为起点规划到目标的 A* 路径，缓存到 g_path
 * @param  target  目标点（世界系 mm）
 * @param  out_len 输出路径点数
 * @retval CSVC_OK / CSVC_ERR / CSVC_ERR_PARAM / CSVC_ERR_INIT
 * @note   起点用里程计位姿，须与地图世界系同系（上电由 app 调 set_pose 对齐）
 */
static csvc_status_t csvc_plan(map_point_t target, uint16_t *out_len)
{
    chassis_odom_t   odom;          /* 当前位姿，作规划起点 */
    chassis_status_t cs = CHASSIS_ERR; /* 取位姿结果 */
    map_status_t     ms = MAP_ERR;  /* 规划结果 */
    uint16_t         len = 0U;      /* 路径点数 */

    if (out_len == NULL) {
        return CSVC_ERR_PARAM;
    }
    /* 持锁取位姿快照（与里程计线程写互斥），算路径在锁外 */
    (void)osMutexAcquire(g_pose_lock, osWaitForever);
    cs = chassis_get_odom(&odom);
    (void)osMutexRelease(g_pose_lock);
    if (cs != CHASSIS_OK) {
        return CSVC_ERR_INIT;
    }
    ms = map_adp_find_path((int16_t)odom.x_mm, (int16_t)odom.y_mm,
                           target.x_mm, target.y_mm,
                           g_path, CSVC_PATH_MAX, &len);
    if (ms != MAP_OK) {
        return CSVC_ERR;
    }
    *out_len = len;
    return CSVC_OK;
}

/**
 * @brief  里程计线程：广播读同刻采样四轮 → 持锁积分（20ms）
 * @param  arg 未用
 * @note   下发(mh_request_pos_all)与 WaitAll 等待全程锁外；锁只护写里程计那一下
 */
static void csvc_odom_task(void *arg)
{
    float    gyro = 0.0f;                   /* 角速度（附带量） */
    float    yaw  = 0.0f;                   /* 航向 deg */
    uint8_t  fail = 0U;                     /* 连续超时计数 */
    uint32_t flags = 0U;                    /* 等齐返回的标志位 */
    uint32_t wake = osKernelGetTickCount(); /* 周期唤醒基准 */

    (void)arg;
    for (;;) {
        /* 1) 读 yaw：无新帧沿用上次有效值（积分需要） */
        if (hwt101_adp_read(&gyro, &yaw) != HWT101_OK) {
            yaw = g_last_yaw;
        } else {
            g_last_yaw = yaw;
        }
        /* 2) 清四位 → 一帧广播读 → 限时等齐（下发与等待全程锁外） */
        (void)osEventFlagsClear(g_pos_flags, CSVC_POS_FLAGS_ALL);
        (void)mh_request_pos_all(g_mh);
        flags = osEventFlagsWait(g_pos_flags, CSVC_POS_FLAGS_ALL,
                                 osFlagsWaitAll, CSVC_READ_TMO_MS);
        if ((flags & osFlagsError) == 0U) {
            /* 3) 四轮齐：持锁写里程计（只护这一下，µs 级） */
            (void)osMutexAcquire(g_pose_lock, osWaitForever);
            (void)chassis_odom(yaw, CSVC_ODOM_DT_S);
            (void)osMutexRelease(g_pose_lock);
            fail = 0U;                         /* 成功即重新计数 */
        } else if (fail < CSVC_FAIL_MAX) {
            /* 4) 超时：下周期自然重广播；恰好连续 10 次报一次故障 */
            fail++;
            if (fail == CSVC_FAIL_MAX) {
                (void)osEventFlagsSet(g_fault, CSVC_FAULT_FLAG);
                CSVC_LOGW("odom timeout x%u -> fault", (unsigned)fail);
            }
        }
        wake += CSVC_PERIOD_MS;
        (void)osDelayUntil(wake);
    }
}

/**
 * @brief  控制线程（主角）：整段持锁读位姿→算控制→下发（20ms）
 * @param  arg 未用
 * @note   实测整段 ~70µs，下发(mh_speed)为非阻塞入队；上报走 UART 留在锁外
 */
static void csvc_ctrl_task(void *arg)
{
    csvc_cmd_t     cmd;                           /* 当前执行命令 */
    chassis_odom_t odom;                          /* 里程计快照 */
    uint32_t       wake = osKernelGetTickCount(); /* 周期唤醒基准 */

    (void)arg;
    (void)memset(&cmd, 0, sizeof(cmd));
    (void)memset(&odom, 0, sizeof(odom));
    for (;;) {
        /* 1) 取最新命令；无新命令沿用上一条 */
        (void)osMessageQueueGet(g_queue, &cmd, NULL, 0U);

        /* 2) 整段持锁：读位姿一致 + 算控制 + 下发；故障 latch 内置判定 */
        (void)osMutexAcquire(g_pose_lock, osWaitForever);
        (void)chassis_get_odom(&odom);
        if (g_faulted == 0U) {
            switch (cmd.mode) {
                case CMD_FREE:
                    (void)chassis_set_vel(cmd.u.fr.vx, cmd.u.fr.vy, cmd.u.fr.wz);
                    break;
                case CMD_ARC:
                    (void)chassis_set_radius(cmd.u.arc.linear, cmd.u.arc.r,
                                             cmd.u.arc.insitu);
                    break;
                case CMD_LINE:
                    (void)chassis_nav(cmd.u.line.tgt, cmd.u.line.yaw,
                                      cmd.u.line.v, cmd.u.line.w, cmd.u.line.fin);
                    if ((cmd.u.line.fin != NULL) && (*cmd.u.line.fin != 0U)) {
                        g_nav_fin = NULL;
                        cmd.mode = CMD_NONE;
                    }
                    break;
                case CMD_PATH:
                    (void)chassis_follow(g_path, cmd.u.path.len, cmd.u.path.v,
                                         cmd.u.path.yaw, cmd.u.path.fin);
                    if ((cmd.u.path.fin != NULL) && (*cmd.u.path.fin != 0U)) {
                        g_nav_fin = NULL;
                        cmd.mode = CMD_NONE;
                    }
                    break;
                default:
                    (void)chassis_stop();
                    break;
            }
        }
        /* 3) */
        csvc_report(&odom);
        (void)osMutexRelease(g_pose_lock);


        wake += CSVC_PERIOD_MS;
        (void)osDelayUntil(wake);
    }
}

/**
 * @brief  错误监控线程：等故障信号 → 持锁停车 + latch（简单停车）
 * @param  arg 未用
 * @note   latch 先于停车下发；控制线程随后锁内见 g_faulted 不再下发，停车不被覆盖
 */
static void csvc_guard_task(void *arg)
{
    (void)arg;
    for (;;) {
        /* 等里程计线程的故障信号（连续超时达阈值） */
        (void)osEventFlagsWait(g_fault, CSVC_FAULT_FLAG,
                               osFlagsWaitAny, osWaitForever);
        g_faulted = 1U;                        /* latch 先置，再停车 */
        (void)osMutexAcquire(g_pose_lock, osWaitForever);
        (void)chassis_stop();
        (void)osMutexRelease(g_pose_lock);
        csvc_cancel_nav();                     /* 释放在途导航，唤醒上层等待 */
        CSVC_LOGE("odom lost, chassis parked");
    }
}

csvc_status_t csvc_init(void)
{
    motor_handler_t *mh = NULL; /* 电机 handler 句柄 */

    if (g_inited != 0U) {
        return CSVC_OK;
    }
    mh = system_motor_handler();
    if (mh == NULL) {
        return CSVC_ERR_INIT;            /* 电机子系统未组装 */
    }
    g_mh = mh;                           /* 缓存 handler 供广播读 */
    if (chassis_init(mh) != CHASSIS_OK) {
        return CSVC_ERR_INIT;            /* 接入底盘失败 */
    }
    if (map_adp_init() != MAP_OK) {
        return CSVC_ERR_INIT;            /* 地图/障碍载入失败 */
    }
    g_pos_flags = system_pos_flags();    /* 复用装配根的四轮到位标志 */
    if (g_pos_flags == NULL) {
        return CSVC_ERR_INIT;            /* 装配根未建标志 */
    }
    g_pose_lock = osMutexNew(NULL);
    if (g_pose_lock == NULL) {
        return CSVC_ERR_RES;
    }
    g_fault = osEventFlagsNew(NULL);
    if (g_fault == NULL) {
        return CSVC_ERR_RES;
    }
    g_queue = osMessageQueueNew(1U, sizeof(csvc_cmd_t), NULL);
    if (g_queue == NULL) {
        return CSVC_ERR_RES;
    }
    /* 起三线程：里程计采集 / 控制下发 / 错误监控 */
    g_odom_task = osThreadNew(csvc_odom_task, NULL, &g_odom_attr);
    if (g_odom_task == NULL) {
        return CSVC_ERR_RES;
    }
    g_ctrl_task = osThreadNew(csvc_ctrl_task, NULL, &g_ctrl_attr);
    if (g_ctrl_task == NULL) {
        return CSVC_ERR_RES;
    }
    g_guard_task = osThreadNew(csvc_guard_task, NULL, &g_guard_attr);
    if (g_guard_task == NULL) {
        return CSVC_ERR_RES;
    }
    g_inited = 1U;
    return CSVC_OK;
}

csvc_status_t csvc_set_pose(map_point_t pos, float yaw_deg)
{
    chassis_status_t ret = CHASSIS_ERR; /* 重定位结果 */

    if (g_inited == 0U) {
        return CSVC_ERR_INIT;
    }
    /* 持锁写位姿（与里程计线程写、控制线程读互斥） */
    (void)osMutexAcquire(g_pose_lock, osWaitForever);
    ret = chassis_set_pose(pos, yaw_deg);
    (void)osMutexRelease(g_pose_lock);
    return (ret == CHASSIS_OK) ? CSVC_OK : CSVC_ERR;
}

csvc_status_t csvc_free(float vx, float vy, float wz)
{
    csvc_cmd_t cmd; /* 待投递命令 */

    (void)memset(&cmd, 0, sizeof(cmd));
    cmd.mode = CMD_FREE;
    cmd.u.fr.vx = vx;
    cmd.u.fr.vy = vy;
    cmd.u.fr.wz = wz;
    csvc_cancel_nav();              /* 手动速度打断进行中导航 */
    return csvc_post(&cmd);
}

csvc_status_t csvc_arc(float linear, float r, bool insitu)
{
    csvc_cmd_t cmd; /* 待投递命令 */

    (void)memset(&cmd, 0, sizeof(cmd));
    cmd.mode = CMD_ARC;
    cmd.u.arc.linear = linear;
    cmd.u.arc.r = r;
    cmd.u.arc.insitu = insitu;
    csvc_cancel_nav();              /* 手动画圆打断进行中导航 */
    return csvc_post(&cmd);
}

csvc_status_t csvc_nav(uint8_t mode, map_point_t target, float yaw_deg,
                       float v, float w, uint8_t *finished)
{
    csvc_cmd_t    cmd;                 /* 待投递命令 */
    csvc_status_t ret = CSVC_ERR;       /* 步骤结果 */
    uint16_t      len = 0U;            /* 路径点数 */

    if (finished == NULL) {
        return CSVC_ERR_PARAM;
    }
    if (g_inited == 0U) {
        return CSVC_ERR_INIT;
    }
    /* 仲裁：上一条导航未完成则拒绝新导航 */
    if ((g_nav_fin != NULL) && (*g_nav_fin == 0U)) {
        return CSVC_BUSY;
    }
    (void)memset(&cmd, 0, sizeof(cmd));

    if (mode == CSVC_NAV_LINE) {
        cmd.mode = CMD_LINE;
        cmd.u.line.tgt = target;
        cmd.u.line.yaw = yaw_deg;
        cmd.u.line.v = v;
        cmd.u.line.w = w;
        cmd.u.line.fin = finished;
    } else if (mode == CSVC_NAV_PATH) {
        ret = csvc_plan(target, &len);
        if (ret != CSVC_OK) {
            return ret;
        }
        cmd.mode = CMD_PATH;
        cmd.u.path.yaw = yaw_deg;
        cmd.u.path.v = v;
        cmd.u.path.len = len;
        cmd.u.path.fin = finished;
    } else {
        return CSVC_ERR_PARAM;
    }

    *finished = 0U;
    g_nav_fin = finished;
    ret = csvc_post(&cmd);
    if (ret != CSVC_OK) {
        *finished = 1U;
        g_nav_fin = NULL;
    }
    return ret;
}
