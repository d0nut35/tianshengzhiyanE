/**
 * @file    app_main.c
 * @brief   应用入口：上电时序 + BLE 调度
 * @author  haoyu
 * @note    - 时序：system_assembly_init → csvc_init → csvc_set_pose → BLE
 *          - BLE：ISR(on_rx_raw)入队 → app_task 取队 ble_adp_process → on_nav
 *          - on_nav 给 rad，本层转 deg 再投 csvc（csvc 对上统一 deg）
 */

#include "app_main.h"

#include <string.h>

#include "cmsis_os2.h"

#include "system_assembly.h"
#include "chassis_service.h"
#include "ble_adaption.h"
#include "hwt101_adaption.h"
#include "lsensor/lsensor_handler.h"

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

#define APP_RAD2DEG       57.29578f /* rad→deg */
#define APP_BLE_CHUNK     64U       /* 单次 BLE 字节块上限 */
#define APP_BLE_QDEPTH    8U        /* BLE 字节块队列深度 */
#define APP_TASK_STACK    2048U     /* BLE 调度任务栈字节 */

/* 第二点白线对齐参数 */
#define APP_CW_W_DEG      (-5.0f)  /* 顺时针扫描角速度，deg/s */
#define APP_CCW_W_DEG     5.0f     /* 逆时针扫描角速度，deg/s */
#define APP_TURN_W_DEG    8.0f      /* 回到中值的角速度，deg/s */
#define APP_YAW_TOL_DEG   0.8f      /* 回中角度容差，deg */
#define APP_MID_YAW_OFS   -3.0f      /* 中值补偿，沿 IMU yaw 正向，deg */
#define APP_SCAN_MS       5U        /* 灰度与 IMU 轮询周期，ms */
#define APP_STOP_MS       80U       /* 停车后机械稳定时间，ms */
#define APP_IMU_TMO_MS    200U      /* 等待有效 IMU 角度超时，ms */
#define APP_IMU_UNLOCK_MS 210U      /* 解锁后到置零帧的间隔，ms */
#define APP_IMU_ZERO_MS   510U      /* 置零后到保存帧的间隔，ms */
#define APP_SWEEP_TMO_MS  6000U     /* 单向找线边缘超时，ms */
#define APP_TURN_TMO_MS   6000U     /* 回到中值角度超时，ms */

/* 上电初始位姿（世界系，按场地标定） */
#define APP_START_X_MM    1200
#define APP_START_Y_MM    350
#define APP_START_YAW_DEG 0.0f

/* BLE 原始字节块（ISR 拷入 → 任务取出喂解析器） */
typedef struct {
    uint8_t  data[APP_BLE_CHUNK]; /* 字节缓冲 */
    uint16_t len;                 /* 有效长度 */
} app_chunk_t;

static osMessageQueueId_t g_ble_q = NULL; /* BLE 字节块队列 */
static osThreadId_t       g_task = NULL;  /* BLE 调度任务 */
static uint8_t            g_nav_fin = 1U; /* 导航完成标志，1=空闲 */
static uint8_t            g_app_up = 0;  /* 应用启动标志 */

static const osThreadAttr_t g_task_attr = {
    .name       = "app_ble",
    .stack_size = APP_TASK_STACK,
    .priority   = osPriorityNormal,
};

static void app_on_rx_raw(const uint8_t *data, uint16_t len);
static void app_on_nav(ble_nav_mode_t mode, float x_mm, float y_mm,
                       float yaw_rad, float v_mm_s, float w_rad_s);
static void app_task(void *arg);
static uint32_t app_ms_ticks(uint32_t ms);
static float app_ang_norm(float angle);
static app_status_t app_get_yaw(float *yaw_deg);
static app_status_t app_stop_motion(void);
static app_status_t app_seek_edge(lsensor_id_t id, float wz,
                                  float *yaw_deg);
static app_status_t app_turn_mid(float target);
static app_status_t app_imu_calib(float yaw_deg);
static app_status_t app_imu_cfg(void);
static app_status_t app_align_line(map_point_t pos, float yaw_deg);
static app_status_t app_nav_wait(map_point_t pt, float yaw_deg,
                                 float v, float w);
static app_status_t app_seek_line(void);
static app_status_t app_go_platform(void);
static app_status_t app_go_stairs(void);
static app_status_t app_go_depot(void);
static app_status_t app_go_home(void);

/**
 * @brief  BLE 原始字节回调（ISR 上下文）：拷贝入队，非阻塞
 * @param  data 字节缓冲
 * @param  len  字节长度
 */
static void app_on_rx_raw(const uint8_t *data, uint16_t len)
{
    app_chunk_t chunk; /* 待入队字节块 */

    if ((data == NULL) || (len == 0U) || (g_ble_q == NULL)) {
        return;
    }
    chunk.len = (len > APP_BLE_CHUNK) ? APP_BLE_CHUNK : len;
    (void)memcpy(chunk.data, data, chunk.len);
    (void)osMessageQueuePut(g_ble_q, &chunk, 0U, 0U); /* ISR 非阻塞 */
}

/**
 * @brief  BLE 导航命令回调（任务上下文）：rad→deg 后投 csvc
 * @param  mode    导航模式
 * @param  x_mm    目标 x，mm
 * @param  y_mm    目标 y，mm
 * @param  yaw_rad 目标航向，rad
 * @param  v_mm_s  线速度，mm/s
 * @param  w_rad_s 角速度，rad/s
 */
static void app_on_nav(ble_nav_mode_t mode, float x_mm, float y_mm,
                       float yaw_rad, float v_mm_s, float w_rad_s)
{
    uint8_t     svc_mode; /* csvc 导航模式 */
    map_point_t tgt;      /* 目标点 */

    svc_mode = (mode == BLE_NAV_PATH) ? CSVC_NAV_PATH : CSVC_NAV_LINE;
    tgt.x_mm = (int16_t)x_mm;
    tgt.y_mm = (int16_t)y_mm;
    (void)csvc_nav(svc_mode, tgt, yaw_rad * APP_RAD2DEG, v_mm_s,
                   w_rad_s * APP_RAD2DEG, &g_nav_fin);
}

/**
 * @brief  将毫秒转换成当前 CMSIS-OS tick 数
 * @param  ms 毫秒数
 * @retval tick 数，最小为 1
 */
static uint32_t app_ms_ticks(uint32_t ms)
{
    uint32_t tick_hz = osKernelGetTickFreq(); /* OS 每秒 tick 数 */
    uint32_t ticks;                           /* 换算后的 tick 数 */

    if (tick_hz == 0U) {
        return 1U;
    }
    ticks = ((tick_hz * ms) + 999U) / 1000U;
    return (ticks == 0U) ? 1U : ticks;
}

/**
 * @brief  将角度归一化到 [-180, 180)
 * @param  angle 待归一化角度，deg
 * @retval 归一化角度，deg
 */
static float app_ang_norm(float angle)
{
    while (angle >= 180.0f) {
        angle -= 360.0f;
    }
    while (angle < -180.0f) {
        angle += 360.0f;
    }
    return angle;
}

/**
 * @brief  在限定时间内读取一帧有效 IMU 航向
 * @param  yaw_deg 输出原始航向，deg
 * @retval APP_OK / APP_ERR
 */
static app_status_t app_get_yaw(float *yaw_deg)
{
    float gyro = 0.0f;                      /* 附带的 Z 轴角速度 */
    uint32_t start = osKernelGetTickCount(); /* 超时起点 */
    uint32_t tmo = app_ms_ticks(APP_IMU_TMO_MS); /* 超时 tick 数 */

    if (yaw_deg == NULL) {
        return APP_ERR;
    }
    while ((osKernelGetTickCount() - start) < tmo) {
        if (hwt101_adp_read(&gyro, yaw_deg) == HWT101_OK) {
            return APP_OK;
        }
        osDelay(APP_SCAN_MS);
    }
    return APP_ERR;
}

/**
 * @brief  投递停车命令并等待底盘稳定
 * @retval APP_OK / APP_ERR
 */
static app_status_t app_stop_motion(void)
{
    if (csvc_free(0.0f, 0.0f, 0.0f) != CSVC_OK) {
        return APP_ERR;
    }
    osDelay(APP_STOP_MS);
    return APP_OK;
}

/**
 * @brief  原地旋转至指定灰度传感器由压线变为高电平
 * @param  id      灰度传感器编号
 * @param  wz      旋转角速度，deg/s
 * @param  yaw_deg 高电平触发瞬间的 IMU 原始航向
 * @retval APP_OK / APP_ERR
 */
static app_status_t app_seek_edge(lsensor_id_t id, float wz, float *yaw_deg)
{
    lsensor_level_t level;                   /* 当前传感器电平 */
    float gyro = 0.0f;                       /* 附带的 Z 轴角速度 */
    float edge_yaw = 0.0f;                   /* 高电平触发瞬间的航向 */
    uint32_t start = osKernelGetTickCount(); /* 扫描超时起点 */
    uint32_t tmo = app_ms_ticks(APP_SWEEP_TMO_MS); /* 超时 tick 数 */
    uint8_t yaw_valid = 0U;                  /* 触发角度有效标志 */

    if (yaw_deg == NULL) {
        return APP_ERR;
    }
    level = lsh_get_level(id);
    if (level != LSENSOR_LEVEL_LOW) {
        return APP_ERR;
    }
    if (csvc_free(0.0f, 0.0f, wz) != CSVC_OK) {
        return APP_ERR;
    }
    while ((osKernelGetTickCount() - start) < tmo) {
        level = lsh_get_level(id);
        if (level == LSENSOR_LEVEL_HIGH) {
            if (hwt101_adp_read(&gyro, &edge_yaw) == HWT101_OK) {
                *yaw_deg = edge_yaw;
                yaw_valid = 1U;
            }
            if (app_stop_motion() != APP_OK) {
                return APP_ERR;
            }
            if (yaw_valid != 0U) {
                return APP_OK;
            }
            return app_get_yaw(yaw_deg);
        }
        if (level == LSENSOR_LEVEL_INVALID) {
            break;
        }
        osDelay(APP_SCAN_MS);
    }
    (void)app_stop_motion();
    return APP_ERR;
}

/**
 * @brief  以定速原地旋转到两个白线边缘角度的中值
 * @param  target IMU 原始目标航向，deg
 * @retval APP_OK / APP_ERR
 */
static app_status_t app_turn_mid(float target)
{
    float yaw = 0.0f;                         /* 当前 IMU 原始航向 */
    float err;                                /* 最短航向误差 */
    float prev_err;                           /* 上次航向误差 */
    float wz;                                 /* 底盘旋转指令 */
    uint32_t start = osKernelGetTickCount();  /* 回中超时起点 */
    uint32_t tmo = app_ms_ticks(APP_TURN_TMO_MS); /* 超时 tick 数 */

    if (app_get_yaw(&yaw) != APP_OK) {
        return APP_ERR;
    }
    err = app_ang_norm(target - yaw);
    if ((err >= -APP_YAW_TOL_DEG) && (err <= APP_YAW_TOL_DEG)) {
        return app_stop_motion();
    }

    /* 底盘命令与 IMU 原始 yaw 同向，按最短误差选择旋转方向。 */
    wz = (err > 0.0f) ? APP_TURN_W_DEG : -APP_TURN_W_DEG;
    prev_err = err;
    if (csvc_free(0.0f, 0.0f, wz) != CSVC_OK) {
        return APP_ERR;
    }
    while ((osKernelGetTickCount() - start) < tmo) {
        if (app_get_yaw(&yaw) != APP_OK) {
            break;
        }
        err = app_ang_norm(target - yaw);
        if (((err >= -APP_YAW_TOL_DEG) && (err <= APP_YAW_TOL_DEG)) ||
            ((err * prev_err) <= 0.0f)) {
            return app_stop_motion();
        }
        prev_err = err;
        osDelay(APP_SCAN_MS);
    }
    (void)app_stop_motion();
    return APP_ERR;
}

/**
 * @brief  软件标定：把当前 IMU 航向标定为指定角度（带重试）
 * @param  yaw_deg IMU 帧目标航向，deg
 * @retval APP_OK / APP_ERR
 */
static app_status_t app_imu_calib(float yaw_deg)
{
    uint32_t start = osKernelGetTickCount(); /* 超时起点 */
    uint32_t tmo = app_ms_ticks(APP_IMU_TMO_MS); /* 超时 tick 数 */

    while ((osKernelGetTickCount() - start) < tmo) {
        if (hwt101_adp_set_yaw(yaw_deg) == HWT101_OK) {
            return APP_OK;
        }
        osDelay(APP_SCAN_MS);
    }
    return APP_ERR;
}

/**
 * @brief  陀螺仪上电配置：解锁 → Z 轴角度置零 → 保存（原 freertos.c Main_task 时序）
 * @retval APP_OK / APP_ERR
 * @note   需在陀螺仪 DMA 接收启动前调用；失败仅影响上电零位，不阻止后续装配
 */
static app_status_t app_imu_cfg(void)
{
    hwt101_status_t ret; /* 寄存器写结果 */

    ret = hwt101_adp_write_reg(HWT101_REG_UNLOCK, HWT101_UNLOCK_DL,
                               HWT101_UNLOCK_DH);
    osDelay(APP_IMU_UNLOCK_MS);
    if (ret == HWT101_OK) {
        ret = hwt101_adp_write_reg(HWT101_REG_CALIYAW, 0x00U, 0x00U);
    }
    osDelay(APP_IMU_ZERO_MS);
    if (ret == HWT101_OK) {
        ret = hwt101_adp_write_reg(HWT101_REG_SAVE, 0x00U, 0x00U);
    }
    return (ret == HWT101_OK) ? APP_OK : APP_ERR;
}

/**
 * @brief  扫取白线两侧边缘、回中并按指定位姿软件标定航向
 * @param  pos     对齐完成后的世界系标定坐标
 * @param  yaw_deg 对齐完成后的世界系航向，deg
 * @retval APP_OK / APP_ERR
 */
static app_status_t app_align_line(map_point_t pos, float yaw_deg)
{
    float cw_yaw = 0.0f;   /* 5 号离线时的顺时针边缘角度 */
    float ccw_yaw = 0.0f;  /* 2 号离线时的逆时针边缘角度 */
    float mid_yaw;         /* 两个边缘的环形角度中值 */

    if (app_seek_edge(LSENSOR_ID_5, APP_CW_W_DEG, &cw_yaw) != APP_OK) {
        APP_LOGE("sensor 5 edge fail");
        return APP_ERR;
    }
    if (app_seek_edge(LSENSOR_ID_2, APP_CCW_W_DEG, &ccw_yaw) != APP_OK) {
        APP_LOGE("sensor 2 edge fail");
        return APP_ERR;
    }
    mid_yaw = app_ang_norm(cw_yaw +
                           (0.5f * app_ang_norm(ccw_yaw - cw_yaw)) +
                           APP_MID_YAW_OFS);
    APP_LOGI("line yaw cw=%d ccw=%d mid=%d",
             (int)cw_yaw, (int)ccw_yaw, (int)mid_yaw);
    if (app_turn_mid(mid_yaw) != APP_OK) {
        APP_LOGE("turn mid fail");
        return APP_ERR;
    }
    /* IMU 帧与世界帧反号（chassis 侧取负对齐地图系） */
    if (app_imu_calib(-yaw_deg) != APP_OK) {
        APP_LOGE("imu calib fail");
        return APP_ERR;
    }
    if (csvc_set_pose(pos, yaw_deg) != CSVC_OK) {
        APP_LOGE("set pose fail");
        return APP_ERR;
    }
    return APP_OK;
}

/**
 * @brief  发起路径导航并阻塞等待到点
 * @param  pt      目标点，世界系 mm
 * @param  yaw_deg 到点航向，deg
 * @param  v       线速度，mm/s
 * @param  w       角速度上限，deg/s
 * @retval APP_OK / APP_ERR
 */
static app_status_t app_nav_wait(map_point_t pt, float yaw_deg,
                                 float v, float w)
{
    g_nav_fin = 0U;
    if (csvc_nav(CSVC_NAV_PATH, pt, yaw_deg, v, w,
                 &g_nav_fin) != CSVC_OK) {
        g_nav_fin = 1U; /* 失败恢复空闲态 */
        return APP_ERR;
    }
    while (g_nav_fin == 0U) {
        osDelay(10U);
    }
    return APP_OK;
}

/**
 * @brief  右移找线：2、5 号同时压线后停车
 * @retval APP_OK=找到线且已停稳 / APP_ERR
 * @note   无论是否找到线均投递停车命令
 */
static app_status_t app_seek_line(void)
{
    lsensor_level_t sen_2;  /* 2 号循迹传感器电平 */
    lsensor_level_t sen_5;  /* 5 号循迹传感器电平 */
    uint8_t found = 0U;     /* 双传感器同时压线标志 */

    if (csvc_free(20.0f, 0.0f, 0.0f) != CSVC_OK) {
        APP_LOGE("right move fail");
    } else {
        for (;;) {
            sen_2 = lsh_get_level(LSENSOR_ID_2);
            sen_5 = lsh_get_level(LSENSOR_ID_5);
            if ((sen_2 == LSENSOR_LEVEL_LOW) &&
                (sen_5 == LSENSOR_LEVEL_LOW)) {
                found = 1U;
                break;
            }
            if ((sen_2 == LSENSOR_LEVEL_INVALID) ||
                (sen_5 == LSENSOR_LEVEL_INVALID)) {
                APP_LOGE("lsensor invalid");
                break;
            }
            osDelay(APP_SCAN_MS);
        }
    }
    if (csvc_free(0.0f, 0.0f, 0.0f) != CSVC_OK) {
        APP_LOGE("stop fail");
        return APP_ERR;
    }
    osDelay(20U); /* 等待控制任务执行停车命令 */
    return (found != 0U) ? APP_OK : APP_ERR;
}

/**
 * @brief  去圆台并对齐：导航、找线、按 IMU 航向标定位姿
 * @retval APP_OK / APP_ERR
 */
static app_status_t app_go_platform(void)
{
    float imu_wz = 0.0f;  /* IMU Z 轴角速度，读取附带量 */
    float imu_yaw = 0.0f; /* IMU 原始偏航角，deg */

    if (app_nav_wait((map_point_t){.x_mm = 500, .y_mm = 4300},
                     180.0f, 500.0f, 30.0f) != APP_OK) {
        APP_LOGE("point 1 nav fail");
        return APP_ERR;
    }
    if (app_seek_line() != APP_OK) {
        return APP_ERR;
    }
    if (hwt101_adp_read(&imu_wz, &imu_yaw) != HWT101_OK) {
        APP_LOGE("imu read fail");
        return APP_ERR;
    }
    /* IMU 帧与世界帧反号（chassis 侧取负对齐地图系） */
    if (csvc_set_pose((map_point_t){.x_mm = 450, .y_mm = 4300},
                      -imu_yaw) != CSVC_OK) {
        APP_LOGE("set pose fail");
        return APP_ERR;
    }
    return APP_OK;
}

/**
 * @brief  去阶梯并对齐：导航、找线、白线对齐标定
 * @retval APP_OK / APP_ERR
 */
static app_status_t app_go_stairs(void)
{
    if (app_nav_wait((map_point_t){.x_mm = 2030, .y_mm = 2850},
                     0.0f, 500.0f, 60.0f) != APP_OK) {
        APP_LOGE("point 2 nav fail");
        return APP_ERR;
    }
    if (app_seek_line() != APP_OK) {
        return APP_ERR;
    }
    if (app_align_line((map_point_t){.x_mm = 2130, .y_mm = 2850},
                       0.0f) != APP_OK) {
        APP_LOGE("point 2 align fail");
        return APP_ERR;
    }
    return APP_OK;
}

/**
 * @brief  去立体仓库并对齐：导航、找线、白线对齐标定
 * @retval APP_OK / APP_ERR
 */
static app_status_t app_go_depot(void)
{
    if (app_nav_wait((map_point_t){.x_mm = 480, .y_mm = 2500},
                     180.0f, 500.0f, 60.0f) != APP_OK) {
        APP_LOGE("point 3 nav fail");
        return APP_ERR;
    }
    if (app_seek_line() != APP_OK) {
        return APP_ERR;
    }
    if (app_align_line((map_point_t){.x_mm = 380, .y_mm = 2500},
                       180.0f) != APP_OK) {
        APP_LOGE("point 3 align fail");
        return APP_ERR;
    }
    return APP_OK;
}

/**
 * @brief  回原点：直线导航到点（不走 A*），不做找线对齐
 * @retval APP_OK / APP_ERR
 */
static app_status_t app_go_home(void)
{
    g_nav_fin = 0U;
    if (csvc_nav(CSVC_NAV_LINE, (map_point_t){.x_mm = 1200, .y_mm = 350},
                 180.0f, 500.0f, 30.0f, &g_nav_fin) != CSVC_OK) {
        g_nav_fin = 1U; /* 失败恢复空闲态 */
        APP_LOGE("home nav fail");
        return APP_ERR;
    }
    while (g_nav_fin == 0U) {
        osDelay(10U);
    }
    return APP_OK;
}

/**
 * @brief  应用主任务：串行执行任务点后转 BLE 调度循环
 * @param  arg 未用
 */
static void app_task(void *arg)
{
    app_chunk_t chunk; /* 出队字节块 */

    (void)arg;
    /* 等待应用启动 */
    while (g_app_up == 0U) {
        osDelay(10U);
    }
    osDelay(1000U); /* 等底盘服务稳定 */

    // /* 画圆：半径 350mm，线速度 250mm/s（可调） */
    // (void)csvc_arc(250.0f, -350.0f, false);

    /* 任务点失败只记日志，继续执行后续任务点 */
    (void)app_go_platform();
    osDelay(3000U);
    (void)app_go_stairs();
    osDelay(3000U);

    /* 直线导航（不走 A*）到 (1700,2500) → (1550,2500)，航向 180°，到点后画圆 */
    g_nav_fin = 0U;
    if (csvc_nav(CSVC_NAV_LINE, (map_point_t){.x_mm = 1700, .y_mm = 2520},
                 180.0f, 500.0f, 60.0f, &g_nav_fin) != CSVC_OK) {
        g_nav_fin = 1U;
        APP_LOGE("line nav 1 fail");
    } else {
        while (g_nav_fin == 0U) {
            osDelay(10U);
        }
        g_nav_fin = 0U;
        if (csvc_nav(CSVC_NAV_LINE, (map_point_t){.x_mm = 1550, .y_mm = 2520},
                     180.0f, 500.0f, 60.0f, &g_nav_fin) != CSVC_OK) {
            g_nav_fin = 1U;
            APP_LOGE("line nav 2 fail");
        } else {
            while (g_nav_fin == 0U) {
                osDelay(10U);
            }
            osDelay(1000U); /* 等底盘稳定 */
            (void)csvc_arc(200.0f, -350.0f, false);
        }
    }
    osDelay(10000U);
    (void)app_go_depot();
    osDelay(3000U);
    (void)app_go_home();
    osDelay(3000U);

    for (;;) {
        if (osMessageQueueGet(g_ble_q, &chunk, NULL, osWaitForever) == osOK) {
            (void)ble_adp_process(chunk.data, chunk.len);
        }
    }
}

app_status_t app_init(void)
{
    map_point_t start; /* 上电初始位姿坐标 */

    /* 0) 陀螺仪上电配置（失败只记日志，后续仍可软件标定航向） */
    if (app_imu_cfg() != APP_OK) {
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
    /* 4) BLE 字节队列 + 适配层（注入 ISR 入队 / 命令上抛回调） */
    g_ble_q = osMessageQueueNew(APP_BLE_QDEPTH, sizeof(app_chunk_t), NULL);
    if (g_ble_q == NULL) {
        APP_LOGE("ble queue fail");
        return APP_ERR;
    }
    APP_LOGI("ble queue init success");
    if (ble_adp_init(app_on_rx_raw, app_on_nav) != BLE_OK) {
        APP_LOGE("ble adp init fail");
        return APP_ERR;
    }
    APP_LOGI("ble adp init success");
    /* 5) 起 BLE 调度任务 */
    g_task = osThreadNew(app_task, NULL, &g_task_attr);
    if (g_task == NULL) {
        APP_LOGE("app task fail");
        return APP_ERR;
    }
    g_app_up = 1;
    APP_LOGI("app up");
    return APP_OK;
}
