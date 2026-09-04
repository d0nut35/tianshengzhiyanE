/**
 * @file    app_main.h
 * @brief   应用入口：上电时序编排 + BLE 调度（App 层）
 * @author  haoyu
 * @note    - 由 freertos.c 的 MX_FREERTOS_Init 调一次
 *          - 编排：system_assembly_init → csvc_init → 对齐位姿 → BLE 调度
 */

#ifndef APP_MAIN_H
#define APP_MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* 状态码：OK==0 */
typedef enum {
    APP_OK  = 0,  /* 成功 */
    APP_ERR = 1,  /* 上电失败（已 LOG） */
} app_status_t;

/**
 * @brief  应用上电初始化：装配子系统 + 起底盘服务 + 起 BLE 调度任务
 * @retval APP_OK / APP_ERR
 */
app_status_t app_init(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_MAIN_H */
