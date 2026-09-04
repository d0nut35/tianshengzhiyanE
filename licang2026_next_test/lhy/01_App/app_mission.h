/**
 * @file    app_mission.h
 * @brief   比赛任务编排示例：航点队列 + 到点动作（点到点抓取/放置）
 * @author  haoyu
 * @note    - 用法：app_mission_init() 起编排任务；app_mission_push() 塞航点
 *          - 编排任务串行：csvc_nav 导航到点 → 等到达 → grab/release → 下一个
 *          - 机械臂动作为桩 app_arm_*，需接你的夹爪/机械臂实现
 */

#ifndef APP_MISSION_H
#define APP_MISSION_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#include "app_main.h"   /* app_status_t */
#include "map.h"        /* map_point_t */

/* 到点后的动作 */
typedef enum {
    MISSION_ACT_NONE = 0,  /* 只到点，不动作 */
    MISSION_ACT_GRAB,      /* 到点后抓取 */
    MISSION_ACT_RELEASE,   /* 到点后放置 */
} mission_act_t;

/**
 * @brief  起航点编排任务（建航点队列 + 串行执行任务）
 * @retval APP_OK / APP_ERR
 */
app_status_t app_mission_init(void);

/**
 * @brief  往航点队列塞一个航点 + 到点动作（"想去哪点就发哪点"）
 * @param  pt      目标点（世界系 mm）
 * @param  yaw_deg 到点航向，deg
 * @param  act     到点后执行的动作
 * @retval APP_OK / APP_ERR（队列满或未初始化）
 */
app_status_t app_mission_push(map_point_t pt, float yaw_deg, mission_act_t act);

#ifdef __cplusplus
}
#endif

#endif /* APP_MISSION_H */
