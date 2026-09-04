/**
 * @file    mission_app.h
 * @brief   整机比赛任务的唯一上层状态机接口。
 *
 * Mission只向底盘发送阶段命令；坐标、速度和路线参数仍由底盘持有。
 * Mission直接调用现有Device接口；设备回调只唤醒Mission，不在回调中推进状态。
 */

#ifndef MISSION_APP_H
#define MISSION_APP_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    MISSION_APP_OK = 0,
    MISSION_APP_ERR_PARAM,
    MISSION_APP_ERR_STATE,
    MISSION_APP_ERR_BUSY,
    MISSION_APP_ERR_RESOURCE,
    MISSION_APP_ERR_IO,
} mission_app_status_t;

/** Mission当前唯一等待步骤；不存在并行修改的第二套flow状态。 */
typedef enum {
    MISSION_STATE_BOOT = 0,  
    MISSION_STATE_WAIT_HOME,  
    MISSION_STATE_WAIT_CHASSIS_READY,
    MISSION_STATE_READY,

    MISSION_STATE_WAIT_PLATFORM,
    MISSION_STATE_PLATFORM_WAIT_POSE,
    MISSION_STATE_PLATFORM_WAIT_VISION,
    MISSION_STATE_PLATFORM_WAIT_TARGET,
    MISSION_STATE_PLATFORM_WAIT_GRASP,
    MISSION_STATE_PLATFORM_WAIT_RELEASE,
    MISSION_STATE_PLATFORM_WAIT_STORAGE,
    MISSION_STATE_PLATFORM_WAIT_RETURN,
    MISSION_STATE_PLATFORM_WAIT_DEPARTURE_POSE,

    MISSION_STATE_WAIT_STAIRS,
    MISSION_STATE_STAIR_WAIT_POSE,
    MISSION_STATE_STAIR_WAIT_LAYER,
    MISSION_STATE_STAIR_WAIT_VISION_START,
    MISSION_STATE_STAIR_SCANNING,
    MISSION_STATE_STAIR_WAIT_PAUSE,
    MISSION_STATE_STAIR_WAIT_GRASP,
    MISSION_STATE_STAIR_WAIT_STORAGE,
    MISSION_STATE_STAIR_WAIT_RETURN,
    MISSION_STATE_STAIR_WAIT_VISION_RESUME,
    MISSION_STATE_STAIR_WAIT_RESUME,
    
    MISSION_STATE_STOPPING,
    MISSION_STATE_STOPPED,
    MISSION_STATE_COMPLETE,
    MISSION_STATE_FAULT,
} mission_state_t;

typedef enum {
    MISSION_COLOR_NONE = 0,
    MISSION_COLOR_RED,
    MISSION_COLOR_BLUE,
} mission_color_t;

typedef enum {
    MISSION_STAIR_NONE = 0,
    MISSION_STAIR_LOW,
    MISSION_STAIR_HIGH,
    MISSION_STAIR_MID,
} mission_stair_layer_t;

typedef enum {
    MISSION_USER_COMMAND_NONE = 0,
    MISSION_USER_COMMAND_START_RED,
    MISSION_USER_COMMAND_START_BLUE,
    MISSION_USER_COMMAND_STOP,
} mission_user_command_t;

typedef struct {
    mission_state_t state;
    mission_color_t color;
    mission_stair_layer_t stair_layer;
    uint16_t chassis_request_id;
    uint8_t platform_balls;
    uint8_t stair_balls;
    uint8_t storage_slot;
    uint8_t fault_code;
} mission_app_snapshot_t;

/** 注册现有Device回调，并创建Mission命令队列和唯一Mission任务。 */
mission_app_status_t mission_app_init(void);

/** 从任务上下文提交红方、蓝方或停止命令。 */
mission_app_status_t mission_app_submit_command(mission_user_command_t command);

/** 复制当前状态，供调试命令读取。 */
mission_app_status_t mission_app_get_snapshot(mission_app_snapshot_t *snapshot);

#ifdef __cplusplus
}
#endif

#endif /* MISSION_APP_H */
