/**
 * @file    chassis_mission_link.h
 * @brief   Mission任务与底盘任务之间的公共消息约定。
 *
 * 双方只通过两个单向队列传递本文件中的小消息。消息不携带坐标、速度、
 * HAL对象、RTOS对象或设备数据，底盘路线参数由底盘内部管理。
 */

#ifndef CHASSIS_MISSION_LINK_H
#define CHASSIS_MISSION_LINK_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define CHASSIS_MISSION_LINK_VERSION       1U
#define CHASSIS_MISSION_REQUEST_ID_INVALID 0U

/** Mission -> 底盘。 */
typedef uint8_t mission_command_type_t;
enum {
    MISSION_CMD_NONE = 0U,
    MISSION_CMD_MISSION_READY, /* Mission初始化完成。 */
    MISSION_CMD_GO_PLATFORM,   /* 执行起点到圆盘工作位。 */
    MISSION_CMD_GO_STAIRS,     /* 执行圆盘到阶梯工作位。 */
    MISSION_CMD_CAM_READY,     /* 当前阶梯层视觉已经准备好。 */
    MISSION_CMD_STAIR_STOP,    /* 识别到球，请求暂停横移。 */
    MISSION_CMD_STAIR_RESUME,  /* 抓取完成，请求恢复横移。 */
    MISSION_CMD_STOP,          /* 安全停止当前底盘动作。 */
};

/** 底盘 -> Mission。 */
typedef uint8_t chassis_command_type_t;
enum {
    CHASSIS_CMD_NONE = 0U,
    CHASSIS_CMD_MISSION_READY,   /* 底盘初始化完成。 */
    CHASSIS_CMD_PLATFORM_READY,  /* 已到圆盘工作位。 */
    CHASSIS_CMD_STAIRS_READY,    /* 已到阶梯起始工作位。 */
    CHASSIS_CMD_STAIR_LOW,       /* 低层扫描段就绪。 */
    CHASSIS_CMD_STAIR_HIGH,      /* 高层扫描段就绪。 */
    CHASSIS_CMD_STAIR_MID,       /* 中层扫描段就绪。 */
    CHASSIS_CMD_STAIR_PAUSE,     /* 横移已经实际暂停。 */
    CHASSIS_CMD_STAIR_RESUME,    /* 横移已经恢复。 */
    CHASSIS_CMD_STAIRS_FINISHED, /* 中层结束，阶梯路段完成。 */
    CHASSIS_CMD_STOPPED,         /* 底盘已经停止。 */
};

/** Mission写入底盘命令队列。 */
typedef struct {
    uint16_t request_id;
    mission_command_type_t type;
    uint8_t is_ready;
} chassis_mission_command_t;

/** 底盘写入Mission事件队列。 */
typedef struct {
    uint16_t request_id;
    chassis_command_type_t type;
    uint8_t is_ready;
} chassis_mission_event_t;

#ifdef __cplusplus
}
#endif

#endif /* CHASSIS_MISSION_LINK_H */
