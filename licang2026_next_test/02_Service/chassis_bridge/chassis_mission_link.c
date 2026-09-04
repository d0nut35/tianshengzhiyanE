/**
 * @file    chassis_mission_link.c
 * @brief   Mission任务与底盘任务之间的双向消息队列。
 */

#include "chassis_mission_link.h"

osMessageQueueId_t chassis_command_queue;
osMessageQueueId_t mission_event_queue;

bool chassis_mission_link_init(void)
{
    chassis_command_queue = osMessageQueueNew(
        CHASSIS_MISSION_QUEUE_DEPTH,
        sizeof(chassis_mission_command_t),
        NULL);
    mission_event_queue = osMessageQueueNew(
        CHASSIS_MISSION_QUEUE_DEPTH,
        sizeof(chassis_mission_event_t),
        NULL);

    return (chassis_command_queue != NULL) &&
           (mission_event_queue != NULL);
}
