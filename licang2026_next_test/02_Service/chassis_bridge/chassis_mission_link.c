/**
 * @file    chassis_mission_link.c
 * @brief   Mission任务与底盘任务之间的双向消息队列。
 */

#include "chassis_mission_link.h"

osMessageQueueId_t chassis_command_queue;
osMessageQueueId_t mission_event_queue;
static osThreadId_t g_mission_task;

bool chassis_mission_link_init(void)
{
    if ((chassis_command_queue != NULL) || (mission_event_queue != NULL)) {
        return false;
    }

    chassis_command_queue = osMessageQueueNew(
        CHASSIS_MISSION_QUEUE_DEPTH,
        sizeof(chassis_mission_command_t),
        NULL);
    mission_event_queue = osMessageQueueNew(
        CHASSIS_MISSION_QUEUE_DEPTH,
        sizeof(chassis_mission_event_t),
        NULL);

    if ((chassis_command_queue == NULL) || (mission_event_queue == NULL)) {
        if (chassis_command_queue != NULL) {
            (void)osMessageQueueDelete(chassis_command_queue);
            chassis_command_queue = NULL;
        }
        if (mission_event_queue != NULL) {
            (void)osMessageQueueDelete(mission_event_queue);
            mission_event_queue = NULL;
        }
        return false;
    }
    return true;
}

bool chassis_mission_link_bind_mission_task(osThreadId_t mission_task)
{
    if ((mission_task == NULL) || (g_mission_task != NULL)) {
        return false;
    }
    g_mission_task = mission_task;
    return true;
}

bool chassis_mission_link_send_command(
    const chassis_mission_command_t *command,
    uint32_t timeout_ticks)
{
    if ((command == NULL) || (chassis_command_queue == NULL)) {
        return false;
    }
    return osMessageQueuePut(
               chassis_command_queue,
               command,
               0U,
               timeout_ticks) == osOK;
}

bool chassis_mission_link_post_event(
    const chassis_mission_event_t *event,
    uint32_t timeout_ticks)
{
    uint32_t flags;

    if ((event == NULL) || (mission_event_queue == NULL) ||
        (g_mission_task == NULL)) {
        return false;
    }
    if (osMessageQueuePut(
            mission_event_queue,
            event,
            0U,
            timeout_ticks) != osOK) {
        return false;
    }

    flags = osThreadFlagsSet(g_mission_task, CHASSIS_MISSION_FLAG_EVENT);
    return (flags & osFlagsError) == 0U;
}
