/**
 * @file    app_link.h
 * @brief   底盘侧 Mission 链路：事件上报、命令等待与命令分发
 * @note    - 底盘只经本文件读写 chassis_mission_link 的两个队列
 *          - MISSION_CMD_STOP 由本层统一处理（停车 + 回 STOPPED）
 *          - 全部为阻塞接口，只能在底盘任务上下文调用
 */

#ifndef APP_LINK_H
#define APP_LINK_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#include "app_main.h"               /* app_status_t */
#include "chassis_mission_link.h"   /* 命令/事件类型 */

/**
 * @brief  命令分发钩子：处理调用方关心的命令
 * @param  cmd 出队的 Mission 命令
 * @param  ctx 调用方上下文
 * @retval 1=本命令已消费 / 0=交由 link_poll 走默认处理
 */
typedef uint8_t (*link_hook_t)(const chassis_mission_command_t *cmd,
                              void *ctx);

/**
 * @brief  向 Mission 事件队列上报一条底盘事件并唤醒 Mission 任务
 * @param  type 事件类型 CHASSIS_CMD_xxx
 * @param  id   原样带回的命令请求编号
 * @param  ok   1=成功/就绪，0=失败
 * @retval APP_OK / APP_ERR
 */
app_status_t link_post(chassis_command_type_t type, uint16_t id, uint8_t ok);

/**
 * @brief  阻塞等待指定 Mission 命令，其余命令按默认规则处理
 * @param  type 期望的命令类型 MISSION_CMD_xxx
 * @param  id   输出该命令的请求编号，回执时原样带回
 * @param  tmo  单次出队等待 tick 数，可为 osWaitForever
 * @retval APP_OK=已收到 / APP_ERR=超时或队列未建立
 * @note   等待期间底盘静止
 */
app_status_t link_wait(mission_command_type_t type, uint16_t *id,
                      uint32_t tmo);

/**
 * @brief  限时取一条 Mission 命令，先交钩子再走默认处理
 * @param  hook 命令分发钩子，可为 NULL
 * @param  ctx  传给钩子的上下文
 * @param  tmo  出队等待 tick 数，可为 osWaitForever
 * @retval 1=已处理一条命令 / 0=超时或队列未建立
 */
uint8_t link_poll(link_hook_t hook, void *ctx, uint32_t tmo);

/**
 * @brief  与 Mission 握手：上报底盘就绪，直到收到 MISSION_CMD_MISSION_READY
 * @note   Mission 只在收到底盘就绪后才回 READY，因此底盘先发、超时重发
 */
void link_handshake(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_LINK_H */
