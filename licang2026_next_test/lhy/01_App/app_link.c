/**
 * @file    app_link.c
 * @brief   底盘侧 Mission 链路收发与命令分发核
 * @note    - 未被钩子消费的命令走 cmd_default：STOP 停车回执，其余丢弃
 *          - 停车经 align_stop，保证退出时底盘静止
 */

#include "app_link.h"

#include <stddef.h>

#include "cmsis_os2.h"

#include "app_util.h"
#include "chassis_align.h"

/* ===== 调试日志：0=不编译进固件，1=经 RTT 输出 ===== */
#ifndef APP_LOG_EN
#define APP_LOG_EN   0
#endif
#if APP_LOG_EN
#include "cat_log.h"
#define LK_LOGI(fmt, ...)  LOGI("[link] " fmt, ##__VA_ARGS__)
#define LK_LOGE(fmt, ...)  LOGE("[link] " fmt, ##__VA_ARGS__)
#else
#define LK_LOGI(fmt, ...)  do {} while (0)
#define LK_LOGE(fmt, ...)  do {} while (0)
#endif

#define LK_BOOT_REQ_ID  1U     /* 握手事件的非零请求编号 */
#define LK_RETRY_MS     1000U  /* 未收到 Mission 就绪时重发间隔，ms */
#define LK_POST_MS      100U   /* 事件入队等待上限，ms */

static void cmd_default(const chassis_mission_command_t *cmd);

/** @copydoc link_post */
app_status_t link_post(chassis_command_type_t type, uint16_t id, uint8_t ok)
{
    chassis_mission_event_t evt; /* 待上报事件 */

    evt.request_id = id; /* 原样带回，Mission 靠此匹配阶段 */
    evt.type = type;
    evt.is_ready = ok;
    if (!chassis_mission_link_post_event(&evt,
                                         util_ms_ticks(LK_POST_MS))) {
        LK_LOGE("post evt %d fail", (int)type);
        return APP_ERR;
    }
    return APP_OK;
}

/**
 * @brief  未被调用方消费的命令的默认处理
 * @param  cmd 出队的 Mission 命令
 * @note   STOP 无论当前阶段都要停车并回执，其余命令记录后丢弃
 */
static void cmd_default(const chassis_mission_command_t *cmd)
{
    if (cmd->type == MISSION_CMD_STOP) {
        (void)align_stop();
        (void)link_post(CHASSIS_CMD_STOPPED, cmd->request_id, 1U);
    } else {
        LK_LOGE("drop cmd %d", (int)cmd->type);
    }
}

/** @copydoc link_poll */
uint8_t link_poll(link_hook_t hook, void *ctx, uint32_t tmo)
{
    chassis_mission_command_t cmd; /* 出队的 Mission 命令 */

    if (chassis_command_queue == NULL) {
        return 0U;
    }
    if (osMessageQueueGet(chassis_command_queue, &cmd, NULL, tmo) != osOK) {
        return 0U;
    }
    /* 钩子先于默认处理，使阶段私有命令不被当成未知命令丢弃 */
    if ((hook != NULL) && (hook(&cmd, ctx) != 0U)) {
        return 1U;
    }
    cmd_default(&cmd);
    return 1U;
}

/** @copydoc link_wait */
app_status_t link_wait(mission_command_type_t type, uint16_t *id,
                      uint32_t tmo)
{
    chassis_mission_command_t cmd; /* 出队的 Mission 命令 */

    if ((id == NULL) || (chassis_command_queue == NULL)) {
        return APP_ERR;
    }
    for (;;) {
        if (osMessageQueueGet(chassis_command_queue, &cmd, NULL,
                              tmo) != osOK) {
            return APP_ERR;
        }
        if (cmd.type == type) {
            *id = cmd.request_id;
            return APP_OK;
        }
        cmd_default(&cmd); /* 非期望命令不结束等待，继续下一次出队 */
    }
}

/** @copydoc link_handshake */
void link_handshake(void)
{
    uint16_t id; /* Mission 回执编号，握手阶段不使用 */

    /* 握手不设总超时：Mission 未起来时底盘只能一直重发 */
    for (;;) {
        (void)link_post(CHASSIS_CMD_MISSION_READY, LK_BOOT_REQ_ID, 1U);
        if (link_wait(MISSION_CMD_MISSION_READY, &id,
                      util_ms_ticks(LK_RETRY_MS)) == APP_OK) {
            LK_LOGI("mission link up");
            return;
        }
    }
}
