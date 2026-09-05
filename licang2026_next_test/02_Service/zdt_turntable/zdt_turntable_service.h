/** @file zdt_turntable_service.h @brief ZDT转盘Service与设备语义接口。 */
#ifndef ZDT_TURNTABLE_SERVICE_H
#define ZDT_TURNTABLE_SERVICE_H
#ifdef __cplusplus
extern "C" {
#endif
#include "turn_bsp.h"

typedef struct {
    uint8_t address;
    uint32_t timeout_ms;
    uint32_t emm_pulses_per_revolution;
} turn_config_t;

typedef void (*turn_done_fn_t)(void *user_ctx, uint32_t request_id,
    zdt_turntable_status_t status, const zdt_turntable_response_t *response);

/** 初始化转盘语义Service；UART7复用器必须已初始化。 */
zdt_turntable_status_t turn_init(const turn_config_t *config);
/** 查询0x1A并缓存固件、闭环和Scale能力。 */
zdt_turntable_status_t turn_query_options(turn_done_fn_t done_cb, void *user_ctx);
/** 查询0x3A电机状态。 */
zdt_turntable_status_t turn_query_status(turn_done_fn_t done_cb, void *user_ctx);
/** 发送已确认Emm固件的位置命令。 */
zdt_turntable_status_t turn_move_emm(
    const zdt_turntable_position_command_t *command,
    turn_done_fn_t done_cb, void *user_ctx);
/** 发送立即停止命令。 */
zdt_turntable_status_t turn_stop(turn_done_fn_t done_cb, void *user_ctx);

#if !LICANG_RELEASE_MINIMAL
/** 测试和诊断用查询入口。 */
zdt_turntable_status_t turn_query(uint8_t function, turn_done_fn_t done_cb,
    void *user_ctx);
/** 非最小构建保留X/Emm固件通用位置入口。 */
zdt_turntable_status_t turn_move(const zdt_turntable_position_command_t *command,
    turn_done_fn_t done_cb, void *user_ctx);
#endif

#ifdef __cplusplus
}
#endif
#endif /* ZDT_TURNTABLE_SERVICE_H */
