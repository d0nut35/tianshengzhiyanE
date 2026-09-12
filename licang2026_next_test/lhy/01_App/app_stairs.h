/**
 * @file    app_stairs.h
 * @brief   阶梯三层连续横移段：层事件上报 + 暂停/恢复
 * @note    - 阻塞接口，只能在底盘任务上下文调用
 *          - 需在底盘已到阶梯起始工作位（即低层起点）后调用
 */

#ifndef APP_STAIRS_H
#define APP_STAIRS_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#include "app_main.h"   /* app_status_t */

/**
 * @brief  阶梯三层连续横移：低层等 CAM_READY 起步，越层只报事件不停车，
 *         中层走完停车并报 STAIRS_FINISHED
 * @param  req_id 阶梯阶段请求编号，回执时原样带回
 * @retval APP_OK / APP_ERR=位姿读取或命令下发失败（已停车）
 */
app_status_t stairs_sweep(uint16_t req_id);

#ifdef __cplusplus
}
#endif

#endif /* APP_STAIRS_H */
