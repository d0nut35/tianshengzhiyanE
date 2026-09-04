/**
 * @file    lsc16_service_os.h
 * @brief   LSC16 Service的CMSIS-RTOS2单worker接入层。
 */

#ifndef LSC16_SERVICE_OS_H
#define LSC16_SERVICE_OS_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#include "lsc16_service.h"

typedef struct lsc16_service_os lsc16_service_os_t;

/**
 * @brief 装配UART8/DMA/路由并创建LSC16 worker。
 * @param report_cb 舵控板主动回报出口，可为NULL。
 * @param report_ctx 原样传给report_cb的上下文。
 * @return LSC16_OK表示成功，否则返回资源创建或底层绑定错误。
 * @pre 必须在osKernelInitialize()之后、osKernelStart()之前调用。
 */
lsc16_status_t lsc16_service_os_init(
    lsc16_report_fn_t report_cb,
    void *report_ctx);

/**
 * @brief 将一个舵控请求提交给CMSIS-RTOS2 worker。
 * @param request 待提交请求。
 * @param queue_timeout_ms 等待消息队列空位的最长时间。
 * @return LSC16_OK表示已入队，否则返回参数、状态或队列满错误。
 * @note 请求结构会按值复制，包括最多16个舵机目标；调用者的栈对象可立即释放。
 */
lsc16_status_t lsc16_service_os_submit(
    const lsc16_request_t *request,
    uint32_t queue_timeout_ms);

/**
 * @brief 主动推进一次底层Service状态机。
 * @note 仅供测试或诊断；正式FreeRTOS路径由唯一worker拥有。
 */
void lsc16_service_os_process_once(void);

/**
 * @brief 获取默认LSC16 Service的统计快照。
 * @param stats 输出统计对象。
 * @return LSC16_OK表示成功，否则返回参数或未初始化错误。
 */
lsc16_status_t lsc16_service_os_get_stats(lsc16_service_stats_t *stats);

/**
 * @brief 获取默认OS装配对象。
 * @return 初始化成功后返回有效对象；尚未初始化时返回NULL。
 * @note 仅用于系统装配和诊断，不应由业务层直接修改内部成员。
 */
lsc16_service_os_t *lsc16_service_os_get_default(void);

#ifdef __cplusplus
}
#endif

#endif /* LSC16_SERVICE_OS_H */
