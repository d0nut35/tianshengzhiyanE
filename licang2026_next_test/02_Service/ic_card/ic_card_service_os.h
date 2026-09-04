/**
 * @file    ic_card_service_os.h
 * @brief   IC卡Service的CMSIS-RTOS2直连UART7装配层。
 */

#ifndef IC_CARD_SERVICE_OS_H
#define IC_CARD_SERVICE_OS_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#include "ic_card_service.h"

/**
 * @brief 创建消息队列和worker，并完成IC卡UART7直连链路装配。
 * @return IC_CARD_OK表示初始化成功，否则返回资源创建或底层绑定错误。
 * @pre 必须在osKernelInitialize()之后、osKernelStart()之前调用。
 */
ic_card_status_t ic_card_service_os_init(void);

/**
 * @brief 将一个IC卡请求提交给CMSIS-RTOS2 worker。
 * @param request 待提交请求。
 * @param queue_timeout_ms 等待消息队列空位的最长时间。
 * @return IC_CARD_OK表示已入队，否则返回参数、状态或队列满错误。
 * @note 请求在返回前按值复制，调用者可立即释放栈对象。
 */
ic_card_status_t ic_card_service_os_submit(
    const ic_card_request_t *request,
    uint32_t queue_timeout_ms);

/**
 * @brief 主动推进一次底层Service状态机。
 * @note 仅供测试或诊断；正式FreeRTOS路径由唯一worker拥有。
 */
void ic_card_service_os_process_once(void);

/**
 * @brief 获取默认IC卡Service的统计快照。
 * @param stats 输出统计对象。
 * @return IC_CARD_OK表示成功，否则返回参数或未初始化错误。
 */
ic_card_status_t ic_card_service_os_get_stats(ic_card_service_stats_t *stats);

#ifdef __cplusplus
}
#endif

#endif /* IC_CARD_SERVICE_OS_H */
