/** @file zdt_turntable_service.h @brief ZDT转盘经UART7复用器提交事务。 */

#ifndef ZDT_TURNTABLE_SERVICE_H
#define ZDT_TURNTABLE_SERVICE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stddef.h>
#include <stdint.h>

#include "turn_bsp.h"

/**
 * @brief 一笔ZDT事务的异步完成回调。
 * @param user_ctx 提交请求时保存的调用者上下文。
 * @param request_id 原始请求标识。
 * @param status 最终事务或协议状态。
 * @param response 成功或设备错误时的解析结果，其余错误时为NULL。
 * @warning response仅在回调执行期间有效，需要保存时必须立即复制。
 */
typedef void (*zdt_turntable_done_fn_t)(
    void *user_ctx,
    uint32_t request_id,
    zdt_turntable_status_t status,
    const zdt_turntable_response_t *response);

/**
 * @brief Device交给Service的一笔完整事务。
 * @note Service在mux_submit()返回前保持本结构和frame有效，mux再按值复制TX。
 */
typedef struct {
    uint32_t request_id;
    uint8_t frame[ZDT_TURNTABLE_FRAME_MAX];
    size_t frame_len;
    uint8_t expected_address;
    uint8_t expected_function;
    uint32_t timeout_ms;
    zdt_turntable_done_fn_t done_cb;
    void *user_ctx;
} zdt_turntable_request_t;

/**
 * @brief 提交一笔非阻塞ZDT事务。
 * @param submit_ctx 保留给Device提交函数签名，本实现不使用。
 * @param request 完整请求，TX数据在函数返回前由mux复制。
 * @return OK只表示已进入mux队列；动作ACK不代表机械到位。
 * @note 同一转盘上一笔完成前返回BUSY，超时和STOP均走相同事务路径。
 */
zdt_turntable_status_t turn_service_submit(
    void *submit_ctx,
    const zdt_turntable_request_t *request);

#ifdef __cplusplus
}
#endif

#endif /* ZDT_TURNTABLE_SERVICE_H */
