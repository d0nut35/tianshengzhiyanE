/**
 * @file    ic_card_device.h
 * @brief   比赛任务使用的IC卡读球Device接口。
 *
 * 上层只请求“读取一个球”，无需知道A3、块号、校验或UART7。当前直连测试
 * 使用UART7 Service OS；后续复用通道1接入时保持此语义接口不变。
 */

#ifndef IC_CARD_DEVICE_H
#define IC_CARD_DEVICE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#include "ic_ball_rule_2026.h"
#include "ic_card_device_config.h"
#include "ic_card_service.h"

typedef struct {
    ic_card_ball_info_t ball;
#if IC_CARD_DEVICE_RAW_RESULT_ENABLE
    uint8_t block_data[IC_CARD_BLOCK_DATA_SIZE];
    ic_card_response_t response;
#endif
} ic_card_ball_result_t;

/**
 * @brief IC比赛球读取完成回调类型。
 * @param user_ctx 提交请求时提供的用户上下文。
 * @param request_id 已完成请求的编号。
 * @param status 最终事务和规则解析状态。
 * @param result 成功时的球结果，失败时为NULL。
 * @warning result只在回调期间有效；需要入队或绑定转盘槽位时必须立即按值复制。
 */
typedef void (*ic_card_device_read_done_fn_t)(
    void *user_ctx,
    uint32_t request_id,
    ic_card_status_t status,
    const ic_card_ball_result_t *result);

/**
 * @brief Device向具体事务链路提交IC卡请求的抽象接口。
 * @param submit_ctx 初始化时绑定的链路上下文。
 * @param request 待提交请求；实现必须在返回前复制其内容。
 * @param queue_timeout_ms 等待底层队列空位的最长时间。
 * @return 提交结果。
 * @note 直连模式由IC卡Service OS实现，复用模式可转成mult_uart事务。
 */
typedef ic_card_status_t (*ic_card_device_submit_fn_t)(
    void *submit_ctx,
    const ic_card_request_t *request,
    uint32_t queue_timeout_ms);

/**
 * @brief 初始化直连UART7的Service OS和Device请求管理。
 * @return 初始化成功返回IC_CARD_OK，否则返回具体错误。
 */
#if IC_CARD_DEVICE_EXTENDED_API_ENABLE
ic_card_status_t ic_card_device_init(void);
#endif

/**
 * @brief 使用调用者提供的事务链路初始化IC卡Device。
 * @param submit_fn 非阻塞提交函数。
 * @param submit_ctx 原样传给submit_fn的上下文。
 * @return 初始化结果。
 * @warning 本函数不初始化UART或worker，调用者必须先装配好具体传输链路。
 */
ic_card_status_t ic_card_device_init_with_transport(
    ic_card_device_submit_fn_t submit_fn,
    void *submit_ctx);

/**
 * @brief 读取2026比赛球的0扇区块1。
 * @param led_beep_prompt true表示A3命令要求读卡成功时蜂鸣/LED提示。
 * @param done_cb 最终完成回调，允许为NULL。
 * @param user_ctx 原样传给done_cb的用户上下文。
 * @return 请求入队状态；实际读取结果由done_cb异步返回。
 */
ic_card_status_t ic_card_device_read_competition_ball(
    bool led_beep_prompt,
    ic_card_device_read_done_fn_t done_cb,
    void *user_ctx);

/**
 * @brief 查询读卡器地址、模式、蜂鸣器或自动读卡配置。
 * @param command B0/B1/B2/B8中的一种查询命令。
 * @param done_cb 查询完成回调，允许为NULL。
 * @param user_ctx 原样传给done_cb的用户上下文。
 * @return 请求入队状态；模块响应由done_cb异步返回。
 */
#if IC_CARD_DEVICE_EXTENDED_API_ENABLE
ic_card_status_t ic_card_device_query(
    ic_card_command_t command,
    ic_card_request_done_fn_t done_cb,
    void *user_ctx);
#endif

#ifdef __cplusplus
}
#endif

#endif /* IC_CARD_DEVICE_H */
