/**
 * @file    ic_card_device.c
 * @brief   IC卡Service响应到比赛球行列语义的转换实现。
 */

#include "ic_card_device.h"

#include <stdbool.h>
#include <string.h>

#if IC_CARD_DEVICE_EXTENDED_API_ENABLE
#include "ic_card_service_os.h"
#endif

typedef struct {
    bool initialized;
    ic_card_device_submit_fn_t submit_fn;
    void *submit_ctx;
    uint32_t next_request_id;
    bool read_pending;
    uint32_t read_request_id;
    ic_card_device_read_done_fn_t read_done_cb;
    void *read_user_ctx;
} ic_card_device_context_t;

static ic_card_device_context_t g_ic_card_device;

/**
 * @brief 将Device抽象提交调用转发到默认直连Service OS。
 * @param submit_ctx 未使用。
 * @param request IC卡事务请求。
 * @param queue_timeout_ms OS队列等待时间。
 * @return 直连Service OS提交结果。
 */
#if IC_CARD_DEVICE_EXTENDED_API_ENABLE
static ic_card_status_t ic_card_device_direct_submit(
    void *submit_ctx,
    const ic_card_request_t *request,
    uint32_t queue_timeout_ms)
{
    (void)submit_ctx;
    return ic_card_service_os_submit(request, queue_timeout_ms);
}
#endif

/**
 * @brief 分配一个非零请求编号。
 * @return 本次请求的唯一编号。
 * @note 编号自然回绕时主动跳过0，便于把0识别为“尚未分配”。
 */
static uint32_t ic_card_device_allocate_request_id(void)
{
    ++g_ic_card_device.next_request_id;
    if (g_ic_card_device.next_request_id == 0U) {
        ++g_ic_card_device.next_request_id;
    }
    return g_ic_card_device.next_request_id;
}

/**
 * @brief 把Service原始响应转换为比赛球结果并转交用户回调。
 * @param user_ctx Device层上下文，由提交请求时绑定。
 * @param request_id 已完成请求的编号。
 * @param status Service给出的事务结果。
 * @param response 成功时的读卡器响应；失败时可能为NULL。
 * @note 本函数运行在普通worker上下文，不在ISR中执行。它先释放pending槽位
 *       再调用用户代码，因此回调可以立即提交下一次读球。
 */
static void ic_card_device_read_bridge(
    void *user_ctx,
    uint32_t request_id,
    ic_card_status_t status,
    const ic_card_response_t *response)
{
    ic_card_device_context_t *ctx = (ic_card_device_context_t *)user_ctx;
    ic_card_device_read_done_fn_t done_cb = NULL;
    void *done_ctx = NULL;
    ic_card_ball_result_t result;
    uint8_t block_data[IC_CARD_BLOCK_DATA_SIZE];

    if ((ctx == NULL) || !ctx->read_pending ||
        (ctx->read_request_id != request_id)) {
        return;
    }
    done_cb = ctx->read_done_cb;
    done_ctx = ctx->read_user_ctx;
    ctx->read_pending = false;
    ctx->read_done_cb = NULL;
    ctx->read_user_ctx = NULL;

    (void)memset(&result, 0, sizeof(result));
    if ((status == IC_CARD_OK) && (response != NULL)) {
#if IC_CARD_DEVICE_RAW_RESULT_ENABLE
        result.response = *response;
#endif
        status = ic_block_data(
            response,
            IC_CARD_DEVICE_ADDRESS,
            block_data);
        if (status == IC_CARD_OK) {
            if (!ic_ball_rule_2026_decode(block_data, &result.ball)) {
                status = IC_CARD_ERR_PROTOCOL;
            }
#if IC_CARD_DEVICE_RAW_RESULT_ENABLE
            (void)memcpy(result.block_data, block_data, sizeof(block_data));
#endif
        }
    }
    if (done_cb != NULL) {
        done_cb(
            done_ctx,
            request_id,
            status,
            (status == IC_CARD_OK) ? &result : NULL);
    }
}

/**
 * @brief 初始化IC读球Device层及其Service OS依赖。
 * @return 成功返回IC_CARD_OK；重复初始化或底层初始化失败时返回对应错误。
 * @note 本函数只建立软件对象和worker，不代表真实读卡器通信已经成功。
 */
#if IC_CARD_DEVICE_EXTENDED_API_ENABLE
ic_card_status_t ic_card_device_init(void)
{
    ic_card_status_t status;

    if (g_ic_card_device.initialized) {
        return IC_CARD_ERR_STATE;
    }
    (void)memset(&g_ic_card_device, 0, sizeof(g_ic_card_device));
    status = ic_card_service_os_init();
    return (status == IC_CARD_OK) ?
        ic_card_device_init_with_transport(ic_card_device_direct_submit, NULL) :
        status;
}
#endif

/** @copydoc ic_card_device_init_with_transport() */
ic_card_status_t ic_card_device_init_with_transport(
    ic_card_device_submit_fn_t submit_fn,
    void *submit_ctx)
{
    if (submit_fn == NULL) {
        return IC_CARD_ERR_PARAM;
    }
    if (g_ic_card_device.initialized) {
        return IC_CARD_ERR_STATE;
    }
    (void)memset(&g_ic_card_device, 0, sizeof(g_ic_card_device));
    g_ic_card_device.submit_fn = submit_fn;
    g_ic_card_device.submit_ctx = submit_ctx;
    g_ic_card_device.initialized = true;
    return IC_CARD_OK;
}

/**
 * @brief 提交一次“读取并解释比赛球”的异步请求。
 * @param led_beep_prompt 是否要求模块在成功读卡时进行蜂鸣/LED提示。
 * @param done_cb 事务最终完成回调，允许为NULL。
 * @param user_ctx 原样传给done_cb的用户上下文。
 * @return 请求成功入队返回IC_CARD_OK，否则返回未初始化、忙或队列错误。
 * @note 返回OK只表示请求已经入队，球值必须以后续完成回调为准。
 */
ic_card_status_t ic_card_device_read_competition_ball(
    bool led_beep_prompt,
    ic_card_device_read_done_fn_t done_cb,
    void *user_ctx)
{
    ic_card_request_t request;
    ic_card_status_t status;

    if (!g_ic_card_device.initialized) {
        return IC_CARD_ERR_NOT_INIT;
    }
    if (g_ic_card_device.read_pending) {
        return IC_CARD_ERR_BUSY;
    }

    (void)memset(&request, 0, sizeof(request));
    request.request_id = ic_card_device_allocate_request_id();
    request.type = IC_CARD_REQUEST_READ_BLOCK;
    request.address = IC_CARD_DEVICE_ADDRESS;
    request.timeout_ms = IC_CARD_DEVICE_READ_TIMEOUT_MS;
    request.data.read_block.block = IC_CARD_DEVICE_DATA_BLOCK;
    request.data.read_block.led_beep_prompt = led_beep_prompt;
    request.done_cb = ic_card_device_read_bridge;
    request.user_ctx = &g_ic_card_device;

    g_ic_card_device.read_pending = true;
    g_ic_card_device.read_request_id = request.request_id;
    g_ic_card_device.read_done_cb = done_cb;
    g_ic_card_device.read_user_ctx = user_ctx;
    status = g_ic_card_device.submit_fn(
        g_ic_card_device.submit_ctx,
        &request,
        IC_CARD_DEVICE_QUEUE_TIMEOUT_MS);
    if (status != IC_CARD_OK) {
        g_ic_card_device.read_pending = false;
        g_ic_card_device.read_done_cb = NULL;
        g_ic_card_device.read_user_ctx = NULL;
    }
    return status;
}

/**
 * @brief 提交一条读卡器参数查询请求。
 * @param command 允许的B0/B1/B2/B8查询命令。
 * @param done_cb 查询完成回调，允许为NULL。
 * @param user_ctx 原样传给done_cb的用户上下文。
 * @return 请求入队结果；返回OK不代表模块已经回复。
 */
#if IC_CARD_DEVICE_EXTENDED_API_ENABLE
ic_card_status_t ic_card_device_query(
    ic_card_command_t command,
    ic_card_request_done_fn_t done_cb,
    void *user_ctx)
{
    ic_card_request_t request;

    if (!g_ic_card_device.initialized) {
        return IC_CARD_ERR_NOT_INIT;
    }
    (void)memset(&request, 0, sizeof(request));
    request.request_id = ic_card_device_allocate_request_id();
    request.type = IC_CARD_REQUEST_QUERY;
    request.address = IC_CARD_DEVICE_ADDRESS;
    request.timeout_ms = IC_CARD_DEVICE_READ_TIMEOUT_MS;
    request.data.query.command = command;
    request.done_cb = done_cb;
    request.user_ctx = user_ctx;
    return g_ic_card_device.submit_fn(
        g_ic_card_device.submit_ctx,
        &request,
        IC_CARD_DEVICE_QUEUE_TIMEOUT_MS);
}
#endif
