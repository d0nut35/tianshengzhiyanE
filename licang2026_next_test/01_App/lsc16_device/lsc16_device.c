/**
 * @file    lsc16_device.c
 * @brief   LSC16语义命令到Service请求的转换实现。
 */

#include "lsc16_device.h"

#include <stdbool.h>
#include <string.h>

typedef struct {
    bool initialized;
    uint32_t next_request_id;
    lsc16_device_report_fn_t report_cb;
    void *report_ctx;
    struct {
        bool in_use;
        uint32_t request_id;
        lsc16_device_done_fn_t done_cb;
        void *user_ctx;
    } pending[LSC16_SERVICE_QUEUE_DEPTH + 1U];
} lsc16_device_context_t;

static lsc16_device_context_t g_lsc16_device;

/**
 * @brief 分配一个非零的LSC16请求编号。
 * @return 新请求编号。
 * @note 自然回绕时跳过0，便于调试时区分“尚未分配”。
 */
static uint32_t lsc16_device_allocate_request_id(void)
{
    ++g_lsc16_device.next_request_id;
    if (g_lsc16_device.next_request_id == 0U) {
        ++g_lsc16_device.next_request_id;
    }
    return g_lsc16_device.next_request_id;
}

/**
 * @brief 把Service解析出的控制板主动回报转交给App订阅者。
 * @param user_ctx Device层上下文。
 * @param report_events 本轮累积的回报事件位。
 * @param report 最近一次回报内容，只在回调期间有效。
 */
static void lsc16_device_report_bridge(
    void *user_ctx,
    uint32_t report_events,
    const lsc16_report_t *report)
{
    lsc16_device_context_t *ctx = (lsc16_device_context_t *)user_ctx;

    if ((ctx != NULL) && (ctx->report_cb != NULL)) {
        ctx->report_cb(ctx->report_ctx, report_events, report);
    }
}

/**
 * @brief 按请求编号找到用户回调并发布最终发送结果。
 * @param user_ctx Device层上下文。
 * @param request_id 已完成请求编号。
 * @param status Service给出的最终状态。
 * @note 先释放pending槽位再调用用户代码，使回调内可以立即提交下一动作。
 */
static void lsc16_device_done_bridge(
    void *user_ctx,
    uint32_t request_id,
    lsc16_status_t status)
{
    uint32_t i;

    (void)user_ctx;
    for (i = 0U; i < (LSC16_SERVICE_QUEUE_DEPTH + 1U); ++i) {
        if (g_lsc16_device.pending[i].in_use &&
            (g_lsc16_device.pending[i].request_id == request_id)) {
            lsc16_device_done_fn_t done_cb =
                g_lsc16_device.pending[i].done_cb;
            void *done_ctx = g_lsc16_device.pending[i].user_ctx;
            g_lsc16_device.pending[i].in_use = false;
            if (done_cb != NULL) {
                done_cb(done_ctx, request_id, status);
            }
            return;
        }
    }
}

/**
 * @brief 为语义请求分配ID、保存用户回调并提交到Service OS。
 * @param request 待提交请求；函数会补齐request_id和内部桥接回调。
 * @return 请求入队状态。
 * @note 入队失败时会回收刚占用的pending槽位，避免永久误报忙。
 */
static lsc16_status_t lsc16_device_submit(lsc16_request_t *request)
{
    uint32_t i;
    lsc16_status_t status;

    if ((request == NULL) || !g_lsc16_device.initialized) {
        return (request == NULL) ? LSC16_ERR_PARAM : LSC16_ERR_NOT_INIT;
    }
    request->request_id = lsc16_device_allocate_request_id();
    for (i = 0U; i < (LSC16_SERVICE_QUEUE_DEPTH + 1U); ++i) {
        if (!g_lsc16_device.pending[i].in_use) {
            g_lsc16_device.pending[i].in_use = true;
            g_lsc16_device.pending[i].request_id = request->request_id;
            g_lsc16_device.pending[i].done_cb = request->done_cb;
            g_lsc16_device.pending[i].user_ctx = request->user_ctx;
            request->done_cb = lsc16_device_done_bridge;
            request->user_ctx = &g_lsc16_device;
            status = lsc16_service_submit(request);
            if (status != LSC16_OK) {
                g_lsc16_device.pending[i].in_use = false;
            }
            return status;
        }
    }
    return LSC16_ERR_BUSY;
}

/**
 * @brief 初始化LSC16 Device层及其Service OS。
 * @return 成功返回LSC16_OK，重复初始化或底层失败返回对应错误。
 */
lsc16_status_t lsc16_device_init(void)
{
    lsc16_status_t status;

    if (g_lsc16_device.initialized) {
        return LSC16_ERR_STATE;
    }
    (void)memset(&g_lsc16_device, 0, sizeof(g_lsc16_device));
    status = lsc16_service_init(
        lsc16_device_report_bridge,
        &g_lsc16_device);
    if (status == LSC16_OK) {
        g_lsc16_device.initialized = true;
    }
    return status;
}

/**
 * @brief 设置动作组状态和电池回报订阅者。
 * @param report_cb App回调；传NULL表示只解析、不向App发布。
 * @param user_ctx 原样传给report_cb的用户上下文。
 * @return Device已初始化时返回LSC16_OK，否则返回未初始化错误。
 */
lsc16_status_t lsc16_device_set_report_callback(
    lsc16_device_report_fn_t report_cb,
    void *user_ctx)
{
    if (!g_lsc16_device.initialized) {
        return LSC16_ERR_NOT_INIT;
    }
    g_lsc16_device.report_cb = report_cb;
    g_lsc16_device.report_ctx = user_ctx;
    return LSC16_OK;
}

/**
 * @brief 提交一次单舵机位置运动请求。
 * @param servo_id 舵机ID，范围0~15。
 * @param position 目标脉宽，范围500~2500。
 * @param move_time_ms 运动时间，单位ms。
 * @param done_cb UART命令事务完成回调，允许为NULL。
 * @param user_ctx 原样传给done_cb的上下文。
 * @return 请求入队状态。
 * @note 本接口复用多舵机编码路径，避免产生两套0x03实现。
 */
lsc16_status_t lsc16_device_move_servo(
    uint8_t servo_id,
    uint16_t position,
    uint16_t move_time_ms,
    lsc16_device_done_fn_t done_cb,
    void *user_ctx)
{
    lsc16_servo_target_t target = {servo_id, position};
    return lsc16_device_move_servos(
        &target,
        1U,
        move_time_ms,
        done_cb,
        user_ctx);
}

/**
 * @brief 提交一次多舵机同步运动请求。
 * @param targets 舵机目标数组。
 * @param servo_count 目标数量，范围1~16。
 * @param move_time_ms 所有目标共用的运动时间，单位ms。
 * @param done_cb UART命令事务完成回调，允许为NULL。
 * @param user_ctx 原样传给done_cb的上下文。
 * @return 参数合法且成功入队时返回LSC16_OK。
 * @warning 完成回调只证明命令事务结束，不等于机械运动已经到位。
 */
lsc16_status_t lsc16_device_move_servos(
    const lsc16_servo_target_t *targets,
    uint8_t servo_count,
    uint16_t move_time_ms,
    lsc16_device_done_fn_t done_cb,
    void *user_ctx)
{
    lsc16_request_t request;

    if ((targets == NULL) || (servo_count == 0U) ||
        (servo_count > LSC16_SERVO_COUNT_MAX)) {
        return LSC16_ERR_PARAM;
    }
    (void)memset(&request, 0, sizeof(request));
    request.type = LSC16_REQUEST_MOVE_SERVOS;
    request.data.move.count = servo_count;
    request.data.move.move_time_ms = move_time_ms;
    (void)memcpy(
        request.data.move.targets,
        targets,
        (size_t)servo_count * sizeof(targets[0]));
    request.done_cb = done_cb;
    request.user_ctx = user_ctx;
    return lsc16_device_submit(&request);
}

/**
 * @brief 提交运行动作组请求。
 * @param action_group 动作组编号。
 * @param repeat_count 重复次数，0表示循环执行。
 * @param done_cb 命令事务完成回调，允许为NULL。
 * @param user_ctx 原样传给done_cb的上下文。
 * @return 请求入队状态。
 * @note App不直接接触UART8；动作实际完成应观察0x08主动回报。
 */
lsc16_status_t lsc16_device_run_action_group(
    uint8_t action_group,
    uint16_t repeat_count,
    lsc16_device_done_fn_t done_cb,
    void *user_ctx)
{
    lsc16_request_t request;

    (void)memset(&request, 0, sizeof(request));
    request.type = LSC16_REQUEST_RUN_ACTION_GROUP;
    request.data.action_run.action_group = action_group;
    request.data.action_run.repeat_count = repeat_count;
    request.done_cb = done_cb;
    request.user_ctx = user_ctx;
    return lsc16_device_submit(&request);
}

/**
 * @brief 提交停止当前动作组请求。
 * @param done_cb 命令事务完成回调，允许为NULL。
 * @param user_ctx 原样传给done_cb的上下文。
 * @return 请求入队状态。
 */
lsc16_status_t lsc16_device_stop_action_group(
    lsc16_device_done_fn_t done_cb,
    void *user_ctx)
{
    lsc16_request_t request;

    (void)memset(&request, 0, sizeof(request));
    request.type = LSC16_REQUEST_STOP_ACTION_GROUP;
    request.done_cb = done_cb;
    request.user_ctx = user_ctx;
    return lsc16_device_submit(&request);
}

/**
 * @brief 提交动作组速度设置请求。
 * @param action_group 动作组编号，0xFF表示全部动作组。
 * @param speed_percent 速度百分比参数。
 * @param done_cb 命令事务完成回调，允许为NULL。
 * @param user_ctx 原样传给done_cb的上下文。
 * @return 请求入队状态。
 */
lsc16_status_t lsc16_device_set_action_group_speed(
    uint8_t action_group,
    uint16_t speed_percent,
    lsc16_device_done_fn_t done_cb,
    void *user_ctx)
{
    lsc16_request_t request;

    (void)memset(&request, 0, sizeof(request));
    request.type = LSC16_REQUEST_SET_ACTION_SPEED;
    request.data.action_speed.action_group = action_group;
    request.data.action_speed.speed_percent = speed_percent;
    request.done_cb = done_cb;
    request.user_ctx = user_ctx;
    return lsc16_device_submit(&request);
}

/**
 * @brief 提交控制板电池电压查询请求。
 * @param done_cb 查询命令发送完成回调，允许为NULL。
 * @param user_ctx 原样传给done_cb的上下文。
 * @return 请求入队状态。
 * @note 电压数值通过控制板主动回报订阅接口异步发布。
 */
lsc16_status_t lsc16_device_request_battery_voltage(
    lsc16_device_done_fn_t done_cb,
    void *user_ctx)
{
    lsc16_request_t request;

    (void)memset(&request, 0, sizeof(request));
    request.type = LSC16_REQUEST_GET_BATTERY;
    request.done_cb = done_cb;
    request.user_ctx = user_ctx;
    return lsc16_device_submit(&request);
}
