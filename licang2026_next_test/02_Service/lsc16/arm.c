/**
 * @file    arm.c
 * @brief   机械臂CMSIS-RTOS2队列、worker和UART路由实现。
 */

#include "arm.h"

#include <stdbool.h>
#include <string.h>

#include "cmsis_os.h"
#include "uart_dispatch.h"

#define ARM_FLAG_WORK                   (1UL << 0)
#define ARM_WORKER_STACK                (512U * 4U)

typedef enum {
    LSC16_REQUEST_MOVE_SERVOS = 0,
    LSC16_REQUEST_RUN_ACTION_GROUP,
    LSC16_REQUEST_STOP_ACTION_GROUP,
    LSC16_REQUEST_SET_ACTION_SPEED,
    LSC16_REQUEST_GET_BATTERY,
} lsc16_request_type_t;

typedef struct {
    uint8_t count;
    uint16_t move_time_ms;
    lsc16_servo_target_t targets[LSC16_SERVO_COUNT_MAX];
} lsc16_move_request_t;

typedef struct {
    uint8_t action_group;
    uint16_t repeat_count;
} lsc16_action_run_request_t;

typedef struct {
    uint8_t action_group;
    uint16_t speed_percent;
} lsc16_action_speed_request_t;

typedef struct {
    uint32_t request_id;
    lsc16_request_type_t type;
    union {
        lsc16_move_request_t move;
        lsc16_action_run_request_t action_run;
        lsc16_action_speed_request_t action_speed;
    } data;
    arm_done_fn_t done_cb;
    void *user_ctx;
} lsc16_request_t;

typedef struct {
    bool initialized;
    bool active_valid;
    lsc16_t device;
    lsc16_request_t active;
    osMessageQueueId_t queue;
    osThreadId_t worker;
    arm_report_fn_t report_cb;
    void *report_ctx;
    uint32_t next_request_id;
    volatile uint32_t tx_sequence;
    volatile uint32_t error_sequence;
    uint32_t handled_tx_sequence;
    uint32_t handled_error_sequence;
    arm_stats_t stats;
} lsc16_service_context_t;

static lsc16_service_context_t g_lsc16_service;

static const osThreadAttr_t g_lsc16_worker_attr = {
    .name = "lsc16Svc",
    .stack_size = ARM_WORKER_STACK,
    .priority = (osPriority_t)osPriorityNormal,
};

/** 校验请求类型和舵机参数，避免无效请求进入异步队列。 */
static bool lsc16_request_is_valid(const lsc16_request_t *request)
{
    uint8_t i;

    if ((request == NULL) ||
        ((uint32_t)request->type > (uint32_t)LSC16_REQUEST_GET_BATTERY)) {
        return false;
    }
    if (request->type != LSC16_REQUEST_MOVE_SERVOS) {
        return true;
    }
    if ((request->data.move.count == 0U) ||
        (request->data.move.count > LSC16_SERVO_COUNT_MAX)) {
        return false;
    }
    for (i = 0U; i < request->data.move.count; ++i) {
        if ((request->data.move.targets[i].id > LSC16_SERVO_ID_MAX) ||
            (request->data.move.targets[i].position < LSC16_SERVO_POSITION_MIN) ||
            (request->data.move.targets[i].position > LSC16_SERVO_POSITION_MAX)) {
            return false;
        }
    }
    return true;
}

/** ISR只登记不可合并的完成/错误序号并唤醒worker。 */
static void lsc16_isr_notify(void *user_ctx, lsc16_isr_event_t event)
{
    lsc16_service_context_t *ctx = (lsc16_service_context_t *)user_ctx;

    if (ctx == NULL) {
        return;
    }
    if (event == LSC16_ISR_EVENT_TX_COMPLETE) {
        ++ctx->tx_sequence;
    } else if (event == LSC16_ISR_EVENT_ERROR) {
        ++ctx->error_sequence;
    }
    if (ctx->worker != NULL) {
        (void)osThreadFlagsSet(ctx->worker, ARM_FLAG_WORK);
    }
}

static bool lsc16_dispatch_tx(void *user_ctx, UART_HandleTypeDef *huart)
{
    lsc16_service_context_t *ctx = (lsc16_service_context_t *)user_ctx;
    return (ctx != NULL) && lsc16_handle_tx_complete(&ctx->device, huart);
}

static bool lsc16_dispatch_rx(
    void *user_ctx,
    UART_HandleTypeDef *huart,
    uint16_t rx_len)
{
    lsc16_service_context_t *ctx = (lsc16_service_context_t *)user_ctx;
    return (ctx != NULL) &&
        lsc16_handle_rx_event(&ctx->device, huart, rx_len);
}

static bool lsc16_dispatch_error(void *user_ctx, UART_HandleTypeDef *huart)
{
    lsc16_service_context_t *ctx = (lsc16_service_context_t *)user_ctx;
    return (ctx != NULL) && lsc16_handle_error(&ctx->device, huart);
}

/** 先释放在途槽位再回调，允许回调立即提交下一动作。 */
static void lsc16_complete_active(
    lsc16_service_context_t *ctx,
    lsc16_status_t status)
{
    lsc16_request_t completed;

    if (!ctx->active_valid) {
        return;
    }
    completed = ctx->active;
    ctx->active_valid = false;
    ++ctx->stats.completed;
    if (status != LSC16_OK) {
        ++ctx->stats.io_errors;
    }
    if (completed.done_cb != NULL) {
        completed.done_cb(completed.user_ctx, completed.request_id, status);
    }
}

/** 把统一请求转换为BSP命令；TX缓冲由BSP持续持有。 */
static lsc16_status_t lsc16_start_request(
    lsc16_service_context_t *ctx,
    const lsc16_request_t *request)
{
    switch (request->type) {
    case LSC16_REQUEST_MOVE_SERVOS:
        return lsc16_move_servos(
            &ctx->device,
            request->data.move.targets,
            request->data.move.count,
            request->data.move.move_time_ms);
    case LSC16_REQUEST_RUN_ACTION_GROUP:
        return lsc16_run_action_group(
            &ctx->device,
            request->data.action_run.action_group,
            request->data.action_run.repeat_count);
    case LSC16_REQUEST_STOP_ACTION_GROUP:
        return lsc16_stop_action_group(&ctx->device);
    case LSC16_REQUEST_SET_ACTION_SPEED:
        return lsc16_set_action_group_speed(
            &ctx->device,
            request->data.action_speed.action_group,
            request->data.action_speed.speed_percent);
    case LSC16_REQUEST_GET_BATTERY:
        return lsc16_request_battery_voltage(&ctx->device);
    default:
        return LSC16_ERR_PARAM;
    }
}

/** 在worker上下文解析并交付舵控板主动回报。 */
static void lsc16_publish_reports(lsc16_service_context_t *ctx)
{
    uint32_t events;
    lsc16_report_t report;

    lsc16_process(&ctx->device);
    events = lsc16_take_report_events(&ctx->device);
    if (events == LSC16_REPORT_EVENT_NONE) {
        return;
    }
    ++ctx->stats.reports;
    if ((ctx->report_cb != NULL) &&
        (lsc16_get_last_report(&ctx->device, &report) == LSC16_OK)) {
        ctx->report_cb(ctx->report_ctx, events, &report);
    }
}

/** 处理ISR事件并从唯一CMSIS队列启动下一笔请求。 */
static void lsc16_process_service(lsc16_service_context_t *ctx)
{
    lsc16_status_t status;

    lsc16_publish_reports(ctx);
    if (ctx->handled_error_sequence != ctx->error_sequence) {
        ctx->handled_error_sequence = ctx->error_sequence;
        ctx->handled_tx_sequence = ctx->tx_sequence;
        status = lsc16_recover(&ctx->device);
        lsc16_complete_active(
            ctx,
            (status == LSC16_OK) ? LSC16_ERR_IO : status);
    } else if (ctx->handled_tx_sequence != ctx->tx_sequence) {
        ctx->handled_tx_sequence = ctx->tx_sequence;
        lsc16_complete_active(ctx, LSC16_OK);
    }

    while (!ctx->active_valid) {
        if (osMessageQueueGet(ctx->queue, &ctx->active, NULL, 0U) != osOK) {
            return;
        }
        ctx->active_valid = true;
        status = lsc16_start_request(ctx, &ctx->active);
        if (status == LSC16_OK) {
            return;
        }
        lsc16_complete_active(ctx, status);
    }
}

static void lsc16_worker_entry(void *argument)
{
    lsc16_service_context_t *ctx = (lsc16_service_context_t *)argument;

    for (;;) {
        lsc16_process_service(ctx);
        (void)osThreadFlagsWait(
            ARM_FLAG_WORK,
            osFlagsWaitAny,
            osWaitForever);
    }
}

lsc16_status_t arm_init(void)
{
    lsc16_service_context_t *ctx = &g_lsc16_service;
    uart_dispatch_handler_t handler = {0};
    uart_dispatch_handle_t dispatch_handle;
    lsc16_status_t status;

    if (ctx->initialized) {
        return LSC16_ERR_STATE;
    }
    (void)memset(ctx, 0, sizeof(*ctx));
    ctx->queue = osMessageQueueNew(
        ARM_QUEUE_DEPTH,
        sizeof(lsc16_request_t),
        NULL);
    if (ctx->queue == NULL) {
        return LSC16_ERR_IO;
    }
    status = lsc16_init(&ctx->device);
    if (status != LSC16_OK) {
        return status;
    }
    status = lsc16_bind_isr_notify(&ctx->device, lsc16_isr_notify, ctx);
    if (status != LSC16_OK) {
        return status;
    }
    handler.tx_complete = lsc16_dispatch_tx;
    handler.rx_event = lsc16_dispatch_rx;
    handler.error = lsc16_dispatch_error;
    handler.user_ctx = ctx;
    if (!uart_dispatch_register(&handler, &dispatch_handle)) {
        return LSC16_ERR_IO;
    }
    ctx->worker = osThreadNew(lsc16_worker_entry, ctx, &g_lsc16_worker_attr);
    if (ctx->worker == NULL) {
        return LSC16_ERR_IO;
    }
    ctx->initialized = true;
    return LSC16_OK;
}

lsc16_status_t arm_on_report(arm_report_fn_t callback, void *user_ctx)
{
    if (!g_lsc16_service.initialized) {
        return LSC16_ERR_NOT_INIT;
    }
    g_lsc16_service.report_cb = callback;
    g_lsc16_service.report_ctx = user_ctx;
    return LSC16_OK;
}

static lsc16_status_t arm_submit(lsc16_request_t *request)
{
    lsc16_service_context_t *ctx = &g_lsc16_service;
    osStatus_t os_status;

    if (!ctx->initialized) {
        return (request == NULL) ? LSC16_ERR_PARAM : LSC16_ERR_NOT_INIT;
    }
    if (!lsc16_request_is_valid(request)) {
        ++ctx->stats.rejected;
        return LSC16_ERR_PARAM;
    }
    ++ctx->next_request_id;
    if (ctx->next_request_id == 0U) {
        ++ctx->next_request_id;
    }
    request->request_id = ctx->next_request_id;
    os_status = osMessageQueuePut(
        ctx->queue,
        request,
        0U,
        0U);
    if (os_status != osOK) {
        ++ctx->stats.rejected;
        return ((os_status == osErrorResource) || (os_status == osErrorTimeout))
            ? LSC16_ERR_QUEUE_FULL
            : LSC16_ERR_IO;
    }
    ++ctx->stats.submitted;
    (void)osThreadFlagsSet(ctx->worker, ARM_FLAG_WORK);
    return LSC16_OK;
}

lsc16_status_t arm_move(
    uint8_t servo_id,
    uint16_t position,
    uint16_t move_time_ms,
    arm_done_fn_t done_cb,
    void *user_ctx)
{
    lsc16_servo_target_t target = {servo_id, position};

    return arm_move_all(
        &target,
        1U,
        move_time_ms,
        done_cb,
        user_ctx);
}

lsc16_status_t arm_move_all(
    const lsc16_servo_target_t *targets,
    uint8_t servo_count,
    uint16_t move_time_ms,
    arm_done_fn_t done_cb,
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
    return arm_submit(&request);
}

lsc16_status_t arm_run(
    uint8_t action_group,
    uint16_t repeat_count,
    arm_done_fn_t done_cb,
    void *user_ctx)
{
    lsc16_request_t request;

    (void)memset(&request, 0, sizeof(request));
    request.type = LSC16_REQUEST_RUN_ACTION_GROUP;
    request.data.action_run.action_group = action_group;
    request.data.action_run.repeat_count = repeat_count;
    request.done_cb = done_cb;
    request.user_ctx = user_ctx;
    return arm_submit(&request);
}

lsc16_status_t arm_stop(arm_done_fn_t done_cb, void *user_ctx)
{
    lsc16_request_t request;

    (void)memset(&request, 0, sizeof(request));
    request.type = LSC16_REQUEST_STOP_ACTION_GROUP;
    request.done_cb = done_cb;
    request.user_ctx = user_ctx;
    return arm_submit(&request);
}

lsc16_status_t arm_speed(
    uint8_t action_group,
    uint16_t speed_percent,
    arm_done_fn_t done_cb,
    void *user_ctx)
{
    lsc16_request_t request;

    (void)memset(&request, 0, sizeof(request));
    request.type = LSC16_REQUEST_SET_ACTION_SPEED;
    request.data.action_speed.action_group = action_group;
    request.data.action_speed.speed_percent = speed_percent;
    request.done_cb = done_cb;
    request.user_ctx = user_ctx;
    return arm_submit(&request);
}

lsc16_status_t arm_battery(arm_done_fn_t done_cb, void *user_ctx)
{
    lsc16_request_t request;

    (void)memset(&request, 0, sizeof(request));
    request.type = LSC16_REQUEST_GET_BATTERY;
    request.done_cb = done_cb;
    request.user_ctx = user_ctx;
    return arm_submit(&request);
}

lsc16_status_t arm_get_stats(arm_stats_t *stats)
{
    if (stats == NULL) {
        return LSC16_ERR_PARAM;
    }
    if (!g_lsc16_service.initialized) {
        return LSC16_ERR_NOT_INIT;
    }
    *stats = g_lsc16_service.stats;
    return LSC16_OK;
}
