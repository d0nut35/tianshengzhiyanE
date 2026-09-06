/**
 * @file    mission_app.c
 * @brief   整机比赛任务初版状态机实现。
 *
 * 本文件是Mission状态的唯一写入者。底盘消息由公共队列保存，设备完成
 * 回调只记录结果并设置线程标志；所有状态转换都在mission_task_entry中串行执行。
 */

#include "mission_app.h"

#include <string.h>

#include "FreeRTOS.h"
#include "ball_manifest_core.h"
#include "cmsis_os.h"
#include "cmsis_compiler.h"
#include "ic_card_service.h"
#include "task.h"

#include "chassis_mission_link.h"
#include "arm.h"
#include "mission_config.h"
#include "mux_service.h"
#include "nano_vision_core.h"
#include "gate.h"
#include "zdt_turntable_service.h"

#define MISSION_FLAG_COMMAND       (1UL << 1)
#define MISSION_FLAG_ARM_OK        (1UL << 2)
#define MISSION_FLAG_ARM_FAIL      (1UL << 3)
#define MISSION_FLAG_VISION_DONE   (1UL << 4)
#define MISSION_FLAG_IC_DONE       (1UL << 5)
#define MISSION_FLAG_ZDT_DONE      (1UL << 6)

#define MISSION_ALL_FLAGS \
    (CHASSIS_MISSION_FLAG_EVENT | MISSION_FLAG_COMMAND | \
     MISSION_FLAG_ARM_OK | MISSION_FLAG_ARM_FAIL | \
     MISSION_FLAG_VISION_DONE)

typedef enum {
    MISSION_VISION_SCENE_PLATFORM = 1,
    MISSION_VISION_SCENE_STAIR,
} mission_vision_scene_t;

typedef enum {
    MISSION_STORAGE_REGION_PLATFORM = 1,
    MISSION_STORAGE_REGION_STAIR,
} mission_storage_region_t;

typedef enum {
    MISSION_VISION_IDLE = 0,
    MISSION_VISION_STARTING,
    MISSION_VISION_LISTENING,
    MISSION_VISION_ACKING,
    MISSION_VISION_STOPPING,
} mission_vision_phase_t;

typedef enum {
    MISSION_FAULT_NONE = 0,
    MISSION_FAULT_TIMEOUT,
    MISSION_FAULT_CHASSIS,
    MISSION_FAULT_ARM,
    MISSION_FAULT_VISION,
    MISSION_FAULT_STORAGE,
    MISSION_FAULT_QUEUE,
    MISSION_FAULT_PROTOCOL,
} mission_fault_t;

typedef struct {
    mission_vision_phase_t phase;
    bool inflight;
    bool stop_requested;
    uint8_t next_sequence;
    uint16_t next_session_id;
    uint16_t session_id;
    nano_vision_scene_t scene;
    uint8_t tx[NANO_VISION_FRAME_MAX];
    volatile mult_uart_status_t mail_status;
    volatile uint16_t mail_len;
    uint8_t mail_data[NANO_VISION_FRAME_MAX];
} mission_vision_t;

typedef struct {
    volatile ic_card_status_t ic_status;
    ic_ball_t ic_ball;
    volatile zdt_turntable_status_t zdt_status;
    volatile bool zdt_has_response;
    zdt_turntable_response_t zdt_response;
} mission_storage_t;

typedef struct {
    /* RTOS对象只在初始化阶段写入。 */
    osThreadId_t task;
    osMessageQueueId_t command_queue;

    /* 单一state描述当前唯一允许完成的异步操作。 */
    mission_state_t state;
    mission_color_t color;
    mission_stair_layer_t stair_layer;
    uint16_t request_id;
    uint32_t deadline_tick;

    /* 比赛流程数据由mission_task_entry唯一写入。 */
    uint8_t platform_balls;
    uint8_t stair_balls;
    uint8_t storage_slot;
    uint8_t fault_code;

    /* 设备协议细节收在子对象中，顶层流程仍只使用一个state。 */
    mission_vision_t vision;
    mission_storage_t storage;
    ball_manifest_t manifest;

    /* 初始化握手允许Mission和底盘以任意先后顺序完成。 */
    bool initialized;
    bool chassis_ready;
    uint8_t active_arm_group;
} mission_context_t;

static mission_context_t g_mission;

static const osThreadAttr_t g_mission_task_attr = {
    .name = "mission_app",
    .stack_size = MISSION_TASK_STACK_SIZE,
    .priority = osPriorityNormal,
};

static uint32_t mission_ms_to_ticks(uint32_t ms);
static uint32_t mission_wait_ticks(const mission_context_t *ctx);
static void mission_task_entry(void *argument);
static void mission_arm_tx_done(
    void *user_ctx,
    uint32_t request_id,
    lsc16_status_t status);
static void mission_arm_report(
    void *user_ctx,
    uint32_t report_events,
    const lsc16_report_t *report);
static void mission_vision_done(
    void *user_ctx,
    const mux_completion_t *completion);
static void mission_ic_done(
    void *user_ctx,
    uint32_t request_id,
    ic_card_status_t status,
    const ic_result_t *result);
static void mission_zdt_done(
    void *user_ctx,
    uint32_t request_id,
    zdt_turntable_status_t status,
    const zdt_turntable_response_t *response);
static void mission_enter_state(
    mission_context_t *ctx,
    mission_state_t state,
    uint32_t timeout_ms);
static void mission_fail(mission_context_t *ctx, mission_fault_t fault);
static uint16_t mission_next_request_id(mission_context_t *ctx);
static bool mission_send_chassis(
    mission_command_type_t type,
    uint16_t request_id);
static bool mission_start_arm(
    mission_context_t *ctx,
    uint8_t action_group,
    mission_state_t wait_state);
static bool mission_start_vision(
    mission_context_t *ctx,
    mission_vision_scene_t scene,
    mission_stair_layer_t layer,
    mission_state_t wait_state);
static bool mission_stop_vision(mission_context_t *ctx);
static void mission_reset_vision(mission_context_t *ctx);
static bool mission_store_ball(
    mission_context_t *ctx,
    mission_storage_region_t region);
static void mission_handle_vision(mission_context_t *ctx);
static void mission_vision_process(mission_context_t *ctx);
static void mission_try_ready(mission_context_t *ctx);
static void mission_start_run(mission_context_t *ctx, mission_color_t color);
static void mission_start_stair_layer(mission_context_t *ctx);
static uint8_t mission_stair_grasp_group(mission_stair_layer_t layer);
static void mission_handle_command(
    mission_context_t *ctx,
    mission_user_command_t command);
static void mission_handle_chassis(
    mission_context_t *ctx,
    const chassis_mission_event_t *event);
static void mission_handle_arm(mission_context_t *ctx, bool success);
static void mission_handle_storage(mission_context_t *ctx);
static void mission_check_timeout(mission_context_t *ctx);

/** 把毫秒转换为CMSIS-RTOS tick，非零毫秒至少返回1 tick。 */
static uint32_t mission_ms_to_ticks(uint32_t ms)
{
    uint32_t tick_hz = osKernelGetTickFreq();
    uint32_t ticks = (ms * tick_hz + 999U) / 1000U;

    return (ticks == 0U) ? 1U : ticks;
}

/** 把复用串口事务结果转换为Nano协议层状态。 */
static nano_vision_status_t mission_map_vision_status(mult_uart_status_t status)
{
    if (status == MULT_UART_OK) return NANO_VISION_OK;
    if (status == MULT_UART_ERR_TIMEOUT) return NANO_VISION_ERR_TIMEOUT;
    if (status == MULT_UART_ERR_BUSY) return NANO_VISION_ERR_BUSY;
    if (status == MULT_UART_ERR_QUEUE_FULL) return NANO_VISION_ERR_QUEUE_FULL;
    return NANO_VISION_ERR_IO;
}

/** LSC16命令事务失败时唤醒Mission；发送成功仍需等待动作组0x08回报。 */
static void mission_arm_tx_done(
    void *user_ctx,
    uint32_t request_id,
    lsc16_status_t status)
{
    mission_context_t *ctx = (mission_context_t *)user_ctx;

    (void)request_id;
    if ((ctx != NULL) && (ctx->task != NULL) && (status != LSC16_OK)) {
        (void)osThreadFlagsSet(ctx->task, MISSION_FLAG_ARM_FAIL);
    }
}

/** LSC16主动回报只转成Mission唤醒信号，状态转换仍由Mission任务完成。 */
static void mission_arm_report(
    void *user_ctx,
    uint32_t report_events,
    const lsc16_report_t *report)
{
    mission_context_t *ctx = (mission_context_t *)user_ctx;
    uint32_t flag = 0U;

    if ((ctx == NULL) || (report == NULL) ||
        (report->action_group != ctx->active_arm_group)) {
        return;
    }
    if ((report_events & LSC16_REPORT_EVENT_ACTION_COMPLETED) != 0U) {
        flag = MISSION_FLAG_ARM_OK;
    } else if ((report_events & (LSC16_REPORT_EVENT_ACTION_STOPPED |
                                 LSC16_REPORT_EVENT_INVALID_FRAME)) != 0U) {
        flag = MISSION_FLAG_ARM_FAIL;
    }
    if ((ctx != NULL) && (ctx->task != NULL) && (flag != 0U)) {
        (void)osThreadFlagsSet(ctx->task, flag);
    }
}

/** 复用串口回调只复制Nano回复并唤醒Mission，协议解释仍在Mission任务中。 */
static void mission_vision_done(
    void *user_ctx,
    const mux_completion_t *completion)
{
    mission_context_t *ctx = (mission_context_t *)user_ctx;
    size_t copy_len;

    if ((ctx == NULL) || (completion == NULL)) return;
    ctx->vision.mail_status = completion->status;
    copy_len = completion->rx_len;
    if (copy_len > sizeof(ctx->vision.mail_data)) {
        copy_len = sizeof(ctx->vision.mail_data);
        ctx->vision.mail_status = MULT_UART_ERR_OVERFLOW;
    }
    if ((copy_len > 0U) && (completion->rx_data != NULL)) {
        (void)memcpy(ctx->vision.mail_data, completion->rx_data, copy_len);
    }
    ctx->vision.mail_len = (uint16_t)copy_len;
    ctx->vision.inflight = false;
    __DMB();
    (void)osThreadFlagsSet(ctx->task, MISSION_FLAG_VISION_DONE);
}

/** IC完成回调保存按值结果；五次重试和建档由Mission任务决定。 */
static void mission_ic_done(
    void *user_ctx,
    uint32_t request_id,
    ic_card_status_t status,
    const ic_result_t *result)
{
    mission_context_t *ctx = (mission_context_t *)user_ctx;

    (void)request_id;
    if (ctx == NULL) return;
    ctx->storage.ic_status = status;
    if ((status == IC_CARD_OK) && (result != NULL)) {
        ctx->storage.ic_ball = result->ball;
    }
    __DMB();
    (void)osThreadFlagsSet(ctx->task, MISSION_FLAG_IC_DONE);
}

/** ZDT完成回调保存回复；到位、堵转和PB0判断留给Mission任务。 */
static void mission_zdt_done(
    void *user_ctx,
    uint32_t request_id,
    zdt_turntable_status_t status,
    const zdt_turntable_response_t *response)
{
    mission_context_t *ctx = (mission_context_t *)user_ctx;

    (void)request_id;
    if (ctx == NULL) return;
    ctx->storage.zdt_status = status;
    ctx->storage.zdt_has_response = (response != NULL);
    if (response != NULL) ctx->storage.zdt_response = *response;
    __DMB();
    (void)osThreadFlagsSet(ctx->task, MISSION_FLAG_ZDT_DONE);
}

/** 返回距当前状态超时的剩余tick；无超时状态永久阻塞等待事件。 */
static uint32_t mission_wait_ticks(const mission_context_t *ctx)
{
    uint32_t now;
    if (ctx->deadline_tick == 0U) {
        return osWaitForever;
    }
    now = osKernelGetTickCount();
    if ((int32_t)(ctx->deadline_tick - now) <= 0) {
        return 0U;
    }
    return ctx->deadline_tick - now;
}

/** 设置顶层状态及该状态的绝对超时；timeout_ms为0表示不计时。 */
static void mission_enter_state(
    mission_context_t *ctx,
    mission_state_t state,
    uint32_t timeout_ms)
{
    ctx->state = state;
    ctx->deadline_tick = (timeout_ms == 0U)
        ? 0U
        : osKernelGetTickCount() + mission_ms_to_ticks(timeout_ms);
}

/** 锁存首个故障、停止视觉并请求底盘停车。 */
static void mission_fail(mission_context_t *ctx, mission_fault_t fault)
{
    uint16_t request_id;

    if ((ctx->state == MISSION_STATE_FAULT) ||
        (ctx->state == MISSION_STATE_STOPPED)) {
        return;
    }
    ctx->fault_code = (uint8_t)fault;
    mission_reset_vision(ctx);
    request_id = mission_next_request_id(ctx);
    (void)mission_send_chassis(MISSION_CMD_STOP, request_id);
    mission_enter_state(ctx, MISSION_STATE_FAULT, 0U);
}

/** 生成非零请求编号，回绕时跳过协议保留值0。 */
static uint16_t mission_next_request_id(mission_context_t *ctx)
{
    ++ctx->request_id;
    if (ctx->request_id == CHASSIS_MISSION_REQUEST_ID_INVALID) {
        ++ctx->request_id;
    }
    return ctx->request_id;
}

/** 向底盘命令队列写入当前线性流程的一条控制命令。 */
static bool mission_send_chassis(
    mission_command_type_t type,
    uint16_t request_id)
{
    chassis_mission_command_t command;

    command.request_id = request_id;
    command.type = type;
    command.is_ready = 1U;
    return chassis_mission_link_send_command(&command, 0U);
}

/** 提交动作组；当前状态本身记录该动作完成后应该进入哪一步。 */
static bool mission_start_arm(
    mission_context_t *ctx,
    uint8_t action_group,
    mission_state_t wait_state)
{
    ctx->active_arm_group = action_group;
    if (arm_run(
            action_group,
            1U,
            mission_arm_tx_done,
            ctx) != LSC16_OK) {
        ctx->active_arm_group = 0U;
        return false;
    }
    mission_enter_state(ctx, wait_state, MISSION_OPERATION_TIMEOUT_MS);
    return true;
}

/** 生成Nano帧使用的非零8位序号。 */
static uint8_t mission_next_vision_sequence(mission_vision_t *vision)
{
    ++vision->next_sequence;
    if (vision->next_sequence == 0U) ++vision->next_sequence;
    return vision->next_sequence;
}

/** 提交一笔Nano读写事务并保持发送缓冲区到回调完成。 */
static nano_vision_status_t mission_submit_vision_transfer(
    mission_context_t *ctx,
    mult_uart_operation_t operation,
    size_t tx_len,
    uint32_t timeout_ms)
{
    mux_transfer_t transfer;
    mult_uart_status_t status;

    if (ctx->vision.inflight) return NANO_VISION_ERR_BUSY;
    (void)memset(&transfer, 0, sizeof(transfer));
    transfer.device = MISSION_VISION_DEVICE_ID;
    transfer.operation = operation;
    transfer.tx_data = (tx_len > 0U) ? ctx->vision.tx : NULL;
    transfer.tx_len = tx_len;
    transfer.rx_capacity = (operation == MULT_UART_OP_WRITE) ?
        0U : NANO_VISION_FRAME_MAX;
    transfer.io_timeout_ms = timeout_ms;
    transfer.done_cb = mission_vision_done;
    transfer.user_ctx = ctx;
    ctx->vision.inflight = true;
    status = mux_submit(&transfer);
    if (status != MULT_UART_OK) ctx->vision.inflight = false;
    return mission_map_vision_status(status);
}

/** 清除当前视觉会话状态；已经结束的复用事务无需另行取消。 */
static void mission_reset_vision(mission_context_t *ctx)
{
    ctx->vision.phase = MISSION_VISION_IDLE;
    ctx->vision.session_id = 0U;
    ctx->vision.scene = NANO_VISION_SCENE_NONE;
    ctx->vision.stop_requested = false;
}

/** 开启Nano会话；场景和颜色由F7命令决定，Nano无需人工切换模式。 */
static bool mission_start_vision(
    mission_context_t *ctx,
    mission_vision_scene_t scene,
    mission_stair_layer_t layer,
    mission_state_t wait_state)
{
    nano_vision_session_t session;
    nano_vision_status_t status;
    size_t tx_len = 0U;

    if (ctx->vision.phase != MISSION_VISION_IDLE) return false;
    ++ctx->vision.next_session_id;
    if (ctx->vision.next_session_id == 0U) ++ctx->vision.next_session_id;
    ctx->vision.session_id = ctx->vision.next_session_id;
    if (scene == MISSION_VISION_SCENE_PLATFORM) {
        ctx->vision.scene = NANO_VISION_SCENE_TURNTABLE;
    } else if (layer == MISSION_STAIR_LOW) {
        ctx->vision.scene = NANO_VISION_SCENE_STAIR_LOW;
    } else if (layer == MISSION_STAIR_HIGH) {
        ctx->vision.scene = NANO_VISION_SCENE_STAIR_HIGH;
    } else if (layer == MISSION_STAIR_MID) {
        ctx->vision.scene = NANO_VISION_SCENE_STAIR_MID;
    } else {
        mission_reset_vision(ctx);
        return false;
    }
    session.session_id = ctx->vision.session_id;
    session.scene = ctx->vision.scene;
    session.target_color = (ctx->color == MISSION_COLOR_RED) ?
        NANO_VISION_COLOR_RED : NANO_VISION_COLOR_BLUE;
    status = nano_vision_build_session_start_frame(
        mission_next_vision_sequence(&ctx->vision),
        &session,
        ctx->vision.tx,
        sizeof(ctx->vision.tx),
        &tx_len);
    if (status != NANO_VISION_OK) {
        mission_reset_vision(ctx);
        return false;
    }
    ctx->vision.phase = MISSION_VISION_STARTING;
    mission_enter_state(ctx, wait_state, MISSION_OPERATION_TIMEOUT_MS);
    status = mission_submit_vision_transfer(
        ctx, MULT_UART_OP_WRITE_READ, tx_len, MISSION_VISION_TIMEOUT_MS);
    if (status != NANO_VISION_OK) {
        mission_reset_vision(ctx);
        return false;
    }
    return true;
}

/** 结束无目标的旧层会话，收到STOPPED后再开启下一层会话。 */
static bool mission_stop_vision(mission_context_t *ctx)
{
    nano_vision_status_t status;
    size_t tx_len = 0U;

    if ((ctx->vision.phase == MISSION_VISION_IDLE) ||
        (ctx->vision.session_id == 0U)) {
        return true;
    }
    if (ctx->vision.inflight) {
        ctx->vision.stop_requested = true;
        return true;
    }
    ctx->vision.stop_requested = false;
    status = nano_vision_build_session_stop_frame(
        mission_next_vision_sequence(&ctx->vision),
        ctx->vision.session_id,
        ctx->vision.tx,
        sizeof(ctx->vision.tx),
        &tx_len);
    if (status != NANO_VISION_OK) return false;
    ctx->vision.phase = MISSION_VISION_STOPPING;
    return mission_submit_vision_transfer(
               ctx, MULT_UART_OP_WRITE_READ, tx_len,
               MISSION_VISION_TIMEOUT_MS) == NANO_VISION_OK;
}

/** 连续读取PB0，只有全部样本为高才认为槽位已经对准。 */
static bool mission_gate_is_stably_high(void)
{
    uint8_t sample;

    for (sample = 0U; sample < MISSION_GATE_CONFIRM_SAMPLES; ++sample) {
        if (!gate_read()) return false;
        if ((sample + 1U) < MISSION_GATE_CONFIRM_SAMPLES) {
            (void)osDelay(mission_ms_to_ticks(
                MISSION_GATE_CONFIRM_INTERVAL_MS));
        }
    }
    return true;
}

/** 等待一笔设备事务完成；设备回调只写结果并设置对应线程标志。 */
static bool mission_wait_device(uint32_t flag, uint32_t timeout_ms)
{
    uint32_t result = osThreadFlagsWait(
        flag, osFlagsWaitAny, mission_ms_to_ticks(timeout_ms));

    return ((result & osFlagsError) == 0U) && ((result & flag) != 0U);
}

/** 按正式Emm固件参数提交一次粗调或微调运动。 */
static zdt_turntable_status_t mission_submit_slot_motion(
    mission_context_t *ctx,
    uint32_t angle_0p1deg,
    uint16_t emm_speed_rpm)
{
    zdt_turntable_position_command_t command = {0};

    command.direction = MISSION_SLOT_USE_CW ?
        ZDT_TURNTABLE_DIR_CW : ZDT_TURNTABLE_DIR_CCW;
    command.mode = ZDT_TURNTABLE_POS_RELATIVE_LAST_TARGET;
    command.speed = emm_speed_rpm;
    command.angle_0p1deg = angle_0p1deg;
    command.emm_acceleration = MISSION_ZDT_ACCEL;
    return turn_move_emm(&command, mission_zdt_done, ctx);
}

/** 提交ZDT事务前清除旧完成标志，提交后同步等待当前事务结果。 */
static bool mission_wait_zdt(mission_context_t *ctx)
{
    return mission_wait_device(
               MISSION_FLAG_ZDT_DONE,
               MISSION_ZDT_IO_TIMEOUT_MS + 100U) &&
           (ctx->storage.zdt_status == ZDT_TURNTABLE_OK) &&
           ctx->storage.zdt_has_response;
}

/** 将成功读卡或READ_FAILED结果追加到当前比赛球档案。 */
static bool mission_record_ball(
    mission_context_t *ctx,
    mission_storage_region_t storage_region,
    bool read_ok)
{
    ball_manifest_region_t region;
    ball_manifest_color_t color;
    ball_manifest_status_t status;

    region = (storage_region == MISSION_STORAGE_REGION_PLATFORM) ?
        BALL_MANIFEST_REGION_TURNTABLE : BALL_MANIFEST_REGION_STAIR;
    color = (ctx->color == MISSION_COLOR_RED) ?
        BALL_MANIFEST_COLOR_RED : BALL_MANIFEST_COLOR_BLUE;
    if (read_ok) {
        status = ball_manifest_append(
            &ctx->manifest,
            region,
            color,
            ctx->storage.ic_ball.code,
            ctx->storage.ic_ball.row,
            ctx->storage.ic_ball.column,
            ctx->storage_slot);
    } else {
        status = ball_manifest_append_read_failed(
            &ctx->manifest, region, color, ctx->storage_slot);
    }
    return status == BALL_MANIFEST_OK;
}

/**
 * 读取一次球并写入档案。IC失败最多重试五次；耗尽后记录READ_FAILED继续存球。
 */
static bool mission_read_ball(
    mission_context_t *ctx,
    mission_storage_region_t region)
{
    uint8_t attempt;
    bool read_ok = false;

    for (attempt = 0U; attempt < MISSION_IC_MAX_ATTEMPTS; ++attempt) {
        (void)osThreadFlagsClear(MISSION_FLAG_IC_DONE);
        ctx->storage.ic_status = IC_CARD_ERR_BUSY;
        if ((ic_read(
                 MISSION_IC_OPERATION_PROMPT != 0U,
                 mission_ic_done,
                 ctx) == IC_CARD_OK) &&
            mission_wait_device(
                MISSION_FLAG_IC_DONE,
                IC_READ_TIMEOUT_MS + 100U) &&
            (ctx->storage.ic_status == IC_CARD_OK)) {
            read_ok = true;
            break;
        }
        if ((attempt + 1U) < MISSION_IC_MAX_ATTEMPTS) {
            (void)osDelay(mission_ms_to_ticks(MISSION_IC_RETRY_MS));
        }
    }
    return mission_record_ball(ctx, region, read_ok);
}

/** 转盘走完一格；粗转到位后用PB0确认，未对准时按原参数逐步微调。 */
static bool mission_advance_slot(mission_context_t *ctx)
{
    const zdt_turntable_response_t *response = &ctx->storage.zdt_response;
    uint32_t started_tick = osKernelGetTickCount();
    uint8_t fine_steps = 0U;
    bool coarse = true;

    for (;;) {
        (void)osThreadFlagsClear(MISSION_FLAG_ZDT_DONE);
        ctx->storage.zdt_has_response = false;
        if ((mission_submit_slot_motion(
                 ctx,
                 coarse ? MISSION_ZDT_COARSE_ANGLE_0P1DEG :
                          MISSION_ZDT_FINE_ANGLE_0P1DEG,
                 coarse ? MISSION_ZDT_SPEED_RPM :
                          MISSION_ZDT_FINE_SPEED_RPM) !=
             ZDT_TURNTABLE_OK) ||
            !mission_wait_zdt(ctx) ||
            ((response->kind != ZDT_TURNTABLE_REPLY_ACK) &&
             (response->kind != ZDT_TURNTABLE_REPLY_REACHED))) {
            return false;
        }
        coarse = false;

        do {
            if ((osKernelGetTickCount() - started_tick) >=
                mission_ms_to_ticks(MISSION_ZDT_SLOT_TIMEOUT_MS)) {
                return false;
            }
            (void)osDelay(mission_ms_to_ticks(MISSION_ZDT_STATUS_POLL_MS));
            (void)osThreadFlagsClear(MISSION_FLAG_ZDT_DONE);
            ctx->storage.zdt_has_response = false;
            if ((turn_query_status(mission_zdt_done, ctx) !=
                 ZDT_TURNTABLE_OK) ||
                !mission_wait_zdt(ctx) ||
                (response->kind != ZDT_TURNTABLE_REPLY_STATUS) ||
                !response->data.motor_status.enabled ||
                response->data.motor_status.stalled ||
                response->data.motor_status.stall_protected ||
                response->data.motor_status.power_loss_latched) {
                return false;
            }
        } while (!response->data.motor_status.reached);

        if (mission_gate_is_stably_high() ||
            (fine_steps >= MISSION_ZDT_FINE_MAX_STEPS)) {
            return true;
        }
        ++fine_steps;
    }
}

/** Mission只调用这一个入口：读IC并建档，然后将车载转盘推进到下一槽。 */
static bool mission_store_ball(
    mission_context_t *ctx,
    mission_storage_region_t region)
{
    return mission_read_ball(ctx, region) && mission_advance_slot(ctx);
}

/** 上电只查询一次转盘固件和闭环配置，后续存球直接使用缓存结果。 */
static bool mission_prepare_zdt(mission_context_t *ctx)
{
    const zdt_turntable_response_t *response = &ctx->storage.zdt_response;

    (void)osThreadFlagsClear(MISSION_FLAG_ZDT_DONE);
    ctx->storage.zdt_has_response = false;
    if ((turn_query_options(mission_zdt_done, ctx) != ZDT_TURNTABLE_OK) ||
        !mission_wait_zdt(ctx) ||
        (response->kind != ZDT_TURNTABLE_REPLY_OPTIONS) ||
        !response->data.options.closed_loop ||
        (response->data.options.firmware != ZDT_TURNTABLE_FIRMWARE_EMM)) {
        return false;
    }
    return true;
}

static void mission_handle_vision(mission_context_t *ctx)
{
    nano_vision_status_t status;
    nano_vision_session_t session;
    nano_vision_event_t event;
    nano_vision_event_ack_t ack;
    size_t tx_len = 0U;

    status = mission_map_vision_status(ctx->vision.mail_status);
    if (ctx->vision.stop_requested &&
        (ctx->vision.phase != MISSION_VISION_STOPPING)) {
        ctx->vision.stop_requested = false;
        if (!mission_stop_vision(ctx)) {
            mission_fail(ctx, MISSION_FAULT_VISION);
        }
        return;
    }
    if ((ctx->vision.phase == MISSION_VISION_LISTENING) &&
        (status == NANO_VISION_ERR_TIMEOUT)) {
        return;
    }
    if (status != NANO_VISION_OK) {
        mission_fail(ctx, MISSION_FAULT_VISION);
        return;
    }
    if (ctx->vision.phase == MISSION_VISION_ACKING) {
        return;
    }
    if (ctx->vision.phase == MISSION_VISION_STOPPING) {
        uint16_t stopped_session = 0U;

        status = nano_vision_decode_session_stopped(
            ctx->vision.mail_data, ctx->vision.mail_len, &stopped_session);
        if ((status != NANO_VISION_OK) ||
            (stopped_session != ctx->vision.session_id)) {
            mission_fail(ctx, MISSION_FAULT_VISION);
            return;
        }
        mission_reset_vision(ctx);
        if (ctx->state == MISSION_STATE_STAIR_WAIT_LAYER) {
            mission_start_stair_layer(ctx);
        }
        return;
    }
    if (ctx->vision.phase == MISSION_VISION_STARTING) {
        status = nano_vision_decode_session_ready(
            ctx->vision.mail_data, ctx->vision.mail_len, &session);
        if ((status != NANO_VISION_OK) ||
            (session.session_id != ctx->vision.session_id) ||
            (session.scene != ctx->vision.scene) ||
            (session.target_color != ((ctx->color == MISSION_COLOR_RED) ?
                NANO_VISION_COLOR_RED : NANO_VISION_COLOR_BLUE))) {
            mission_fail(ctx, MISSION_FAULT_VISION);
            return;
        }
        ctx->vision.phase = MISSION_VISION_LISTENING;
        if (ctx->state == MISSION_STATE_PLATFORM_WAIT_VISION) {
            mission_enter_state(ctx, MISSION_STATE_PLATFORM_WAIT_TARGET,
                                MISSION_OPERATION_TIMEOUT_MS);
        } else if (ctx->state == MISSION_STATE_STAIR_WAIT_VISION_RESUME) {
            if (!mission_send_chassis(MISSION_CMD_STAIR_RESUME,
                                      ctx->request_id)) {
                mission_fail(ctx, MISSION_FAULT_QUEUE);
                return;
            }
            mission_enter_state(ctx, MISSION_STATE_STAIR_WAIT_RESUME,
                                MISSION_OPERATION_TIMEOUT_MS);
        } else {
            if (!mission_send_chassis(MISSION_CMD_CAM_READY, ctx->request_id)) {
                mission_fail(ctx, MISSION_FAULT_QUEUE);
                return;
            }
            mission_enter_state(ctx, MISSION_STATE_STAIR_SCANNING,
                                MISSION_OPERATION_TIMEOUT_MS);
        }
        return;
    }
    if (ctx->vision.phase != MISSION_VISION_LISTENING) return;
    status = nano_vision_decode_event(
        ctx->vision.mail_data, ctx->vision.mail_len, &event);
    if ((status != NANO_VISION_OK) ||
        (event.session_id != ctx->vision.session_id) ||
        (event.observation.scene != ctx->vision.scene) ||
        (event.observation.status != NANO_VISION_OBS_VALID) ||
        (event.observation.color != ((ctx->color == MISSION_COLOR_RED) ?
            NANO_VISION_COLOR_RED : NANO_VISION_COLOR_BLUE)) ||
        (event.observation.age_ms > MISSION_VISION_EVENT_MAX_AGE_MS)) {
        return;
    }
    ack.session_id = ctx->vision.session_id;
    ack.frame_id = event.observation.frame_id;
    status = nano_vision_build_event_ack_frame(
        mission_next_vision_sequence(&ctx->vision),
        &ack,
        ctx->vision.tx,
        sizeof(ctx->vision.tx),
        &tx_len);
    if (status != NANO_VISION_OK) {
        mission_fail(ctx, MISSION_FAULT_VISION);
        return;
    }
    ctx->vision.phase = MISSION_VISION_ACKING;
    if (mission_submit_vision_transfer(
            ctx, MULT_UART_OP_WRITE, tx_len, MISSION_VISION_TIMEOUT_MS) !=
        NANO_VISION_OK) {
        mission_fail(ctx, MISSION_FAULT_VISION);
        return;
    }
    if (ctx->state == MISSION_STATE_STAIR_SCANNING) {
        if (!mission_send_chassis(MISSION_CMD_STAIR_STOP, ctx->request_id)) {
            mission_fail(ctx, MISSION_FAULT_QUEUE);
            return;
        }
        mission_enter_state(ctx, MISSION_STATE_STAIR_WAIT_PAUSE,
                            MISSION_OPERATION_TIMEOUT_MS);
    } else {
        /* ACK发送完成后由mission_vision_process启动动作组12。 */
        mission_enter_state(ctx, MISSION_STATE_PLATFORM_WAIT_GRASP,
                            MISSION_OPERATION_TIMEOUT_MS);
    }
}

static void mission_vision_process(mission_context_t *ctx)
{
    uint8_t grasp_group;

    if ((ctx->vision.phase == MISSION_VISION_IDLE) ||
        ctx->vision.inflight) {
        return;
    }
    if (ctx->vision.phase == MISSION_VISION_LISTENING) {
        if (mission_submit_vision_transfer(
                ctx, MULT_UART_OP_READ, 0U,
                MISSION_VISION_READ_TIMEOUT_MS) != NANO_VISION_OK) {
            return;
        }
    } else if (ctx->vision.phase == MISSION_VISION_ACKING) {
        mission_reset_vision(ctx);
        if (ctx->state == MISSION_STATE_PLATFORM_WAIT_GRASP) {
            if (!mission_start_arm(ctx, MISSION_PLATFORM_GRASP_GROUP,
                                   MISSION_STATE_PLATFORM_WAIT_GRASP)) {
                mission_fail(ctx, MISSION_FAULT_ARM);
            }
        } else if (ctx->state == MISSION_STATE_STAIR_WAIT_ACK) {
            grasp_group = mission_stair_grasp_group(ctx->stair_layer);
            if ((grasp_group == 0U) ||
                !mission_start_arm(ctx, grasp_group,
                                   MISSION_STATE_STAIR_WAIT_GRASP)) {
                mission_fail(ctx, (grasp_group == 0U) ?
                    MISSION_FAULT_PROTOCOL : MISSION_FAULT_ARM);
            }
        }
    }
}

/** 动作组10和底盘初始化均完成后，回复握手并进入READY。 */
static void mission_try_ready(mission_context_t *ctx)
{
    if ((ctx->state != MISSION_STATE_WAIT_CHASSIS_READY) ||
        !ctx->chassis_ready) {
        return;
    }
    if (!mission_send_chassis(MISSION_CMD_MISSION_READY, ctx->request_id)) {
        mission_fail(ctx, MISSION_FAULT_QUEUE);
        return;
    }
    mission_enter_state(ctx, MISSION_STATE_READY, 0U);
}

/** 清空本轮计数并请求底盘执行起点到圆盘路线。 */
static void mission_start_run(mission_context_t *ctx, mission_color_t color)
{
    uint16_t request_id = mission_next_request_id(ctx);

    ctx->color = color;
    ctx->platform_balls = 0U;
    ctx->stair_balls = 0U;
    ctx->storage_slot = 0U;
    ctx->fault_code = MISSION_FAULT_NONE;
    if (!mission_send_chassis(MISSION_CMD_GO_PLATFORM, request_id)) {
        mission_fail(ctx, MISSION_FAULT_QUEUE);
        return;
    }
    mission_enter_state(
        ctx,
        MISSION_STATE_WAIT_PLATFORM,
        MISSION_OPERATION_TIMEOUT_MS);
}

/** 当前层起点就绪后开启对应视觉；抓满2球则只放行底盘走完整段。 */
static void mission_start_stair_layer(mission_context_t *ctx)
{
    if (ctx->stair_layer == MISSION_STAIR_NONE) {
        mission_enter_state(ctx, MISSION_STATE_STAIR_WAIT_LAYER,
                            MISSION_OPERATION_TIMEOUT_MS);
        return;
    }
    if (ctx->stair_balls >= MISSION_STAIR_BALL_COUNT) {
        if (!mission_send_chassis(MISSION_CMD_CAM_READY, ctx->request_id)) {
            mission_fail(ctx, MISSION_FAULT_QUEUE);
            return;
        }
        mission_enter_state(ctx, MISSION_STATE_STAIR_SCANNING,
                            MISSION_OPERATION_TIMEOUT_MS);
        return;
    }
    if (!mission_start_vision(
            ctx,
            MISSION_VISION_SCENE_STAIR,
            ctx->stair_layer,
            MISSION_STATE_STAIR_WAIT_VISION_START)) {
        mission_fail(ctx, MISSION_FAULT_VISION);
    }
}

/** 把当前阶梯层映射到已经标定的抓取动作组。 */
static uint8_t mission_stair_grasp_group(mission_stair_layer_t layer)
{
    if (layer == MISSION_STAIR_LOW) {
        return MISSION_STAIR_LOW_GROUP;
    }
    if (layer == MISSION_STAIR_HIGH) {
        return MISSION_STAIR_HIGH_GROUP;
    }
    if (layer == MISSION_STAIR_MID) {
        return MISSION_STAIR_MID_GROUP;
    }
    return 0U;
}

/** 处理用户命令；非READY启动命令被忽略，STOP在运行阶段始终有效。 */
static void mission_handle_command(
    mission_context_t *ctx,
    mission_user_command_t command)
{
    uint16_t request_id;

    if (command == MISSION_USER_COMMAND_STOP) {
        if ((ctx->state == MISSION_STATE_STOPPED) ||
            (ctx->state == MISSION_STATE_BOOT)) {
            return;
        }
        request_id = mission_next_request_id(ctx);
        if (!mission_send_chassis(MISSION_CMD_STOP, request_id)) {
            mission_fail(ctx, MISSION_FAULT_QUEUE);
            return;
        }
        mission_enter_state(
            ctx,
            MISSION_STATE_STOPPING,
            MISSION_OPERATION_TIMEOUT_MS);
        return;
    }
    if (ctx->state != MISSION_STATE_READY) {
        return;
    }
    if (command == MISSION_USER_COMMAND_START_RED) {
        mission_start_run(ctx, MISSION_COLOR_RED);
    } else if (command == MISSION_USER_COMMAND_START_BLUE) {
        mission_start_run(ctx, MISSION_COLOR_BLUE);
    }
}

/** 记录底盘层级通知，并只在状态、请求编号和ready值均匹配时推进。 */
static void mission_handle_chassis(
    mission_context_t *ctx,
    const chassis_mission_event_t *event)
{
    mission_stair_layer_t layer;
    uint8_t grasp_group;

    if (event->type == CHASSIS_CMD_MISSION_READY) {
        if ((ctx->state != MISSION_STATE_WAIT_HOME) &&
            (ctx->state != MISSION_STATE_WAIT_CHASSIS_READY)) {
            return;
        }
        if (event->is_ready == 0U) {
            mission_fail(ctx, MISSION_FAULT_CHASSIS);
            return;
        }
        ctx->chassis_ready = true;
        ctx->request_id = event->request_id;
        mission_try_ready(ctx);
        return;
    }
    if ((event->request_id != ctx->request_id) ||
        (event->is_ready == 0U)) {
        if ((event->request_id == ctx->request_id) &&
            (event->is_ready == 0U)) {
            mission_fail(ctx, MISSION_FAULT_CHASSIS);
        }
        return;
    }
    if ((ctx->state == MISSION_STATE_STOPPING) &&
        (event->type == CHASSIS_CMD_STOPPED)) {
        mission_enter_state(ctx, MISSION_STATE_STOPPED, 0U);
        return;
    }
    if ((ctx->state == MISSION_STATE_WAIT_PLATFORM) &&
        (event->type == CHASSIS_CMD_PLATFORM_READY)) {
        if (!mission_start_arm(
                ctx,
                MISSION_PLATFORM_VISION_GROUP,
                MISSION_STATE_PLATFORM_WAIT_POSE)) {
            mission_fail(ctx, MISSION_FAULT_ARM);
        }
        return;
    }
    if ((ctx->state == MISSION_STATE_WAIT_STAIRS) &&
        (event->type == CHASSIS_CMD_STAIRS_READY)) {
        ctx->stair_layer = MISSION_STAIR_NONE;
        if (!mission_start_arm(
                ctx,
                MISSION_STAIR_VISION_GROUP,
                MISSION_STATE_STAIR_WAIT_POSE)) {
            mission_fail(ctx, MISSION_FAULT_ARM);
        }
        return;
    }
    if ((event->type == CHASSIS_CMD_STAIR_LOW) ||
        (event->type == CHASSIS_CMD_STAIR_HIGH) ||
        (event->type == CHASSIS_CMD_STAIR_MID)) {
        if (event->type == CHASSIS_CMD_STAIR_LOW) {
            layer = MISSION_STAIR_LOW;
        } else if (event->type == CHASSIS_CMD_STAIR_HIGH) {
            layer = MISSION_STAIR_HIGH;
        } else {
            layer = MISSION_STAIR_MID;
        }
        if ((ctx->state != MISSION_STATE_STAIR_WAIT_POSE) &&
            (ctx->state != MISSION_STATE_STAIR_WAIT_LAYER) &&
            (ctx->state != MISSION_STATE_STAIR_SCANNING)) {
            return;
        }
        ctx->stair_layer = layer;
        if ((ctx->state == MISSION_STATE_STAIR_SCANNING) &&
            (ctx->vision.phase != MISSION_VISION_IDLE)) {
            mission_enter_state(ctx, MISSION_STATE_STAIR_WAIT_LAYER,
                                MISSION_OPERATION_TIMEOUT_MS);
            if (!mission_stop_vision(ctx)) {
                mission_fail(ctx, MISSION_FAULT_VISION);
            }
        } else if (ctx->state != MISSION_STATE_STAIR_WAIT_POSE) {
            mission_start_stair_layer(ctx);
        }
        return;
    }
    if ((event->type == CHASSIS_CMD_STAIR_PAUSE) &&
        (ctx->state == MISSION_STATE_STAIR_WAIT_PAUSE)) {
        if (ctx->vision.phase == MISSION_VISION_ACKING) {
            mission_enter_state(ctx, MISSION_STATE_STAIR_WAIT_ACK,
                                MISSION_OPERATION_TIMEOUT_MS);
            return;
        }
        grasp_group = mission_stair_grasp_group(ctx->stair_layer);
        if (grasp_group == 0U) {
            mission_fail(ctx, MISSION_FAULT_PROTOCOL);
            return;
        }
        if (!mission_start_arm(
                ctx,
                grasp_group,
                MISSION_STATE_STAIR_WAIT_GRASP)) {
            mission_fail(ctx, MISSION_FAULT_ARM);
        }
        return;
    }
    if ((event->type == CHASSIS_CMD_STAIR_RESUME) &&
        (ctx->state == MISSION_STATE_STAIR_WAIT_RESUME)) {
        mission_enter_state(ctx, MISSION_STATE_STAIR_SCANNING,
                            MISSION_OPERATION_TIMEOUT_MS);
        return;
    }
    if (event->type == CHASSIS_CMD_STAIRS_FINISHED) {
        if ((ctx->vision.phase != MISSION_VISION_IDLE) &&
            !mission_stop_vision(ctx)) {
            mission_fail(ctx, MISSION_FAULT_VISION);
            return;
        }
        mission_enter_state(ctx, MISSION_STATE_COMPLETE, 0U);
    }
}

/** 处理唯一在途动作组结果，并按圆盘或阶梯子流程继续。 */
static void mission_handle_arm(mission_context_t *ctx, bool success)
{
    if ((ctx->state != MISSION_STATE_WAIT_HOME) &&
        (ctx->state != MISSION_STATE_PLATFORM_WAIT_POSE) &&
        (ctx->state != MISSION_STATE_PLATFORM_WAIT_GRASP) &&
        (ctx->state != MISSION_STATE_PLATFORM_WAIT_RELEASE) &&
        (ctx->state != MISSION_STATE_PLATFORM_WAIT_RETURN) &&
        (ctx->state != MISSION_STATE_PLATFORM_WAIT_DEPARTURE_POSE) &&
        (ctx->state != MISSION_STATE_STAIR_WAIT_POSE) &&
        (ctx->state != MISSION_STATE_STAIR_WAIT_GRASP) &&
        (ctx->state != MISSION_STATE_STAIR_WAIT_RETURN)) {
        return;
    }
    if (!success) {
        mission_fail(ctx, MISSION_FAULT_ARM);
        return;
    }
    if (ctx->state == MISSION_STATE_WAIT_HOME) {
        if (!mission_prepare_zdt(ctx)) {
            mission_fail(ctx, MISSION_FAULT_STORAGE);
            return;
        }
        mission_enter_state(ctx, MISSION_STATE_WAIT_CHASSIS_READY,
                            MISSION_READY_TIMEOUT_MS);
        mission_try_ready(ctx);
        return;
    }
    if (ctx->state == MISSION_STATE_PLATFORM_WAIT_POSE) {
        if (!mission_start_vision(
                ctx,
                MISSION_VISION_SCENE_PLATFORM,
                MISSION_STAIR_NONE,
                MISSION_STATE_PLATFORM_WAIT_VISION)) {
            mission_fail(ctx, MISSION_FAULT_VISION);
        }
        return;
    }
    if (ctx->state == MISSION_STATE_PLATFORM_WAIT_GRASP) {
        if ((uint8_t)(ctx->platform_balls + 1U) >=
            MISSION_PLATFORM_BALL_COUNT) {
            if (!mission_start_arm(
                    ctx,
                    MISSION_PLATFORM_RELEASE_GROUP,
                    MISSION_STATE_PLATFORM_WAIT_RELEASE)) {
                mission_fail(ctx, MISSION_FAULT_ARM);
            }
            return;
        }
        mission_enter_state(ctx, MISSION_STATE_PLATFORM_WAIT_STORAGE,
                            MISSION_OPERATION_TIMEOUT_MS);
        if (!mission_store_ball(ctx, MISSION_STORAGE_REGION_PLATFORM)) {
            mission_fail(ctx, MISSION_FAULT_STORAGE);
        } else {
            mission_handle_storage(ctx);
        }
        return;
    }
    if (ctx->state == MISSION_STATE_PLATFORM_WAIT_RELEASE) {
        mission_enter_state(ctx, MISSION_STATE_PLATFORM_WAIT_STORAGE,
                            MISSION_OPERATION_TIMEOUT_MS);
        if (!mission_store_ball(ctx, MISSION_STORAGE_REGION_PLATFORM)) {
            mission_fail(ctx, MISSION_FAULT_STORAGE);
        } else {
            mission_handle_storage(ctx);
        }
        return;
    }
    if (ctx->state == MISSION_STATE_PLATFORM_WAIT_RETURN) {
        if (!mission_start_vision(
                ctx,
                MISSION_VISION_SCENE_PLATFORM,
                MISSION_STAIR_NONE,
                MISSION_STATE_PLATFORM_WAIT_VISION)) {
            mission_fail(ctx, MISSION_FAULT_VISION);
        }
        return;
    }
    if (ctx->state == MISSION_STATE_PLATFORM_WAIT_DEPARTURE_POSE) {
        (void)mission_next_request_id(ctx);
        if (!mission_send_chassis(MISSION_CMD_GO_STAIRS, ctx->request_id)) {
            mission_fail(ctx, MISSION_FAULT_QUEUE);
            return;
        }
        mission_enter_state(ctx, MISSION_STATE_WAIT_STAIRS,
                            MISSION_OPERATION_TIMEOUT_MS);
        return;
    }
    if (ctx->state == MISSION_STATE_STAIR_WAIT_POSE) {
        mission_start_stair_layer(ctx);
        return;
    }
    if (ctx->state == MISSION_STATE_STAIR_WAIT_GRASP) {
        if (!mission_start_arm(
                ctx,
                MISSION_STAIR_VISION_GROUP,
                MISSION_STATE_STAIR_WAIT_RETURN)) {
            mission_fail(ctx, MISSION_FAULT_ARM);
        }
        return;
    }
    if (ctx->state == MISSION_STATE_STAIR_WAIT_RETURN) {
        mission_enter_state(ctx, MISSION_STATE_STAIR_WAIT_STORAGE,
                            MISSION_OPERATION_TIMEOUT_MS);
        if (!mission_store_ball(ctx, MISSION_STORAGE_REGION_STAIR)) {
            mission_fail(ctx, MISSION_FAULT_STORAGE);
        } else {
            mission_handle_storage(ctx);
        }
    }
}

/** 完成一次存球后更新计数，继续当前区域或进入下一底盘阶段。 */
static void mission_handle_storage(mission_context_t *ctx)
{
    mission_state_t completed_state = ctx->state;

    if ((completed_state != MISSION_STATE_PLATFORM_WAIT_STORAGE) &&
        (completed_state != MISSION_STATE_STAIR_WAIT_STORAGE)) {
        return;
    }
    ++ctx->storage_slot;
    if (completed_state == MISSION_STATE_PLATFORM_WAIT_STORAGE) {
        ++ctx->platform_balls;
        if (ctx->platform_balls >= MISSION_PLATFORM_BALL_COUNT) {
            if (!mission_start_arm(
                    ctx,
                    MISSION_HOME_ACTION_GROUP,
                    MISSION_STATE_PLATFORM_WAIT_DEPARTURE_POSE)) {
                mission_fail(ctx, MISSION_FAULT_ARM);
            }
        } else if (!mission_start_arm(
                       ctx,
                       MISSION_PLATFORM_VISION_GROUP,
                       MISSION_STATE_PLATFORM_WAIT_RETURN)) {
            mission_fail(ctx, MISSION_FAULT_ARM);
        }
        return;
    }
    if (completed_state == MISSION_STATE_STAIR_WAIT_STORAGE) {
        ++ctx->stair_balls;
        if (ctx->stair_balls >= MISSION_STAIR_BALL_COUNT) {
            if (!mission_send_chassis(
                    MISSION_CMD_STAIR_RESUME, ctx->request_id)) {
                mission_fail(ctx, MISSION_FAULT_QUEUE);
                return;
            }
            mission_enter_state(ctx, MISSION_STATE_STAIR_WAIT_RESUME,
                                MISSION_OPERATION_TIMEOUT_MS);
        } else if (!mission_start_vision(
                       ctx,
                       MISSION_VISION_SCENE_STAIR,
                       ctx->stair_layer,
                       MISSION_STATE_STAIR_WAIT_VISION_RESUME)) {
            mission_fail(ctx, MISSION_FAULT_VISION);
        } else {
            /* 视觉会话就绪后由mission_handle_vision发送STAIR_RESUME。 */
        }
    }
}

/** 对所有有期限的等待状态实施统一超时停机。 */
static void mission_check_timeout(mission_context_t *ctx)
{
    if ((ctx->deadline_tick != 0U) &&
        ((int32_t)(osKernelGetTickCount() - ctx->deadline_tick) >= 0)) {
        mission_fail(ctx, MISSION_FAULT_TIMEOUT);
    }
}

/**
 * Mission唯一任务：阻塞等待线程标志，醒来后依次处理命令、底盘和设备结果。
 * 底盘先入队再置位；本任务被唤醒后一次性排空底盘事件队列。
 */
static void mission_task_entry(void *argument)
{
    mission_context_t *ctx = (mission_context_t *)argument;
    chassis_mission_event_t chassis_event;
    mission_user_command_t command;
    uint32_t flags;

    mission_enter_state(ctx, MISSION_STATE_WAIT_HOME, 0U);
    for (;;) {
        flags = osThreadFlagsWait(
            MISSION_ALL_FLAGS,
            osFlagsWaitAny,
            mission_wait_ticks(ctx));

        if ((flags & osFlagsError) == 0U) {
            if ((flags & MISSION_FLAG_COMMAND) != 0U) {
                while (osMessageQueueGet(
                           ctx->command_queue,
                           &command,
                           NULL,
                           0U) == osOK) {
                    mission_handle_command(ctx, command);
                }
            }
            if ((flags & CHASSIS_MISSION_FLAG_EVENT) != 0U) {
                while (osMessageQueueGet(
                           mission_event_queue,
                           &chassis_event,
                           NULL,
                           0U) == osOK) {
                    mission_handle_chassis(ctx, &chassis_event);
                }
            }
            if ((flags & MISSION_FLAG_ARM_FAIL) != 0U) {
                mission_handle_arm(ctx, false);
            } else if ((flags & MISSION_FLAG_ARM_OK) != 0U) {
                mission_handle_arm(ctx, true);
            }
            if ((flags & MISSION_FLAG_VISION_DONE) != 0U) {
                mission_handle_vision(ctx);
            }
        }
        mission_vision_process(ctx);
        mission_check_timeout(ctx);
    }
}

/**
 * @brief 最小联调任务：动作组11完成后读卡一次，再让转盘完整前进一格。
 * @param argument 指向本文件唯一Mission上下文。
 * @note 读卡只提交一次；无论读卡是否成功都继续测试转盘，结果保存在storage中。
 */
static void ap_test_task(void *argument)
{
    mission_context_t *ctx = (mission_context_t *)argument;
    uint32_t flags;

    /* 动作组完成必须以舵控板主动回报为准，UART发送完成不代表机械动作完成。 */
    ctx->active_arm_group = MISSION_PLATFORM_VISION_GROUP;
    if (arm_run(
            MISSION_PLATFORM_VISION_GROUP,
            1U,
            mission_arm_tx_done,
            ctx) != LSC16_OK) {
        goto failed;
    }
    flags = osThreadFlagsWait(
        MISSION_FLAG_ARM_OK | MISSION_FLAG_ARM_FAIL,
        osFlagsWaitAny,
        mission_ms_to_ticks(MISSION_OPERATION_TIMEOUT_MS));
    if (((flags & osFlagsError) != 0U) ||
        ((flags & MISSION_FLAG_ARM_OK) == 0U)) {
        goto failed;
    }

    /* 读卡回调按值保存球号、行列和状态；本测试不重试，也不写入球档案。 */
    (void)osThreadFlagsClear(MISSION_FLAG_IC_DONE);
    ctx->storage.ic_status = IC_CARD_ERR_BUSY;
    if (ic_read(
            MISSION_IC_OPERATION_PROMPT != 0U,
            mission_ic_done,
            ctx) == IC_CARD_OK) {
        (void)mission_wait_device(
            MISSION_FLAG_IC_DONE,
            IC_READ_TIMEOUT_MS + 100U);
    }

    /* Emm运动前先查询并缓存固件、闭环和Scale配置。 */
    if (!mission_prepare_zdt(ctx)) {
        goto failed;
    }

    /* 沿用正式Mission的粗转、到位轮询、PB0确认和逐步微调闭环。 */
    if (!mission_advance_slot(ctx)) {
        goto failed;
    }

    ctx->storage_slot = 1U;
    ctx->state = MISSION_STATE_COMPLETE;
    for (;;) {
        (void)osDelay(1000U);
    }

failed:
    ctx->fault_code = (uint8_t)MISSION_FAULT_STORAGE;
    ctx->state = MISSION_STATE_FAULT;
    /* 联调失败后只请求一次转盘停止，不创建恢复或重试流程。 */
    (void)turn_stop(NULL, NULL);
    for (;;) {
        (void)osDelay(1000U);
    }
}

/** 初始化IC和转盘语义层，注册机械臂回报并创建唯一联调任务。 */
mission_app_status_t ap_test_init(void)
{
    mission_context_t *ctx = &g_mission;
    turn_config_t turn_config = {
        MISSION_ZDT_ADDRESS,
        MISSION_ZDT_IO_TIMEOUT_MS,
        MISSION_ZDT_EMM_PULSES_PER_REV,
    };

    if (ctx->initialized) {
        return MISSION_APP_ERR_STATE;
    }
    (void)memset(ctx, 0, sizeof(*ctx));

    /* mux_init()和arm_init()已由MX_FREERTOS_Init()先行完成。 */
    if (ic_init() != IC_CARD_OK) {
        return MISSION_APP_ERR_IO;
    }
    if (turn_init(&turn_config) != ZDT_TURNTABLE_OK) {
        return MISSION_APP_ERR_IO;
    }
    if (arm_on_report(mission_arm_report, ctx) != LSC16_OK) {
        return MISSION_APP_ERR_IO;
    }

    ctx->state = MISSION_STATE_BOOT;
    ctx->initialized = true;
    ctx->task = osThreadNew(ap_test_task, ctx, &g_mission_task_attr);
    if (ctx->task == NULL) {
        ctx->initialized = false;
        return MISSION_APP_ERR_RESOURCE;
    }
    return MISSION_APP_OK;
}

mission_app_status_t mission_app_init(void)
{
    mission_context_t *ctx = &g_mission;

    if (ctx->initialized) {
        return MISSION_APP_ERR_STATE;
    }
    (void)memset(ctx, 0, sizeof(*ctx));
    ball_manifest_init(&ctx->manifest);
    if (ic_init() != IC_CARD_OK) {
        return MISSION_APP_ERR_IO;
    }
    {
        turn_config_t config = {
            MISSION_ZDT_ADDRESS,
            MISSION_ZDT_IO_TIMEOUT_MS,
            MISSION_ZDT_EMM_PULSES_PER_REV,
        };
        if (turn_init(&config) != ZDT_TURNTABLE_OK) {
            return MISSION_APP_ERR_IO;
        }
    }
    ctx->command_queue = osMessageQueueNew(
        MISSION_COMMAND_QUEUE_DEPTH,
        sizeof(mission_user_command_t),
        NULL);
    if (ctx->command_queue == NULL) {
        return MISSION_APP_ERR_RESOURCE;
    }
    ctx->task = osThreadNew(mission_task_entry, ctx, &g_mission_task_attr);
    if (ctx->task == NULL) {
        return MISSION_APP_ERR_RESOURCE;
    }
    if (!chassis_mission_link_bind_mission_task(ctx->task)) {
        return MISSION_APP_ERR_RESOURCE;
    }
    if (arm_on_report(mission_arm_report, ctx) != LSC16_OK) {
        return MISSION_APP_ERR_IO;
    }
    ctx->initialized = true;
    ctx->active_arm_group = MISSION_HOME_ACTION_GROUP;
    if (arm_run(
            MISSION_HOME_ACTION_GROUP,
            1U,
            mission_arm_tx_done,
            ctx) != LSC16_OK) {
        return MISSION_APP_ERR_IO;
    }
    return MISSION_APP_OK;
}

mission_app_status_t mission_app_submit_command(mission_user_command_t command)
{
    mission_context_t *ctx = &g_mission;
    uint32_t flags;

    if ((command <= MISSION_USER_COMMAND_NONE) ||
        (command > MISSION_USER_COMMAND_STOP)) {
        return MISSION_APP_ERR_PARAM;
    }
    if (!ctx->initialized) {
        return MISSION_APP_ERR_STATE;
    }
    if (osMessageQueuePut(ctx->command_queue, &command, 0U, 0U) != osOK) {
        return MISSION_APP_ERR_BUSY;
    }
    flags = osThreadFlagsSet(ctx->task, MISSION_FLAG_COMMAND);
    return ((flags & osFlagsError) == 0U)
        ? MISSION_APP_OK
        : MISSION_APP_ERR_IO;
}

mission_app_status_t mission_app_get_snapshot(mission_app_snapshot_t *snapshot)
{
    mission_context_t *ctx = &g_mission;

    if (snapshot == NULL) {
        return MISSION_APP_ERR_PARAM;
    }
    if (!ctx->initialized) {
        return MISSION_APP_ERR_STATE;
    }
    taskENTER_CRITICAL();
    snapshot->state = ctx->state;
    snapshot->color = ctx->color;
    snapshot->stair_layer = ctx->stair_layer;
    snapshot->chassis_request_id = ctx->request_id;
    snapshot->platform_balls = ctx->platform_balls;
    snapshot->stair_balls = ctx->stair_balls;
    snapshot->storage_slot = ctx->storage_slot;
    snapshot->fault_code = ctx->fault_code;
    taskEXIT_CRITICAL();
    return MISSION_APP_OK;
}
