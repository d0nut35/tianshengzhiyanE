/**
 * @file    mission_app.c
 * @brief   整机比赛任务初版状态机实现。
 *
 * 本文件是Mission状态的唯一写入者。底盘消息由公共队列保存，设备完成
 * 回调只记录结果并设置线程标志；所有状态转换都在mission_task_entry中串行执行。
 */

#include "mission_app.h"

#include <stdio.h>
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

#if MISSION_CHASSIS_ROUTE_TEST_ENABLED
#include "debug_uart1.h"
#endif

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
    MISSION_VISION_SCENE_SMALL_DISC,
} mission_vision_scene_t;

typedef enum {
    MISSION_STORAGE_REGION_PLATFORM = 1,
    MISSION_STORAGE_REGION_STAIR,
    MISSION_STORAGE_REGION_SMALL_DISC,
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

#if MISSION_CHASSIS_ROUTE_TEST_ENABLED
/** 无线联调首条运动指令确定本轮采用纯路径或单目标模式。 */
typedef enum {
    MISSION_TEST_MODE_IDLE = 0,
    MISSION_TEST_MODE_PATH,
    MISSION_TEST_MODE_TARGET,
} mission_test_mode_t;

/** 测试阶段既用于目标选择，也用于限制纯路径指令顺序。 */
typedef enum {
    MISSION_TEST_STAGE_NONE = 0,
    MISSION_TEST_STAGE_PLATFORM,
    MISSION_TEST_STAGE_STAIRS,
    MISSION_TEST_STAGE_SMALL_DISC,
    MISSION_TEST_STAGE_DEPOT,
    MISSION_TEST_STAGE_HOME,
    MISSION_TEST_STAGE_DONE,
} mission_test_stage_t;
#endif

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
    mission_stair_layer_t stair_layer;
    uint16_t request_id;
    uint32_t deadline_tick;

    /* 比赛流程数据由mission_task_entry唯一写入。 */
    uint8_t platform_balls;
    uint8_t stair_balls;
    uint8_t small_disc_balls;
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
#if MISSION_CHASSIS_ROUTE_TEST_ENABLED
    uint8_t arm_boot_state;                 /* 0=未尝试，1=注册失败，2=下发失败，3=已入队。 */
    volatile uint32_t arm_last_action_report; /* 高位为事件，低8位为动作组。 */
#endif
} mission_context_t;

static mission_context_t g_mission;

/* 当前比赛红蓝方由Mission唯一维护，其他模块只读并据此选择地图。 */
volatile mission_color_t g_mission_side = MISSION_COLOR_NONE;

#if MISSION_CHASSIS_ROUTE_TEST_ENABLED
/** USART1无线联调任务的运行状态和文本缓冲区。 */
typedef struct {
    debug_uart1_t debug;                   /* 现有DMA空闲接收封装。 */
    mission_test_mode_t mode;              /* 本轮测试方式。 */
    mission_test_stage_t target;           /* 唯一启用抓取的区域。 */
    mission_test_stage_t expected;         /* 下一个未完成的纯路径阶段。 */
    uint8_t depot_position;                /* 0=未进仓库，1~4=当前位置。 */
    uint8_t reported_ball_count;           /* 已自动打印的球记录数。 */
    bool arm_ready;                         /* 动作组10完成回报已被测试任务确认。 */
    bool stop_requested;                   /* STOP已下发，等待停车回执。 */
    char text[224];                        /* 单条可读回复缓冲区。 */
} mission_wireless_test_t;

static mission_wireless_test_t g_wireless_test;
#endif

static const osThreadAttr_t g_mission_task_attr = {
    .name = "mission_app",
    .stack_size = MISSION_TASK_STACK_SIZE,
    .priority = osPriorityNormal,
};

/** 将毫秒换算为CMSIS-RTOS tick。 */
static uint32_t mission_ms_to_ticks(uint32_t ms);
/** 计算Mission任务等待当前状态超时所需的tick。 */
static uint32_t mission_wait_ticks(const mission_context_t *ctx);
/** Mission主任务入口，串行处理命令、底盘和设备事件。 */
static void mission_task_entry(void *argument);
/** 机械臂可选、底盘就绪后接收USART1指令的独立联调任务。 */
static void mission_wireless_test_entry(void *argument);
/** 机械臂命令发送完成回调；发送失败时唤醒Mission。 */
static void mission_arm_tx_done(
    void *user_ctx,
    uint32_t request_id,
    lsc16_status_t status);
/** 机械臂动作组完成回调；把成功或失败结果通知Mission。 */
static void mission_arm_report(
    void *user_ctx,
    uint32_t report_events,
    const lsc16_report_t *report);
/** Nano视觉复用串口事务完成回调。 */
static void mission_vision_done(
    void *user_ctx,
    const mux_completion_t *completion);
/** IC卡读取完成回调。 */
static void mission_ic_done(
    void *user_ctx,
    uint32_t request_id,
    ic_card_status_t status,
    const ic_result_t *result);
/** 转盘电机事务完成回调。 */
static void mission_zdt_done(
    void *user_ctx,
    uint32_t request_id,
    zdt_turntable_status_t status,
    const zdt_turntable_response_t *response);
/** 切换Mission状态并设置该状态的超时时间。 */
static void mission_enter_state(
    mission_context_t *ctx,
    mission_state_t state,
    uint32_t timeout_ms);
/** 记录故障、停止视觉和底盘，并进入故障状态。 */
static void mission_fail(mission_context_t *ctx, mission_fault_t fault);
/** 生成下一条非零底盘请求编号。 */
static uint16_t mission_next_request_id(mission_context_t *ctx);
/** 将一条带request_id的命令发到底盘队列。 */
static bool mission_send_chassis(
    mission_command_type_t type,
    uint16_t request_id);
/** 启动机械臂动作组并进入指定等待状态。 */
static bool mission_start_arm(
    mission_context_t *ctx,
    uint8_t action_group,
    mission_state_t wait_state);
/** 启动指定场景的Nano视觉会话并进入指定等待状态。 */
static bool mission_start_vision(
    mission_context_t *ctx,
    mission_vision_scene_t scene,
    mission_stair_layer_t layer,
    mission_state_t wait_state);
/** 请求Nano停止当前视觉会话。 */
static bool mission_stop_vision(mission_context_t *ctx);
/** 清空当前视觉会话的本地运行状态。 */
static void mission_reset_vision(mission_context_t *ctx);
/** 读取小球IC信息并将车载转盘推进一格。 */
static bool mission_store_ball(
    mission_context_t *ctx,
    mission_storage_region_t region);
/** 解析一次Nano事务结果并推进视觉状态。 */
static void mission_handle_vision(mission_context_t *ctx);
/** 视觉事务空闲后继续接收数据或启动抓取动作。 */
static void mission_vision_process(mission_context_t *ctx);
/** 双方就绪后完成握手并进入READY状态。 */
static void mission_try_ready(mission_context_t *ctx);
/** 按指定红蓝方启动一轮正式任务。 */
static void mission_start_run(mission_context_t *ctx, mission_color_t color);
/** 启动当前阶梯层视觉；抓满后只放行底盘。 */
static void mission_start_stair_layer(mission_context_t *ctx);
/** 将低、高、中层映射到动作组14、15、16。 */
static uint8_t mission_stair_grasp_group(mission_stair_layer_t layer);
/** 阶梯视觉结束后执行动作组18，进入小圆盘前的机械臂过渡。 */
static void mission_start_stair_exit(mission_context_t *ctx);
/** 小圆盘视觉结束后执行动作组21，再回动作组10进入仓库。 */
static void mission_start_small_disc_exit(mission_context_t *ctx);
/** 处理用户START和STOP命令。 */
static void mission_handle_command(
    mission_context_t *ctx,
    mission_user_command_t command);
/** 处理底盘上报事件并推进Mission状态。 */
static void mission_handle_chassis(
    mission_context_t *ctx,
    const chassis_mission_event_t *event);
/** 处理当前机械臂动作组的完成结果。 */
static void mission_handle_arm(mission_context_t *ctx, bool success);
/** 完成一次存球后的计数和后续流程。 */
static void mission_handle_storage(mission_context_t *ctx);
/** 检查当前状态是否到期并触发自动启动或故障。 */
static void mission_check_timeout(mission_context_t *ctx);

#if MISSION_CHASSIS_ROUTE_TEST_ENABLED
/** 输出无线测试文本。 */
static void mission_test_write(const char *text);
/** 输出当前测试状态。 */
static void mission_test_print_status(const mission_context_t *ctx);
/** 输出一条小球档案。 */
static void mission_test_print_ball(
    const mission_context_t *ctx,
    uint8_t sequence);
/** 输出全部小球档案。 */
static void mission_test_print_balls(const mission_context_t *ctx);
/** 打印本轮新追加的小球档案。 */
static void mission_test_print_new_balls(const mission_context_t *ctx);
/** 读取并规范化一条USART1命令。 */
static bool mission_test_take_command(char *command, size_t capacity);
/** 处理运动期间仍允许执行的查询和停车指令。 */
static bool mission_test_handle_aux_command(
    mission_context_t *ctx,
    const char *command);
/** 在2秒阶段停留期间继续响应查询和停车。 */
static bool mission_test_pause(mission_context_t *ctx);
/** 在可响应STOP的前提下等待一条底盘事件。 */
static bool mission_test_wait_chassis_event(
    mission_context_t *ctx,
    chassis_command_type_t expected,
    uint16_t request_id);
/** 发送底盘命令并等待对应事件。 */
static bool mission_test_send_wait(
    mission_context_t *ctx,
    mission_command_type_t command,
    chassis_command_type_t expected,
    mission_state_t wait_state);
/** 跳过转盘业务，仅验证导航并停留2秒。 */
static bool mission_test_skip_platform(mission_context_t *ctx);
/** 跳过阶梯视觉抓取，放行并走完整个阶梯后停留2秒。 */
static bool mission_test_skip_stairs(mission_context_t *ctx);
/** 跳过小圆盘视觉抓取，完整绕行后停留2秒。 */
static bool mission_test_skip_small_disc(mission_context_t *ctx);
/** 运行指定目标区域的正式视觉抓取子流程。 */
static bool mission_test_run_target(
    mission_context_t *ctx,
    mission_test_stage_t target);
#endif

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

#if MISSION_CHASSIS_ROUTE_TEST_ENABLED
    /* 诊断保留原始组号，便于发现完成回报未匹配当前动作组。 */
    if ((ctx != NULL) && (report != NULL) &&
        ((report_events & (LSC16_REPORT_EVENT_ACTION_STARTED |
                           LSC16_REPORT_EVENT_ACTION_STOPPED |
                           LSC16_REPORT_EVENT_ACTION_COMPLETED)) != 0U)) {
        ctx->arm_last_action_report =
            ((report_events & 0xFFU) << 8) | report->action_group;
    }
#endif
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
    /* 0) 记录唯一顶层状态，后续事件只由该状态对应的分支处理。 */
    ctx->state = state;
    /* 1) 0表示永久等待；非0转换成绝对截止tick供主任务统一检查。 */
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
    } else if (scene == MISSION_VISION_SCENE_SMALL_DISC) {
        ctx->vision.scene = NANO_VISION_SCENE_SMALL_DISC;
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
    session.target_color = (g_mission_side == MISSION_COLOR_RED) ?
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

    if (storage_region == MISSION_STORAGE_REGION_PLATFORM) {
        region = BALL_MANIFEST_REGION_TURNTABLE;
    } else if (storage_region == MISSION_STORAGE_REGION_STAIR) {
        region = BALL_MANIFEST_REGION_STAIR;
    } else if (storage_region == MISSION_STORAGE_REGION_SMALL_DISC) {
        region = BALL_MANIFEST_REGION_PILLAR;
    } else {
        return false;
    }
    color = (g_mission_side == MISSION_COLOR_RED) ?
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

/**
 * @brief 解析Nano事务结果并推进当前视觉会话。
 * @param ctx Mission上下文。
 * @note 仅在Mission任务中调用；设备回调只负责保存数据并唤醒任务。
 */
static void mission_handle_vision(mission_context_t *ctx)
{
    nano_vision_status_t status;
    nano_vision_session_t session;
    nano_vision_event_t event;
    nano_vision_event_ack_t ack;
    size_t tx_len = 0U;

    /* 0) 先处理等待当前串口事务结束后才能执行的停止请求。 */
    status = mission_map_vision_status(ctx->vision.mail_status);
    if (ctx->vision.stop_requested &&
        (ctx->vision.phase != MISSION_VISION_STOPPING)) {
        ctx->vision.stop_requested = false;
        if (!mission_stop_vision(ctx)) {
            mission_fail(ctx, MISSION_FAULT_VISION);
        }
        return;
    }
    /* 1) 监听超时表示本轮没有新帧，保持会话并继续接收。 */
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
    /* 2) 停止回包必须属于当前会话，确认后才允许切换阶梯层。 */
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
        } else if (ctx->state == MISSION_STATE_STAIR_WAIT_VISION_END) {
            mission_start_stair_exit(ctx);
        } else if (ctx->state == MISSION_STATE_SMALL_DISC_WAIT_VISION_END) {
            mission_start_small_disc_exit(ctx);
        }
        return;
    }
    /* 3) READY必须同时匹配会话、场景和目标颜色。 */
    if (ctx->vision.phase == MISSION_VISION_STARTING) {
        status = nano_vision_decode_session_ready(
            ctx->vision.mail_data, ctx->vision.mail_len, &session);
        if ((status != NANO_VISION_OK) ||
            (session.session_id != ctx->vision.session_id) ||
            (session.scene != ctx->vision.scene) ||
            (session.target_color != ((g_mission_side == MISSION_COLOR_RED) ?
                NANO_VISION_COLOR_RED : NANO_VISION_COLOR_BLUE))) {
            mission_fail(ctx, MISSION_FAULT_VISION);
            return;
        }
        ctx->vision.phase = MISSION_VISION_LISTENING;
        /* 4) 圆盘开始监听；阶梯恢复等回报，首次进入本层则发CAM_READY。 */
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
        } else if (ctx->state ==
                   MISSION_STATE_SMALL_DISC_WAIT_VISION_START) {
            if (!mission_send_chassis(MISSION_CMD_SMALL_DISC_START,
                                      ctx->request_id)) {
                mission_fail(ctx, MISSION_FAULT_QUEUE);
                return;
            }
            mission_enter_state(ctx, MISSION_STATE_SMALL_DISC_RUNNING,
                                MISSION_OPERATION_TIMEOUT_MS);
        } else if (ctx->state ==
                   MISSION_STATE_SMALL_DISC_WAIT_VISION_RESUME) {
            if (!mission_send_chassis(MISSION_CMD_SMALL_DISC_RESUME,
                                      ctx->request_id)) {
                mission_fail(ctx, MISSION_FAULT_QUEUE);
                return;
            }
            mission_enter_state(ctx, MISSION_STATE_SMALL_DISC_WAIT_RESUME,
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
    /* 5) 只在底盘实际进入对应扫描状态后接收小球事件。 */
    if ((ctx->vision.phase != MISSION_VISION_LISTENING) ||
        ((ctx->state != MISSION_STATE_PLATFORM_WAIT_TARGET) &&
         (ctx->state != MISSION_STATE_STAIR_SCANNING) &&
         (ctx->state != MISSION_STATE_SMALL_DISC_RUNNING))) {
        return;
    }
    status = nano_vision_decode_event(
        ctx->vision.mail_data, ctx->vision.mail_len, &event);
    if ((status != NANO_VISION_OK) ||
        (event.session_id != ctx->vision.session_id) ||
        (event.observation.scene != ctx->vision.scene) ||
        (event.observation.status != NANO_VISION_OBS_VALID) ||
        (event.observation.color != ((g_mission_side == MISSION_COLOR_RED) ?
            NANO_VISION_COLOR_RED : NANO_VISION_COLOR_BLUE)) ||
        (event.observation.age_ms > MISSION_VISION_EVENT_MAX_AGE_MS)) {
        return;
    }
    /* 6) 先确认该视觉帧；运动中的阶梯和小圆盘还要请求底盘停车。 */
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
    } else if (ctx->state == MISSION_STATE_SMALL_DISC_RUNNING) {
        if (!mission_send_chassis(MISSION_CMD_SMALL_DISC_STOP,
                                  ctx->request_id)) {
            mission_fail(ctx, MISSION_FAULT_QUEUE);
            return;
        }
        mission_enter_state(ctx, MISSION_STATE_SMALL_DISC_WAIT_PAUSE,
                            MISSION_OPERATION_TIMEOUT_MS);
    } else if (ctx->state == MISSION_STATE_PLATFORM_WAIT_TARGET) {
        /* ACK发送完成后由mission_vision_process启动动作组12。 */
        mission_enter_state(ctx, MISSION_STATE_PLATFORM_WAIT_GRASP,
                            MISSION_OPERATION_TIMEOUT_MS);
    }
}

/**
 * @brief 在视觉事务空闲后继续接收事件或执行已确认的抓取动作。
 * @param ctx Mission上下文。
 */
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
        } else if (ctx->state == MISSION_STATE_SMALL_DISC_WAIT_ACK) {
            if (!mission_start_arm(ctx, MISSION_SMALL_DISC_GRASP_GROUP,
                                   MISSION_STATE_SMALL_DISC_WAIT_GRASP)) {
                mission_fail(ctx, MISSION_FAULT_ARM);
            }
        }
    }
}

/**
 * @brief 在机械臂和底盘均就绪后完成握手并进入READY。
 * @param ctx Mission上下文。
 * @note READY保持约4秒后自动按红方启动；底盘此时阻塞等待GO_PLATFORM。
 */
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
    mission_enter_state(ctx, MISSION_STATE_READY,
                        MISSION_AUTO_START_DELAY_MS);
}

/**
 * @brief 启动一轮正式任务并请求底盘前往圆盘工作位。
 * @param ctx Mission上下文。
 * @param color 本轮目标球颜色。
 * @note 本函数原本即为红蓝方命令共用的启动入口，本次未新增该函数。
 */
static void mission_start_run(mission_context_t *ctx, mission_color_t color)
{
    /* 0) 新一轮任务使用新的request_id，隔离上一轮底盘回包。 */
    uint16_t request_id = mission_next_request_id(ctx);

    /* 1) 保存红蓝方并清空本轮小球、槽位和故障计数。 */
    g_mission_side = color;
    ctx->platform_balls = 0U;
    ctx->stair_balls = 0U;
    ctx->small_disc_balls = 0U;
    ctx->storage_slot = 0U;
    ctx->fault_code = MISSION_FAULT_NONE;
    /* 2) 请求底盘去圆盘工作位。 */
    if (!mission_send_chassis(MISSION_CMD_GO_PLATFORM, request_id)) {
        mission_fail(ctx, MISSION_FAULT_QUEUE);
        return;
    }
    /* 3) 等待底盘带相同request_id上报PLATFORM_READY。 */
    mission_enter_state(
        ctx,
        MISSION_STATE_WAIT_PLATFORM,
        MISSION_OPERATION_TIMEOUT_MS);
}

/** 当前层起点就绪后开启对应视觉；抓满2球则只放行底盘走完整段。 */
static void mission_start_stair_layer(mission_context_t *ctx)
{
    /* 0) 动作组13可能先于层事件完成，此时只等待底盘上报层号。 */
    if (ctx->stair_layer == MISSION_STAIR_NONE) {
        mission_enter_state(ctx, MISSION_STATE_STAIR_WAIT_LAYER,
                            MISSION_OPERATION_TIMEOUT_MS);
        return;
    }
    /* 1) 已抓满2球后不再启动视觉，只允许底盘走完剩余层。 */
    if (ctx->stair_balls >= MISSION_STAIR_BALL_COUNT) {
        if (!mission_send_chassis(MISSION_CMD_CAM_READY, ctx->request_id)) {
            mission_fail(ctx, MISSION_FAULT_QUEUE);
            return;
        }
        mission_enter_state(ctx, MISSION_STATE_STAIR_SCANNING,
                            MISSION_OPERATION_TIMEOUT_MS);
        return;
    }
    /* 2) 未抓满时启动当前层视觉，收到匹配READY后再放行底盘。 */
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

/** 阶梯视觉已停止后，以18到10两段动作保证移动到小圆盘前机械臂安全。 */
static void mission_start_stair_exit(mission_context_t *ctx)
{
    if (!mission_start_arm(ctx, MISSION_STAIR_EXIT_GROUP,
                           MISSION_STATE_STAIR_WAIT_EXIT)) {
        mission_fail(ctx, MISSION_FAULT_ARM);
    }
}

/** 小圆盘完整绕行后先执行21撤离，再由动作完成分支回到动作组10。 */
static void mission_start_small_disc_exit(mission_context_t *ctx)
{
    if (!mission_start_arm(ctx, MISSION_SMALL_DISC_EXIT_GROUP,
                           MISSION_STATE_SMALL_DISC_WAIT_EXIT)) {
        mission_fail(ctx, MISSION_FAULT_ARM);
    }
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

    /* 0) 握手事件允许底盘先于机械臂完成初始化。 */
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
    /* 1) 正式流程只接收当前request_id且is_ready=1的回报。 */
    if ((event->request_id != ctx->request_id) ||
        (event->is_ready == 0U)) {
        if ((event->request_id == ctx->request_id) &&
            (event->is_ready == 0U)) {
            mission_fail(ctx, MISSION_FAULT_CHASSIS);
        }
        return;
    }
    /* 2) 全局停车完成后保持STOPPED，不再推进任务。 */
    if ((ctx->state == MISSION_STATE_STOPPING) &&
        (event->type == CHASSIS_CMD_STOPPED)) {
        mission_enter_state(ctx, MISSION_STATE_STOPPED, 0U);
        return;
    }
    /* 3) 圆盘到位后先执行动作组11，再开启圆盘视觉。 */
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
    /* 4) 阶梯到位后先执行动作组13，层事件可以提前保存。 */
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
    /* 5) LOW/HIGH/MID切层时先停旧视觉，再启动对应层会话。 */
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
    /* 6) 只有底盘确认实际停车后才执行当前层抓取动作组。 */
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
    /* 7) 底盘确认恢复后重新进入当前层扫描状态。 */
    if ((event->type == CHASSIS_CMD_STAIR_RESUME) &&
        (ctx->state == MISSION_STATE_STAIR_WAIT_RESUME)) {
        mission_enter_state(ctx, MISSION_STATE_STAIR_SCANNING,
                            MISSION_OPERATION_TIMEOUT_MS);
        return;
    }
    /* 8) 阶梯走完后停止视觉，再按18、10的顺序准备前往小圆盘。 */
    if (event->type == CHASSIS_CMD_STAIRS_FINISHED) {
        if (ctx->vision.phase != MISSION_VISION_IDLE) {
            mission_enter_state(ctx, MISSION_STATE_STAIR_WAIT_VISION_END,
                                MISSION_OPERATION_TIMEOUT_MS);
            if (!mission_stop_vision(ctx)) {
                mission_fail(ctx, MISSION_FAULT_VISION);
            }
        } else {
            mission_start_stair_exit(ctx);
        }
        return;
    }
    /* 9) 小圆盘到位后先执行动作组19，完成后才开启场景6。 */
    if ((ctx->state == MISSION_STATE_WAIT_SMALL_DISC) &&
        (event->type == CHASSIS_CMD_SMALL_DISC_READY)) {
        if (!mission_start_arm(ctx, MISSION_SMALL_DISC_VISION_GROUP,
                               MISSION_STATE_SMALL_DISC_WAIT_POSE)) {
            mission_fail(ctx, MISSION_FAULT_ARM);
        }
        return;
    }
    /* 10) ACK和实际停车都完成后，才允许动作组20抓取并放球。 */
    if ((ctx->state == MISSION_STATE_SMALL_DISC_WAIT_PAUSE) &&
        (event->type == CHASSIS_CMD_SMALL_DISC_PAUSED)) {
        if (ctx->vision.phase == MISSION_VISION_ACKING) {
            mission_enter_state(ctx, MISSION_STATE_SMALL_DISC_WAIT_ACK,
                                MISSION_OPERATION_TIMEOUT_MS);
        } else if (!mission_start_arm(
                       ctx,
                       MISSION_SMALL_DISC_GRASP_GROUP,
                       MISSION_STATE_SMALL_DISC_WAIT_GRASP)) {
            mission_fail(ctx, MISSION_FAULT_ARM);
        }
        return;
    }
    /* 11) 底盘确认恢复后继续等待视觉事件或完整绕圈结束。 */
    if ((ctx->state == MISSION_STATE_SMALL_DISC_WAIT_RESUME) &&
        (event->type == CHASSIS_CMD_SMALL_DISC_RESUMED)) {
        mission_enter_state(ctx, MISSION_STATE_SMALL_DISC_RUNNING,
                            MISSION_OPERATION_TIMEOUT_MS);
        return;
    }
    /* 12) 抓球数量不改变完整绕圈条件；结束后按21、10进入仓库。 */
    if ((ctx->state == MISSION_STATE_SMALL_DISC_RUNNING) &&
        (event->type == CHASSIS_CMD_SMALL_DISC_FINISHED)) {
        if (ctx->vision.phase != MISSION_VISION_IDLE) {
            mission_enter_state(ctx,
                                MISSION_STATE_SMALL_DISC_WAIT_VISION_END,
                                MISSION_OPERATION_TIMEOUT_MS);
            if (!mission_stop_vision(ctx)) {
                mission_fail(ctx, MISSION_FAULT_VISION);
            }
        } else {
            mission_start_small_disc_exit(ctx);
        }
        return;
    }
}

/** 处理唯一在途动作组结果，并按圆盘、阶梯或小圆盘子流程继续。 */
static void mission_handle_arm(mission_context_t *ctx, bool success)
{
    /* 0) 只处理当前状态正在等待的动作组回报。 */
    if ((ctx->state != MISSION_STATE_WAIT_HOME) &&
        (ctx->state != MISSION_STATE_PLATFORM_WAIT_POSE) &&
        (ctx->state != MISSION_STATE_PLATFORM_WAIT_GRASP) &&
        (ctx->state != MISSION_STATE_PLATFORM_WAIT_AVOID) &&
        (ctx->state != MISSION_STATE_PLATFORM_WAIT_RETURN) &&
        (ctx->state != MISSION_STATE_PLATFORM_WAIT_DEPARTURE_POSE) &&
        (ctx->state != MISSION_STATE_STAIR_WAIT_POSE) &&
        (ctx->state != MISSION_STATE_STAIR_WAIT_GRASP) &&
        (ctx->state != MISSION_STATE_STAIR_WAIT_RETURN) &&
        (ctx->state != MISSION_STATE_STAIR_WAIT_EXIT) &&
        (ctx->state != MISSION_STATE_STAIR_WAIT_SAFE) &&
        (ctx->state != MISSION_STATE_SMALL_DISC_WAIT_POSE) &&
        (ctx->state != MISSION_STATE_SMALL_DISC_WAIT_GRASP) &&
        (ctx->state != MISSION_STATE_SMALL_DISC_WAIT_RETURN) &&
        (ctx->state != MISSION_STATE_SMALL_DISC_WAIT_EXIT) &&
        (ctx->state != MISSION_STATE_SMALL_DISC_WAIT_SAFE)) {
        return;
    }
    /* 1) 任一动作组失败都进入统一故障停车流程。 */
    if (!success) {
        mission_fail(ctx, MISSION_FAULT_ARM);
        return;
    }
    /* 2) 上电动作组10完成后检查转盘，再等待底盘握手。 */
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
    /* 3) 动作组11到位后启动圆盘视觉。 */
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
    /* 4) 动作组12完成后，前四球回11，第五球执行17避让。 */
    if (ctx->state == MISSION_STATE_PLATFORM_WAIT_GRASP) {
        if ((uint8_t)(ctx->platform_balls + 1U) >=
            MISSION_PLATFORM_BALL_COUNT) {
            if (!mission_start_arm(
                    ctx,
                    MISSION_PLATFORM_AVOID_GROUP,
                    MISSION_STATE_PLATFORM_WAIT_AVOID)) {
                mission_fail(ctx, MISSION_FAULT_ARM);
            }
            return;
        }
        /* 前四球先回动作组11，避免机械臂妨碍转盘转动。 */
        if (!mission_start_arm(
                ctx,
                MISSION_PLATFORM_VISION_GROUP,
                MISSION_STATE_PLATFORM_WAIT_RETURN)) {
            mission_fail(ctx, MISSION_FAULT_ARM);
        }
        return;
    }
    if (ctx->state == MISSION_STATE_PLATFORM_WAIT_AVOID) {
        /* 第五球等动作组17完成避让后再读卡并转动转盘。 */
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
        /* 前四球等动作组11回到识别姿态后再读卡并转动转盘。 */
        mission_enter_state(ctx, MISSION_STATE_PLATFORM_WAIT_STORAGE,
                            MISSION_OPERATION_TIMEOUT_MS);
        if (!mission_store_ball(ctx, MISSION_STORAGE_REGION_PLATFORM)) {
            mission_fail(ctx, MISSION_FAULT_STORAGE);
        } else {
            mission_handle_storage(ctx);
        }
        return;
    }
    /* 5) 圆盘存满并执行动作组10后，请求底盘去阶梯。 */
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
    /* 6) 阶梯入口动作组13完成后启动当前层。 */
    if (ctx->state == MISSION_STATE_STAIR_WAIT_POSE) {
        mission_start_stair_layer(ctx);
        return;
    }
    /* 7) 动作组14/15/16抓取完成后统一回动作组13。 */
    if (ctx->state == MISSION_STATE_STAIR_WAIT_GRASP) {
        if (!mission_start_arm(
                ctx,
                MISSION_STAIR_VISION_GROUP,
                MISSION_STATE_STAIR_WAIT_RETURN)) {
            mission_fail(ctx, MISSION_FAULT_ARM);
        }
        return;
    }
    /* 8) 动作组13回位后读IC并将转盘推进一格。 */
    if (ctx->state == MISSION_STATE_STAIR_WAIT_RETURN) {
        mission_enter_state(ctx, MISSION_STATE_STAIR_WAIT_STORAGE,
                            MISSION_OPERATION_TIMEOUT_MS);
        if (!mission_store_ball(ctx, MISSION_STORAGE_REGION_STAIR)) {
            mission_fail(ctx, MISSION_FAULT_STORAGE);
        } else {
            mission_handle_storage(ctx);
        }
        return;
    }
    /* 9) 阶梯结束先由18过渡到10，再通知底盘前往小圆盘。 */
    if (ctx->state == MISSION_STATE_STAIR_WAIT_EXIT) {
        if (!mission_start_arm(ctx, MISSION_HOME_ACTION_GROUP,
                               MISSION_STATE_STAIR_WAIT_SAFE)) {
            mission_fail(ctx, MISSION_FAULT_ARM);
        }
        return;
    }
    if (ctx->state == MISSION_STATE_STAIR_WAIT_SAFE) {
        (void)mission_next_request_id(ctx);
        if (!mission_send_chassis(MISSION_CMD_GO_SMALL_DISC,
                                  ctx->request_id)) {
            mission_fail(ctx, MISSION_FAULT_QUEUE);
            return;
        }
        mission_enter_state(ctx, MISSION_STATE_WAIT_SMALL_DISC,
                            MISSION_OPERATION_TIMEOUT_MS);
        return;
    }
    /* 10) 到达小圆盘后，动作组19到位才启动独立场景6。 */
    if (ctx->state == MISSION_STATE_SMALL_DISC_WAIT_POSE) {
        if (!mission_start_vision(
                ctx,
                MISSION_VISION_SCENE_SMALL_DISC,
                MISSION_STAIR_NONE,
                MISSION_STATE_SMALL_DISC_WAIT_VISION_START)) {
            mission_fail(ctx, MISSION_FAULT_VISION);
        }
        return;
    }
    /* 11) 动作组20抓取放球完成后，先回动作组19再读IC和转动转盘。 */
    if (ctx->state == MISSION_STATE_SMALL_DISC_WAIT_GRASP) {
        if (!mission_start_arm(ctx, MISSION_SMALL_DISC_VISION_GROUP,
                               MISSION_STATE_SMALL_DISC_WAIT_RETURN)) {
            mission_fail(ctx, MISSION_FAULT_ARM);
        }
        return;
    }
    if (ctx->state == MISSION_STATE_SMALL_DISC_WAIT_RETURN) {
        mission_enter_state(ctx, MISSION_STATE_SMALL_DISC_WAIT_STORAGE,
                            MISSION_OPERATION_TIMEOUT_MS);
        if (!mission_store_ball(ctx, MISSION_STORAGE_REGION_SMALL_DISC)) {
            mission_fail(ctx, MISSION_FAULT_STORAGE);
        } else {
            mission_handle_storage(ctx);
        }
        return;
    }
    /* 12) 完整绕圈后由21过渡到10，再下发已有仓库1号位命令。 */
    if (ctx->state == MISSION_STATE_SMALL_DISC_WAIT_EXIT) {
        if (!mission_start_arm(ctx, MISSION_HOME_ACTION_GROUP,
                               MISSION_STATE_SMALL_DISC_WAIT_SAFE)) {
            mission_fail(ctx, MISSION_FAULT_ARM);
        }
        return;
    }
    if (ctx->state == MISSION_STATE_SMALL_DISC_WAIT_SAFE) {
        (void)mission_next_request_id(ctx);
        if (!mission_send_chassis(MISSION_CMD_GO_DEPOT_1, ctx->request_id)) {
            mission_fail(ctx, MISSION_FAULT_QUEUE);
            return;
        }
        /* 仓库到位回执尚未定义，先无超时等待后续仓库流程接入。 */
        mission_enter_state(ctx, MISSION_STATE_WAIT_DEPOT_1, 0U);
    }
}

/** 完成一次存球后更新计数，继续当前区域或进入下一底盘阶段。 */
static void mission_handle_storage(mission_context_t *ctx)
{
    mission_state_t completed_state = ctx->state;

    /* 0) 只接受圆盘、阶梯或小圆盘存球流程的完成结果。 */
    if ((completed_state != MISSION_STATE_PLATFORM_WAIT_STORAGE) &&
        (completed_state != MISSION_STATE_STAIR_WAIT_STORAGE) &&
        (completed_state != MISSION_STATE_SMALL_DISC_WAIT_STORAGE)) {
        return;
    }
    /* 1) 每次读卡和转盘推进成功后占用一个新槽位。 */
    ++ctx->storage_slot;
    if (completed_state == MISSION_STATE_PLATFORM_WAIT_STORAGE) {
        /* 2) 圆盘满5球后执行动作组10，否则继续下一球视觉。 */
        ++ctx->platform_balls;
        if (ctx->platform_balls >= MISSION_PLATFORM_BALL_COUNT) {
            if (!mission_start_arm(
                    ctx,
                    MISSION_HOME_ACTION_GROUP,
                    MISSION_STATE_PLATFORM_WAIT_DEPARTURE_POSE)) {
                mission_fail(ctx, MISSION_FAULT_ARM);
            }
        } else if (!mission_start_vision(
                       ctx,
                       MISSION_VISION_SCENE_PLATFORM,
                       MISSION_STAIR_NONE,
                       MISSION_STATE_PLATFORM_WAIT_VISION)) {
            mission_fail(ctx, MISSION_FAULT_VISION);
        }
        return;
    }
    if (completed_state == MISSION_STATE_STAIR_WAIT_STORAGE) {
        /* 3) 阶梯满2球后直接恢复；未满时先重启本层视觉。 */
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
        return;
    }
    if (completed_state == MISSION_STATE_SMALL_DISC_WAIT_STORAGE) {
        /* 4) 第一球重新开启场景6；第二球后只恢复并走完整个剩余圆周。 */
        ++ctx->small_disc_balls;
        if (ctx->small_disc_balls >= MISSION_SMALL_DISC_BALL_COUNT) {
            if (!mission_send_chassis(MISSION_CMD_SMALL_DISC_RESUME,
                                      ctx->request_id)) {
                mission_fail(ctx, MISSION_FAULT_QUEUE);
                return;
            }
            mission_enter_state(ctx, MISSION_STATE_SMALL_DISC_WAIT_RESUME,
                                MISSION_OPERATION_TIMEOUT_MS);
        } else if (!mission_start_vision(
                       ctx,
                       MISSION_VISION_SCENE_SMALL_DISC,
                       MISSION_STAIR_NONE,
                       MISSION_STATE_SMALL_DISC_WAIT_VISION_RESUME)) {
            mission_fail(ctx, MISSION_FAULT_VISION);
        }
    }
}

/**
 * @brief 处理当前状态的期限到达事件。
 * @param ctx Mission上下文。
 * @note READY到期时自动启动红方；其他有期限状态到期时进入超时故障。
 */
static void mission_check_timeout(mission_context_t *ctx)
{
    if ((ctx->deadline_tick != 0U) &&
        ((int32_t)(osKernelGetTickCount() - ctx->deadline_tick) >= 0)) {
        if (ctx->state == MISSION_STATE_READY) {
            mission_start_run(ctx, MISSION_COLOR_RED);
            return;
        }
        mission_fail(ctx, MISSION_FAULT_TIMEOUT);
    }
}

/**
 * @brief Mission唯一任务，串行处理用户命令、底盘事件和设备结果。
 * @param argument 指向全局Mission上下文。
 * @note 底盘先入队再置位；任务醒来后一次性排空底盘事件队列。
 */
static void mission_task_entry(void *argument)
{
    mission_context_t *ctx = (mission_context_t *)argument;
    chassis_mission_event_t chassis_event;
    mission_user_command_t command;
    uint32_t flags;

    /* 0) 上电先等待动作组10完成，不设置超时。 */
    mission_enter_state(ctx, MISSION_STATE_WAIT_HOME, 0U);
    for (;;) {
        /* 1) 等待任一事件；等待时长由当前状态的截止时间决定。 */
        flags = osThreadFlagsWait(
            MISSION_ALL_FLAGS,
            osFlagsWaitAny,
            mission_wait_ticks(ctx));

        if ((flags & osFlagsError) == 0U) {
            /* 2) 排空用户命令队列。 */
            if ((flags & MISSION_FLAG_COMMAND) != 0U) {
                while (osMessageQueueGet(
                           ctx->command_queue,
                           &command,
                           NULL,
                           0U) == osOK) {
                    mission_handle_command(ctx, command);
                }
            }
            /* 3) 排空底盘事件队列，保持底盘上报顺序。 */
            if ((flags & CHASSIS_MISSION_FLAG_EVENT) != 0U) {
                while (osMessageQueueGet(
                           mission_event_queue,
                           &chassis_event,
                           NULL,
                           0U) == osOK) {
                    mission_handle_chassis(ctx, &chassis_event);
                }
            }
            /* 4) 当前只允许一个动作组在途，失败优先于成功处理。 */
            if ((flags & MISSION_FLAG_ARM_FAIL) != 0U) {
                mission_handle_arm(ctx, false);
            } else if ((flags & MISSION_FLAG_ARM_OK) != 0U) {
                mission_handle_arm(ctx, true);
            }
            /* 5) 解析本轮Nano串口事务结果。 */
            if ((flags & MISSION_FLAG_VISION_DONE) != 0U) {
                mission_handle_vision(ctx);
            }
        }
        /* 6) 提交下一次视觉事务，并统一检查当前状态是否超时。 */
        mission_vision_process(ctx);
        mission_check_timeout(ctx);
    }
}

#if MISSION_CHASSIS_ROUTE_TEST_ENABLED
/** 将测试阶段转换成无线串口可读名称。 */
static const char *mission_test_stage_name(mission_test_stage_t stage)
{
    if (stage == MISSION_TEST_STAGE_PLATFORM) return "PLATFORM";
    if (stage == MISSION_TEST_STAGE_STAIRS) return "STAIRS";
    if (stage == MISSION_TEST_STAGE_SMALL_DISC) return "DISC";
    if (stage == MISSION_TEST_STAGE_DEPOT) return "DEPOT";
    if (stage == MISSION_TEST_STAGE_HOME) return "HOME";
    if (stage == MISSION_TEST_STAGE_DONE) return "DONE";
    return "NONE";
}

/** 将球来源区域转换成无线串口可读名称。 */
static const char *mission_test_region_name(ball_manifest_region_t region)
{
    if (region == BALL_MANIFEST_REGION_TURNTABLE) return "PLATFORM";
    if (region == BALL_MANIFEST_REGION_STAIR) return "STAIRS";
    if (region == BALL_MANIFEST_REGION_PILLAR) return "DISC";
    return "INVALID";
}

/** 将Mission目标颜色转换成无线串口可读名称。 */
static const char *mission_test_color_name(mission_color_t color)
{
    if (color == MISSION_COLOR_RED) return "RED";
    if (color == MISSION_COLOR_BLUE) return "BLUE";
    return "NONE";
}

/** 将档案中的球颜色转换成无线串口可读名称。 */
static const char *mission_test_record_color_name(
    ball_manifest_color_t color)
{
    if (color == BALL_MANIFEST_COLOR_RED) return "RED";
    if (color == BALL_MANIFEST_COLOR_BLUE) return "BLUE";
    return "INVALID";
}

/** 将球档案状态转换成无线串口可读名称。 */
static const char *mission_test_record_state_name(ball_manifest_state_t state)
{
    if (state == BALL_MANIFEST_STATE_STORED) return "STORED";
    if (state == BALL_MANIFEST_STATE_PLACED) return "PLACED";
    if (state == BALL_MANIFEST_STATE_READ_FAILED) return "READ_FAILED";
    return "INVALID";
}

/** @copydoc mission_test_write */
static void mission_test_write(const char *text)
{
    if (text != NULL) {
        (void)debug_uart1_write_text(&g_wireless_test.debug, text);
    }
}

/** @copydoc mission_test_print_status */
static void mission_test_print_status(const mission_context_t *ctx)
{
    const char *mode = "IDLE";

    if (g_wireless_test.mode == MISSION_TEST_MODE_PATH) mode = "PATH";
    if (g_wireless_test.mode == MISSION_TEST_MODE_TARGET) mode = "ROUTE";
    (void)snprintf(
        g_wireless_test.text,
        sizeof(g_wireless_test.text),
        "STATUS MODE=%s TARGET=%s EXPECT=%s STATE=%u COLOR=%s "
        "BALLS=%u SLOT=%u FAULT=%u ARM_BOOT=%u ARM_READY=%u "
        "ARM_GROUP=%u ARM_EVENT=0x%02X\r\n",
        mode,
        mission_test_stage_name(g_wireless_test.target),
        mission_test_stage_name(g_wireless_test.expected),
        (unsigned)ctx->state,
        mission_test_color_name(g_mission_side),
        (unsigned)ctx->manifest.count,
        (unsigned)ctx->storage_slot,
        (unsigned)ctx->fault_code,
        (unsigned)ctx->arm_boot_state,
        (unsigned)g_wireless_test.arm_ready,
        (unsigned)(ctx->arm_last_action_report & 0xFFU),
        (unsigned)((ctx->arm_last_action_report >> 8) & 0xFFU));
    mission_test_write(g_wireless_test.text);
}

/** @copydoc mission_test_print_ball */
static void mission_test_print_ball(
    const mission_context_t *ctx,
    uint8_t sequence)
{
    ball_manifest_record_t record;

    if (ball_manifest_get(&ctx->manifest, sequence, &record) !=
        BALL_MANIFEST_OK) {
        mission_test_write("ERR BALL\r\n");
        return;
    }
    (void)snprintf(
        g_wireless_test.text,
        sizeof(g_wireless_test.text),
        "BALL N=%u RAW_SEQ=%u REGION=%s COLOR=%s IC=0x%02X "
        "ROW=%u COL=%u SLOT=%u STATE=%s\r\n",
        (unsigned)record.sequence + 1U,
        (unsigned)record.sequence,
        mission_test_region_name(record.region),
        mission_test_record_color_name(record.color),
        (unsigned)record.ic_code,
        (unsigned)record.target_row,
        (unsigned)record.target_column,
        (unsigned)record.storage_slot,
        mission_test_record_state_name(record.state));
    mission_test_write(g_wireless_test.text);
}

/** @copydoc mission_test_print_balls */
static void mission_test_print_balls(const mission_context_t *ctx)
{
    uint8_t i;

    (void)snprintf(
        g_wireless_test.text,
        sizeof(g_wireless_test.text),
        "BALL COUNT=%u\r\n",
        (unsigned)ctx->manifest.count);
    mission_test_write(g_wireless_test.text);
    for (i = 0U; i < ctx->manifest.count; ++i) {
        mission_test_print_ball(ctx, i);
    }
}

/** @copydoc mission_test_print_new_balls */
static void mission_test_print_new_balls(const mission_context_t *ctx)
{
    while (g_wireless_test.reported_ball_count < ctx->manifest.count) {
        mission_test_print_ball(ctx, g_wireless_test.reported_ball_count);
        ++g_wireless_test.reported_ball_count;
    }
}

/** @copydoc mission_test_take_command */
static bool mission_test_take_command(char *command, size_t capacity)
{
    uint8_t raw[DEBUG_UART1_RX_BUFFER_SIZE];
    size_t raw_len = 0U;
    size_t in;
    size_t out = 0U;
    bool pending_space = false;

    if ((command == NULL) || (capacity < 2U) ||
        !debug_uart1_take_message(
            &g_wireless_test.debug,
            raw,
            sizeof(raw),
            &raw_len)) {
        return false;
    }
    for (in = 0U; in < raw_len; ++in) {
        char ch = (char)raw[in];

        if ((ch == ' ') || (ch == '\t') || (ch == '\r') || (ch == '\n')) {
            pending_space = (out > 0U);
            continue;
        }
        if (pending_space && (out + 1U < capacity)) {
            command[out++] = ' ';
        }
        pending_space = false;
        if ((ch >= 'a') && (ch <= 'z')) {
            ch = (char)(ch - 'a' + 'A');
        }
        if (out + 1U < capacity) {
            command[out++] = ch;
        }
    }
    command[out] = '\0';
    return (out > 0U);
}

/** @copydoc mission_test_handle_aux_command */
static bool mission_test_handle_aux_command(
    mission_context_t *ctx,
    const char *command)
{
    unsigned int ball_number;

    if (strcmp(command, "HELP") == 0) {
        mission_test_write(
            "PATH: PLATFORM STAIR DISC DEPOT D1 D2 D3 D4\r\n"
            "DEPOT: HOME HOME_DIRECT\r\n"
            "TARGET: ROUTE PLATFORM|STAIRS|DISC RED|BLUE, ROUTE DEPOT\r\n"
            "QUERY: STATUS BALLS BALL n STOP HELP\r\n");
        return true;
    }
    if (strcmp(command, "STATUS") == 0) {
        mission_test_print_status(ctx);
        return true;
    }
    if (strcmp(command, "BALLS") == 0) {
        mission_test_print_balls(ctx);
        return true;
    }
    if ((sscanf(command, "BALL %u", &ball_number) == 1) &&
        (ball_number >= 1U) &&
        (ball_number <= ctx->manifest.count)) {
        mission_test_print_ball(ctx, (uint8_t)(ball_number - 1U));
        return true;
    }
    if (strncmp(command, "BALL ", 5U) == 0) {
        mission_test_write("ERR BALL\r\n");
        return true;
    }
    if (strcmp(command, "STOP") == 0) {
        uint16_t request_id;

        if (g_wireless_test.stop_requested ||
            (ctx->state == MISSION_STATE_STOPPED)) {
            mission_test_write("STOPPED\r\n");
            return true;
        }
        request_id = mission_next_request_id(ctx);
        if (!mission_send_chassis(MISSION_CMD_STOP, request_id)) {
            mission_test_write("ERR STOP\r\n");
            return true;
        }
        g_wireless_test.stop_requested = true;
        mission_enter_state(ctx, MISSION_STATE_STOPPING,
                            MISSION_OPERATION_TIMEOUT_MS);
        mission_test_write("OK STOP\r\n");
        return true;
    }
    return false;
}

/** @copydoc mission_test_pause */
static bool mission_test_pause(mission_context_t *ctx)
{
    char command[DEBUG_UART1_RX_BUFFER_SIZE];
    uint32_t deadline = osKernelGetTickCount() +
        mission_ms_to_ticks(MISSION_CHASSIS_ROUTE_TEST_PAUSE_MS);

    while ((int32_t)(deadline - osKernelGetTickCount()) > 0) {
        if (mission_test_take_command(command, sizeof(command))) {
            if (!mission_test_handle_aux_command(ctx, command)) {
                mission_test_write("BUSY\r\n");
            }
            if (g_wireless_test.stop_requested) {
                (void)mission_test_wait_chassis_event(
                    ctx, CHASSIS_CMD_STOPPED, ctx->request_id);
                return false;
            }
        }
        osDelay(mission_ms_to_ticks(MISSION_WIRELESS_POLL_MS));
    }
    return true;
}

/** @copydoc mission_test_wait_chassis_event */
static bool mission_test_wait_chassis_event(
    mission_context_t *ctx,
    chassis_command_type_t expected,
    uint16_t request_id)
{
    chassis_mission_event_t event;
    char command[DEBUG_UART1_RX_BUFFER_SIZE];
    uint32_t deadline = osKernelGetTickCount() +
        mission_ms_to_ticks(MISSION_OPERATION_TIMEOUT_MS);
    uint32_t poll = mission_ms_to_ticks(MISSION_WIRELESS_POLL_MS);

    for (;;) {
        if (mission_test_take_command(command, sizeof(command)) &&
            !mission_test_handle_aux_command(ctx, command)) {
            mission_test_write("BUSY\r\n");
        }
        if (osMessageQueueGet(
                mission_event_queue, &event, NULL, poll) == osOK) {
            if (g_wireless_test.stop_requested &&
                (event.request_id == ctx->request_id) &&
                (event.type == CHASSIS_CMD_STOPPED)) {
                mission_enter_state(ctx, MISSION_STATE_STOPPED, 0U);
                mission_test_write("STOPPED\r\n");
                return (expected == CHASSIS_CMD_STOPPED);
            }
            if (g_wireless_test.stop_requested) {
                continue;
            }
            if ((event.request_id == request_id) &&
                (event.type == expected)) {
                return (event.is_ready != 0U);
            }
        }
        if ((int32_t)(osKernelGetTickCount() - deadline) >= 0) {
            return false;
        }
    }
}

/** @copydoc mission_test_send_wait */
static bool mission_test_send_wait(
    mission_context_t *ctx,
    mission_command_type_t command,
    chassis_command_type_t expected,
    mission_state_t wait_state)
{
    uint16_t request_id = mission_next_request_id(ctx);

    mission_enter_state(ctx, wait_state, MISSION_OPERATION_TIMEOUT_MS);
    return mission_send_chassis(command, request_id) &&
        mission_test_wait_chassis_event(ctx, expected, request_id);
}

/** @copydoc mission_test_skip_platform */
static bool mission_test_skip_platform(mission_context_t *ctx)
{
    return mission_test_send_wait(
               ctx,
               MISSION_CMD_GO_PLATFORM,
               CHASSIS_CMD_PLATFORM_READY,
               MISSION_STATE_WAIT_PLATFORM) &&
        mission_test_pause(ctx);
}

/** @copydoc mission_test_skip_stairs */
static bool mission_test_skip_stairs(mission_context_t *ctx)
{
    chassis_mission_event_t event;
    char command_text[DEBUG_UART1_RX_BUFFER_SIZE];
    uint16_t request_id = mission_next_request_id(ctx);
    uint32_t deadline;
    uint32_t poll = mission_ms_to_ticks(MISSION_WIRELESS_POLL_MS);

    mission_enter_state(ctx, MISSION_STATE_WAIT_STAIRS,
                        MISSION_OPERATION_TIMEOUT_MS);
    if (!mission_send_chassis(MISSION_CMD_GO_STAIRS, request_id) ||
        !mission_test_wait_chassis_event(
            ctx, CHASSIS_CMD_STAIRS_READY, request_id)) {
        return false;
    }
    deadline = osKernelGetTickCount() +
        mission_ms_to_ticks(MISSION_OPERATION_TIMEOUT_MS);
    for (;;) {
        if (mission_test_take_command(command_text, sizeof(command_text)) &&
            !mission_test_handle_aux_command(ctx, command_text)) {
            mission_test_write("BUSY\r\n");
        }
        if (osMessageQueueGet(
                mission_event_queue, &event, NULL, poll) == osOK) {
            if (g_wireless_test.stop_requested &&
                (event.request_id == ctx->request_id) &&
                (event.type == CHASSIS_CMD_STOPPED)) {
                mission_enter_state(ctx, MISSION_STATE_STOPPED, 0U);
                mission_test_write("STOPPED\r\n");
                return false;
            }
            if (event.request_id != request_id) {
                continue;
            }
            if (event.is_ready == 0U) return false;
            /* 纯路径测试不启动视觉，逐层确认底盘的停车等待。 */
            if ((event.type == CHASSIS_CMD_STAIR_LOW) ||
                (event.type == CHASSIS_CMD_STAIR_HIGH) ||
                (event.type == CHASSIS_CMD_STAIR_MID)) {
                if (!mission_send_chassis(MISSION_CMD_CAM_READY, request_id)) {
                    return false;
                }
            } else if (event.type == CHASSIS_CMD_STAIRS_FINISHED) {
                return mission_test_pause(ctx);
            }
        }
        if ((int32_t)(osKernelGetTickCount() - deadline) >= 0) {
            return false;
        }
    }
}

/** @copydoc mission_test_skip_small_disc */
static bool mission_test_skip_small_disc(mission_context_t *ctx)
{
    uint16_t request_id;

    if (!mission_test_send_wait(
            ctx,
            MISSION_CMD_GO_SMALL_DISC,
            CHASSIS_CMD_SMALL_DISC_READY,
            MISSION_STATE_WAIT_SMALL_DISC)) {
        return false;
    }
    request_id = ctx->request_id;
    if (!mission_send_chassis(MISSION_CMD_SMALL_DISC_START, request_id) ||
        !mission_test_wait_chassis_event(
            ctx, CHASSIS_CMD_SMALL_DISC_FINISHED, request_id)) {
        return false;
    }
    return mission_test_pause(ctx);
}

/**
 * @brief 复用正式子流程处理目标区域事件，在动作组10完成时截断后续路线
 * @param ctx Mission上下文
 * @param target 本次唯一启用视觉抓取的区域
 * @return true=目标完整完成，false=停车或故障
 * @note 正式mission_task_entry及其自动转场不修改；截断只存在于测试任务。
 */
static bool mission_test_run_target(
    mission_context_t *ctx,
    mission_test_stage_t target)
{
    chassis_mission_event_t chassis_event;
    char command[DEBUG_UART1_RX_BUFFER_SIZE];
    uint32_t flags;
    uint32_t wait_ticks;
    bool finish_arm;

    if (target == MISSION_TEST_STAGE_PLATFORM) {
        if (!mission_test_send_wait(
                ctx, MISSION_CMD_GO_PLATFORM, CHASSIS_CMD_PLATFORM_READY,
                MISSION_STATE_WAIT_PLATFORM) ||
            !mission_start_arm(ctx, MISSION_PLATFORM_VISION_GROUP,
                               MISSION_STATE_PLATFORM_WAIT_POSE)) {
            return false;
        }
    } else if (target == MISSION_TEST_STAGE_STAIRS) {
        if (!mission_test_skip_platform(ctx) ||
            !mission_test_send_wait(
                ctx, MISSION_CMD_GO_STAIRS, CHASSIS_CMD_STAIRS_READY,
                MISSION_STATE_WAIT_STAIRS)) {
            return false;
        }
        ctx->stair_layer = MISSION_STAIR_NONE;
        if (!mission_start_arm(ctx, MISSION_STAIR_VISION_GROUP,
                               MISSION_STATE_STAIR_WAIT_POSE)) {
            return false;
        }
    } else if (target == MISSION_TEST_STAGE_SMALL_DISC) {
        if (!mission_test_skip_platform(ctx) ||
            !mission_test_skip_stairs(ctx) ||
            !mission_test_send_wait(
                ctx, MISSION_CMD_GO_SMALL_DISC,
                CHASSIS_CMD_SMALL_DISC_READY,
                MISSION_STATE_WAIT_SMALL_DISC) ||
            !mission_start_arm(ctx, MISSION_SMALL_DISC_VISION_GROUP,
                               MISSION_STATE_SMALL_DISC_WAIT_POSE)) {
            return false;
        }
    } else if (target == MISSION_TEST_STAGE_DEPOT) {
        if (!mission_test_skip_platform(ctx) ||
            !mission_test_skip_stairs(ctx) ||
            !mission_test_skip_small_disc(ctx) ||
            !mission_test_send_wait(
                ctx, MISSION_CMD_GO_DEPOT_1,
                CHASSIS_CMD_DEPOT_1_READY,
                MISSION_STATE_WAIT_DEPOT_1)) {
            return false;
        }
        g_wireless_test.depot_position = 1U;
        return true;
    } else {
        return false;
    }

    for (;;) {
        wait_ticks = mission_wait_ticks(ctx);
        if ((wait_ticks == osWaitForever) ||
            (wait_ticks > mission_ms_to_ticks(MISSION_WIRELESS_POLL_MS))) {
            wait_ticks = mission_ms_to_ticks(MISSION_WIRELESS_POLL_MS);
        }
        flags = osThreadFlagsWait(
            MISSION_ALL_FLAGS, osFlagsWaitAny, wait_ticks);
        if ((flags & osFlagsError) == 0U) {
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
                finish_arm =
                    ((target == MISSION_TEST_STAGE_PLATFORM) &&
                     (ctx->state ==
                      MISSION_STATE_PLATFORM_WAIT_DEPARTURE_POSE)) ||
                    ((target == MISSION_TEST_STAGE_STAIRS) &&
                     (ctx->state == MISSION_STATE_STAIR_WAIT_SAFE)) ||
                    ((target == MISSION_TEST_STAGE_SMALL_DISC) &&
                     (ctx->state == MISSION_STATE_SMALL_DISC_WAIT_SAFE));
                if (finish_arm) {
                    ctx->active_arm_group = 0U;
                    mission_enter_state(ctx, MISSION_STATE_COMPLETE, 0U);
                } else {
                    mission_handle_arm(ctx, true);
                }
            }
            if ((flags & MISSION_FLAG_VISION_DONE) != 0U) {
                mission_handle_vision(ctx);
            }
        }
        mission_vision_process(ctx);
        mission_check_timeout(ctx);
        mission_test_print_new_balls(ctx);
        if (mission_test_take_command(command, sizeof(command)) &&
            !mission_test_handle_aux_command(ctx, command)) {
            mission_test_write("BUSY\r\n");
        }
        if (ctx->state == MISSION_STATE_COMPLETE) return true;
        if ((ctx->state == MISSION_STATE_STOPPED) ||
            (ctx->state == MISSION_STATE_FAULT)) {
            return false;
        }
    }
}

/**
 * @brief USART1无线联调任务：运行纯路径分段测试或单目标视觉抓取测试
 * @param argument 指向全局Mission上下文
 * @note 底盘仍只接收既有Mission命令；正式Mission主任务不参与本测试。
 */
static void mission_wireless_test_entry(void *argument)
{
    static const mission_command_type_t depot_commands[] = {
        MISSION_CMD_GO_DEPOT_1,
        MISSION_CMD_GO_DEPOT_2,
        MISSION_CMD_GO_DEPOT_3,
        MISSION_CMD_GO_DEPOT_4,
    };
    static const chassis_command_type_t depot_events[] = {
        CHASSIS_CMD_DEPOT_1_READY,
        CHASSIS_CMD_DEPOT_2_READY,
        CHASSIS_CMD_DEPOT_3_READY,
        CHASSIS_CMD_DEPOT_4_READY,
    };
    mission_context_t *ctx = (mission_context_t *)argument;
    chassis_mission_event_t event;
    char command[DEBUG_UART1_RX_BUFFER_SIZE];
    uint32_t flags;
    uint16_t request_id;
    uint8_t chassis_ready = 0U;
    uint8_t depot;
    bool ok;

    (void)memset(&g_wireless_test, 0, sizeof(g_wireless_test));
    if (!debug_uart1_init(&g_wireless_test.debug)) {
        mission_enter_state(ctx, MISSION_STATE_FAULT, 0U);
        ctx->fault_code = MISSION_FAULT_PROTOCOL;
        return;
    }
    mission_test_write("MISSION WIRELESS TEST BOOT\r\n");

    /* [lyx] 测试模式只等底盘握手；动作组10成功才开放抓球ROUTE。 */
    mission_enter_state(ctx, MISSION_STATE_WAIT_CHASSIS_READY, 0U);
    while (chassis_ready == 0U) {
        flags = osThreadFlagsWait(
            CHASSIS_MISSION_FLAG_EVENT | MISSION_FLAG_ARM_OK |
                MISSION_FLAG_ARM_FAIL,
            osFlagsWaitAny,
            osWaitForever);
        if ((flags & MISSION_FLAG_ARM_FAIL) != 0U) {
            g_wireless_test.arm_ready = false;
        } else if ((flags & MISSION_FLAG_ARM_OK) != 0U) {
            g_wireless_test.arm_ready = true;
        }
        if ((flags & CHASSIS_MISSION_FLAG_EVENT) != 0U) {
            while (osMessageQueueGet(
                       mission_event_queue, &event, NULL, 0U) == osOK) {
                if ((event.type == CHASSIS_CMD_MISSION_READY) &&
                    (event.is_ready != 0U)) {
                    ctx->request_id = event.request_id;
                    chassis_ready = 1U;
                }
            }
        }
    }
    if (!mission_send_chassis(MISSION_CMD_MISSION_READY, ctx->request_id)) {
        mission_fail(ctx, MISSION_FAULT_QUEUE);
        mission_test_write("FAULT LINK\r\n");
        return;
    }
    mission_enter_state(ctx, MISSION_STATE_READY, 0U);
    g_wireless_test.expected = MISSION_TEST_STAGE_PLATFORM;
    mission_test_write("READY PLATFORM STAIR DISC DEPOT OR ROUTE\r\n");

    for (;;) {
        /* [lyx] 动作组10可晚于底盘握手完成，空闲时继续接收其结果。 */
        flags = osThreadFlagsWait(
            MISSION_FLAG_ARM_OK | MISSION_FLAG_ARM_FAIL,
            osFlagsWaitAny, 0U);
        /* 0超时无事件会返回osFlagsErrorResource，不能按事件位解释。 */
        if ((flags & osFlagsError) == 0U) {
            if ((flags & MISSION_FLAG_ARM_FAIL) != 0U) {
                g_wireless_test.arm_ready = false;
            } else if ((flags & MISSION_FLAG_ARM_OK) != 0U) {
                g_wireless_test.arm_ready = true;
            }
        }
        if (!mission_test_take_command(command, sizeof(command))) {
            osDelay(mission_ms_to_ticks(MISSION_WIRELESS_POLL_MS));
            continue;
        }
        if (mission_test_handle_aux_command(ctx, command)) {
            if (g_wireless_test.stop_requested &&
                (ctx->state != MISSION_STATE_STOPPED)) {
                (void)mission_test_wait_chassis_event(
                    ctx, CHASSIS_CMD_STOPPED, ctx->request_id);
            }
            continue;
        }
        if (g_wireless_test.stop_requested ||
            (ctx->state == MISSION_STATE_STOPPED) ||
            ((ctx->state == MISSION_STATE_COMPLETE) &&
             (g_wireless_test.target != MISSION_TEST_STAGE_DEPOT))) {
            mission_test_write("ERR RESET REQUIRED\r\n");
            continue;
        }

        /* 1) ROUTE命令自动跳过前置区域，只在目标区域执行正式业务。 */
        if ((strcmp(command, "ROUTE PLATFORM RED") == 0) ||
            (strcmp(command, "ROUTE PLATFORM BLUE") == 0) ||
            (strcmp(command, "ROUTE STAIRS RED") == 0) ||
            (strcmp(command, "ROUTE STAIRS BLUE") == 0) ||
            (strcmp(command, "ROUTE DISC RED") == 0) ||
            (strcmp(command, "ROUTE DISC BLUE") == 0) ||
            (strcmp(command, "ROUTE DEPOT") == 0)) {
            if (g_wireless_test.mode != MISSION_TEST_MODE_IDLE) {
                mission_test_write("ERR RESET REQUIRED\r\n");
                continue;
            }
            if (strncmp(command, "ROUTE PLATFORM", 14U) == 0) {
                g_wireless_test.target = MISSION_TEST_STAGE_PLATFORM;
            } else if (strncmp(command, "ROUTE STAIRS", 12U) == 0) {
                g_wireless_test.target = MISSION_TEST_STAGE_STAIRS;
            } else if (strncmp(command, "ROUTE DISC", 10U) == 0) {
                g_wireless_test.target = MISSION_TEST_STAGE_SMALL_DISC;
            } else {
                g_wireless_test.target = MISSION_TEST_STAGE_DEPOT;
            }
            /* [lyx] 抓球ROUTE不能在机械臂未就绪时静默跳过动作组。 */
            if ((g_wireless_test.target != MISSION_TEST_STAGE_DEPOT) &&
                !g_wireless_test.arm_ready) {
                mission_test_write("ERR ARM UNAVAILABLE\r\n");
                continue;
            }
            g_wireless_test.mode = MISSION_TEST_MODE_TARGET;
            g_mission_side = (strstr(command, " BLUE") != NULL)
                ? MISSION_COLOR_BLUE : MISSION_COLOR_RED;
            if (g_wireless_test.target == MISSION_TEST_STAGE_DEPOT) {
                g_mission_side = MISSION_COLOR_NONE;
            }
            ctx->platform_balls = 0U;
            ctx->stair_balls = 0U;
            ctx->small_disc_balls = 0U;
            ctx->storage_slot = 0U;
            ctx->fault_code = MISSION_FAULT_NONE;
            ball_manifest_init(&ctx->manifest);
            g_wireless_test.reported_ball_count = 0U;
            if ((g_wireless_test.target != MISSION_TEST_STAGE_DEPOT) &&
                !mission_prepare_zdt(ctx)) {
                mission_fail(ctx, MISSION_FAULT_STORAGE);
                mission_test_write("FAULT STORAGE INIT\r\n");
                continue;
            }
            (void)snprintf(
                g_wireless_test.text,
                sizeof(g_wireless_test.text),
                "OK ROUTE TARGET=%s COLOR=%s\r\n",
                mission_test_stage_name(g_wireless_test.target),
                mission_test_color_name(g_mission_side));
            mission_test_write(g_wireless_test.text);
            ok = mission_test_run_target(ctx, g_wireless_test.target);
            if (ok) {
                g_wireless_test.expected =
                    (g_wireless_test.target == MISSION_TEST_STAGE_DEPOT)
                    ? MISSION_TEST_STAGE_DEPOT : MISSION_TEST_STAGE_DONE;
                (void)snprintf(
                    g_wireless_test.text,
                    sizeof(g_wireless_test.text),
                    "DONE %s\r\n",
                    mission_test_stage_name(g_wireless_test.target));
                mission_test_write(g_wireless_test.text);
                if (g_wireless_test.target == MISSION_TEST_STAGE_DEPOT) {
                    mission_test_write(
                        "READY D1 D2 D3 D4 HOME HOME_DIRECT\r\n");
                }
            } else if (!g_wireless_test.stop_requested) {
                mission_fail(ctx, MISSION_FAULT_CHASSIS);
                mission_test_write("FAULT ROUTE\r\n");
            }
            continue;
        }

        /* 2) 纯路径指令直达目标阶段；已完成的前段不重复执行。 */
        if (strcmp(command, "PLATFORM") == 0) {
            if ((g_wireless_test.mode != MISSION_TEST_MODE_IDLE) ||
                (g_wireless_test.expected != MISSION_TEST_STAGE_PLATFORM)) {
                mission_test_write("ERR ORDER EXPECT=PLATFORM\r\n");
                continue;
            }
            g_wireless_test.mode = MISSION_TEST_MODE_PATH;
            g_mission_side = MISSION_COLOR_NONE;
            ok = mission_test_skip_platform(ctx);
            if (ok) {
                g_wireless_test.expected = MISSION_TEST_STAGE_STAIRS;
                mission_test_write("DONE PLATFORM\r\nREADY STAIR DISC DEPOT\r\n");
            }
        } else if ((strcmp(command, "STAIR") == 0) ||
                   (strcmp(command, "STAIRS") == 0)) {
            if ((g_wireless_test.mode == MISSION_TEST_MODE_TARGET) ||
                ((g_wireless_test.expected != MISSION_TEST_STAGE_PLATFORM) &&
                 (g_wireless_test.expected != MISSION_TEST_STAGE_STAIRS))) {
                (void)snprintf(
                    g_wireless_test.text,
                    sizeof(g_wireless_test.text),
                    "ERR ORDER EXPECT=%s\r\n",
                    mission_test_stage_name(g_wireless_test.expected));
                mission_test_write(g_wireless_test.text);
                continue;
            }
            g_wireless_test.mode = MISSION_TEST_MODE_PATH;
            g_mission_side = MISSION_COLOR_NONE;
            ok = ((g_wireless_test.expected != MISSION_TEST_STAGE_PLATFORM) ||
                  mission_test_skip_platform(ctx)) &&
                 mission_test_skip_stairs(ctx);
            if (ok) {
                g_wireless_test.expected = MISSION_TEST_STAGE_SMALL_DISC;
                mission_test_write("DONE STAIRS\r\nREADY DISC DEPOT\r\n");
            }
        } else if (strcmp(command, "DISC") == 0) {
            if ((g_wireless_test.mode == MISSION_TEST_MODE_TARGET) ||
                ((g_wireless_test.expected != MISSION_TEST_STAGE_PLATFORM) &&
                 (g_wireless_test.expected != MISSION_TEST_STAGE_STAIRS) &&
                 (g_wireless_test.expected !=
                  MISSION_TEST_STAGE_SMALL_DISC))) {
                (void)snprintf(
                    g_wireless_test.text,
                    sizeof(g_wireless_test.text),
                    "ERR ORDER EXPECT=%s\r\n",
                    mission_test_stage_name(g_wireless_test.expected));
                mission_test_write(g_wireless_test.text);
                continue;
            }
            g_wireless_test.mode = MISSION_TEST_MODE_PATH;
            g_mission_side = MISSION_COLOR_NONE;
            ok = ((g_wireless_test.expected != MISSION_TEST_STAGE_PLATFORM) ||
                  mission_test_skip_platform(ctx)) &&
                 ((g_wireless_test.expected == MISSION_TEST_STAGE_SMALL_DISC) ||
                  mission_test_skip_stairs(ctx)) &&
                 mission_test_skip_small_disc(ctx);
            if (ok) {
                g_wireless_test.expected = MISSION_TEST_STAGE_DEPOT;
                mission_test_write("DONE DISC\r\nREADY DEPOT\r\n");
            }
        } else if (strcmp(command, "DEPOT") == 0) {
            if ((g_wireless_test.mode == MISSION_TEST_MODE_TARGET) ||
                ((g_wireless_test.expected != MISSION_TEST_STAGE_PLATFORM) &&
                 (g_wireless_test.expected != MISSION_TEST_STAGE_STAIRS) &&
                 (g_wireless_test.expected != MISSION_TEST_STAGE_SMALL_DISC) &&
                 (g_wireless_test.expected != MISSION_TEST_STAGE_DEPOT))) {
                (void)snprintf(
                    g_wireless_test.text,
                    sizeof(g_wireless_test.text),
                    "ERR ORDER EXPECT=%s\r\n",
                    mission_test_stage_name(g_wireless_test.expected));
                mission_test_write(g_wireless_test.text);
                continue;
            }
            g_wireless_test.mode = MISSION_TEST_MODE_PATH;
            g_mission_side = MISSION_COLOR_NONE;
            /* 小圆盘完成后先停车，再沿用原命令进入仓库1号位。 */
            ok = ((g_wireless_test.expected != MISSION_TEST_STAGE_PLATFORM) ||
                  mission_test_skip_platform(ctx)) &&
                 ((g_wireless_test.expected != MISSION_TEST_STAGE_PLATFORM &&
                   g_wireless_test.expected != MISSION_TEST_STAGE_STAIRS) ||
                  mission_test_skip_stairs(ctx)) &&
                 ((g_wireless_test.expected == MISSION_TEST_STAGE_DEPOT) ||
                  mission_test_skip_small_disc(ctx)) &&
                 mission_test_send_wait(
                     ctx, MISSION_CMD_GO_DEPOT_1,
                     CHASSIS_CMD_DEPOT_1_READY,
                     MISSION_STATE_WAIT_DEPOT_1);
            if (ok && mission_test_pause(ctx)) {
                /* 直达仓库成功后开放D1~D4和回家指令。 */
                g_wireless_test.expected = MISSION_TEST_STAGE_DEPOT;
                g_wireless_test.depot_position = 1U;
                mission_test_write(
                    "DONE DEPOT\r\n"
                    "READY D1 D2 D3 D4 HOME HOME_DIRECT\r\n");
            } else {
                ok = false;
            }
        } else if ((command[0] == 'D') &&
                   (command[1] >= '1') && (command[1] <= '4') &&
                   (command[2] == '\0')) {
            if ((g_wireless_test.depot_position == 0U) ||
                (g_wireless_test.expected != MISSION_TEST_STAGE_DEPOT)) {
                mission_test_write("ERR ORDER EXPECT=DEPOT\r\n");
                continue;
            }
            depot = (uint8_t)(command[1] - '0');
            ok = mission_test_send_wait(
                ctx,
                depot_commands[depot - 1U],
                depot_events[depot - 1U],
                MISSION_STATE_WAIT_DEPOT_1);
            if (ok) {
                g_wireless_test.depot_position = depot;
                (void)snprintf(
                    g_wireless_test.text,
                    sizeof(g_wireless_test.text),
                    "DONE D%u\r\n",
                    (unsigned)depot);
                mission_test_write(g_wireless_test.text);
            }
        } else if ((strcmp(command, "HOME") == 0) ||
                   (strcmp(command, "HOME_DIRECT") == 0)) {
            if ((g_wireless_test.depot_position == 0U) ||
                (g_wireless_test.expected != MISSION_TEST_STAGE_DEPOT)) {
                mission_test_write("ERR ORDER EXPECT=DEPOT\r\n");
                continue;
            }
            ok = true;
            /* HOME先回1号位；HOME_DIRECT用于验证当前位置直接回家。 */
            if ((strcmp(command, "HOME") == 0) &&
                (g_wireless_test.depot_position != 1U)) {
                ok = mission_test_send_wait(
                    ctx, MISSION_CMD_GO_DEPOT_1,
                    CHASSIS_CMD_DEPOT_1_READY,
                    MISSION_STATE_WAIT_DEPOT_1);
                if (ok) g_wireless_test.depot_position = 1U;
            }
            if (ok) {
                request_id = mission_next_request_id(ctx);
                mission_enter_state(ctx, MISSION_STATE_WAIT_DEPOT_1,
                                    MISSION_OPERATION_TIMEOUT_MS);
                ok = mission_send_chassis(
                         MISSION_CMD_DEPOT_OK, request_id) &&
                    mission_test_wait_chassis_event(
                        ctx, CHASSIS_CMD_HOME_READY, request_id);
            }
            if (ok) {
                g_wireless_test.expected = MISSION_TEST_STAGE_DONE;
                mission_enter_state(ctx, MISSION_STATE_COMPLETE, 0U);
                mission_test_write(
                    (strcmp(command, "HOME_DIRECT") == 0)
                    ? "DONE HOME_DIRECT\r\n" : "DONE HOME\r\n");
            }
        } else {
            (void)snprintf(
                g_wireless_test.text,
                sizeof(g_wireless_test.text),
                "ERR ORDER EXPECT=%s\r\n",
                mission_test_stage_name(g_wireless_test.expected));
            mission_test_write(g_wireless_test.text);
            continue;
        }

        if (!ok && !g_wireless_test.stop_requested &&
            (ctx->state != MISSION_STATE_FAULT)) {
            mission_fail(ctx, MISSION_FAULT_CHASSIS);
            mission_test_write("FAULT PATH\r\n");
        }
    }
}
#endif

/** @copydoc mission_app_init() */
mission_app_status_t mission_app_init(void)
{
    mission_context_t *ctx = &g_mission;

    /* 0) 初始化只允许执行一次。 */
    if (ctx->initialized) {
        return MISSION_APP_ERR_STATE;
    }
    /* 1) 清空运行上下文并初始化小球档案。 */
    (void)memset(ctx, 0, sizeof(*ctx));
    g_mission_side = MISSION_COLOR_NONE;
    ball_manifest_init(&ctx->manifest);
    /* 2) 目标区域测试会复用正式读卡和车载转盘服务。 */
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
#if !MISSION_CHASSIS_ROUTE_TEST_ENABLED
    /* 3) 正式流程创建用户命令队列；无线测试直接读取USART1。 */
    ctx->command_queue = osMessageQueueNew(
        MISSION_COMMAND_QUEUE_DEPTH,
        sizeof(mission_user_command_t),
        NULL);
    if (ctx->command_queue == NULL) {
        return MISSION_APP_ERR_RESOURCE;
    }
#endif
#if MISSION_CHASSIS_ROUTE_TEST_ENABLED
    /* 测试模式只创建独立无线任务，不改动正式Mission主任务。 */
    ctx->task = osThreadNew(
        mission_wireless_test_entry, ctx, &g_mission_task_attr);
#else
    ctx->task = osThreadNew(mission_task_entry, ctx, &g_mission_task_attr);
#endif
    if (ctx->task == NULL) {
        return MISSION_APP_ERR_RESOURCE;
    }
    /* 5) 将底盘和机械臂异步回报绑定到Mission任务。 */
    if (!chassis_mission_link_bind_mission_task(ctx->task)) {
        return MISSION_APP_ERR_RESOURCE;
    }
#if MISSION_CHASSIS_ROUTE_TEST_ENABLED
    /* [lyx] 测试模式尝试动作组10，但缺臂或下发失败不阻塞纯路径。 */
    ctx->initialized = true;
    if (arm_on_report(mission_arm_report, ctx) == LSC16_OK) {
        ctx->active_arm_group = MISSION_HOME_ACTION_GROUP;
        if (arm_run(
                MISSION_HOME_ACTION_GROUP,
                1U,
                mission_arm_tx_done,
                ctx) != LSC16_OK) {
            ctx->active_arm_group = 0U;
            ctx->arm_boot_state = 2U;
        } else {
            ctx->arm_boot_state = 3U;
        }
    } else {
        ctx->arm_boot_state = 1U;
    }
#else
    if (arm_on_report(mission_arm_report, ctx) != LSC16_OK) {
        return MISSION_APP_ERR_IO;
    }
    /* 6) 启动动作组10；其完成回报会继续初始化握手。 */
    ctx->initialized = true;
    ctx->active_arm_group = MISSION_HOME_ACTION_GROUP;
    if (arm_run(
            MISSION_HOME_ACTION_GROUP,
            1U,
            mission_arm_tx_done,
            ctx) != LSC16_OK) {
        return MISSION_APP_ERR_IO;
    }
#endif
    return MISSION_APP_OK;
}

/** @copydoc mission_app_submit_command() */
mission_app_status_t mission_app_submit_command(mission_user_command_t command)
{
#if MISSION_CHASSIS_ROUTE_TEST_ENABLED
    /* 无线测试任务直接解析USART1，不接收正式红蓝方用户命令。 */
    (void)command;
    return MISSION_APP_ERR_STATE;
#else
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
#endif
}

/** @copydoc mission_app_get_snapshot() */
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
    snapshot->color = g_mission_side;
    snapshot->stair_layer = ctx->stair_layer;
    snapshot->chassis_request_id = ctx->request_id;
    snapshot->platform_balls = ctx->platform_balls;
    snapshot->stair_balls = ctx->stair_balls;
    snapshot->small_disc_balls = ctx->small_disc_balls;
    snapshot->storage_slot = ctx->storage_slot;
    snapshot->fault_code = ctx->fault_code;
    taskEXIT_CRITICAL();
    return MISSION_APP_OK;
}
