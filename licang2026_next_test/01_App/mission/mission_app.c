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

/* [lyx] 复用无线测试的USART1封装；仅Mission任务格式化，禁止回调中打印。 */
#if MISSION_DEPOT_TRACE_ENABLED && !MISSION_CHASSIS_ROUTE_TEST_ENABLED
#include "debug_uart1.h"
static debug_uart1_t g_depot_debug;
static char g_depot_trace_text[128];
#define DEPOT_TRACE(...) do { \
    (void)snprintf(g_depot_trace_text, sizeof(g_depot_trace_text), __VA_ARGS__); \
    (void)debug_uart1_write_text(&g_depot_debug, g_depot_trace_text); \
} while (0)
#else
#define DEPOT_TRACE(...) ((void)0)
#endif

#if MISSION_CHASSIS_ROUTE_TEST_ENABLED
#include "debug_uart1.h"
#define BLOCK_TRACE(...) do { \
    (void)snprintf(g_wireless_test.text, sizeof(g_wireless_test.text), __VA_ARGS__); \
    mission_test_write(g_wireless_test.text); \
} while (0)
#else
#define BLOCK_TRACE(...) DEPOT_TRACE(__VA_ARGS__)
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
    MISSION_VISION_SCENE_DEPOT_DIGIT,
    MISSION_VISION_SCENE_BLOCK_DIGIT,
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
    MISSION_VISION_MODEL_QUERYING,
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
    volatile bool inflight;
    volatile bool completion_pending; /* 回调已到但Mission尚未检查，不能复用mail/tx。 */
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
    uint8_t platform_attempts; /* 动作12成功入队计一次，最多7次；成功收球仍最多5个。 */
    bool platform_read_ok;
    uint8_t stair_balls;
    uint8_t small_disc_balls;
    uint8_t storage_slot;
    uint8_t fault_code;

    /* 正式仓库使用真实0基槽位；异常标记按档案序号保存，不删除原始记录。 */
    uint8_t current_slot;
    uint8_t depot_target_slot;
    uint8_t depot_position;
    uint8_t depot_column;
    uint8_t depot_first_digit;
    uint8_t depot_digit;
    uint8_t depot_columns_used;
    uint8_t depot_sequence;
    uint8_t depot_row;
    uint16_t depot_abnormal_mask;
    bool depot_preparing;

    /* 设备协议细节收在子对象中，顶层流程仍只使用一个state。 */
    mission_vision_t vision;
    mission_storage_t storage;
    ball_manifest_t manifest;

    /* 初始化握手允许Mission和底盘以任意先后顺序完成。 */
    bool initialized;
    bool chassis_ready;
    bool block_model_ready;
    uint32_t model_next_query_tick;
    uint32_t model_query_retries; /* 起点连续无效查询数；有效模型回复后清零。 */
    nano_vision_block_result_t block_result;
    bool block_result_received;
    /* 无线搬运与正式共用；源层按stage降序，digit只表示D4目标层。 */
    uint8_t block_stage;
    uint8_t block_step;
    uint8_t block_point;
    uint8_t block_digit;
    uint8_t block_found_mask;
    uint8_t block_placed_mask;
    uint8_t active_arm_group;
    volatile bool arm_home_ready;          /* 仅动作10完成回报置位；新动作或异常使姿态失效。 */
#if MISSION_CHASSIS_ROUTE_TEST_ENABLED
    uint8_t arm_boot_state;                 /* 0=未尝试，1=注册失败，2=下发失败，3=已入队。 */
#endif
#if MISSION_CHASSIS_ROUTE_TEST_ENABLED || MISSION_DEPOT_TRACE_ENABLED
    volatile uint32_t arm_last_action_report; /* 高位为事件，低8位为动作组。 */
#endif
} mission_context_t;


/* 阶梯诊断只在正式日志开关打开时存在；不参与状态机或接收判定。 */
#if MISSION_DEPOT_TRACE_ENABLED && !MISSION_CHASSIS_ROUTE_TEST_ENABLED
typedef struct {
    uint16_t sid;
    uint8_t scene;
    uint32_t tick;
    uint32_t accepted;
    uint32_t rejected[7]; /* decode/SID/scene/status/color/age/state，首次失败。 */
    bool has_rejected;
    nano_vision_event_t last_rejected;
} mission_stair_diag_t;
static mission_stair_diag_t g_stair_diag, g_stair_closed;
static bool g_stair_closed_pending;
static bool g_stair_run_active;
static uint32_t g_stair_events, g_stair_grasps, g_stair_ic_ok, g_stair_ic_failed;
static uint32_t g_stair_arm_report;

static bool mission_stair_trace_active(const mission_context_t *ctx)
{
    return (ctx->state >= MISSION_STATE_WAIT_STAIRS) &&
           (ctx->state <= MISSION_STATE_STAIR_WAIT_SAFE);
}
static unsigned long mission_stair_time_ms(void)
{
    return (unsigned long)(((uint64_t)osKernelGetTickCount() * 1000U) /
                           osKernelGetTickFreq());
}
#define STAIR_TRACE(ctx, fmt, ...) do { \
    if (mission_stair_trace_active(ctx)) \
        DEPOT_TRACE("[S] T=%lu " fmt, mission_stair_time_ms(), ##__VA_ARGS__); \
} while (0)
#define STAIR_COUNT(name) (++g_stair_##name)
#define STAIR_RESET() do { \
    memset(&g_stair_diag, 0, sizeof(g_stair_diag)); \
    g_stair_closed_pending = false; \
    g_stair_run_active = true; \
    g_stair_arm_report = 0U; \
    g_stair_events = g_stair_grasps = g_stair_ic_ok = g_stair_ic_failed = 0U; \
} while (0)
static void mission_stair_session(mission_context_t *ctx, bool begin)
{
    if (begin) {
        if (!mission_stair_trace_active(ctx)) return;
        memset(&g_stair_diag, 0, sizeof(g_stair_diag));
        g_stair_diag.sid = ctx->vision.session_id;
        g_stair_diag.scene = (uint8_t)ctx->vision.scene;
        g_stair_diag.tick = osKernelGetTickCount();
    } else if ((g_stair_diag.sid != 0U) &&
               (g_stair_diag.sid == ctx->vision.session_id)) {
        /* ACK/STOP处理时只保存；主循环提交关键操作后再输出，避免插在抓取前。 */
        g_stair_closed = g_stair_diag;
        g_stair_closed_pending = true;
        g_stair_diag.sid = 0U;
    }
}
static void mission_stair_reject(mission_context_t *ctx, nano_vision_status_t status,
                                 const nano_vision_event_t *event)
{
    unsigned reason;
    if (g_stair_diag.sid != ctx->vision.session_id || g_stair_diag.sid == 0U) return;
    if (status != NANO_VISION_OK) reason = 0U;
    else if (event->session_id != ctx->vision.session_id) reason = 1U;
    else if (event->observation.scene != ctx->vision.scene) reason = 2U;
    else if (event->observation.status != NANO_VISION_OBS_VALID) reason = 3U;
    else if (event->observation.color != ((g_mission_side == MISSION_COLOR_RED) ?
             NANO_VISION_COLOR_RED : NANO_VISION_COLOR_BLUE)) reason = 4U;
    else reason = 5U;
    ++g_stair_diag.rejected[reason];
    g_stair_diag.has_rejected = (status == NANO_VISION_OK);
    if (g_stair_diag.has_rejected) g_stair_diag.last_rejected = *event;
}
static void mission_stair_summary(const mission_stair_diag_t *d, bool end)
{
    if (d->has_rejected) {
        DEPOT_TRACE("[S] SID=%u SCENE=%u END=%u EVENT=%lu BAD=%u/%u/%u/%u/%u/%u\r\n",
            (unsigned)d->sid, (unsigned)d->scene, (unsigned)end, (unsigned long)d->accepted,
            (unsigned)d->last_rejected.session_id, (unsigned)d->last_rejected.observation.frame_id,
            (unsigned)d->last_rejected.observation.scene, (unsigned)d->last_rejected.observation.status,
            (unsigned)d->last_rejected.observation.color, (unsigned)d->last_rejected.observation.age_ms);
    } else {
        DEPOT_TRACE("[S] SID=%u SCENE=%u END=%u EVENT=%lu BAD=NA\r\n",
            (unsigned)d->sid, (unsigned)d->scene, (unsigned)end, (unsigned long)d->accepted);
    }
    DEPOT_TRACE("[S] SID=%u REJ=%lu,%lu,%lu,%lu,%lu,%lu,%lu\r\n",
        (unsigned)d->sid, (unsigned long)d->rejected[0],
        (unsigned long)d->rejected[1], (unsigned long)d->rejected[2],
        (unsigned long)d->rejected[3], (unsigned long)d->rejected[4],
        (unsigned long)d->rejected[5], (unsigned long)d->rejected[6]);
}
static void mission_stair_poll(mission_context_t *ctx)
{
    uint32_t now = osKernelGetTickCount();
    if (mission_stair_trace_active(ctx) &&
        g_stair_arm_report != ctx->arm_last_action_report) {
        g_stair_arm_report = ctx->arm_last_action_report;
        STAIR_TRACE(ctx, "ARM_RAW GROUP=%u EVENT=0x%02X WAIT=%u\r\n",
            (unsigned)(g_stair_arm_report & 0xFFU),
            (unsigned)((g_stair_arm_report >> 8) & 0xFFU),
            (unsigned)ctx->active_arm_group);
    }
    if (g_stair_closed_pending) {
        mission_stair_summary(&g_stair_closed, true);
        g_stair_closed_pending = false;
    } else if ((g_stair_diag.sid != 0U) &&
               ((now - g_stair_diag.tick) >= osKernelGetTickFreq())) {
        mission_stair_summary(&g_stair_diag, false);
        g_stair_diag.tick = now;
    }
}
#define STAIR_SESSION(ctx, begin) mission_stair_session(ctx, begin)
#define STAIR_REJECT(ctx, status, event) mission_stair_reject(ctx, status, event)
#define STAIR_STATE_REJECT() (++g_stair_diag.rejected[6])
#define STAIR_ACCEPT() do { ++g_stair_diag.accepted; ++g_stair_events; } while (0)
#define STAIR_POLL(ctx) mission_stair_poll(ctx)
#define STAIR_TOTAL(ctx) do { if (g_stair_run_active) { \
    DEPOT_TRACE("[S] END EVENTS=%lu GRASP_DONE=%lu IC_OK=%lu READ_FAILED=%lu COUNT=%u\r\n", \
    (unsigned long)g_stair_events, (unsigned long)g_stair_grasps, \
    (unsigned long)g_stair_ic_ok, (unsigned long)g_stair_ic_failed, (unsigned)(ctx)->stair_balls); \
    g_stair_run_active = false; } } while (0)
#else
#define STAIR_TRACE(...) ((void)0)
#define STAIR_COUNT(...) ((void)0)
#define STAIR_RESET(...) ((void)0)
#define STAIR_SESSION(...) ((void)0)
#define STAIR_REJECT(...) ((void)0)
#define STAIR_STATE_REJECT(...) ((void)0)
#define STAIR_ACCEPT(...) ((void)0)
#define STAIR_POLL(...) ((void)0)
#define STAIR_TOTAL(...) ((void)0)
#endif

static mission_context_t g_mission;

/* 当前比赛红蓝方由Mission唯一维护，其他模块只读并据此选择地图。 */
volatile mission_color_t g_mission_side = MISSION_COLOR_NONE;

#if MISSION_CHASSIS_ROUTE_TEST_ENABLED
/** USART1无线联调任务的运行状态和文本缓冲区。 */
typedef struct {
    ic_ball_t ball;
    bool placed;
} mission_test_ball_t;

typedef struct {
    debug_uart1_t debug;                   /* 现有DMA空闲接收封装。 */
    mission_test_mode_t mode;              /* 本轮测试方式。 */
    mission_test_stage_t target;           /* 唯一启用抓取的区域。 */
    mission_test_stage_t expected;         /* 下一个未完成的纯路径阶段。 */
    uint8_t depot_position;                /* 0=未进仓库，1~4=当前位置。 */
    uint8_t reported_ball_count;           /* 已自动打印的球记录数。 */
    uint8_t depot_digit;                   /* 当前停车位已确认的底部数字，0表示尚无。 */
    bool arm_ready;                         /* 动作组10完成回报已被测试任务确认。 */
    bool stop_requested;                   /* STOP已下发，等待停车回执。 */
    bool small_disc_at_start;              /* DISC_READY已停车，只允许DISC_RUN启动绕行。 */
    bool depot_wait_ball_home;             /* ROUTE DEPOT BALL已在D1停车，等待装球命令。 */
    bool ball_home;                        /* 人工装球测试已开始。 */
    bool ball_home_ready;                  /* 9球读完且转盘已停在12槽。 */
    bool block_move;                       /* 新搬运测试使用共用状态机；旧数字测试不抓放。 */
    uint8_t current_slot;                  /* 当前取球工位对应的物理槽号1~12。 */
    uint8_t manual_count;
    mission_test_ball_t manual_balls[BALL_MANIFEST_CAPACITY];
    char text[224];                        /* 单条可读回复缓冲区。 */
} mission_wireless_test_t;

static mission_wireless_test_t g_wireless_test;
#define PLATFORM_TRACE(...) do { \
    if (g_wireless_test.target == MISSION_TEST_STAGE_PLATFORM) { \
        (void)snprintf(g_wireless_test.text, sizeof(g_wireless_test.text), __VA_ARGS__); \
        (void)debug_uart1_write_text(&g_wireless_test.debug, g_wireless_test.text); \
    } \
} while (0)
#else
#define PLATFORM_TRACE(...) ((void)0)
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
static void mission_model_process(mission_context_t *ctx);
/** 双方就绪后完成握手并进入READY状态。 */
static void mission_try_ready(mission_context_t *ctx);
/** 按已选定的红蓝方启动一轮正式任务。 */
static void mission_start_run(mission_context_t *ctx);
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
static void mission_block_begin(mission_context_t *ctx);
static void mission_block_start_layer(mission_context_t *ctx);
static void mission_block_move(mission_context_t *ctx, uint8_t point,
                               mission_state_t wait_state);
static void mission_block_position_done(mission_context_t *ctx);
static void mission_block_digit_done(mission_context_t *ctx);
static void mission_block_arm_done(mission_context_t *ctx);
#if !MISSION_CHASSIS_ROUTE_TEST_ENABLED
static void mission_depot_next_ball(mission_context_t *ctx);
static void mission_depot_start_digit(mission_context_t *ctx);
static void mission_depot_digit_done(mission_context_t *ctx);
static void mission_depot_arm_done(mission_context_t *ctx);
static void mission_depot_prepare(mission_context_t *ctx);
#endif

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
/** 在仓库单个停车位读取数字，NONE最多等待3秒。 */
static bool mission_test_read_depot_digit(mission_context_t *ctx, uint8_t *digit);
/** ROUTE DEPOT到D1后自动完成四站数字测试。 */
static bool mission_test_run_depot_digits(mission_context_t *ctx);
/** 预装九球后自动逐槽读卡并转到12槽。 */
static bool mission_test_ball_home_load(mission_context_t *ctx);
static bool mission_test_run_depot_balls(mission_context_t *ctx);
/** 等待无线测试动作组完成，STOP或超时不放行后续转盘动作。 */
static bool mission_test_run_arm_group(mission_context_t *ctx, uint8_t group);
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
        ctx->arm_home_ready = false;
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

#if MISSION_CHASSIS_ROUTE_TEST_ENABLED || MISSION_DEPOT_TRACE_ENABLED
    /* 诊断保留原始组号；回调只保存，任务中打印，避免阻塞设备回报。 */
    if ((ctx != NULL) && (report != NULL) &&
        ((report_events & (LSC16_REPORT_EVENT_ACTION_STARTED |
                           LSC16_REPORT_EVENT_ACTION_STOPPED |
                           LSC16_REPORT_EVENT_ACTION_COMPLETED)) != 0U)) {
        ctx->arm_last_action_report =
            ((report_events & 0xFFU) << 8) | report->action_group;
    }
#endif
    if ((ctx != NULL) &&
        ((report_events & (LSC16_REPORT_EVENT_ACTION_STARTED |
                           LSC16_REPORT_EVENT_ACTION_STOPPED |
                           LSC16_REPORT_EVENT_INVALID_FRAME)) != 0U)) {
        ctx->arm_home_ready = false;
    }
    if ((ctx == NULL) || (report == NULL) ||
        (report->action_group != ctx->active_arm_group)) {
        return;
    }
    if ((report_events & (LSC16_REPORT_EVENT_ACTION_STOPPED |
                         LSC16_REPORT_EVENT_INVALID_FRAME)) != 0U) {
        flag = MISSION_FLAG_ARM_FAIL;
    } else if ((report_events & LSC16_REPORT_EVENT_ACTION_COMPLETED) != 0U) {
        ctx->arm_home_ready =
            (report->action_group == MISSION_HOME_ACTION_GROUP) &&
            (ctx->state != MISSION_STATE_STOPPING) &&
            (ctx->state != MISSION_STATE_STOPPED) &&
            (ctx->state != MISSION_STATE_FAULT);
        flag = MISSION_FLAG_ARM_OK;
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
    /* [lyx] 先发布结果待处理标记，再释放在途标记；空闲不等于事务成功。 */
    __DMB();
    ctx->vision.completion_pending = true;
    ctx->vision.inflight = false;
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
        if ((ctx->state == MISSION_STATE_WAIT_HOME) ||
            (ctx->state == MISSION_STATE_WAIT_CHASSIS_READY)) {
            return mission_ms_to_ticks(MISSION_MODEL_QUERY_INTERVAL_MS);
        }
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
    DEPOT_TRACE("[M] STATE=%u D=%u SLOT=%u ARM=%u V=%u\r\n",
                (unsigned)state, (unsigned)ctx->depot_position,
                (unsigned)(ctx->current_slot + 1U),
                (unsigned)ctx->active_arm_group, (unsigned)ctx->vision.phase);
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
    ctx->arm_home_ready = false;
    /* 积木故障时停止在途动作，保留夹持姿态，不自动释放或收臂。 */
    if ((ctx->state >= MISSION_STATE_BLOCK_WAIT_DIGIT) &&
        (ctx->state <= MISSION_STATE_BLOCK_WAIT_HOME)) {
        BLOCK_TRACE("[B] FAULT=%u STATE=%u ROW=%u D%u DIGIT=%u\r\n",
            (unsigned)fault, (unsigned)ctx->state,
            (unsigned)(3U - ctx->block_stage), (unsigned)ctx->depot_position,
            (unsigned)ctx->block_digit);
        (void)arm_stop(NULL, NULL);
        ctx->active_arm_group = 0U;
    }
    DEPOT_TRACE("[M] FAULT=%u STATE=%u ARM_LAST=%lu V=%u IO=%u\r\n",
                (unsigned)fault, (unsigned)ctx->state,
                (unsigned long)ctx->arm_last_action_report,
                (unsigned)ctx->vision.phase, (unsigned)ctx->vision.mail_status);
    /* [lyx] 保留在途缓冲区，事务结束后尝试关闭视觉；故障时同时停车。 */
    (void)mission_stop_vision(ctx);
    (void)turn_stop(NULL, NULL);
    request_id = mission_next_request_id(ctx);
    (void)mission_send_chassis(MISSION_CMD_STOP, request_id);
    mission_enter_state(ctx, MISSION_STATE_FAULT, 0U);
    STAIR_SESSION(ctx, false);
    STAIR_TOTAL(ctx);
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
    ctx->arm_home_ready = false;
    ctx->active_arm_group = action_group;
    DEPOT_TRACE("[M] ARM SEND=%u WAIT=%u\r\n",
                (unsigned)action_group, (unsigned)wait_state);
    if (arm_run(
            action_group,
            1U,
            mission_arm_tx_done,
            ctx) != LSC16_OK) {
        ctx->active_arm_group = 0U;
        return false;
    }
    mission_enter_state(ctx, wait_state, MISSION_OPERATION_TIMEOUT_MS);
    if (action_group == MISSION_PLATFORM_GRASP_GROUP) {
        ++ctx->platform_attempts;
        PLATFORM_TRACE("[P] T=%lu ARM12 SEND TRY=%u OK=%u\r\n",
            (unsigned long)osKernelGetTickCount(), (unsigned)ctx->platform_attempts,
            (unsigned)ctx->platform_balls);
    }
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

    if (ctx->vision.inflight || ctx->vision.completion_pending) {
        return NANO_VISION_ERR_BUSY;
    }
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
    STAIR_SESSION(ctx, false);
    ctx->vision.phase = MISSION_VISION_IDLE;
    ctx->vision.session_id = 0U;
    ctx->vision.scene = NANO_VISION_SCENE_NONE;
    ctx->vision.stop_requested = false;
}

/** 起点状态查询与识别会话互斥，共用单份DMA缓冲区及回调邮箱。 */
static void mission_model_process(mission_context_t *ctx)
{
    size_t tx_len;
    nano_vision_status_t status;
    if (ctx->block_model_ready ||
        ((ctx->state != MISSION_STATE_WAIT_HOME) &&
         (ctx->state != MISSION_STATE_WAIT_CHASSIS_READY)) ||
        (ctx->vision.phase != MISSION_VISION_IDLE) ||
        ctx->vision.inflight || ctx->vision.completion_pending ||
        ((int32_t)(osKernelGetTickCount() - ctx->model_next_query_tick) < 0)) return;
    if (nano_vision_build_model_query_frame(
            mission_next_vision_sequence(&ctx->vision), ctx->vision.tx,
            sizeof(ctx->vision.tx), &tx_len) != NANO_VISION_OK) return;
    ctx->model_next_query_tick = osKernelGetTickCount() +
        mission_ms_to_ticks(MISSION_MODEL_QUERY_INTERVAL_MS);
    ctx->vision.phase = MISSION_VISION_MODEL_QUERYING;
    status = mission_submit_vision_transfer(ctx, MULT_UART_OP_WRITE_READ,
            tx_len, MISSION_VISION_TIMEOUT_MS);
    DEPOT_TRACE("[BOOT] MODEL QUERY SEQ=%u SUBMIT=%u\r\n",
                (unsigned)ctx->vision.next_sequence, (unsigned)status);
    if (status != NANO_VISION_OK) {
        ctx->vision.phase = MISSION_VISION_IDLE;
    }
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
    DEPOT_TRACE("[M] VISION START SID=%u KIND=%u\r\n",
                (unsigned)ctx->vision.session_id, (unsigned)scene);
    if (scene == MISSION_VISION_SCENE_PLATFORM) {
        ctx->vision.scene = NANO_VISION_SCENE_TURNTABLE;
    } else if (scene == MISSION_VISION_SCENE_SMALL_DISC) {
        ctx->vision.scene = NANO_VISION_SCENE_SMALL_DISC;
    } else if (scene == MISSION_VISION_SCENE_DEPOT_DIGIT) {
        ctx->vision.scene = NANO_VISION_SCENE_WAREHOUSE_DIGIT;
    } else if (scene == MISSION_VISION_SCENE_BLOCK_DIGIT) {
        ctx->vision.scene = NANO_VISION_SCENE_BLOCK_DIGIT;
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
    session.target_color = ((scene == MISSION_VISION_SCENE_DEPOT_DIGIT) ||
                            (scene == MISSION_VISION_SCENE_BLOCK_DIGIT)) ?
        NANO_VISION_COLOR_ANY :
        ((g_mission_side == MISSION_COLOR_RED) ?
         NANO_VISION_COLOR_RED : NANO_VISION_COLOR_BLUE);
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
    STAIR_SESSION(ctx, true);
    STAIR_TRACE(ctx, "START SID=%u SCENE=%u COLOR=%u LAYER=%u\r\n",
        (unsigned)session.session_id, (unsigned)session.scene,
        (unsigned)session.target_color, (unsigned)layer);
    /* 仓库切相机不设总期限；START只发一次，后续短读轮询等待同一会话READY。 */
    if (scene == MISSION_VISION_SCENE_DEPOT_DIGIT) {
        ctx->deadline_tick = 0U;
    } else if (scene == MISSION_VISION_SCENE_BLOCK_DIGIT) {
        mission_enter_state(ctx, wait_state, MISSION_BLOCK_PREPARE_TIMEOUT_MS);
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
    if (ctx->vision.inflight || ctx->vision.completion_pending) {
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
    status = mission_submit_vision_transfer(
        ctx, MULT_UART_OP_WRITE_READ, tx_len, MISSION_VISION_TIMEOUT_MS);
    if (status == NANO_VISION_OK) {
        STAIR_TRACE(ctx, "STOP_SENT SID=%u\r\n", (unsigned)ctx->vision.session_id);
    }
    return status == NANO_VISION_OK;
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
    uint16_t emm_speed_rpm,
    zdt_turntable_direction_t direction)
{
    zdt_turntable_position_command_t command = {0};

    command.direction = direction;
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
 * IC最多读五次；圆盘读失败不建档，阶梯/小圆盘仍记录READ_FAILED继续存球。
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
            if (region == MISSION_STORAGE_REGION_STAIR) {
                STAIR_TRACE(ctx, "IC TRY=%u STATUS=%u OK=1 SLOT=%u\r\n",
                    (unsigned)(attempt + 1U), (unsigned)ctx->storage.ic_status,
                    (unsigned)(ctx->storage_slot + 1U));
            }
            break;
        }
        if (region == MISSION_STORAGE_REGION_STAIR) {
            STAIR_TRACE(ctx, "IC TRY=%u STATUS=%u OK=0 SLOT=%u\r\n",
                (unsigned)(attempt + 1U), (unsigned)ctx->storage.ic_status,
                (unsigned)(ctx->storage_slot + 1U));
        }
        if ((attempt + 1U) < MISSION_IC_MAX_ATTEMPTS) {
            (void)osDelay(mission_ms_to_ticks(MISSION_IC_RETRY_MS));
        }
    }
    if (region == MISSION_STORAGE_REGION_PLATFORM) {
        ctx->platform_read_ok = read_ok;
        PLATFORM_TRACE("[P] T=%lu IC=%s TRY=%u READS=%u SLOT=%u TURN=%u\r\n",
            (unsigned long)osKernelGetTickCount(), read_ok ? "OK" : "FAIL",
            (unsigned)ctx->platform_attempts,
            (unsigned)(read_ok ? attempt + 1U : MISSION_IC_MAX_ATTEMPTS),
            (unsigned)(ctx->storage_slot + 1U), (unsigned)read_ok);
        if (!read_ok) return true; /* 可重试的夹空/未读卡，不是设备存储故障。 */
    }
    if (region == MISSION_STORAGE_REGION_STAIR) {
        if (read_ok) STAIR_COUNT(ic_ok); else STAIR_COUNT(ic_failed);
        STAIR_TRACE(ctx, "IC_RESULT=%s SLOT0=%u PHYS=%u\r\n", read_ok ? "VALID" : "READ_FAILED",
            (unsigned)ctx->storage_slot, (unsigned)(ctx->storage_slot + 1U));
    }
    return mission_record_ball(ctx, region, read_ok);
}

/** 所有转槽保留同向PB0校准，微调耗尽后继续；电机/通信故障仍停车。 */
static bool mission_advance_slot(
    mission_context_t *ctx,
    zdt_turntable_direction_t direction,
    uint8_t *fine_used)
{
    const zdt_turntable_response_t *response = &ctx->storage.zdt_response;
    uint32_t started_tick = osKernelGetTickCount();
    uint8_t fine_steps = 0U;
    bool coarse = true;
    zdt_turntable_status_t submitted;
    bool completed;
#if MISSION_CHASSIS_ROUTE_TEST_ENABLED
    char command[DEBUG_UART1_RX_BUFFER_SIZE];
#else
    mission_user_command_t command;
#endif

    for (;;) {
#if !MISSION_CHASSIS_ROUTE_TEST_ENABLED
        /* [lyx] 同步单槽事务间仍响应正式STOP，不依赖无线命令轮询。 */
        while (osMessageQueueGet(ctx->command_queue, &command, NULL, 0U) == osOK) {
            mission_handle_command(ctx, command);
        }
        if ((ctx->state == MISSION_STATE_STOPPING) ||
            (ctx->state == MISSION_STATE_STOPPED) ||
            (ctx->state == MISSION_STATE_FAULT)) return false;
#endif
#if MISSION_CHASSIS_ROUTE_TEST_ENABLED
        if (mission_test_take_command(command, sizeof(command)) &&
            !mission_test_handle_aux_command(ctx, command)) {
            mission_test_write("BUSY\r\n");
        }
        if (g_wireless_test.stop_requested) return false;
#endif
        (void)osThreadFlagsClear(MISSION_FLAG_ZDT_DONE);
        ctx->storage.zdt_has_response = false;
        submitted = mission_submit_slot_motion(
                 ctx,
                 coarse ? ((direction == ZDT_TURNTABLE_DIR_CCW) ?
                     MISSION_ZDT_REVERSE_COARSE_ANGLE_0P1DEG :
                     MISSION_ZDT_COARSE_ANGLE_0P1DEG) :
                          MISSION_ZDT_FINE_ANGLE_0P1DEG,
                 coarse ? MISSION_ZDT_SPEED_RPM :
                          MISSION_ZDT_FINE_SPEED_RPM,
                 direction);
        completed = (submitted == ZDT_TURNTABLE_OK) && mission_wait_zdt(ctx);
        if (!completed ||
            ((response->kind != ZDT_TURNTABLE_REPLY_ACK) &&
             (response->kind != ZDT_TURNTABLE_REPLY_REACHED))) {
            DEPOT_TRACE("[M] TURN FAIL=MOVE SLOT=%u DIR=%u FINE=%u SUBMIT=%u IO=%u HAS=%u KIND=%u\r\n",
                (unsigned)(ctx->current_slot + 1U), (unsigned)direction,
                (unsigned)fine_steps, (unsigned)submitted,
                (unsigned)ctx->storage.zdt_status, (unsigned)ctx->storage.zdt_has_response,
                ctx->storage.zdt_has_response ? (unsigned)response->kind : 0U);
            return false;
        }
        coarse = false;

        do {
#if !MISSION_CHASSIS_ROUTE_TEST_ENABLED
            while (osMessageQueueGet(ctx->command_queue, &command, NULL, 0U) == osOK) {
                mission_handle_command(ctx, command);
            }
            if ((ctx->state == MISSION_STATE_STOPPING) ||
                (ctx->state == MISSION_STATE_STOPPED) ||
                (ctx->state == MISSION_STATE_FAULT)) return false;
#endif
#if MISSION_CHASSIS_ROUTE_TEST_ENABLED
            if (mission_test_take_command(command, sizeof(command)) &&
                !mission_test_handle_aux_command(ctx, command)) {
                mission_test_write("BUSY\r\n");
            }
            if (g_wireless_test.stop_requested) return false;
#endif
            if ((osKernelGetTickCount() - started_tick) >=
                mission_ms_to_ticks(MISSION_ZDT_SLOT_TIMEOUT_MS)) {
                DEPOT_TRACE("[M] TURN FAIL=TIMEOUT SLOT=%u DIR=%u FINE=%u\r\n",
                    (unsigned)(ctx->current_slot + 1U), (unsigned)direction,
                    (unsigned)fine_steps);
                return false;
            }
            (void)osDelay(mission_ms_to_ticks(MISSION_ZDT_STATUS_POLL_MS));
            (void)osThreadFlagsClear(MISSION_FLAG_ZDT_DONE);
            ctx->storage.zdt_has_response = false;
            submitted = turn_query_status(mission_zdt_done, ctx);
            completed = (submitted == ZDT_TURNTABLE_OK) && mission_wait_zdt(ctx);
            if (!completed || (response->kind != ZDT_TURNTABLE_REPLY_STATUS)) {
                DEPOT_TRACE("[M] TURN FAIL=STATUS SLOT=%u DIR=%u FINE=%u SUBMIT=%u IO=%u HAS=%u KIND=%u\r\n",
                    (unsigned)(ctx->current_slot + 1U), (unsigned)direction,
                    (unsigned)fine_steps, (unsigned)submitted,
                    (unsigned)ctx->storage.zdt_status, (unsigned)ctx->storage.zdt_has_response,
                    ctx->storage.zdt_has_response ? (unsigned)response->kind : 0U);
                return false;
            }
            if (!response->data.motor_status.enabled ||
                response->data.motor_status.stalled ||
                response->data.motor_status.stall_protected ||
                response->data.motor_status.power_loss_latched) {
                DEPOT_TRACE("[M] TURN FAIL=MOTOR SLOT=%u EN=%u STALL=%u PROTECT=%u POWER=%u\r\n",
                    (unsigned)(ctx->current_slot + 1U),
                    (unsigned)response->data.motor_status.enabled,
                    (unsigned)response->data.motor_status.stalled,
                    (unsigned)response->data.motor_status.stall_protected,
                    (unsigned)response->data.motor_status.power_loss_latched);
                return false;
            }
        } while (!response->data.motor_status.reached);

        if (mission_gate_is_stably_high()) {
            if (fine_used != NULL) *fine_used = fine_steps;
            return true;
        }
        if (fine_steps >= MISSION_ZDT_FINE_MAX_STEPS) {
            if (fine_used != NULL) *fine_used = fine_steps;
            DEPOT_TRACE("[M] TURN PB0 UNCONFIRMED SLOT=%u DIR=%u FINE=%u CONTINUE\r\n",
                (unsigned)(ctx->current_slot + 1U), (unsigned)direction,
                (unsigned)fine_steps);
#if MISSION_CHASSIS_ROUTE_TEST_ENABLED
            mission_test_write("TURN PB0 UNCONFIRMED FINE LIMIT CONTINUE\r\n");
#endif
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
    if (!mission_read_ball(ctx, region)) return false;
    if (region == MISSION_STORAGE_REGION_STAIR) {
        STAIR_TRACE(ctx, "TURN FROM0=%u PHYS=%u DIR=%u\r\n", (unsigned)ctx->current_slot,
            (unsigned)(ctx->current_slot + 1U), (unsigned)MISSION_SLOT_USE_CW);
    }
    if ((region == MISSION_STORAGE_REGION_PLATFORM) && !ctx->platform_read_ok) {
        return true; /* 仅圆盘保留同一空槽，其他区域行为不变。 */
    }
    /* 圆盘/阶梯/绕桩均保留微调，达到上限后不再强制等待PB0。 */
    if (!mission_advance_slot(ctx,
            MISSION_SLOT_USE_CW ? ZDT_TURNTABLE_DIR_CW :
                                  ZDT_TURNTABLE_DIR_CCW, NULL)) return false;
    /* 初始物理1槽对应0；PB0可选时，当前位置按完成的一槽运动推定。 */
    ctx->current_slot = MISSION_SLOT_USE_CW ?
        (uint8_t)((ctx->current_slot + 1U) % 12U) :
        (uint8_t)((ctx->current_slot + 11U) % 12U);
    if (region == MISSION_STORAGE_REGION_STAIR) {
        STAIR_TRACE(ctx, "TURN_DONE PB0=OPTIONAL SLOT0=%u PHYS=%u\r\n",
            (unsigned)ctx->current_slot, (unsigned)(ctx->current_slot + 1U));
    }
    return true;
}

/** 上电只查询一次转盘固件和闭环配置，后续存球直接使用缓存结果。 */
static bool mission_prepare_zdt(mission_context_t *ctx)
{
    const zdt_turntable_response_t *response = &ctx->storage.zdt_response;
    zdt_turntable_status_t status;
    bool completed;

    (void)osThreadFlagsClear(MISSION_FLAG_ZDT_DONE);
    ctx->storage.zdt_has_response = false;
    status = turn_query_options(mission_zdt_done, ctx);
    DEPOT_TRACE("[BOOT] ZDT OPTIONS QUERY SUBMIT=%u\r\n", (unsigned)status);
    if (status != ZDT_TURNTABLE_OK) return false;
    completed = mission_wait_zdt(ctx);
    DEPOT_TRACE("[BOOT] ZDT OPTIONS RX DONE=%u IO=%u HAS=%u KIND=%u\r\n",
                (unsigned)completed, (unsigned)ctx->storage.zdt_status,
                (unsigned)ctx->storage.zdt_has_response,
                ctx->storage.zdt_has_response ? (unsigned)response->kind : 0U);
    if (!completed || (response->kind != ZDT_TURNTABLE_REPLY_OPTIONS)) return false;
    /* 只在正确回复类型下解释options联合体；打印仍不参与就绪判定。 */
    DEPOT_TRACE("[BOOT] ZDT OPTIONS CLOSED=%u FW=%u\r\n",
                (unsigned)response->data.options.closed_loop,
                (unsigned)response->data.options.firmware);
    if (!response->data.options.closed_loop ||
        (response->data.options.firmware != ZDT_TURNTABLE_FIRMWARE_EMM)) {
        return false;
    }
    return true;
}

/** 本方物理D号不倒换底盘命令；空列用途为红D4、蓝D1。 */
static uint8_t mission_block_place_point(void)
{
    return (g_mission_side == MISSION_COLOR_BLUE) ? 1U : 4U;
}

/** 蓝方三层均从D2正向扫描；红方保留高层正向、中底层反向。 */
static uint8_t mission_block_scan_point_id(uint8_t stage, uint8_t step)
{
    if (g_mission_side == MISSION_COLOR_BLUE) return (uint8_t)(step + 2U);
    return (stage == 0U) ? (uint8_t)(step + 1U) : (uint8_t)(3U - step);
}

/** 单次积木动作仍走既有完成回报；日志不参与到位判定。 */
static void mission_block_run_arm(mission_context_t *ctx, uint8_t group,
                                  mission_state_t wait_state)
{
    BLOCK_TRACE("[B] ARM START=%u ROW=%u D%u TARGET_ROW=%u\r\n",
        (unsigned)group, (unsigned)(3U - ctx->block_stage),
        (unsigned)ctx->depot_position, (unsigned)ctx->block_digit);
    if (!mission_start_arm(ctx, group, wait_state))
        mission_fail(ctx, MISSION_FAULT_ARM);
}

static void mission_block_start_layer(mission_context_t *ctx)
{
    static const uint8_t groups[] = {MISSION_BLOCK_HIGH_VISION_GROUP,
        MISSION_BLOCK_MID_VISION_GROUP, MISSION_BLOCK_LOW_VISION_GROUP};
    ctx->block_step = 0U;
    ctx->block_digit = 0U;
    mission_block_run_arm(ctx, groups[ctx->block_stage], MISSION_STATE_BLOCK_WAIT_POSE);
}

/** 放置完成或本层三个点均未确认后，才允许进入下一源层。 */
static void mission_block_next_layer(mission_context_t *ctx)
{
    if (ctx->block_stage < 2U) {
        ++ctx->block_stage;
        mission_block_start_layer(ctx);
    } else mission_block_run_arm(ctx, MISSION_HOME_ACTION_GROUP, MISSION_STATE_BLOCK_FINISH);
}

static void mission_block_position_done(mission_context_t *ctx)
{
    BLOCK_TRACE("[B] REACHED D%u ROW=%u TARGET_ROW=%u\r\n",
        (unsigned)ctx->depot_position, (unsigned)(3U - ctx->block_stage),
        (unsigned)ctx->block_digit);
    if (ctx->state == MISSION_STATE_BLOCK_WAIT_POSITION) {
        mission_enter_state(ctx, MISSION_STATE_BLOCK_SETTLE, MISSION_BLOCK_SETTLE_MS);
    } else if (ctx->state == MISSION_STATE_BLOCK_WAIT_D4) {
        if (ctx->block_digit != 0U) {
            static const uint8_t groups[] = {MISSION_BLOCK_LOW_PLACE_GROUP,
                MISSION_BLOCK_MID_PLACE_GROUP, MISSION_BLOCK_HIGH_PLACE_GROUP};
            /* block_digit在有效结果确认时限定1～3，与源层无关。 */
            mission_block_run_arm(ctx, groups[ctx->block_digit - 1U],
                                  MISSION_STATE_BLOCK_WAIT_PLACE);
        } else mission_block_next_layer(ctx);
    } else if (ctx->state == MISSION_STATE_BLOCK_RETURN_DEPOT) {
        BLOCK_TRACE("[B] DONE FOUND=0x%02X PLACED=0x%02X AT_D%u\r\n",
            (unsigned)ctx->block_found_mask, (unsigned)ctx->block_placed_mask,
            (unsigned)ctx->depot_position);
#if MISSION_CHASSIS_ROUTE_TEST_ENABLED
        /* 新无线测试不放小球；D1实际停车后才允许回家。 */
        if (!mission_send_chassis(MISSION_CMD_DEPOT_OK, mission_next_request_id(ctx))) {
            mission_fail(ctx, MISSION_FAULT_QUEUE);
            return;
        }
        mission_enter_state(ctx, MISSION_STATE_BLOCK_WAIT_HOME, MISSION_OPERATION_TIMEOUT_MS);
#else
        /* 正式红D1/蓝D2入口已经实际到位；独立状态防止再进积木。 */
        mission_depot_prepare(ctx);
#endif
    }
}

/** 复用仓库双向横移；当前位置只在匹配到位回报后更新。 */
static void mission_block_move(mission_context_t *ctx, uint8_t point,
                               mission_state_t wait_state)
{
    ctx->block_point = point;
    if (ctx->depot_position == point) {
        mission_enter_state(ctx, wait_state, 0U);
        mission_block_position_done(ctx);
        return;
    }
    BLOCK_TRACE("[B] MOVE D%u->D%u ROW=%u TARGET_ROW=%u\r\n",
        (unsigned)ctx->depot_position, (unsigned)point,
        (unsigned)(3U - ctx->block_stage), (unsigned)ctx->block_digit);
    if (!mission_send_chassis((mission_command_type_t)(MISSION_CMD_GO_DEPOT_1 + point - 1U),
                              mission_next_request_id(ctx))) {
        mission_fail(ctx, MISSION_FAULT_QUEUE);
        return;
    }
    mission_enter_state(ctx, wait_state, MISSION_OPERATION_TIMEOUT_MS);
}

static void mission_block_scan_point(mission_context_t *ctx)
{
    uint8_t point = mission_block_scan_point_id(ctx->block_stage, ctx->block_step);
    mission_block_move(ctx, point, MISSION_STATE_BLOCK_WAIT_POSITION);
}

static void mission_block_arm_done(mission_context_t *ctx)
{
    BLOCK_TRACE("[B] ARM DONE=%u ROW=%u D%u TARGET_ROW=%u\r\n",
        (unsigned)ctx->active_arm_group, (unsigned)(3U - ctx->block_stage),
        (unsigned)ctx->depot_position, (unsigned)ctx->block_digit);
    ctx->active_arm_group = 0U; /* 本组已消费，重复旧组回报不能再次推进。 */
    switch (ctx->state) {
    case MISSION_STATE_BLOCK_PREPARE:
        mission_block_start_layer(ctx);
        break;
    case MISSION_STATE_BLOCK_WAIT_POSE:
        mission_block_scan_point(ctx);
        break;
    case MISSION_STATE_BLOCK_WAIT_GRASP:
        /* 完成夹取保持姿态到本方空列：红D4、蓝D1；数字只指定目标层。 */
        mission_block_move(ctx, mission_block_place_point(), MISSION_STATE_BLOCK_WAIT_D4);
        break;
    case MISSION_STATE_BLOCK_WAIT_PLACE:
        ctx->block_placed_mask |= (uint8_t)(1U << (2U - ctx->block_stage));
        mission_block_next_layer(ctx);
        break;
    case MISSION_STATE_BLOCK_FINISH:
        /* 蓝方正式从D1去有数字的D2；无线两方仍到D1回家、不放小球。 */
        mission_block_move(ctx,
#if MISSION_CHASSIS_ROUTE_TEST_ENABLED
            1U,
#else
            (g_mission_side == MISSION_COLOR_BLUE) ? 2U : 1U,
#endif
            MISSION_STATE_BLOCK_RETURN_DEPOT);
        break;
    default:
        break;
    }
}

static void mission_block_begin(mission_context_t *ctx)
{
    ctx->block_stage = ctx->block_step = ctx->block_digit = 0U;
    ctx->block_found_mask = ctx->block_placed_mask = 0U;
    ctx->block_result_received = false;
    ctx->depot_position = 1U; /* 调用入口已经确认首次D1到位。 */
    BLOCK_TRACE("[B] BEGIN SIDE=%u AT_D1 PLACE_D%u\r\n",
        (unsigned)g_mission_side, (unsigned)mission_block_place_point());
    if (ctx->arm_home_ready) mission_block_start_layer(ctx);
    else mission_block_run_arm(ctx, MISSION_HOME_ACTION_GROUP, MISSION_STATE_BLOCK_PREPARE);
}

/** 仅在当前结果ACK发送成功且事务邮箱已消费后调用。 */
static void mission_block_digit_done(mission_context_t *ctx)
{
    static const uint8_t groups[] = {MISSION_BLOCK_HIGH_GRASP_GROUP,
        MISSION_BLOCK_MID_GRASP_GROUP, MISSION_BLOCK_LOW_GRASP_GROUP};
    const nano_vision_block_result_t *result = &ctx->block_result;
    ctx->block_result_received = false;
    BLOCK_TRACE("[B] ROW=%u D%u DIGIT=%u SID=%u STATUS=%u REASON=%u FRAMES=%u\r\n",
        (unsigned)(3U - ctx->block_stage), (unsigned)ctx->depot_position,
        (unsigned)result->digit, (unsigned)result->session_id,
        (unsigned)result->status, (unsigned)result->reason, (unsigned)result->frames);
    if ((result->status == NANO_VISION_BLOCK_DIGIT) &&
        (result->digit >= 1U) && (result->digit <= 3U)) {
        ctx->block_digit = result->digit;
        ctx->block_found_mask |= (uint8_t)(1U << (2U - ctx->block_stage));
        mission_block_run_arm(ctx, groups[ctx->block_stage], MISSION_STATE_BLOCK_WAIT_GRASP);
    } else if (result->status == NANO_VISION_BLOCK_NO_VALID) {
        if (++ctx->block_step < 3U) mission_block_scan_point(ctx);
        else {
            BLOCK_TRACE("[B] ROW=%u NOT_FOUND TO_D%u\r\n",
                (unsigned)(3U - ctx->block_stage), (unsigned)mission_block_place_point());
            mission_block_move(ctx, mission_block_place_point(), MISSION_STATE_BLOCK_WAIT_D4);
        }
    } else mission_fail(ctx, MISSION_FAULT_VISION);
}

/**
 * @brief 解析Nano事务结果并推进当前视觉会话。
 * @param ctx Mission上下文。
 * @note 仅在Mission任务中调用；设备回调只负责保存数据并唤醒任务。
 */
/** 积木只在专属等待状态消费终态；旧SID/READY不能改变当前点的计时或结果。 */
static void mission_handle_block_result(mission_context_t *ctx)
{
    nano_vision_session_t ready;
    nano_vision_block_result_t result;
    nano_vision_event_ack_t ack;
    size_t tx_len;
    if (ctx->state != MISSION_STATE_BLOCK_WAIT_DIGIT) return;
    if ((ctx->vision.phase == MISSION_VISION_STARTING) &&
        (nano_vision_decode_session_ready(ctx->vision.mail_data,
            ctx->vision.mail_len, &ready) == NANO_VISION_OK)) {
        if ((ready.session_id != ctx->vision.session_id) ||
            (ready.scene != NANO_VISION_SCENE_BLOCK_DIGIT) ||
            (ready.target_color != NANO_VISION_COLOR_ANY)) return;
        ctx->vision.phase = MISSION_VISION_LISTENING;
        mission_enter_state(ctx, MISSION_STATE_BLOCK_WAIT_DIGIT,
                            MISSION_BLOCK_RESULT_TIMEOUT_MS);
        return;
    }
    if ((nano_vision_decode_block_result(ctx->vision.mail_data,
            ctx->vision.mail_len, &result) != NANO_VISION_OK) ||
        (result.session_id != ctx->vision.session_id)) return;
    /* 开相机或模型失败可能在READY之前返回FAULT，不能将其当作空仓。 */
    if ((ctx->vision.phase == MISSION_VISION_STARTING) &&
        (result.status != NANO_VISION_BLOCK_FAULT)) return;
    if ((result.status != NANO_VISION_BLOCK_FAULT) &&
        (result.age_ms > MISSION_VISION_EVENT_MAX_AGE_MS)) return;
    ack.session_id = result.session_id;
    ack.frame_id = result.frame_id;
    if (nano_vision_build_event_ack_frame(
            mission_next_vision_sequence(&ctx->vision), &ack,
            ctx->vision.tx, sizeof(ctx->vision.tx), &tx_len) != NANO_VISION_OK) {
        mission_fail(ctx, MISSION_FAULT_VISION);
        return;
    }
    ctx->block_result = result;
    ctx->block_result_received = true;
    ctx->vision.phase = MISSION_VISION_ACKING;
    if (mission_submit_vision_transfer(ctx, MULT_UART_OP_WRITE, tx_len,
            MISSION_VISION_TIMEOUT_MS) != NANO_VISION_OK)
        mission_fail(ctx, MISSION_FAULT_VISION);
}

static void mission_handle_vision(mission_context_t *ctx)
{
    nano_vision_status_t status;
    nano_vision_session_t session;
    nano_vision_event_t event;
    nano_vision_digit_event_t digit_event;
    nano_vision_event_ack_t ack;
    size_t tx_len = 0U;

    /* 0) 先处理等待当前串口事务结束后才能执行的停止请求。 */
    if (!ctx->vision.completion_pending) return;
    ctx->vision.completion_pending = false;
    status = mission_map_vision_status(ctx->vision.mail_status);
    if (ctx->vision.phase == MISSION_VISION_MODEL_QUERYING) {
        nano_vision_model_report_t report;
        nano_vision_status_t decoded;
        ctx->vision.phase = MISSION_VISION_IDLE;
        DEPOT_TRACE("[BOOT] MODEL RX IO=%u LEN=%u TYPE=%u SEQ=%u EXPECT=%u\r\n",
            (unsigned)status, (unsigned)ctx->vision.mail_len,
            (ctx->vision.mail_len > 3U) ? (unsigned)ctx->vision.mail_data[3] : 0U,
            (ctx->vision.mail_len > 4U) ? (unsigned)ctx->vision.mail_data[4] : 0U,
            (unsigned)ctx->vision.next_sequence);
        /* 仅起点查询容错；STOP/故障或已出发后，迟到回复不得重新放行。 */
        if ((ctx->state != MISSION_STATE_WAIT_HOME) &&
            (ctx->state != MISSION_STATE_WAIT_CHASSIS_READY)) return;
        if (status != NANO_VISION_OK) {
            ctx->block_model_ready = false;
            ++ctx->model_query_retries;
            DEPOT_TRACE("[BOOT] MODEL RETRY=%lu IO=%u RAW=%u\r\n",
                (unsigned long)ctx->model_query_retries, (unsigned)status,
                (unsigned)ctx->vision.mail_status);
            return;
        }
        decoded = nano_vision_decode_model_state(ctx->vision.mail_data,
                ctx->vision.mail_len, &report);
        DEPOT_TRACE("[BOOT] MODEL DECODE=%u\r\n", (unsigned)decoded);
        if ((decoded != NANO_VISION_OK) ||
            (ctx->vision.mail_data[4] != ctx->vision.next_sequence)) {
            ctx->block_model_ready = false;
            ++ctx->model_query_retries;
            DEPOT_TRACE("[BOOT] MODEL RETRY=%lu DECODE=%u SEQ=%u EXPECT=%u\r\n",
                (unsigned long)ctx->model_query_retries, (unsigned)decoded,
                (ctx->vision.mail_len > 4U) ? (unsigned)ctx->vision.mail_data[4] : 0U,
                (unsigned)ctx->vision.next_sequence);
            return;
        }
        ctx->model_query_retries = 0U;
        DEPOT_TRACE("[BOOT] MODEL STATE=%u REASON=%u\r\n",
                    (unsigned)report.state, (unsigned)report.reason);
        if (report.state == NANO_VISION_MODEL_ERROR) {
            mission_fail(ctx, MISSION_FAULT_VISION);
            return;
        }
        ctx->block_model_ready = report.state == NANO_VISION_MODEL_READY;
#if !MISSION_CHASSIS_ROUTE_TEST_ENABLED
        mission_try_ready(ctx);
#endif
        return;
    }
    if ((ctx->vision.phase == MISSION_VISION_STARTING) &&
        ((ctx->vision.scene == NANO_VISION_SCENE_WAREHOUSE_DIGIT) ||
         (ctx->vision.scene == NANO_VISION_SCENE_BLOCK_DIGIT)) &&
        !ctx->vision.stop_requested &&
        (status == NANO_VISION_ERR_TIMEOUT)) {
        /* 切换C100可能耗时较长；超时仅释放本笔总线事务，不重发START。 */
        return;
    }
    /* 正常监听超时不刷屏；保留会话应答、数字帧和通信错误。 */
    if ((ctx->depot_position != 0U) &&
        !((ctx->vision.phase == MISSION_VISION_LISTENING) &&
          (status == NANO_VISION_ERR_TIMEOUT))) {
        DEPOT_TRACE("[M] VISION RX PH=%u STATUS=%u LEN=%u SID=%u\r\n",
                    (unsigned)ctx->vision.phase, (unsigned)status,
                    (unsigned)ctx->vision.mail_len, (unsigned)ctx->vision.session_id);
    }
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
#if MISSION_DEPOT_TRACE_ENABLED && !MISSION_CHASSIS_ROUTE_TEST_ENABLED
            bool stair = mission_stair_trace_active(ctx);
#endif
            mission_fail(ctx, MISSION_FAULT_VISION);
#if MISSION_DEPOT_TRACE_ENABLED && !MISSION_CHASSIS_ROUTE_TEST_ENABLED
            if (stair) DEPOT_TRACE("[S] STOPPED_REJECT DECODE=%u SID=%u\r\n",
                (unsigned)status, (unsigned)stopped_session);
#endif
            return;
        }
        STAIR_TRACE(ctx, "STOPPED SID=%u\r\n", (unsigned)stopped_session);
        mission_reset_vision(ctx);
        if (ctx->state == MISSION_STATE_DEPOT_DIGIT_STOP) {
            mission_fail(ctx, MISSION_FAULT_VISION);
            return;
        }
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
    if ((ctx->vision.scene == NANO_VISION_SCENE_BLOCK_DIGIT) &&
        ((ctx->vision.phase == MISSION_VISION_STARTING) ||
         (ctx->vision.phase == MISSION_VISION_LISTENING))) {
        mission_handle_block_result(ctx);
        return;
    }
    if (ctx->vision.phase == MISSION_VISION_STARTING) {
        status = nano_vision_decode_session_ready(
            ctx->vision.mail_data, ctx->vision.mail_len, &session);
        if ((status != NANO_VISION_OK) ||
            (session.session_id != ctx->vision.session_id) ||
            (session.scene != ctx->vision.scene) ||
            (session.target_color !=
             ((ctx->vision.scene == NANO_VISION_SCENE_WAREHOUSE_DIGIT) ?
              NANO_VISION_COLOR_ANY :
              ((g_mission_side == MISSION_COLOR_RED) ?
               NANO_VISION_COLOR_RED : NANO_VISION_COLOR_BLUE)))) {
#if MISSION_DEPOT_TRACE_ENABLED && !MISSION_CHASSIS_ROUTE_TEST_ENABLED
            bool stair = mission_stair_trace_active(ctx);
#endif
            mission_fail(ctx, MISSION_FAULT_VISION);
#if MISSION_DEPOT_TRACE_ENABLED && !MISSION_CHASSIS_ROUTE_TEST_ENABLED
            /* 解码失败不读取未初始化字段；DECODE非0时后三个0仅为占位。 */
            if (stair) DEPOT_TRACE("[S] READY_REJECT DECODE=%u SID=%u SCENE=%u COLOR=%u\r\n",
                (unsigned)status, status == NANO_VISION_OK ? (unsigned)session.session_id : 0U,
                status == NANO_VISION_OK ? (unsigned)session.scene : 0U,
                status == NANO_VISION_OK ? (unsigned)session.target_color : 0U);
#endif
            return;
        }
        ctx->vision.phase = MISSION_VISION_LISTENING;
        if (ctx->state == MISSION_STATE_PLATFORM_WAIT_VISION) {
            PLATFORM_TRACE("[P] T=%lu READY SID=%u TRY=%u OK=%u\r\n",
                (unsigned long)osKernelGetTickCount(), (unsigned)ctx->vision.session_id,
                (unsigned)ctx->platform_attempts, (unsigned)ctx->platform_balls);
        }
        DEPOT_TRACE("[M] VISION READY SID=%u SCENE=%u\r\n",
                    (unsigned)ctx->vision.session_id, (unsigned)ctx->vision.scene);
        /* 仓库位已停车，不需要向底盘发送扫描放行命令。 */
        if (ctx->state == MISSION_STATE_DEPOT_WAIT_DIGIT) {
#if !MISSION_CHASSIS_ROUTE_TEST_ENABLED
            mission_enter_state(ctx, MISSION_STATE_DEPOT_WAIT_DIGIT,
                                MISSION_DEPOT_DIGIT_WAIT_MS);
#endif
            return;
        }
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
        STAIR_TRACE(ctx, "READY SID=%u SCENE=%u STATE=%u ID=%u\r\n",
            (unsigned)ctx->vision.session_id, (unsigned)ctx->vision.scene,
            (unsigned)ctx->state, (unsigned)ctx->request_id);
        return;
    }
    /* 5) 小球只在对应扫描状态接收；仓库数字只在停车位接收。 */
    if ((ctx->vision.phase != MISSION_VISION_LISTENING) ||
        ((ctx->state != MISSION_STATE_PLATFORM_WAIT_TARGET) &&
         (ctx->state != MISSION_STATE_STAIR_SCANNING) &&
         (ctx->state != MISSION_STATE_SMALL_DISC_RUNNING) &&
         (ctx->state != MISSION_STATE_DEPOT_WAIT_DIGIT))) {
        STAIR_STATE_REJECT();
        return;
    }
    if (ctx->state == MISSION_STATE_DEPOT_WAIT_DIGIT) {
        /* 数字只接受本次会话的新鲜1~3事件，确认后Nano自动关闭会话。 */
        status = nano_vision_decode_digit_event(
            ctx->vision.mail_data, ctx->vision.mail_len, &digit_event);
        if ((status != NANO_VISION_OK) ||
            (digit_event.session_id != ctx->vision.session_id) ||
            (digit_event.age_ms > MISSION_VISION_EVENT_MAX_AGE_MS)) {
            DEPOT_TRACE("[M] DIGIT REJECT DECODE=%u\r\n", (unsigned)status);
            return;
        }
        DEPOT_TRACE("[M] DIGIT RX=%u SID=%u AGE=%u\r\n",
                    (unsigned)digit_event.digit, (unsigned)digit_event.session_id,
                    (unsigned)digit_event.age_ms);
        ack.session_id = ctx->vision.session_id;
        ack.frame_id = digit_event.frame_id;
        status = nano_vision_build_event_ack_frame(
            mission_next_vision_sequence(&ctx->vision), &ack,
            ctx->vision.tx, sizeof(ctx->vision.tx), &tx_len);
        if (status != NANO_VISION_OK) {
            mission_fail(ctx, MISSION_FAULT_VISION);
            return;
        }
        ctx->vision.phase = MISSION_VISION_ACKING;
        if (mission_submit_vision_transfer(
                ctx, MULT_UART_OP_WRITE, tx_len,
                MISSION_VISION_TIMEOUT_MS) != NANO_VISION_OK) {
            mission_fail(ctx, MISSION_FAULT_VISION);
            return;
        }
#if MISSION_CHASSIS_ROUTE_TEST_ENABLED
        g_wireless_test.depot_digit = digit_event.digit;
#else
        ctx->depot_digit = digit_event.digit;
        /* [lyx] 数字已收到，继续等待ACK事务成功，不能提前开始下一会话。 */
        mission_enter_state(ctx, MISSION_STATE_DEPOT_WAIT_DIGIT,
                            MISSION_OPERATION_TIMEOUT_MS);
#endif
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
        STAIR_REJECT(ctx, status, &event);
        return;
    }
    if (ctx->state == MISSION_STATE_STAIR_SCANNING) STAIR_ACCEPT();
    /* 6) 先确认该视觉帧；运动中的阶梯和小圆盘还要请求底盘停车。 */
    if (ctx->state == MISSION_STATE_PLATFORM_WAIT_TARGET) {
        PLATFORM_TRACE("[P] T=%lu EVENT SID=%u FRAME=%u AGE=%u DX=%d DY=%d\r\n",
            (unsigned long)osKernelGetTickCount(), (unsigned)event.session_id,
            (unsigned)event.observation.frame_id, (unsigned)event.observation.age_ms,
            (int)event.observation.offset_x_px, (int)event.observation.offset_y_px);
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
        STAIR_TRACE(ctx, "EVENT SID=%u FRAME=%u SCENE=%u COLOR=%u AGE=%u ACK_SUBMIT STOP_SENT\r\n",
            (unsigned)event.session_id, (unsigned)event.observation.frame_id,
            (unsigned)event.observation.scene, (unsigned)event.observation.color,
            (unsigned)event.observation.age_ms);
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
        ctx->vision.inflight || ctx->vision.completion_pending) {
        return;
    }
    if ((ctx->vision.phase == MISSION_VISION_LISTENING) ||
        ((ctx->vision.phase == MISSION_VISION_STARTING) &&
         ((ctx->vision.scene == NANO_VISION_SCENE_WAREHOUSE_DIGIT) ||
          (ctx->vision.scene == NANO_VISION_SCENE_BLOCK_DIGIT)))) {
        if (mission_submit_vision_transfer(
                ctx, MULT_UART_OP_READ, 0U,
                MISSION_VISION_READ_TIMEOUT_MS) != NANO_VISION_OK) {
            return;
        }
    } else if (ctx->vision.phase == MISSION_VISION_ACKING) {
        DEPOT_TRACE("[M] ACK DONE STATE=%u DIGIT=%u\r\n",
                    (unsigned)ctx->state, (unsigned)ctx->depot_digit);
        STAIR_TRACE(ctx, "ACK_DONE SID=%u STATE=%u\r\n",
            (unsigned)ctx->vision.session_id, (unsigned)ctx->state);
        mission_reset_vision(ctx);
#if !MISSION_CHASSIS_ROUTE_TEST_ENABLED
        if (ctx->state == MISSION_STATE_DEPOT_WAIT_DIGIT) {
            mission_depot_digit_done(ctx);
            return;
        }
#endif
        if ((ctx->state == MISSION_STATE_BLOCK_WAIT_DIGIT) &&
            ctx->block_result_received) {
            /* ACK事务已被消费，缓冲区可复用；正式/无线搬运才在此推进。 */
#if MISSION_CHASSIS_ROUTE_TEST_ENABLED
            if (g_wireless_test.block_move) mission_block_digit_done(ctx);
#else
            mission_block_digit_done(ctx);
#endif
            return;
        }
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
 * @note READY保持约4秒后按已选颜色启动；底盘此时阻塞等待GO_PLATFORM。
 */
static void mission_try_ready(mission_context_t *ctx)
{
    if ((ctx->state != MISSION_STATE_WAIT_CHASSIS_READY) ||
        !ctx->chassis_ready ||
        !ctx->arm_home_ready || !ctx->block_model_ready ||
        (g_mission_side == MISSION_COLOR_NONE)) {
        return;
    }
    if (!mission_send_chassis(MISSION_CMD_MISSION_READY, ctx->request_id)) {
        mission_fail(ctx, MISSION_FAULT_QUEUE);
        return;
    }
    DEPOT_TRACE("[BOOT] GATE READY ARM10=1 CHASSIS=1 MODEL=1 START_IN=%uMS\r\n",
                (unsigned)MISSION_AUTO_START_DELAY_MS);
    mission_enter_state(ctx, MISSION_STATE_READY,
                        MISSION_AUTO_START_DELAY_MS);
}

/**
 * @brief 启动一轮正式任务并请求底盘前往圆盘工作位。
 * @param ctx Mission上下文。
 * @note 颜色已由假按钮选定，启动时不再覆盖；后续可接入实体按钮。
 */
static void mission_start_run(mission_context_t *ctx)
{
    /* 0) 新一轮任务使用新的request_id，隔离上一轮底盘回包。 */
    uint16_t request_id = mission_next_request_id(ctx);

    DEPOT_TRACE("[BOOT] AUTO START SIDE=%u GO_PLATFORM ID=%u\r\n",
                (unsigned)g_mission_side, (unsigned)request_id);
    /* 1) 保留已选红蓝方，只清空本轮小球、槽位和故障计数。 */
    ctx->platform_balls = 0U;
    ctx->platform_attempts = 0U;
    ctx->platform_read_ok = false;
    ctx->stair_balls = 0U;
    ctx->small_disc_balls = 0U;
    ctx->storage_slot = 0U;
    ctx->current_slot = 0U; /* 初始物理1槽须在开赛前人工对准，不是机械归零。 */
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
        STAIR_TRACE(ctx, "SKIP_LIMIT LAYER=%u COUNT=%u\r\n",
            (unsigned)ctx->stair_layer, (unsigned)ctx->stair_balls);
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

/** 处理用户命令；手动启动暂不使用，STOP在运行阶段始终有效。 */
static void mission_handle_command(
    mission_context_t *ctx,
    mission_user_command_t command)
{
    uint16_t request_id;

    if (command == MISSION_USER_COMMAND_STOP) {
        ctx->arm_home_ready = false;
        if ((ctx->state == MISSION_STATE_STOPPED) ||
            (ctx->state == MISSION_STATE_BOOT)) {
            return;
        }
        if ((ctx->state >= MISSION_STATE_BLOCK_WAIT_DIGIT) &&
            (ctx->state <= MISSION_STATE_BLOCK_WAIT_HOME)) {
            (void)arm_stop(NULL, NULL);
            ctx->active_arm_group = 0U;
        }
        (void)turn_stop(NULL, NULL);
        (void)mission_stop_vision(ctx);
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
    /* 手动启动暂不使用；正式流程在READY到期后按已选颜色自动启动。
    if (ctx->state != MISSION_STATE_READY) {
        return;
    }
    if (command == MISSION_USER_COMMAND_START_RED) {
        mission_start_run(ctx);
    } else if (command == MISSION_USER_COMMAND_START_BLUE) {
        mission_start_run(ctx);
    }
    */
}

/** 记录底盘层级通知，并只在状态、请求编号和ready值均匹配时推进。 */
static void mission_handle_chassis(
    mission_context_t *ctx,
    const chassis_mission_event_t *event)
{
    mission_stair_layer_t layer;
    uint8_t grasp_group;
    DEPOT_TRACE("[M] CHASSIS RX=%u ID=%u EXPECT=%u READY=%u\r\n",
                (unsigned)event->type, (unsigned)event->request_id,
                (unsigned)ctx->request_id, (unsigned)event->is_ready);

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
        STAIR_TRACE(ctx, "CHASSIS_REJECT TYPE=%u ID=%u EXPECT=%u READY=%u\r\n",
            (unsigned)event->type, (unsigned)event->request_id,
            (unsigned)ctx->request_id, (unsigned)event->is_ready);
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
#if !MISSION_CHASSIS_ROUTE_TEST_ENABLED
    if ((ctx->state == MISSION_STATE_WAIT_DEPOT_1) &&
        (event->type == CHASSIS_CMD_DEPOT_1_READY)) {
        mission_block_begin(ctx);
        return;
    }
    if ((ctx->state == MISSION_STATE_DEPOT_WAIT_POSITION) &&
        (event->type == (uint8_t)(CHASSIS_CMD_DEPOT_1_READY +
                                  ctx->depot_position - 1U))) {
        /* 红D4无数字；蓝D4有数字，必须按C100实测逻辑列放球。 */
        if ((ctx->depot_position == 4U) && (g_mission_side != MISSION_COLOR_BLUE)) {
            ctx->depot_column = 4U;
            mission_depot_next_ball(ctx);
        } else {
            ctx->depot_first_digit = 0U;
            mission_depot_start_digit(ctx);
        }
        return;
    }
    if ((ctx->state == MISSION_STATE_DEPOT_RETURN_ENTRY) &&
        (event->type == CHASSIS_CMD_DEPOT_1_READY)) {
        /* 蓝方小球收尾返D1实际停车后才允许回家，不重启积木/球档案。 */
        ctx->depot_position = 1U;
        mission_enter_state(ctx, MISSION_STATE_DEPOT_DWELL, 1000U);
        return;
    }
    if ((ctx->state == MISSION_STATE_DEPOT_WAIT_HOME) &&
        (event->type == CHASSIS_CMD_HOME_READY)) {
        mission_enter_state(ctx, MISSION_STATE_COMPLETE, 0U);
        return;
    }
#endif
    if (((ctx->state == MISSION_STATE_BLOCK_WAIT_POSITION) ||
         (ctx->state == MISSION_STATE_BLOCK_WAIT_D4) ||
         (ctx->state == MISSION_STATE_BLOCK_RETURN_DEPOT)) &&
        (event->type == (uint8_t)(CHASSIS_CMD_DEPOT_1_READY +
                                 ctx->block_point - 1U))) {
        ctx->depot_position = ctx->block_point;
        mission_block_position_done(ctx);
        return;
    }
    if ((ctx->state == MISSION_STATE_BLOCK_WAIT_HOME) &&
        (event->type == CHASSIS_CMD_HOME_READY)) {
        mission_enter_state(ctx, MISSION_STATE_COMPLETE, 0U);
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
            STAIR_TRACE(ctx, "LAYER_REJECT TYPE=%u STATE=%u\r\n",
                (unsigned)event->type, (unsigned)ctx->state);
            return;
        }
        /* 底盘段号保持LOW/HIGH/MID；蓝方实物层序为中/高/低。
         * 仅在接收处映射一次，视觉场景与夹取动作共用实际层号。 */
        if (g_mission_side == MISSION_COLOR_BLUE) {
            if (layer == MISSION_STAIR_LOW) layer = MISSION_STAIR_MID;
            else if (layer == MISSION_STAIR_MID) layer = MISSION_STAIR_LOW;
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
        STAIR_TRACE(ctx, "LAYER=%u RAW=%u SIDE=%u STATE=%u COUNT=%u\r\n",
            (unsigned)layer, (unsigned)event->type, (unsigned)g_mission_side,
            (unsigned)ctx->state, (unsigned)ctx->stair_balls);
        return;
    }
    /* 6) 只有底盘确认实际停车后才执行当前层抓取动作组。 */
    if ((event->type == CHASSIS_CMD_STAIR_PAUSE) &&
        (ctx->state == MISSION_STATE_STAIR_WAIT_PAUSE)) {
        if (ctx->vision.phase == MISSION_VISION_ACKING) {
            mission_enter_state(ctx, MISSION_STATE_STAIR_WAIT_ACK,
                                MISSION_OPERATION_TIMEOUT_MS);
            STAIR_TRACE(ctx, "PAUSE ID=%u WAIT_ACK\r\n", (unsigned)ctx->request_id);
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
        STAIR_TRACE(ctx, "PAUSE ID=%u GRASP_SENT=%u\r\n",
            (unsigned)ctx->request_id, (unsigned)grasp_group);
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
        STAIR_TRACE(ctx, "STAIRS_FINISHED COUNT=%u\r\n", (unsigned)ctx->stair_balls);
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

/** 12完成后前四球回11继续识别，最后球直接回10再读卡转槽。 */
static void mission_platform_prepare_storage(mission_context_t *ctx)
{
    bool last_ball = (uint8_t)(ctx->platform_balls + 1U) >= MISSION_PLATFORM_BALL_COUNT;
    if (!mission_start_arm(ctx,
            last_ball ? MISSION_HOME_ACTION_GROUP : MISSION_PLATFORM_VISION_GROUP,
            last_ball ? MISSION_STATE_PLATFORM_WAIT_AVOID : MISSION_STATE_PLATFORM_WAIT_RETURN)) {
        mission_fail(ctx, MISSION_FAULT_ARM);
    }
}

/** 圆盘结束时10已完成；正式去阶梯，无线单区域测试仍在此停止。 */
static void mission_finish_platform(mission_context_t *ctx)
{
#if MISSION_CHASSIS_ROUTE_TEST_ENABLED
    ctx->active_arm_group = 0U;
    mission_enter_state(ctx, MISSION_STATE_COMPLETE, 0U);
#else
    (void)mission_next_request_id(ctx);
    if (!mission_send_chassis(MISSION_CMD_GO_STAIRS, ctx->request_id)) {
        mission_fail(ctx, MISSION_FAULT_QUEUE);
        return;
    }
    STAIR_RESET();
    mission_enter_state(ctx, MISSION_STATE_WAIT_STAIRS, MISSION_OPERATION_TIMEOUT_MS);
    STAIR_TRACE(ctx, "GO_STAIRS ID=%u\r\n", (unsigned)ctx->request_id);
#endif
}

/** 处理唯一在途动作组结果，并按圆盘、阶梯或小圆盘子流程继续。 */
static void mission_handle_arm(mission_context_t *ctx, bool success)
{
    DEPOT_TRACE("[M] ARM DONE=%u OK=%u STATE=%u LAST=%lu\r\n",
                (unsigned)ctx->active_arm_group, (unsigned)success,
                (unsigned)ctx->state, (unsigned long)ctx->arm_last_action_report);
    if ((ctx->state == MISSION_STATE_BLOCK_PREPARE) ||
        (ctx->state == MISSION_STATE_BLOCK_WAIT_POSE) ||
        (ctx->state == MISSION_STATE_BLOCK_WAIT_GRASP) ||
        (ctx->state == MISSION_STATE_BLOCK_WAIT_PLACE) ||
        (ctx->state == MISSION_STATE_BLOCK_FINISH)) {
        if (!success) mission_fail(ctx, MISSION_FAULT_ARM);
        else mission_block_arm_done(ctx);
        return;
    }
#if !MISSION_CHASSIS_ROUTE_TEST_ENABLED
    if ((ctx->state == MISSION_STATE_DEPOT_PREPARE) ||
        (ctx->state == MISSION_STATE_DEPOT_SAFE) ||
        (ctx->state == MISSION_STATE_DEPOT_AVOID) ||
        (ctx->state == MISSION_STATE_DEPOT_PICK) ||
        (ctx->state == MISSION_STATE_DEPOT_PLACE) ||
        (ctx->state == MISSION_STATE_DEPOT_RETURN)) {
        if (!success) mission_fail(ctx, MISSION_FAULT_ARM);
        else mission_depot_arm_done(ctx);
        return;
    }
#endif
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
    /* 3) 首次11到位或最后球读卡失败后回11，先稳定再启动圆盘视觉。 */
    if (ctx->state == MISSION_STATE_PLATFORM_WAIT_POSE) {
        mission_enter_state(ctx, MISSION_STATE_PLATFORM_SETTLE,
                            MISSION_PLATFORM_SETTLE_MS);
        return;
    }
    /* 4) 动作组12完成后，前四球回11，第五球直接回10。 */
    if (ctx->state == MISSION_STATE_PLATFORM_WAIT_GRASP) {
        PLATFORM_TRACE("[P] T=%lu ARM12 DONE TRY=%u OK=%u\r\n",
            (unsigned long)osKernelGetTickCount(), (unsigned)ctx->platform_attempts,
            (unsigned)ctx->platform_balls);
        mission_platform_prepare_storage(ctx);
        return;
    }
    if (ctx->state == MISSION_STATE_PLATFORM_WAIT_AVOID) {
        /* 第五球等动作10完成后再读卡转槽，后续不重复收臂。 */
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
        mission_finish_platform(ctx);
        return;
    }
    /* 6) 阶梯入口动作组13完成后启动当前层。 */
    if (ctx->state == MISSION_STATE_STAIR_WAIT_POSE) {
        mission_start_stair_layer(ctx);
        return;
    }
    /* 7) 动作组14/15/16抓取完成后统一回动作组13。 */
    if (ctx->state == MISSION_STATE_STAIR_WAIT_GRASP) {
        STAIR_COUNT(grasps);
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
        STAIR_TOTAL(ctx);
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
        /* 等匹配D1实际到位后进入积木；返D1才接原转盘和仓库视觉。 */
        mission_enter_state(ctx, MISSION_STATE_WAIT_DEPOT_1,
                            MISSION_OPERATION_TIMEOUT_MS);
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
    if ((completed_state != MISSION_STATE_PLATFORM_WAIT_STORAGE) ||
        ctx->platform_read_ok) ++ctx->storage_slot;
    if (completed_state == MISSION_STATE_PLATFORM_WAIT_STORAGE) {
        /* 2) 圆盘读卡成功才计球；满5球或已尝试7次均收臂结束，不再重试。 */
        if (ctx->platform_read_ok) ++ctx->platform_balls;
        PLATFORM_TRACE("[P] T=%lu RESULT TRY=%u OK=%u NEXT_SLOT=%u\r\n",
            (unsigned long)osKernelGetTickCount(), (unsigned)ctx->platform_attempts,
            (unsigned)ctx->platform_balls, (unsigned)(ctx->storage_slot + 1U));
        if ((ctx->platform_balls >= MISSION_PLATFORM_BALL_COUNT) ||
            (ctx->platform_attempts >= MISSION_PLATFORM_MAX_ATTEMPTS)) {
            if (ctx->arm_home_ready) {
                mission_finish_platform(ctx);
            } else if (!mission_start_arm(
                    ctx,
                    MISSION_HOME_ACTION_GROUP,
                    MISSION_STATE_PLATFORM_WAIT_DEPARTURE_POSE)) {
                mission_fail(ctx, MISSION_FAULT_ARM);
            }
        } else if (ctx->active_arm_group == MISSION_HOME_ACTION_GROUP) {
            /* 最后球候选已回10；读卡失败后先回11，才能重新识别。 */
            if (!mission_start_arm(ctx, MISSION_PLATFORM_VISION_GROUP,
                                   MISSION_STATE_PLATFORM_WAIT_POSE)) {
                mission_fail(ctx, MISSION_FAULT_ARM);
            }
        } else {
            /* 成功转槽或读卡失败处理结束后计时，不让转盘运动覆盖稳定等待。 */
            mission_enter_state(ctx, MISSION_STATE_PLATFORM_SETTLE,
                                MISSION_PLATFORM_SETTLE_MS);
        }
        return;
    }
    if (completed_state == MISSION_STATE_STAIR_WAIT_STORAGE) {
        /* 3) 阶梯满2球后直接恢复；未满时先重启本层视觉。 */
        ++ctx->stair_balls;
        STAIR_TRACE(ctx, "STORED COUNT=%u NEXT_SLOT0=%u\r\n",
            (unsigned)ctx->stair_balls, (unsigned)ctx->storage_slot);
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

/** 0基槽号的CCW步数，正式与无线共用；当前槽即目标槽时无需转动。 */
static uint8_t mission_depot_ccw_steps(uint8_t current_slot, uint8_t target_slot)
{
    return (uint8_t)((current_slot + 12U - target_slot) % 12U);
}

#if !MISSION_CHASSIS_ROUTE_TEST_ENABLED
/** D1预检只标记异常球；未知目标、重复格、无效槽位不阻断其余正常球。 */
static void mission_depot_prepare(mission_context_t *ctx)
{
    ball_manifest_record_t record;
    uint16_t targets = 0U;
    uint16_t slots = 0U;
    uint8_t i;

    if ((ctx->manifest.count > BALL_MANIFEST_CAPACITY) ||
        (ctx->current_slot >= 12U)) {
        mission_fail(ctx, MISSION_FAULT_STORAGE);
        return;
    }
    ctx->depot_abnormal_mask = 0U;
    for (i = 0U; i < ctx->manifest.count; ++i) {
        uint16_t target_bit;
        uint16_t slot_bit;
        if ((ball_manifest_get(&ctx->manifest, i, &record) != BALL_MANIFEST_OK) ||
            (record.state != BALL_MANIFEST_STATE_STORED) ||
            (record.storage_slot >= 12U) ||
            (record.target_row < 1U) || (record.target_row > 3U) ||
            (record.target_column < 1U) || (record.target_column > 4U) ||
            (record.ic_code != (uint8_t)((record.target_row << 4) |
                                         record.target_column))) {
            ctx->depot_abnormal_mask |= (uint16_t)(1U << i);
            continue;
        }
        target_bit = (uint16_t)(1U << ((record.target_row - 1U) * 4U +
                                       record.target_column - 1U));
        slot_bit = (uint16_t)(1U << record.storage_slot);
        /* 同一目标只保留第一颗有效球；不修改manifest允许重复追加的契约。 */
        if (((targets & target_bit) != 0U) || ((slots & slot_bit) != 0U)) {
            ctx->depot_abnormal_mask |= (uint16_t)(1U << i);
            continue;
        }
        targets |= target_bit;
        slots |= slot_bit;
    }
    /* 入口已确认红D1/蓝D2到位，蓝方不得把真实D2改记为D1。 */
    if (g_mission_side != MISSION_COLOR_BLUE) ctx->depot_position = 1U;
    ctx->depot_columns_used = 0U;
    ctx->depot_first_digit = 0U;
    ctx->depot_preparing = true;
    ctx->depot_target_slot = 11U; /* 物理12槽；必须实际移动，不能伪造当前位置。 */
    DEPOT_TRACE("[M] DEPOT BEGIN BALLS=%u SKIP=0x%X SLOT=%u TARGET=12\r\n",
                (unsigned)ctx->manifest.count, (unsigned)ctx->depot_abnormal_mask,
                (unsigned)(ctx->current_slot + 1U));
    /* 已确认动作10时直接寻槽；其他姿态先等动作10完成，不能只看下发组号。 */
    if (ctx->arm_home_ready) {
        mission_enter_state(ctx, MISSION_STATE_DEPOT_SEEK, 1U);
    } else if (!mission_start_arm(ctx, MISSION_HOME_ACTION_GROUP,
                                  MISSION_STATE_DEPOT_PREPARE)) {
        mission_fail(ctx, MISSION_FAULT_ARM);
    }
}

/** 每次确认都建立独立场景5会话；3秒计时从READY开始。 */
static void mission_depot_start_digit(mission_context_t *ctx)
{
    ctx->depot_digit = 0U;
    DEPOT_TRACE("[M] DIGIT START D=%u PASS=%u\r\n",
                (unsigned)ctx->depot_position,
                (ctx->depot_first_digit == 0U) ? 1U : 2U);
    if (!mission_start_vision(ctx, MISSION_VISION_SCENE_DEPOT_DIGIT,
                              MISSION_STAIR_NONE,
                              MISSION_STATE_DEPOT_WAIT_DIGIT)) {
        mission_fail(ctx, MISSION_FAULT_VISION);
    }
}

/** ACK成功后才消费数字；前三站须两次一致，且逻辑列不得重复。 */
static void mission_depot_digit_done(mission_context_t *ctx)
{
    uint8_t digit = ctx->depot_digit;
    DEPOT_TRACE("[M] DIGIT CHECK=%u FIRST=%u USED=0x%X\r\n",
                (unsigned)digit, (unsigned)ctx->depot_first_digit,
                (unsigned)ctx->depot_columns_used);
    if ((digit < 1U) || (digit > 3U)) {
        mission_fail(ctx, MISSION_FAULT_VISION);
    } else if (ctx->depot_first_digit == 0U) {
        ctx->depot_first_digit = digit;
        mission_depot_start_digit(ctx);
    } else if ((digit != ctx->depot_first_digit) ||
               ((ctx->depot_columns_used & (1U << digit)) != 0U)) {
        mission_fail(ctx, MISSION_FAULT_VISION);
    } else {
        ctx->depot_columns_used |= (uint8_t)(1U << digit);
        ctx->depot_column = digit;
        mission_depot_next_ball(ctx);
    }
}

/** 每颗完成后按实际当前位置选本列CCW最近球；层高只决定放置动作。 */
static void mission_depot_next_ball(mission_context_t *ctx)
{
    ball_manifest_record_t record;
    uint8_t best_sequence = BALL_MANIFEST_CAPACITY;
    uint8_t best_steps = 12U;
    uint8_t best_row = 0U;
    uint8_t best_slot = 0U;
    uint8_t i;
    for (i = 0U; i < ctx->manifest.count; ++i) {
        uint8_t steps;
        if ((ctx->depot_abnormal_mask & (1U << i)) != 0U) continue;
        if (ball_manifest_get(&ctx->manifest, i, &record) != BALL_MANIFEST_OK) {
            mission_fail(ctx, MISSION_FAULT_STORAGE);
            return;
        }
        if ((record.state != BALL_MANIFEST_STATE_STORED) ||
            (record.target_column != ctx->depot_column)) continue;
        steps = mission_depot_ccw_steps(ctx->current_slot, record.storage_slot);
        if (steps < best_steps) {
            best_steps = steps;
            best_sequence = i;
            best_row = record.target_row;
            best_slot = record.storage_slot;
        }
    }
    if (best_sequence != BALL_MANIFEST_CAPACITY) {
        ctx->depot_sequence = best_sequence;
        ctx->depot_row = best_row;
        ctx->depot_target_slot = best_slot;
        DEPOT_TRACE("[M] BALL SEQ=%u ROW=%u COL=%u SLOT=%u\r\n",
                    (unsigned)best_sequence, (unsigned)best_row,
                    (unsigned)ctx->depot_column, (unsigned)(best_slot + 1U));
        if (ctx->arm_home_ready) {
            mission_enter_state(ctx, MISSION_STATE_DEPOT_SEEK, 1U);
        } else if (!mission_start_arm(ctx, MISSION_HOME_ACTION_GROUP,
                                      MISSION_STATE_DEPOT_AVOID)) {
            mission_fail(ctx, MISSION_FAULT_ARM);
        }
        return;
    }
    if (ctx->depot_position == 4U) {
        if (g_mission_side == MISSION_COLOR_BLUE) {
            /* 蓝D4是最后数字列；收臂后返回空列D1，等匹配到位再回家。 */
            if (!mission_send_chassis(MISSION_CMD_GO_DEPOT_1,
                                      mission_next_request_id(ctx))) {
                mission_fail(ctx, MISSION_FAULT_QUEUE);
                return;
            }
            mission_enter_state(ctx, MISSION_STATE_DEPOT_RETURN_ENTRY,
                                MISSION_OPERATION_TIMEOUT_MS);
            return;
        }
        /* 红D4最后一次收臂完成（无球则到位）后停1秒，直接回家。 */
        mission_enter_state(ctx, MISSION_STATE_DEPOT_DWELL, 1000U);
        return;
    }
    ++ctx->depot_position;
    if (!mission_send_chassis(
            (mission_command_type_t)(MISSION_CMD_GO_DEPOT_1 +
                                      ctx->depot_position - 1U),
            mission_next_request_id(ctx))) {
        mission_fail(ctx, MISSION_FAULT_QUEUE);
        return;
    }
    mission_enter_state(ctx, MISSION_STATE_DEPOT_WAIT_POSITION,
                        MISSION_OPERATION_TIMEOUT_MS);
}

/** 每个动作组都等控制器完成回报；23已包含高层撤离，放置后直接回10。 */
static void mission_depot_arm_done(mission_context_t *ctx)
{
    uint8_t group;
    mission_state_t next;
    switch (ctx->state) {
    case MISSION_STATE_DEPOT_PREPARE:
    case MISSION_STATE_DEPOT_AVOID:
        /* 逐槽工作由主循环驱动，保持已完成的动作10姿态直到目标槽到位。 */
        mission_enter_state(ctx, MISSION_STATE_DEPOT_SEEK, 1U);
        return;
    case MISSION_STATE_DEPOT_SAFE:
        if (ctx->depot_preparing) {
            ctx->depot_preparing = false;
            mission_depot_start_digit(ctx);
            return;
        }
        group = MISSION_DEPOT_PICK_GROUP;
        next = MISSION_STATE_DEPOT_PICK;
        break;
    case MISSION_STATE_DEPOT_PICK:
        group = (ctx->depot_row == 3U) ? MISSION_DEPOT_ROW3_GROUP :
            ((ctx->depot_row == 2U) ? MISSION_DEPOT_ROW2_GROUP :
                                     MISSION_DEPOT_ROW1_GROUP);
        next = MISSION_STATE_DEPOT_PLACE;
        break;
    case MISSION_STATE_DEPOT_PLACE:
        group = MISSION_HOME_ACTION_GROUP;
        next = MISSION_STATE_DEPOT_RETURN;
        break;
    case MISSION_STATE_DEPOT_RETURN:
        if (ball_manifest_mark_placed(&ctx->manifest, ctx->depot_sequence) !=
            BALL_MANIFEST_OK) {
            mission_fail(ctx, MISSION_FAULT_STORAGE);
            return;
        }
        mission_depot_next_ball(ctx);
        return;
    default:
        return;
    }
    if (!mission_start_arm(ctx, group, next)) mission_fail(ctx, MISSION_FAULT_ARM);
}
#endif

/**
 * @brief 处理当前状态的期限到达事件。
 * @param ctx Mission上下文。
 * @note READY到期时按已选颜色自动启动；其他有期限状态到期时进入超时故障。
 */
static void mission_check_timeout(mission_context_t *ctx)
{
    if ((ctx->deadline_tick != 0U) &&
        ((int32_t)(osKernelGetTickCount() - ctx->deadline_tick) >= 0)) {
        if (ctx->state == MISSION_STATE_READY) {
            mission_start_run(ctx);
            return;
        }
        if (ctx->state == MISSION_STATE_PLATFORM_SETTLE) {
            /* 非阻塞等待，期间主循环仍可处理STOP；正式与无线共用。 */
            PLATFORM_TRACE("[P] T=%lu SETTLE DONE MS=%u -> VISION\r\n",
                (unsigned long)osKernelGetTickCount(), (unsigned)MISSION_PLATFORM_SETTLE_MS);
            if (!mission_start_vision(ctx, MISSION_VISION_SCENE_PLATFORM,
                                      MISSION_STAIR_NONE,
                                      MISSION_STATE_PLATFORM_WAIT_VISION)) {
                mission_fail(ctx, MISSION_FAULT_VISION);
            }
            return;
        }
        if (ctx->state == MISSION_STATE_BLOCK_SETTLE) {
            ctx->block_result_received = false;
            if (!mission_start_vision(ctx, MISSION_VISION_SCENE_BLOCK_DIGIT,
                                      MISSION_STAIR_NONE, MISSION_STATE_BLOCK_WAIT_DIGIT))
                mission_fail(ctx, MISSION_FAULT_VISION);
            return;
        }
#if !MISSION_CHASSIS_ROUTE_TEST_ENABLED
        if (ctx->state == MISSION_STATE_DEPOT_SEEK) {
            /* 仓库保留PB0校准，微调耗尽也继续；电机到位/故障保护仍有效。 */
            if (ctx->current_slot != ctx->depot_target_slot) {
                zdt_turntable_direction_t direction = ctx->depot_preparing ?
                    ZDT_TURNTABLE_DIR_CW : ZDT_TURNTABLE_DIR_CCW;
                if (!mission_advance_slot(ctx, direction, NULL)) {
                    if ((ctx->state != MISSION_STATE_STOPPING) &&
                        (ctx->state != MISSION_STATE_STOPPED)) {
                        mission_fail(ctx, MISSION_FAULT_STORAGE);
                    }
                    return;
                }
                ctx->current_slot = (uint8_t)((ctx->current_slot +
                    ((direction == ZDT_TURNTABLE_DIR_CW) ? 1U : 11U)) % 12U);
            }
            if (ctx->current_slot == ctx->depot_target_slot) {
                DEPOT_TRACE("[M] SLOT REACHED=%u HOME_READY=%u\r\n",
                            (unsigned)(ctx->current_slot + 1U),
                            (unsigned)ctx->arm_home_ready);
                /* 转盘运动不改变臂姿态；仍在10就直接识别或取球，不重复收臂。 */
                if (ctx->arm_home_ready) {
                    mission_enter_state(ctx, MISSION_STATE_DEPOT_SAFE, 0U);
                    mission_depot_arm_done(ctx);
                } else if (!mission_start_arm(ctx, MISSION_HOME_ACTION_GROUP,
                                              MISSION_STATE_DEPOT_SAFE)) {
                    mission_fail(ctx, MISSION_FAULT_ARM);
                }
            } else {
                mission_enter_state(ctx, MISSION_STATE_DEPOT_SEEK, 1U);
            }
            return;
        }
        if (ctx->state == MISSION_STATE_DEPOT_WAIT_DIGIT) {
            /* NONE先关闭会话并等STOPPED；不沿用无线巡检的继续横移策略。 */
            DEPOT_TRACE("[M] DIGIT TIMEOUT D=%u V=%u -> STOP SESSION\r\n",
                        (unsigned)ctx->depot_position, (unsigned)ctx->vision.phase);
            mission_enter_state(ctx, MISSION_STATE_DEPOT_DIGIT_STOP,
                                MISSION_OPERATION_TIMEOUT_MS);
            if (!mission_stop_vision(ctx)) mission_fail(ctx, MISSION_FAULT_VISION);
            return;
        }
        if (ctx->state == MISSION_STATE_DEPOT_DWELL) {
            if (!mission_send_chassis(MISSION_CMD_DEPOT_OK,
                                      mission_next_request_id(ctx))) {
                mission_fail(ctx, MISSION_FAULT_QUEUE);
                return;
            }
            mission_enter_state(ctx, MISSION_STATE_DEPOT_WAIT_HOME,
                                MISSION_OPERATION_TIMEOUT_MS);
            return;
        }
#endif
        DEPOT_TRACE("[M] TIMEOUT STATE=%u ARM=%u LAST=%lu\r\n",
                    (unsigned)ctx->state, (unsigned)ctx->active_arm_group,
                    (unsigned long)ctx->arm_last_action_report);
        mission_fail(ctx, MISSION_FAULT_TIMEOUT);
    }
}

#if MISSION_DEPOT_TRACE_ENABLED && !MISSION_CHASSIS_ROUTE_TEST_ENABLED
/** 起点每秒汇总阻塞条件，不在回调打印、不更改就绪或故障策略。 */
static void mission_boot_trace(mission_context_t *ctx, uint32_t *next_tick)
{
    uint32_t now = osKernelGetTickCount();
    if (((ctx->state != MISSION_STATE_WAIT_HOME) &&
         (ctx->state != MISSION_STATE_WAIT_CHASSIS_READY)) ||
        ((int32_t)(now - *next_tick) < 0)) return;
    *next_tick = now + mission_ms_to_ticks(MISSION_MODEL_QUERY_INTERVAL_MS);
    DEPOT_TRACE("[BOOT] WAIT STATE=%u ARM10=%u CHASSIS=%u MODEL=%u V=%u BUSY=%u PENDING=%u LAST=%lu\r\n",
        (unsigned)ctx->state, (unsigned)ctx->arm_home_ready,
        (unsigned)ctx->chassis_ready, (unsigned)ctx->block_model_ready,
        (unsigned)ctx->vision.phase, (unsigned)ctx->vision.inflight,
        (unsigned)ctx->vision.completion_pending, (unsigned long)ctx->arm_last_action_report);
}
#endif

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
#if MISSION_DEPOT_TRACE_ENABLED && !MISSION_CHASSIS_ROUTE_TEST_ENABLED
    uint32_t boot_trace_next_tick = 0U;
    /* 只启用诊断输出，不解析无线测试命令；关闭开关即不占用该调试实例。 */
    (void)debug_uart1_init(&g_depot_debug);
#endif
    DEPOT_TRACE("[M] FORMAL DEPOT/STAIR TRACE ON\r\n");
    DEPOT_TRACE("[BOOT] FORMAL SIDE=%s ARM10 REQUIRED CHASSIS + MODEL POLL=%uMS\r\n",
        (g_mission_side == MISSION_COLOR_BLUE) ? "BLUE" :
        ((g_mission_side == MISSION_COLOR_RED) ? "RED" : "NONE"),
        (unsigned)MISSION_MODEL_QUERY_INTERVAL_MS);
    mission_enter_state(ctx, MISSION_STATE_WAIT_HOME, 0U);
    for (;;) {
        /* 1) 等待任一事件；等待时长由当前状态的截止时间决定。 */
#if MISSION_DEPOT_TRACE_ENABLED && !MISSION_CHASSIS_ROUTE_TEST_ENABLED
        mission_boot_trace(ctx, &boot_trace_next_tick);
#endif
        mission_model_process(ctx);
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
        STAIR_POLL(ctx);
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
        "ARM_GROUP=%u ARM_EVENT=0x%02X MODEL_READY=%u\r\n",
        mode,
        mission_test_stage_name(g_wireless_test.target),
        mission_test_stage_name(g_wireless_test.expected),
        (unsigned)ctx->state,
        mission_test_color_name(g_mission_side),
        (unsigned)(g_wireless_test.ball_home ? g_wireless_test.manual_count :
                   ctx->manifest.count),
        (unsigned)(g_wireless_test.ball_home ? g_wireless_test.current_slot :
                   ctx->storage_slot),
        (unsigned)ctx->fault_code,
        (unsigned)ctx->arm_boot_state,
        (unsigned)g_wireless_test.arm_ready,
        (unsigned)(ctx->arm_last_action_report & 0xFFU),
        (unsigned)((ctx->arm_last_action_report >> 8) & 0xFFU),
        (unsigned)ctx->block_model_ready);
    mission_test_write(g_wireless_test.text);
}

/** @copydoc mission_test_print_ball */
static void mission_test_print_ball(
    const mission_context_t *ctx,
    uint8_t sequence)
{
    ball_manifest_record_t record;

    if (g_wireless_test.ball_home) {
        const mission_test_ball_t *manual;

        if (sequence >= g_wireless_test.manual_count) {
            mission_test_write("ERR BALL\r\n");
            return;
        }
        manual = &g_wireless_test.manual_balls[sequence];
        (void)snprintf(g_wireless_test.text, sizeof(g_wireless_test.text),
                       "BALL N=%u REGION=MANUAL IC=0x%02X ROW=%u COL=%u "
                       "SLOT=%u STATE=%s\r\n",
                       (unsigned)sequence + 1U,
                       (unsigned)manual->ball.code,
                       (unsigned)manual->ball.row,
                       (unsigned)manual->ball.column,
                       (unsigned)sequence + 1U,
                       manual->placed ? "PLACED" : "STORED");
        mission_test_write(g_wireless_test.text);
        return;
    }

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
        (unsigned)(g_wireless_test.ball_home ? g_wireless_test.manual_count :
                   ctx->manifest.count));
    mission_test_write(g_wireless_test.text);
    for (i = 0U; i < (g_wireless_test.ball_home ?
                      g_wireless_test.manual_count : ctx->manifest.count); ++i) {
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
            "SELECT: RED or BLUE before handshake\r\n"
            "PATH: PLATFORM STAIR DISC DEPOT D1 D2 D3 D4\r\n"
            "DISC_READY: GO TO DISC START AND WAIT\r\n"
            "DISC_RUN: RUN FROM DISC_READY THEN STOP\r\n"
            "DEPOT: HOME HOME_DIRECT\r\n"
            "TARGET: ROUTE PLATFORM|STAIRS|DISC|DEPOT\r\n"
            "ROUTE DEPOT: AUTO D1-D4 DIGIT\r\n"
            "ROUTE BLOCK DIGIT: PURE PATH THEN 28/29/30 SCAN, D4, HOME\r\n"
            "ROUTE BLOCK MOVE: SCAN/GRASP/PLACE, D1, HOME; NO BALL PLACE\r\n"
            "BALL HOME: AT D1 AFTER ROUTE DEPOT BALL\r\n"
            "TURN CW|CCW: TEST ONE SLOT BEFORE BALL HOME\r\n"
            "ROUTE DEPOT BALL: D1 THEN BALL HOME THEN AUTO PLACE\r\n"
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
        (ball_number <= (g_wireless_test.ball_home ?
                         g_wireless_test.manual_count : ctx->manifest.count))) {
        mission_test_print_ball(ctx, (uint8_t)(ball_number - 1U));
        return true;
    }
    if ((strncmp(command, "BALL ", 5U) == 0) &&
        (strcmp(command, "BALL HOME") != 0)) {
        mission_test_write("ERR BALL\r\n");
        return true;
    }
    if (strcmp(command, "STOP") == 0) {
        uint16_t request_id;

        ctx->arm_home_ready = false;

        if (g_wireless_test.stop_requested ||
            (ctx->state == MISSION_STATE_STOPPED)) {
            mission_test_write("STOPPED\r\n");
            return true;
        }
        request_id = mission_next_request_id(ctx);
        if ((ctx->state >= MISSION_STATE_BLOCK_WAIT_DIGIT) &&
            (ctx->state <= MISSION_STATE_BLOCK_WAIT_HOME)) {
            (void)arm_stop(NULL, NULL);
            ctx->active_arm_group = 0U;
            (void)mission_stop_vision(ctx);
        }
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

/** 纯路径分两步：先到绕行起点等待人工摆臂，再放行既有底盘绕行。 */
static bool mission_test_disc_path_command(mission_context_t *ctx, const char *command)
{
    bool ok;
    if ((strcmp(command, "DISC_READY") != 0) &&
        (strcmp(command, "DISC_RUN") != 0)) return false;
    if (strcmp(command, "DISC_READY") == 0) {
        if (g_wireless_test.small_disc_at_start) {
            mission_test_write("READY DISC_RUN\r\n");
            return true;
        }
        if ((g_wireless_test.mode == MISSION_TEST_MODE_TARGET) ||
            ((g_wireless_test.expected != MISSION_TEST_STAGE_PLATFORM) &&
             (g_wireless_test.expected != MISSION_TEST_STAGE_STAIRS) &&
             (g_wireless_test.expected != MISSION_TEST_STAGE_SMALL_DISC))) {
            mission_test_write("ERR DISC_READY ORDER\r\n");
            return true;
        }
        g_wireless_test.mode = MISSION_TEST_MODE_PATH;
        ok = ((g_wireless_test.expected != MISSION_TEST_STAGE_PLATFORM) ||
              mission_test_skip_platform(ctx)) &&
             ((g_wireless_test.expected == MISSION_TEST_STAGE_SMALL_DISC) ||
              mission_test_skip_stairs(ctx)) &&
             mission_test_send_wait(ctx, MISSION_CMD_GO_SMALL_DISC,
                 CHASSIS_CMD_SMALL_DISC_READY, MISSION_STATE_WAIT_SMALL_DISC);
        if (ok) {
            g_wireless_test.small_disc_at_start = true;
            g_wireless_test.expected = MISSION_TEST_STAGE_SMALL_DISC;
            /* 到位后不计超时、不启动视觉或动作组，持续等人工DISC_RUN。 */
            mission_enter_state(ctx, MISSION_STATE_WAIT_SMALL_DISC, 0U);
            mission_test_write("DONE DISC_READY\r\nREADY DISC_RUN\r\n");
            return true;
        }
    } else {
        if (!g_wireless_test.small_disc_at_start) {
            mission_test_write("ERR WAIT DISC_READY\r\n");
            return true;
        }
        g_wireless_test.small_disc_at_start = false;
        /* 沿用到位请求号；不重复GO_SMALL_DISC，也不重新经过前置路线。 */
        mission_enter_state(ctx, MISSION_STATE_WAIT_SMALL_DISC,
                            MISSION_OPERATION_TIMEOUT_MS);
        ok = mission_send_chassis(MISSION_CMD_SMALL_DISC_START, ctx->request_id) &&
             mission_test_wait_chassis_event(ctx, CHASSIS_CMD_SMALL_DISC_FINISHED,
                                             ctx->request_id);
        if (ok) {
            g_wireless_test.expected = MISSION_TEST_STAGE_DEPOT;
            mission_enter_state(ctx, MISSION_STATE_READY, 0U);
            mission_test_write("DONE DISC_RUN\r\nREADY DEPOT\r\n");
            return true;
        }
    }
    if (!g_wireless_test.stop_requested) {
        mission_fail(ctx, MISSION_FAULT_CHASSIS);
        mission_test_write("FAULT ROUTE\r\n");
    }
    return true;
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

/** 每站单独建立场景5会话；有效数字立即通过，持续无数字约3秒才报NONE。 */
static bool mission_test_read_depot_digit(mission_context_t *ctx, uint8_t *digit)
{
    char command[DEBUG_UART1_RX_BUFFER_SIZE];
    uint32_t flags;
    uint32_t deadline = 0U;
    bool none_pending = false;
    bool aborting = false;

    g_wireless_test.depot_digit = 0U;
    if (!mission_start_vision(ctx, MISSION_VISION_SCENE_DEPOT_DIGIT,
                              MISSION_STAIR_NONE,
                              MISSION_STATE_DEPOT_WAIT_DIGIT)) {
        mission_fail(ctx, MISSION_FAULT_VISION);
        return false;
    }
    for (;;) {
        flags = osThreadFlagsWait(MISSION_FLAG_VISION_DONE,
                                  osFlagsWaitAny,
                                  mission_ms_to_ticks(MISSION_WIRELESS_POLL_MS));
        if (((flags & osFlagsError) == 0U) &&
            ((flags & MISSION_FLAG_VISION_DONE) != 0U)) {
            mission_handle_vision(ctx);
        }
        if (mission_test_take_command(command, sizeof(command)) &&
            !mission_test_handle_aux_command(ctx, command)) {
            mission_test_write("BUSY\r\n");
        }
        if (g_wireless_test.stop_requested && !aborting) {
            aborting = true;
            if (ctx->vision.phase != MISSION_VISION_IDLE) {
                (void)mission_stop_vision(ctx);
            }
        }
        if (ctx->state == MISSION_STATE_FAULT) return false;
        if (aborting) {
            if (ctx->vision.phase == MISSION_VISION_IDLE) {
                (void)mission_test_wait_chassis_event(
                    ctx, CHASSIS_CMD_STOPPED, ctx->request_id);
                return false;
            }
            mission_vision_process(ctx);
            continue;
        }
        if ((ctx->vision.phase == MISSION_VISION_LISTENING) &&
            (deadline == 0U)) {
            deadline = osKernelGetTickCount() +
                mission_ms_to_ticks(MISSION_DEPOT_DIGIT_WAIT_MS);
        }
        if ((g_wireless_test.depot_digit != 0U) &&
            (ctx->vision.phase == MISSION_VISION_IDLE)) {
            *digit = g_wireless_test.depot_digit;
            return true;
        }
        if (!none_pending && (deadline != 0U) &&
            ((int32_t)(osKernelGetTickCount() - deadline) >= 0)) {
            none_pending = true;
            if (!mission_stop_vision(ctx)) {
                mission_fail(ctx, MISSION_FAULT_VISION);
                return false;
            }
        }
        if (none_pending && (ctx->vision.phase == MISSION_VISION_IDLE)) {
            *digit = 0U;
            return true;
        }
        mission_vision_process(ctx);
        mission_check_timeout(ctx);
    }
}

/** 九球预装完毕后自动读卡、逐槽转动；任一槽失败即停止，不跳过球。 */
static bool mission_test_ball_home_load(mission_context_t *ctx)
{
    uint8_t attempt;
    uint8_t moves;
    mission_test_ball_t *record;
    char command[DEBUG_UART1_RX_BUFFER_SIZE];

    if (!g_wireless_test.ball_home || g_wireless_test.ball_home_ready) {
        return false;
    }
    while (g_wireless_test.manual_count < BALL_MANIFEST_CAPACITY) {
        for (attempt = 0U; attempt < MISSION_IC_MAX_ATTEMPTS; ++attempt) {
            if (mission_test_take_command(command, sizeof(command)) &&
                !mission_test_handle_aux_command(ctx, command)) {
                mission_test_write("BUSY\r\n");
            }
            if (g_wireless_test.stop_requested) return false;
            (void)osThreadFlagsClear(MISSION_FLAG_IC_DONE);
            ctx->storage.ic_status = IC_CARD_ERR_BUSY;
            if ((ic_read(MISSION_IC_OPERATION_PROMPT != 0U,
                         mission_ic_done, ctx) == IC_CARD_OK) &&
                mission_wait_device(MISSION_FLAG_IC_DONE,
                                    IC_READ_TIMEOUT_MS + 100U) &&
                (ctx->storage.ic_status == IC_CARD_OK) &&
                (ctx->storage.ic_ball.kind == IC_BALL_TARGET) &&
                (ctx->storage.ic_ball.row >= 1U) &&
                (ctx->storage.ic_ball.row <= 3U) &&
                (ctx->storage.ic_ball.column >= 1U) &&
                (ctx->storage.ic_ball.column <= 4U)) break;
            if ((attempt + 1U) < MISSION_IC_MAX_ATTEMPTS) {
                (void)osDelay(mission_ms_to_ticks(MISSION_IC_RETRY_MS));
            }
        }
        if (attempt == MISSION_IC_MAX_ATTEMPTS) {
            (void)snprintf(g_wireless_test.text, sizeof(g_wireless_test.text),
                           "FAULT IC SLOT=%u RESET REQUIRED\r\n",
                           (unsigned)g_wireless_test.current_slot);
            mission_test_write(g_wireless_test.text);
            mission_fail(ctx, MISSION_FAULT_STORAGE);
            return false;
        }
        record = &g_wireless_test.manual_balls[g_wireless_test.manual_count];
        record->ball = ctx->storage.ic_ball;
        record->placed = false;
        ++g_wireless_test.manual_count;
        mission_test_print_ball(ctx,
                                (uint8_t)(g_wireless_test.manual_count - 1U));
        moves = (g_wireless_test.manual_count == BALL_MANIFEST_CAPACITY) ?
            3U : 1U;
        while (moves-- > 0U) {
            if (!mission_advance_slot(ctx,
                    MISSION_SLOT_USE_CW ? ZDT_TURNTABLE_DIR_CW :
                                          ZDT_TURNTABLE_DIR_CCW, NULL)) {
                (void)turn_stop(NULL, NULL);
                if (!g_wireless_test.stop_requested) {
                    mission_fail(ctx, MISSION_FAULT_STORAGE);
                }
                mission_test_write("SLOT POSITION UNKNOWN RESET REQUIRED\r\n");
                return false;
            }
            g_wireless_test.current_slot =
                (g_wireless_test.current_slot == 12U) ? 1U :
                (uint8_t)(g_wireless_test.current_slot + 1U);
            (void)snprintf(g_wireless_test.text, sizeof(g_wireless_test.text),
                           "SLOT=%u\r\n",
                           (unsigned)g_wireless_test.current_slot);
            mission_test_write(g_wireless_test.text);
        }
    }
    if (mission_test_take_command(command, sizeof(command)) &&
        !mission_test_handle_aux_command(ctx, command)) {
        mission_test_write("BUSY\r\n");
    }
    if (g_wireless_test.stop_requested) return false;
    /* 装球转盘期间保持动作10；已完成该姿态则不重复下发。 */
    if (!mission_test_run_arm_group(ctx, MISSION_HOME_ACTION_GROUP)) {
        if (!g_wireless_test.stop_requested) {
            mission_fail(ctx, MISSION_FAULT_ARM);
        }
        mission_test_write("FAULT BALL HOME ARM 10\r\n");
        return false;
    }
    g_wireless_test.ball_home_ready = true;
    mission_test_write("BALL HOME READY SLOT=12\r\n");
    return true;
}

/** 等动作组完成回报；STOP会同时请求机械臂停止，不能视作放置成功。 */
static bool mission_test_run_arm_group(mission_context_t *ctx, uint8_t group)
{
    char command[DEBUG_UART1_RX_BUFFER_SIZE];
    uint32_t flags;

    if ((group == MISSION_HOME_ACTION_GROUP) && ctx->arm_home_ready &&
        !g_wireless_test.stop_requested) {
        mission_enter_state(ctx, MISSION_STATE_WAIT_DEPOT_1, 0U);
        return true;
    }
    (void)osThreadFlagsClear(MISSION_FLAG_ARM_OK | MISSION_FLAG_ARM_FAIL);
    if (!mission_start_arm(ctx, group, MISSION_STATE_DEPOT_WAIT_ARM)) return false;
    for (;;) {
        flags = osThreadFlagsWait(MISSION_FLAG_ARM_OK | MISSION_FLAG_ARM_FAIL,
                                  osFlagsWaitAny,
                                  mission_ms_to_ticks(MISSION_WIRELESS_POLL_MS));
        if (mission_test_take_command(command, sizeof(command)) &&
            !mission_test_handle_aux_command(ctx, command)) {
            mission_test_write("BUSY\r\n");
        }
        if (g_wireless_test.stop_requested ||
            ((int32_t)(osKernelGetTickCount() - ctx->deadline_tick) >= 0)) {
            (void)arm_stop(NULL, NULL);
            ctx->arm_home_ready = false;
            ctx->active_arm_group = 0U;
            return false;
        }
        if ((flags & osFlagsError) == 0U) {
            if ((flags & MISSION_FLAG_ARM_FAIL) != 0U) ctx->arm_home_ready = false;
            ctx->active_arm_group = 0U;
            mission_enter_state(ctx, MISSION_STATE_WAIT_DEPOT_1, 0U);
            return ((flags & MISSION_FLAG_ARM_OK) != 0U) &&
                   ((flags & MISSION_FLAG_ARM_FAIL) == 0U);
        }
    }
}

/** 卸球从12槽开始逆向逐格定位；每步均用PB0同向微调确认。 */
static bool mission_test_seek_ball_slot(mission_context_t *ctx, uint8_t slot)
{
    uint8_t fine_used;

    while (g_wireless_test.current_slot != slot) {
        fine_used = 0U;
        if (g_wireless_test.stop_requested ||
            !mission_advance_slot(ctx, ZDT_TURNTABLE_DIR_CCW, &fine_used)) {
            (void)turn_stop(NULL, NULL);
            return false;
        }
        g_wireless_test.current_slot =
            (g_wireless_test.current_slot == 1U) ? 12U :
            (uint8_t)(g_wireless_test.current_slot - 1U);
        (void)snprintf(g_wireless_test.text, sizeof(g_wireless_test.text),
                       "CCW SLOT=%u PB0=OPTIONAL FINE=%u\r\n",
                       (unsigned)g_wireless_test.current_slot,
                       (unsigned)fine_used);
        mission_test_write(g_wireless_test.text);
    }
    return true;
}

/** 每站确认列号，按实际当前槽选本列CCW最近球，层高决定完整放置动作。 */
static bool mission_test_run_depot_balls(mission_context_t *ctx)
{
    static const mission_command_type_t commands[] = {
        MISSION_CMD_GO_DEPOT_2, MISSION_CMD_GO_DEPOT_3,
        MISSION_CMD_GO_DEPOT_4,
    };
    static const chassis_command_type_t events[] = {
        CHASSIS_CMD_DEPOT_2_READY, CHASSIS_CMD_DEPOT_3_READY,
        CHASSIS_CMD_DEPOT_4_READY,
    };
    uint8_t depot, digit, confirmation, row, i, used_columns = 0U;

    /* 相同目标格不应出现；出发前拦截可避免放到一半才发现冲突。 */
    for (i = 0U; i < BALL_MANIFEST_CAPACITY; ++i) {
        uint8_t j;
        for (j = 0U; j < i; ++j) {
            if ((g_wireless_test.manual_balls[i].ball.row ==
                 g_wireless_test.manual_balls[j].ball.row) &&
                (g_wireless_test.manual_balls[i].ball.column ==
                 g_wireless_test.manual_balls[j].ball.column)) {
                mission_test_write("FAULT DUPLICATE BALL TARGET\r\n");
                return false;
            }
        }
    }
    /* 蓝D1只放积木，数字列D2～D4；物理D号不是IC逻辑列号。 */
    for (depot = (g_mission_side == MISSION_COLOR_BLUE) ? 2U : 1U; depot <= 4U; ++depot) {
        if ((depot > 1U) &&
            !mission_test_send_wait(ctx, commands[depot - 2U],
                                    events[depot - 2U],
                                    MISSION_STATE_WAIT_DEPOT_1)) return false;
        g_wireless_test.depot_position = depot;
        digit = 4U; /* 红方物理D4保留固定逻辑第4列；蓝方随后读取实测数字。 */
        if ((depot < 4U) || (g_mission_side == MISSION_COLOR_BLUE)) {
            if (!mission_test_read_depot_digit(ctx, &digit) ||
                (digit < 1U) || (digit > 3U) ||
                !mission_test_read_depot_digit(ctx, &confirmation) ||
                (digit != confirmation)) {
                mission_test_write("FAULT DEPOT DIGIT\r\n");
                return false;
            }
        }
        if ((used_columns & (1U << digit)) != 0U) {
            mission_test_write("FAULT DUPLICATE DEPOT COLUMN\r\n");
            return false;
        }
        used_columns |= (uint8_t)(1U << digit);
        mission_enter_state(ctx, MISSION_STATE_WAIT_DEPOT_1, 0U);
        (void)snprintf(g_wireless_test.text, sizeof(g_wireless_test.text),
                       "DIGIT D%u=%u\r\n", (unsigned)depot, (unsigned)digit);
        mission_test_write(g_wireless_test.text);
        for (;;) {
            uint8_t best_sequence = BALL_MANIFEST_CAPACITY;
            uint8_t best_steps = 12U;
            mission_test_ball_t *ball;
            for (i = 0U; i < BALL_MANIFEST_CAPACITY; ++i) {
                uint8_t steps;
                ball = &g_wireless_test.manual_balls[i];
                if (ball->placed || (ball->ball.column != digit)) continue;
                /* 无线槽号为1基，转换后共用CCW距离；不退回12槽重新排序。 */
                steps = mission_depot_ccw_steps(
                    (uint8_t)(g_wireless_test.current_slot - 1U), i);
                if (steps < best_steps) {
                    best_steps = steps;
                    best_sequence = i;
                }
            }
            if (best_sequence == BALL_MANIFEST_CAPACITY) break;
            i = best_sequence;
            ball = &g_wireless_test.manual_balls[i];
            row = ball->ball.row;
            /* 转盘前确认动作10，已到位不重发；PB0确认后直接动作22取球。 */
            if (!mission_test_run_arm_group(
                    ctx, MISSION_HOME_ACTION_GROUP) ||
                !mission_test_seek_ball_slot(ctx, (uint8_t)(i + 1U)) ||
                !mission_test_run_arm_group(ctx, MISSION_DEPOT_PICK_GROUP) ||
                !mission_test_run_arm_group(ctx,
                    (row == 3U) ? MISSION_DEPOT_ROW3_GROUP :
                    ((row == 2U) ? MISSION_DEPOT_ROW2_GROUP :
                                     MISSION_DEPOT_ROW1_GROUP)) ||
                !mission_test_run_arm_group(ctx, MISSION_HOME_ACTION_GROUP)) {
                mission_test_write("FAULT DEPOT PLACE\r\n");
                return false;
            }
            ball->placed = true;
            mission_test_print_ball(ctx, i);
        }
    }
    for (i = 0U; i < BALL_MANIFEST_CAPACITY; ++i) {
        if (!g_wireless_test.manual_balls[i].placed) {
            mission_test_write("FAULT UNPLACED BALL\r\n");
            return false;
        }
    }
    if (g_mission_side == MISSION_COLOR_BLUE) {
        if (!mission_test_send_wait(ctx, MISSION_CMD_GO_DEPOT_1,
                CHASSIS_CMD_DEPOT_1_READY, MISSION_STATE_WAIT_DEPOT_1)) return false;
        g_wireless_test.depot_position = 1U; /* 实际返D1后仍等原手动HOME指令。 */
    }
    return true;
}

/** ROUTE DEPOT复用现有底盘点位命令，逐站输出最终数字后才横移。 */
static bool mission_test_run_depot_digits(mission_context_t *ctx)
{
    static const mission_command_type_t commands[] = {
        MISSION_CMD_GO_DEPOT_2, MISSION_CMD_GO_DEPOT_3,
        MISSION_CMD_GO_DEPOT_4,
    };
    static const chassis_command_type_t events[] = {
        CHASSIS_CMD_DEPOT_2_READY, CHASSIS_CMD_DEPOT_3_READY,
        CHASSIS_CMD_DEPOT_4_READY,
    };
    uint8_t depot;
    uint8_t digit;

    /* 蓝方空D1不等C100数字，只巡检D2～D4；红方保留原巡检范围。 */
    for (depot = (g_mission_side == MISSION_COLOR_BLUE) ? 2U : 1U; depot <= 4U; ++depot) {
        if ((depot > 1U) &&
            !mission_test_send_wait(ctx, commands[depot - 2U],
                                    events[depot - 2U],
                                    MISSION_STATE_WAIT_DEPOT_1)) {
            return false;
        }
        g_wireless_test.depot_position = depot;
        if (!mission_test_read_depot_digit(ctx, &digit)) return false;
        mission_enter_state(ctx, MISSION_STATE_WAIT_DEPOT_1, 0U);
        if (digit == 0U) {
            (void)snprintf(g_wireless_test.text, sizeof(g_wireless_test.text),
                           "DIGIT D%u=NONE\r\n", (unsigned)depot);
        } else {
            (void)snprintf(g_wireless_test.text, sizeof(g_wireless_test.text),
                           "DIGIT D%u=%u\r\n",
                           (unsigned)depot, (unsigned)digit);
        }
        mission_test_write(g_wireless_test.text);
    }
    return true;
}

/** 只等待时间并轮询查询/STOP，不阻塞无线接收或借延时推定机械臂完成。 */
static bool mission_test_block_delay(mission_context_t *ctx, uint32_t delay_ms)
{
    char command[DEBUG_UART1_RX_BUFFER_SIZE];
    uint32_t deadline = osKernelGetTickCount() + mission_ms_to_ticks(delay_ms);
    while ((int32_t)(deadline - osKernelGetTickCount()) > 0) {
        if (mission_test_take_command(command, sizeof(command)) &&
            !mission_test_handle_aux_command(ctx, command)) mission_test_write("BUSY\r\n");
        if (g_wireless_test.stop_requested || (ctx->state == MISSION_STATE_FAULT)) return false;
        osDelay(mission_ms_to_ticks(MISSION_WIRELESS_POLL_MS));
    }
    return true;
}

/** 每点新SID；F7只有通信看门狗，NO_VALID业务限时由Nano用新鲜帧完成。 */
static bool mission_test_read_block_digit(mission_context_t *ctx, nano_vision_block_result_t *result)
{
    char command[DEBUG_UART1_RX_BUFFER_SIZE];
    uint32_t flags;
    bool aborting = false;
    ctx->block_result_received = false;
    if (!mission_start_vision(ctx, MISSION_VISION_SCENE_BLOCK_DIGIT,
            MISSION_STAIR_NONE, MISSION_STATE_BLOCK_WAIT_DIGIT)) {
        mission_fail(ctx, MISSION_FAULT_VISION);
        return false;
    }
    for (;;) {
        flags = osThreadFlagsWait(MISSION_FLAG_VISION_DONE, osFlagsWaitAny,
                                  mission_ms_to_ticks(MISSION_WIRELESS_POLL_MS));
        if (((flags & osFlagsError) == 0U) && ((flags & MISSION_FLAG_VISION_DONE) != 0U))
            mission_handle_vision(ctx);
        if (mission_test_take_command(command, sizeof(command)) &&
            !mission_test_handle_aux_command(ctx, command)) mission_test_write("BUSY\r\n");
        if (ctx->state == MISSION_STATE_FAULT) return false;
        if (g_wireless_test.stop_requested && !aborting) {
            aborting = true;
            if (!mission_stop_vision(ctx)) return false;
        }
        if (aborting && (ctx->vision.phase == MISSION_VISION_IDLE)) return false;
        if (!aborting && ctx->block_result_received &&
            (ctx->vision.phase == MISSION_VISION_IDLE)) {
            *result = ctx->block_result;
            return true; /* ACK发送成功后才允许移动或处理设备故障。 */
        }
        mission_vision_process(ctx);
        mission_check_timeout(ctx);
    }
}

static bool mission_test_block_move(mission_context_t *ctx, uint8_t point)
{
    static const mission_command_type_t commands[] = {MISSION_CMD_GO_DEPOT_1,
        MISSION_CMD_GO_DEPOT_2, MISSION_CMD_GO_DEPOT_3, MISSION_CMD_GO_DEPOT_4};
    static const chassis_command_type_t events[] = {CHASSIS_CMD_DEPOT_1_READY,
        CHASSIS_CMD_DEPOT_2_READY, CHASSIS_CMD_DEPOT_3_READY, CHASSIS_CMD_DEPOT_4_READY};
    if (g_wireless_test.depot_position == point) return true;
    if (!mission_test_send_wait(ctx, commands[point - 1U], events[point - 1U],
            MISSION_STATE_WAIT_DEPOT_1)) return false;
    g_wireless_test.depot_position = point;
    return true;
}

/** 三层每行至多一个、红D4/蓝D1初始空；旧测试只识别/跳站，不抓放。 */
static bool mission_test_run_block_digits(mission_context_t *ctx)
{
    static const uint8_t groups[] = {MISSION_BLOCK_HIGH_VISION_GROUP,
        MISSION_BLOCK_MID_VISION_GROUP, MISSION_BLOCK_LOW_VISION_GROUP};
    static const char *const reasons[] = {"NONE", "CONFIRMED", "NO_CANDIDATE",
        "LOW_SCORE", "AMBIGUOUS", "UNSTABLE", "CAMERA_NO_FRAME", "FRAME_GAP",
        "STALE_FRAME", "INSUFFICIENT_FRAMES", "MODEL_ERROR", "MODE_NOT_READY", "CAMERA_ERROR"};
    nano_vision_block_result_t result;
    uint8_t stage, step, row, point, found_mask = 0U;
    for (stage = 0U; stage < 3U; ++stage) {
        row = (uint8_t)(3U - stage);
        /* 首层确认10；用户确认28/29/30可直接切换，仍逐组等待完成回报。 */
        if (((stage == 0U) &&
             !mission_test_run_arm_group(ctx, MISSION_HOME_ACTION_GROUP)) ||
            !mission_test_run_arm_group(ctx, groups[stage])) {
            if (!g_wireless_test.stop_requested) mission_fail(ctx, MISSION_FAULT_ARM);
            return false;
        }
        for (step = 0U; step < 3U; ++step) {
            point = mission_block_scan_point_id(stage, step);
            if (!mission_test_block_move(ctx, point) ||
                !mission_test_block_delay(ctx, MISSION_BLOCK_SETTLE_MS) ||
                !mission_test_read_block_digit(ctx, &result)) return false;
            (void)snprintf(g_wireless_test.text, sizeof(g_wireless_test.text),
                "BLOCK ROW=%u D%u DIGIT=%u TARGET_ROW=%u RESULT=%s REASON=%s FRAMES=%u ELAPSED=%ums SID=%u\r\n",
                (unsigned)row, (unsigned)point, (unsigned)result.digit,
                (unsigned)result.digit,
                (result.status == NANO_VISION_BLOCK_DIGIT) ? "DIGIT" :
                ((result.status == NANO_VISION_BLOCK_NO_VALID) ? "NO_VALID" : "FAULT"),
                reasons[result.reason], (unsigned)result.frames,
                (unsigned)result.elapsed_ms, (unsigned)result.session_id);
            mission_test_write(g_wireless_test.text);
            if (result.status == NANO_VISION_BLOCK_FAULT) {
                mission_fail(ctx, MISSION_FAULT_VISION);
                return false;
            }
            if (result.status == NANO_VISION_BLOCK_DIGIT) {
                found_mask |= (uint8_t)(1U << (row - 1U));
                if (!mission_test_block_delay(ctx, MISSION_BLOCK_FOUND_PAUSE_MS)) return false;
                break; /* 停1秒后直达本方空列，该层后续位置不再识别。 */
            }
        }
        if ((found_mask & (1U << (row - 1U))) == 0U) {
            (void)snprintf(g_wireless_test.text, sizeof(g_wireless_test.text),
                "BLOCK ROW=%u NOT_FOUND TO_D%u\r\n", (unsigned)row,
                (unsigned)mission_block_place_point());
            mission_test_write(g_wireless_test.text);
        }
        if (!mission_test_block_move(ctx, mission_block_place_point())) return false;
    }
    if (!mission_test_run_arm_group(ctx, MISSION_HOME_ACTION_GROUP)) {
        if (!g_wireless_test.stop_requested) mission_fail(ctx, MISSION_FAULT_ARM);
        return false;
    }
    if (!mission_test_send_wait(ctx, MISSION_CMD_DEPOT_OK, CHASSIS_CMD_HOME_READY,
                               MISSION_STATE_DEPOT_WAIT_HOME)) return false;
    g_wireless_test.expected = MISSION_TEST_STAGE_DONE;
    mission_enter_state(ctx, MISSION_STATE_COMPLETE, 0U);
    (void)snprintf(g_wireless_test.text, sizeof(g_wireless_test.text),
        "DONE BLOCK DIGIT HOME FOUND_ROWS_MASK=0x%02X\r\n", (unsigned)found_mask);
    mission_test_write(g_wireless_test.text);
    return true;
}

/** 无线只负责轮询事件/STOP；抓放决策与正式共用，不能用延时推定完成。 */
static bool mission_test_run_block_move(mission_context_t *ctx)
{
    chassis_mission_event_t event;
    char command[DEBUG_UART1_RX_BUFFER_SIZE];
    uint32_t flags;
    mission_block_begin(ctx);
    while ((ctx->state != MISSION_STATE_COMPLETE) &&
           (ctx->state != MISSION_STATE_FAULT)) {
        flags = osThreadFlagsWait(MISSION_ALL_FLAGS, osFlagsWaitAny,
                                  mission_ms_to_ticks(MISSION_WIRELESS_POLL_MS));
        if (mission_test_take_command(command, sizeof(command)) &&
            !mission_test_handle_aux_command(ctx, command)) mission_test_write("BUSY\r\n");
        if (g_wireless_test.stop_requested) {
            /* 先消费在途视觉回调并关闭会话，STOP底盘回报留给外层等待。 */
            if (((flags & osFlagsError) == 0U) && ((flags & MISSION_FLAG_VISION_DONE) != 0U))
                mission_handle_vision(ctx);
        } else if ((flags & osFlagsError) == 0U) {
            if ((flags & CHASSIS_MISSION_FLAG_EVENT) != 0U) {
                while (osMessageQueueGet(mission_event_queue, &event, NULL, 0U) == osOK)
                    mission_handle_chassis(ctx, &event);
                g_wireless_test.depot_position = ctx->depot_position;
            }
            if ((flags & MISSION_FLAG_ARM_FAIL) != 0U) mission_handle_arm(ctx, false);
            else if ((flags & MISSION_FLAG_ARM_OK) != 0U) mission_handle_arm(ctx, true);
            if ((flags & MISSION_FLAG_VISION_DONE) != 0U) mission_handle_vision(ctx);
        }
        mission_vision_process(ctx);
        mission_check_timeout(ctx);
        if (g_wireless_test.stop_requested && (ctx->vision.phase == MISSION_VISION_IDLE))
            return false;
    }
    if (ctx->state != MISSION_STATE_COMPLETE) return false;
    g_wireless_test.expected = MISSION_TEST_STAGE_DONE;
    BLOCK_TRACE("DONE BLOCK MOVE D1 HOME FOUND_ROWS_MASK=0x%02X PLACED_ROWS_MASK=0x%02X\r\n",
        (unsigned)ctx->block_found_mask, (unsigned)ctx->block_placed_mask);
    return true;
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
    uint8_t fine_used;
    bool ok;
    bool place_balls;

    (void)memset(&g_wireless_test, 0, sizeof(g_wireless_test));
    if (!debug_uart1_init(&g_wireless_test.debug)) {
        mission_enter_state(ctx, MISSION_STATE_FAULT, 0U);
        ctx->fault_code = MISSION_FAULT_PROTOCOL;
        return;
    }
    mission_test_write("MISSION WIRELESS TEST BOOT\r\nSELECT RED OR BLUE\r\n");

    /* 先由无线指令选色，再允许底盘开始握手；本轮颜色此后保持不变。 */
    while (g_mission_side == MISSION_COLOR_NONE) {
        if (mission_test_take_command(command, sizeof(command))) {
            if (strcmp(command, "RED") == 0) {
                g_mission_side = MISSION_COLOR_RED;
            } else if (strcmp(command, "BLUE") == 0) {
                g_mission_side = MISSION_COLOR_BLUE;
            } else if ((strcmp(command, "HELP") == 0) ||
                       (strcmp(command, "STATUS") == 0)) {
                (void)mission_test_handle_aux_command(ctx, command);
            } else {
                mission_test_write("ERR SELECT RED OR BLUE\r\n");
            }
        }
        osDelay(mission_ms_to_ticks(MISSION_WIRELESS_POLL_MS));
    }
    (void)snprintf(
        g_wireless_test.text, sizeof(g_wireless_test.text),
        "OK COLOR=%s WAIT CHASSIS\r\n",
        mission_test_color_name(g_mission_side));
    mission_test_write(g_wireless_test.text);

    /* 选色后留在起点，动作10完成、底盘就绪、模型预加载均成功才握手。 */
    mission_enter_state(ctx, MISSION_STATE_WAIT_CHASSIS_READY, 0U);
    mission_test_write("WAIT ARM 10 + CHASSIS + BLOCK MODEL\r\n");
    while ((chassis_ready == 0U) || !ctx->arm_home_ready || !ctx->block_model_ready) {
        mission_model_process(ctx);
        flags = osThreadFlagsWait(
            CHASSIS_MISSION_FLAG_EVENT | MISSION_FLAG_ARM_OK |
                MISSION_FLAG_ARM_FAIL | MISSION_FLAG_VISION_DONE,
            osFlagsWaitAny,
            mission_ms_to_ticks(MISSION_WIRELESS_POLL_MS));
        if ((flags & osFlagsError) == 0U) {
        if ((flags & MISSION_FLAG_ARM_FAIL) != 0U) {
            g_wireless_test.arm_ready = false;
            mission_fail(ctx, MISSION_FAULT_ARM);
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
                    ctx->chassis_ready = true;
                }
            }
        }
        if ((flags & MISSION_FLAG_VISION_DONE) != 0U) mission_handle_vision(ctx);
        }
        if (mission_test_take_command(command, sizeof(command)) &&
            !mission_test_handle_aux_command(ctx, command)) mission_test_write("WAIT START GATE\r\n");
        if (g_wireless_test.stop_requested || (ctx->state == MISSION_STATE_FAULT)) {
            (void)arm_stop(NULL, NULL);
            mission_test_write("START GATE ABORTED RESET REQUIRED\r\n");
            break;
        }
    }
    if (g_wireless_test.stop_requested || (ctx->state == MISSION_STATE_FAULT)) {
        for (;;) {
            if (ctx->vision.completion_pending) mission_handle_vision(ctx);
            if (mission_test_take_command(command, sizeof(command)))
                (void)mission_test_handle_aux_command(ctx, command);
            osDelay(mission_ms_to_ticks(MISSION_WIRELESS_POLL_MS));
        }
    }
    if (!mission_send_chassis(MISSION_CMD_MISSION_READY, ctx->request_id)) {
        mission_fail(ctx, MISSION_FAULT_QUEUE);
        mission_test_write("FAULT LINK\r\n");
        return;
    }
    mission_enter_state(ctx, MISSION_STATE_READY, 0U);
    g_wireless_test.expected = MISSION_TEST_STAGE_PLATFORM;
    mission_test_write("READY ARM=10 MODEL=READY PLATFORM STAIR DISC DEPOT OR ROUTE\r\n");

    for (;;) {
        /* 空闲时继续接收机械臂回报，姿态缓存仍以实际完成回报为准。 */
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
        if ((strcmp(command, "RED") == 0) ||
            (strcmp(command, "BLUE") == 0)) {
            mission_test_write("ERR COLOR LOCKED\r\n");
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
            (ctx->state == MISSION_STATE_FAULT) ||
            ((ctx->state == MISSION_STATE_COMPLETE) &&
             (g_wireless_test.target != MISSION_TEST_STAGE_DEPOT))) {
            mission_test_write("ERR RESET REQUIRED\r\n");
            continue;
        }

        if (mission_test_disc_path_command(ctx, command)) continue;
        /* 底盘正等待START时，不允许DEPOT或其他路线越过这一等待点。 */
        if (g_wireless_test.small_disc_at_start) {
            mission_test_write("ERR WAIT DISC_RUN\r\n");
            continue;
        }

        if (strcmp(command, "BALL HOME") == 0) {
            if (!g_wireless_test.depot_wait_ball_home ||
                (g_wireless_test.depot_position != 1U) ||
                g_wireless_test.ball_home) {
                mission_test_write("ERR WAIT ROUTE DEPOT BALL D1\r\n");
                continue;
            }
            if (!g_wireless_test.arm_ready) {
                mission_test_write("ERR ARM UNAVAILABLE\r\n");
                continue;
            }
            if (!mission_prepare_zdt(ctx)) {
                mission_fail(ctx, MISSION_FAULT_STORAGE);
                mission_test_write("FAULT STORAGE INIT\r\n");
                continue;
            }
            if (!mission_test_run_arm_group(
                    ctx, MISSION_HOME_ACTION_GROUP)) {
                if (!g_wireless_test.stop_requested) {
                    mission_fail(ctx, MISSION_FAULT_ARM);
                }
                mission_test_write("FAULT BALL HOME ARM 10\r\n");
                continue;
            }
            /* 人工先将1号槽对准取球工位；本指令不执行绝对归零。 */
            g_wireless_test.depot_wait_ball_home = false;
            g_wireless_test.ball_home = true;
            g_wireless_test.current_slot = 1U;
            mission_test_write(
                "BALL HOME AUTO START ASSUME SLOT=1\r\n");
            ok = mission_test_ball_home_load(ctx);
            if (ok) ok = mission_test_run_depot_balls(ctx);
            if (ok) {
                g_wireless_test.expected = MISSION_TEST_STAGE_DEPOT;
                mission_test_write(
                    "DONE DEPOT BALL\r\nREADY HOME HOME_DIRECT\r\n");
            } else if (!g_wireless_test.stop_requested) {
                mission_fail(ctx, MISSION_FAULT_STORAGE);
                mission_test_write("FAULT ROUTE\r\n");
            }
            continue;
        }
        if ((strcmp(command, "TURN CW") == 0) ||
            (strcmp(command, "TURN CCW") == 0)) {
            if ((g_wireless_test.mode != MISSION_TEST_MODE_IDLE) ||
                g_wireless_test.ball_home || !g_wireless_test.arm_ready) {
                mission_test_write("ERR TURN REQUIRES IDLE ARM READY\r\n");
                continue;
            }
            if (!mission_prepare_zdt(ctx)) {
                mission_fail(ctx, MISSION_FAULT_STORAGE);
                mission_test_write("FAULT STORAGE INIT\r\n");
                continue;
            }
            if (!mission_test_run_arm_group(
                    ctx, MISSION_HOME_ACTION_GROUP)) {
                if (!g_wireless_test.stop_requested) {
                    mission_fail(ctx, MISSION_FAULT_ARM);
                }
                mission_test_write("FAULT TURN ARM 10\r\n");
                continue;
            }
            fine_used = 0U;
            /* 独立逐槽测试不更新装球槽号；反转时粗调、微调均为CCW。 */
            ok = mission_advance_slot(ctx,
                (strcmp(command, "TURN CW") == 0) ?
                    ZDT_TURNTABLE_DIR_CW : ZDT_TURNTABLE_DIR_CCW,
                &fine_used);
            if (!ok) {
                (void)turn_stop(NULL, NULL);
                if (!g_wireless_test.stop_requested) {
                    mission_fail(ctx, MISSION_FAULT_STORAGE);
                }
                mission_test_write("FAULT TURN SLOT UNKNOWN\r\n");
            } else {
                if (!mission_test_run_arm_group(ctx, MISSION_HOME_ACTION_GROUP)) {
                    if (!g_wireless_test.stop_requested) {
                        mission_fail(ctx, MISSION_FAULT_ARM);
                    }
                    mission_test_write("FAULT TURN ARM 10\r\n");
                    continue;
                }
                (void)snprintf(g_wireless_test.text,
                               sizeof(g_wireless_test.text),
                               "TURN %s REACHED PB0=OPTIONAL FINE=%u\r\n",
                               (strcmp(command, "TURN CW") == 0) ? "CW" : "CCW",
                               (unsigned)fine_used);
                mission_test_write(g_wireless_test.text);
            }
            continue;
        }

        if ((strcmp(command, "ROUTE BLOCK DIGIT") == 0) ||
            (strcmp(command, "ROUTE BLOCK MOVE") == 0)) {
            if (g_wireless_test.mode != MISSION_TEST_MODE_IDLE) {
                mission_test_write("ERR RESET REQUIRED\r\n");
                continue;
            }
            if (!ctx->arm_home_ready || !ctx->block_model_ready) {
                mission_test_write("ERR WAIT ARM 10 MODEL READY\r\n");
                continue;
            }
            g_wireless_test.mode = MISSION_TEST_MODE_TARGET;
            g_wireless_test.target = MISSION_TEST_STAGE_DEPOT;
            g_wireless_test.block_move = strcmp(command, "ROUTE BLOCK MOVE") == 0;
            mission_test_write(g_wireless_test.block_move ?
                "OK ROUTE BLOCK MOVE GRASP PLACE D1 HOME NO_BALL_PLACE\r\n" :
                "OK ROUTE BLOCK DIGIT NO_GRASP NO_PLACE\r\n");
            ok = mission_test_run_target(ctx, MISSION_TEST_STAGE_DEPOT) &&
                 (g_wireless_test.block_move ? mission_test_run_block_move(ctx) :
                                               mission_test_run_block_digits(ctx));
            if (!ok && !g_wireless_test.stop_requested) {
                if (ctx->state != MISSION_STATE_FAULT) mission_fail(ctx, MISSION_FAULT_CHASSIS);
                mission_test_write(g_wireless_test.block_move ?
                    "FAULT BLOCK MOVE\r\n" : "FAULT BLOCK DIGIT\r\n");
            } else if (g_wireless_test.stop_requested) {
                (void)mission_test_wait_chassis_event(ctx, CHASSIS_CMD_STOPPED, ctx->request_id);
            }
            continue;
        }

        /* 1) ROUTE命令自动跳过前置区域，只在目标区域执行正式业务。 */
        if ((strcmp(command, "ROUTE PLATFORM") == 0) ||
            (strcmp(command, "ROUTE STAIRS") == 0) ||
            (strcmp(command, "ROUTE DISC") == 0) ||
            (strcmp(command, "ROUTE DEPOT") == 0) ||
            (strcmp(command, "ROUTE DEPOT BALL") == 0)) {
            place_balls = (strcmp(command, "ROUTE DEPOT BALL") == 0);
            if (g_wireless_test.mode != MISSION_TEST_MODE_IDLE) {
                mission_test_write("ERR RESET REQUIRED\r\n");
                continue;
            }
            if (place_balls && !g_wireless_test.arm_ready) {
                mission_test_write("ERR ARM UNAVAILABLE\r\n");
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
            PLATFORM_TRACE("[P] DIAG ON TICK_HZ=%lu MAX_BALLS=5 MAX_TRIES=7\r\n",
                           (unsigned long)osKernelGetTickFreq());
            ctx->platform_balls = 0U;
            ctx->platform_attempts = 0U;
            ctx->platform_read_ok = false;
            ctx->current_slot = 0U;
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
            if (ok && place_balls) {
                /* [lyx] D1停车后等待人工确认BALL HOME，避免出发前转盘改变车体姿态。 */
                g_wireless_test.depot_wait_ball_home = true;
                mission_test_write("READY D1 SEND BALL HOME\r\n");
                continue;
            }
            if (ok && (g_wireless_test.target == MISSION_TEST_STAGE_DEPOT)) {
                ok = mission_test_run_depot_digits(ctx);
            }
            if (ok) {
                g_wireless_test.expected =
                    (g_wireless_test.target == MISSION_TEST_STAGE_DEPOT)
                    ? MISSION_TEST_STAGE_DEPOT : MISSION_TEST_STAGE_DONE;
                (void)snprintf(
                    g_wireless_test.text,
                    sizeof(g_wireless_test.text),
                    "DONE %s%s\r\n",
                    mission_test_stage_name(g_wireless_test.target),
                    place_balls ? " BALL" : "");
                mission_test_write(g_wireless_test.text);
                if (g_wireless_test.target == MISSION_TEST_STAGE_DEPOT) {
                    mission_test_write(
                        "READY HOME HOME_DIRECT\r\n");
                }
            } else if (!g_wireless_test.stop_requested) {
                mission_fail(ctx, place_balls ? MISSION_FAULT_STORAGE :
                                                MISSION_FAULT_CHASSIS);
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
#if MISSION_CHASSIS_ROUTE_TEST_ENABLED
    g_mission_side = MISSION_COLOR_NONE;
#else
    /* 正式假按钮按构建选色，两方都走同一正式Mission；实体按键尚未接入。 */
    g_mission_side = MISSION_FORMAL_BLUE_SIDE ? MISSION_COLOR_BLUE : MISSION_COLOR_RED;
#endif
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
    snapshot->current_slot = ctx->current_slot;
    snapshot->abnormal_ball_mask = ctx->depot_abnormal_mask;
    taskEXIT_CRITICAL();
    return MISSION_APP_OK;
}
