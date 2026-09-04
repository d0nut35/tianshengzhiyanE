/**
 * @file    lsc16_test_common.h
 * @brief   LSC16 USART1测试命令解析与回报文本格式化接口。
 */

#ifndef LSC16_TEST_COMMON_H
#define LSC16_TEST_COMMON_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "lsc16_core.h"

typedef enum {
    LSC16_TEST_COMMAND_INVALID = 0,
    LSC16_TEST_COMMAND_SERVO,
    LSC16_TEST_COMMAND_ACTION,
    LSC16_TEST_COMMAND_STOP,
    LSC16_TEST_COMMAND_BATTERY,
    LSC16_TEST_COMMAND_STATUS,
} lsc16_test_command_t;

/** 动作组测试的运行阶段；仅用于测试安全保护，不改变LSC16正式协议。 */
typedef enum {
    LSC16_TEST_ACTION_PHASE_IDLE = 0,
    LSC16_TEST_ACTION_PHASE_WAIT_TX,
    LSC16_TEST_ACTION_PHASE_WAIT_STARTED,
    LSC16_TEST_ACTION_PHASE_WAIT_COMPLETED,
    LSC16_TEST_ACTION_PHASE_FAULT,
} lsc16_test_action_phase_t;

/** 轮询动作保护器时返回的超时类型。 */
typedef enum {
    LSC16_TEST_ACTION_TIMEOUT_NONE = 0,
    LSC16_TEST_ACTION_TIMEOUT_TX,
    LSC16_TEST_ACTION_TIMEOUT_STARTED,
    LSC16_TEST_ACTION_TIMEOUT_COMPLETED,
} lsc16_test_action_timeout_t;

/**
 * @brief 动作组测试的运行期保护状态。
 * @note fault_latched一旦置位，只能通过重新初始化（复位）清除。
 */
typedef struct {
    lsc16_test_action_phase_t phase;
    bool fault_latched;
    uint32_t deadline_ms;
    uint32_t timeout_count;
    uint32_t repeated_reject_count;
} lsc16_test_action_guard_t;

/** @brief 解析USART1文本命令，忽略末尾空白和ASCII大小写。 */
lsc16_test_command_t lsc16_test_parse_command(
    const uint8_t *data,
    size_t len);

/** @brief 返回命令对应的固定大写名称。 */
const char *lsc16_test_command_name(lsc16_test_command_t command);

/**
 * @brief 将一组控制板主动回报格式化为串口助手可读文本。
 * @return 写入的字符数；参数或容量不合法时返回0。
 */
size_t lsc16_test_format_report(
    uint32_t report_events,
    const lsc16_report_t *report,
    char *text,
    size_t capacity);

/** @brief 初始化动作组测试保护器。 */
void lsc16_test_action_guard_init(lsc16_test_action_guard_t *guard);

/**
 * @brief 在提交ACTION前占用动作生命周期并启动TX超时。
 * @return OK表示允许提交；BUSY表示已有动作；STATE表示故障已锁存。
 */
lsc16_status_t lsc16_test_action_guard_begin(
    lsc16_test_action_guard_t *guard,
    uint32_t now_ms,
    uint32_t tx_timeout_ms);

/** @brief ACTION提交失败时回滚尚未启动的动作占用。 */
void lsc16_test_action_guard_cancel_begin(lsc16_test_action_guard_t *guard);

/** @brief 记录ACTION发送完成并切换到等待开始回报。 */
void lsc16_test_action_guard_on_tx_done(
    lsc16_test_action_guard_t *guard,
    uint32_t now_ms,
    uint32_t started_timeout_ms);

/** @brief 记录0x06开始回报并切换到等待0x08完成回报。 */
void lsc16_test_action_guard_on_started(
    lsc16_test_action_guard_t *guard,
    uint32_t now_ms,
    uint32_t completed_timeout_ms);

/** @brief 记录0x08自然完成回报并释放动作占用。 */
void lsc16_test_action_guard_on_completed(lsc16_test_action_guard_t *guard);

/** @brief 记录正常0x07停止回报；故障锁存状态不会被清除。 */
void lsc16_test_action_guard_on_stopped(lsc16_test_action_guard_t *guard);

/** @brief 将动作保护器置为必须复位才能清除的故障状态。 */
void lsc16_test_action_guard_latch_fault(lsc16_test_action_guard_t *guard);

/** @brief 轮询TX、开始回报和完成回报的阶段超时。 */
lsc16_test_action_timeout_t lsc16_test_action_guard_poll(
    lsc16_test_action_guard_t *guard,
    uint32_t now_ms);

/** @brief 查询动作是否仍处在发送或运行生命周期中。 */
bool lsc16_test_action_guard_is_active(
    const lsc16_test_action_guard_t *guard);

#endif /* LSC16_TEST_COMMON_H */
