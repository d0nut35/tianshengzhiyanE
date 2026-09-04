/** @file lsc16_test_common.c @brief LSC16命令与回报文本公共实现。 */

#include "lsc16_test_common.h"

#include <stdbool.h>
#include <stdio.h>
#include <string.h>

static bool lsc16_test_time_reached(uint32_t now_ms, uint32_t deadline_ms)
{
    return (int32_t)(now_ms - deadline_ms) >= 0;
}

static bool lsc16_test_command_equals(
    const uint8_t *data,
    size_t len,
    const char *expected)
{
    size_t i;
    size_t expected_len;

    while ((len > 0U) && ((data[len - 1U] == '\r') ||
                          (data[len - 1U] == '\n') ||
                          (data[len - 1U] == ' ') ||
                          (data[len - 1U] == '\t'))) {
        --len;
    }
    expected_len = strlen(expected);
    if (len != expected_len) {
        return false;
    }
    for (i = 0U; i < len; ++i) {
        uint8_t ch = data[i];
        if ((ch >= 'a') && (ch <= 'z')) {
            ch = (uint8_t)(ch - ('a' - 'A'));
        }
        if (ch != (uint8_t)expected[i]) {
            return false;
        }
    }
    return true;
}

lsc16_test_command_t lsc16_test_parse_command(
    const uint8_t *data,
    size_t len)
{
    if ((data == NULL) || (len == 0U)) {
        return LSC16_TEST_COMMAND_INVALID;
    }
    if (lsc16_test_command_equals(data, len, "SERVO")) {
        return LSC16_TEST_COMMAND_SERVO;
    }
    if (lsc16_test_command_equals(data, len, "ACTION")) {
        return LSC16_TEST_COMMAND_ACTION;
    }
    if (lsc16_test_command_equals(data, len, "STOP")) {
        return LSC16_TEST_COMMAND_STOP;
    }
    if (lsc16_test_command_equals(data, len, "BATTERY")) {
        return LSC16_TEST_COMMAND_BATTERY;
    }
    if (lsc16_test_command_equals(data, len, "STATUS")) {
        return LSC16_TEST_COMMAND_STATUS;
    }
    return LSC16_TEST_COMMAND_INVALID;
}

const char *lsc16_test_command_name(lsc16_test_command_t command)
{
    switch (command) {
    case LSC16_TEST_COMMAND_SERVO: return "SERVO";
    case LSC16_TEST_COMMAND_ACTION: return "ACTION";
    case LSC16_TEST_COMMAND_STOP: return "STOP";
    case LSC16_TEST_COMMAND_BATTERY: return "BATTERY";
    case LSC16_TEST_COMMAND_STATUS: return "STATUS";
    default: return "INVALID";
    }
}

size_t lsc16_test_format_report(
    uint32_t report_events,
    const lsc16_report_t *report,
    char *text,
    size_t capacity)
{
    int count;

    if ((report == NULL) || (text == NULL) || (capacity == 0U)) {
        return 0U;
    }
    if ((report_events & LSC16_REPORT_EVENT_INVALID_FRAME) != 0U) {
        count = snprintf(text, capacity, "LSC16 ERROR INVALID REPORT\r\n");
    } else if ((report_events & LSC16_REPORT_EVENT_BATTERY_UPDATED) != 0U) {
        count = snprintf(text, capacity, "BATTERY %u mV\r\n",
                         (unsigned)report->battery_mv);
    } else if ((report_events & LSC16_REPORT_EVENT_ACTION_COMPLETED) != 0U) {
        count = snprintf(text, capacity,
                         "ACTION COMPLETED GROUP=%u REPEAT=%u\r\n",
                         (unsigned)report->action_group,
                         (unsigned)report->repeat_count);
    } else if ((report_events & LSC16_REPORT_EVENT_ACTION_STOPPED) != 0U) {
        count = snprintf(text, capacity, "ACTION STOPPED\r\n");
    } else if ((report_events & LSC16_REPORT_EVENT_ACTION_STARTED) != 0U) {
        count = snprintf(text, capacity,
                         "ACTION STARTED GROUP=%u REPEAT=%u\r\n",
                         (unsigned)report->action_group,
                         (unsigned)report->repeat_count);
    } else {
        return 0U;
    }
    if ((count <= 0) || ((size_t)count >= capacity)) {
        return 0U;
    }
    return (size_t)count;
}

void lsc16_test_action_guard_init(lsc16_test_action_guard_t *guard)
{
    if (guard != NULL) {
        (void)memset(guard, 0, sizeof(*guard));
        guard->phase = LSC16_TEST_ACTION_PHASE_IDLE;
    }
}

lsc16_status_t lsc16_test_action_guard_begin(
    lsc16_test_action_guard_t *guard,
    uint32_t now_ms,
    uint32_t tx_timeout_ms)
{
    if ((guard == NULL) || (tx_timeout_ms == 0U)) {
        return LSC16_ERR_PARAM;
    }
    if (guard->fault_latched) {
        return LSC16_ERR_STATE;
    }
    if (guard->phase != LSC16_TEST_ACTION_PHASE_IDLE) {
        ++guard->repeated_reject_count;
        return LSC16_ERR_BUSY;
    }
    guard->phase = LSC16_TEST_ACTION_PHASE_WAIT_TX;
    guard->deadline_ms = now_ms + tx_timeout_ms;
    return LSC16_OK;
}

void lsc16_test_action_guard_cancel_begin(lsc16_test_action_guard_t *guard)
{
    if ((guard != NULL) &&
        (guard->phase == LSC16_TEST_ACTION_PHASE_WAIT_TX) &&
        !guard->fault_latched) {
        guard->phase = LSC16_TEST_ACTION_PHASE_IDLE;
        guard->deadline_ms = 0U;
    }
}

void lsc16_test_action_guard_on_tx_done(
    lsc16_test_action_guard_t *guard,
    uint32_t now_ms,
    uint32_t started_timeout_ms)
{
    if ((guard != NULL) &&
        (guard->phase == LSC16_TEST_ACTION_PHASE_WAIT_TX) &&
        (started_timeout_ms > 0U)) {
        guard->phase = LSC16_TEST_ACTION_PHASE_WAIT_STARTED;
        guard->deadline_ms = now_ms + started_timeout_ms;
    }
}

void lsc16_test_action_guard_on_started(
    lsc16_test_action_guard_t *guard,
    uint32_t now_ms,
    uint32_t completed_timeout_ms)
{
    if ((guard != NULL) && !guard->fault_latched &&
        ((guard->phase == LSC16_TEST_ACTION_PHASE_WAIT_TX) ||
         (guard->phase == LSC16_TEST_ACTION_PHASE_WAIT_STARTED)) &&
        (completed_timeout_ms > 0U)) {
        guard->phase = LSC16_TEST_ACTION_PHASE_WAIT_COMPLETED;
        guard->deadline_ms = now_ms + completed_timeout_ms;
    }
}

void lsc16_test_action_guard_on_completed(lsc16_test_action_guard_t *guard)
{
    if ((guard != NULL) && !guard->fault_latched &&
        (guard->phase != LSC16_TEST_ACTION_PHASE_IDLE)) {
        guard->phase = LSC16_TEST_ACTION_PHASE_IDLE;
        guard->deadline_ms = 0U;
    }
}

void lsc16_test_action_guard_on_stopped(lsc16_test_action_guard_t *guard)
{
    lsc16_test_action_guard_on_completed(guard);
}

void lsc16_test_action_guard_latch_fault(lsc16_test_action_guard_t *guard)
{
    if (guard != NULL) {
        guard->fault_latched = true;
        guard->phase = LSC16_TEST_ACTION_PHASE_FAULT;
        guard->deadline_ms = 0U;
    }
}

lsc16_test_action_timeout_t lsc16_test_action_guard_poll(
    lsc16_test_action_guard_t *guard,
    uint32_t now_ms)
{
    lsc16_test_action_timeout_t timeout = LSC16_TEST_ACTION_TIMEOUT_NONE;

    if ((guard == NULL) || guard->fault_latched ||
        (guard->phase == LSC16_TEST_ACTION_PHASE_IDLE) ||
        !lsc16_test_time_reached(now_ms, guard->deadline_ms)) {
        return LSC16_TEST_ACTION_TIMEOUT_NONE;
    }
    if (guard->phase == LSC16_TEST_ACTION_PHASE_WAIT_TX) {
        timeout = LSC16_TEST_ACTION_TIMEOUT_TX;
    } else if (guard->phase == LSC16_TEST_ACTION_PHASE_WAIT_STARTED) {
        timeout = LSC16_TEST_ACTION_TIMEOUT_STARTED;
    } else if (guard->phase == LSC16_TEST_ACTION_PHASE_WAIT_COMPLETED) {
        timeout = LSC16_TEST_ACTION_TIMEOUT_COMPLETED;
    }
    if (timeout != LSC16_TEST_ACTION_TIMEOUT_NONE) {
        ++guard->timeout_count;
        lsc16_test_action_guard_latch_fault(guard);
    }
    return timeout;
}

bool lsc16_test_action_guard_is_active(
    const lsc16_test_action_guard_t *guard)
{
    return (guard != NULL) &&
        (guard->phase != LSC16_TEST_ACTION_PHASE_IDLE) &&
        (guard->phase != LSC16_TEST_ACTION_PHASE_FAULT);
}
