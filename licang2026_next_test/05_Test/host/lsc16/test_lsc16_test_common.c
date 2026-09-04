#include <assert.h>
#include <stdio.h>
#include <string.h>

#include "lsc16_test_common.h"

int main(void)
{
    lsc16_report_t report = {0};
    lsc16_test_action_guard_t guard;
    char text[96];

    assert(lsc16_test_parse_command(
        (const uint8_t *)"servo\r\n", 7U) == LSC16_TEST_COMMAND_SERVO);
    assert(lsc16_test_parse_command(
        (const uint8_t *)"ACTION ", 7U) == LSC16_TEST_COMMAND_ACTION);
    assert(lsc16_test_parse_command(
        (const uint8_t *)"stop", 4U) == LSC16_TEST_COMMAND_STOP);
    assert(lsc16_test_parse_command(
        (const uint8_t *)"battery", 7U) == LSC16_TEST_COMMAND_BATTERY);
    assert(lsc16_test_parse_command(
        (const uint8_t *)"STATUS", 6U) == LSC16_TEST_COMMAND_STATUS);
    assert(lsc16_test_parse_command(
        (const uint8_t *)"MOVE", 4U) == LSC16_TEST_COMMAND_INVALID);

    report.action_group = 3U;
    report.repeat_count = 1U;
    assert(lsc16_test_format_report(
        LSC16_REPORT_EVENT_ACTION_STARTED,
        &report,
        text,
        sizeof(text)) > 0U);
    assert(strcmp(text, "ACTION STARTED GROUP=3 REPEAT=1\r\n") == 0);

    report.battery_mv = 7400U;
    assert(lsc16_test_format_report(
        LSC16_REPORT_EVENT_BATTERY_UPDATED,
        &report,
        text,
        sizeof(text)) > 0U);
    assert(strcmp(text, "BATTERY 7400 mV\r\n") == 0);

    lsc16_test_action_guard_init(&guard);
    assert(guard.phase == LSC16_TEST_ACTION_PHASE_IDLE);
    assert(!guard.fault_latched);
    assert(lsc16_test_action_guard_begin(&guard, 100U, 20U) == LSC16_OK);
    assert(lsc16_test_action_guard_is_active(&guard));
    assert(lsc16_test_action_guard_begin(&guard, 101U, 20U) == LSC16_ERR_BUSY);
    assert(guard.repeated_reject_count == 1U);
    assert(lsc16_test_action_guard_poll(&guard, 119U) ==
           LSC16_TEST_ACTION_TIMEOUT_NONE);
    lsc16_test_action_guard_on_tx_done(&guard, 120U, 30U);
    assert(guard.phase == LSC16_TEST_ACTION_PHASE_WAIT_STARTED);
    lsc16_test_action_guard_on_started(&guard, 130U, 40U);
    assert(guard.phase == LSC16_TEST_ACTION_PHASE_WAIT_COMPLETED);
    assert(lsc16_test_action_guard_poll(&guard, 169U) ==
           LSC16_TEST_ACTION_TIMEOUT_NONE);
    lsc16_test_action_guard_on_completed(&guard);
    assert(guard.phase == LSC16_TEST_ACTION_PHASE_IDLE);
    assert(!guard.fault_latched);

    assert(lsc16_test_action_guard_begin(&guard, 200U, 10U) == LSC16_OK);
    assert(lsc16_test_action_guard_poll(&guard, 210U) ==
           LSC16_TEST_ACTION_TIMEOUT_TX);
    assert(guard.phase == LSC16_TEST_ACTION_PHASE_FAULT);
    assert(guard.fault_latched);
    assert(guard.timeout_count == 1U);
    assert(lsc16_test_action_guard_begin(&guard, 211U, 10U) ==
           LSC16_ERR_STATE);

    lsc16_test_action_guard_init(&guard);
    assert(lsc16_test_action_guard_begin(&guard, 0xFFFFFFF0U, 32U) ==
           LSC16_OK);
    assert(lsc16_test_action_guard_poll(&guard, 0x0000000FU) ==
           LSC16_TEST_ACTION_TIMEOUT_NONE);
    assert(lsc16_test_action_guard_poll(&guard, 0x00000010U) ==
           LSC16_TEST_ACTION_TIMEOUT_TX);

    lsc16_test_action_guard_init(&guard);
    assert(lsc16_test_action_guard_begin(&guard, 300U, 10U) == LSC16_OK);
    lsc16_test_action_guard_on_tx_done(&guard, 301U, 10U);
    assert(lsc16_test_action_guard_poll(&guard, 311U) ==
           LSC16_TEST_ACTION_TIMEOUT_STARTED);

    lsc16_test_action_guard_init(&guard);
    assert(lsc16_test_action_guard_begin(&guard, 400U, 10U) == LSC16_OK);
    lsc16_test_action_guard_on_started(&guard, 401U, 10U);
    assert(lsc16_test_action_guard_poll(&guard, 411U) ==
           LSC16_TEST_ACTION_TIMEOUT_COMPLETED);

    puts("LSC16 USART1 command and action guard tests passed");
    return 0;
}
