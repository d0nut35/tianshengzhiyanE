#include <assert.h>
#include <stdio.h>
#include <string.h>

#include "mult_uart_test_protocol.h"

static void test_switch_commands(void)
{
    const uint8_t channel_0[] = {0xFFU, 0x00U, 0xAAU};
    const uint8_t channel_3[] = {0xFFU, 0x03U, 0xAAU};
    const uint8_t invalid[] = {0xFFU, 0x04U, 0xAAU};
    const uint8_t ordinary[] = {0xFFU, 0x01U, 0xAAU, 0x00U};
    mult_uart_test_input_t result;

    result = mult_uart_test_parse_input(channel_0, sizeof(channel_0));
    assert(result.kind == MULT_UART_TEST_INPUT_SWITCH);
    assert(result.requested_channel == 0U);

    result = mult_uart_test_parse_input(channel_3, sizeof(channel_3));
    assert(result.kind == MULT_UART_TEST_INPUT_SWITCH);
    assert(result.requested_channel == 3U);

    result = mult_uart_test_parse_input(invalid, sizeof(invalid));
    assert(result.kind == MULT_UART_TEST_INPUT_INVALID_CHANNEL);
    assert(result.requested_channel == 4U);

    result = mult_uart_test_parse_input(ordinary, sizeof(ordinary));
    assert(result.kind == MULT_UART_TEST_INPUT_ECHO);
}

static void test_text_format(void)
{
    uint8_t output[MULT_UART_TEST_TX_CAPACITY];
    const uint8_t input[] = {0x12U, 0x34U, 0xABU};
    size_t len;

    len = mult_uart_test_format_announcement(
        2U,
        output,
        sizeof(output));
    assert(len == strlen("now is channel 2\r\n"));
    assert(memcmp(output, "now is channel 2\r\n", len) == 0);

    len = mult_uart_test_format_echo(
        0U,
        input,
        sizeof(input),
        output,
        sizeof(output));
    assert(len == strlen("channel 0:12 34 AB\r\n"));
    assert(memcmp(output, "channel 0:12 34 AB\r\n", len) == 0);

    len = mult_uart_test_format_invalid_channel(
        output,
        sizeof(output));
    assert(len == strlen("invalid channel\r\n"));
    assert(memcmp(output, "invalid channel\r\n", len) == 0);
}

static void test_capacity_guards(void)
{
    uint8_t output[8];
    const uint8_t input[] = {0x01U};

    assert(mult_uart_test_format_announcement(
        0U,
        output,
        sizeof(output)) == 0U);
    assert(mult_uart_test_format_echo(
        0U,
        input,
        sizeof(input),
        output,
        sizeof(output)) == 0U);
    assert(mult_uart_test_format_invalid_channel(
        output,
        sizeof(output)) == 0U);
}

int main(void)
{
    test_switch_commands();
    test_text_format();
    test_capacity_guards();
    puts("mult_uart test protocol tests passed");
    return 0;
}
