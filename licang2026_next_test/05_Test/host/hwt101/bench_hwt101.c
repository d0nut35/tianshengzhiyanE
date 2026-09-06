/* Host timing only: real driver code, mocked HAL, no scan instrumentation. */
#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <windows.h>

#include "hwt101_adaption.h"
#include "usart.h"

static DMA_HandleTypeDef dma;
UART_HandleTypeDef huart2 = {&dma, 0U};
static uint32_t irq_mask;
static uint8_t *rx_dst;
static uint32_t sink;
static const uint8_t packet[22] = {
    0x55, 0x52, 0, 0, 0, 0, 0, 0x40, 0, 0, 0xE7,
    0x55, 0x53, 0, 0, 0, 0, 0, 0x40, 0, 0, 0xE8
};

uint32_t test_irq_get(void) { return irq_mask; }
void test_irq_disable(void) { irq_mask = 1U; }
void test_irq_restore(uint32_t mask) { irq_mask = mask; }

HAL_StatusTypeDef HAL_UARTEx_ReceiveToIdle_DMA(
    UART_HandleTypeDef *uart, uint8_t *data, uint16_t len)
{
    assert(uart == &huart2 && len == 64U);
    rx_dst = data;
    return HAL_OK;
}

HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *uart, uint8_t *data,
                                  uint16_t len, uint32_t timeout)
{
    (void)uart;
    (void)data;
    (void)len;
    (void)timeout;
    return HAL_OK;
}

static void setup(void)
{
    assert(hwt101_adp_init() == HWT101_OK);
    assert(hwt101_adp_start() == HWT101_OK);
    memcpy(rx_dst, packet, sizeof(packet));
    assert(hwt101_adp_rx_isr(sizeof(packet)) == HWT101_OK);
    memcpy(rx_dst, packet, sizeof(packet));
    assert(hwt101_adp_rx_isr(sizeof(packet)) == HWT101_OK);
}

static double measure(unsigned mode, unsigned iterations)
{
    LARGE_INTEGER start;
    LARGE_INTEGER end;
    LARGE_INTEGER frequency;
    float gyro = 0.0f;
    float yaw = 0.0f;
    unsigned i;
    uint32_t result = 0U;

    QueryPerformanceFrequency(&frequency);
    QueryPerformanceCounter(&start);
    for (i = 0U; i < iterations; i++) {
        if (mode != 0U) {
            result += hwt101_adp_rx_isr(sizeof(packet));
        }
        if (mode != 2U) {
            result += hwt101_adp_read(&gyro, &yaw);
        }
    }
    QueryPerformanceCounter(&end);
    sink += result;
    if (mode != 2U) {
        assert(gyro == 1000.0f && yaw == 90.0f);
    }
    return (double)(end.QuadPart - start.QuadPart) * 1.0e9 /
           (double)frequency.QuadPart / (double)iterations;
}

int main(void)
{
    const char *names[3] = {"repeat_read", "rx_then_first_read", "rx_isr"};
    double times[9];
    double value;
    unsigned mode;
    unsigned round;
    unsigned j;

    setup();
    for (mode = 0U; mode < 3U; mode++) {
        (void)measure(mode, 10000U);
        for (round = 0U; round < 9U; round++) {
            times[round] = measure(mode, 1000000U);
        }
        for (round = 1U; round < 9U; round++) {
            value = times[round];
            j = round;
            while (j > 0U && times[j - 1U] > value) {
                times[j] = times[j - 1U];
                j--;
            }
            times[j] = value;
        }
        printf("%s: median %.2f ns/op; min %.2f; max %.2f\n",
               names[mode], times[4], times[0], times[8]);
    }
    assert(sink == 0U);
    puts("Windows host only; HAL mocked; these are NOT Cortex-M7 cycles.");
    return 0;
}
