/* Host-only: generated source adds two scan hooks, never to the firmware. */
#include <assert.h>
#include <float.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "driver_under_test.c"

static DMA_HandleTypeDef dma;
UART_HandleTypeDef huart2 = {&dma, 0U};
static uint32_t irq_mask;
static uint32_t rx_calls;
static uint32_t parse_calls;
static uint32_t tx_calls;
static HAL_StatusTypeDef rx_result = HAL_OK;
static HAL_StatusTypeDef tx_result = HAL_OK;
static uint8_t *rx_dst;
static uint8_t tx_frame[5];
static unsigned hook;
static uint8_t next_frame[22];

uint32_t test_irq_get(void)
{
    return irq_mask;
}

void test_irq_disable(void)
{
    irq_mask = 1U;
}

void test_irq_restore(uint32_t mask)
{
    irq_mask = mask;
}

void test_parse_enter(void)
{
    assert(irq_mask == 0U);
    parse_calls++;
}

HAL_StatusTypeDef HAL_UARTEx_ReceiveToIdle_DMA(
    UART_HandleTypeDef *uart, uint8_t *data, uint16_t len)
{
    assert(uart == &huart2);
    assert(len == 64U);
    assert(data == g_buf[g_active]);
    rx_calls++;
    if (rx_result == HAL_OK) {
        rx_dst = data;
    }
    return rx_result;
}

HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *uart, uint8_t *data,
                                  uint16_t len, uint32_t timeout)
{
    assert(uart == &huart2);
    assert(len == 5U);
    assert(timeout == 100U);
    memcpy(tx_frame, data, len);
    tx_calls++;
    return tx_result;
}

static void frame_make(uint8_t *buf, uint8_t id, int16_t raw)
{
    uint8_t sum = 0U;
    unsigned i;

    memset(buf, 0, 11U);
    buf[0] = 0x55U;
    buf[1] = id;
    buf[6] = (uint8_t)raw;
    buf[7] = (uint8_t)((uint16_t)raw >> 8U);
    for (i = 0U; i < 10U; i++) {
        sum = (uint8_t)(sum + buf[i]);
    }
    buf[10] = sum;
}

static void pair_make(uint8_t *buf, int16_t gyro, int16_t yaw)
{
    frame_make(buf, 0x52U, gyro);
    frame_make(buf + 11U, 0x53U, yaw);
}

static void feed(const uint8_t *data, uint16_t len)
{
    const uint16_t copy_len = (len > 64U) ? 64U : len;

    memcpy(rx_dst, data, copy_len);
    assert(hwt101_adp_rx_isr(len) == HWT101_OK);
}

void test_scan_step(void)
{
    unsigned mode = hook;
    float gyro = 0.0f;
    float yaw = 0.0f;

    hook = 0U;
    if ((mode == 1U) || (mode == 2U) || (mode == 5U)) {
        feed(next_frame, 22U);
        if (mode == 2U) {
            feed(next_frame, 22U);
        }
        if (mode == 5U) {
            assert(hwt101_adp_read(&gyro, &yaw) == HWT101_OK);
        }
    } else if (mode == 3U) {
        assert(hwt101_adp_err_isr() == HWT101_OK);
    } else if (mode == 4U) {
        assert(hwt101_adp_read(&gyro, &yaw) == HWT101_OK);
    }
}

static void reset_module(void)
{
    /* Reset represents power-on, without exposing production reset hooks. */
    g_started = 0U;
    rx_result = HAL_OK;
    tx_result = HAL_OK;
    huart2.hdmarx = &dma;
    irq_mask = 0U;
    hook = 0U;
    assert(hwt101_adp_init() == HWT101_OK);
    assert(hwt101_adp_start() == HWT101_OK);
}

static void read_expect(hwt101_status_t status, float gyro_val, float yaw_val)
{
    float gyro = 123.0f;
    float yaw = 456.0f;

    assert(hwt101_adp_read(&gyro, &yaw) == status);
    if (status == HWT101_OK) {
        assert(gyro == gyro_val);
        assert(yaw == yaw_val);
    } else {
        assert(gyro == 123.0f);
        assert(yaw == 456.0f);
    }
    assert(irq_mask == 0U);
}

static void test_start_tx(void)
{
    uint8_t buf[64] = {0};
    float gyro = 1.0f;
    float yaw = 2.0f;
    uint32_t count;

    assert(hwt101_adp_start() == HWT101_ERR_INIT);
    assert(hwt101_adp_rx_isr(22U) == HWT101_ERR_INIT);
    assert(hwt101_adp_err_isr() == HWT101_ERR_INIT);
    assert(hwt101_adp_read(&gyro, &yaw) == HWT101_ERR_INIT);
    assert(hwt101_adp_set_yaw(0.0f) == HWT101_ERR_INIT);
    assert(hwt101_adp_read(NULL, &yaw) == HWT101_ERR_PARAM);
    assert(hwt101_adp_read(&gyro, NULL) == HWT101_ERR_PARAM);
    assert(hwt101_adp_write_reg(0x69U, 0x88U, 0xB5U) == HWT101_OK);
    assert(memcmp(tx_frame, "\xFF\xAA\x69\x88\xB5", 5U) == 0);
    tx_result = HAL_TIMEOUT;
    assert(hwt101_adp_write_reg(0U, 0U, 0U) == HWT101_ERR_TMO);
    tx_result = HAL_BUSY;
    assert(hwt101_adp_write_reg(0U, 0U, 0U) == HWT101_ERR);
    tx_result = HAL_ERROR;
    assert(hwt101_adp_write_reg(0U, 0U, 0U) == HWT101_ERR);
    assert(tx_calls == 4U);
    huart2.hdmarx = NULL;
    assert(hwt101_adp_init() == HWT101_ERR_PARAM);
    huart2.hdmarx = &dma;
    assert(hwt101_adp_init() == HWT101_OK);
    assert(hwt101_adp_init() == HWT101_OK);
    read_expect(HWT101_ERR_RES, 0.0f, 0.0f);
    rx_result = HAL_BUSY;
    assert(hwt101_adp_start() == HWT101_ERR);
    assert(g_started == 0U);
    assert((huart2.disabled & UART_IT_IDLE) != 0U);
    rx_result = HAL_OK;
    assert(hwt101_adp_start() == HWT101_OK);
    count = rx_calls;
    assert(hwt101_adp_start() == HWT101_OK);
    assert(rx_calls == count);
    assert(hwt101_adp_init() == HWT101_ERR);
    assert((dma.disabled & DMA_IT_HT) != 0U);
    pair_make(buf, 16384, -16384);
    feed(buf, 22U);
    read_expect(HWT101_OK, 1000.0f, -90.0f);
    irq_mask = 1U;
    assert(hwt101_adp_read(&gyro, &yaw) == HWT101_OK);
    assert(irq_mask == 1U);
    irq_mask = 0U;
    puts("PASS init/start/parameters/fixed huart2/TX/PRIMASK");
}

static void test_cache_calib(void)
{
    uint8_t buf[64] = {0};
    uint32_t count;
    unsigned i;

    reset_module();
    pair_make(buf, 16384, 16384);
    feed(buf, 22U);
    count = parse_calls;
    for (i = 0U; i < 100U; i++) {
        read_expect(HWT101_OK, 1000.0f, 90.0f);
    }
    assert(parse_calls == count + 1U);
    assert(hwt101_adp_set_yaw(-180.0f) == HWT101_OK);
    read_expect(HWT101_OK, 1000.0f, -180.0f);
    assert(hwt101_adp_set_yaw(1080.0f) == HWT101_OK);
    read_expect(HWT101_OK, 1000.0f, 0.0f);
    assert(hwt101_adp_set_yaw(-1080.0f) == HWT101_OK);
    read_expect(HWT101_OK, 1000.0f, 0.0f);
    assert(hwt101_adp_set_yaw(INFINITY) == HWT101_ERR_PARAM);
    assert(hwt101_adp_set_yaw(NAN) == HWT101_ERR_PARAM);
    read_expect(HWT101_OK, 1000.0f, 0.0f);
    assert(hwt101_adp_set_yaw(FLT_MAX) == HWT101_OK);
    assert(isfinite(g_yaw_ofs));
    assert(g_yaw_ofs >= -180.0f && g_yaw_ofs < 180.0f);
    assert(parse_calls == count + 1U);
    assert(hwt101_adp_set_yaw(90.0f) == HWT101_OK);
    frame_make(buf, 0x53U, -32768);
    feed(buf, 11U);
    read_expect(HWT101_OK, 123.0f, -180.0f);
    frame_make(buf, 0x52U, 100);
    feed(buf, 11U);
    read_expect(HWT101_ERR_RES, 0.0f, 0.0f);
    count = parse_calls;
    read_expect(HWT101_ERR_RES, 0.0f, 0.0f);
    assert(parse_calls == count);
    assert(hwt101_adp_set_yaw(30.0f) == HWT101_ERR_RES);
    frame_make(buf, 0x53U, 0);
    feed(buf, 11U);
    read_expect(HWT101_OK, 123.0f, 0.0f);
    g_rx_seq = UINT32_MAX;
    feed(buf, 11U);
    assert(g_rx_seq == 0U);
    read_expect(HWT101_OK, 123.0f, 0.0f);
    puts("PASS repeated reads/cache invalidation/calibration/sequence wrap");
}

static void test_interleave(void)
{
    uint8_t buf[64] = {0};
    unsigned mode;

    pair_make(buf, 0, 0);
    pair_make(next_frame, 8192, 8192);
    for (mode = 1U; mode <= 5U; mode++) {
        reset_module();
        feed(buf, 22U);
        hook = mode;
        read_expect((mode == 4U) ? HWT101_OK : HWT101_ERR_RES,
                    0.0f, 0.0f);
        if (mode == 3U) {
            read_expect(HWT101_ERR_RES, 0.0f, 0.0f);
            feed(next_frame, 22U);
        }
        if (mode != 4U) {
            read_expect(HWT101_OK, 500.0f, 45.0f);
        }
    }
    puts("PASS mid-scan one/two RX events/error/nested readers/newer cache");
}

static void test_recovery(void)
{
    uint8_t buf[64] = {0};
    unsigned i;

    pair_make(buf, 8192, 8192);
    for (i = 0U; i < 2U; i++) {
        reset_module();
        feed(buf, 22U);
        read_expect(HWT101_OK, 500.0f, 45.0f);
        rx_result = HAL_ERROR;
        if (i == 0U) {
            assert(hwt101_adp_rx_isr(22U) == HWT101_ERR);
        } else {
            assert(hwt101_adp_err_isr() == HWT101_ERR);
        }
        assert(g_started == 0U);
        read_expect(HWT101_ERR_RES, 0.0f, 0.0f);
        rx_result = HAL_OK;
        assert(hwt101_adp_start() == HWT101_OK);
        read_expect(HWT101_ERR_RES, 0.0f, 0.0f);
        feed(buf, 22U);
        read_expect(HWT101_OK, 500.0f, 45.0f);
    }
    puts("PASS RX/error restart failures and task retry");
}

/* Independent forward scanner mirrors the pre-change protocol contract. */
static hwt_sample_t reference(const uint8_t *data, uint16_t len)
{
    hwt_sample_t result = {0};
    uint16_t i = 0U;
    uint8_t sum;
    uint8_t j;
    int16_t raw;

    while (i < len) {
        if (data[i] != 0x55U) {
            i++;
            continue;
        }
        if (len - i < 11) {
            break;
        }
        sum = 0U;
        for (j = 0U; j < 10U; j++) {
            sum = (uint8_t)(sum + data[i + j]);
        }
        if (sum != data[i + 10U]) {
            i++;
            continue;
        }
        raw = (int16_t)((uint16_t)data[i + 6U] |
                       ((uint16_t)data[i + 7U] << 8U));
        if (data[i + 1U] == 0x52U) {
            result.gyro = ((float)raw * 2000.0f) / 32768.0f;
            result.flags |= HWT101_HAS_GYRO;
        } else if (data[i + 1U] == 0x53U) {
            result.yaw = ((float)raw * 180.0f) / 32768.0f;
            result.flags |= HWT101_HAS_YAW;
        } else {
            i++;
            continue;
        }
        i += 11U;
    }
    return result;
}

static void check_packet(const uint8_t *buf, uint16_t len)
{
    const hwt_sample_t expect = reference(buf, len > 64U ? 64U : len);

    feed(buf, len);
    read_expect((expect.flags & HWT101_HAS_YAW) ? HWT101_OK : HWT101_ERR_RES,
                (expect.flags & HWT101_HAS_GYRO) ? expect.gyro : 123.0f,
                expect.yaw);
}

static uint32_t rand_u32(void)
{
    static uint32_t value = 0x87123abcU;

    value ^= value << 13U;
    value ^= value >> 17U;
    value ^= value << 5U;
    return value;
}

static void test_large_angles(void)
{
    uint8_t buf[22];
    uint32_t bits;
    float target;
    float expect;
    unsigned i;

    reset_module();
    pair_make(buf, 0, 0);
    feed(buf, 22U);
    for (i = 0U; i < 30000U; i++) {
        bits = rand_u32();
        memcpy(&target, &bits, sizeof(target));
        if (!isfinite(target)) {
            assert(hwt101_adp_set_yaw(target) == HWT101_ERR_PARAM);
            continue;
        }
        expect = fmodf(target, 360.0f);
        if (expect >= 180.0f) {
            expect -= 360.0f;
        } else if (expect < -180.0f) {
            expect += 360.0f;
        }
        assert(hwt101_adp_set_yaw(target) == HWT101_OK);
        read_expect(HWT101_OK, 0.0f, expect);
    }
    puts("PASS 30000 float calibration inputs against modulo reference");
}

static void test_protocol(void)
{
    uint8_t buf[64] = {0};
    unsigned i;
    unsigned j;
    unsigned pos;
    uint16_t len;
    hwt_sample_t result;

    reset_module();
    pair_make(buf, -32768, 32767);
    for (len = 0U; len <= 64U; len++) {
        check_packet(buf, len);
    }
    check_packet(buf, UINT16_MAX);
    memset(buf, 0x55, sizeof(buf));
    frame_make(buf + 3U, 0x53U, 12345);
    frame_make(buf + 16U, 0x52U, -12345);
    frame_make(buf + 27U, 0x53U, -32768);
    frame_make(buf + 38U, 0x52U, 32767);
    frame_make(buf + 49U, 0x53U, 1234);
    buf[59]++;
    check_packet(buf, 64U);
    for (i = 0U; i < 30000U; i++) {
        len = (uint16_t)(rand_u32() % 65U);
        for (j = 0U; j < sizeof(buf); j++) {
            buf[j] = (uint8_t)rand_u32();
        }
        if (len >= 11U) {
            pos = rand_u32() % (len - 10U);
            frame_make(buf + pos, (uint8_t)(0x51U + rand_u32() % 4U),
                       (int16_t)rand_u32());
            if ((i % 3U) == 0U) {
                buf[pos + 10U]++;
            }
        }
        check_packet(buf, len);
    }
    for (i = 0U; i <= UINT16_MAX; i++) {
        pair_make(buf, (int16_t)i, (int16_t)i);
        parse_buf(buf, 22U, &result);
        assert(result.gyro == ((float)(int16_t)i * 2000.0f) / 32768.0f);
        assert(result.yaw == ((float)(int16_t)i * 180.0f) / 32768.0f);
    }
    puts("PASS lengths 0..64/oversize/noise/latest frames/bad checksum");
    puts("PASS 30000 differential packets and all 65536 int16 values");
}

int main(void)
{
    test_start_tx();
    test_cache_calib();
    test_interleave();
    test_recovery();
    test_protocol();
    test_large_angles();
    puts("All HWT101 host tests passed (HAL mocked; not board validation).");
    return 0;
}
