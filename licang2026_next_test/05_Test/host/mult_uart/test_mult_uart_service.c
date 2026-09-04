#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "mult_uart_core.h"
#include "mult_uart_service.h"

/*
 * fake_call_t 不是为了模拟 UART 数据本身，而是记录 Service 调 Core port 的
 * 顺序。Service 层最容易出错的地方常常不是“调没调”，而是“先 RX 还是先 TX”
 * 以及“切通道前后 EN 是否正确”，所以测试用调用日志来观察时序。
 */
typedef enum {
    CALL_ENABLE_LOW = 1,
    CALL_ENABLE_HIGH,
    CALL_SELECT_00,
    CALL_SELECT_01,
    CALL_SELECT_10,
    CALL_SELECT_11,
    CALL_TX,
    CALL_RX,
    CALL_ABORT,
} fake_call_t;

/*
 * fake_port_t 是 PC 侧的“假硬件”。它把 GPIO/UART/DMA 的结果变成可控返回值，
 * 让我们能注入 select/tx/rx/abort 失败，而不需要真正的 STM32 板子。
 */
typedef struct {
    fake_call_t calls[64];
    size_t call_count;
    const uint8_t *last_tx_data;
    size_t last_tx_len;
    uint8_t *last_rx_data;
    size_t last_rx_capacity;
    mult_uart_status_t enable_status;
    mult_uart_status_t select_status;
    mult_uart_status_t tx_status;
    mult_uart_status_t rx_status;
    mult_uart_status_t abort_status;
} fake_port_t;

/*
 * fake_app_t 模拟上层应用。完成回调里的 rx_data 只在回调期间有效，因此测试
 * 会立刻复制一份 rx_snapshots，用来验证 Service 的缓冲区生命周期契约。
 */
typedef struct {
    uint32_t now;
    mult_uart_completion_t completions[16];
    uint8_t rx_snapshots[16][MULT_UART_SERVICE_RX_MAX];
    size_t completion_count;
} fake_app_t;

static int g_failures;

#define CHECK(condition)                                                        \
    do {                                                                        \
        if (!(condition)) {                                                      \
            (void)fprintf(stderr, "FAIL %s:%d: %s\n",                          \
                          __FILE__, __LINE__, #condition);                       \
            g_failures++;                                                        \
        }                                                                        \
    } while (0)

static void fake_record(fake_port_t *fake, fake_call_t call)
{
    if (fake->call_count < (sizeof(fake->calls) / sizeof(fake->calls[0]))) {
        fake->calls[fake->call_count] = call;
    }
    fake->call_count++;
}

static mult_uart_status_t fake_write_select(
    void *ctx,
    bool a_high,
    bool b_high)
{
    fake_port_t *fake = (fake_port_t *)ctx;
    uint32_t value = (a_high ? 1U : 0U) | (b_high ? 2U : 0U);

    fake_record(fake, (fake_call_t)(CALL_SELECT_00 + value));
    return fake->select_status;
}

static mult_uart_status_t fake_write_enable(void *ctx, bool level_high)
{
    fake_port_t *fake = (fake_port_t *)ctx;

    fake_record(fake, level_high ? CALL_ENABLE_HIGH : CALL_ENABLE_LOW);
    return fake->enable_status;
}

static mult_uart_status_t fake_start_tx(
    void *ctx,
    const uint8_t *data,
    size_t len)
{
    fake_port_t *fake = (fake_port_t *)ctx;

    fake_record(fake, CALL_TX);
    fake->last_tx_data = data;
    fake->last_tx_len = len;
    return fake->tx_status;
}

static mult_uart_status_t fake_start_rx(
    void *ctx,
    uint8_t *data,
    size_t capacity)
{
    fake_port_t *fake = (fake_port_t *)ctx;

    fake_record(fake, CALL_RX);
    fake->last_rx_data = data;
    fake->last_rx_capacity = capacity;
    return fake->rx_status;
}

static mult_uart_status_t fake_abort(void *ctx)
{
    fake_port_t *fake = (fake_port_t *)ctx;

    fake_record(fake, CALL_ABORT);
    return fake->abort_status;
}

static const mult_uart_port_ops_t g_fake_ops = {
    fake_write_select,
    fake_write_enable,
    fake_start_tx,
    fake_start_rx,
    fake_abort,
    NULL,
};

static uint32_t fake_now(void *ctx)
{
    return ((fake_app_t *)ctx)->now;
}

static void fake_done(
    void *user_ctx,
    const mult_uart_completion_t *completion)
{
    fake_app_t *app = (fake_app_t *)user_ctx;
    size_t index = app->completion_count;

    CHECK(index < (sizeof(app->completions) / sizeof(app->completions[0])));
    if (index >= (sizeof(app->completions) / sizeof(app->completions[0]))) {
        return;
    }

    app->completions[index] = *completion;
    if ((completion->rx_data != NULL) && (completion->rx_len > 0U)) {
        CHECK(completion->rx_len <= MULT_UART_SERVICE_RX_MAX);
        (void)memcpy(
            app->rx_snapshots[index],
            completion->rx_data,
            completion->rx_len);
    }
    app->completion_count++;
}

static void fake_port_init(fake_port_t *fake)
{
    (void)memset(fake, 0, sizeof(*fake));
    fake->enable_status = MULT_UART_OK;
    fake->select_status = MULT_UART_OK;
    fake->tx_status = MULT_UART_OK;
    fake->rx_status = MULT_UART_OK;
    fake->abort_status = MULT_UART_OK;
}

static void init_stack(
    mult_uart_bus_t *bus,
    mult_uart_service_t *service,
    fake_port_t *fake,
    fake_app_t *app)
{
    mult_uart_config_t bus_config = {
        true,
        true,
        false,
        0U,
    };
    mult_uart_port_t port = {&g_fake_ops, fake};
    mult_uart_service_config_t service_config = {0};

    fake_port_init(fake);
    (void)memset(app, 0, sizeof(*app));
    (void)memset(bus, 0, sizeof(*bus));
    (void)memset(service, 0, sizeof(*service));
    CHECK(mult_uart_init(bus, &bus_config, &port) == MULT_UART_OK);

    service_config.bus = bus;
    service_config.now_ms = fake_now;
    service_config.time_ctx = app;
    CHECK(mult_uart_service_init(service, &service_config) == MULT_UART_OK);
    CHECK(mult_uart_service_start(service) == MULT_UART_OK);
}

/*
 * 验证 submit 会复制 TX 数据：应用提交后立刻修改原数组，真正送到 port 的
 * 仍然应该是旧内容。这能防止“请求排队等待期间，上层缓冲区被复用”带来的坑。
 */
static void test_write_copies_tx_data(void)
{
    mult_uart_bus_t bus;
    mult_uart_service_t service;
    fake_port_t fake;
    fake_app_t app;
    uint8_t tx[] = {'a', 'b', 'c'};
    mult_uart_request_t request = {
        10U,
        MULT_UART_OP_WRITE,
        MULT_UART_CHANNEL_1,
        tx,
        sizeof(tx),
        0U,
        100U,
        fake_done,
        &app,
    };

    init_stack(&bus, &service, &fake, &app);
    CHECK(mult_uart_service_submit(&service, &request) == MULT_UART_OK);
    tx[0] = 'x';

    mult_uart_service_process_once(&service);
    CHECK(fake.calls[1] == CALL_SELECT_01);
    CHECK(fake.calls[2] == CALL_ENABLE_LOW);
    CHECK(fake.calls[3] == CALL_TX);
    CHECK(fake.last_tx_len == 3U);
    CHECK(memcmp(fake.last_tx_data, "abc", 3U) == 0);

    mult_uart_on_tx_complete_isr(&bus);
    mult_uart_service_process_once(&service);
    CHECK(app.completion_count == 1U);
    CHECK(app.completions[0].request_id == 10U);
    CHECK(app.completions[0].status == MULT_UART_OK);
}

/*
 * SELECT只负责本地GPIO选通，不启动UART DMA，也不等待通道外设回包。
 * 该回归测试防止以后误把SELECT写成一笔需要TX/RX完成事件的事务。
 */
static void test_select_completes_without_uart_dma(void)
{
    mult_uart_bus_t bus;
    mult_uart_service_t service;
    fake_port_t fake;
    fake_app_t app;
    size_t calls_before;
    size_t i;
    mult_uart_request_t request = {
        9U,
        MULT_UART_OP_SELECT,
        MULT_UART_CHANNEL_2,
        NULL,
        0U,
        0U,
        0U,
        fake_done,
        &app,
    };

    init_stack(&bus, &service, &fake, &app);
    calls_before = fake.call_count;
    CHECK(mult_uart_service_submit(&service, &request) == MULT_UART_OK);
    mult_uart_service_process_once(&service);

    CHECK(app.completion_count == 1U);
    CHECK(app.completions[0].request_id == 9U);
    CHECK(app.completions[0].operation == MULT_UART_OP_SELECT);
    CHECK(app.completions[0].channel == MULT_UART_CHANNEL_2);
    CHECK(app.completions[0].status == MULT_UART_OK);
    CHECK(fake.call_count == calls_before + 2U);
    CHECK(fake.calls[calls_before] == CALL_SELECT_10);
    CHECK(fake.calls[calls_before + 1U] == CALL_ENABLE_LOW);
    for (i = calls_before; i < fake.call_count; ++i) {
        CHECK(fake.calls[i] != CALL_TX);
        CHECK(fake.calls[i] != CALL_RX);
        CHECK(fake.calls[i] != CALL_ABORT);
    }
}

/*
 * 验证 WRITE_READ 的两个核心时序：
 * 1. 先启动 RX，再启动 TX，避免快响应丢包；
 * 2. RX 完成后不能立刻回调，必须等 TX 也完成，整笔事务才算成功。
 */
static void test_write_read_waits_for_tx_and_rx(void)
{
    mult_uart_bus_t bus;
    mult_uart_service_t service;
    fake_port_t fake;
    fake_app_t app;
    const uint8_t tx[] = {'p', 'i', 'n', 'g'};
    const uint8_t rx[] = {'o', 'k'};
    mult_uart_request_t request = {
        11U,
        MULT_UART_OP_WRITE_READ,
        MULT_UART_CHANNEL_2,
        tx,
        sizeof(tx),
        8U,
        100U,
        fake_done,
        &app,
    };

    init_stack(&bus, &service, &fake, &app);
    CHECK(mult_uart_service_submit(&service, &request) == MULT_UART_OK);
    mult_uart_service_process_once(&service);
    CHECK(fake.calls[3] == CALL_RX);
    CHECK(fake.calls[4] == CALL_TX);
    CHECK(fake.last_rx_capacity == 8U);

    (void)memcpy(fake.last_rx_data, rx, sizeof(rx));
    mult_uart_on_rx_complete_isr(&bus, sizeof(rx));
    mult_uart_service_process_once(&service);
    CHECK(app.completion_count == 0U);

    mult_uart_on_tx_complete_isr(&bus);
    mult_uart_service_process_once(&service);
    CHECK(app.completion_count == 1U);
    CHECK(app.completions[0].status == MULT_UART_OK);
    CHECK(app.completions[0].rx_len == sizeof(rx));
    CHECK(memcmp(app.rx_snapshots[0], rx, sizeof(rx)) == 0);
}

/*
 * 验证固定深度队列的边界，以及 stop() 会把队列里尚未执行的请求统一取消。
 */
static void test_queue_full_and_stop_cancel(void)
{
    mult_uart_bus_t bus;
    mult_uart_service_t service;
    fake_port_t fake;
    fake_app_t app;
    mult_uart_request_t request = {
        20U,
        MULT_UART_OP_SELECT,
        MULT_UART_CHANNEL_0,
        NULL,
        0U,
        0U,
        0U,
        fake_done,
        &app,
    };
    size_t i;

    init_stack(&bus, &service, &fake, &app);
    for (i = 0U; i < MULT_UART_SERVICE_QUEUE_DEPTH; ++i) {
        request.request_id = 20U + (uint32_t)i;
        CHECK(mult_uart_service_submit(&service, &request) == MULT_UART_OK);
    }
    request.request_id = 99U;
    CHECK(mult_uart_service_submit(&service, &request) ==
          MULT_UART_ERR_QUEUE_FULL);

    CHECK(mult_uart_service_stop(&service) == MULT_UART_OK);
    CHECK(app.completion_count == MULT_UART_SERVICE_QUEUE_DEPTH);
    for (i = 0U; i < app.completion_count; ++i) {
        CHECK(app.completions[i].status == MULT_UART_ERR_CANCELLED);
    }
}

/*
 * 验证超时路径会调用 Core abort，再以 TIMEOUT 通知上层。
 */
static void test_timeout_aborts_active_request(void)
{
    mult_uart_bus_t bus;
    mult_uart_service_t service;
    fake_port_t fake;
    fake_app_t app;
    const uint8_t tx[] = {'a'};
    mult_uart_request_t request = {
        30U,
        MULT_UART_OP_WRITE,
        MULT_UART_CHANNEL_3,
        tx,
        sizeof(tx),
        0U,
        5U,
        fake_done,
        &app,
    };

    init_stack(&bus, &service, &fake, &app);
    CHECK(mult_uart_service_submit(&service, &request) == MULT_UART_OK);
    mult_uart_service_process_once(&service);
    CHECK(app.completion_count == 0U);

    app.now = 5U;
    mult_uart_service_process_once(&service);
    CHECK(app.completion_count == 1U);
    CHECK(app.completions[0].status == MULT_UART_ERR_TIMEOUT);
    CHECK(fake.calls[fake.call_count - 1U] == CALL_ABORT);
}

/*
 * 验证超出 Service 静态缓冲区上限的请求不会入队，同时统计 invalid/overflow。
 */
static void test_oversize_request_rejected_before_queue(void)
{
    mult_uart_bus_t bus;
    mult_uart_service_t service;
    fake_port_t fake;
    fake_app_t app;
    uint8_t tx[MULT_UART_SERVICE_TX_MAX + 1U];
    mult_uart_service_stats_t stats;
    mult_uart_request_t request = {
        40U,
        MULT_UART_OP_WRITE,
        MULT_UART_CHANNEL_0,
        tx,
        sizeof(tx),
        0U,
        0U,
        fake_done,
        &app,
    };

    init_stack(&bus, &service, &fake, &app);
    CHECK(mult_uart_service_submit(&service, &request) ==
          MULT_UART_ERR_OVERFLOW);
    CHECK(mult_uart_service_get_stats(&service, &stats) == MULT_UART_OK);
    CHECK(stats.invalid_request == 1U);
    CHECK(stats.overflow == 1U);
    CHECK(stats.submitted == 0U);
}

int main(void)
{
    test_select_completes_without_uart_dma();
    test_write_copies_tx_data();
    test_write_read_waits_for_tx_and_rx();
    test_queue_full_and_stop_cancel();
    test_timeout_aborts_active_request();
    test_oversize_request_rejected_before_queue();

    if (g_failures == 0) {
        puts("mult_uart_service tests passed");
    }
    return g_failures;
}
