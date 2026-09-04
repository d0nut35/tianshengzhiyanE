#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "mult_uart_core.h"

#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))

/* fake port 操作日志：用于验证 GPIO、延时和 DMA 调用的严格先后顺序。 */
typedef enum {
    CALL_ENABLE_LOW = 1,
    CALL_ENABLE_HIGH,
    CALL_SELECT_00,
    CALL_SELECT_01,
    CALL_SELECT_10,
    CALL_SELECT_11,
    CALL_DELAY,
    CALL_TX,
    CALL_RX,
    CALL_ABORT,
} fake_call_t;

/*
 * PC 侧平台替身。除记录参数外，每个 port 操作都可注入失败结果；
 * complete_tx_during_start 用于模拟 DMA 在 start 返回前完成的极端竞态。
 */
typedef struct {
    fake_call_t calls[32];
    size_t call_count;
    uint32_t last_delay_us;
    const uint8_t *last_tx_data;
    size_t last_tx_len;
    uint8_t *last_rx_data;
    size_t last_rx_capacity;
    mult_uart_status_t enable_status;
    mult_uart_status_t select_status;
    mult_uart_status_t tx_status;
    mult_uart_status_t rx_status;
    mult_uart_status_t abort_status;
    mult_uart_bus_t *sync_complete_bus;
    bool complete_tx_during_start;
} fake_port_t;

/* 复制最后一次 ISR 事件，避免测试保存 Core 回调中的栈指针。 */
typedef struct {
    size_t count;
    mult_uart_event_t last;
} fake_event_sink_t;

static int g_failures;

/* 简单断言宏：不中断后续用例，最终统一返回失败数量。 */
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
    if (fake->call_count < ARRAY_SIZE(fake->calls)) {
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
    if (fake->complete_tx_during_start &&
        (fake->sync_complete_bus != NULL)) {
        mult_uart_on_tx_complete_isr(fake->sync_complete_bus);
    }
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

static void fake_delay(void *ctx, uint32_t delay_us)
{
    fake_port_t *fake = (fake_port_t *)ctx;

    fake_record(fake, CALL_DELAY);
    fake->last_delay_us = delay_us;
}

static const mult_uart_port_ops_t g_fake_ops = {
    fake_write_select,
    fake_write_enable,
    fake_start_tx,
    fake_start_rx,
    fake_abort,
    fake_delay,
};

/** @brief 把 fake port 恢复为“所有平台调用成功”的默认状态。 */
static void fake_init(fake_port_t *fake)
{
    (void)memset(fake, 0, sizeof(*fake));
    fake->enable_status = MULT_UART_OK;
    fake->select_status = MULT_UART_OK;
    fake->tx_status = MULT_UART_OK;
    fake->rx_status = MULT_UART_OK;
    fake->abort_status = MULT_UART_OK;
}

/** @brief 生成低有效、软件管理 EN 的常用测试配置。 */
static mult_uart_config_t managed_config(bool break_before_switch)
{
    mult_uart_config_t config;

    config.manage_enable = true;
    config.enable_active_low = true;
    config.break_before_switch = break_before_switch;
    config.switch_settle_us = 7U;
    return config;
}

/** @brief 清零并初始化一个 managed bus，减少各测试的重复样板。 */
static mult_uart_status_t init_managed_bus(
    mult_uart_bus_t *bus,
    fake_port_t *fake,
    bool break_before_switch)
{
    mult_uart_config_t config = managed_config(break_before_switch);
    mult_uart_port_t port = {&g_fake_ops, fake};

    (void)memset(bus, 0, sizeof(*bus));
    return mult_uart_init(bus, &config, &port);
}

/** @brief ISR 事件接收器，只复制事件内容和累计次数。 */
static void fake_event_callback(
    void *user_ctx,
    const mult_uart_event_t *event)
{
    fake_event_sink_t *sink = (fake_event_sink_t *)user_ctx;

    sink->count++;
    sink->last = *event;
}

/** @brief 建立“已绑定回调、已选通通道0、已使能”的测试前置状态。 */
static void prepare_enabled_channel(
    mult_uart_bus_t *bus,
    fake_port_t *fake,
    fake_event_sink_t *sink)
{
    CHECK(init_managed_bus(bus, fake, false) == MULT_UART_OK);
    CHECK(mult_uart_bind_event(bus, fake_event_callback, sink) == MULT_UART_OK);
    CHECK(mult_uart_select(bus, MULT_UART_CHANNEL_0) == MULT_UART_OK);
    CHECK(mult_uart_enable(bus) == MULT_UART_OK);
}

/* 验证低有效 EN 的安全初始化、先选后使能和正常禁用。 */
static void test_init_and_enable_levels(void)
{
    mult_uart_bus_t bus;
    fake_port_t fake;

    fake_init(&fake);
    CHECK(init_managed_bus(&bus, &fake, false) == MULT_UART_OK);
    CHECK(bus.initialized);
    CHECK(!bus.enabled);
    CHECK(bus.state == MULT_UART_STATE_IDLE);
    CHECK(bus.current_channel ==
          (mult_uart_channel_t)MULT_UART_CHANNEL_INVALID);
    CHECK(fake.call_count == 1U);
    CHECK(fake.calls[0] == CALL_ENABLE_HIGH);

    CHECK(mult_uart_enable(&bus) == MULT_UART_ERR_STATE);
    CHECK(mult_uart_select(&bus, MULT_UART_CHANNEL_0) == MULT_UART_OK);
    CHECK(mult_uart_enable(&bus) == MULT_UART_OK);
    CHECK(bus.enabled);
    CHECK(fake.calls[3] == CALL_ENABLE_LOW);
    CHECK(mult_uart_disable(&bus) == MULT_UART_OK);
    CHECK(!bus.enabled);
    CHECK(fake.calls[4] == CALL_ENABLE_HIGH);
}

/* 验证缺失必需 port 能力和互相矛盾的配置会在 init 阶段被拒绝。 */
static void test_invalid_port_configuration(void)
{
    mult_uart_bus_t bus;
    fake_port_t fake;
    mult_uart_config_t config = managed_config(false);
    mult_uart_port_ops_t ops = g_fake_ops;
    mult_uart_port_t port = {&ops, &fake};

    fake_init(&fake);
    (void)memset(&bus, 0, sizeof(bus));
    ops.start_rx_dma = NULL;
    CHECK(mult_uart_init(&bus, &config, &port) == MULT_UART_ERR_PARAM);
    CHECK(!bus.initialized);

    ops = g_fake_ops;
    config.break_before_switch = true;
    config.manage_enable = false;
    CHECK(mult_uart_init(&bus, &config, &port) == MULT_UART_ERR_PARAM);

    config.break_before_switch = false;
    config.switch_settle_us = 1U;
    ops.delay_us = NULL;
    CHECK(mult_uart_init(&bus, &config, &port) == MULT_UART_ERR_PARAM);
}

/* 验证硬件固定使能模式不访问 EN，并拒绝软件 disable。 */
static void test_unmanaged_enable(void)
{
    mult_uart_bus_t bus;
    fake_port_t fake;
    mult_uart_config_t config = {false, true, false, 0U};
    mult_uart_port_t port = {&g_fake_ops, &fake};

    fake_init(&fake);
    (void)memset(&bus, 0, sizeof(bus));
    CHECK(mult_uart_init(&bus, &config, &port) == MULT_UART_OK);
    CHECK(bus.enabled);
    CHECK(fake.call_count == 0U);
    CHECK(mult_uart_disable(&bus) == MULT_UART_ERR_UNSUPPORTED);
}

/* 验证高有效 EN 的实际 GPIO 电平与低有效配置相反。 */
static void test_active_high_enable_levels(void)
{
    mult_uart_bus_t bus;
    fake_port_t fake;
    mult_uart_config_t config = {true, false, false, 0U};
    mult_uart_port_t port = {&g_fake_ops, &fake};

    fake_init(&fake);
    (void)memset(&bus, 0, sizeof(bus));
    CHECK(mult_uart_init(&bus, &config, &port) == MULT_UART_OK);
    CHECK(fake.calls[0] == CALL_ENABLE_LOW);
    CHECK(mult_uart_enable(&bus) == MULT_UART_ERR_STATE);
    CHECK(mult_uart_select(&bus, MULT_UART_CHANNEL_0) == MULT_UART_OK);
    CHECK(mult_uart_enable(&bus) == MULT_UART_OK);
    CHECK(fake.calls[2] == CALL_ENABLE_HIGH);
    CHECK(mult_uart_disable(&bus) == MULT_UART_OK);
    CHECK(fake.calls[3] == CALL_ENABLE_LOW);
}

/* 验证初始化阶段写 EN 失败会完整回滚到 UNINIT。 */
static void test_init_enable_failure_rolls_back(void)
{
    mult_uart_bus_t bus;
    fake_port_t fake;
    mult_uart_config_t config = managed_config(false);
    mult_uart_port_t port = {&g_fake_ops, &fake};

    fake_init(&fake);
    fake.enable_status = MULT_UART_ERR_IO;
    (void)memset(&bus, 0, sizeof(bus));
    CHECK(mult_uart_init(&bus, &config, &port) == MULT_UART_ERR_IO);
    CHECK(!bus.initialized);
    CHECK(bus.state == MULT_UART_STATE_UNINIT);
    CHECK(bus.port.ops == NULL);
}

/* 验证四通道 B:A 真值、稳定延时及相同通道缓存。 */
static void test_channel_truth_and_cache(void)
{
    mult_uart_bus_t bus;
    fake_port_t fake;
    const fake_call_t expected[] = {
        CALL_ENABLE_HIGH,
        CALL_SELECT_00, CALL_DELAY,
        CALL_SELECT_01, CALL_DELAY,
        CALL_SELECT_10, CALL_DELAY,
        CALL_SELECT_11, CALL_DELAY,
    };
    size_t i;

    fake_init(&fake);
    CHECK(init_managed_bus(&bus, &fake, false) == MULT_UART_OK);
    CHECK(mult_uart_select(&bus, MULT_UART_CHANNEL_0) == MULT_UART_OK);
    CHECK(mult_uart_select(&bus, MULT_UART_CHANNEL_1) == MULT_UART_OK);
    CHECK(mult_uart_select(&bus, MULT_UART_CHANNEL_2) == MULT_UART_OK);
    CHECK(mult_uart_select(&bus, MULT_UART_CHANNEL_3) == MULT_UART_OK);
    CHECK(mult_uart_select(&bus, MULT_UART_CHANNEL_3) == MULT_UART_OK);
    CHECK(fake.call_count == ARRAY_SIZE(expected));
    for (i = 0U; i < ARRAY_SIZE(expected); ++i) {
        CHECK(fake.calls[i] == expected[i]);
    }
    CHECK(fake.last_delay_us == 7U);
    CHECK(mult_uart_select(&bus,
          (mult_uart_channel_t)MULT_UART_CHANNEL_COUNT) ==
          MULT_UART_ERR_PARAM);
}

/* 验证已使能时严格执行“断开→A/B→延时→恢复”的切换顺序。 */
static void test_break_before_switch(void)
{
    mult_uart_bus_t bus;
    fake_port_t fake;
    const fake_call_t expected[] = {
        CALL_ENABLE_HIGH,
        CALL_SELECT_00, CALL_DELAY,
        CALL_ENABLE_LOW,
        CALL_ENABLE_HIGH, CALL_SELECT_01, CALL_DELAY, CALL_ENABLE_LOW,
    };
    size_t i;

    fake_init(&fake);
    CHECK(init_managed_bus(&bus, &fake, true) == MULT_UART_OK);
    CHECK(mult_uart_select(&bus, MULT_UART_CHANNEL_0) == MULT_UART_OK);
    CHECK(mult_uart_enable(&bus) == MULT_UART_OK);
    CHECK(mult_uart_select(&bus, MULT_UART_CHANNEL_1) == MULT_UART_OK);
    CHECK(bus.enabled);
    CHECK(bus.current_channel == MULT_UART_CHANNEL_1);
    CHECK(fake.call_count == ARRAY_SIZE(expected));
    for (i = 0U; i < ARRAY_SIZE(expected); ++i) {
        CHECK(fake.calls[i] == expected[i]);
    }
}

/* 验证先断后切模式中 A/B 写失败后保持物理禁用。 */
static void test_failed_break_switch_stays_disabled(void)
{
    mult_uart_bus_t bus;
    fake_port_t fake;

    fake_init(&fake);
    CHECK(init_managed_bus(&bus, &fake, true) == MULT_UART_OK);
    CHECK(mult_uart_select(&bus, MULT_UART_CHANNEL_0) == MULT_UART_OK);
    CHECK(mult_uart_enable(&bus) == MULT_UART_OK);
    fake.select_status = MULT_UART_ERR_IO;
    CHECK(mult_uart_select(&bus, MULT_UART_CHANNEL_1) == MULT_UART_ERR_IO);
    CHECK(!bus.enabled);
    CHECK(bus.state == MULT_UART_STATE_IDLE);
    CHECK(bus.current_channel ==
          (mult_uart_channel_t)MULT_UART_CHANNEL_INVALID);
    CHECK(fake.calls[fake.call_count - 2U] == CALL_ENABLE_HIGH);
    CHECK(fake.calls[fake.call_count - 1U] == CALL_SELECT_01);
}

/* 验证非 break 切换失败时 managed EN 会尽力断开未知通道。 */
static void test_failed_live_switch_disables_module(void)
{
    mult_uart_bus_t bus;
    fake_port_t fake;

    fake_init(&fake);
    CHECK(init_managed_bus(&bus, &fake, false) == MULT_UART_OK);
    CHECK(mult_uart_select(&bus, MULT_UART_CHANNEL_0) == MULT_UART_OK);
    CHECK(mult_uart_enable(&bus) == MULT_UART_OK);
    fake.select_status = MULT_UART_ERR_IO;
    CHECK(mult_uart_select(&bus, MULT_UART_CHANNEL_1) == MULT_UART_ERR_IO);
    CHECK(!bus.enabled);
    CHECK(bus.state == MULT_UART_STATE_IDLE);
    CHECK(bus.current_channel ==
          (mult_uart_channel_t)MULT_UART_CHANNEL_INVALID);
    CHECK(fake.calls[fake.call_count - 2U] == CALL_SELECT_01);
    CHECK(fake.calls[fake.call_count - 1U] == CALL_ENABLE_HIGH);
}

/* 验证 TX/RX 事件字段，并覆盖 start 返回前同步完成的竞态。 */
static void test_dma_events_and_start_race(void)
{
    mult_uart_bus_t bus;
    fake_port_t fake;
    fake_event_sink_t sink = {0};
    uint8_t data[8] = {0};

    fake_init(&fake);
    prepare_enabled_channel(&bus, &fake, &sink);

    CHECK(mult_uart_start_tx_dma(&bus, data, sizeof(data)) == MULT_UART_OK);
    CHECK(bus.state == MULT_UART_STATE_ACTIVE);
    CHECK(bus.tx_active);
    CHECK(fake.last_tx_data == data);
    CHECK(fake.last_tx_len == sizeof(data));
    mult_uart_on_tx_complete_isr(&bus);
    CHECK(bus.state == MULT_UART_STATE_IDLE);
    CHECK(sink.count == 1U);
    CHECK(sink.last.type == MULT_UART_EVENT_TX_COMPLETE);

    CHECK(mult_uart_start_rx_dma(&bus, data, sizeof(data)) == MULT_UART_OK);
    CHECK(bus.state == MULT_UART_STATE_ACTIVE);
    CHECK(bus.rx_active);
    mult_uart_on_rx_complete_isr(&bus, 3U);
    CHECK(bus.state == MULT_UART_STATE_IDLE);
    CHECK(sink.count == 2U);
    CHECK(sink.last.type == MULT_UART_EVENT_RX_COMPLETE);
    CHECK(sink.last.rx_len == 3U);

    fake.sync_complete_bus = &bus;
    fake.complete_tx_during_start = true;
    CHECK(mult_uart_start_tx_dma(&bus, data, 1U) == MULT_UART_OK);
    CHECK(bus.state == MULT_UART_STATE_IDLE);
    CHECK(sink.count == 3U);
}

/* 验证先挂 RX 再启动 TX，两个方向可同时 ACTIVE 且独立完成。 */
static void test_full_duplex_rx_before_tx(void)
{
    mult_uart_bus_t bus;
    fake_port_t fake;
    fake_event_sink_t sink = {0};
    uint8_t tx_data[2] = {0xA5U, 0x5AU};
    uint8_t rx_data[8] = {0};

    fake_init(&fake);
    prepare_enabled_channel(&bus, &fake, &sink);
    CHECK(mult_uart_start_rx_dma(&bus, rx_data, sizeof(rx_data)) ==
          MULT_UART_OK);
    CHECK(mult_uart_start_tx_dma(&bus, tx_data, sizeof(tx_data)) ==
          MULT_UART_OK);
    CHECK(bus.state == MULT_UART_STATE_ACTIVE);
    CHECK(bus.rx_active);
    CHECK(bus.tx_active);

    mult_uart_on_tx_complete_isr(&bus);
    CHECK(bus.state == MULT_UART_STATE_ACTIVE);
    CHECK(bus.rx_active);
    CHECK(!bus.tx_active);
    mult_uart_on_rx_complete_isr(&bus, 3U);
    CHECK(bus.state == MULT_UART_STATE_IDLE);
    CHECK(!bus.rx_active);
    CHECK(sink.count == 2U);
    CHECK(sink.last.type == MULT_UART_EVENT_RX_COMPLETE);
}

/* 验证 DMA 的前置条件、port 启动失败回滚和错误方向 ISR 过滤。 */
static void test_dma_rejections_and_start_failure(void)
{
    mult_uart_bus_t bus;
    fake_port_t fake;
    fake_event_sink_t sink = {0};
    uint8_t data[4] = {0};

    fake_init(&fake);
    CHECK(init_managed_bus(&bus, &fake, false) == MULT_UART_OK);
    CHECK(mult_uart_start_tx_dma(&bus, data, sizeof(data)) ==
          MULT_UART_ERR_STATE);
    CHECK(mult_uart_select(&bus, MULT_UART_CHANNEL_0) == MULT_UART_OK);
    CHECK(mult_uart_enable(&bus) == MULT_UART_OK);
    CHECK(mult_uart_start_tx_dma(&bus, data, sizeof(data)) ==
          MULT_UART_ERR_STATE);
    CHECK(mult_uart_bind_event(&bus, fake_event_callback, &sink) ==
          MULT_UART_OK);

    fake.tx_status = MULT_UART_ERR_BUSY;
    CHECK(mult_uart_start_tx_dma(&bus, data, sizeof(data)) ==
          MULT_UART_ERR_BUSY);
    CHECK(bus.state == MULT_UART_STATE_IDLE);

    fake.tx_status = MULT_UART_OK;
    CHECK(mult_uart_start_tx_dma(&bus, data, sizeof(data)) == MULT_UART_OK);
    CHECK(mult_uart_select(&bus, MULT_UART_CHANNEL_1) ==
          MULT_UART_ERR_BUSY);
    CHECK(mult_uart_bind_event(&bus, NULL, NULL) == MULT_UART_ERR_BUSY);
    mult_uart_on_rx_complete_isr(&bus, 1U);
    CHECK(bus.state == MULT_UART_STATE_ACTIVE);
    CHECK(bus.tx_active);
    CHECK(sink.count == 0U);
    mult_uart_on_tx_complete_isr(&bus);
}

/* 验证空闲 abort 幂等，以及 RX 上报长度越界进入 ERROR。 */
static void test_rx_overflow_and_idle_abort(void)
{
    mult_uart_bus_t bus;
    fake_port_t fake;
    fake_event_sink_t sink = {0};
    uint8_t data[4] = {0};
    size_t call_count;

    fake_init(&fake);
    prepare_enabled_channel(&bus, &fake, &sink);
    call_count = fake.call_count;
    CHECK(mult_uart_abort(&bus) == MULT_UART_OK);
    CHECK(fake.call_count == call_count);
    CHECK(bus.current_channel == MULT_UART_CHANNEL_0);

    CHECK(mult_uart_start_rx_dma(&bus, data, sizeof(data)) == MULT_UART_OK);
    mult_uart_on_rx_complete_isr(&bus, sizeof(data) + 1U);
    CHECK(bus.state == MULT_UART_STATE_ERROR);
    CHECK(!bus.rx_active);
    CHECK(bus.current_channel ==
          (mult_uart_channel_t)MULT_UART_CHANNEL_INVALID);
    CHECK(sink.count == 1U);
    CHECK(sink.last.type == MULT_UART_EVENT_ERROR);
    CHECK(sink.last.status == MULT_UART_ERR_OVERFLOW);
    CHECK(sink.last.rx_len == sizeof(data) + 1U);
}

/* 验证 UART 错误事件、abort 恢复及空闲状态迟到 ISR 过滤。 */
static void test_error_abort_and_late_isr(void)
{
    mult_uart_bus_t bus;
    fake_port_t fake;
    fake_event_sink_t sink = {0};
    uint8_t data[4] = {0};

    fake_init(&fake);
    prepare_enabled_channel(&bus, &fake, &sink);
    CHECK(mult_uart_start_rx_dma(&bus, data, sizeof(data)) == MULT_UART_OK);
    mult_uart_on_error_isr(&bus, 0x1234U);
    CHECK(bus.state == MULT_UART_STATE_ERROR);
    CHECK(bus.current_channel ==
          (mult_uart_channel_t)MULT_UART_CHANNEL_INVALID);
    CHECK(sink.count == 1U);
    CHECK(sink.last.type == MULT_UART_EVENT_ERROR);
    CHECK(sink.last.status == MULT_UART_ERR_IO);
    CHECK(sink.last.port_error == 0x1234U);

    CHECK(mult_uart_abort(&bus) == MULT_UART_OK);
    CHECK(bus.state == MULT_UART_STATE_IDLE);
    CHECK(fake.calls[fake.call_count - 1U] == CALL_ABORT);
    mult_uart_on_rx_complete_isr(&bus, 4U);
    CHECK(sink.count == 1U);

    mult_uart_on_error_isr(&bus, 0x5678U);
    CHECK(bus.state == MULT_UART_STATE_IDLE);
    CHECK(sink.count == 1U);
}

/* 验证 abort 失败和 deinit 禁用 EN 失败时保留可诊断状态。 */
static void test_abort_and_deinit_failures(void)
{
    mult_uart_bus_t bus;
    fake_port_t fake;
    fake_event_sink_t sink = {0};
    uint8_t data[2] = {0};

    fake_init(&fake);
    prepare_enabled_channel(&bus, &fake, &sink);
    CHECK(mult_uart_start_rx_dma(&bus, data, sizeof(data)) == MULT_UART_OK);
    fake.abort_status = MULT_UART_ERR_IO;
    CHECK(mult_uart_abort(&bus) == MULT_UART_ERR_IO);
    CHECK(bus.state == MULT_UART_STATE_ERROR);
    CHECK(bus.current_channel ==
          (mult_uart_channel_t)MULT_UART_CHANNEL_INVALID);

    fake.abort_status = MULT_UART_OK;
    CHECK(mult_uart_abort(&bus) == MULT_UART_OK);
    CHECK(bus.state == MULT_UART_STATE_IDLE);

    CHECK(mult_uart_select(&bus, MULT_UART_CHANNEL_0) == MULT_UART_OK);
    fake.enable_status = MULT_UART_ERR_IO;
    CHECK(mult_uart_deinit(&bus) == MULT_UART_ERR_IO);
    CHECK(bus.initialized);
    CHECK(bus.state == MULT_UART_STATE_ERROR);
}

/* 验证有 DMA 在途时禁止 deinit，完成后可安全释放对象。 */
static void test_deinit_requires_idle(void)
{
    mult_uart_bus_t bus;
    fake_port_t fake;
    fake_event_sink_t sink = {0};
    uint8_t data[2] = {0};

    fake_init(&fake);
    prepare_enabled_channel(&bus, &fake, &sink);
    CHECK(mult_uart_start_tx_dma(&bus, data, sizeof(data)) == MULT_UART_OK);
    CHECK(mult_uart_deinit(&bus) == MULT_UART_ERR_BUSY);
    mult_uart_on_tx_complete_isr(&bus);
    CHECK(mult_uart_deinit(&bus) == MULT_UART_OK);
    CHECK(!bus.initialized);
    CHECK(bus.state == MULT_UART_STATE_UNINIT);
    CHECK(fake.calls[fake.call_count - 1U] == CALL_ENABLE_HIGH);
}

int main(void)
{
    /* 按生命周期、通道控制、DMA、错误恢复的顺序执行全部 Core 用例。 */
    test_init_and_enable_levels();
    test_invalid_port_configuration();
    test_unmanaged_enable();
    test_active_high_enable_levels();
    test_init_enable_failure_rolls_back();
    test_channel_truth_and_cache();
    test_break_before_switch();
    test_failed_break_switch_stays_disabled();
    test_failed_live_switch_disables_module();
    test_dma_events_and_start_race();
    test_full_duplex_rx_before_tx();
    test_dma_rejections_and_start_failure();
    test_rx_overflow_and_idle_abort();
    test_error_abort_and_late_isr();
    test_abort_and_deinit_failures();
    test_deinit_requires_idle();

    if (g_failures != 0) {
        (void)fprintf(stderr, "%d test(s) failed\n", g_failures);
        return 1;
    }

    (void)printf("All mult_uart core tests passed.\n");
    return 0;
}
