/**
 * @file    test_zdt_turntable_service.c
 * @brief   ZDT平台无关事务Service的PC fake传输测试。
 */

#include <stdio.h>
#include <string.h>

#include "zdt_turntable_service.h"

static int failures;
/** 记录失败但继续执行同一进程中的其余断言。 */
#define CHECK(x) do { if (!(x)) { printf("FAIL line %d: %s\n", __LINE__, #x); failures++; } } while (0)

/** fake UART、时基和完成结果的可观测状态。 */
typedef struct {
    zdt_turntable_service_t *service;
    uint32_t now;
    uint8_t *rx;
    size_t rx_capacity;
    unsigned tx_count;
    unsigned abort_count;
    unsigned done_count;
    zdt_turntable_status_t done_status;
    zdt_turntable_response_t response;
} fake_t;

/** @brief 模拟异步TX启动并记录启动次数。 */
static zdt_turntable_status_t fake_tx(void *ctx, const uint8_t *data, size_t len)
{
    fake_t *fake = (fake_t *)ctx;
    CHECK(data != NULL && len > 0U);
    fake->tx_count++;
    return ZDT_TURNTABLE_OK;
}

/** @brief 捕获Service提供的长期RX缓冲区，供用例注入设备响应。 */
static zdt_turntable_status_t fake_rx(void *ctx, uint8_t *data, size_t capacity)
{
    fake_t *fake = (fake_t *)ctx;
    fake->rx = data;
    fake->rx_capacity = capacity;
    return ZDT_TURNTABLE_OK;
}

/** @brief 模拟同步abort并记录调用次数。 */
static zdt_turntable_status_t fake_abort(void *ctx)
{
    ((fake_t *)ctx)->abort_count++;
    return ZDT_TURNTABLE_OK;
}

/** @brief 返回用例可控的毫秒时基。 */
static uint32_t fake_now(void *ctx) { return ((fake_t *)ctx)->now; }
/** @brief PC单线程测试无需真实worker唤醒。 */
static void fake_notify(void *ctx) { (void)ctx; }

/** @brief 捕获Service完成状态和按值复制的解析响应。 */
static void fake_done(void *ctx, uint32_t id, zdt_turntable_status_t status,
                      const zdt_turntable_response_t *response)
{
    fake_t *fake = (fake_t *)ctx;
    CHECK(id == 7U);
    fake->done_count++;
    fake->done_status = status;
    if (response != NULL) fake->response = *response;
}

/** @brief 用fake port和fake时基初始化一套独立Service。 */
static void init_service(zdt_turntable_service_t *service, fake_t *fake)
{
    zdt_turntable_service_config_t config;
    memset(service, 0, sizeof(*service));
    memset(fake, 0, sizeof(*fake));
    fake->service = service;
    memset(&config, 0, sizeof(config));
    config.port.tx_start = fake_tx;
    config.port.rx_start = fake_rx;
    config.port.abort = fake_abort;
    config.port.ctx = fake;
    config.now_ms = fake_now;
    config.time_ctx = fake;
    config.notify_worker = fake_notify;
    config.notify_ctx = fake;
    CHECK(zdt_turntable_service_init(service, &config) == ZDT_TURNTABLE_OK);
}

/** @brief 构造固定ID的0x3A状态查询事务。 */
static zdt_turntable_request_t make_request(fake_t *fake)
{
    zdt_turntable_request_t request;
    memset(&request, 0, sizeof(request));
    request.request_id = 7U;
    request.frame[0] = 1U; request.frame[1] = 0x3AU; request.frame[2] = 0x6BU;
    request.frame_len = 3U;
    request.expected_address = 1U;
    request.expected_function = 0x3AU;
    request.timeout_ms = 100U;
    request.done_cb = fake_done;
    request.user_ctx = fake;
    return request;
}

/** @brief 验证先RX后TX、响应解析、完成回调和收尾abort。 */
static void test_success(void)
{
    zdt_turntable_service_t service;
    fake_t fake;
    zdt_turntable_request_t request;
    const uint8_t reply[] = {1U, 0x3AU, 0x83U, 0x6BU};
    init_service(&service, &fake);
    request = make_request(&fake);
    CHECK(zdt_turntable_service_submit(&service, &request) == ZDT_TURNTABLE_OK);
    zdt_turntable_service_process_once(&service);
    CHECK(fake.tx_count == 1U && fake.rx != NULL);
    memcpy(fake.rx, reply, sizeof(reply));
    zdt_turntable_service_on_rx_event_isr(&service, sizeof(reply));
    zdt_turntable_service_process_once(&service);
    CHECK(fake.done_count == 1U);
    CHECK(fake.done_status == ZDT_TURNTABLE_OK);
    CHECK(fake.response.kind == ZDT_TURNTABLE_REPLY_STATUS);
    CHECK(fake.abort_count == 1U);
}

/** @brief 验证无响应达到deadline时abort并回调超时。 */
static void test_timeout(void)
{
    zdt_turntable_service_t service;
    fake_t fake;
    zdt_turntable_request_t request;
    init_service(&service, &fake);
    request = make_request(&fake);
    CHECK(zdt_turntable_service_submit(&service, &request) == ZDT_TURNTABLE_OK);
    zdt_turntable_service_process_once(&service);
    fake.now = 100U;
    zdt_turntable_service_process_once(&service);
    CHECK(fake.done_count == 1U);
    CHECK(fake.done_status == ZDT_TURNTABLE_ERR_TIMEOUT);
    CHECK(fake.abort_count == 1U);
}

/** @brief 运行全部ZDT Service PC fake测试。 */
int main(void)
{
    test_success();
    test_timeout();
    if (failures != 0) return 1;
    puts("zdt_turntable_service tests passed");
    return 0;
}
