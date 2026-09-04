/**
 * @file    test_zdt_turntable_device.c
 * @brief   ZDT Device固件分流、角度换算和Scale语义的PC fake测试。
 */

#include <stdio.h>
#include <string.h>

#include "zdt_turntable_device.h"

static int failures;
/** 记录失败但继续执行同一进程中的其余断言。 */
#define CHECK(x) do { if (!(x)) { printf("FAIL line %d: %s\n", __LINE__, #x); failures++; } } while (0)

typedef struct {
    unsigned submitted;
    zdt_turntable_request_t request;
} injected_submit_t;

/** @brief 捕获Device经抽象提交接口生成的事务。 */
static zdt_turntable_status_t fake_injected_submit(
    void *ctx, const zdt_turntable_request_t *request)
{
    injected_submit_t *fake = (injected_submit_t *)ctx;
    fake->request = *request;
    fake->submitted++;
    return ZDT_TURNTABLE_OK;
}

/** fake transport保存最近一次TX帧和Service提供的RX缓冲区。 */
typedef struct {
    uint32_t now;
    uint8_t tx[ZDT_TURNTABLE_FRAME_MAX];
    size_t tx_len;
    uint8_t *rx;
    size_t rx_capacity;
} fake_t;

/** @brief 捕获Device经Service提交的最终协议帧。 */
static zdt_turntable_status_t fake_tx(void *ctx, const uint8_t *data, size_t len)
{
    fake_t *fake = (fake_t *)ctx;
    CHECK(data != NULL && len <= sizeof(fake->tx));
    memcpy(fake->tx, data, len);
    fake->tx_len = len;
    return ZDT_TURNTABLE_OK;
}

/** @brief 捕获Service的长期RX缓冲区，本组构帧测试不注入设备响应。 */
static zdt_turntable_status_t fake_rx(void *ctx, uint8_t *data, size_t capacity)
{
    fake_t *fake = (fake_t *)ctx;
    fake->rx = data;
    fake->rx_capacity = capacity;
    return ZDT_TURNTABLE_OK;
}

/** @brief 模拟同步清理传输状态。 */
static zdt_turntable_status_t fake_abort(void *ctx)
{
    (void)ctx;
    return ZDT_TURNTABLE_OK;
}

/** @brief 返回用例可控的毫秒时基。 */
static uint32_t fake_now(void *ctx) { return ((fake_t *)ctx)->now; }
/** @brief PC单线程测试不需要真实worker通知。 */
static void fake_notify(void *ctx) { (void)ctx; }

/**
 * @brief 初始化一套独立Service和Device，并注入已确认的固件选项。
 * @param service 输出Service实例。
 * @param device 输出Device实例。
 * @param fake 输出fake transport状态。
 * @param firmware 本用例使用的固件类型。
 * @param scaled_input OPTIONS bit7状态。
 */
static void init_device(
    zdt_turntable_service_t *service,
    zdt_turntable_device_t *device,
    fake_t *fake,
    zdt_turntable_firmware_t firmware,
    bool scaled_input)
{
    zdt_turntable_service_config_t service_config;
    zdt_turntable_device_config_t device_config = {1U, 500U, 3200U};

    memset(service, 0, sizeof(*service));
    memset(device, 0, sizeof(*device));
    memset(fake, 0, sizeof(*fake));
    memset(&service_config, 0, sizeof(service_config));
    service_config.port.tx_start = fake_tx;
    service_config.port.rx_start = fake_rx;
    service_config.port.abort = fake_abort;
    service_config.port.ctx = fake;
    service_config.now_ms = fake_now;
    service_config.time_ctx = fake;
    service_config.notify_worker = fake_notify;
    service_config.notify_ctx = fake;
    CHECK(zdt_turntable_service_init(service, &service_config) ==
          ZDT_TURNTABLE_OK);
    CHECK(zdt_turntable_device_init(device, service, &device_config) ==
          ZDT_TURNTABLE_OK);
    device->firmware_known = true;
    device->firmware = firmware;
    device->closed_loop = true;
    device->scaled_input = scaled_input;
}

/** @brief 提交运动请求并推进一次Service，使fake捕获发送帧。 */
static zdt_turntable_status_t submit_move(
    zdt_turntable_service_t *service,
    zdt_turntable_device_t *device,
    const zdt_turntable_position_command_t *command)
{
    zdt_turntable_status_t status = zdt_turntable_device_move_angle(
        device, command, NULL, NULL);
    if (status == ZDT_TURNTABLE_OK) {
        zdt_turntable_service_process_once(service);
    }
    return status;
}

/** @brief 验证Emm角度转脉冲的四舍五入和Scale速度补偿。 */
static void test_emm_conversion_and_scale(void)
{
    zdt_turntable_service_t service;
    zdt_turntable_device_t device;
    fake_t fake;
    zdt_turntable_position_command_t command = {
        ZDT_TURNTABLE_DIR_CW,
        ZDT_TURNTABLE_POS_RELATIVE_LAST_TARGET,
        60U, 0U, 0U, 100U, 50U
    };

    init_device(&service, &device, &fake, ZDT_TURNTABLE_FIRMWARE_EMM, false);
    CHECK(submit_move(&service, &device, &command) == ZDT_TURNTABLE_OK);
    CHECK(fake.tx_len == 13U);
    CHECK(fake.tx[3] == 0x00U && fake.tx[4] == 0x3CU);
    CHECK(fake.tx[6] == 0x00U && fake.tx[7] == 0x00U &&
          fake.tx[8] == 0x00U && fake.tx[9] == 0x59U);

    init_device(&service, &device, &fake, ZDT_TURNTABLE_FIRMWARE_EMM, true);
    CHECK(submit_move(&service, &device, &command) == ZDT_TURNTABLE_OK);
    CHECK(fake.tx[3] == 0x02U && fake.tx[4] == 0x58U);

    command.speed = 301U;
    CHECK(zdt_turntable_device_move_angle(
        &device, &command, NULL, NULL) == ZDT_TURNTABLE_ERR_PARAM);
}

/** @brief 验证X固件Scale保持上层0.1度角度单位不变。 */
static void test_x_scale(void)
{
    zdt_turntable_service_t service;
    zdt_turntable_device_t device;
    fake_t fake;
    zdt_turntable_position_command_t command = {
        ZDT_TURNTABLE_DIR_CCW,
        ZDT_TURNTABLE_POS_ABSOLUTE_ZERO,
        600U, 200U, 200U, 100U, 0U
    };

    init_device(&service, &device, &fake, ZDT_TURNTABLE_FIRMWARE_X, true);
    CHECK(submit_move(&service, &device, &command) == ZDT_TURNTABLE_OK);
    CHECK(fake.tx_len == 16U);
    CHECK(fake.tx[9] == 0x00U && fake.tx[10] == 0x00U &&
          fake.tx[11] == 0x03U && fake.tx[12] == 0xE8U);
}

/** @brief 验证绝对零位置经Device层也会生成零脉冲Emm帧。 */
static void test_emm_absolute_zero(void)
{
    zdt_turntable_service_t service;
    zdt_turntable_device_t device;
    fake_t fake;
    zdt_turntable_position_command_t command = {
        ZDT_TURNTABLE_DIR_CW,
        ZDT_TURNTABLE_POS_ABSOLUTE_ZERO,
        60U, 0U, 0U, 0U, 50U
    };

    init_device(&service, &device, &fake, ZDT_TURNTABLE_FIRMWARE_EMM, false);
    CHECK(submit_move(&service, &device, &command) == ZDT_TURNTABLE_OK);
    CHECK(fake.tx[6] == 0U && fake.tx[7] == 0U &&
          fake.tx[8] == 0U && fake.tx[9] == 0U);
    CHECK(fake.tx[10] == ZDT_TURNTABLE_POS_ABSOLUTE_ZERO);
}

/** @brief 验证复用链路可以绕过直连Service注入事务提交函数。 */
static void test_injected_submit(void)
{
    zdt_turntable_device_t device = {0};
    zdt_turntable_device_config_t config = {1U, 500U, 3200U};
    injected_submit_t fake = {0};

    CHECK(zdt_turntable_device_init_with_submit(
        &device, fake_injected_submit, &fake, &config) == ZDT_TURNTABLE_OK);
    CHECK(zdt_turntable_device_query(
        &device, 0x1FU, NULL, NULL) == ZDT_TURNTABLE_OK);
    CHECK(fake.submitted == 1U);
    CHECK(fake.request.expected_address == 1U);
    CHECK(fake.request.expected_function == 0x1FU);
    CHECK(fake.request.frame_len == 3U);
    CHECK(fake.request.frame[0] == 1U && fake.request.frame[1] == 0x1FU &&
          fake.request.frame[2] == ZDT_TURNTABLE_CHECK_BYTE);
}

/** @brief 运行全部ZDT Device PC fake测试。 */
int main(void)
{
    test_emm_conversion_and_scale();
    test_x_scale();
    test_emm_absolute_zero();
    test_injected_submit();
    if (failures != 0) return 1;
    puts("zdt_turntable_device tests passed");
    return 0;
}
