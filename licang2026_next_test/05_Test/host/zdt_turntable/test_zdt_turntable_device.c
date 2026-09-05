/** @file test_zdt_turntable_device.c @brief ZDT Device语义主机测试。 */

#include <stdio.h>
#include <string.h>

#include "zdt_turntable_device.h"

static int failures;
#define CHECK(x) do { if (!(x)) { printf("FAIL line %d: %s\n", __LINE__, #x); failures++; } } while (0)

typedef struct {
    unsigned submitted;
    zdt_turntable_status_t status;
    zdt_turntable_request_t request;
} fake_submit_t;

static zdt_turntable_status_t fake_submit(
    void *ctx,
    const zdt_turntable_request_t *request)
{
    fake_submit_t *fake = (fake_submit_t *)ctx;

    fake->request = *request;
    ++fake->submitted;
    return fake->status;
}

static void init_device(
    zdt_turntable_device_t *device,
    fake_submit_t *fake,
    zdt_turntable_firmware_t firmware,
    bool scaled_input)
{
    zdt_turntable_device_config_t config = {1U, 500U, 3200U};

    (void)memset(device, 0, sizeof(*device));
    (void)memset(fake, 0, sizeof(*fake));
    CHECK(zdt_turntable_device_init_with_submit(
        device, fake_submit, fake, &config) == ZDT_TURNTABLE_OK);
    device->firmware_known = true;
    device->firmware = firmware;
    device->closed_loop = true;
    device->scaled_input = scaled_input;
}

static void test_emm_conversion_and_scale(void)
{
    zdt_turntable_device_t device;
    fake_submit_t fake;
    zdt_turntable_position_command_t command = {
        ZDT_TURNTABLE_DIR_CW,
        ZDT_TURNTABLE_POS_RELATIVE_LAST_TARGET,
        60U, 0U, 0U, 100U, 50U
    };

    init_device(&device, &fake, ZDT_TURNTABLE_FIRMWARE_EMM, false);
    CHECK(zdt_turntable_device_move_angle(
        &device, &command, NULL, NULL) == ZDT_TURNTABLE_OK);
    CHECK(fake.request.frame_len == 13U);
    CHECK(fake.request.frame[3] == 0x00U && fake.request.frame[4] == 0x3CU);
    CHECK(fake.request.frame[6] == 0x00U && fake.request.frame[7] == 0x00U &&
          fake.request.frame[8] == 0x00U && fake.request.frame[9] == 0x59U);

    init_device(&device, &fake, ZDT_TURNTABLE_FIRMWARE_EMM, true);
    CHECK(zdt_turntable_device_move_angle(
        &device, &command, NULL, NULL) == ZDT_TURNTABLE_OK);
    CHECK(fake.request.frame[3] == 0x02U && fake.request.frame[4] == 0x58U);
    command.speed = 301U;
    CHECK(zdt_turntable_device_move_angle(
        &device, &command, NULL, NULL) == ZDT_TURNTABLE_ERR_PARAM);
}

static void test_x_scale(void)
{
    zdt_turntable_device_t device;
    fake_submit_t fake;
    zdt_turntable_position_command_t command = {
        ZDT_TURNTABLE_DIR_CCW,
        ZDT_TURNTABLE_POS_ABSOLUTE_ZERO,
        600U, 200U, 200U, 100U, 0U
    };

    init_device(&device, &fake, ZDT_TURNTABLE_FIRMWARE_X, true);
    CHECK(zdt_turntable_device_move_angle(
        &device, &command, NULL, NULL) == ZDT_TURNTABLE_OK);
    CHECK(fake.request.frame_len == 16U);
    CHECK(fake.request.frame[9] == 0x00U && fake.request.frame[10] == 0x00U &&
          fake.request.frame[11] == 0x03U && fake.request.frame[12] == 0xE8U);
}

static void test_emm_absolute_zero(void)
{
    zdt_turntable_device_t device;
    fake_submit_t fake;
    zdt_turntable_position_command_t command = {
        ZDT_TURNTABLE_DIR_CW,
        ZDT_TURNTABLE_POS_ABSOLUTE_ZERO,
        60U, 0U, 0U, 0U, 50U
    };

    init_device(&device, &fake, ZDT_TURNTABLE_FIRMWARE_EMM, false);
    CHECK(zdt_turntable_device_move_angle(
        &device, &command, NULL, NULL) == ZDT_TURNTABLE_OK);
    CHECK(fake.request.frame[6] == 0U && fake.request.frame[7] == 0U &&
          fake.request.frame[8] == 0U && fake.request.frame[9] == 0U);
    CHECK(fake.request.frame[10] == ZDT_TURNTABLE_POS_ABSOLUTE_ZERO);
}

static void test_query_and_minimal_paths(void)
{
    zdt_turntable_device_t device;
    fake_submit_t fake;
    zdt_turntable_position_command_t command = {
        ZDT_TURNTABLE_DIR_CW,
        ZDT_TURNTABLE_POS_RELATIVE_LAST_TARGET,
        60U, 0U, 0U, 100U, 50U
    };

    init_device(&device, &fake, ZDT_TURNTABLE_FIRMWARE_EMM, false);
    CHECK(zdt_turntable_device_query(
        &device, 0x1FU, NULL, NULL) == ZDT_TURNTABLE_OK);
    CHECK(fake.request.expected_address == 1U);
    CHECK(fake.request.expected_function == 0x1FU);
    CHECK(fake.request.frame_len == 3U);

    CHECK(zdt_turntable_device_move_emm_angle(
        &device, &command, NULL, NULL) == ZDT_TURNTABLE_OK);
    CHECK(fake.request.expected_function == 0xFDU);
    CHECK(fake.request.frame_len == 13U);
    CHECK(zdt_turntable_device_query_status(
        &device, NULL, NULL) == ZDT_TURNTABLE_OK);
    CHECK(fake.submitted == 3U);
    CHECK(fake.request.expected_function == 0x3AU);
    CHECK(fake.request.frame_len == 3U);
    CHECK(zdt_turntable_device_stop(
        &device, NULL, NULL) == ZDT_TURNTABLE_OK);
    CHECK(fake.request.expected_function == 0xFEU);
}

int main(void)
{
    test_emm_conversion_and_scale();
    test_x_scale();
    test_emm_absolute_zero();
    test_query_and_minimal_paths();
    if (failures != 0) return 1;
    puts("zdt_turntable_device tests passed");
    return 0;
}
