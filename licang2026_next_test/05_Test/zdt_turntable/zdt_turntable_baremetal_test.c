/** @file zdt_turntable_baremetal_test.c @brief ZDT UART7直连裸机测试。 */

#include "zdt_turntable_baremetal_test.h"

#include <string.h>

#include "debug_uart1.h"
#include "test_config.h"
#include "uart_dispatch.h"
#include "zdt_turntable_device.h"
#include "zdt_turntable_test_common.h"
#include "zdt_turntable_test_config.h"
#include "zdt_turntable_uart7_hal.h"

#if ZDT_TURNTABLE_BAREMETAL_TEST_ENABLED

/** 裸机测试独占的完整对象集合；所有DMA缓冲均由成员对象长期持有。 */
typedef struct {
    bool initialized;
    bool pending;
    debug_uart1_t debug;
    zdt_turntable_service_t service;
    zdt_turntable_uart7_hal_t adapter;
    zdt_turntable_device_t device;
    uart_dispatch_handle_t dispatch_handle;
    char text[160];
} zdt_baremetal_context_t;

/** 单实例测试上下文，生命周期覆盖整个程序运行期。 */
static zdt_baremetal_context_t g_zdt_baremetal;

/**
 * @brief 为平台无关Service提供HAL毫秒时基。
 * @param ctx 未使用。
 * @return 当前HAL tick。
 */
static uint32_t zdt_bare_now(void *ctx)
{
    (void)ctx;
    return HAL_GetTick();
}

/**
 * @brief 裸机模式worker通知占位函数。
 * @param ctx 未使用。
 * @note 主循环持续调用process_once，因此无需额外唤醒机制。
 */
static void zdt_bare_notify(void *ctx)
{
    (void)ctx;
}

/**
 * @brief uart_dispatch的TX完成过滤handler。
 * @param ctx 裸机测试上下文。
 * @param huart 产生回调的UART句柄。
 * @return UART7事件被ZDT适配器认领时返回true。
 */
static bool zdt_bare_tx(void *ctx, UART_HandleTypeDef *huart)
{
    zdt_baremetal_context_t *test = (zdt_baremetal_context_t *)ctx;
    return zdt_turntable_uart7_hal_handle_tx_complete(&test->adapter, huart);
}

/**
 * @brief uart_dispatch的ReceiveToIdle过滤handler。
 * @param ctx 裸机测试上下文。
 * @param huart 产生回调的UART句柄。
 * @param len 本次DMA有效字节数。
 * @return UART7事件被ZDT适配器认领时返回true。
 */
static bool zdt_bare_rx(void *ctx, UART_HandleTypeDef *huart, uint16_t len)
{
    zdt_baremetal_context_t *test = (zdt_baremetal_context_t *)ctx;
    return zdt_turntable_uart7_hal_handle_rx_event(&test->adapter, huart, len);
}

/**
 * @brief uart_dispatch的UART错误过滤handler。
 * @param ctx 裸机测试上下文。
 * @param huart 产生回调的UART句柄。
 * @return UART7事件被ZDT适配器认领时返回true。
 */
static bool zdt_bare_error(void *ctx, UART_HandleTypeDef *huart)
{
    zdt_baremetal_context_t *test = (zdt_baremetal_context_t *)ctx;
    return zdt_turntable_uart7_hal_handle_error(&test->adapter, huart);
}

/**
 * @brief 在主循环上下文格式化并输出一笔事务的最终结果。
 * @param ctx 裸机测试上下文。
 * @param request_id 请求ID，本测试不展示。
 * @param status 最终状态。
 * @param response 可选解析响应。
 * @note 释放pending后才输出，后续USART1命令可继续提交。
 */
static void zdt_bare_done(
    void *ctx,
    uint32_t request_id,
    zdt_turntable_status_t status,
    const zdt_turntable_response_t *response)
{
    zdt_baremetal_context_t *test = (zdt_baremetal_context_t *)ctx;
    size_t len;
    (void)request_id;
    test->pending = false;
    len = zdt_turntable_test_format_result(
        status, response, test->text, sizeof(test->text));
    if (len > 0U) {
        (void)debug_uart1_write(&test->debug, (const uint8_t *)test->text, len);
    }
}

/**
 * @brief 将人工测试命令映射为Device语义请求。
 * @param test 裸机测试上下文。
 * @param command 已解析调试命令。
 * @return Device提交结果。
 * @warning MOVE命令只有在运动锁显式打开时才会下发；默认角度为相对上次
 *          目标位置的10.0度占位值，并非最终槽距。
 */
static zdt_turntable_status_t zdt_bare_submit(
    zdt_baremetal_context_t *test, zdt_turntable_test_command_t command)
{
    zdt_turntable_position_command_t position = {
        ZDT_TURNTABLE_DIR_CW,
        ZDT_TURNTABLE_POS_RELATIVE_LAST_TARGET,
        ZDT_TURNTABLE_TEST_X_SPEED_0P1RPM,
        ZDT_TURNTABLE_TEST_X_ACCEL_RPM_S,
        ZDT_TURNTABLE_TEST_X_DECEL_RPM_S,
        ZDT_TURNTABLE_TEST_STEP_ANGLE_0P1DEG,
        ZDT_TURNTABLE_TEST_EMM_ACCEL,
    };

    if (command == ZDT_TEST_COMMAND_OPTIONS) {
        return zdt_turntable_device_query_options(
            &test->device, zdt_bare_done, test);
    }
    if (command == ZDT_TEST_COMMAND_VERSION) {
        return zdt_turntable_device_query(&test->device, 0x1FU, zdt_bare_done, test);
    }
    if (command == ZDT_TEST_COMMAND_STATUS) {
        return zdt_turntable_device_query(&test->device, 0x3AU, zdt_bare_done, test);
    }
    if (command == ZDT_TEST_COMMAND_POSITION) {
        return zdt_turntable_device_query(&test->device, 0x36U, zdt_bare_done, test);
    }
    if (command == ZDT_TEST_COMMAND_STOP) {
        return zdt_turntable_device_stop(&test->device, zdt_bare_done, test);
    }
    if ((command == ZDT_TEST_COMMAND_MOVE_CW) ||
        (command == ZDT_TEST_COMMAND_MOVE_CCW)) {
#if ZDT_TURNTABLE_TEST_MOTION_ARMED
        position.direction = (command == ZDT_TEST_COMMAND_MOVE_CW) ?
            ZDT_TURNTABLE_DIR_CW : ZDT_TURNTABLE_DIR_CCW;
        if (test->device.firmware == ZDT_TURNTABLE_FIRMWARE_EMM) {
            position.speed = ZDT_TURNTABLE_TEST_EMM_SPEED_RPM;
        }
        return zdt_turntable_device_move_angle(
            &test->device, &position, zdt_bare_done, test);
#else
        (void)debug_uart1_write_text(
            &test->debug, "MOTION LOCKED. SET ZDT_TURNTABLE_TEST_MOTION_ARMED=1\r\n");
        return ZDT_TURNTABLE_ERR_UNSUPPORTED;
#endif
    }
    return ZDT_TURNTABLE_ERR_UNSUPPORTED;
}

/** @copydoc zdt_turntable_baremetal_test_init() */
zdt_turntable_status_t zdt_turntable_baremetal_test_init(void)
{
    zdt_baremetal_context_t *test = &g_zdt_baremetal;
    zdt_turntable_port_t port;
    zdt_turntable_service_config_t service_config;
    zdt_turntable_device_config_t device_config = {
        ZDT_TURNTABLE_TEST_ADDRESS,
        ZDT_TURNTABLE_TEST_TIMEOUT_MS,
        ZDT_TURNTABLE_TEST_EMM_PULSES_PER_REV,
    };
    uart_dispatch_handler_t handler = {0};
    zdt_turntable_status_t status;

    (void)memset(test, 0, sizeof(*test));
    test->dispatch_handle = UART_DISPATCH_HANDLE_INVALID;
    if (!debug_uart1_init(&test->debug)) return ZDT_TURNTABLE_ERR_IO;
    status = zdt_turntable_uart7_hal_bind(&test->adapter, &test->service, &port);
    if (status != ZDT_TURNTABLE_OK) return status;
    /* 全局HAL回调仍只由uart_dispatch拥有，本测试注册句柄过滤handler。 */
    handler.tx_complete = zdt_bare_tx;
    handler.rx_event = zdt_bare_rx;
    handler.error = zdt_bare_error;
    handler.user_ctx = test;
    if (!uart_dispatch_register(&handler, &test->dispatch_handle)) {
        return ZDT_TURNTABLE_ERR_IO;
    }
    (void)memset(&service_config, 0, sizeof(service_config));
    service_config.port = port;
    service_config.now_ms = zdt_bare_now;
    service_config.notify_worker = zdt_bare_notify;
    service_config.notify_ctx = test;
    status = zdt_turntable_service_init(&test->service, &service_config);
    if (status != ZDT_TURNTABLE_OK) return status;
    status = zdt_turntable_device_init(&test->device, &test->service, &device_config);
    if (status != ZDT_TURNTABLE_OK) return status;
    test->initialized = true;
    /* 只读查询必须先确认固件和闭环选项，初始化阶段不驱动电机。 */
    (void)debug_uart1_write_text(&test->debug,
        "ZDT DIRECT READY. REQUIRE RESPONSE=Receive; EMM MOVE REQUIRES MSTEP=16\r\n"
        "FIRST SEND VERSION OPTIONS STATUS POSITION\r\n");
    return ZDT_TURNTABLE_OK;
}

/** @copydoc zdt_turntable_baremetal_test_process() */
void zdt_turntable_baremetal_test_process(void)
{
    zdt_baremetal_context_t *test = &g_zdt_baremetal;
    uint8_t data[DEBUG_UART1_RX_BUFFER_SIZE];
    size_t len;
    zdt_turntable_test_command_t command;
    zdt_turntable_status_t status;

    if (!test->initialized) return;
    zdt_turntable_service_process_once(&test->service);
    if (!debug_uart1_take_message(&test->debug, data, sizeof(data), &len)) return;
    if (test->pending) {
        (void)debug_uart1_write_text(&test->debug, "ZDT BUSY\r\n");
        return;
    }
    command = zdt_turntable_test_parse_command(data, len);
    if (command == ZDT_TEST_COMMAND_INVALID) {
        (void)debug_uart1_write_text(&test->debug,
            "USE VERSION OPTIONS STATUS POSITION MOVE_CW MOVE_CCW STOP\r\n");
        return;
    }
    status = zdt_bare_submit(test, command);
    if (status == ZDT_TURNTABLE_OK) {
        test->pending = true;
        (void)debug_uart1_write_text(&test->debug, "ZDT REQUEST START\r\n");
    } else if (status != ZDT_TURNTABLE_ERR_UNSUPPORTED) {
        (void)debug_uart1_write_text(&test->debug, "ZDT SUBMIT ERROR\r\n");
    }
}

#else
/** @brief 测试开关关闭时的不可用占位入口。 */
zdt_turntable_status_t zdt_turntable_baremetal_test_init(void)
{
    return ZDT_TURNTABLE_ERR_UNSUPPORTED;
}
/** @brief 测试开关关闭时的空处理入口。 */
void zdt_turntable_baremetal_test_process(void) {}
#endif
