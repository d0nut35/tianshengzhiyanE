/** @file zdt_turntable_freertos_test.c @brief ZDT UART7直连FreeRTOS测试。 */

#include "zdt_turntable_freertos_test.h"

#include <stdbool.h>
#include <string.h>

#include "cmsis_os.h"
#include "debug_uart1.h"
#include "test_config.h"
#include "uart_dispatch.h"
#include "zdt_turntable_device.h"
#include "zdt_turntable_test_common.h"
#include "zdt_turntable_test_config.h"
#include "zdt_turntable_uart7_hal.h"

#if ZDT_TURNTABLE_FREERTOS_TEST_ENABLED

#define ZDT_TEST_TASK_STACK       (512U * 4U)
#define ZDT_TEST_FLAG_EVENT       (1UL << 0)

/**
 * @brief FreeRTOS测试独占上下文。
 * @note ISR只经Service写事件标志；result由完成回调写、唯一测试任务消费。
 */
typedef struct {
    bool initialized;
    bool pending;
    debug_uart1_t debug;
    zdt_turntable_service_t service;
    zdt_turntable_uart7_hal_t adapter;
    zdt_turntable_device_t device;
    uart_dispatch_handle_t dispatch_handle;
    osThreadId_t task;
    volatile bool result_ready;
    volatile zdt_turntable_status_t result_status;
    zdt_turntable_response_t result;
    char text[160];
} zdt_freertos_context_t;

/** 单实例测试上下文，生命周期覆盖整个程序运行期。 */
static zdt_freertos_context_t g_zdt_freertos;

/** ZDT测试worker属性；栈大小按CMSIS-RTOS2字节单位填写。 */
static const osThreadAttr_t g_zdt_task_attr = {
    .name = "zdtTurntableTest",
    .stack_size = ZDT_TEST_TASK_STACK,
    .priority = (osPriority_t)osPriorityNormal,
};

/**
 * @brief 为Service提供RTOS tick时基。
 * @param ctx 未使用。
 * @return 当前内核tick；本工程tick周期为1ms。
 */
static uint32_t zdt_rtos_now(void *ctx)
{
    (void)ctx;
    return osKernelGetTickCount();
}

/**
 * @brief 从提交路径或UART ISR唤醒唯一测试worker。
 * @param ctx FreeRTOS测试上下文。
 * @note osThreadFlagsSet允许从ISR调用；这里只发布事件，不解析协议。
 */
static void zdt_rtos_notify(void *ctx)
{
    zdt_freertos_context_t *test = (zdt_freertos_context_t *)ctx;
    if ((test != NULL) && (test->task != NULL)) {
        (void)osThreadFlagsSet(test->task, ZDT_TEST_FLAG_EVENT);
    }
}

/**
 * @brief uart_dispatch的TX完成过滤handler。
 * @param ctx FreeRTOS测试上下文。
 * @param huart 产生回调的UART句柄。
 * @return UART7事件被认领时返回true。
 */
static bool zdt_rtos_tx(void *ctx, UART_HandleTypeDef *huart)
{
    zdt_freertos_context_t *test = (zdt_freertos_context_t *)ctx;
    return zdt_turntable_uart7_hal_handle_tx_complete(&test->adapter, huart);
}

/**
 * @brief uart_dispatch的ReceiveToIdle过滤handler。
 * @param ctx FreeRTOS测试上下文。
 * @param huart 产生回调的UART句柄。
 * @param len 本次DMA有效字节数。
 * @return UART7事件被认领时返回true。
 */
static bool zdt_rtos_rx(void *ctx, UART_HandleTypeDef *huart, uint16_t len)
{
    zdt_freertos_context_t *test = (zdt_freertos_context_t *)ctx;
    return zdt_turntable_uart7_hal_handle_rx_event(&test->adapter, huart, len);
}

/**
 * @brief uart_dispatch的UART错误过滤handler。
 * @param ctx FreeRTOS测试上下文。
 * @param huart 产生回调的UART句柄。
 * @return UART7事件被认领时返回true。
 */
static bool zdt_rtos_error(void *ctx, UART_HandleTypeDef *huart)
{
    zdt_freertos_context_t *test = (zdt_freertos_context_t *)ctx;
    return zdt_turntable_uart7_hal_handle_error(&test->adapter, huart);
}

/**
 * @brief 缓存Service完成结果并唤醒测试worker。
 * @param ctx FreeRTOS测试上下文。
 * @param request_id 请求ID，本测试不展示。
 * @param status 最终事务状态。
 * @param response 可选解析响应，存在时立即按值复制。
 * @note 回调由同一worker内的process_once触发，不直接写USART1。
 */
static void zdt_rtos_done(
    void *ctx,
    uint32_t request_id,
    zdt_turntable_status_t status,
    const zdt_turntable_response_t *response)
{
    zdt_freertos_context_t *test = (zdt_freertos_context_t *)ctx;
    (void)request_id;
    test->result_status = status;
    if (response != NULL) test->result = *response;
    test->result_ready = true;
    zdt_rtos_notify(test);
}

/**
 * @brief 将人工测试命令映射为Device语义请求。
 * @param test FreeRTOS测试上下文。
 * @param command 已解析调试命令。
 * @return Device提交结果。
 * @warning MOVE使用相对上次目标位置模式和10.0度占位值。是否允许真实运动
 *          由zdt_turntable_test_config.h中的显式安全锁决定，不能依据本注释猜测。
 */
static zdt_turntable_status_t zdt_rtos_submit(
    zdt_freertos_context_t *test, zdt_turntable_test_command_t command)
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

    if (command == ZDT_TEST_COMMAND_OPTIONS)
        return zdt_turntable_device_query_options(&test->device, zdt_rtos_done, test);
    if (command == ZDT_TEST_COMMAND_VERSION)
        return zdt_turntable_device_query(&test->device, 0x1FU, zdt_rtos_done, test);
    if (command == ZDT_TEST_COMMAND_STATUS)
        return zdt_turntable_device_query(&test->device, 0x3AU, zdt_rtos_done, test);
    if (command == ZDT_TEST_COMMAND_POSITION)
        return zdt_turntable_device_query(&test->device, 0x36U, zdt_rtos_done, test);
    if (command == ZDT_TEST_COMMAND_STOP)
        return zdt_turntable_device_stop(&test->device, zdt_rtos_done, test);
    if ((command == ZDT_TEST_COMMAND_MOVE_CW) ||
        (command == ZDT_TEST_COMMAND_MOVE_CCW)) {
#if ZDT_TURNTABLE_TEST_MOTION_ARMED
        position.direction = (command == ZDT_TEST_COMMAND_MOVE_CW) ?
            ZDT_TURNTABLE_DIR_CW : ZDT_TURNTABLE_DIR_CCW;
        if (test->device.firmware == ZDT_TURNTABLE_FIRMWARE_EMM)
            position.speed = ZDT_TURNTABLE_TEST_EMM_SPEED_RPM;
        return zdt_turntable_device_move_angle(
            &test->device, &position, zdt_rtos_done, test);
#else
        (void)debug_uart1_write_text(&test->debug,
            "MOTION LOCKED. SET ZDT_TURNTABLE_TEST_MOTION_ARMED=1\r\n");
        return ZDT_TURNTABLE_ERR_UNSUPPORTED;
#endif
    }
    return ZDT_TURNTABLE_ERR_UNSUPPORTED;
}

/**
 * @brief 唯一ZDT测试worker：推进Service、输出结果并消费USART1命令。
 * @param argument FreeRTOS测试上下文。
 * @note 10 tick有限等待同时保证无UART事件时仍能推进事务超时判断。
 */
static void zdt_rtos_task(void *argument)
{
    zdt_freertos_context_t *test = (zdt_freertos_context_t *)argument;
    uint8_t data[DEBUG_UART1_RX_BUFFER_SIZE];
    size_t data_len;
    size_t text_len;
    zdt_turntable_test_command_t command;
    zdt_turntable_status_t status;

    (void)debug_uart1_write_text(&test->debug,
        "ZDT RTOS DIRECT READY. REQUIRE RESPONSE=Receive; EMM MOVE REQUIRES MSTEP=16\r\n"
        "FIRST SEND VERSION OPTIONS STATUS POSITION\r\n");
    for (;;) {
        zdt_turntable_service_process_once(&test->service);
        if (test->result_ready) {
            test->result_ready = false;
            test->pending = false;
            text_len = zdt_turntable_test_format_result(
                test->result_status,
                (test->result_status == ZDT_TURNTABLE_OK ||
                 test->result_status == ZDT_TURNTABLE_ERR_DEVICE) ?
                    &test->result : NULL,
                test->text,
                sizeof(test->text));
            if (text_len > 0U)
                (void)debug_uart1_write(
                    &test->debug, (const uint8_t *)test->text, text_len);
        }
        if (debug_uart1_take_message(
                &test->debug, data, sizeof(data), &data_len)) {
            if (test->pending) {
                (void)debug_uart1_write_text(&test->debug, "ZDT BUSY\r\n");
            } else {
                command = zdt_turntable_test_parse_command(data, data_len);
                if (command == ZDT_TEST_COMMAND_INVALID) {
                    (void)debug_uart1_write_text(&test->debug,
                        "USE VERSION OPTIONS STATUS POSITION MOVE_CW MOVE_CCW STOP\r\n");
                } else {
                    status = zdt_rtos_submit(test, command);
                    if (status == ZDT_TURNTABLE_OK) {
                        test->pending = true;
                        (void)debug_uart1_write_text(
                            &test->debug, "ZDT REQUEST START\r\n");
                    } else if (status != ZDT_TURNTABLE_ERR_UNSUPPORTED) {
                        (void)debug_uart1_write_text(
                            &test->debug, "ZDT SUBMIT ERROR\r\n");
                    }
                }
            }
        }
        (void)osThreadFlagsWait(
            ZDT_TEST_FLAG_EVENT, osFlagsWaitAny, 10U);
    }
}

/** @copydoc zdt_turntable_freertos_test_init() */
zdt_turntable_status_t zdt_turntable_freertos_test_init(void)
{
    zdt_freertos_context_t *test = &g_zdt_freertos;
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
    handler.tx_complete = zdt_rtos_tx;
    handler.rx_event = zdt_rtos_rx;
    handler.error = zdt_rtos_error;
    handler.user_ctx = test;
    if (!uart_dispatch_register(&handler, &test->dispatch_handle))
        return ZDT_TURNTABLE_ERR_IO;
    (void)memset(&service_config, 0, sizeof(service_config));
    service_config.port = port;
    service_config.now_ms = zdt_rtos_now;
    service_config.notify_worker = zdt_rtos_notify;
    service_config.notify_ctx = test;
    status = zdt_turntable_service_init(&test->service, &service_config);
    if (status != ZDT_TURNTABLE_OK) return status;
    status = zdt_turntable_device_init(&test->device, &test->service, &device_config);
    if (status != ZDT_TURNTABLE_OK) return status;
    test->task = osThreadNew(zdt_rtos_task, test, &g_zdt_task_attr);
    if (test->task == NULL) return ZDT_TURNTABLE_ERR_IO;
    test->initialized = true;
    return ZDT_TURNTABLE_OK;
}

#else
/** @brief 测试开关关闭时的不可用占位入口。 */
zdt_turntable_status_t zdt_turntable_freertos_test_init(void)
{
    return ZDT_TURNTABLE_ERR_UNSUPPORTED;
}
#endif
