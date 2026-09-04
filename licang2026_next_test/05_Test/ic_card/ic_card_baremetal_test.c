/**
 * @file    ic_card_baremetal_test.c
 * @brief   IC卡UART7直连裸机测试状态机。
 *
 * 电脑通过USART1发送BALL_READY后才执行一次A3读取；这对应比赛中“球已经
 * 放稳→控制器请求读取→拿到结果后才允许转盘移动”的真实时序。
 */

#include "ic_card_baremetal_test.h"

#include <string.h>

#include "debug_uart1.h"
#include "ic_ball_rule_2026.h"
#include "ic_card_device_config.h"
#include "ic_card_service.h"
#include "ic_card_test_common.h"
#include "ic_card_test_config.h"
#include "ic_card_uart7_hal.h"
#include "test_config.h"
#include "uart_dispatch.h"

#if IC_CARD_BAREMETAL_TEST_ENABLED

typedef struct {
    bool initialized;
    bool read_pending;
    uint32_t startup_deadline_ms;
    debug_uart1_t debug;
    ic_card_t device;
    ic_card_port_t port;
    ic_card_uart7_hal_t adapter;
    ic_card_service_t service;
    uart_dispatch_handle_t dispatch_handle;
    bool dispatch_registered;
    volatile bool service_event;
    char text[128];
} ic_card_baremetal_context_t;

static ic_card_baremetal_context_t g_ic_card_baremetal;

/**
 * @brief 为裸机IC Service提供HAL毫秒时基。
 * @param ctx 未使用的时间上下文。
 * @return HAL_GetTick()当前值。
 */
static uint32_t ic_card_baremetal_now_ms(void *ctx)
{
    (void)ctx;
    return HAL_GetTick();
}

/**
 * @brief 记录裸机Service收到ISR通知。
 * @param ctx 裸机测试上下文。
 * @warning ISR中只置位；主循环负责推进Service和格式化结果。
 */
static void ic_card_baremetal_notify(void *ctx)
{
    ic_card_baremetal_context_t *test = (ic_card_baremetal_context_t *)ctx;
    if (test != NULL) {
        test->service_event = true;
    }
}

/**
 * @brief 把UART7 TX完成事件转交IC HAL适配器。
 * @param ctx 裸机测试上下文。
 * @param huart 产生回调的UART句柄。
 * @return 事件被认领时返回true。
 */
static bool ic_card_baremetal_dispatch_tx(void *ctx, UART_HandleTypeDef *huart)
{
    ic_card_baremetal_context_t *test = (ic_card_baremetal_context_t *)ctx;
    return (test != NULL) && ic_card_uart7_hal_handle_tx_complete(
        &test->adapter, huart);
}

/**
 * @brief 把UART7 ReceiveToIdle事件转交IC HAL适配器。
 * @param ctx 裸机测试上下文。
 * @param huart 产生回调的UART句柄。
 * @param rx_len 本次有效字节数。
 * @return 事件被认领时返回true。
 */
static bool ic_card_baremetal_dispatch_rx(
    void *ctx,
    UART_HandleTypeDef *huart,
    uint16_t rx_len)
{
    ic_card_baremetal_context_t *test = (ic_card_baremetal_context_t *)ctx;
    return (test != NULL) && ic_card_uart7_hal_handle_rx_event(
        &test->adapter, huart, rx_len);
}

/**
 * @brief 把UART7错误事件转交IC HAL适配器。
 * @param ctx 裸机测试上下文。
 * @param huart 产生错误的UART句柄。
 * @return 事件被认领时返回true。
 */
static bool ic_card_baremetal_dispatch_error(
    void *ctx,
    UART_HandleTypeDef *huart)
{
    ic_card_baremetal_context_t *test = (ic_card_baremetal_context_t *)ctx;
    return (test != NULL) && ic_card_uart7_hal_handle_error(
        &test->adapter, huart);
}

/**
 * @brief 将一次读卡结果转换为串口助手可读文本。
 * @param ctx 裸机测试上下文。
 * @param request_id 已完成请求编号。
 * @param status 最终读取状态。
 * @param response 成功时的响应，失败时可能为NULL。
 * @note 结果按值复制并格式化后才由USART1发送，不保存Service短生命周期指针。
 */
static void ic_card_baremetal_read_done(
    void *ctx,
    uint32_t request_id,
    ic_card_status_t status,
    const ic_card_response_t *response)
{
    ic_card_baremetal_context_t *test = (ic_card_baremetal_context_t *)ctx;
    ic_card_ball_result_t result;
    size_t text_len;

    (void)request_id;
    test->read_pending = false;
    if ((status == IC_CARD_OK) && (response != NULL)) {
        (void)memset(&result, 0, sizeof(result));
        result.response = *response;
        status = ic_card_extract_block_data(
            response, IC_CARD_DEVICE_ADDRESS, result.block_data);
        if (status == IC_CARD_OK) {
            if (!ic_ball_rule_2026_decode(result.block_data, &result.ball)) {
                status = IC_CARD_ERR_PROTOCOL;
            }
        }
        if (status == IC_CARD_OK) {
            text_len = ic_card_test_format_success(
                &result, test->text, sizeof(test->text));
            if (text_len > 0U) {
                (void)debug_uart1_write(
                    &test->debug, (const uint8_t *)test->text, text_len);
            }
            return;
        }
    }
    text_len = ic_card_test_format_error(
        status, test->text, sizeof(test->text));
    if (text_len > 0U) {
        (void)debug_uart1_write(
            &test->debug, (const uint8_t *)test->text, text_len);
    }
}

/**
 * @brief 按初始化逆序撤销裸机测试资源。
 * @param test 裸机测试上下文。
 * @note 撤销已注册路由，避免下次初始化留下幽灵handler。
 */
static void ic_card_baremetal_rollback(ic_card_baremetal_context_t *test)
{
    if (test->dispatch_registered) {
        (void)uart_dispatch_unregister(test->dispatch_handle);
    }
    if (test->service.initialized && !test->service.active_valid &&
        (test->service.queue_count == 0U)) {
        (void)ic_card_service_deinit(&test->service);
    }
    if (test->device.initialized) {
        (void)ic_card_deinit(&test->device);
    }
    debug_uart1_deinit(&test->debug);
    (void)memset(test, 0, sizeof(*test));
}

/**
 * @brief 装配IC卡UART7直连裸机测试。
 * @return 初始化结果。
 * @note 只建立通信链路，必须等USART1收到BALL_READY或READ才发起读卡。
 */
ic_card_status_t ic_card_baremetal_test_init(void)
{
    ic_card_baremetal_context_t *test = &g_ic_card_baremetal;
    ic_card_uart7_hal_config_t hal_config;
    ic_card_service_config_t service_config;
    uart_dispatch_handler_t handler = {0};
    ic_card_status_t status;

    if (test->initialized) {
        return IC_CARD_ERR_STATE;
    }
    (void)memset(test, 0, sizeof(*test));
    test->dispatch_handle = UART_DISPATCH_HANDLE_INVALID;
    if (!debug_uart1_init(&test->debug)) {
        return IC_CARD_ERR_IO;
    }
    ic_card_uart7_hal_make_config(&hal_config);
    status = ic_card_uart7_hal_bind(
        &test->adapter, &test->device, &hal_config, &test->port);
    if (status != IC_CARD_OK) {
        ic_card_baremetal_rollback(test);
        return status;
    }
    handler.tx_complete = ic_card_baremetal_dispatch_tx;
    handler.rx_event = ic_card_baremetal_dispatch_rx;
    handler.error = ic_card_baremetal_dispatch_error;
    handler.user_ctx = test;
    if (!uart_dispatch_register(&handler, &test->dispatch_handle)) {
        ic_card_baremetal_rollback(test);
        return IC_CARD_ERR_IO;
    }
    test->dispatch_registered = true;
    status = ic_card_init(&test->device, &test->port);
    if (status != IC_CARD_OK) {
        ic_card_baremetal_rollback(test);
        return status;
    }
    (void)memset(&service_config, 0, sizeof(service_config));
    service_config.device = &test->device;
    service_config.now_ms = ic_card_baremetal_now_ms;
    service_config.notify_worker = ic_card_baremetal_notify;
    service_config.notify_ctx = test;
    status = ic_card_service_init(&test->service, &service_config);
    if (status != IC_CARD_OK) {
        ic_card_baremetal_rollback(test);
        return status;
    }
    test->startup_deadline_ms = HAL_GetTick() + IC_CARD_TEST_STARTUP_DELAY_MS;
    test->initialized = true;
    (void)debug_uart1_write_text(
        &test->debug,
        "IC CARD TEST READY. PLACE BALL THEN SEND BALL_READY\r\n");
    return IC_CARD_OK;
}

/**
 * @brief 在主循环推进调试命令、Service事务和超时状态机。
 * @note 必须被非阻塞地高频调用，不能放入长延时之后。
 */
void ic_card_baremetal_test_process(void)
{
    ic_card_baremetal_context_t *test = &g_ic_card_baremetal;
    uint8_t command[DEBUG_UART1_RX_BUFFER_SIZE];
    size_t command_len;
    ic_card_request_t request;
    ic_card_status_t status;

    if (!test->initialized) {
        return;
    }
    test->service_event = false;
    ic_card_service_process_once(&test->service);
    if (!debug_uart1_take_message(
        &test->debug, command, sizeof(command), &command_len)) {
        return;
    }
    if (!ic_card_test_is_read_trigger(command, command_len)) {
        (void)debug_uart1_write_text(
            &test->debug, "UNKNOWN COMMAND. USE BALL_READY\r\n");
        return;
    }
    if ((int32_t)(HAL_GetTick() - test->startup_deadline_ms) < 0) {
        (void)debug_uart1_write_text(
            &test->debug, "READER STARTUP WAIT\r\n");
        return;
    }
    if (test->read_pending) {
        (void)debug_uart1_write_text(&test->debug, "BALL READ BUSY\r\n");
        return;
    }

    (void)memset(&request, 0, sizeof(request));
    request.request_id = 1U;
    request.type = IC_CARD_REQUEST_READ_BLOCK;
    request.address = IC_CARD_DEVICE_ADDRESS;
    request.timeout_ms = IC_CARD_DEVICE_READ_TIMEOUT_MS;
    request.data.read_block.block = IC_CARD_DEVICE_DATA_BLOCK;
    request.data.read_block.led_beep_prompt =
        (IC_CARD_TEST_LED_BEEP_PROMPT != 0U);
    request.done_cb = ic_card_baremetal_read_done;
    request.user_ctx = test;
    status = ic_card_service_submit(&test->service, &request);
    if (status == IC_CARD_OK) {
        test->read_pending = true;
        (void)debug_uart1_write_text(&test->debug, "BALL READ START\r\n");
    } else {
        size_t text_len = ic_card_test_format_error(
            status, test->text, sizeof(test->text));
        if (text_len > 0U) {
            (void)debug_uart1_write(
                &test->debug, (const uint8_t *)test->text, text_len);
        }
    }
}

#else

/**
 * @brief 测试未启用时的空初始化实现。
 * @return 固定返回IC_CARD_ERR_UNSUPPORTED。
 */
ic_card_status_t ic_card_baremetal_test_init(void)
{
    return IC_CARD_ERR_UNSUPPORTED;
}

/**
 * @brief 测试未启用时的空处理函数。
 * @note 保留同名接口可避免main.c因测试开关变化而增加条件编译分支。
 */
void ic_card_baremetal_test_process(void)
{
}

#endif /* IC_CARD_BAREMETAL_TEST_ENABLED */
