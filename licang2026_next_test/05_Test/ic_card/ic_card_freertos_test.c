/**
 * @file    ic_card_freertos_test.c
 * @brief   IC卡Service OS/Core/UART7完整FreeRTOS链路测试。
 *
 * 测试任务轮询USART1命令，BALL_READY只提交一次Service事务；读卡完成回调只
 * 复制结果并置线程标志，文本格式化和阻塞发送留在测试任务，避免占用Service worker。
 */

#include "ic_card_freertos_test.h"

#include <stdbool.h>
#include <string.h>

#include "cmsis_os.h"
#include "debug_uart1.h"
#include "ic_card_service.h"
#include "ic_card_test_common.h"
#include "ic_card_test_config.h"
#include "test_config.h"

#if IC_CARD_FREERTOS_TEST_ENABLED

#define IC_CARD_TEST_FLAG_DONE       (1UL << 0)
#define IC_CARD_TEST_TASK_STACK      (512U * 4U)
#define IC_CARD_TEST_POLL_MS         10U

typedef struct {
    bool initialized;
    bool read_pending;
    debug_uart1_t debug;
    osThreadId_t task;
    volatile bool result_ready;
    volatile ic_card_status_t result_status;
    uint32_t next_request_id;
    ic_result_t result;
    char text[128];
} ic_card_freertos_context_t;

static ic_card_freertos_context_t g_ic_card_freertos;

static const osThreadAttr_t g_ic_card_test_task_attr = {
    .name = "icCardTest",
    .stack_size = IC_CARD_TEST_TASK_STACK,
    .priority = (osPriority_t)osPriorityNormal,
};

/**
 * @brief 将Device完成结果按值投递给独立测试任务。
 * @param ctx FreeRTOS测试上下文。
 * @param request_id 已完成请求编号。
 * @param status 最终读球状态。
 * @param result 成功时的球结果，失败时为NULL。
 * @note 回调运行在IC Service worker中，按值复制后立刻返回，避免
 *       USART1阻塞发送拖住UART7事务调度。
 */
static void ic_card_freertos_read_done(
    void *ctx,
    uint32_t request_id,
    ic_card_status_t status,
    const ic_card_response_t *response)
{
    ic_card_freertos_context_t *test = (ic_card_freertos_context_t *)ctx;
    uint8_t block[IC_CARD_BLOCK_DATA_SIZE];

    (void)request_id;
    if (test == NULL) {
        return;
    }
    if ((status == IC_CARD_OK) && (response != NULL)) {
        status = ic_block_data(response, IC_ADDRESS, block);
        if ((status == IC_CARD_OK) &&
            !ic_decode_ball(block, &test->result.ball)) {
            status = IC_CARD_ERR_PROTOCOL;
        }
    }
    test->result_status = status;
    test->result_ready = true;
    if (test->task != NULL) {
        (void)osThreadFlagsSet(test->task, IC_CARD_TEST_FLAG_DONE);
    }
}

/**
 * @brief IC卡FreeRTOS测试任务入口。
 * @param argument 测试上下文。
 * @note 本任务是USART1和测试状态的唯一普通上下文所有者。
 */
static void ic_card_freertos_task_entry(void *argument)
{
    ic_card_freertos_context_t *test = (ic_card_freertos_context_t *)argument;
    uint8_t command[DEBUG_UART1_RX_BUFFER_SIZE];
    size_t command_len;
    size_t text_len;
    ic_card_status_t status;
    ic_card_request_t request;

    (void)debug_uart1_write_text(
        &test->debug,
        "IC CARD RTOS TEST READY. PLACE BALL THEN SEND BALL_READY\r\n");
    /* 厂家要求上电前5秒不要设置参数；这里虽然只读卡，也留出稳定时间。 */
    (void)osDelay(IC_CARD_TEST_STARTUP_DELAY_MS);

    for (;;) {
        if (test->result_ready) {
            test->result_ready = false;
            test->read_pending = false;
            if (test->result_status == IC_CARD_OK) {
                text_len = ic_card_test_format_success(
                    &test->result, test->text, sizeof(test->text));
            } else {
                text_len = ic_card_test_format_error(
                    test->result_status, test->text, sizeof(test->text));
            }
            if (text_len > 0U) {
                (void)debug_uart1_write(
                    &test->debug, (const uint8_t *)test->text, text_len);
            }
        }

        if (debug_uart1_take_message(
            &test->debug, command, sizeof(command), &command_len)) {
            if (!ic_card_test_is_read_trigger(command, command_len)) {
                (void)debug_uart1_write_text(
                    &test->debug, "UNKNOWN COMMAND. USE BALL_READY\r\n");
            } else if (test->read_pending) {
                (void)debug_uart1_write_text(
                    &test->debug, "BALL READ BUSY\r\n");
            } else {
                (void)memset(&request, 0, sizeof(request));
                ++test->next_request_id;
                if (test->next_request_id == 0U) ++test->next_request_id;
                request.request_id = test->next_request_id;
                request.type = IC_CARD_REQUEST_READ_BLOCK;
                request.address = IC_ADDRESS;
                request.timeout_ms = IC_READ_TIMEOUT_MS;
                request.data.read_block.block = IC_DATA_BLOCK;
                request.data.read_block.led_beep_prompt =
                    (IC_CARD_TEST_LED_BEEP_PROMPT != 0U);
                request.done_cb = ic_card_freertos_read_done;
                request.user_ctx = test;
                status = ic_service_submit(&request, 0U);
                if (status == IC_CARD_OK) {
                    test->read_pending = true;
                    (void)debug_uart1_write_text(
                        &test->debug, "BALL READ START\r\n");
                } else {
                    text_len = ic_card_test_format_error(
                        status, test->text, sizeof(test->text));
                    if (text_len > 0U) {
                        (void)debug_uart1_write(
                            &test->debug,
                            (const uint8_t *)test->text,
                            text_len);
                    }
                }
            }
        }

        (void)osThreadFlagsWait(
            IC_CARD_TEST_FLAG_DONE,
            osFlagsWaitAny,
            IC_CARD_TEST_POLL_MS);
    }
}

/**
 * @brief 初始化USART1调试口和独立测试任务。
 * @return 初始化成功返回IC_CARD_OK，否则回滚已建立资源。
 */
ic_card_status_t ic_card_freertos_test_init(void)
{
    ic_card_freertos_context_t *test = &g_ic_card_freertos;

    if (test->initialized) {
        return IC_CARD_ERR_STATE;
    }
    (void)memset(test, 0, sizeof(*test));
    if (!debug_uart1_init(&test->debug)) {
        return IC_CARD_ERR_IO;
    }
    test->task = osThreadNew(
        ic_card_freertos_task_entry,
        test,
        &g_ic_card_test_task_attr);
    if (test->task == NULL) {
        /* 创建任务失败时注销USART1路由，避免再次初始化留下重复handler。 */
        debug_uart1_deinit(&test->debug);
        return IC_CARD_ERR_IO;
    }
    test->initialized = true;
    return IC_CARD_OK;
}

#else

/**
 * @brief 测试未启用时的空初始化实现。
 * @return 固定返回IC_CARD_ERR_UNSUPPORTED。
 */
ic_card_status_t ic_card_freertos_test_init(void)
{
    return IC_CARD_ERR_UNSUPPORTED;
}

#endif /* IC_CARD_FREERTOS_TEST_ENABLED */
