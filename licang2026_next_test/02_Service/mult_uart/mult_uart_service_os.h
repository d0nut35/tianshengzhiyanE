/**
 * @file    mult_uart_service_os.h
 * @brief   mult_uart Service 的 CMSIS-RTOS2 接入层。
 *
 * P4a 的 mult_uart_service 只提供平台无关 worker engine；本文件负责把它
 * 接进 CubeMX 当前生成的 CMSIS-RTOS2/FreeRTOS 环境：应用把请求放入 OS
 * message queue，后台 worker task 串行驱动真正的 Service。
 */

#ifndef MULT_UART_SERVICE_OS_H
#define MULT_UART_SERVICE_OS_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#include "mult_uart_service.h"

/**
 * @brief RTOS接入层的只读调试对象类型。
 * @note 具体CMSIS、HAL、队列和adapter字段只在.c中定义，避免应用仅为调用
 *       submit()就被迫包含STM32 HAL和RTOS实现头文件。
 */
typedef struct mult_uart_service_os mult_uart_service_os_t;

/**
 * @brief RTOS worker运行诊断快照。
 * @note 全部字段只用于测试和故障定位，业务逻辑不得依赖这些计数。
 */
typedef struct {
    uint32_t worker_loop_count;
    uint32_t os_submit_count;
    uint32_t os_dequeue_count;
    uint32_t os_queue_count;
    uint32_t notify_error_count;
    uint32_t service_submit_count;
    uint32_t service_complete_count;
    uint32_t uart_error_count;
    uint32_t last_uart_error;
} mult_uart_service_os_diagnostics_t;

/**
 * @brief 初始化默认 mult_uart RTOS service，并创建请求队列和 worker task。
 *
 * @pre main.c 已完成 MX_GPIO_Init()、MX_DMA_Init()、MX_UART7_Init()。
 * @note 应在 osKernelInitialize() 之后、osKernelStart() 之前调用；本工程放在
 *       MX_FREERTOS_Init() 的 USER CODE Init 段。
 * @return MULT_UART_OK表示成功，否则返回资源创建或底层绑定错误。
 */
mult_uart_status_t mult_uart_service_os_init(void);

/**
 * @brief 从应用任务提交一个请求。
 *
 * queue_timeout_ms 只表示“等待放入 OS 队列的时间”，不表示本次 UART 事务的
 * I/O 超时；真正的事务超时仍由 request->io_timeout_ms 控制。
 * @param request 待提交事务。
 * @param queue_timeout_ms 等待OS队列空位的最长时间。
 * @return MULT_UART_OK表示已入队，否则返回参数、状态或队列满错误。
 * @note TX数据在返回前按值复制，调用者可立即释放原始缓冲区。
 */
mult_uart_status_t mult_uart_service_os_submit(
    const mult_uart_request_t *request,
    uint32_t queue_timeout_ms);

/**
 * @brief 推进一次 OS 接入层，供 worker task 和 PC fake 测试复用。
 * @note 正式FreeRTOS路径由唯一worker拥有，业务任务不应直接调用。
 */
void mult_uart_service_os_process_once(void);

/**
 * @brief 读取底层 Service 统计信息。
 * @param stats 输出统计对象。
 * @return MULT_UART_OK表示成功，否则返回参数或未初始化错误。
 */
mult_uart_status_t mult_uart_service_os_get_stats(
    mult_uart_service_stats_t *stats);

/**
 * @brief 获取worker、OS请求队列和平台无关Service的联合诊断快照。
 * @param diagnostics 接收诊断信息的对象。
 * @return MULT_UART_OK表示成功，否则返回参数或未初始化错误。
 */
mult_uart_status_t mult_uart_service_os_get_diagnostics(
    mult_uart_service_os_diagnostics_t *diagnostics);

/**
 * @brief 暴露默认对象，方便调试器观察内部状态。
 * @return 初始化成功后返回有效对象；尚未初始化时返回NULL。
 * @note 业务层不得直接修改对象内部状态。
 */
mult_uart_service_os_t *mult_uart_service_os_get_default(void);

#ifdef __cplusplus
}
#endif

#endif /* MULT_UART_SERVICE_OS_H */
