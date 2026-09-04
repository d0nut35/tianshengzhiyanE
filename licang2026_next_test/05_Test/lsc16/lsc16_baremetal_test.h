/** @file lsc16_baremetal_test.h @brief LSC16 USART1命令式裸机测试入口。 */

#ifndef LSC16_BAREMETAL_TEST_H
#define LSC16_BAREMETAL_TEST_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#include "lsc16_core.h"

typedef enum {
    LSC16_BAREMETAL_PHASE_IDLE = 0,
    LSC16_BAREMETAL_PHASE_TX,
    LSC16_BAREMETAL_PHASE_ERROR,
} lsc16_baremetal_phase_t;

typedef struct {
    bool initialized;
    bool motion_armed;
    lsc16_baremetal_phase_t phase;
    uint32_t servo_tx_completed;
    uint32_t action_tx_completed;
    uint32_t action_started_reports;
    uint32_t action_completed_reports;
    uint32_t invalid_reports;
    lsc16_status_t last_status;
    lsc16_report_t last_report;
} lsc16_baremetal_test_status_t;

/** @brief 初始化UART8 LSC16与USART1电脑命令端口，不自动发送运动。 */
lsc16_status_t lsc16_baremetal_test_init(void);

/** @brief 在裸机主循环消费USART1命令并输出事务与回报文本。 */
void lsc16_baremetal_test_process(void);

/** @brief 获取测试状态快照，保留给程序化诊断使用。 */
lsc16_status_t lsc16_baremetal_test_get_status(
    lsc16_baremetal_test_status_t *status);

#ifdef __cplusplus
}
#endif

#endif /* LSC16_BAREMETAL_TEST_H */
