#ifndef HWT_TEST_USART_H
#define HWT_TEST_USART_H

#include <stdint.h>

typedef enum {
    HAL_OK = 0,
    HAL_ERROR,
    HAL_BUSY,
    HAL_TIMEOUT
} HAL_StatusTypeDef;

typedef struct {
    uint32_t disabled;
} DMA_HandleTypeDef;

typedef struct {
    DMA_HandleTypeDef *hdmarx;
    uint32_t disabled;
} UART_HandleTypeDef;

extern UART_HandleTypeDef huart2;

#define UART_IT_IDLE 0x10U
#define DMA_IT_HT 0x08U
#define __HAL_UART_DISABLE_IT(uart, mask) ((uart)->disabled |= (mask))
#define __HAL_DMA_DISABLE_IT(dma, mask) ((dma)->disabled |= (mask))
#define __get_PRIMASK() test_irq_get()
#define __disable_irq() test_irq_disable()
#define __set_PRIMASK(mask) test_irq_restore(mask)

uint32_t test_irq_get(void);
void test_irq_disable(void);
void test_irq_restore(uint32_t mask);
void test_parse_enter(void);
void test_scan_step(void);
HAL_StatusTypeDef HAL_UARTEx_ReceiveToIdle_DMA(
    UART_HandleTypeDef *uart, uint8_t *data, uint16_t len);
HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *uart, uint8_t *data,
                                  uint16_t len, uint32_t timeout);

#endif
