/* Host GPIO register fixture; production sampling still uses its real table. */
#ifndef ST_TEST_MAIN_H
#define ST_TEST_MAIN_H
#include <stdint.h>
typedef struct { volatile uint32_t IDR; } test_gpio_t;
extern test_gpio_t test_gpio;
#define GPIOC (&test_gpio)
#define L_SENSOR_1_Pin (1U << 9)
#define L_SENSOR_2_Pin (1U << 8)
#define L_SENSOR_3_Pin (1U << 7)
#define L_SENSOR_4_Pin (1U << 6)
#define L_SENSOR_5_Pin (1U << 5)
#define L_SENSOR_6_Pin (1U << 4)
#endif
