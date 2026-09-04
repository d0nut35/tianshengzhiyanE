/**
 * @file    mult_uart_board_config.h
 * @brief   licang2026v1 的UART复用器板级策略配置。
 *
 * 引脚资源由CubeMX生成的main.h/usart.h提供；本文件只集中保存不能从
 * HAL句柄推导出的复用器策略。后续拿到复用板资料和实物后，只在这里
 * 定稿EN极性、切换方式和稳定时间，不把这些值散落到Service或设备协议。
 */

#ifndef MULT_UART_BOARD_CONFIG_H
#define MULT_UART_BOARD_CONFIG_H

/*
 * 下列默认值来自F4参考复用框架，尚未在F7真实复用板上验证：
 * - MCU管理EN；
 * - EN低电平有效；
 * - 切换A/B前先禁用EN；
 * - A/B提交后等待5us。
 *
 * 用户后续提供复用模块资料或实测结果后，必须按真实硬件修正。当前将
 * M_EN现绑定PD11并接CD4052的INH脚。低有效策略意味着PD11高电平为禁用、
 * 低电平为选通；为避免Service接管前短暂接通通道，CubeMX应把PD11初始值
 * 配置为高。当前`.ioc`和生成的`gpio.c`已经确认PD11初始为高。
 */
#define MULT_UART_BOARD_MANAGE_ENABLE          1
#define MULT_UART_BOARD_ENABLE_ACTIVE_LOW      1
#define MULT_UART_BOARD_BREAK_BEFORE_SWITCH    1
#define MULT_UART_BOARD_SWITCH_SETTLE_US       5U

#endif /* MULT_UART_BOARD_CONFIG_H */
