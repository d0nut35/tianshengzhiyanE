/**
 * @file    chassis_bridge.h
 * @brief   lhy底盘子系统接入桥：UART事件路由 + 任务上下文启动入口。
 *
 * lhy/目录是队友底盘工程（ZDT四轮 + HWT101 + 灰度 + 导航）的原样拷贝。
 * 其02_Service/it_dispatch.c直接定义HAL UART全局回调，与本工程
 * 04_Bsp/uart_dispatch的唯一回调冲突，因此Keil中不编译该文件，改由
 * 本模块向uart_dispatch注册一个先过滤句柄的handler：huart3~6电机回传
 * 转交zdt_adp_rx_isr()，huart2陀螺仪转交hwt101_adp_rx_isr()。
 *
 * 本模块不解析任何协议，也不持有底盘对象；BLE(USART1)暂不接线。
 * huart与电机槽位的对应关系必须与lhy/04_Bsp/zdt_motor/zdt_motor_adaption.c
 * 中g_cfg的顺序保持一致。
 */

#ifndef CHASSIS_BRIDGE_H
#define CHASSIS_BRIDGE_H

#ifdef __cplusplus
extern "C" {
#endif

/** 桥接层状态码；lhy内部各层状态不向上透传。 */
typedef enum {
    CHASSIS_BRIDGE_OK = 0,
    CHASSIS_BRIDGE_ERR_STATE,
    CHASSIS_BRIDGE_ERR_IO,
    CHASSIS_BRIDGE_ERR_APP,
} chassis_bridge_status_t;

/**
 * @brief 向公共UART路由登记底盘所用UART的接收与错误事件handler。
 * @return CHASSIS_BRIDGE_OK表示注册成功，否则返回状态或路由表已满错误。
 * @pre main.c已完成MX_USART2/3/6与MX_UART4/5初始化。
 * @note 应在osKernelStart()之前调用；本函数不启动任何DMA，也不阻塞。
 */
chassis_bridge_status_t chassis_bridge_init(void);

/**
 * @brief 启动lhy底盘应用（调用app_init()装配电机、陀螺仪、底盘服务）。
 * @return CHASSIS_BRIDGE_OK表示应用已启动，否则返回状态或应用初始化错误。
 * @warning app_init()会阻塞直到四轮电机握手完成（无电机时无限等待），
 *          必须在调度器启动后的任务上下文调用，且只能调用一次。
 */
chassis_bridge_status_t chassis_bridge_boot(void);

#ifdef __cplusplus
}
#endif

#endif /* CHASSIS_BRIDGE_H */
