# mult_uart 从F4移植到F7说明

## 1. 本次交付边界

本次交付的是“已通过来源PC fake回归和F7源码编译、尚未真实硬件闭环”的底层框架：

```text
具体设备协议（后续）
        ↓
01_App/mult_uart_device
        ↓
02_Service/mult_uart_service_os
        ↓
02_Service/mult_uart_service
        ↓
03_Middleware/mult_uart Core
        ↓
04_Bsp/mult_uart STM32F7 HAL adapter
        ↓
UART7中断发送 + RX DMA + PD9/PD10/PD11
```

当前已补齐独立裸机、FreeRTOS和PC fake测试；具体外设协议仍由各设备模块负责。

## 2. 文件职责

- `03_Middleware/mult_uart/mult_uart_core.h/.c`：平台无关Core，管理通道、EN、TX/RX异步状态和ISR事件。
- `04_Bsp/mult_uart/mult_uart_stm32_hal.h/.c`：F7适配，当前绑定UART7中断发送、RX DMA1和PD9/10/11。
- `04_Bsp/mult_uart/mult_uart_board_config.h`：集中保存EN极性、break-before-switch和稳定时间等待确认策略。
- `04_Bsp/uart_dispatch/uart_dispatch.h/.c`：唯一HAL UART回调路由；后续UART8模块也应注册handler，不能再定义全局callback。
- `02_Service/mult_uart/mult_uart_service.h/.c`：平台无关静态队列、超时和事务状态机。
- `02_Service/mult_uart/mult_uart_service_os.h/.c`：CMSIS-RTOS2队列和唯一worker。
- `01_App/mult_uart_device/mult_uart_device.h/.c`：`device_id -> channel -> Service`事务入口。
- `Core/Src/freertos.c`：只做系统装配，当前初始化Service OS和Device层。
- `05_Test/host/mult_uart`：PC fake/单元测试，只由PC编译器运行，不加入Keil目标。

## 3. F7硬件绑定

```text
公共串口：UART7
RX：PE7
TX：PE8
A ：PD9 / M_A
B ：PD10 / M_B
EN/INH：PD11 / M_EN
TX：`HAL_UART_Transmit_IT()`异步中断发送，不占用DMA Stream
RX DMA：DMA1 Stream3 Channel5
UART IRQ：UART7_IRQn
```

适配器使用`HAL_UART_Transmit_IT()`和`HAL_UARTEx_ReceiveToIdle_DMA()`；RX启动后关闭HT中断，使Core只把IDLE或满缓冲视为一次RX结束。TX完成仍由`uart_dispatch`从UART7全局中断分发。`abort`同步停止HAL UART并清UART7和RX DMA的NVIC pending，避免旧ISR污染下一事务。

## 4. 正常事务流

```text
设备模块调用 mult_uart_device_submit()
→ Device检查同设备是否已有pending事务
→ Service OS在提交返回前复制TX数据并放入CMSIS队列
→ worker把请求搬入平台无关Service
→ Service选择通道并使能复用器
→ WRITE_READ先启动RX DMA，再启动TX中断发送
→ HAL回调进入uart_dispatch
→ F7 adapter按huart7过滤并上报Core事件
→ ISR只设置事件并唤醒worker
→ worker完成超时/错误/成功收敛并调用Device回调
```

`submit()`返回OK只代表排队成功；最终结果只能从完成回调获取。回调中的`rx_data`只在回调执行期间有效，协议模块必须立即解析或复制。

## 5. 当前暂定项

`mult_uart_board_config.h`当前保留F4参考值：软件管理EN、EN低有效、切换前先禁用、稳定5us。它们尚未在F7真实复用板上验证。

UART7当前固定115200，底层还没有按通道切换波特率。具体模块接入前必须由用户决定：统一设备波特率，或扩展UART7 bus service在无UART收发活动、EN禁用期间安全重配UART。

## 6. 后续模块接入规则

具体IC、ZDT、二维码和视觉模块应放在各自App/Device目录，并且：

- 只调用`mult_uart_device_submit()`；
- 不直接调用HAL UART；
- 不直接写PD9/PD10/PD11；
- 不长期保存底层RX指针；
- 不在ISR中解析协议；
- 先完成独立测试，再更换transport/通道绑定进行复用集成。
