# UART复用板四通道CH340测试说明

> 2026-08-14旧PE9/PE10/PE11绑定下，修正公共端接线后裸机和FreeRTOS四通道流程均已通过。2026-08-17选择脚已迁移为PD9/PD10/PD11；新引脚尚需重新执行本说明的回归，PD11 EN/INH及切换波形仍未定量测量。

## 1. 测试目标

本测试在四个复用通道尚未连接正式设备时，用4个CH340验证：

- UART7 DMA发送和ReceiveToIdle DMA接收；
- PD9/PD10选择线与PD11 EN/INH时序；
- 通道0~3的真实通路、隔离和编号；
- 裸机Core/HAL链路；
- FreeRTOS Device/Service/Core/HAL完整链路。

这是硬件验收程序，不是比赛正式App。

## 2. 测试拓扑

```text
STM32F7 UART7
      │
  复用板公共端
      ├─ 通道0 ─ CH340-0 ─ 电脑串口助手0
      ├─ 通道1 ─ CH340-1 ─ 电脑串口助手1
      ├─ 通道2 ─ CH340-2 ─ 电脑串口助手2
      └─ 通道3 ─ CH340-3 ─ 电脑串口助手3
```

F7引脚：

```text
PE7  = UART7_RX
PE8  = UART7_TX
PD9  = M_A
PD10 = M_B
PD11 = M_EN / CD4052 INH
```

本块复用板公共端丝印采用板卡视角，已由用户实机确认的成功接线是：

```text
F7 PE8 / UART7_TX -> 复用板公共TXD
F7 PE7 / UART7_RX <- 复用板公共RXD
F7 GND             -> 复用板GND
```

这里的`公共TXD/RXD`不能按普通独立串口设备的TX/RX交叉规则猜测。此前按
`F7 TX -> 公共RX、F7 RX <- 公共TX`连接时无正常现象，反过来后裸机和
FreeRTOS测试均正常。

CH340必须使用3.3V TTL电平并与F7/复用板共地。串口信号交叉连接：

```text
复用通道TXn -> CH340 RXD
复用通道RXn <- CH340 TXD
GND              <-> CH340 GND
```

未确认供电方案前，不要把4个CH340的VCC线同时并到目标板电源。

## 3. 串口助手设置

四个串口全部设置为：

```text
115200 baud
8 data bits
No parity
1 stop bit
No flow control
```

接收区使用ASCII/文本显示。发送切换命令时使用HEX发送，不能把
`FF 01 AA`当成八个ASCII字符发出。

## 4. 测试协议

上电默认选择通道0，并立即发送：

```text
now is channel 0\r\n
```

之后当前通道每3秒发送一次同类提示。

切换命令必须从“当前已选通道”的CH340发送：

```text
FF 00 AA    切到通道0
FF 01 AA    切到通道1
FF 02 AA    切到通道2
FF 03 AA    切到通道3
```

只有一次ReceiveToIdle收到的数据恰好为3字节，且符合`FF xx AA`时，才按切换命令解析。合法切换后，新通道会立即发送一次：

```text
now is channel N\r\n
```

同时从此时刻重新开始3秒周期。

非法命令，例如`FF 04 AA`，保持原通道并返回：

```text
invalid channel\r\n
```

其他数据转换为大写HEX纯文本回显。例如在通道0发送`12 34 AB`，返回：

```text
channel 0:12 34 AB\r\n
```

单次ReceiveToIdle测试缓冲为32字节。超过32字节的连续数据可能被分成多次文本回显，本测试不把它当作具体设备协议帧。

## 5. 单点测试开关

只修改：

```text
05_Test/test_config.h
```

默认正式状态：

```c
#define LICANG_ACTIVE_TEST LICANG_TEST_NONE
```

裸机测试：

```c
#define LICANG_ACTIVE_TEST LICANG_TEST_MULT_UART_BAREMETAL
```

FreeRTOS完整链路测试：

```c
#define LICANG_ACTIVE_TEST LICANG_TEST_MULT_UART_FREERTOS
```

两种测试通过同一宏选择，无法同时编译为活动路径。测试结束后必须恢复`LICANG_TEST_NONE`。

## 6. 推荐实机顺序

1. 不接正式IC、ZDT、二维码和Nano，只连4个CH340。
2. 先启用裸机测试，Clean/Rebuild并下载。
3. 确认只有CH340-0立即收到通道0提示，其他三路保持静默。
4. 从当前窗口依次发送`FF 01 AA`、`FF 02 AA`、`FF 03 AA`、`FF 00 AA`，检查提示是否转移到唯一目标窗口。
5. 每个通道发送普通HEX数据，检查纯文本回显。
6. 发送`FF 04 AA`，确认原通道返回`invalid channel`且不切换。
7. 裸机通过后改为FreeRTOS测试，Clean/Rebuild后重复同样流程。
8. 两种路径通过后恢复`LICANG_TEST_NONE`。

## 7. 本轮实机结论与剩余板级参数

已完成：

- 裸机模式的默认通道提示、四通道切换、普通数据回显和非法通道处理通过；
- FreeRTOS模式重复同一完整流程通过；
- 公共端实际接线方向已按上文确认；
- 基本通道编号、提示转移和未选通通道隔离符合测试预期。

尚未定量确认：

当前代码中的EN低有效、break-before-switch和5us稳定延时是沿用F4参考值，不是F7复用板的实机结论。

- PD11准确有效电平及上电、复位、重新烧录时的瞬态；
- A/B/EN切换波形、break-before-switch实际空窗和5us稳定时间；
- 不同正式设备、不同波特率下的通道切换和事务恢复。

如果上电无任何通道输出，不要立即修改协议逻辑，应先核对：

- PD11 EN/INH有效电平；
- 复用板公共端与UART7 TX/RX的方向；
- 通道TXn/RXn与CH340 TXD/RXD的交叉连接；
- A/B真值与实际通道编号；
- 所有设备是否共地及CH340是否为3.3V TTL。

## 8. 与比赛正式调度的边界

FreeRTOS测试为了随时接收电脑命令，会连续提交最长3秒的`WRITE_READ`测试事务。这种长接收窗口只属于验收夹具。

比赛正式App中，IC、ZDT、二维码和视觉必须使用各自真实的短事务超时，收到应答后立即释放复用总线，不能直接复用本测试的3秒占用策略。
