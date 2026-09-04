# UART7复用模块FreeRTOS集成测试说明

## 1. 本实验验证什么

电脑通过USART1调试口发送ASCII命令，F7在FreeRTOS下把设备请求提交给
`mult_uart_device`。复用Service负责选择UART7通道、收发DMA、超时和恢复，
设备协议仍由原IC卡和ZDT Core/Device负责。

| 通道 | 设备 | 本轮能力 |
| --- | --- | --- |
| 0 | 视觉/Nano | 仅验证`SELECT 0`，协议尚未提供 |
| 1 | M2940B-HA IC卡 | 查询参数、读取比赛球 |
| 2 | ZDT_X42S上层转盘 | 版本、选项、状态、位置及受锁保护的运动 |
| 3 | 预留 | 仅验证`SELECT 3`，二维码已取消 |

LSC16固定使用UART8，不经过UART7复用器，也不属于本测试。

当前复用板通道切换、通道1 IC读球、通道2 ZDT只读查询及实际运动均已在真实F7
上完成基础验证；PB0也已实测为“螺丝对准时1、未对准时0”。新增的
`ZDT_SLOT_CW`自动逐槽状态机已经接入；引入后全部既有PC fake回归继续通过，Keil
目标也完成编译链接。现有PC fake没有直接模拟PB0和整段自动状态机，本命令本身仍
需按本文步骤完成真实转盘回归，不能用编译通过代替实机结果。

## 2. 首次上电前检查

1. F7、复用板、IC卡TTL地、ZDT TTL地、调试CH340必须共地。
2. UART信号必须是3.3 V TTL；不要接RS232电平。
3. M2940B-HA按厂家V1.0规格书使用`3.3 V`直流供电，UART也是`3.3 V TTL`；
   厂家建议电源入口并联一个不小于`22 uF`的电容。
4. ZDT使用10-29 V独立电源，建议12 V或24 V、3 A以上；不能接F7的3.3 V/5 V。
5. 不带电插拔ZDT电源线和电机线。运动前卸载或清空碰撞区域，并准备硬件断电。
6. ZDT先确认`MStep=16`、`Response=Receive`。代码使用地址1、115200、固定
   `0x6B`校验和Emm每圈3200脉冲。
7. IC卡应为地址`0x20`、115200、命令模式，且内部Key A与比赛球一致。

## 3. 接线

### 3.1 F7到复用板

公共端采用复用板丝印视角。以下接法已在四通道CH340实验中实测成功，不能改成
普通设备常见的交叉命名接法：

```text
F7 PE8  / UART7_TX -> 复用板公共TXD
F7 PE7  / UART7_RX <- 复用板公共RXD
F7 PD9  / M_A       -> 复用板A
F7 PD10 / M_B       -> 复用板B
F7 PD11 / M_EN      -> 复用板EN/INH
F7 GND              -> 复用板GND
```

不要用“空闲时PD11一定高或低”单独判断程序是否正常；通道选择和收发由事务状态机
管理。若测量A/B/EN，应以命令前后的波形和通道真值为准。

### 3.2 IC卡接通道1

```text
复用板TX1 -> IC卡RX
复用板RX1 <- IC卡TX
复用板GND <-> IC卡GND
IC卡电源  -> 按模块标注供电
```

PB1/`IC_CARD_IO`可选接IC卡IO；当前读球事务不依赖PB1。

### 3.3 ZDT接通道2

```text
复用板TX2 -> ZDT TTL R/A/H（电机RX）
复用板RX2 <- ZDT TTL T/B/L（电机TX）
复用板GND <-> ZDT GND
ZDT电源   -> 10-29 V独立电源
```

只使用ZDT的TTL接口，不要把TTL线接到RS485接口。

### 3.4 USART1调试口

```text
F7 PA9  / USART1_TX -> CH340 RXD
F7 PA10 / USART1_RX <- CH340 TXD
F7 GND               -> CH340 GND
```

串口助手设置为`115200, 8-N-1`、无流控、ASCII发送和接收。建议发送CRLF；代码也
接受不带CRLF的单条发送。

## 4. 编译和上电现象

测试总开关位于`05_Test/test_config.h`：

```c
#define LICANG_ACTIVE_TEST LICANG_TEST_MULT_UART_MODULES_FREERTOS
```

ZDT运动锁位于`05_Test/mult_uart_modules/mult_uart_modules_test_config.h`，首次必须保持：

```c
#define MULT_UART_MODULES_ZDT_MOTION_ARMED 0U
```

烧录并复位后，程序先等待约5.2秒。在此期间不会自动发送模块协议，也不会让电机
运动；随后USART1应输出：

```text
UART7 MODULES FREERTOS READY. SEND HELP
```

若没有READY，先检查USART1的PA9/PA10、CH340电平、共地和波特率。READY只证明
调试任务启动，不证明IC或ZDT通信成功。
首次真实通道选择完成前，`STATUS`显示`SELECTED=-1`；这表示底层尚未
写入A/B并稳定通道，不是通道0。收到`SELECT CHn OK`后才显示对应数字。

## 5. 命令规则

发送`HELP`可看到完整命令。任意时刻只允许一笔设备命令在途；上一笔未完成时发送
新命令会返回`BUSY: WAIT CURRENT COMMAND`。测试层另有2秒完成通知看门狗；若内部
通知链异常，会输出`INTERNAL COMPLETION TIMEOUT: COMMAND GATE RELEASED`并解除命令门，
避免一次异常造成后续所有命令永久BUSY。该提示表示软件完成链异常，不表示外设成功。

设备命令会自动选择正确通道，不要求先手动`SELECT`。`IC_READ`自动使用通道1，
`ZDT_STATUS`自动使用通道2。`SELECT n`只切换通道，不发送设备协议。

### 5.1 基础通道切换

依次发送：

```text
STATUS
SELECT 0
SELECT 1
SELECT 2
SELECT 3
```

预期分别看到当前状态以及`SELECT CH0 OK`到`SELECT CH3 OK`。这一步证明GPIO选择和
复用Service事务完成，但不证明通道上的设备能通信。若用逻辑分析仪测量，A/B应随
0到3变化，EN在break-before-switch期间有短暂关闭过程。

新版`STATUS`还会输出三个内部诊断计数：

```text
SELECT_CB=<SELECT完成回调次数> EVENT_DROP=<协议事件投递失败次数> WATCHDOG=<超时解锁次数>
```

正常情况下，每成功执行一次`SELECT n`，`SELECT_CB`增加1，`EVENT_DROP`和`WATCHDOG`
保持0。`SELECT`不启动UART TX/RX DMA，也不等待Nano、IC卡或ZDT回包。

随后还会输出worker链路计数：

```text
OS WORKER_LOOP=<循环数> OS_SUB=<OS入队> OS_DEQ=<OS出队> OS_Q=<当前排队> NOTIFY_ERR=<唤醒错误> SVC_SUB=<Service提交> SVC_DONE=<Service完成> UART_ERR=<UART错误次数> UART_LAST=<最后错误位>
```

一次正常`SELECT`应满足`OS_SUB=OS_DEQ=SVC_SUB=SVC_DONE=1`、`OS_Q=0`，且
`WORKER_LOOP`持续增加。若`OS_SUB=1、OS_DEQ=0、OS_Q=1`，说明请求已经进入OS队列，
但worker没有搬运；若`SVC_SUB=1、SVC_DONE=0`，说明请求已进入Service但没有收敛完成。
`UART_LAST`按STM32 HAL位值解释：`00000001=PE`、`00000002=NE`、
`00000004=FE`、`00000008=ORE`、`00000010=DMA`；多项可按位同时出现。

### 5.2 PB0光电门原始电平

发送`GATE`直接读取PB0，不使用UART7复用通道。返回`GATE PB0=0`或
`GATE PB0=1`。当前实机已经重复确认：螺丝远离时为0，对准时为1；自动逐槽命令
仍会连续采样3次高电平才宣布到位。详细接线、电平安全和记录表见
`00_Doc/PB0光电门_FreeRTOS测试说明.md`。

### 5.3 IC卡通道1

先把球移开，依次发送查询命令，每条完成后再发下一条，建议间隔至少100 ms：

```text
IC_ADDRESS
IC_MODE
IC_BEEPER
IC_AUTO
```

成功输出格式如下：

```text
IC CMD=B0 STATUS=00 RAW=...
IC CMD=B1 STATUS=00 RAW=...
IC CMD=B2 STATUS=00 RAW=...
IC CMD=B8 STATUS=00 RAW=...
```

以`D:\DONUT\硬件资料\IC卡`中的厂家手册为准，当前参数的判读方法是：

| 命令 | F7发送 | 正常回复及含义 |
| --- | --- | --- |
| `IC_ADDRESS` | `02 08 B0 00 00 00 00 45` | 地址为`0x20`时应为`02 08 B0 00 00 20 00 65` |
| `IC_MODE` | `02 08 B1 20 00 00 00 64` | 9字节；第6字节`01`表示命令模式，第7字节是自动读块号，第8字节是上传模式 |
| `IC_BEEPER` | `02 08 B2 20 00 00 00 67` | 开启时应为`02 08 B2 20 00 01 00 66` |
| `IC_AUTO` | `02 08 B8 20 00 00 00 6D` | “读一次”时应为`02 08 B8 20 00 00 00 6D`；连续读为`01` |

必须用`RAW`原始字节和手册核对实际参数，不能只看`STATUS=00`猜参数内容。

放上比赛IC球并发送`IC_READ`。按厂家《高频读写器使用手册V1.0.5》和
STM32示例代码，F7发送开启本次蜂鸣/灯提示的块1帧
`01 08 A3 20 01 01 00 75`。合法球示例：

```text
BALL OK CODE=0x23 ROW=2 COL=3 DATA=23 23 ... 23
```

取走球再发`IC_READ`，应在约500 ms后得到IC超时或无卡错误，系统不能死锁；重新
放球后再次发送应能恢复。蜂鸣不等于F7已经解析出合法球值。

### 5.4 ZDT通道2只读测试

保持运动锁为0，依次发送：

```text
ZDT_VERSION
ZDT_OPTIONS
ZDT_STATUS
ZDT_POSITION
```

当前直连实测过的参考响应是：

```text
VERSION FW=201 HW_SERIES=2 HW_TYPE=3 HW_VER=20
OPTIONS RAW=0006 MOTOR=1.8 FW=EMM CLOSED=1 DIR=CW ...
STATUS ENABLE=1 REACHED=1 STALL=0 PROTECT=0 ...
POSITION SIGN=+ RAW=<当前位置原始值>
```

`ZDT_OPTIONS`必须成功确认`FW=EMM`、`CLOSED=1`，Device才允许生成运动帧。若返回
`ZDT ERROR 6`通常是超时，优先检查通道2、TTL接线、地址、115200和共地。

## 6. ZDT小角度运动测试

只读测试稳定、机械区域安全且硬件断电手段就绪后，才把运动锁改为1并重新编译烧录。
程序仍不会上电自动运动。每次复位后先发送`ZDT_OPTIONS`，再执行：

```text
ZDT_POSITION
ZDT_CW
ZDT_STATUS
ZDT_POSITION
ZDT_CCW
ZDT_STATUS
ZDT_POSITION
```

当前单槽测试使用相对上一输入目标位置模式，电机目标角度150.2度、Emm速度
60 RPM、加速度50。实机连续五次约30度电机运动后，转盘刚好移动一个约30度
槽位，说明机构约为5:1传动；五次累计位置约27341 RAW。新参数按`MStep=16`
换算为1335脉冲，理论电机角度约150.1875度，单次位置变化预计约27341 RAW。
连续等角度
命令以理论目标为基准，但正式流程仍必须等待到位后再推进状态机。`ZDT_CW`或
`ZDT_CCW`返回`ACK RECEIVED. NOT POSITION REACHED`只表示接收命令，不表示机械
到位。应观察约一个槽位运动，并用`REACHED=1`及位置变化确认。实际CW/CCW方向必须标定。

光电门精定位另提供四条位置模式命令：`ZDT_COARSE_CW/CCW`以60 RPM粗走电机
140.0度，`ZDT_FINE_CW/CCW`以15 RPM每次微调电机1.0度。上电初始PB0应为1；
粗调到位后可能直接落入下一螺丝的高电平窗口，也可能为0；为0时沿同一方向逐次
微调，每一步都等`REACHED=1`后读取`GATE`。实测前两个槽位粗调后直接为1，后续
槽位曾需要2次微调，说明必须保留PB0闭环，不能只使用固定角度。手动命令用于标定。

### 6.1 单命令自动逐槽测试

新增双向命令：

```text
ZDT_SLOT_CW
ZDT_SLOT_CCW
```

两条命令使用同一状态机，但粗定位和后续微调始终保持命令指定的同一方向：

```text
确认起点PB0连续3次为1
-> 位置模式粗走电机140.0度/60 RPM
-> 每50 ms查询一次ZDT_STATUS，等待REACHED=1
-> PB0连续3次为1：完成
-> 否则位置模式微调电机1.0度/15 RPM
-> 再次等待REACHED并检查PB0
-> 最多微调10次，总流程最多8秒
```

每次复位后必须先成功执行一次`ZDT_OPTIONS`，并确认起始螺丝已经对准、`GATE`
返回1，再发送`ZDT_SLOT_CW`或`ZDT_SLOT_CCW`。正常输出示例：

```text
SLOT START DIR=CW COARSE=140.0DEG
SLOT OK DIR=CW FINE=0 PB0=1
```

若粗定位后需要两次微调，则最终行为：

```text
SLOT OK DIR=CW FINE=2 PB0=1
```

反向命令使用相同输出格式：

```text
SLOT START DIR=CCW COARSE=140.0DEG
SLOT OK DIR=CCW FINE=<实际次数> PB0=1
```

CW一圈实测微调次数为`5、0、0、2、2、10、10、6、0`；最后一个未安装螺丝的
位置在10次微调后正确返回`GATE_NOT_FOUND`，没有误报到位。这证明当前光电门闭环
方法现阶段可行，也说明不同螺丝的安装误差较大，暂不应仅按平均值提高粗定位角度。
CCW会受到传动反向间隙影响，必须另做一整圈测试，不能直接把CW数据当成反向标定。

执行期间其他设备命令会返回`BUSY: WAIT CURRENT COMMAND`；`HELP`、测试层`STATUS`
和原始`GATE`仍可读取。测试层`STATUS`中的`SLOT_ACTIVE=1`表示自动流程正在运行，
`SLOT_STATE`是内部阶段编号，`SLOT_FINE`是已经执行的微调次数。

常见拒绝或失败输出：

| 输出 | 含义与处理 |
| --- | --- |
| `SLOT ERROR SEND ZDT_OPTIONS FIRST` | 本次复位后尚未识别Emm固件；先发送`ZDT_OPTIONS` |
| `SLOT ERROR START PB0 NOT STABLY HIGH` | 起点没有稳定对准；先手动校准到`GATE PB0=1` |
| `SLOT ERROR DIR=... GATE_NOT_FOUND` | 10次微调后仍未找到螺丝；也可能是该位置尚未安装螺丝 |
| `SLOT ERROR DIR=... MOTOR_FAULT` | 电机未使能、堵转、保护或掉电锁存；停止机械实验并排查 |
| `SLOT ERROR DIR=... TIMEOUT` | 整个自动定位超过8秒，不能把该槽位当成到位 |

第一轮请连续执行一整圈，每次都记录`FINE`值并肉眼确认确实移动到相邻槽位。
稳定标准建议为：每次均输出`SLOT OK`、最终PB0为1、没有跳槽，且微调次数始终
不超过3。若经常接近10次，应重新标定粗定位角度，不要简单扩大无限搜索范围。

`ZDT_STOP`仍经过软件FIFO并等待串口应答，不是硬实时急停。发生碰撞、失控或异常
噪声时直接硬件断电/关闭驱动，不要等待串口STOP。

完成运动实验后把运动锁恢复0。全部实验结束后把总开关恢复`LICANG_TEST_NONE`并
重新编译，防止测试任务进入正式业务固件。

## 7. 常见故障定位

| 现象 | 优先检查 |
| --- | --- |
| 无READY | USART1 PA9/PA10、CH340 3.3 V TTL、115200、共地；等待5.2秒 |
| `SELECT CHn ERROR` | PD9/PD10/PD11、复用板供电、Device/Service初始化 |
| `PENDING=1`持续超过2秒 | 查看`SELECT_CB/EVENT_DROP/WATCHDOG`；这是测试层完成通知异常，不是通道外设没回包 |
| SELECT正常但设备超时 | 通道端TXn/RXn交叉、设备供电/地址/波特率、通道号 |
| IC查询有回包但读球失败 | 地址0x20、命令模式、Key A、块1、16字节球数据 |
| ZDT VERSION正常但OPTIONS错误 | 第二代手册、地址1、0x6B、完整5字节0x1A返回 |
| ZDT只读正常但运动锁提示 | 预期安全行为；完成机械检查后修改专用锁并重编译 |
| ZDT ACK但不转或不到位 | 电机电源、使能、闭环、堵转/保护、速度和机械负载 |
| `ZDT_SLOT_CW`立即拒绝 | 先执行`ZDT_OPTIONS`并让起点PB0稳定为1；确认运动锁已解锁 |
| 自动逐槽超过10次微调 | 停止测试，检查光电门、螺丝间距、传动间隙和140度粗定位参数 |
| 烧录后偶发异常 | 完全断电重上电并等READY；检查上电瞬态、松动接线和共地 |

## 8. 通过判据

只有在真实F7上完成以下项目后才能标记实机通过：

1. USART1稳定输出READY并可重复接收命令。
2. 四个`SELECT`均成功，通道切换和隔离无串扰。
3. IC通道1四项查询、有效球、无球超时和超时后恢复均通过。
4. ZDT通道2四项只读查询重复通过，OPTIONS确认Emm闭环配置。
5. 解锁后10度CW/CCW、状态到位、位置变化和异常时硬件停止符合预期。
6. 保存日期、接线、供电、设备参数、原始回包和异常现象。

PC fake通过只证明构帧、解析、transport转换和回调生命周期；Keil
`0 Error / 0 Warning`只证明目标工程能编译链接，两者都不能替代实机现象。
