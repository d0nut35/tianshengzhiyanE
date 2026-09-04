# ZDT_X42S转盘电机UART7直连测试说明

> 2026-08-14实机状态：UART7直连只读通信已经通过，实物确认是Emm固件；机械运动、角度、方向、到位、位置变化和STOP仍待验证。

## 1. 模式选择

转盘“每次转到下一固定槽位”使用位置模式，不使用速度模式。首版推荐：

- 正常逐槽推进：`相对上一输入目标位置`（运动模式`0x00`）。连续命令以理论
  目标为基准，不会因为上一槽存在小的到位误差而把误差直接累加到下一槽。
- 调试单次角度：也使用位置模式，只把测试角度设置为安全小角度。
- 完成可靠回零和槽位标定后：可增加绝对位置模式（运动模式`0x01`），用于
  指定槽号直达及周期性消除累计坐标误差。
- `相对当前实时位置`（运动模式`0x02`）只适合临时点动；逐槽循环会把每次
  实际误差带入下一目标，因此不作为正式索引方案。

X42S同时内置X和Emm两种固件，位置命令格式不同。程序必须先发送`OPTIONS`
读取`0x1A`确认固件类型和闭环状态，再允许构造运动帧，不能只看产品型号猜测。

## 2. 当前软件与验证边界

- 已实现X固件梯形加减速位置命令`0xFD`和Emm位置命令`0xFD`。
- 已实现`VERSION/OPTIONS/STATUS/POSITION/STOP`及固定角度运动接口。
- 已实现UART7直连HAL adapter、平台无关Service、Device接口、裸机和FreeRTOS
  测试；HAL UART全局回调仍由`04_Bsp/uart_dispatch`唯一拥有。
- PC fake和Keil编译只证明协议字节、状态流与工程接入，不代表电机已转动。
- 已完成UART7直连`VERSION/OPTIONS/STATUS/POSITION`实机只读通信；没有经过复用板。
- 实机`OPTIONS`原始帧为`01 1A 00 06 6B`，对应1.8度电机、Emm固件、闭环、CW正方向、Scale关闭。
- 尚未完成CW/CCW机械运动、角度、到位、位置变化、负载和STOP验证。
- 后续接UART7复用通道2时，只替换transport，不复制ZDT协议Core。

## 3. 接线与供电

首次直连不接复用板：

```text
F7 PE8 / UART7_TX -> ZDT TTL R/A/H（电机RX）
F7 PE7 / UART7_RX <- ZDT TTL T/B/L（电机TX）
F7 GND             -> ZDT GND
ZDT电机电源         -> 10-29V独立电源，建议12V或24V、3A以上
USART1 PA9/PA10     -> CH340，用于电脑文本命令与结果观察
```

不要带电插拔供电线和电机线。首次运动应卸下转盘负载或确保不会碰撞，并准备
可立即断开的电机电源。F7和电机必须共地，但电机电源不能接到F7的3.3V/5V。

## 4. 测试开关与安全锁

在`05_Test/test_config.h`中一次只启用一种：

```c
LICANG_TEST_ZDT_TURNTABLE_BAREMETAL
LICANG_TEST_ZDT_TURNTABLE_FREERTOS
```

运动还受`05_Test/zdt_turntable/zdt_turntable_test_config.h`中的二级锁保护：

```c
#define ZDT_TURNTABLE_TEST_MOTION_ARMED 1U
```

当前现场正在进行运动实验，因此运动锁暂为1，总测试开关为
`LICANG_TEST_ZDT_TURNTABLE_FREERTOS`。烧录不会自动触发运动，但调试口发送
`MOVE_CW/MOVE_CCW`会真实驱动电机。实验结束必须把运动锁恢复0，并把总测试
开关恢复`LICANG_TEST_NONE`。

`ZDT_TURNTABLE_TEST_STEP_ANGLE_0P1DEG`单位为0.1度。当前100仅表示首次卸载
测试用10.0度，不是最终槽距。最终值应按转盘槽数和传动比计算：

```text
电机轴单槽角 = 360度 / 槽数 * 传动比
```

若电机直接驱动转盘且有N个等分槽，才可直接使用`360/N`度。

## 5. 串口命令与顺序

USART1调试串口为115200、8N1，发送ASCII文本：

```text
VERSION    读取固件/硬件版本0x1F
OPTIONS    读取X/Emm、闭环、方向和Scale位0x1A
STATUS     读取使能、到位、堵转和保护状态0x3A
POSITION   读取实时位置0x36
MOVE_CW    按配置角度顺时针位置运动（需解锁）
MOVE_CCW   按配置角度逆时针位置运动（需解锁）
STOP       立即停止0xFE
```

完整实验顺序（本轮步骤1~2已经完成，当前从步骤3继续）：

1. 保持运动锁为0，先发送`VERSION`、`OPTIONS`、`STATUS`、`POSITION`并保存原始
   UART7 TX/RX和USART1文本。
2. 确认地址1、115200、固定`0x6B`校验、闭环模式和X/Emm类型；若不同，先改
   测试配置或代码，不写电机EEPROM参数。
3. 卸载并确认方向安全后，把运动锁改1，仅执行一次`MOVE_CW`小角度测试。
4. 收到`ACK RECEIVED`只代表电机接收命令，随后发送`STATUS`确认`REACHED=1`，
   再发送`POSITION`核对实际位置。不能把`0x02`当作已经到位。
5. 执行一次`MOVE_CCW`验证反向；异常时发送`STOP`并断开电机电源。
6. 裸机通过后，再用FreeRTOS配置重复同样流程。

本轮只读实机结果：

```text
VERSION FW=201 HW_SERIES=2 HW_TYPE=3 HW_VER=20
OPTIONS原始RX：01 1A 00 06 6B
OPTIONS解析：1.8度电机、Emm、闭环、CW正方向、Scale关闭
STATUS ENABLE=1 REACHED=1 STALL=0 PROTECT=0 POWERLOSS=0
POSITION SIGN=+ RAW=3
```

小屏幕第二行显示`0.00err`是手册所示Emm固件界面，不应把其中的`err`直接
当作通信错误码。运动前还必须在电机菜单人工确认`MStep=16`、
`Response=Receive`。按当前10度、60RPM、加速度50配置，预计发送：

```text
01 FD 00 00 3C 32 00 00 00 59 00 00 6B
```

其中89脉冲对应约10.0125度；这是计算预期，不是实机角度验证结果。

## 6. 到位与后续槽位标定

本轮测试必须保持`Response=Receive`，运动命令通常先返回`地址 FD 02 6B`。
收到该帧只表示命令已接收；到位确认采用：

- 当前测试：运动后查询`STATUS`的`Prf_TF`并读取`POSITION`；
- 正式App：结合回零、槽位标定和必要的光电门反馈，不只依赖发送完成。

当前Service一笔事务只消费一帧，因此不支持`Response=Both`。在扩展多帧事务
之前不要把电机设置为Both。`STOP`同样经过软件事务FIFO，不是独立硬件急停；
发生机械危险时应直接切断电机动力电源。

位置到达窗口手册默认0.8度。是否满足槽位机械精度必须实测，不能仅因
`Prf_TF=1`就认定球槽对齐可靠。
