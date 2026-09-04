# LSC16 USART1命令测试说明

## 1. 测试方式

LSC16舵控板继续使用UART8连接F7，电脑通过USART1连接CH340。裸机和FreeRTOS
测试上电后只等待电脑命令，不再自动执行单舵机和动作组，也不要求使用Keil
Watch判断测试结果。

```text
F7 PE1 / UART8_TX -> LSC16 RX
F7 PE0 / UART8_RX <- LSC16 TX
F7 GND             -> LSC16 GND
F7 PA9 / USART1_TX -> CH340 RX
F7 PA10/ USART1_RX <- CH340 TX
F7 GND             -> CH340 GND
```

UART8为`9600, 8-N-1`，USART1串口助手为`115200, 8-N-1`。舵机电源必须由
舵控板的合适独立电源提供，不能由F7的3.3V引脚带动。

## 2. 测试开关和运动锁

在`05_Test/test_config.h`中二选一：

```c
LICANG_TEST_LSC16_BAREMETAL
LICANG_TEST_LSC16_FREERTOS
```

运动还受`05_Test/lsc16/lsc16_test_config.h`保护：

```c
#define LSC16_TEST_MOTION_ARMED 0U
```

保持0时，`SERVO`和`ACTION`只返回锁定提示，不发送运动帧；`STOP`、`BATTERY`
和`STATUS`仍然可以使用。确认舵机ID、目标位置、动作时间、动作组编号和机械安全
后才临时改为1。实验结束必须恢复0，并将活动测试恢复`LICANG_TEST_NONE`。

## 3. USART1命令及正常现象

串口助手以ASCII文本发送，允许附带CR/LF：

```text
STATUS   输出运动锁、事务忙碌、发送及回报计数
BATTERY  查询舵控板电池电压
SERVO    按测试配置移动一次单舵机（需解锁）
ACTION   按测试配置运行一次动作组（需解锁）
STOP     停止当前动作组，始终允许发送
```

上电正常提示：

```text
LSC16 BAREMETAL READY. USE SERVO ACTION STOP BATTERY STATUS
```

或：

```text
LSC16 RTOS READY. USE SERVO ACTION STOP BATTERY STATUS
```

保持运动锁为0发送`SERVO`或`ACTION`，应看到：

```text
MOTION LOCKED. SET LSC16_TEST_MOTION_ARMED=1
```

此时舵机不得运动。

发送`BATTERY`后，正常过程为：

```text
BATTERY REQUEST START
BATTERY TX DONE. WAIT BATTERY REPORT
BATTERY 7400 mV
```

电压值以实物返回为准。只有`BATTERY ... mV`表示收到了控制板回报，TX DONE只
表示查询命令已发完。

解锁后发送`SERVO`，正常过程为：

```text
SERVO REQUEST START
SERVO TX DONE. OBSERVE ACTUAL MOTION
```

LSC16协议没有规定单舵机`0x03`成功应答，因此还必须观察指定ID舵机是否按测试
配置移动到安全位置。`SERVO TX DONE`不能作为机械到位证据。

解锁后发送`ACTION`，正常过程为：

```text
ACTION REQUEST START
ACTION TX DONE. WAIT ACTION STARTED/COMPLETED
ACTION STARTED GROUP=0 REPEAT=1
ACTION COMPLETED GROUP=0 REPEAT=1
```

组号以测试配置为准。必须同时看到实际动作、`ACTION STARTED`和
`ACTION COMPLETED`，才说明动作组命令、开始回报和自然完成回报都正常。

发送`STOP`后可能看到：

```text
STOP REQUEST START
STOP TX DONE. WAIT ACTION STOPPED OR OBSERVE MOTION
ACTION STOPPED
```

若控制板未回传停止帧，仍需以机械动作实际停止为准。危险情况下不要只依赖串口
命令，应直接切断舵机电源。

## 4. 推荐实机顺序

1. 保持运动锁为0，启用裸机测试，确认READY提示。
2. 发送`STATUS`、`BATTERY`，确认USART1和UART8双向通信。
3. 发送`SERVO`、`ACTION`，确认均被运动锁拦截且机械不动。
4. 人工核对安全参数后临时解锁，只发送一次`SERVO`并观察实际方向和范围。
5. 确认安全动作组已下载后发送一次`ACTION`，观察动作及开始/完成回报。
6. 在动作组运行时测试`STOP`，确认停止效果及可能的停止回报。
7. 切换FreeRTOS测试，重复上述完整流程。
8. 测试结束恢复运动锁0和`LICANG_TEST_NONE`。

当前只有软件实现、PC fake和Keil编译验证；上述串口现象与机械动作均尚未在真实
F7、LSC16和舵机上验证。
