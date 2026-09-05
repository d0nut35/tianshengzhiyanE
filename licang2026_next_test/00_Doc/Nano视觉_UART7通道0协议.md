# Nano视觉 UART7通道0协议

## 1. 边界

- 物理链路：UART7复用通道0，`115200, 8-N-1`，无流控。
- F7拥有事务和复用板控制权；Nano不得操作复用板选择脚或直接控制其他通道。
- 旧V1轮询保留供固定假数据和回退测试；轻量抓取使用V2会话事件模式，等待目标期间不发送轮询。
- F7端所有通道0事务必须通过`mult_uart_device_submit()`提交，设备号0映射通道0。
- 整场任务需要在同一进程内切换圆盘和阶梯三层参数，Nano正式启动必须使用
  `--scene auto --mode auto`；固定场景启动只用于单场景标定和诊断。
- 本协议只冻结当前最小球识别语义，不包含底盘、圆盘区、积木数字或完整比赛状态机。
- Nano只给出颜色和相对抓取中心的偏差；`BALL_ALIGNED`由F7按容差和连续帧数判定。

## 2. 通用帧

所有多字节整数均为小端。

| 偏移 | 长度 | 字段 | 说明 |
|---:|---:|---|---|
| 0 | 1 | SOF0 | `0xA5` |
| 1 | 1 | SOF1 | `0x5A` |
| 2 | 1 | VERSION | 当前`0x01` |
| 3 | 1 | TYPE | 见消息类型表 |
| 4 | 1 | SEQ | F7递增；Nano原样回显 |
| 5 | 1 | PAYLOAD_LEN | 当前最大24字节 |
| 6 | N | PAYLOAD | 消息载荷 |
| 6+N | 2 | CRC16 | CRC-16/CCITT-FALSE，小端 |

CRC参数：多项式`0x1021`、初值`0xFFFF`、不反射、无最终异或；计算范围从`VERSION`到载荷末尾，不包含SOF和CRC。标准字符串`123456789`的结果为`0x29B1`。

消息类型：

| TYPE | 方向 | 语义 |
|---:|---|---|
| `0x01` | F7→Nano | 兼容轮询POLL |
| `0x02` | F7→Nano | SESSION_START |
| `0x03` | F7→Nano | SESSION_STOP |
| `0x04` | F7→Nano | EVENT_ACK |
| `0x81` | Nano→F7 | 兼容观测OBSERVATION |
| `0x82` | Nano→F7 | SESSION_READY |
| `0x83` | Nano→F7 | VISION_EVENT |
| `0x84` | Nano→F7 | SESSION_STOPPED |

## 3. F7轮询 `TYPE=0x01`

载荷固定2字节：

| 偏移 | 字段 | 值 |
|---:|---|---|
| 0 | SCENE | `1=BALL_TURNTABLE`，`2=BALL_STAIR_LOW`，`3=BALL_STAIR_HIGH`，`4=BALL_STAIR_MID` |
| 1 | TARGET_COLOR | `0=任意`，`1=红`，`2=蓝` |

## 4. Nano观测 `TYPE=0x81`

载荷固定12字节：

| 偏移 | 长度 | 字段 | 说明 |
|---:|---:|---|---|
| 0 | 1 | SCENE | 回显实际处理场景 |
| 1 | 1 | STATUS | `0=无目标`，`1=有效`，`2=相机错误`，`3=模式未就绪` |
| 2 | 1 | COLOR | `0=无`，`1=红`，`2=蓝`；有效观测不能为0 |
| 3 | 1 | QUALITY | `0~100`，只用于诊断和后续门限 |
| 4 | 2 | OFFSET_X_PX | 有符号像素偏差；正数表示球心在抓取中心右侧 |
| 6 | 2 | OFFSET_Y_PX | 有符号像素偏差；正数表示球心在抓取中心下侧 |
| 8 | 2 | FRAME_ID | Nano相机帧序号，允许自然回绕 |
| 10 | 2 | AGE_MS | 观测生成到发送响应的年龄，过旧结果不得触发对齐 |

## 5. V2会话事件模式

### 5.1 SESSION_START / SESSION_READY

载荷均为4字节：`SESSION_ID(u16) + SCENE(u8) + TARGET_COLOR(u8)`。
`SESSION_ID`由F7递增且不能为0；READY必须回显同一个会话、场景和颜色。

F7发送START时通过`mult_uart_device_submit()`执行WRITE_READ。收到匹配READY后，
F7通过同一接口提交READ并保持通道0等待视觉事件。等待期间Nano不发送无目标帧，
F7也不发送POLL；IC和ZDT事务必须等视觉会话结束后再提交。

### 5.2 VISION_EVENT

载荷固定14字节：`SESSION_ID(u16) + OBSERVATION原12字节载荷`。Nano只有在：

1. 收到START之后的新相机帧；
2. 场景和目标颜色匹配；
3. fast ROI、面积和结果年龄均满足；

时才发送EVENT。一个session只锁存第一个合格事件；未收到ACK时按固定间隔重发
相同SESSION_ID和FRAME_ID，不能用下一帧覆盖尚未确认的事件。

### 5.3 EVENT_ACK

载荷固定4字节：`SESSION_ID(u16) + FRAME_ID(u16)`。F7只有在CRC、会话、场景、
颜色、结果年龄以及启动后新帧检查全部通过后才ACK。Nano收到匹配ACK后关闭本session。
F7在ACK发送完成后才允许触发动作组12，保证Nano不会继续把同一球作为新事件发送。

### 5.4 SESSION_STOP / SESSION_STOPPED

载荷均为`SESSION_ID(u16)`。取消识别时F7发送STOP；Nano清除待发事件并回复STOPPED。
超时情况下F7允许清理本地session，但不得因此触发机械动作。

### 5.5 多球时序

```text
动作11完成 -> 新SESSION_START -> READY -> 等待一次EVENT -> ACK -> 动作12
动作12完成 -> 动作11 -> 新SESSION_ID重新开始
```

上一球的EVENT因SESSION_ID不同不能触发下一球；单球抓取完成后不会自动重新识别。

## 6. V1轮询兼容判定语义

1. F7发出轮询并保存SEQ，只有SEQ一致且CRC正确的观测才能更新状态。
2. 场景、目标颜色、结果年龄和X/Y偏差全部满足配置时，才累计一次有效对齐样本。
3. 任一帧不满足条件、协议错误或事务超时，连续计数立即清零。
4. 必须达到配置的连续确认帧数才产生`BALL_ALIGNED`；禁止单帧触发抓取。
5. 当前Core的PC测试参数仅为假数据示例：X容差10 px、Y容差8 px、结果年龄100 ms、连续3帧、连续3次超时离线。真实参数必须在机械安装后分别标定`BALL_TURNTABLE`、`BALL_STAIR_LOW`、`BALL_STAIR_HIGH`和`BALL_STAIR_MID`。
6. F7链路超时或连续事务超时后标记Nano离线，不触发机械臂动作。

## 7. 当前验证边界

- V1轮询已通过PC假数据和Nano/F7通道0实机通信；此前20 Hz轮询抓取存在显示负载和触发时序问题，因此不再作为轻量抓取正式路径。
- V2 START/READY/EVENT/ACK/STOP的Python/C编解码、黄金帧、CRC和主机测试已通过。
- V2 F7任务已接入`mult_uart_device_submit()`的WRITE_READ、READ和WRITE事务；
  低/高/中三层场景的C/Python协议测试及正式Keil链接已通过，Nano/F7分层场景切换实机尚未验证。
