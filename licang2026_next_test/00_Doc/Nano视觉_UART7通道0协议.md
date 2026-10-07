# Nano视觉 UART7通道0协议

> 当前范围：球和C100仓库数字已接入Mission，用户曾反馈实机联调；MF500积木场景7、模型状态及积木终态已实现并通过主机验证。2026-10-07用户反馈首帧修复后三层无线测试完整跑通、无误识别或故障停车，完整积木抓放仍未接入。

## 1. 边界

- 物理链路：UART7复用通道0，`115200, 8-N-1`，无流控。
- F7拥有事务和复用板控制权；Nano不得操作复用板选择脚或直接控制其他通道。
- 旧V1轮询保留供固定假数据和回退测试；轻量抓取使用V2会话事件模式，等待目标期间不发送轮询。
- F7端所有通道0事务必须通过`mult_uart_device_submit()`提交，设备号0映射通道0。
- 整场任务需要在同一进程内切换圆盘和阶梯三层参数，Nano正式启动必须使用
  `--scene auto --mode auto`；固定场景启动只用于单场景标定和诊断。
- 小圆盘单独标定时可使用`--scene small_disc --mode auto`，但仍需F7发送场景`6`的会话命令才开始识别。
- 本协议只冻结当前最小球识别和仓库数字通信语义，不包含底盘、Mission仓库流程或完整比赛状态机。
- Nano只给出颜色和相对抓取中心的偏差；`BALL_ALIGNED`由F7按容差和连续帧数判定。

## 2. 通用帧

所有多字节整数均为小端。

| 偏移 | 长度 | 字段 | 说明 |
|---:|---:|---|---|
| 0 | 1 | SOF0 | `0xA5` |
| 1 | 1 | SOF1 | `0x5A` |
| 2 | 1 | VERSION | 当前`0x01` |
| 3 | 1 | TYPE | 见消息类型表 |
| 4 | 1 | SEQ | 命令响应回显F7序号；会话事件由Nano递增，按SID确认 |
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
| `0x05` | F7→Nano | MODEL_QUERY，查询积木模型状态 |
| `0x81` | Nano→F7 | 兼容观测OBSERVATION |
| `0x82` | Nano→F7 | SESSION_READY |
| `0x83` | Nano→F7 | VISION_EVENT |
| `0x84` | Nano→F7 | SESSION_STOPPED |
| `0x85` | Nano→F7 | DIGIT_EVENT |
| `0x86` | Nano→F7 | MODEL_STATE |
| `0x87` | Nano→F7 | BLOCK_RESULT |

## 3. F7轮询 `TYPE=0x01`

载荷固定2字节：

| 偏移 | 字段 | 值 |
|---:|---|---|
| 0 | SCENE | `1=BALL_TURNTABLE`，`2=BALL_STAIR_LOW`，`3=BALL_STAIR_HIGH`，`4=BALL_STAIR_MID`，`5=WAREHOUSE_DIGIT`，`6=BALL_SMALL_DISC` |
| 1 | TARGET_COLOR | `0=任意`，`1=红`，`2=蓝` |

仓库数字正式流程使用V2会话事件模式，不使用轮询观测结果。

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
球场景的`TARGET_COLOR`只能为红或蓝；仓库数字场景5和积木数字场景7必须为任意颜色`0`。积木只使用会话，不使用POLL。

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

### 5.3 仓库数字 DIGIT_EVENT

载荷固定8字节：

| 偏移 | 长度 | 字段 | 说明 |
|---:|---:|---|---|
| 0 | 2 | SESSION_ID | 非零会话号，小端 |
| 2 | 1 | DIGIT | 只允许`1`、`2`、`3` |
| 3 | 1 | QUALITY | `0~100` |
| 4 | 2 | FRAME_ID | Nano相机帧序号，小端，允许自然回绕 |
| 6 | 2 | AGE_MS | 识别结果年龄，小端 |

仓库数字事件使用现有`EVENT_ACK`确认，ACK中的`SESSION_ID`和`FRAME_ID`必须与事件一致。

### 5.4 EVENT_ACK

载荷固定4字节：`SESSION_ID(u16) + FRAME_ID(u16)`。F7只有在CRC、会话、结果取值、
结果年龄以及启动后新帧检查全部通过后才ACK；球事件还必须校验场景和颜色。
Nano收到匹配ACK后关闭本session。球流程中，F7在ACK发送完成后才允许触发动作组12，
保证Nano不会继续把同一球作为新事件发送。

### 5.5 SESSION_STOP / SESSION_STOPPED

载荷均为`SESSION_ID(u16)`。取消识别时F7发送STOP；Nano清除待发事件并回复STOPPED。
超时情况下F7允许清理本地session，但不得因此触发机械动作。

### 5.6 多球时序

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
5. 当前Core的PC测试参数仅为假数据示例：X容差10 px、Y容差8 px、结果年龄100 ms、连续3帧、连续3次超时离线。真实参数必须在机械安装后分别标定`BALL_TURNTABLE`、`BALL_STAIR_LOW`、`BALL_STAIR_HIGH`、`BALL_STAIR_MID`和`BALL_SMALL_DISC`。
6. F7链路超时或连续事务超时后标记Nano离线，不触发机械臂动作。

## 7. 积木场景7及起点模型握手

MODEL_QUERY载荷1字节：SCENE=7。MODEL_STATE载荷3字节：SCENE=7、STATE（0=LOADING、1=READY、2=ERROR）、REASON（正常0，模型失败10）。查询不打开相机或启动识别。

Nano程序启动异步加载并预热模型；F7起点每1秒查询，收到匹配序号的READY且动作10与底盘均就绪才放行。加载期间留在起点，不使用10秒限制模型加载。

BLOCK_RESULT载荷14字节：

| 偏移 | 长度 | 字段 |
| ---: | ---: | --- |
| 0 | 2 | SESSION_ID |
| 2 | 1 | STATUS：1=DIGIT、2=NO_VALID、3=FAULT |
| 3 | 1 | DIGIT：成功1/2/3，其余0 |
| 4 | 1 | QUALITY：0～100 |
| 5 | 1 | REASON |
| 6 | 2 | FRAME_ID |
| 8 | 2 | AGE_MS |
| 10 | 2 | FRAMES |
| 12 | 2 | ELAPSED_MS |

原因：1=CONFIRMED、2=NO_CANDIDATE、3=LOW_SCORE、4=AMBIGUOUS、5=UNSTABLE；故障6=CAMERA_NO_FRAME、7=FRAME_GAP、8=STALE_FRAME、9=INSUFFICIENT_FRAMES、10=MODEL_ERROR、11=MODE_NOT_READY、12=CAMERA_ERROR。

DIGIT要求分数≥80且连续3张新帧一致；NO_VALID要求digit=0、原因2～5、累计至少15张有效新帧且首张有效帧起至少1500ms。低分/无候选等统一跳站，保留原因，不能据此认定物理空仓。缺帧或模型异常不能当空仓。

F7只在动作完成、停车稳定200ms后的专属等待状态接受本SID结果，结果年龄≤120ms。START准备最长10秒，READY后8秒是通信保护；1500ms业务判定由Nano管理。旧C100仓库场景保留等READY后计时，不套用积木准备期限。

Nano准备相机与首帧在后台进行，串口线程可响应STOP。READY时清空旧计数，只累计此后新采样帧；同SID重复START不重置计数。每会话只锁存一个结果，50ms重发直至匹配SID/FRAME_ID的EVENT_ACK或1秒ACK期限。F7等ACK写完成才移动。FAULT允许在READY之前回报；动作异常和设备故障停车。

双方黄金帧：MODEL_QUERY `a55a01050701075ec1`；模型READY `a55a01860703070100c75a`；SID=0x1234、digit=2、quality=88、frame=513、age=24、frames=3、elapsed=192的结果 `a55a0187030e341201025801010218000300c000ec4d`。

## 8. 当前验证边界

- V1轮询已通过PC假数据和Nano/F7通道0实机通信；此前20 Hz轮询抓取存在显示负载和触发时序问题，因此不再作为轻量抓取正式路径。
- V2 START/READY/EVENT/ACK/STOP的Python/C编解码、黄金帧、CRC和主机测试已通过。
- V2 F7任务已接入`mult_uart_device_submit()`的WRITE_READ、READ和WRITE事务；
  低/高/中三层场景的C/Python协议测试及正式Keil链接已通过，Nano/F7分层场景切换实机尚未验证。
- 仓库数字场景、会话目标约束和`DIGIT_EVENT`的Python/C编解码、共享黄金帧、长度、取值及CRC错误测试已通过。
- 仓库数字及小圆盘已接入正式/无线Mission；用户报告此前联调，近期小圆盘基本夹到但停车偏早。不能用旧版PC测试描述覆盖现场反馈。
- 积木模型查询、三层无线走位、旧会话隔离及故障停车通过双方主机测试；Keil编译结果另见CURRENT_STATUS。用户反馈Nano首帧修复后ROUTE BLOCK DIGIT完整跑通、未观察到误识别或故障停车，记录为本轮Nano/F7实机联调通过；异常场景与多轮统计未单独覆盖，完整比赛积木抓放尚未接入。
