# licang2026_next_test 任务进度

> 最近更新：2026-09-05
> 本文件用于快速恢复当前开发进度；详细背景、文档事实和历史结论见`00_Doc/任务衔接记录.md`。

## 0. 2026-09-05新底盘架构迁移快照

本节是当前最高优先级事实来源。当前唯一开发仓库为`D:\programfile\licang\licang2026_next_test`，工程目录为其下的`licang2026_next_test`；旧工程`D:\programfile\project\licang2026v1`只作历史参考，不再写入。

- 当前分支`feature/mission-app-state-machine`，最新本地提交`83ac4c2 完成Mission状态机并精简正式代码`，尚未推送。
- 工作树仍有用户和队友的未提交修改；禁止`reset`、`checkout`、`clean`、`pull`或覆盖现有改动。
- 新工程采用队友`lhy`底盘的Service/App/底层以及`chassis_bridge`，不再迁移旧`chassis_device`、`chassis_runtime`、`mission_route`和`07_Vendor`底盘副本。
- 我方`mission_app`独立任务已接入Nano协议、LSC16、IC、球档案、独立车载转盘和PB0。双方通过`chassis_mission_link.h`中的两个单向队列通信，消息只携带`request_id/type/is_ready`。
- Mission已实现动作组10初始化和READY握手、圆盘固定5球、第五球动作17、返回动作10后前往阶梯、低/高/中阶梯扫描、最多2球、停车确认后抓取、抓后返回13并恢复本层剩余横移。
- IC每球最多读取5次；失败写`READ_FAILED`后继续。PB0修正耗尽后记录并继续。底盘、ZDT通信/保护/超时、机械臂和视觉协议错误仍停机。
- 当前新架构完整Keil构建为`0 Error(s), 0 Warning(s)`。新Mission与底盘侧联动尚未完成，当前新工程未完成F7全流程实机验证；旧集成工程的实机结果不能直接算作新架构验证。
- 底盘侧仍需实现：阻塞读取`chassis_command_queue`、双方READY、`GO_PLATFORM/GO_STAIRS`路线及到位回复、每层就绪与`CAM_READY`放行、阶梯实际暂停/保存剩余行程/恢复、`STAIRS_FINISHED`和可及时处理的`STOP/STOPPED`。
- 仓库识别和倒垛尚未实现，当前Mission在阶梯结束后进入`COMPLETE`。
- Keil IROM上限`0x10000=65536 B`；当前map的Load Region为`0xFDB0=64944 B`，Flash只剩约`592 B`。RAM使用`120464/327680 B`，尚余约`207216 B`。圆盘五球已在镜像内；加入仓库倒垛前必须先释放Flash。
- 第一轮Flash减重应优先处理map中确实占ROM的未使用BLE/app_ble分支和正式运行不需要的RTT printf/日志，争取先腾出至少`6~10 KiB`。当前目标板测试对象未出现在最终map中，从Keil分组移除主要用于防止误启用，不应虚报为已释放Flash。必须保留底盘控制、DMA生命周期、UART句柄过滤和统一回调、ISR/任务边界、超时恢复、`request_id`、队列满判断及机械停止保护。
- 后续默认以完整Keil编译为主；只有平台无关Core或复杂协议规则变更才跑对应PC fake。编译通过仍不等于F7实机通过。

## 0.1 2026-08-31旧集成工程现场进度快照

本节覆盖下方尚未及时清理的旧“当前状态”描述，后续接手应以本节和最新Git提交为准。

### 已由用户实机确认

- LSC16动作组10为安全/初始姿态，11为转盘识别姿态，12完成抓球并放入车载存球转盘；动作组10、11、12均已完成现场确认。
- Nano与F7 UART7通道0已改为事件会话：F7只发送一次`SESSION_START`并挂接接收，Nano无目标时不发送，目标进入fast ROI后发送`VISION_EVENT`，F7以`SESSION_ID+FRAME_ID`确认后才触发动作12。
- CH340 USB串口链路在摄像头运行时稳定；Nano原生GPIO串口曾受摄像头负载影响，保留为对比路径，不作为当前正式链路。
- `WATCH_RED`的`START/READY/EVENT/ACK`无动作流程通过；单球事件抓取通过；已有轻量多球机械循环（动作12→动作11→新视觉会话）也已由用户确认成功。
- 视觉、机械臂、IC读卡、球档案、ZDT逐槽和PB0光电门组成的转盘区域5球完整闭环已经实机通过。前4球均完成抓取、返回动作组11、读卡建档、逐槽到位和下一视觉会话；第5球按设计在读卡建档后直接结束，不再空转一槽。
- 本轮5球均成功读取IC并写入档案，最终输出为`MULTI RECORD SEQ=4 CODE=0x21 ROW=2 COL=1 SLOT=4 COUNT=5/5`和`MULTI COMPLETE COUNT=5/5 RECORDS=5`，未出现档案丢失或越界覆盖。

### 当前软件实现和验证边界

- STM32提交`170f225`已实现视觉事件协议；视觉仓库提交`4fa8012`实现Nano事件上报和ACK重发。
- `turntable_grasp_lite_freertos_test`现已接入现有IC通道1、ZDT通道2、PB0光电门和`ball_manifest_core`，仍只使用一个`graspLite`应用任务；设备完成回调只复制结果并设置线程标志。
- 新多球顺序为：视觉事件→动作12→动作11确认→IC最多读取5次→只追加档案提交→ZDT粗调/微调到下一槽→PB0连续确认→新视觉会话。第五球读卡提交后直接完成，不再多转一槽。
- 球档案容量9，圆盘区域期望5球；记录包含颜色、IC编码、目标行列、物理槽位、状态和校验，追加失败不会覆盖既有记录。
- ZDT使用140.0度电机角粗调、1.0度同方向微调、最多20次、50ms状态查询、8s总超时、PB0连续3次高确认。运动前要求动作11已完成、0x1A OPTIONS确认闭环固件且PB0初始稳定为高；当前尚未加入反向回找。
- 自动测试和Keil完整构建均已通过，结果为`0 Error(s), 0 Warning(s)`；上述IC+转盘+视觉+机械臂5球组合流程也已完成真实F7实机验证。

### 下一次实机顺序

1. 保存下一轮各槽位的`SLOT OK ... FINE=x`，确认20次同方向微调具有足够余量且没有越过光电门目标；只有出现越过现象后才设计反向回找。
2. 清空物理存球转盘并在安全空闲时执行`BALL_RESET`，再用另一目标颜色重复5球闭环，核对颜色会话、IC档案和槽位序号。
3. 任一IC、动作11、ZDT状态或PB0失败都会进入`MULTI FAULT`且禁止自动继续；危险时必须使用硬件急停/断电，软件STOP不能替代硬件急停。

## 1. 当前阶段

F4参考工程中的UART串口复用框架已经迁移到STM32F750V8Tx正式工程；UART8 LSC16机械臂舵控、UART7直连IC读卡器和独立上层转盘ZDT_X42S也已经按同一套Core/HAL adapter/Service/Device架构完成F7封装。

四套模块的软件和工程接入均已完成。2026-08-13已把平台无关Core从BSP抽离到`03_Middleware`，并把PC测试统一归档到`05_Test/host`。随后完成了自编代码的中文Doxygen注释规范化：函数按需要补齐`@brief`、`@param`、`@return`、`@note`和`@warning`，重点说明DMA缓冲区生命周期、ISR限制、异步完成语义、异常恢复和测试安全条件。2026-08-14又按ZDT_X42S第二代手册和Emm实机返回修正了ZDT协议解析、角度换算及测试输出，并补齐相关中文Doxygen注释。

原三模块注释补齐后重新运行全部PC fake测试，并重新编译默认关闭、三模块裸机/FreeRTOS共七种Keil配置；全部通过，结果均为`0 Error(s), 0 Warning(s)`。ZDT Core、Service、Device三套PC fake测试随后也全部通过，ZDT裸机和FreeRTOS两种Keil配置均为`0 Error(s), 0 Warning(s)`。

**当前实机边界：PD9/PD10/PD11下UART7复用板FreeRTOS四通道流程已通过；通道1 IC卡已完成真实比赛球A3读取与行列解析；通道2 ZDT已完成VERSION/OPTIONS/STATUS/POSITION、CW/CCW运动、位置变化、PB0光电门和双向自动逐槽实测。有螺丝的槽位均能闭环到PB0=1，未安装螺丝的位置会在10次微调后返回`GATE_NOT_FOUND`，没有误报到位。实际`LICANG_ACTIVE_TEST`现已切换为`LICANG_TEST_LSC16_FREERTOS`；动作组配置为200、重复1次，但`LSC16_TEST_MOTION_ARMED=0`，不会下发运动帧。LSC16仍未实机验证。Nano上两路USB摄像头直连、MJPG 640×480@30、V4L2双路并行和OpenCV四缓冲采集已经实机通过；ToDesk环境下双路采集叠加MF500 HSV达到既定20 FPS门槛并观察到真实球，但识别仍会间歇丢失，机械臂最终安装后的曝光/光照/HSV标定和脱离ToDesk性能验收尚未完成。暂定让MF500远端利用积木表面黑色字符/轮廓定位带数字白色积木的候选货位，远端不强求稳定分类1/2/3，靠近抓取时再识别并核查积木数字；C100只负责下视仓库底部数字牌，不承担远端积木数字。上述两类视觉任务、UART7通道0协议及正式任务流程仍未实现。**

## 2. UART7串口复用模块

### 已完成

- [x] 迁移平台无关`mult_uart` Core。
- [x] 完成STM32F7 HAL adapter；当前CubeMX标签绑定为UART7、DMA1和PD9/PD10/PD11（M_A/M_B/M_EN）。
- [x] 迁移平台无关Service、CMSIS-RTOS2 worker adapter和Device事务层。
- [x] 建立`04_Bsp/uart_dispatch`，统一拥有HAL UART全局回调；UART7事件由句柄过滤handler认领。
- [x] 修复OS队列只保存TX指针可能造成悬空引用的问题，队列消息改为自己保存TX数据。
- [x] 建立`05_Test/test_config.h`单点互斥开关，默认`LICANG_TEST_NONE`。
- [x] 实现四通道CH340裸机和FreeRTOS测试程序：上电默认通道0，`FF 00~03 AA`切换，切换后立即提示，每3秒提示当前通道，普通数据按大写HEX纯文本回显。
- [x] PC fake测试、协议测试和F7三种测试分支的源码编译检查通过。

### 实机验证状态

- [x] 修正公共端接线后，四通道CH340裸机流程通过。
- [x] 同一接线下，FreeRTOS完整四通道流程通过。
- [x] 实测成功接线按复用板丝印视角为：`PE8/UART7_TX -> 公共TXD`，`PE7/UART7_RX <- 公共RXD`；不能再按普通外设TX/RX交叉规则猜测公共端丝印含义。
- [x] 四通道选择、提示转移、普通数据回显和基本通道隔离已由用户实机确认。
- [x] 2026-08-17选择线从PE9/PE10/PE11迁移到PD9/PD10/PD11后，FreeRTOS四通道提示、切换和通信功能流程已重新实机通过。
- [ ] PD新引脚下的裸机回归，以及PD11 EN/INH准确有效电平、上电/烧录瞬态、break-before-switch波形和5us稳定时间仍未定量确认。
- [x] 实板外部晶振确认是8 MHz；CubeMX和生成代码已从错误的25 MHz修正为8 MHz，系统时钟仍为216 MHz，UART7恢复为真实115200。修正前的固定乱码是时钟基准错误，不是复用协议错误。
- [ ] UART7当前为115200；不同通道的最终波特率策略尚未通过真实模块闭环。

## 3. UART8 LSC16机械臂舵控

### 已完成

- [x] `03_Middleware/lsc16`：平台无关LSC16协议Core；`04_Bsp/lsc16`只保留STM32F7 UART8 HAL adapter。
- [x] `02_Service/lsc16`：平台无关串行Service和CMSIS-RTOS2 worker adapter。
- [x] `01_App/lsc16_device`：单舵机、多舵机、动作组、停止、调速、电池查询等语义接口。
- [x] UART8绑定PE0 RX、PE1 TX、9600 8N1；TX DMA1 Stream0、RX DMA1 Stream6；ReceiveToIdle使用UART8全局中断。
- [x] 用户已在CubeMX启用UART8全局中断并重新生成；已核对`.ioc`、`usart.c`和`stm32f7xx_it.c/.h`一致，UART8 IRQ优先级为5。
- [x] LSC16通过公共`uart_dispatch`注册UART8句柄过滤handler，不重复定义HAL全局回调。
- [x] `05_Test/lsc16`已实现独立裸机和FreeRTOS测试；两者上电后只等待USART1文本命令，不自动运动。电脑可发送`SERVO/ACTION/STOP/BATTERY/STATUS`并直接观察文本结果，无需使用Keil Watch。
- [x] LSC16接线、命令、预期回报和安全顺序已写入`00_Doc/LSC16_USART1命令测试说明.md`。
- [x] 动作组测试区分DMA发送完成、控制板`0x06`开始回报和`0x08`自然结束回报；协议文档没有规定`0x03`单舵机命令的成功应答。
- [x] LSC16 Core和Service的PC fake测试通过，ArmClang源码检查通过。
- [x] CubeMX重新生成后的F7 Keil完整构建通过：`0 Error(s), 0 Warning(s)`，已生成AXF/HEX。

### 安全配置与当前状态

- [x] 当前`LICANG_ACTIVE_TEST=LICANG_TEST_LSC16_FREERTOS`，进入FreeRTOS实机测试准备状态；用户决定优先测FreeRTOS，出现问题时再用裸机隔离。
- [x] 当前`LSC16_TEST_MOTION_ARMED=0`，不会下发`SERVO/ACTION`运动帧；动作组配置为用户指定的200号安全测试动作、重复1次，但实机运行前仍须确认该动作组已下载且机械范围安全。

### 尚未实机验证

- [ ] 没有连接LSC16舵控板，没有下载运行裸机测试。
- [ ] 没有下载运行FreeRTOS测试。
- [ ] 单舵机ID、位置、时间和安全方向尚未标定。
- [ ] 动作组编号和实际动作尚未标定。
- [ ] 没有在真实控制板上确认动作组`0x06`开始、`0x08`完成回传。
- [ ] 单舵机命令只能在实机中通过实际运动确认；不能把DMA发送完成当作舵机执行成功。

## 4. IC读卡器（UART7直连封装与复用通道1实测）

### 已完成

- [x] `03_Middleware/ic_card`：M2940B-HA平台无关协议Core；`04_Bsp/ic_card`只保留UART7 HAL adapter。
- [x] Core已实现A3 Key A读块、B0/B1/B2/B8查询、异或取反校验和无固定帧头的流式拆包；2026球值解释已从协议Core拆到`01_App/ic_card_device/ic_ball_rule_2026.*`。
- [x] `02_Service/ic_card`：平台无关单事务Service和CMSIS-RTOS2队列/worker，包含响应匹配、500ms超时及UART恢复。
- [x] `01_App/ic_card_device`：提供“读取比赛球”语义接口，返回原始16字节、编码、行和列。
- [x] `05_Test/common`：测试专用USART1 PA9/PA10调试端口，RX使用ReceiveToIdle DMA，结果以纯文本发送。
- [x] `05_Test/ic_card`：已实现UART7直连裸机测试和FreeRTOS完整分层测试。电脑通过USART1发送`BALL_READY`或`READ`后才发起一次A3读取。
- [x] `05_Test/host`：复用、LSC16、IC Core/Service及2026球号规则的PC fake测试全部通过；这些文件不加入Keil目标，不占用单片机Flash/RAM。
- [x] 用户已完成CubeMX配置并由工程核对：UART7为PE7 RX/PE8 TX、115200、RX/TX DMA及UART7 IRQ；USART1为PA10 RX/PA9 TX、115200、RX DMA2 Stream2及USART1 IRQ；PB1为`IC_CARD_IO`输入。
- [x] 修复ArmClang 6下CubeMX旧FreeRTOS模板缺少`SystemCoreClock`声明的问题。
- [x] 默认关闭、IC裸机和IC FreeRTOS三种Keil完整构建均为0 Error、0 Warning；复用通道1实机已通过，当前现场已转入LSC16 FreeRTOS测试准备。
- [x] 实机步骤和推荐参数写入`00_Doc/IC卡_UART7直连测试说明.md`。

### 当前比赛建议

- 使用命令模式：球放稳后F7发送一次A3，不使用自动上传决定转盘时序。
- 当前UART7和已知实物配置均为115200、8N1，设备地址使用0x20，读取0扇区块1。
- 2026初步规则要求16字节一致，高半字节为1~3行、低半字节为1~4列；二维码已取消，`0x14`现在解析为第1行第4列，不再沿用2025“干扰球”含义。
- Key A必须用正式比赛样球确认，不能把`FFFFFFFFFFFF`写成已验证事实。
- PB1的IO提示只表示刷卡活动，不携带球值，首轮读球流程不以PB1作为有效数据依据。

### 实机验证与剩余边界

- [x] UART7复用通道1事务适配已经实现，并通过软件、编译和真实F7读球验证。
- [x] 复用通道1执行`IC_READ`已成功读取`0x11`、`0x12`等16字节一致的比赛球数据并解析行列；供电不稳时出现过`IC READ ERROR 10`，供电恢复后可继续成功读取。
- [ ] UART7直连IC裸机和FreeRTOS独立测试未做；当前复用链路已满足主要功能，后续仅在集成异常需要隔离问题时作为诊断实验。
- [ ] B0/B1/B2/B8参数查询尚未全部形成稳定实机记录；Key A、读卡距离、姿态容差、无球超时恢复和相邻球干扰仍需在最终机械结构中验收。
- [ ] 没有实测PB1提示电平的有效极性和持续时间；当前比赛流程不依赖PB1判断球值。

## 5. ZDT_X42S转盘电机（UART7直连与复用通道2）

### 已完成

- [x] `03_Middleware/zdt_turntable`实现X/Emm协议Core；`04_Bsp/zdt_turntable`只保留UART7直连HAL adapter。
- [x] `02_Service/zdt_turntable`实现单事务、超时和串行响应交付；`01_App/zdt_turntable_device`提供版本、配置、状态、位置、停止及固定角度运动接口。
- [x] `05_Test/zdt_turntable`提供USART1文本命令驱动的裸机和FreeRTOS测试；HAL UART全局回调仍由`04_Bsp/uart_dispatch`唯一拥有。
- [x] 根据第二代手册修正OPTIONS为5字节16位标志解析，完整解析MotType、FwType、CtrMode、MotDir、BtLock、Scale和参数锁等级。
- [x] STATUS优先识别`E2/EE`错误帧，补充左右限位、掉电锁存字段；查询不再误收动作ACK；绝对零位置和零脉冲合法。
- [x] 正确区分X与Emm的角度/速度Scale，Emm角度转脉冲增加安全四舍五入和溢出检查。
- [x] ZDT Core、Service、Device PC fake测试全部通过；ZDT裸机和FreeRTOS Keil构建均为`0 Error(s), 0 Warning(s)`。

### 已完成的实机只读验证

- [x] UART7直接连接ZDT，不经过复用板；VERSION返回`FW=201 HW_SERIES=2 HW_TYPE=3 HW_VER=20`。
- [x] OPTIONS原始响应为`01 1A 00 06 6B`，确认当前为1.8度电机、Emm固件、闭环、CW正方向、Scale关闭。
- [x] STATUS返回`ENABLE=1 REACHED=1 STALL=0 PROTECT=0 POWERLOSS=0`；POSITION返回`SIGN=+ RAW=3`。
- [x] 小屏幕第二行显示`0.00err`，结合手册确认是Emm固件显示，不是当前错误码。

### 当前现场、实机验证与剩余边界

- [x] 复用模块FreeRTOS集成测试阶段曾选中专用ZDT运动锁1并完成实测；当前活动开关已切换为`LICANG_TEST_LSC16_FREERTOS`，所以当前烧录目标不应响应ZDT集成测试命令。
- [x] UART7复用通道2完成VERSION/OPTIONS/STATUS/POSITION真实通信；确认`MStep=16`、`Response=Receive`、Emm闭环、Scale关闭。
- [x] CW/CCW位置运动、`REACHED=1`和POSITION连续变化已实机确认；动作ACK仍只解释为“已接收”，状态机继续主动查询STATUS。
- [x] PB0光电门实测“螺丝对准=1、未对准=0”；已接入双向自动逐槽状态机。
- [x] `ZDT_SLOT_CW`一圈有螺丝槽位微调次数为`5、0、0、2、2、10、10、6、0`；缺螺丝位置正确返回`GATE_NOT_FOUND`。
- [x] `ZDT_SLOT_CCW`一圈有螺丝槽位微调次数为`0、0、9、0、10、0、5、0、0`；缺螺丝位置同样正确返回`GATE_NOT_FOUND`。
- [ ] 最后一处螺丝尚未安装；装齐后需要再做CW/CCW完整一圈最终验收。部分槽位恰好需要10次微调，当前搜索余量较小，后续根据装配结果决定是否调整正反向粗角度或最大次数。
- [ ] `ZDT_STOP`效果尚未单独实机验收；STOP经过软件FIFO，不是独立硬件急停，危险时仍须切断电机动力电源。
- [ ] `Response=Both`仍不支持；保持`Response=Receive`。动态多波特率切换尚未实现，但当前IC和ZDT均为115200，不影响本轮两设备集成。

## 6. 公共工程状态

- [x] `main.c`只保留装配、初始化和极薄测试入口。
- [x] 所有目标板测试集中于`05_Test`，通过`05_Test/test_config.h`互斥选择；正式发布默认值应为全关，当前临时选中LSC16 FreeRTOS测试。
- [x] 平台无关Core统一位于`03_Middleware/<module>`；`04_Bsp`只保留硬件适配、板级策略和UART回调路由。
- [x] UART7复用模块和UART8 LSC16不重复定义HAL UART全局回调。
- [x] Keil工程已加入四套模块、Service、Device、公共回调路由和测试文件。
- [x] 正式重要说明放在`00_Doc`。
- [x] `PB1`已由用户在CubeMX配置为`IC_CARD_IO`输入并重新生成，工程核对一致；电平实际行为仍待实物测试。
- [x] `01_App`、`02_Service`、`03_Middleware`、`04_Bsp`和目标板`05_Test`中的自编接口与函数已完成中文Doxygen注释规范化；未修改CubeMX/HAL/FreeRTOS第三方生成代码。
- [x] 原三模块注释规范化后重新执行全部PC fake测试和七种Keil互斥配置，全部通过；ZDT三套PC fake与两种Keil配置也全部通过。
- [x] IC卡通道1和ZDT通道2的复用transport及USART1命令式FreeRTOS集成测试已经实现，并通过既有PC fake回归、Keil编译和真实F7验证。
- [x] PB0板级读取及ZDT双向自动逐槽测试状态机已经实现；Keil双向版本为`0 Error(s), 0 Warning(s)`，有螺丝/缺螺丝两类真实现象均符合设计。
- [x] 视觉工程路径、Windows/Nano软件环境、MF500/C100双摄像头责任和VS Code运行配置已记录到`D:\programfile\opencv\coler_redblue\docs\HANDOFF.md`。
- [x] Nano双USB摄像头直连实测：两路V4L2 MJPG 640×480@30可同时稳定采集；OpenCV使用4个缓冲区时两路约29～30 FPS、失败读取为0。
- [ ] MF500已在Nano观察到真实球识别，但存在间歇丢失；最终安装后的相机参数/HSV标定、无ToDesk性能、MF500远端数字积木候选货位定位和近端数字核查、C100下视仓库底部数字牌、视觉/Nano协议、LSC16实机和正式比赛任务流程尚未完成。

## 7. 已确定的UART7通道责任

- 通道0：预留视觉/Jetson Nano；Windows/Nano视觉工程和双摄像头已经确认，具体UART通信契约仍待设计。
- 通道1：IC读卡器。
- 通道2：独立上层转盘ZDT_X42S，由本F7工程负责。
- 通道3：原XR1503MEX-V1二维码模块通道；2026初步规则已取消二维码，是否保留该接口待后续决定。

## 8. 待做实验总清单

下列项目同时记录已完成和待完成的真实实验。不能因为fake测试或Keil编译通过而勾选实机项。当前活动场景为LSC16 FreeRTOS测试，动作组200、重复1次、运动锁0；烧录后只等待USART1命令，`SERVO/ACTION`当前会被安全锁拒绝。

### E0：所有上电实验的共同检查

- [ ] 根据模块实物丝印和资料确认供电电压、TTL电平并可靠共地。
- [ ] 按模块手册和实物丝印确认TX/RX方向，首次上电前检查是否短路或误接电源；普通独立设备通常交叉连接，但复用板公共TXD/RXD按本轮实测接线，不能套用通用规则。
- [ ] 核对当前下载配置对应的测试开关，禁止两个模块同时占用UART7/DMA/公共回调。
- [ ] 保存每次实验的接线、波特率、测试开关、原始TX/RX和异常现象；若实测推翻文档假设，立即修正文档和板级配置。

### E1：M2940B-HA IC卡UART7直连实验

- [ ] 用厂家上位机或已核对的B0/B1/B2/B8查询命令确认当前地址、工作模式、蜂鸣器和自动读取设置；保留原始响应。
- [ ] 用正式样球确认读卡器内部Key A能够读取0扇区块1，禁止向比赛球写数据或执行A5。
- [ ] 按`00_Doc/IC卡_UART7直连测试说明.md`连接UART7、USART1调试口和可选PB1。
- [ ] 启用`LICANG_TEST_IC_CARD_BAREMETAL`：发送`BALL_READY`，验证一次A3请求、完整响应、16字节一致性以及行列解析。
- [ ] 重复测试无球、错误Key、快速重复触发、超时恢复和再次成功读取。
- [ ] 测试不同球姿态、距离、高度、静止/轻微运动以及相邻球干扰，确定可靠机械放置窗口。
- [ ] 使用逻辑分析仪、示波器或GPIO时间戳测量PB1刷卡提示的有效极性与持续时间；PB1不作为球值有效依据。
- [ ] 启用`LICANG_TEST_IC_CARD_FREERTOS`，重复成功、超时和恢复实验，确认唯一worker和回调路径正常。
- [ ] 实验结束恢复`LICANG_TEST_NONE`并记录真实波特率、地址、Key A验证结论和原始帧。

### E2：UART7四通道复用板CH340实验

- [x] 四个通道只连接3.3V CH340，不连接IC、ZDT、Nano或其他正式模块。
- [x] 启用`LICANG_TEST_MULT_UART_BAREMETAL`，确认上电通道0提示、四通道切换、普通数据回显和非法通道处理正常。
- [x] 修正公共端接线为`PE8/UART7_TX -> 公共TXD`、`PE7/UART7_RX <- 公共RXD`后，裸机四通道流程通过。
- [x] 启用`LICANG_TEST_MULT_UART_FREERTOS`并重复完整四通道流程，FreeRTOS路径通过。
- [x] 用户观察到选择通道的提示正确转移，基本通道隔离、A/B通道编号和数据方向符合当前实现。
- [x] 在当前PD9/PD10/PD11接线上重新执行FreeRTOS四通道功能回归，用户确认复用模块工作正常。
- [ ] 在当前PD9/PD10/PD11接线上补做裸机四通道回归。
- [ ] 实测PD11 EN/INH有效电平、break-before-switch效果和切换稳定时间；据结果修正`mult_uart_board_config.h`，不能继续把F4参考值当成已验证事实。
- [ ] 用示波器/逻辑分析仪补测PD11上电及烧录瞬态、A/B/EN波形、break-before-switch和5us稳定时间。
- [x] 曾切换为`LICANG_TEST_MULT_UART_MODULES_FREERTOS`并完成通道1 IC、通道2 ZDT和PB0集成实测；当前已切换到LSC16 FreeRTOS场景。

### E3：UART8 LSC16机械臂舵控实验

- [ ] 先确认舵控板独立供电、F7与舵控板共地、UART8 TX/RX交叉连接和9600 8N1。
- [ ] 按`00_Doc/LSC16_USART1命令测试说明.md`连接USART1调试口；上电后由电脑发送`STATUS/BATTERY/SERVO/ACTION/STOP`，不使用Keil Watch判断结果。
- [ ] 机械臂卸载、断开危险连杆或置于不会碰撞的安全姿态，确认急停/断电方式。
- [x] 当前已配置`LICANG_TEST_LSC16_FREERTOS`、动作组200、重复1次并保持`LSC16_TEST_MOTION_ARMED=0`；这只是配置确认，不是硬件验证。
- [ ] 保持运动锁0，先用`STATUS/BATTERY`确认调试口、任务和舵控板基本通信。
- [ ] 人工确认200号动作组已经下载到当前舵控板、起始姿态安全且不会碰撞后，再显式置`LSC16_TEST_MOTION_ARMED=1`并重新编译烧录。
- [ ] 发送`ACTION`验证200号动作组，分别观察UART TX完成、实际机械运动以及控制板`0x06`开始/`0x08`完成回报。
- [ ] 验证错误或中止后的恢复；不得把DMA TX完成当成舵机已经到位。
- [ ] 若FreeRTOS路径异常，再切换`LICANG_TEST_LSC16_BAREMETAL`隔离协议、UART/DMA与RTOS队列问题；裸机不再作为本轮强制前置。
- [ ] 完成机械臂各关节安全范围、零位/参考位和动作组编号标定，再逐步组合夹取动作。
- [ ] 实验结束把`LSC16_TEST_MOTION_ARMED`恢复0，并把`LICANG_ACTIVE_TEST`恢复`LICANG_TEST_NONE`。

### E4：ZDT直连运动与后续复用集成实验

- [x] 已为IC协议增加UART7复用通道1 transport，复用既有Core/Device/规则解析，没有复制第二套IC协议。
- [x] 通道1“选通→A3→响应→释放”已在真实F7读取多颗比赛球；仍需补最终机械结构下的无球超时、供电稳定性、姿态/距离和连续球流程验收。
- [x] 阅读第二代ZDT_X42S手册并完成独立上层转盘的Core/Service/Device、UART7直连裸机/FreeRTOS测试和协议注释；没有修改队友负责的底盘ZDT工程。
- [x] 不经过复用板完成ZDT UART7直连VERSION/OPTIONS/STATUS/POSITION只读通信，确认实物为Emm固件。
- [x] 保持`Response=Receive`，已完成CW/CCW运动、STATUS/POSITION到位核对；STOP仍待单独测试。
- [x] 已为ZDT增加UART7复用通道2 transport并复用原Core/Device；真实复用板查询与运动通信通过。
- [x] PB0原始电平、140度粗定位、1度微调、CW/CCW自动逐槽和缺螺丝失败保护已实机通过。
- [x] 双摄像头责任已澄清：爪上MF500负责近端红蓝球；倒垛时暂定在远端利用积木表面黑色字符/轮廓定位3个带数字白色积木的候选货位，不强求远端稳定分类1/2/3，靠近抓取时再识别并核查积木数字。下方C100只负责识别仓库底部数字牌以建立列定义，不承担远端视觉或积木数字。
- [x] Nano双路采集和V4L2/OpenCV四缓冲链路已完成基础实机验证；设备号会随插拔变化，最近一次现场映射为MF500=`/dev/video0`、C100=`/dev/video2`，不能写死为永久编号。
- [ ] 先验证MF500能否在远端稳定找到白色积木候选位置，并在靠近抓取距离后稳定读出积木数字；同时用C100完成仓库底部数字牌识别，每次输出通过多帧一致性确认即可，不额外设置远端核查阶段；然后再冻结UART7通道0消息字段、超时/重试及F7任务语义。
- [x] PB0已完成GPIO原始电平和三次采样确认实测并接入转盘状态机；当前未使用EXTI，后续若任务时序需要再评估中断方案。
- [ ] 通道0/1/2设备波特率若不同，先设计并验证“总线空闲、EN禁用、UART重配、清错误、选通、重新接收”的安全切换流程；不能在DMA活动期间修改波特率。
- [ ] 2026初步规则已取消二维码，通道3/XR1503MEX-V1暂不作为优先实验；只有规则或用户需求恢复时再继续。

### 推荐执行顺序

```text
E0共同检查
→ E2复用板CH340裸机（已通过）
→ E2复用板CH340 FreeRTOS（已通过）
→ E4 ZDT UART7直连只读（已通过）
→ E4 IC卡通道1、ZDT通道2、PB0及双向逐槽（基础实机已通过）
→ Nano双摄像头V4L2/OpenCV四缓冲采集（基础实机已通过）
→ 安装机械臂后固定MF500机位/光照并完成红蓝球稳定性标定；脱离ToDesk补最终性能验收
→ 实测MF500远端利用黑色字符/轮廓定位带数字白色积木候选货位，再验证近端积木数字识别；近端数字与近端红蓝球仍需验证同一手动焦点能否兼顾
→ 采集C100仓库底部数字牌样本，验证单次对齐后的多帧一致性识别
→ 设计Nano↔F7通道0协议及PC/USB-TTL独立测试，再按mult_uart分层接入F7
→ E3 LSC16动作组200 FreeRTOS实机标定（异常时再退回裸机）
→ 安装最后一处螺丝，补CW/CCW整圈和ZDT_STOP最终验收
→ 把视觉、IC读球、转盘槽位和机械臂动作组接入正式任务流程
→ E1 IC卡UART7直连仅在复用读卡异常时用于诊断隔离

E3 LSC16可在机械安全条件具备后独立进行，不依赖UART7复用板；当前已选择FreeRTOS场景但运动锁仍为0。
光电门基础闭环已完成；Nano双路采集基础实机已通过，通道0通信契约仍待C100输出语义明确后设计。
```

当前视觉主线不再是迁移Nano，而是先验证MF500远端利用黑色字符/轮廓定位带数字白色积木候选货位、近端积木数字识别以及近端红蓝球是否能共用当前手动焦点，再完成C100下视仓库底部数字牌的输出定义，随后设计Nano↔F7通道0通信契约；MF500最终HSV标定等机械臂安装和机位/光照固定后完成。LSC16可按“FreeRTOS、动作组200、运动锁0起步”的安全顺序独立测试；它仍未完成真实F7硬件验证。转盘最后一处螺丝、双向最终整圈和STOP仍是机械收尾项。

## 9. CubeMX协作约定

后续涉及UART、DMA、NVIC、GPIO等CubeMX配置时，AI先明确告诉用户需要配置的项目、参数和原因；由用户在CubeMX中配置并生成代码，AI随后检查`.ioc`、生成源码、Keil工程接入和构建结果。未经用户明确要求，AI不直接改`.ioc`或CubeMX自动生成的外设配置。

## 10. 更新约定

不在每个小任务结束后自动写流水账。一轮工作结束时AI先询问是否需要记录；用户明确要求“更新进度”时再更新。若新测试推翻旧结论，应直接修正旧结论或标记其为历史状态，不能让相互冲突的信息同时作为当前结论。

## 11. 2026-08-26 Nano视觉通道0固定假数据实机记录

- [x] Nano与F7采用UART7复用通道0、115200 8N1完成固定假数据实机通信；F7主动轮询，Nano响应带SEQ和CRC16的观测帧。
- [x] `BALL_TURNTABLE`、`BALL_STAIR`两种场景的红/蓝四种组合均连续3帧通过，第三帧达到`ALIGNED=1`。
- [x] 分包、粘包、CRC错误识别和响应丢弃超时已经实测；状态行中的`TIMEOUT`是累计诊断计数，离线判定使用独立的连续超时计数，当前配置为连续3次超时离线。
- [x] 通道0视觉、通道1 IC读卡和通道2 ZDT只读查询完成无动作交叉测试；`OS_SUB=OS_DEQ`、`SVC_SUB=SVC_DONE`、`PENDING=0`且`UART_ERR=0`。无球时出现的`IC READ ERROR 10`为`IC_CARD_ERR_CARD`，放入比赛球后恢复连续读取成功。
- [x] 现场曾因杜邦线/排针接触导致Nano能够收到轮询但F7无法正确接收响应；更换杜邦线后通信恢复。该现象不能归因于协议编码或Nano响应速度。
- [ ] Nano离线后恢复正常响应并重新置`ONLINE=1`需要补一条明确实机记录。
- [ ] 当前只验证固定假数据响应器，尚未把MF500真实识球结果接入串口响应。

### Nano排针异常时的CH340 USB串口备用方案

若Nano J41 UART排针或杜邦线接触不可靠，可把已验证的3.3V CH340插入Nano USB口，使用`/dev/ttyUSB*`绕过J41：

```text
CH340 TXD -> 复用板 RX0
CH340 RXD <- 复用板 TX0
CH340 GND -> 复用板/F7公共GND
CH340 VCC/5V/3.3V -> 不连接
```

启动前用`python3 -m serial.tools.list_ports -v`或`ls -l /dev/serial/by-id/`确认实际设备路径；优先使用`/dev/serial/by-id/...`稳定名称，不能永久写死`/dev/ttyUSB0`。使用前确认CH340 TXD空闲电压约为3.3V，且同一串口只允许一个进程占用。
