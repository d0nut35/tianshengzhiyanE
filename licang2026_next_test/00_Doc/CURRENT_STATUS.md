# STM32当前状态

> 更新日期：2026-09-21。分支、提交和未提交文件以实际Git状态及外层`CURRENT_TASK.md`为准。

## 工程边界

- 当前STM32仓库：`D:\programfile\licang\licang2026_next_test`
- Keil工程目录：仓库下的`licang2026_next_test`
- 旧工程`D:\programfile\project\licang2026v1`只作历史参考。
- `05_Test/test_config.h`当前为`LICANG_TEST_NONE`。

## 当前实现

- `mission_app`负责Nano视觉、LSC16、IC、球档案、车载转盘和PB0。
- Mission通过`chassis_mission_link`与队友底盘通信，圆盘和阶梯流程已经存在。
- 已合入队友拆分后的`app_link/app_route/app_stairs`等底盘模块；整机行为仍需结合当前双方代码验证。
- STM32 `nano_vision` Core已包含圆盘、低/高/中阶梯、小圆盘球视觉场景，以及仓库数字场景和`DIGIT_EVENT`编解码。
- Mission和底盘已接入小圆盘到位、启动、暂停/恢复及结束回执；新增底盘代码标注`[lyx]`。阶梯结束执行18→10，小圆盘抓取执行20→19→读IC/转盘，第一球后重启场景6，第二球后继续剩余绕行，结束执行21→10→GO_DEPOT_1。
- 提交`406ac35`把原自动底盘路线测试扩展为独立USART1无线联调任务，正式`mission_task_entry()`保持不变。纯路径模式按`PLATFORM/STAIRS/DISC/DEPOT/D1~D4`逐段放行；单目标模式用`ROUTE PLATFORM|STAIRS|DISC RED|BLUE`自动跳过前置区域并只在目标区域运行完整视觉抓取，`ROUTE DEPOT`只到仓库1号位。跳过区域结束停留2秒。仓库内`HOME`先回1号位再回家，`HOME_DIRECT`从当前D1~D4位置直接执行已有回家路线，用于对比调参；两条回家指令在进入仓库前均拒绝执行。
- 无线任务复用现有`debug_uart1`（USART1 PA9/PA10，115200），支持`STATUS/BALLS/BALL n/STOP/HELP`；存球后自动输出球序号、区域、颜色、IC目标、槽位和状态。当前`MISSION_CHASSIS_ROUTE_TEST_ENABLED=1`，所以烧录后运行无线测试任务，不运行正式比赛Mission。
- 当前整体流程代码检查未发现结构性问题；圆盘视觉现场识别/触发抓取效果不稳定，下一阶段单独重调视觉参数和摄像头画面，不把该问题归因于F756芯片迁移。
- 底盘沿用累计运动约11秒的绕圈判定，暂停不计入预期运动时长；并非位置闭环整圈验证。仓库已支持`GO_DEPOT_1~4`逐点往返及到位回执，上层正式仓库数字映射、放球和倒垛闭环仍未实现。
- F750V8T6到F756VGT6的器件配置迁移和Keil构建已完成：`.ioc`、器件宏/SVD、启动文件及1 MiB Flash链接范围均已同步；原引脚、时钟、业务代码和优化等级保留。
- 本机已有STM32Cube F7 V1.17.4和Keil F7 DFP 3.1.1。用户截图确认CubeMX可打开并显示F756VGTx/LQFP100；未执行CubeMX重新生成代码。现有时钟代码仍为8 MHz HSE、216 MHz系统时钟及OverDrive。

## 当前边界

- 视觉Python端与STM32 Core的仓库数字协议已通过双方PC测试和共享黄金帧核对。
- 整机仓库视觉链路、Mission仓库状态和最终任务闭环尚未完成。
- 仓库数字协议尚未进行Nano/F7串口实机验证，不能把PC测试写成实机已通过。
- 小圆盘场景值`6`和`VISION_EVENT`已通过Python/C双方PC测试；Mission和底盘代码已接入，停车、夹取及Nano/F7整机尚未实测。
- Nano视觉、球档案、IC、转盘Host测试此前通过；旧mult_uart/lsc16测试入口引用已不存在的源码，全集未通过。此前Mission主机语法检查曾使用F767宏，不代表F750/F756正式编译通过。
- F756链接范围已设为`0x08000000`起始、`0x00100000`大小；VS Code索引已同步F756宏。本机下载算法为`STM32F7x_1024.FLM`，保存在被Git忽略的`.uvoptx`中，其他开发机需自行核对。
- 2026-09-19使用Keil ARMCLANG 6.24对当前F756配置全量重建：0错误、0警告，生成AXF/HEX；map确认Flash上限1 MiB、RAM上限320 KiB。ROM合计60588字节，RW+ZI合计118224字节；优化等级未调整。日志与map位于`MDK-ARM/f756-build.log`和`MDK-ARM/licang2026v1/licang2026v1.map`（均为本地构建产物）。尚未烧录或实机验证。
- 2026-09-21加入`HOME_DIRECT`后，无线测试开关开启时Keil ARMCLANG 6.24构建为0错误、0警告，`Code=66148`、`RO-data=2652`、`RW-data=20`、`ZI-data=118580`；此前临时关闭开关后的正式Mission构建也为0错误、0警告。`ball_manifest_core` Host测试通过。上述结果均未覆盖USART1无线模块、底盘、机械臂、Nano/F7或整机实测。
- 2026-09-21现场补充：用户确认F756已下载运行，机械臂动作组10执行，底盘能够运动，USART1无线命令已试用可行；具体命令清单和逐段偏差尚未记录。串口侧不接供电、改从烧录接口侧5V供电后恢复正常，电气根因未确认。当前无线测试开关已恢复为1；D2~D4直接回家、全路线停车精度、Nano/F7闭环及整机流程仍待验证。
- 2026-09-21按用户要求统一使用O3：Keil目标Optim=4，分组/文件Optim=0继承目标，移除六处文件级-Oz覆盖，保留其余编译参数。ARMCLANG 6.24全量重建为0错误、0警告；Code=93960、RO-data=2580、RW-data=24、ZI-data=118592，Total ROM=96564字节，LR_IROM1=96568字节（上限1 MiB）。无线测试开关保持1，正式Mission及底盘源码未改。此结果仅为编译验证，O3下USART1、Nano/F7、底盘及整机尚未重新实测，未证明速度提升。
- 旧文档中的Flash/RAM数字属于历史构建；当前容量必须重新构建并读取`.map`。
- 编译、PC测试、旧工程实机结果和当前工程整机实测必须分别记录。

## 下一步

1. 先用USART1无线任务实测纯路径模式，逐段标定转盘、阶梯、小圆盘和仓库路线；记录真实停点、速度、偏差和回家停车状态，不在未测前修改正式Mission。
2. 再配合Nano正式响应器的`--show-masks`和`ROUTE PLATFORM|STAIRS|DISC RED|BLUE`单目标模式，分别重调转盘、阶梯和小圆盘的HSV、ROI、面积与抓取参考点。
3. 同步完成F756实板下载、启动、216 MHz时钟、RTOS、USART1/UART7及DMA验证；O3固件仍须重新完成实板与运动回归验证。
4. 底盘和视觉调参稳定后，继续仓库数字映射、球放置和倒垛业务；保留当前目录和工程名，暂缓整体改名为`licang_E`。
