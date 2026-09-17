# STM32当前状态

> 更新日期：2026-09-18。分支、提交和未提交文件以实际Git状态及外层`CURRENT_TASK.md`为准。

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

## 当前边界

- 视觉Python端与STM32 Core的仓库数字协议已通过双方PC测试和共享黄金帧核对。
- 整机仓库视觉链路、Mission仓库状态和最终任务闭环尚未完成。
- 仓库数字协议尚未进行Nano/F7串口实机验证，不能把PC测试写成实机已通过。
- 小圆盘场景值`6`和`VISION_EVENT`已通过Python/C双方PC测试；Mission、停车、夹取和Nano/F7实机尚未接入或验证。
- 旧文档中的Flash/RAM数字属于历史构建；当前容量必须重新构建并读取`.map`。
- 编译、PC测试、旧工程实机结果和当前工程整机实测必须分别记录。

## 下一步

1. 用固定`small_disc`场景完成Nano/F7小圆盘通信和ROI参数标定。
2. 核对协作者的小圆盘底盘绕行接口，再设计停车与夹取接入。
3. 后续正式流程联调时验证仓库数字协议和业务流程。
