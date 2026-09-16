# STM32当前状态

> 更新日期：2026-09-16。分支、提交和未提交文件以实际Git状态及外层`CURRENT_TASK.md`为准。

## 工程边界

- 当前STM32仓库：`D:\programfile\licang\licang2026_next_test`
- Keil工程目录：仓库下的`licang2026_next_test`
- 旧工程`D:\programfile\project\licang2026v1`只作历史参考。
- `05_Test/test_config.h`当前为`LICANG_TEST_NONE`。

## 当前实现

- `mission_app`负责Nano视觉、LSC16、IC、球档案、车载转盘和PB0。
- Mission通过`chassis_mission_link`与队友底盘通信，圆盘和阶梯流程已经存在。
- 已合入队友拆分后的`app_link/app_route/app_stairs`等底盘模块；整机行为仍需结合当前双方代码验证。
- STM32 `nano_vision` Core当前只包含圆盘和低/高/中阶梯球视觉场景。

## 当前边界

- 视觉Python端已经实现仓库数字协议和C100场景；STM32端尚未同步仓库数字场景与结果。
- 整机仓库视觉链路、Mission仓库状态和最终任务闭环尚未完成。
- 旧文档中的Flash/RAM数字属于历史构建；当前容量必须重新构建并读取`.map`。
- 编译、PC测试、旧工程实机结果和当前工程整机实测必须分别记录。

## 下一步

1. 同步STM32仓库数字协议Core并运行协议测试。
2. 核对Mission与底盘仓库节点的命令和回复。
3. 方案确认后再接入正式仓库流程。
