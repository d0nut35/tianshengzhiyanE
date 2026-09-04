# licang2026_next_test 项目协作约定

开始处理本工程任务前，必须先完整阅读：

1. `D:\programfile\licang\licang2026_next_test\licang2026_next_test\00_Doc\TASK_PROGRESS.md`
2. `D:\programfile\licang\licang2026_next_test\licang2026_next_test\00_Doc\任务衔接记录.md`
3. `D:\programfile\licang\licang2026_next_test\licang2026_next_test\00_Doc\新底盘架构上层任务移植方案.md`
4. `D:\programfile\licang\licang2026_next_test\licang2026_next_test\00_Doc\Nano视觉_UART7通道0协议.md`
5. `D:\programfile\licang\licang2026_next_test\licang2026_next_test\00_Doc\mult_uart_F7移植说明.md`
6. `D:\programfile\project\zhuaqu_test1\docs\2025赛题目标与模块清单.md`
7. `D:\programfile\project\zhuaqu_test1\docs\立体仓库控制架构设计.md`
8. `D:\programfile\project\zhuaqu_test1\docs\F7真实引脚与外设分配.md`

当前唯一STM32开发仓库为`D:\programfile\licang\licang2026_next_test`。`D:\programfile\project\licang2026v1`只作历史参考，未经用户明确要求不得修改。开始时先检查当前Git状态；不得使用`reset`、`checkout`、`clean`或`pull`覆盖未提交修改。

维护要求：

1. 正式代码按 App/Device、Service、`03_Middleware`平台无关Core、`04_Bsp` STM32 HAL adapter、CubeMX/HAL分层；不要把协议和测试堆进`main.c`。
2. 所有HAL UART全局回调只允许由`04_Bsp/uart_dispatch`拥有；模块只能注册先过滤UART句柄的handler。
3. 新设备协议只能通过`mult_uart_device_submit()`提交事务，不能直接操作UART7、DMA或A/B/EN。
4. 测试需求由用户后续补充；在此之前不要自行移植F4测试夹具或启用demo。
5. 明确区分“文档确认、工程实现、编译验证、实机验证、暂定配置”，不得把PC编译或fake测试写成F7硬件通过。
6. CubeMX配置变化必须同步`.ioc`和生成代码；修改后检查Keil分组、源码路径和include path。
7. 不要为每个小任务自动写进度流水账；一轮工作结束时先询问用户是否需要记录。
8. 代码功能稳定后补齐职责、状态变化、异常回滚、DMA/ISR并发和缓冲区生命周期注释，再重新编译验证。
9. 用户明确要求记录、更新进度或为新对话保存上下文时，及时更新`00_Doc/TASK_PROGRESS.md`；只有涉及接口、引脚、硬件分配或旧结论变化时，才同步修正详细衔接记录。
10. 平台无关Core统一放在`03_Middleware/<module>`并命名为`<module>_core.c/.h`；`04_Bsp/<module>`只保留板级配置和STM32 HAL适配。目标板测试与PC fake测试都放在`05_Test`，其中PC测试统一位于`05_Test/host/<module>`，不得加入Keil目标。
11. Git提交说明统一使用简短、明确的中文，直接说明本次结果，例如“完成机械臂动作组200测试”“新增Nano视觉通信”“修复转盘光电门超时”；避免只写`update`、`fix`等模糊词。
