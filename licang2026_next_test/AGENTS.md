# licang2026_next_test 项目协作约定

开始处理本工程任务前：

1. 如果`D:\programfile\licang\00_Context`存在，先读取其中的`CONTEXT_INDEX.md`和`CURRENT_TASK.md`，再按索引读取专项资料。
2. 如果外层目录不存在，读取`00_Doc/CURRENT_STATUS.md`和`00_Doc/README.md`，再读取与本次任务直接相关的代码和专项文档。

不要默认完整加载历史进度、工作日志、旧工程资料或所有测试说明。只有追溯历史结论时才读取`00_Doc/TASK_PROGRESS.md`和`00_Doc/任务衔接记录.md`；当前STM32状态以代码、Git状态和`00_Doc/CURRENT_STATUS.md`为准。

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
9. 用户明确要求记录、更新进度或为新对话保存上下文时，更新`D:\programfile\licang\00_Context\CURRENT_TASK.md`；STM32工程事实变化时同步更新`00_Doc/TASK_PROGRESS.md`第0节。历史衔接记录不再追加当前状态。
10. 平台无关Core统一放在`03_Middleware/<module>`并命名为`<module>_core.c/.h`；`04_Bsp/<module>`只保留板级配置和STM32 HAL适配。目标板测试与PC fake测试都放在`05_Test`，其中PC测试统一位于`05_Test/host/<module>`，不得加入Keil目标。
11. Git提交说明统一使用简短、明确的中文，直接说明本次结果，例如“完成机械臂动作组200测试”“新增Nano视觉通信”“修复转盘光电门超时”；避免只写`update`、`fix`等模糊词。
12. 发现代码、`CURRENT_STATUS.md`、正式协议、用户已确认决定互相冲突时，先区分“当前实际行为”和“预期行为”。事实可由代码或测试证明时修正文档；预期行为不明确时停止相关修改并向用户指出具体冲突，不得暗自选择。
