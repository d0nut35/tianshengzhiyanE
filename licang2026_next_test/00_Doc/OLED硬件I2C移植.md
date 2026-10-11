# OLED硬件I2C显示

2026-10-12：从用户桌面`abc`移植江协科技OLED驱动，原作者说明、字库与绘图接口保留。代码集中在`04_Bsp/oled`，不增加Service/Core分层、DMA、中断或独立显示任务。

- 使用现有`hi2c1`，PB8=SCL、PB9=SDA；不改CubeMX引脚、Timing或`.ioc`。
- 默认128×64、0.96寸配置，七位地址0x3C，HAL传入0x78。原SH1106列偏移注释保留，1.3寸型号需按实际屏幕确认。
- `OLED_WriteCommand`使用控制字节0x00，`OLED_WriteData`使用0x40，通过`HAL_I2C_Mem_Write`发送；按页连续发128字节，显存1024字节。
- 每次HAL调用传入30ms超时，任何错误后停止本轮后续发送。`OLED_IsReady()`查询最近通信状态；失败后检查连接并重新调用`OLED_Init()`才能恢复。屏幕缺席不会调用`Error_Handler`。
- 在`StartDefaultTask`等待100个内核tick供电稳定，初始化并显示`LICANG 2026`、`OLED I2C1`和`0123456789`，随后进入原底盘握手。显示失败也继续原启动。当前只显示静态测试页，未接入比赛状态/按键。
- 显示函数只改显存，需调用`OLED_Update()`或`OLED_UpdateArea()`才刷新。由单一任务使用，不能多任务同时绘制/刷新。`OLED_Printf`已改成有界`vsnprintf`。

Keil分组与头文件路径已登记。当前用户无线模式编译0错误0警告，日志`MDK-ARM/oled-i2c-build.log`；真实驱动加HAL替身验证地址、控制字节、页数据、首笔无应答、刷新失败后停止及重新初始化通过。未烧录，亮屏、屏幕方向、实际I2C连接以及未接屏的启动仍待实机确认。桌面原始四个文件未修改。
