# 正式Mission底盘对接要求

接口定义：`02_Service/chassis_bridge/chassis_mission_link.h`  
上层实现：`01_App/mission/mission_app.c`

## 1. 当前上层流程

```text
动作10与底盘初始化完成
→ 双方READY
→ 红方或蓝方START
→ 圆盘区域
→ 阶梯低层、高层、中层
→ COMPLETE
```

圆盘区域：

```text
底盘到圆盘工作位
→ 动作11与Nano圆盘识别
→ 动作12抓球
→ IC读卡
→ 车载转盘前进一格
→ 共处理5球
→ 第5球执行动作17，再回动作10
```

阶梯区域：

```text
底盘到阶梯起点
→ 动作13
→ 低层、高层、中层依次扫描
→ 识别到球后停车
→ 动作14/15/16抓取对应层小球
→ 回动作13
→ IC读卡并转动存球转盘
→ 恢复本层剩余横移
```

阶梯三层合计最多抓2球。当前`STAIRS_FINISHED`后直接进入`COMPLETE`，仓库流程尚未实现。

## 2. 底盘必须提供的行为

| 收到或到达节点 | 底盘必须完成 | 上报Mission |
|---|---|---|
| 底盘初始化完成 | 电机、IMU、底盘控制均可用 | `CHASSIS_CMD_MISSION_READY` |
| `MISSION_CMD_MISSION_READY` | 记录双方READY，保持静止 | 无 |
| `MISSION_CMD_GO_PLATFORM` | 执行起点到圆盘工作位路线并停稳 | `CHASSIS_CMD_PLATFORM_READY` |
| `MISSION_CMD_GO_STAIRS` | 执行圆盘到阶梯起点路线并停稳 | `CHASSIS_CMD_STAIRS_READY` |
| 到达低层起点 | 停在低层扫描起点 | `CHASSIS_CMD_STAIR_LOW` |
| 到达高层起点 | 停在高层扫描起点 | `CHASSIS_CMD_STAIR_HIGH` |
| 到达中层起点 | 停在中层扫描起点 | `CHASSIS_CMD_STAIR_MID` |
| `MISSION_CMD_CAM_READY` | 开始当前层横移 | 无 |
| `MISSION_CMD_STAIR_STOP` | 实际停车并保存本层剩余行程 | `CHASSIS_CMD_STAIR_PAUSE` |
| `MISSION_CMD_STAIR_RESUME` | 从停车位置继续剩余横移 | `CHASSIS_CMD_STAIR_RESUME` |
| 中层扫描完成 | 停止阶梯运动 | `CHASSIS_CMD_STAIRS_FINISHED` |
| `MISSION_CMD_STOP` | 停止当前底盘动作 | `CHASSIS_CMD_STOPPED` |

## 3. 必须满足的通信要求

- 阻塞读取`chassis_command_queue`。
- 通过`chassis_mission_link_post_event()`上报事件。
- 所有回复带回对应命令的原`request_id`。
- `request_id=0`无效；底盘首次READY使用非零编号。
- 成功时`is_ready=1`，失败时`is_ready=0`。
- `PLATFORM_READY`、`STAIRS_READY`和各层事件只能在真正到位后上报。
- `STAIR_PAUSE`只能在真正停稳并保存剩余行程后上报。
- 收到`CAM_READY`前不得开始当前层横移。
- 恢复时继续剩余行程，不重新执行整层横移。
- 运动期间必须能及时处理`STAIR_STOP`和`STOP`。
- 底盘上电后不得自动执行旧平台、阶梯、仓库和回原点路线。

## 4. 正式联调前需要恢复

- `Core/Src/freertos.c`使用`mission_app_init()`，不使用临时`ap_test_init()`。
- 恢复调用`chassis_bridge_boot()`。
- BLE或按键向Mission提交红方开始、蓝方开始和停止命令。
