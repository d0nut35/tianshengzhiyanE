# ZDT 闭环步进电机驱动（BSP）

面向对象的张大头（ZDT/Emm）串口步进电机驱动。**纯逻辑核心与平台解耦**：换 MCU / 总线只改适配层，核心可在 PC 单测。

## 文件结构

| 文件 | 层 | 职责 | 依赖 |
|---|---|---|---|
| `zdt_cmd.{c,h}` | 协议 | 组帧 / 解析（纯函数，只填调用方缓存） | 仅标准库 |
| `zdt_motor.{c,h}` | 对象 | 单电机 / 电机组对象 + 控制逻辑（纯逻辑） | `zdt_cmd` |
| `zdt_motor_adaption.{c,h}` | 适配 | HAL UART+DMA 端口、对象装配、ISR 入口 | `zdt_motor` + STM32 HAL |
| `../../00_Doc/ref/zdt_command_full/` | 归档 | 完整 60 条协议库（不参与编译，按需回捞） | — |

**依赖方向（单向向下）**：`zdt_motor_adaption → zdt_motor → zdt_cmd`。
只有适配层 `#include` HAL；核心两层零平台依赖。

## 对象模型

- `zdt_motor_t`：单电机。状态（地址/方向/速度/位置脉冲）+ 注入的收发接口（`pf_send` / `pf_start_rx`）。
- `zdt_group_t`：电机组。挂载多台电机 + 广播发送接口（`pf_send`），用一帧多机命令同步下发。
- 平台能力一律靠**函数指针注入**，对象不知道底层是哪个 UART。

## 数据流

**控制（TX）**
```
zdt_motor_set_speed × N   (仅缓存各电机速度)
        └─> zdt_group_speed    (拼一帧多机命令)
              └─> group.pf_send  →  HAL_UART_Transmit_DMA   [适配层]
```

**接收（RX）**
```
UART 空闲中断 → it_dispatch(Service, 按 huart 路由)
   └─> zdt_adp_rx_isr(idx, size)       [适配层, ISR]
         ├─ on_frame 回调 → Service FromISR 入队（拷贝该帧）
         └─ 重启接收 pf_start_rx
任务里：出队 → zdt_motor_parse(motor, data, len, &rx)   [纯逻辑]
         ├─ 位置上报 → 更新 motor->pos_pulse（里程计读它）
         └─ 应答/到位/回零完成/错误 → rx.kind 抛给上层
```
> 解析放任务、重启接收放中断；OS 队列归 Service，本驱动不碰 RTOS。

## 快速使用 / 移植

1. **适配层** `zdt_motor_adaption.c` 里改板级配置：UART 句柄、电机地址、默认方向、上报周期。
2. 装配与启动（Service 层）：
```c
zdt_adp_init(motor_rx_enqueue);   /* 注入"收到一帧"回调（回调做 FromISR 入队）*/
zdt_adp_start();                  /* 使能全部电机 + 启动接收 + 开启定时位置上报 */
```
3. 控制（如底盘 `set_motors`，逐个缓存再一帧广播）：
```c
zdt_motor_set_speed(zdt_adp_motor(0), v0, accel);
zdt_motor_set_speed(zdt_adp_motor(1), v1, accel);
zdt_motor_set_speed(zdt_adp_motor(2), v2, accel);
zdt_motor_set_speed(zdt_adp_motor(3), v3, accel);
zdt_group_speed(zdt_adp_group());
```
4. 读位置（里程计）：`int64_t p = zdt_adp_motor(idx)->pos_pulse;`
5. 单机位置 / 回零：`zdt_motor_run_pos(...)` / `zdt_motor_home(...)`。

## 支持的命令（精简集）

| 接口 | 协议 |
|---|---|
| `zdt_motor_enable / disable` | 0xF3 使能 |
| `zdt_motor_run_speed` / `zdt_group_speed` | 0xF6 速度（单 / 多机） |
| `zdt_motor_run_pos` | 0xFD 位置模式 |
| `zdt_motor_home` | 0x9A 回零 |
| `zdt_group_report` | 0x11 定时上报位置 |
| `zdt_group_read_pos` | 0x36 多机读位置（00 AA 一帧，四轮各自回复） |
| `zdt_motor_parse` | 解析返回帧（位置 / 应答 / 完成 / 错误） |

## 返回帧分类（`zdt_rx_kind_t`，对应手册 4.1.2）

| 返回 | 枚举 | 含义 |
|---|---|---|
| 位置上报 | `ZDT_RX_POS` | 实时位置 → 更新 `pos_pulse` |
| `02` | `ZDT_RX_ACK` | 命令接收正确 |
| `12` | `ZDT_RX_HOME_IDLE` | 回零时已在零点/限位，电机未动 |
| `E2` | `ZDT_RX_PARAM_ERR` | 参数错误 / 触发保护 |
| `EE` | `ZDT_RX_FMT_ERR` | 命令格式错误 |
| `9F` | `ZDT_RX_DONE` | 动作完成（`code`：FB/FD 到位、9A 回零、F5 夹爪） |

## 约定

- 状态码统一 `zdt_status_t`，**`OK == 0`**。
- 速度 / 位置用**有符号**入参：符号定方向（相对电机默认正方向），幅值定大小。
- 缓存安全：所有 builder 取 `(buf, cap, …, len)`，按 `cap` 越界检查；**全程无动态内存**。
- 命名 `snake_case`；函数名 ≤20、变量 ≤15、宏 ≤25 字符。
- 多电机访问需上层加锁（速度控制任务 vs 接收解析任务共享对象）。

## 扩展

需要位置窗口、PID、堵转、回零设零点等命令时，从 `00_Doc/ref/zdt_command_full/` 把对应 `ZDT_Build*` builder 捞回 `zdt_cmd`，按本模块命名 / 状态码（`OK==0`）改写即可。
