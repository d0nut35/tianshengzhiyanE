# HWT101 陀螺仪驱动（BSP）

维特智能 HWT101 串口陀螺仪驱动。**纯解析与平台解耦**：协议解包零 HAL、可在 PC 单测；换 MCU / 串口只改适配层。采用**懒扫描**方案——中断只搬数据、不解析，解析放在任务上下文按需触发。

## 文件结构

| 文件 | 层 | 职责 | 依赖 |
|---|---|---|---|
| `hwt101.{c,h}` | 协议 | 帧解析（纯函数：找包头 / 校验 / 解码，只填调用方结构体） | 仅标准库 |
| `hwt101_adaption.{c,h}` | 适配 | huart2 + DMA 双缓冲、ping-pong 收发、懒扫描、ISR 入口 | `hwt101` + STM32 HAL |
| `hwt101_isr_bench.{c,h}` | 测量 | ISR 内"解析 vs 入队"耗时基准（可选，定稿后可删） | FreeRTOS + DWT |

**依赖方向（单向向下）**：`hwt101_adaption → hwt101`。只有适配层 `#include` HAL，解析层零平台依赖。

## 为什么是懒扫描

中断里"解析"会把扫描+校验+解码的开销（实测约 1–3µs）压进 ISR；"入队"要 memcpy + `FromISR`。而懒扫描让**中断只做记长度 + 切缓冲 + 重启 DMA（约 0.1µs）**，把解析推迟到 20ms 控制环里做——HWT101 数据又小又慢，控制环只取"最新 yaw"，懒扫描足矣，且中断最短。

> 边界：懒扫描只解析"最近收完的那块"，帧来得比读取快时中间块会被跳过。对"取最新角度"这正合适；若哪天要**不丢帧地积分陀螺**，才需要到达即解析。

## 数据流

```
接收 (RX, 中断最短)
UART 空闲中断 → it_dispatch(Service, 按 huart 路由)
   └─> hwt101_adp_rx_isr(size)             [适配层, ISR]
         ├─ 记录本块有效长度 g_len[finished]
         ├─ ping-pong 切到另一块
         └─ 重启 ReceiveToIdle DMA（关半传输中断）

读取 (任务, 20ms 控制环)
hwt101_adp_read(&gyro, &yaw)               [适配层, 任务]
   ├─ 取最近收完的块及其长度
   ├─ hwt101_parse(buf, len, &sample)      [纯逻辑: 找 0x55 → 校验 → 解码]
   ├─ 扫描期间又收完一块 → 丢弃(ERR_RES)
   └─ 有新 yaw → 回填输出; 否则 ERR_RES（交上层处理）
```
> 解析放任务、搬数据放中断；HAL 回调归 Service 的 `it_dispatch`，本驱动只暴露 `rx_isr` / `err_isr` 入口。

## 协议（HWT101 标准帧）

| 子帧 | 帧头 | ID | 数据区(索引 6/7) | 解码 |
|---|---|---|---|---|
| 角速度 | `0x55` | `0x52` | WzL / WzH | `raw * 2000 / 32768` → 度/秒 |
| 角度 | `0x55` | `0x53` | YawL / YawH | `raw * 180 / 32768` → 度 |

- 每子帧固定 **11 字节**，第 11 字节 = 前 10 字节累加和（校验）。
- HWT101 每个输出周期**连发上面 2 个子帧（共 22 字节）**，`read` 一趟把 yaw + gyro 都解出来；`HWT101_FRAME_LEN` 指的是单个子帧、不是一次传输。
- 解析器全缓冲找包头，兼容前导乱码；同段多帧时各量取**最后一帧**（最新）。

## 快速使用 / 移植

1. **适配层** `hwt101_adaption.c` 改板级配置：当前固定 `huart2`、缓冲 `HWT101_RX_LEN`（64B）。换串口/芯片只动这里。
2. 装配与启动（Service 层 `system_assembly`）：
```c
hwt101_adp_init();    /* 绑定 huart2 + DMA，清缓存 */
hwt101_adp_start();   /* 启动 ReceiveToIdle DMA 接收 */
```
3. 中断路由（Service 层 `it_dispatch`，独占 HAL 回调）：
```c
/* huart2 的接收事件 / 错误，分别转给本驱动 */
hwt101_adp_rx_isr(size);    /* RxEvent */
hwt101_adp_err_isr();       /* Error */
```
4. 读取（底盘 20ms 控制环，**必须查返回值**）：
```c
float gyro = 0.0f, yaw = 0.0f;
if (hwt101_adp_read(&gyro, &yaw) == HWT101_OK) {
    /* 拿到新 yaw（度），喂里程计前在 chassis 侧转 rad */
}
```
5. 软件标定（任务上下文，替代硬件清零时序）：
```c
/* 把当前航向标定为 30°：读一帧原始 yaw 算偏置，此后 read 输出均含偏置 */
if (hwt101_adp_set_yaw(30.0f) == HWT101_OK) { /* 标定成功 */ }
/* ERR_RES 表示本次无新帧，隔几 ms 重试即可 */
```

## 状态码约定（`hwt101_status_t`，`OK == 0`）

| 码 | 含义 |
|---|---|
| `HWT101_OK` | 解出新 yaw |
| `HWT101_ERR` | 解析层：本段无任何有效帧 |
| `HWT101_ERR_PARAM` | 入参非法（NULL 等） |
| `HWT101_ERR_INIT` | 未初始化 |
| `HWT101_ERR_RES` | 适配层：本次无新 yaw（暂无新数据），由上层处理 |
| `HWT101_ERR_TMO` | HAL 操作超时（预留映射） |

## 约定

- **错误上抛、不内部兜底**：`read` 无新 yaw 时返回 `ERR_RES` 且不写输出，由上层决定"沿用上次值 / 报 IMU 故障"。
- **单位/符号转换不在本层**：本层只出原始度数（yaw/gyro 均为度·度每秒）；deg→rad、`-yaw` 符号约定属底盘侧（`chassis_adaption`）。
- `gyro` 为附带量，本段解出才写；`yaw` 是主信号（里程计用）。
- 跨上下文共享仅 `g_len` / `g_active`（ISR 写、任务读，已 `volatile`）；解析结果在 `read` 单线程内，无需加锁。
- 命名 `snake_case`；函数名 ≤20、变量 ≤15、宏 ≤25 字符。

## 移植到别的板 / 别的陀螺

- 换 UART：改 `hwt101_adaption.c` 里的 `g_uart = &huartX` 与中断路由的句柄判断。
- 换 MCU：只重写 `hwt101_adaption.c`（HAL 接收/重启那几处），`hwt101.c` 不动。
- 换同类协议陀螺：改 `hwt101.h` 的帧常量（`FRAME_HEAD/ID_*/FRAME_LEN`）与 `hwt101.c` 的解码系数。
```
