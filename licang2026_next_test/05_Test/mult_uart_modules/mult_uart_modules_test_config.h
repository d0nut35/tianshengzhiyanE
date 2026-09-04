/** @file mult_uart_modules_test_config.h @brief UART7复用模块集成测试参数。 */

#ifndef MULT_UART_MODULES_TEST_CONFIG_H
#define MULT_UART_MODULES_TEST_CONFIG_H

/** Nano视觉、IC卡、ZDT分别固定接复用通道0、1和2。 */
#define MULT_UART_MODULES_VISION_DEVICE_ID         MULT_UART_DEVICE_0
#define MULT_UART_MODULES_IC_DEVICE_ID             MULT_UART_DEVICE_1
#define MULT_UART_MODULES_ZDT_DEVICE_ID            MULT_UART_DEVICE_2

/** 协议地址、超时和当前Emm细分下每圈脉冲数。 */
#define MULT_UART_MODULES_IC_ADDRESS             0x20U
#define MULT_UART_MODULES_IC_OPERATION_PROMPT      1U
#define MULT_UART_MODULES_IO_TIMEOUT_MS           500U
#define MULT_UART_MODULES_STARTUP_WAIT_MS         5200U
#define MULT_UART_MODULES_ZDT_ADDRESS              1U
#define MULT_UART_MODULES_ZDT_EMM_PULSES_PER_REV 3200U

/**
 * 通道0假数据联调参数。容差和连续帧数只用于通信阶段验证，机械安装后必须
 * 分别为BALL_TURNTABLE和BALL_STAIR重新标定，不能直接作为抓取参数。
 */
#define MULT_UART_MODULES_VISION_IO_TIMEOUT_MS       500U
#define MULT_UART_MODULES_VISION_MAX_RX_CHUNKS          3U
#define MULT_UART_MODULES_VISION_TOLERANCE_X_PX       10U
#define MULT_UART_MODULES_VISION_TOLERANCE_Y_PX        8U
#define MULT_UART_MODULES_VISION_MAX_AGE_MS           100U
#define MULT_UART_MODULES_VISION_LINK_TIMEOUT_MS      500U
#define MULT_UART_MODULES_VISION_CONFIRM_FRAMES         3U
#define MULT_UART_MODULES_VISION_DISCONNECT_TIMEOUTS    3U
#define MULT_UART_MODULES_VISION_POLL_PERIOD_MS         50U
#define MULT_UART_MODULES_VISION_LOG_EVERY_POLLS         5U

/**
 * 转盘高速抓取窗口。实机手动动作12可抓点稳定为DX=23、DY=39；此前运动
 * 抓取有效点约为DX=-18、DY=82。窗口保留两者及少量提前量，但排除画面
 * 边缘候选（曾在DX=-243、DY=-80误触发）。VALID或MODE_NOT_READY均可进入；
 * 响应必须匹配当前轮询SEQ且结果年龄不超过80ms，避免额外等待一帧。
 */
#define MULT_UART_MODULES_VISION_EARLY_MIN_QUALITY       20U
#define MULT_UART_MODULES_VISION_EARLY_MAX_AGE_MS        80U
#define MULT_UART_MODULES_VISION_EARLY_MIN_DX_PX        (-80)
#define MULT_UART_MODULES_VISION_EARLY_MAX_DX_PX          60
#define MULT_UART_MODULES_VISION_EARLY_MIN_DY_PX        (-20)
#define MULT_UART_MODULES_VISION_EARLY_MAX_DY_PX         120

/**
 * LSC16单球抓取动作组。10用于底盘运动/抓取前安全姿态，11用于转盘识别，
 * 12完成一次抓球并放置到圆盘。12自然完成后自动回11，但不会自动授权下一球。
 */
#define MULT_UART_MODULES_LSC16_HOME_GROUP                10U
#define MULT_UART_MODULES_LSC16_VISION_GROUP              11U
#define MULT_UART_MODULES_LSC16_GRASP_PLACE_GROUP         12U
#define MULT_UART_MODULES_LSC16_REPEAT_COUNT               1U
#define MULT_UART_MODULES_LSC16_TX_TIMEOUT_MS           1000U
#define MULT_UART_MODULES_LSC16_STARTED_TIMEOUT_MS      1000U
#define MULT_UART_MODULES_LSC16_COMPLETE_TIMEOUT_MS    30000U
#define MULT_UART_MODULES_LSC16_AUTO_RETURN_VISION          1U

/**
 * 高速连续抓取：动作12放球并自然返回动作11后，最多读取IC三次；只有读卡
 * 成功并提交球档案后才允许内部转盘顺时针走下一槽。每一球重新启动视觉，
 * 当前轮询得到满足年龄和实机窗口的候选即可触发，避免高速圆盘多等一帧。
 * 当前仅开放规则明确为本方5球的圆盘机场景。
 */
#define MULT_UART_MODULES_MULTI_IC_MAX_ATTEMPTS              3U
#define MULT_UART_MODULES_MULTI_IC_RETRY_MS                 150U
#define MULT_UART_MODULES_MULTI_SLOT_USE_CW                   1U

/**
 * LSC16机械运动总锁。首次烧录必须保持0，只验证初始化、命令门和状态输出；
 * 用户确认动作组10/11/12、供电、动作空间和急停后，才可临时改为1。
 */
#ifndef MULT_UART_MODULES_LSC16_MOTION_ARMED
#define MULT_UART_MODULES_LSC16_MOTION_ARMED                1U
#endif

/**
 * 人工单槽运动参数：电机150.2度、60RPM、加速度50、相对上一输入目标位置。
 * 实机五次30度命令移动一个约30度转盘槽位，折算约5:1传动比；150.2度会
 * 生成1335脉冲，与实测单槽位置变化约27341 RAW一致。
 * 粗调使用140.0度/60RPM先停在下一螺丝之前；微调使用1.0度/15RPM沿同方向
 * 逐步寻找PB0高电平。1.0度是电机角度，按约5:1传动后转盘约移动0.2度。
 */
#define MULT_UART_MODULES_ZDT_MOTOR_ANGLE_0P1DEG 1502U
#define MULT_UART_MODULES_ZDT_COARSE_ANGLE_0P1DEG 1400U
#define MULT_UART_MODULES_ZDT_FINE_ANGLE_0P1DEG     10U
#define MULT_UART_MODULES_ZDT_SPEED_RPM             60U
#define MULT_UART_MODULES_ZDT_FINE_SPEED_RPM        15U
#define MULT_UART_MODULES_ZDT_ACCEL                50U

/**
 * 自动逐槽定位参数。CW和CCW都先沿所选方向走粗定位角度，再轮询电机到位；
 * PB0未连续判定为高时，最多沿同方向追加10次1.0度电机微调。超出次数或
 * 总时间后立即退出自动流程并报告错误，禁止继续盲转。正反向目前暂用相同
 * 参数，真实机构若受回差影响，可在完成CCW一整圈标定后再拆成独立参数。
 */
#define MULT_UART_MODULES_ZDT_SLOT_FINE_MAX_STEPS   10U
#define MULT_UART_MODULES_ZDT_SLOT_STATUS_POLL_MS   50U
#define MULT_UART_MODULES_ZDT_SLOT_TIMEOUT_MS      8000U
#define MULT_UART_MODULES_GATE_CONFIRM_SAMPLES       3U
#define MULT_UART_MODULES_GATE_CONFIRM_INTERVAL_MS   5U

/**
 * 机械运动安全锁。首次上电必须保持0，只完成只读查询；确认固定、限位、
 * 急停和转向后才临时改为1。STOP经过软件FIFO，不能代替硬件急停。
 */
#ifndef MULT_UART_MODULES_ZDT_MOTION_ARMED
#define MULT_UART_MODULES_ZDT_MOTION_ARMED          1U
#endif

#endif /* MULT_UART_MODULES_TEST_CONFIG_H */
