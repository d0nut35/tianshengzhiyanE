/**
 * @file    zdt_motor.h
 * @brief   ZDT 闭环步进电机对象（纯逻辑，平台无关）
 * @author  haoyu
 * @note    - 收发经函数指针注入，本层不依赖 HAL / RTOS
 *          - speed/accel/pos 均为协议语义，物理单位换算在适配层
 *          - 组控流程：逐个 set_speed 缓存 → group_speed 一帧广播
 */

#ifndef ZDT_MOTOR_H
#define ZDT_MOTOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* zdt_status_t / zdt_dir_t / ZDT_RPM_MAX / ZDT_ACCEL_MAX */
#include "zdt_cmd.h"

#define ZDT_GROUP_MAX   10U      /* 电机组最大挂载电机数 */

typedef struct zdt_motor zdt_motor_t;
typedef struct zdt_group zdt_group_t;

/* 单电机发送一帧：把组好的帧交给传输层 */
typedef zdt_status_t (*zdt_send_fn_t)(zdt_motor_t *motor,
                                      const uint8_t *frame,
                                      uint16_t size);

/* 重启单电机接收（DMA + 空闲中断由适配层实现） */
typedef zdt_status_t (*zdt_rx_fn_t)(zdt_motor_t *motor);

/* 多电机广播发送一帧（同一总线同时下发） */
typedef zdt_status_t (*zdt_grp_send_fn_t)(const uint8_t *frame,
                                          uint16_t size);

/* 单电机对象 */
struct zdt_motor {
    uint8_t   is_inited;        /* 是否已实例化 */
    uint8_t   is_enabled;       /* 是否已使能 */
    uint8_t   addr;             /* 电机地址 */
    zdt_dir_t dir;              /* 默认正方向 */
    int16_t   speed;            /* 有符号速度，与 dir 反向时为负 */
    uint8_t   accel;            /* 加速度档 */
    int64_t   pos_pulse;        /* 当前位置脉冲，累计有符号 */
    zdt_send_fn_t pf_send;      /* 单电机发送接口 */
    zdt_rx_fn_t   pf_start_rx;  /* 重启接收接口 */
};

/* 电机组对象 */
struct zdt_group {
    uint8_t      is_inited;             /* 是否已实例化 */
    uint8_t      count;                 /* 已挂载电机数 */
    uint16_t     report_ms;             /* 定时上报周期 ms，0 表示不上报 */
    zdt_motor_t *motors[ZDT_GROUP_MAX]; /* 已挂载电机指针表 */
    zdt_grp_send_fn_t pf_send;          /* 多电机广播发送接口 */
};

/**
 * @brief  实例化单电机对象并注入收发接口
 * @param  motor       电机对象
 * @param  addr        电机地址
 * @param  dir         默认正方向
 * @param  pf_send     单电机发送接口，不可为空
 * @param  pf_start_rx 重启接收接口，不可为空
 * @retval ZDT_OK / ZDT_ERR_PARAM
 */
zdt_status_t zdt_motor_inst(zdt_motor_t *motor,
                            uint8_t addr,
                            zdt_dir_t dir,
                            zdt_send_fn_t pf_send,
                            zdt_rx_fn_t pf_start_rx);

/**
 * @brief  立即使能单电机
 * @param  motor 电机对象
 * @retval ZDT_OK / ZDT_ERR_PARAM / ZDT_ERR_INIT
 */
zdt_status_t zdt_motor_enable(zdt_motor_t *motor);

/**
 * @brief  立即失能单电机
 * @param  motor 电机对象
 * @retval ZDT_OK / ZDT_ERR_PARAM / ZDT_ERR_INIT
 */
zdt_status_t zdt_motor_disable(zdt_motor_t *motor);

/**
 * @brief  仅缓存速度档，不发送（供组控前批量设置）
 * @param  motor 电机对象
 * @param  speed 有符号速度，区间 [-ZDT_RPM_MAX, ZDT_RPM_MAX]
 * @param  accel 加速度档，0..ZDT_ACCEL_MAX
 * @retval ZDT_OK / ZDT_ERR_PARAM / ZDT_ERR_INIT
 */
zdt_status_t zdt_motor_set_speed(zdt_motor_t *motor,
                                 int16_t speed,
                                 uint8_t accel);

/**
 * @brief  立即按速度模式运行单电机（发送单电机帧）
 * @param  motor 电机对象
 * @param  speed 有符号速度，区间 [-ZDT_RPM_MAX, ZDT_RPM_MAX]
 * @param  accel 加速度档，0..ZDT_ACCEL_MAX
 * @retval ZDT_OK / ZDT_ERR_PARAM / ZDT_ERR_INIT
 */
zdt_status_t zdt_motor_run_speed(zdt_motor_t *motor,
                                 int16_t speed,
                                 uint8_t accel);

/**
 * @brief  立即按位置模式运行单电机（发送单电机帧）
 * @param  motor 电机对象
 * @param  mode  位置模式
 * @param  pulse 有符号目标脉冲，符号定方向、幅值定步数
 * @param  rpm   速度幅值，0..ZDT_RPM_MAX
 * @param  accel 加速度档，0..ZDT_ACCEL_MAX
 * @retval ZDT_OK / ZDT_ERR_PARAM / ZDT_ERR_INIT
 */
zdt_status_t zdt_motor_run_pos(zdt_motor_t *motor,
                               zdt_pos_mode_t mode,
                               int32_t pulse,
                               uint16_t rpm,
                               uint8_t accel);

/**
 * @brief  触发单电机回零
 * @param  motor 电机对象
 * @param  mode  回零模式
 * @retval ZDT_OK / ZDT_ERR_PARAM / ZDT_ERR_INIT
 */
zdt_status_t zdt_motor_home(zdt_motor_t *motor,
                            zdt_home_mode_t mode);

/**
 * @brief  解析一帧电机返回：位置上报更新 pos_pulse，其余输出事件
 * @param  motor 电机对象
 * @param  data  返回帧缓冲
 * @param  len   帧长度
 * @param  out   解析结果（应答/到位/回零完成/错误），不可为空
 * @retval ZDT_OK / ZDT_ERR_PARAM / ZDT_ERR_INIT
 */
zdt_status_t zdt_motor_parse(zdt_motor_t *motor, const uint8_t *data,
                             uint16_t len, zdt_rx_t *out);

/**
 * @brief  实例化电机组对象并注入广播发送接口
 * @param  group   电机组对象
 * @param  pf_send 多电机广播发送接口，不可为空
 * @retval ZDT_OK / ZDT_ERR_PARAM
 */
zdt_status_t zdt_group_inst(zdt_group_t *group,
                            zdt_grp_send_fn_t pf_send);

/**
 * @brief  将单电机挂载到组的指定槽位
 * @param  group 电机组对象
 * @param  slot  槽位下标，0..ZDT_GROUP_MAX-1
 * @param  motor 已实例化的电机对象
 * @retval ZDT_OK / ZDT_ERR_PARAM / ZDT_ERR_INIT
 */
zdt_status_t zdt_group_bind(zdt_group_t *group,
                            uint16_t slot,
                            zdt_motor_t *motor);

/**
 * @brief  以一帧广播下发组内所有电机的缓存速度
 * @param  group 电机组对象
 * @retval ZDT_OK / ZDT_ERR_PARAM / ZDT_ERR_INIT
 */
zdt_status_t zdt_group_speed(zdt_group_t *group);

/**
 * @brief  配置组内多电机定时上报位置（同步触发）
 * @param  group     电机组对象
 * @param  period_ms 上报周期 ms，0 表示关闭
 * @retval ZDT_OK / ZDT_ERR_PARAM / ZDT_ERR_INIT
 */
zdt_status_t zdt_group_report(zdt_group_t *group,
                              uint16_t period_ms);

/**
 * @brief  多机命令读实时位置（00 AA 一帧逐台 0x36），四轮各自回复
 * @param  group 电机组对象
 * @retval ZDT_OK / ZDT_ERR_PARAM / ZDT_ERR_INIT / ZDT_ERR_RES
 * @note   回复经各路独立 RX 到达，复用 zdt_motor_parse 的 ZDT_RX_POS 解析
 */
zdt_status_t zdt_group_read_pos(zdt_group_t *group);

#ifdef __cplusplus
}
#endif

#endif /* ZDT_MOTOR_H */
