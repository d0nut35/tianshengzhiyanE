/**
 * @file    zdt_cmd.h
 * @brief   ZDT 协议组帧 / 解析（精简版，仅速度控制所需）
 * @author  haoyu
 * @note    - 纯函数：无状态、无平台依赖，只填调用方提供的缓存
 *          - 仅含 enable / speed / report + 多机拼帧 + 位置解析
 *          - 完整命令库见 00_Doc/ref/zdt_command_full，按需回捞
 */

#ifndef ZDT_CMD_H
#define ZDT_CMD_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#define ZDT_RPM_MAX     30000U   /* 协议速度幅值上限 RPM */
#define ZDT_ACCEL_MAX   0xFFU    /* 加速度档上限 */
#define ZDT_FRAME_MAX   36U      /* 单条子帧最大字节，含头与校验，留余量 */
#define ZDT_ADDR_BROADCAST  0x00U   /* 多机命令帧头地址；单帧命令禁用 */

/* 状态码：OK==0，适配层据此逐层映射 */
typedef enum {
    ZDT_OK        = 0,    /* 成功 */
    ZDT_ERR       = 1,    /* 通用错误 */
    ZDT_ERR_PARAM = 2,    /* 参数非法 */
    ZDT_ERR_INIT  = 3,    /* 未初始化 */
    ZDT_ERR_RES   = 4,    /* 资源不可用（缓存不足 / 接口空） */
} zdt_status_t;

/* 方向，与协议方向字节对齐 */
typedef enum {
    ZDT_DIR_CW  = 0U,     /* 顺时针 */
    ZDT_DIR_CCW = 1U,     /* 逆时针 */
} zdt_dir_t;

/* 位置模式（0xFD 数据区） */
typedef enum {
    ZDT_POS_REL_LAST = 0x00U,   /* 相对上次目标位置 */
    ZDT_POS_ABS      = 0x01U,   /* 绝对位置 */
    ZDT_POS_REL_NOW  = 0x02U,   /* 相对当前位置 */
} zdt_pos_mode_t;

/* 回零模式（手册 5.4.2） */
typedef enum {
    ZDT_HOME_NEAREST = 0x00U,   /* 单圈就近 */
    ZDT_HOME_DIR     = 0x01U,   /* 单圈方向 */
    ZDT_HOME_COLLIDE = 0x02U,   /* 碰撞 */
    ZDT_HOME_LIMIT   = 0x03U,   /* 限位开关 */
    ZDT_HOME_ABS     = 0x04U,   /* 多圈绝对零点 */
    ZDT_HOME_PWRLOSS = 0x05U,   /* 掉电位置 */
} zdt_home_mode_t;

/* 速度模式参数（sync 由单发/组发函数自行决定，不入结构体） */
typedef struct {
    zdt_dir_t dir;        /* 方向 */
    uint16_t  rpm;        /* 速度幅值，0..ZDT_RPM_MAX */
    uint8_t   accel;      /* 加速度档，0..ZDT_ACCEL_MAX */
} zdt_speed_t;

/* 位置模式参数 */
typedef struct {
    zdt_dir_t      dir;     /* 方向 */
    uint16_t       rpm;     /* 速度幅值，0..ZDT_RPM_MAX */
    uint8_t        accel;   /* 加速度档，0..ZDT_ACCEL_MAX */
    uint32_t       pulse;   /* 目标脉冲数 */
    zdt_pos_mode_t mode;    /* 位置模式 */
} zdt_pos_t;

/* 多机命令拼帧器：multi_init -> add_speed / add_raw ... -> multi_done */
typedef struct {
    uint8_t *buf;         /* 外部缓存 */
    uint16_t cap;         /* 缓存容量 */
    uint16_t len;         /* 已写入字节数 */
} zdt_multi_t;

/* 电机返回帧类型（4.1.2：返回字节 02/12/E2/EE/9F + 位置上报） */
typedef enum {
    ZDT_RX_NONE = 0,      /* 无法识别 / 校验失败 */
    ZDT_RX_POS,           /* 实时位置上报 */
    ZDT_RX_ACK,           /* 02：命令接收正确 */
    ZDT_RX_HOME_IDLE,     /* 12：回零时已在零点/限位，电机未动 */
    ZDT_RX_PARAM_ERR,     /* E2：参数错误或触发保护 */
    ZDT_RX_FMT_ERR,       /* EE：命令格式错误 */
    ZDT_RX_DONE,          /* 9F：动作完成，看 code 区分 */
} zdt_rx_kind_t;

/* 电机返回帧解析结果 */
typedef struct {
    zdt_rx_kind_t kind;   /* 帧类型 */
    uint8_t       addr;   /* 来源电机地址 */
    uint8_t       code;   /* 功能码：DONE 时 FB/FD=到位、9A=回零、F5=夹爪 */
    int64_t       pulse;  /* kind==ZDT_RX_POS 时有效：位置脉冲 */
} zdt_rx_t;

/**
 * @brief  使能 / 失能帧（0xF3）
 * @param  buf  输出缓存
 * @param  cap  缓存容量
 * @param  addr 电机地址
 * @param  on   true=使能，false=失能
 * @param  len  输出帧长度
 * @retval ZDT_OK / ZDT_ERR_PARAM
 */
zdt_status_t zdt_cmd_enable(uint8_t *buf, uint16_t cap, uint8_t addr,
                            bool on, uint16_t *len);

/**
 * @brief  Emm 速度模式帧（0xF6），立即执行
 * @param  buf  输出缓存
 * @param  cap  缓存容量
 * @param  addr 电机地址
 * @param  spd  速度参数
 * @param  len  输出帧长度
 * @retval ZDT_OK / ZDT_ERR_PARAM
 */
zdt_status_t zdt_cmd_speed(uint8_t *buf, uint16_t cap, uint8_t addr,
                           const zdt_speed_t *spd, uint16_t *len);

/**
 * @brief  Emm 位置模式帧（0xFD），立即执行
 * @param  buf  输出缓存
 * @param  cap  缓存容量
 * @param  addr 电机地址
 * @param  pos  位置参数
 * @param  len  输出帧长度
 * @retval ZDT_OK / ZDT_ERR_PARAM
 */
zdt_status_t zdt_cmd_pos(uint8_t *buf, uint16_t cap, uint8_t addr,
                         const zdt_pos_t *pos, uint16_t *len);

/**
 * @brief  触发回零帧（0x9A），立即执行
 * @param  buf  输出缓存
 * @param  cap  缓存容量
 * @param  addr 电机地址
 * @param  mode 回零模式
 * @param  len  输出帧长度
 * @retval ZDT_OK / ZDT_ERR_PARAM
 */
zdt_status_t zdt_cmd_home(uint8_t *buf, uint16_t cap, uint8_t addr,
                          zdt_home_mode_t mode, uint16_t *len);

/**
 * @brief  配置定时上报实时位置帧（0x11）
 * @param  buf       输出缓存
 * @param  cap       缓存容量
 * @param  addr      电机地址
 * @param  period_ms 上报周期 ms，0 表示关闭
 * @param  len       输出帧长度
 * @retval ZDT_OK / ZDT_ERR_PARAM
 */
zdt_status_t zdt_cmd_report(uint8_t *buf, uint16_t cap, uint8_t addr,
                            uint16_t period_ms, uint16_t *len);

/**
 * @brief  读取实时位置帧（0x36），无数据区
 * @param  buf  输出缓存
 * @param  cap  缓存容量
 * @param  addr 电机地址（≥1，0x00 会被拒绝）
 * @param  len  输出帧长度
 * @retval ZDT_OK / ZDT_ERR_PARAM
 */
zdt_status_t zdt_cmd_read_pos(uint8_t *buf, uint16_t cap, uint8_t addr,
                              uint16_t *len);

/**
 * @brief  初始化多机拼帧器
 * @param  mb  拼帧器
 * @param  buf 外部缓存
 * @param  cap 缓存容量
 * @retval ZDT_OK / ZDT_ERR_PARAM
 */
zdt_status_t zdt_multi_init(zdt_multi_t *mb, uint8_t *buf, uint16_t cap);

/**
 * @brief  向多机帧追加一条速度子命令（内部置 sync=缓存）
 * @param  mb   拼帧器
 * @param  addr 电机地址
 * @param  spd  速度参数
 * @retval ZDT_OK / ZDT_ERR_PARAM / ZDT_ERR_RES
 */
zdt_status_t zdt_multi_add_speed(zdt_multi_t *mb, uint8_t addr,
                                 const zdt_speed_t *spd);

/**
 * @brief  向多机帧追加一条已组好的子帧（如上报帧）
 * @param  mb      拼帧器
 * @param  sub     子帧缓冲
 * @param  sub_len 子帧长度
 * @retval ZDT_OK / ZDT_ERR_PARAM / ZDT_ERR_RES
 */
zdt_status_t zdt_multi_add_raw(zdt_multi_t *mb, const uint8_t *sub,
                               uint16_t sub_len);

/**
 * @brief  收尾多机帧，补总长与校验
 * @param  mb  拼帧器
 * @param  len 输出帧长度
 * @retval ZDT_OK / ZDT_ERR_PARAM
 */
zdt_status_t zdt_multi_done(zdt_multi_t *mb, uint16_t *len);

/**
 * @brief  解析一帧电机返回（位置上报 / 应答 / 完成 / 错误）
 * @param  data 返回帧缓冲
 * @param  len  帧长度
 * @param  out  解析结果，见 zdt_rx_t
 * @retval ZDT_OK / ZDT_ERR_PARAM / ZDT_ERR
 */
zdt_status_t zdt_decode(const uint8_t *data, uint16_t len, zdt_rx_t *out);

#ifdef __cplusplus
}
#endif

#endif /* ZDT_CMD_H */
