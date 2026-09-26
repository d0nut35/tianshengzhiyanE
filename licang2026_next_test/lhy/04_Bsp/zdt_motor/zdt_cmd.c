/**
 * @file    zdt_cmd.c
 * @brief   ZDT 协议组帧 / 解析实现（固定 0x6B 校验）
 * @author  haoyu
 * @note    - 单帧格式：地址 + 功能码 + 数据区 + 0x6B
 *          - 多机帧：00 AA 总长(2B) + 各子帧 + 0x6B
 *          - 多字节字段一律大端；位置符号的方向翻转在 zdt_motor 层
 */

#include "zdt_cmd.h"

#include <string.h>

/* ---- 协议常量 ---- */
#define ZDT_CHK            0x6BU   /* 固定校验字节 */
#define ZDT_SYNC_NOW       0x00U   /* 立即执行 */
#define ZDT_SYNC_BUF       0x01U   /* 缓存待同步触发 */

#define ZDT_CODE_ENABLE    0xF3U   /* 使能控制 */
#define ZDT_CODE_SPEED     0xF6U   /* 速度模式 */
#define ZDT_CODE_POSITION  0xFDU   /* 位置模式 */
#define ZDT_CODE_HOME      0x9AU   /* 触发回零 */
#define ZDT_CODE_REPORT    0x11U   /* 定时返回 */
#define ZDT_CODE_MULTI     0xAAU   /* 多机命令 */
#define ZDT_CODE_POS_RPT   0x36U   /* 实时位置码：读/上报/回复共用 */

#define ZDT_AST_ENABLE     0xABU   /* 使能辅助码 */
#define ZDT_AST_REPORT     0x18U   /* 定时返回辅助码 */

/* 返回数据字节，对应手册 4.1.2 */
#define ZDT_RET_ACK        0x02U   /* 命令接收正确 */
#define ZDT_RET_HOME_IDLE  0x12U   /* 回零时已在零点/限位 */
#define ZDT_RET_PARAM_ERR  0xE2U   /* 参数错误/触发保护 */
#define ZDT_RET_FMT_ERR    0xEEU   /* 命令格式错误 */
#define ZDT_RET_DONE       0x9FU   /* 动作执行完成 */

#define ZDT_POS_FRAME_LEN  8U      /* 位置上报帧长度 */
#define ZDT_RSP_FRAME_LEN  4U      /* 应答帧长度 */
#define ZDT_MULTI_HEAD     4U      /* 多机帧头长：00 AA len_hi len_lo */
#define ZDT_SUB_MIN_LEN    3U      /* 子帧最小长度：addr+code+0x6B */

/* ---- 字节序工具（大端） ---- */

/** @brief 大端写入 16 位无符号值 */
static void write_u16_be(uint8_t *buf, uint16_t val)
{
    buf[0] = (uint8_t)(val >> 8);
    buf[1] = (uint8_t)(val & 0xFFU);
}

/** @brief 大端写入 32 位无符号值 */
static void write_u32_be(uint8_t *buf, uint32_t val)
{
    buf[0] = (uint8_t)((val >> 24) & 0xFFU);
    buf[1] = (uint8_t)((val >> 16) & 0xFFU);
    buf[2] = (uint8_t)((val >> 8)  & 0xFFU);
    buf[3] = (uint8_t)( val        & 0xFFU);
}

/** @brief 大端读取 32 位无符号值 */
static uint32_t read_u32_be(const uint8_t *buf)
{
    return ((uint32_t)buf[0] << 24) |
           ((uint32_t)buf[1] << 16) |
           ((uint32_t)buf[2] << 8)  |
           ((uint32_t)buf[3]        );
}

/**
 * @brief  封装单帧：addr + code + payload + 0x6B
 * @param  buf/cap 输出缓存与容量
 * @param  addr    电机地址
 * @param  code    功能码
 * @param  payload 数据区（可空）
 * @param  plen    数据区长度
 * @param  len     输出整帧长度
 * @retval ZDT_OK / ZDT_ERR_PARAM
 */
static zdt_status_t build_frame(uint8_t *buf, uint16_t cap, uint8_t addr,
                                uint8_t code, const uint8_t *payload,
                                uint16_t plen, uint16_t *len)
{
    uint16_t total = (uint16_t)(plen + 3U); /* 整帧 = 地址+功能码+数据+校验 */

    // 0.校验输出缓存、长度指针、地址与容量
    if ((buf == NULL) || (len == NULL)) {
        return ZDT_ERR_PARAM;
    }
    /* 0x00 仅多机帧头(00 AA)合法，单帧命令必须指定电机 */
    if (addr == ZDT_ADDR_BROADCAST) {
        return ZDT_ERR_PARAM;
    }
    if (total > cap) {
        return ZDT_ERR_PARAM;
    }
    // 1.写入帧头、数据区与校验字节
    buf[0] = addr;
    buf[1] = code;
    if ((payload != NULL) && (plen > 0U)) {
        (void)memcpy(&buf[2], payload, plen);
    }
    buf[2U + plen] = ZDT_CHK;
    *len = total;
    return ZDT_OK;
}

/**
 * @brief  构建使能/失能帧（0xF3，立即同步）
 * @param  buf/cap 输出缓存与容量
 * @param  addr    电机地址
 * @param  on      true=使能，false=失能
 * @param  len     输出帧长度
 * @retval ZDT_OK / ZDT_ERR_PARAM
 */
zdt_status_t zdt_cmd_enable(uint8_t *buf, uint16_t cap, uint8_t addr,
                            bool on, uint16_t *len)
{
    uint8_t payload[3]; /* 辅助码 + 使能态 + 同步标志 */

    // 0.填写使能命令数据区
    payload[0] = ZDT_AST_ENABLE;
    payload[1] = on ? 0x01U : 0x00U;
    payload[2] = ZDT_SYNC_NOW;
    // 1.封装单机命令帧
    return build_frame(buf, cap, addr, ZDT_CODE_ENABLE, payload, 3U, len);
}

/**
 * @brief  构建 Emm 速度模式帧（0xF6，立即执行）
 * @param  buf/cap 输出缓存与容量
 * @param  addr    电机地址
 * @param  spd     速度参数（rpm ≤ ZDT_RPM_MAX）
 * @param  len     输出帧长度
 * @retval ZDT_OK / ZDT_ERR_PARAM
 */
zdt_status_t zdt_cmd_speed(uint8_t *buf, uint16_t cap, uint8_t addr,
                           const zdt_speed_t *spd, uint16_t *len)
{
    uint8_t payload[5]; /* 方向 + 速度(2B) + 加速度 + 同步标志 */

    // 0.校验速度参数
    if ((spd == NULL) || (spd->rpm > ZDT_RPM_MAX)) {
        return ZDT_ERR_PARAM;
    }
    // 1.填写速度命令数据区
    payload[0] = (uint8_t)spd->dir;
    write_u16_be(&payload[1], spd->rpm);
    payload[3] = spd->accel;
    payload[4] = ZDT_SYNC_NOW; /* 单发立即执行 */
    // 2.封装单机命令帧
    return build_frame(buf, cap, addr, ZDT_CODE_SPEED, payload, 5U, len);
}

/**
 * @brief  构建 Emm 位置模式帧（0xFD，立即执行）
 * @param  buf/cap 输出缓存与容量
 * @param  addr    电机地址
 * @param  pos     位置参数（rpm/mode 受校验）
 * @param  len     输出帧长度
 * @retval ZDT_OK / ZDT_ERR_PARAM
 */
zdt_status_t zdt_cmd_pos(uint8_t *buf, uint16_t cap, uint8_t addr,
                         const zdt_pos_t *pos, uint16_t *len)
{
    uint8_t payload[10]; /* 方向+速度(2B)+加速度+脉冲(4B)+模式+同步 */

    // 0.校验位置参数
    if ((pos == NULL) || (pos->rpm > ZDT_RPM_MAX) ||
        ((uint8_t)pos->mode > (uint8_t)ZDT_POS_REL_NOW)) {
        return ZDT_ERR_PARAM;
    }
    // 1.填写位置命令数据区
    payload[0] = (uint8_t)pos->dir;
    write_u16_be(&payload[1], pos->rpm);
    payload[3] = pos->accel;
    write_u32_be(&payload[4], pos->pulse);
    payload[8] = (uint8_t)pos->mode;
    payload[9] = ZDT_SYNC_NOW; /* 单发立即执行 */
    // 2.封装单机命令帧
    return build_frame(buf, cap, addr, ZDT_CODE_POSITION, payload, 10U, len);
}

/**
 * @brief  构建定时上报实时位置帧（0x11，信息码 0x36）
 * @param  buf/cap   输出缓存与容量
 * @param  addr      电机地址
 * @param  period_ms 上报周期 ms
 * @param  len       输出帧长度
 * @retval ZDT_OK / ZDT_ERR_PARAM
 */
zdt_status_t zdt_cmd_report(uint8_t *buf, uint16_t cap, uint8_t addr,
                            uint16_t period_ms, uint16_t *len)
{
    uint8_t payload[4]; /* 辅助码 + 信息码(实时位置) + 周期(2B) */

    // 0.填写上报命令数据区
    payload[0] = ZDT_AST_REPORT;
    payload[1] = ZDT_CODE_POS_RPT;
    write_u16_be(&payload[2], period_ms);
    // 1.封装单机命令帧
    return build_frame(buf, cap, addr, ZDT_CODE_REPORT, payload, 4U, len);
}

/**
 * @brief  构建读取实时位置帧（0x36，无数据区）
 * @param  buf/cap 输出缓存与容量
 * @param  addr    电机地址（≥1，0x00 会被拒绝）
 * @param  len     输出帧长度
 * @retval ZDT_OK / ZDT_ERR_PARAM
 */
zdt_status_t zdt_cmd_read_pos(uint8_t *buf, uint16_t cap, uint8_t addr,
                              uint16_t *len)
{
    // 0.读位置帧仅 addr+功能码+校验，无数据区
    return build_frame(buf, cap, addr, ZDT_CODE_POS_RPT, NULL, 0U, len);
}

/**
 * @brief  构建触发回零帧（0x9A，立即执行）
 * @param  buf/cap 输出缓存与容量
 * @param  addr    电机地址
 * @param  mode    回零模式（≤ZDT_HOME_PWRLOSS）
 * @param  len     输出帧长度
 * @retval ZDT_OK / ZDT_ERR_PARAM
 */
zdt_status_t zdt_cmd_home(uint8_t *buf, uint16_t cap, uint8_t addr,
                          zdt_home_mode_t mode, uint16_t *len)
{
    uint8_t payload[2]; /* 回零模式 + 同步标志 */

    // 0.校验回零模式
    if ((uint8_t)mode > (uint8_t)ZDT_HOME_PWRLOSS) {
        return ZDT_ERR_PARAM;
    }
    // 1.填写回零命令数据区
    payload[0] = (uint8_t)mode;
    payload[1] = ZDT_SYNC_NOW;
    // 2.封装单机命令帧
    return build_frame(buf, cap, addr, ZDT_CODE_HOME, payload, 2U, len);
}

/**
 * @brief  初始化多机拼帧器，写入 00 AA 帧头
 * @param  mb      拼帧器
 * @param  buf/cap 外部缓存与容量
 * @retval ZDT_OK / ZDT_ERR_PARAM
 */
zdt_status_t zdt_multi_init(zdt_multi_t *mb, uint8_t *buf, uint16_t cap)
{
    // 0.校验拼帧器、缓存与容量
    if ((mb == NULL) || (buf == NULL) || (cap < (ZDT_MULTI_HEAD + 1U))) {
        return ZDT_ERR_PARAM;
    }
    // 1.初始化拼帧器并写入多机帧头
    mb->buf = buf;
    mb->cap = cap;
    mb->len = ZDT_MULTI_HEAD;
    buf[0] = ZDT_ADDR_BROADCAST; /* 广播地址 */
    buf[1] = ZDT_CODE_MULTI;     /* 多机命令功能码 */
    buf[2] = 0x00U;          /* 总长高字节，done 回填 */
    buf[3] = 0x00U;          /* 总长低字节，done 回填 */
    return ZDT_OK;
}

/**
 * @brief  向多机帧追加一条自带 0x6B 的子帧
 * @param  mb      拼帧器
 * @param  sub     子帧缓冲
 * @param  sub_len 子帧长度
 * @retval ZDT_OK / ZDT_ERR_PARAM / ZDT_ERR_RES
 */
zdt_status_t zdt_multi_add_raw(zdt_multi_t *mb, const uint8_t *sub,
                               uint16_t sub_len)
{
    // 0.校验拼帧器与子帧指针
    if ((mb == NULL) || (mb->buf == NULL) || (sub == NULL)) {
        return ZDT_ERR_PARAM;
    }
    // 1.校验子帧长度、结尾校验字节与地址
    /* 子帧需自带 0x6B 结尾，且地址非广播 */
    if ((sub_len < ZDT_SUB_MIN_LEN) || (sub[sub_len - 1U] != ZDT_CHK) ||
        (sub[0] == ZDT_ADDR_BROADCAST)) {
        return ZDT_ERR_PARAM;
    }
    if ((uint16_t)(mb->len + sub_len + 1U) > mb->cap) { /* +1 预留末尾校验 */
        return ZDT_ERR_RES;
    }
    // 2.追加子帧并更新当前长度
    (void)memcpy(&mb->buf[mb->len], sub, sub_len);
    mb->len = (uint16_t)(mb->len + sub_len);
    return ZDT_OK;
}

/**
 * @brief  追加一条速度子命令（sync=缓存，待整帧同步执行）
 * @param  mb   拼帧器
 * @param  addr 电机地址
 * @param  spd  速度参数
 * @retval ZDT_OK / ZDT_ERR_PARAM / ZDT_ERR_RES
 */
zdt_status_t zdt_multi_add_speed(zdt_multi_t *mb, uint8_t addr,
                                 const zdt_speed_t *spd)
{
    uint8_t sub[8];          /* 速度子帧缓冲 */
    uint8_t payload[5];      /* 方向+速度(2B)+加速度+同步 */
    uint16_t sub_len = 0U;   /* 子帧长度 */
    zdt_status_t ret = ZDT_ERR; /* 组帧结果 */

    // 0.校验速度参数
    if ((spd == NULL) || (spd->rpm > ZDT_RPM_MAX)) {
        return ZDT_ERR_PARAM;
    }
    // 1.填写并封装速度子帧
    payload[0] = (uint8_t)spd->dir;
    write_u16_be(&payload[1], spd->rpm);
    payload[3] = spd->accel;
    payload[4] = ZDT_SYNC_BUF; /* 多机子帧用缓存，整帧到齐一起执行 */
    ret = build_frame(sub, (uint16_t)sizeof(sub), addr, ZDT_CODE_SPEED,
                      payload, 5U, &sub_len);
    if (ret != ZDT_OK) {
        return ret;
    }
    // 2.追加速度子帧
    return zdt_multi_add_raw(mb, sub, sub_len);
}

/**
 * @brief  收尾多机帧：补末尾 0x6B 并回填总长
 * @param  mb  拼帧器
 * @param  len 输出整帧长度
 * @retval ZDT_OK / ZDT_ERR_PARAM / ZDT_ERR_RES
 */
zdt_status_t zdt_multi_done(zdt_multi_t *mb, uint16_t *len)
{
    uint16_t total = 0U; /* 多机整帧长度 */

    // 0.校验拼帧器、长度指针与剩余容量
    if ((mb == NULL) || (mb->buf == NULL) || (len == NULL)) {
        return ZDT_ERR_PARAM;
    }
    if ((uint16_t)(mb->len + 1U) > mb->cap) {
        return ZDT_ERR_RES;
    }
    // 1.补末尾校验字节并计算整帧长度
    mb->buf[mb->len] = ZDT_CHK; /* 整帧末尾校验 */
    mb->len = (uint16_t)(mb->len + 1U);
    total = mb->len;
    // 2.回填整帧长度
    /* 总长字段含帧头、所有子帧与末尾校验 */
    mb->buf[2] = (uint8_t)(total >> 8);
    mb->buf[3] = (uint8_t)(total & 0xFFU);
    *len = mb->len;
    return ZDT_OK;
}

/**
 * @brief  解析一帧电机返回（位置上报 / 应答），从 data[0] 起
 * @param  data 帧缓冲（不扫描，扫描由 zdt_motor 层做）
 * @param  len  可用字节数
 * @param  out  解析结果，见 zdt_rx_t
 * @retval ZDT_OK / ZDT_ERR_PARAM / ZDT_ERR
 */
zdt_status_t zdt_decode(const uint8_t *data, uint16_t len, zdt_rx_t *out)
{
    // 0.校验输入并清空解析结果
    if ((data == NULL) || (out == NULL)) {
        return ZDT_ERR_PARAM;
    }
    out->kind  = ZDT_RX_NONE;
    out->addr  = 0U;
    out->code  = 0U;
    out->pulse = 0;

    // 1.尝试解析位置上报帧
    /* 位置上报帧：addr 36 sign 脉冲(4B) 6B，共 8 字节（最具体，先判） */
    if ((len >= ZDT_POS_FRAME_LEN) && (data[1] == ZDT_CODE_POS_RPT) &&
        (data[2] <= 1U) && (data[7] == ZDT_CHK)) {
        uint32_t mag = read_u32_be(&data[3]); /* 脉冲绝对值 */

        out->kind = ZDT_RX_POS;
        out->addr = data[0];
        out->code = ZDT_CODE_POS_RPT;
        /* 协议符号 0=正/1=负；按 default_dir 的翻转留给 zdt_motor 层 */
        out->pulse = (data[2] == 0U) ? (int64_t)mag : -(int64_t)mag;
        return ZDT_OK;
    }

    // 2.尝试解析应答帧
    /* 应答帧：addr code 返回字节 6B，共 4 字节 */
    if ((len >= ZDT_RSP_FRAME_LEN) && (data[3] == ZDT_CHK)) {
        out->addr = data[0];
        out->code = data[1];
        switch (data[2]) {
        case ZDT_RET_ACK:
            /* 命令已被驱动器正确接收 */
            out->kind = ZDT_RX_ACK;
            return ZDT_OK;
        case ZDT_RET_HOME_IDLE:
            /* 回零时已经位于零点或限位位置 */
            out->kind = ZDT_RX_HOME_IDLE;
            return ZDT_OK;
        case ZDT_RET_PARAM_ERR:
            /* 命令参数非法或触发驱动器保护 */
            out->kind = ZDT_RX_PARAM_ERR;
            return ZDT_OK;
        case ZDT_RET_FMT_ERR:
            /* 命令帧格式不符合协议 */
            out->kind = ZDT_RX_FMT_ERR;
            return ZDT_OK;
        case ZDT_RET_DONE:
            /* 位置、回零等异步动作已经完成 */
            out->kind = ZDT_RX_DONE;
            return ZDT_OK;
        default:
            /* 未知返回字节交由上层按无法识别处理 */
            break;
        }
    }

    // 3.返回未匹配已知帧
    return ZDT_ERR; /* 无法识别的帧 */
}
