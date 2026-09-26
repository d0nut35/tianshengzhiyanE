/**
 * @file    zdt_motor.c
 * @brief   ZDT 电机 / 电机组对象实现（纯逻辑，平台无关）
 * @author  haoyu
 * @note    - 有符号速度/脉冲：正=默认方向，负=反方向，幅值取绝对值
 *          - 收发经注入的 pf_send / pf_start_rx；本层不碰 HAL/RTOS
 *          - parse 写 pos_pulse 不加锁，调用方(handler)需在持锁下调用
 */

#include "zdt_motor.h"

#include <stddef.h> /* NULL */

/* ===== 调试日志：0=不编译进固件，1=经 RTT 输出 ===== */
#ifndef ZDT_MOTOR_LOG_EN
#define ZDT_MOTOR_LOG_EN   1
#endif
#if ZDT_MOTOR_LOG_EN
#include "cat_log.h"
#define ZDT_LOGI(fmt, ...)  LOGI("[zdt] " fmt, ##__VA_ARGS__)
#else
#define ZDT_LOGI(fmt, ...)  do {} while (0)
#endif

#define ZDT_RX_MIN_FRAME   4U                       /* 最短返回帧(应答)字节 */
#define MULTI_BUF_SPEED    (ZDT_GROUP_MAX * 8U + 8U)  /* 速度多机帧缓冲上界 */
#define MULTI_BUF_REPORT   (ZDT_GROUP_MAX * 7U + 8U)  /* 上报多机帧缓冲上界 */
#define MULTI_BUF_RDPOS    (ZDT_GROUP_MAX * 3U + 8U)  /* 读位置多机帧缓冲 */

/* ---- 内部工具 ---- */

#if ZDT_MOTOR_LOG_EN
/**
 * @brief  打印一帧 TX 字节
 * @param  tag 日志标签
 * @param  buf 帧缓冲
 * @param  len 帧长度
 */
static void zdt_log_frame(const char *tag, const uint8_t *buf, uint16_t len)
{
    static const char s_hex[] = "0123456789ABCDEF"; /* 十六进制查表 */
    char     data[(ZDT_FRAME_MAX * 3U) + 1U];       /* "AA "形式缓存 */
    uint16_t i = 0U;                                /* 帧字节下标 */
    uint16_t pos = 0U;                              /* 输出字符下标 */

    if ((tag == NULL) || (buf == NULL) || (len > ZDT_FRAME_MAX)) {
        return;
    }
    for (i = 0U; i < len; i++) {
        if (i != 0U) {
            data[pos] = ' ';
            pos++;
        }
        data[pos] = s_hex[(buf[i] >> 4) & 0x0FU];
        pos++;
        data[pos] = s_hex[buf[i] & 0x0FU];
        pos++;
    }
    data[pos] = '\0';
    ZDT_LOGI("%s len=%u data=%s", tag, (unsigned)len, data);
}
#endif

/** @brief 按符号选协议方向：非负=默认方向，负=反方向 */
static zdt_dir_t pick_dir(zdt_dir_t base, bool negative)
{
    if (!negative) {
        return base;
    }
    return (base == ZDT_DIR_CW) ? ZDT_DIR_CCW : ZDT_DIR_CW;
}

/** @brief 取有符号速度绝对值（int32 中转防取负溢出） */
static uint16_t speed_abs(int16_t speed)
{
    int32_t val = (int32_t)speed; /* 提升到 32 位 */

    if (val < 0) {
        val = -val;
    }
    return (uint16_t)val;
}

/**
 * @brief  实例化单电机对象并注入收发接口
 * @param  motor              电机对象
 * @param  addr               电机地址（非 0）
 * @param  dir                默认正方向
 * @param  pf_send/pf_start_rx 单发 / 重启接收接口
 * @retval ZDT_OK / ZDT_ERR_PARAM
 */
zdt_status_t zdt_motor_inst(zdt_motor_t *motor, uint8_t addr, zdt_dir_t dir,
                            zdt_send_fn_t pf_send, zdt_rx_fn_t pf_start_rx)
{
    // 0.参数合法性检查：对象指针、接口指针、地址非 0
    if ((motor == NULL) || (pf_send == NULL) || (pf_start_rx == NULL)) {
        return ZDT_ERR_PARAM;
    }
    if (addr == 0U) {
        return ZDT_ERR_PARAM;
    }
    // 1.成员赋初值：is_inited 置 1，is_enabled 置 0，其他成员按参数或默认值赋值
    motor->is_inited   = 1U;
    motor->is_enabled  = 0U;
    motor->addr        = addr;
    motor->dir         = dir;
    motor->speed       = 0;
    motor->accel       = 0U;
    motor->pos_pulse   = 0;
    motor->pf_send     = pf_send;
    motor->pf_start_rx = pf_start_rx;
    return ZDT_OK;
}

/**
 * @brief  立即使能单电机（发 0xF3）
 * @param  motor 电机对象
 * @retval ZDT_OK / ZDT_ERR_PARAM / ZDT_ERR_INIT
 */
zdt_status_t zdt_motor_enable(zdt_motor_t *motor)
{
    uint8_t frame[ZDT_FRAME_MAX]; /* 使能帧缓冲 */
    uint16_t flen = 0U;           /* 帧长度 */
    zdt_status_t ret = ZDT_ERR;   /* 结果 */

    // 0.参数合法性检查：对象指针，是否实例化
    if (motor == NULL) {
        return ZDT_ERR_PARAM;
    }
    if (motor->is_inited == 0U) {
        return ZDT_ERR_INIT;
    }

    // 1.构造使能帧并发送；成功则 is_enabled 置 1
    ret = zdt_cmd_enable(frame, (uint16_t)sizeof(frame), motor->addr,
                         true, &flen);
    if (ret != ZDT_OK) {
        return ret;
    }
    ret = motor->pf_send(motor, frame, flen);
    if (ret == ZDT_OK) {
        motor->is_enabled = 1U;
    }
    return ret;
}

/**
 * @brief  立即失能单电机（发 0xF3）
 * @param  motor 电机对象
 * @retval ZDT_OK / ZDT_ERR_PARAM / ZDT_ERR_INIT
 */
zdt_status_t zdt_motor_disable(zdt_motor_t *motor)
{
    uint8_t frame[ZDT_FRAME_MAX]; /* 失能帧缓冲 */
    uint16_t flen = 0U;           /* 帧长度 */
    zdt_status_t ret = ZDT_ERR;   /* 结果 */

    // 0.参数合法性检查：对象指针，是否实例化
    if (motor == NULL) {
        return ZDT_ERR_PARAM;
    }
    if (motor->is_inited == 0U) {
        return ZDT_ERR_INIT;
    }
    // 1.构造失能帧并发送；成功则 is_enabled 置 0
    ret = zdt_cmd_enable(frame, (uint16_t)sizeof(frame), motor->addr,
                         false, &flen);
    if (ret != ZDT_OK) {
        return ret;
    }
    ret = motor->pf_send(motor, frame, flen);
    if (ret == ZDT_OK) {
        motor->is_enabled = 0U;
    }
    return ret;
}

/**
 * @brief  仅缓存有符号速度与加速度，不发送
 * @param  motor 电机对象
 * @param  speed 有符号速度，|speed| ≤ ZDT_RPM_MAX
 * @param  accel 加速度档
 * @retval ZDT_OK / ZDT_ERR_PARAM / ZDT_ERR_INIT
 */
zdt_status_t zdt_motor_set_speed(zdt_motor_t *motor, int16_t speed,
                                 uint8_t accel)
{
    // 0.参数合法性检查：对象指针、实例化状态、速度范围
    if (motor == NULL) {
        return ZDT_ERR_PARAM;
    }
    if (motor->is_inited == 0U) {
        return ZDT_ERR_INIT;
    }
    if ((speed > (int16_t)ZDT_RPM_MAX) || (speed < -(int16_t)ZDT_RPM_MAX)) {
        return ZDT_ERR_PARAM;
    }
    // 1.缓存速度与加速度，等待单发或组发
    motor->speed = speed; /* 仅缓存，待组发或单发时使用 */
    motor->accel = accel;
    return ZDT_OK;
}

/**
 * @brief  立即按速度模式运行单电机（缓存并发送）
 * @param  motor 电机对象
 * @param  speed 有符号速度
 * @param  accel 加速度档
 * @retval ZDT_OK / ZDT_ERR_PARAM / ZDT_ERR_INIT
 */
zdt_status_t zdt_motor_run_speed(zdt_motor_t *motor, int16_t speed,
                                 uint8_t accel)
{
    uint8_t frame[ZDT_FRAME_MAX]; /* 速度帧缓冲 */
    uint16_t flen = 0U;           /* 帧长度 */
    zdt_speed_t spd;              /* 协议速度参数 */
    zdt_status_t ret = ZDT_ERR;   /* 结果 */

    // 0.校验参数并缓存速度
    ret = zdt_motor_set_speed(motor, speed, accel); /* 含 init/范围校验+缓存 */
    if (ret != ZDT_OK) {
        return ret;
    }
    // 1.转换协议方向与速度幅值
    spd.dir   = pick_dir(motor->dir, speed < 0);
    spd.rpm   = speed_abs(speed);
    spd.accel = accel;
    // 2.构造速度帧并发送
    ret = zdt_cmd_speed(frame, (uint16_t)sizeof(frame), motor->addr,
                        &spd, &flen);
    if (ret != ZDT_OK) {
        return ret;
    }
    return motor->pf_send(motor, frame, flen);
}

/**
 * @brief  立即按位置模式运行单电机（符号定方向）
 * @param  motor 电机对象
 * @param  mode  位置模式
 * @param  pulse 有符号目标脉冲
 * @param  rpm/accel 速度幅值 / 加速度
 * @retval ZDT_OK / ZDT_ERR_PARAM / ZDT_ERR_INIT
 */
zdt_status_t zdt_motor_run_pos(zdt_motor_t *motor, zdt_pos_mode_t mode,
                               int32_t pulse, uint16_t rpm, uint8_t accel)
{
    uint8_t frame[ZDT_FRAME_MAX]; /* 位置帧缓冲 */
    uint16_t flen = 0U;           /* 帧长度 */
    zdt_pos_t pos;                /* 协议位置参数 */
    zdt_status_t ret = ZDT_ERR;   /* 结果 */

    // 0.参数合法性检查：对象指针，是否实例化
    if (motor == NULL) {
        return ZDT_ERR_PARAM;
    }
    if (motor->is_inited == 0U) {
        return ZDT_ERR_INIT;
    }

    // 1.转换协议方向与目标脉冲幅值
    pos.dir   = pick_dir(motor->dir, pulse < 0);
    pos.rpm   = rpm;
    pos.accel = accel;

    /* 符号定方向、幅值定步数；int64 中转避免取负溢出 */
    pos.pulse = (pulse >= 0) ? (uint32_t)pulse : (uint32_t)(-(int64_t)pulse);
    pos.mode  = mode;

    // 2.构造位置帧并发送
    ret = zdt_cmd_pos(frame, (uint16_t)sizeof(frame), motor->addr, &pos, &flen);
    if (ret != ZDT_OK) {
        return ret;
    }
    return motor->pf_send(motor, frame, flen);
}

/**
 * @brief  触发单电机回零（发 0x9A）
 * @param  motor 电机对象
 * @param  mode  回零模式
 * @retval ZDT_OK / ZDT_ERR_PARAM / ZDT_ERR_INIT
 */
zdt_status_t zdt_motor_home(zdt_motor_t *motor, zdt_home_mode_t mode)
{
    uint8_t frame[ZDT_FRAME_MAX]; /* 回零帧缓冲 */
    uint16_t flen = 0U;           /* 帧长度 */
    zdt_status_t ret = ZDT_ERR;   /* 结果 */

    // 0.参数合法性检查：对象指针，是否实例化
    if (motor == NULL) {
        return ZDT_ERR_PARAM;
    }
    if (motor->is_inited == 0U) {
        return ZDT_ERR_INIT;
    }
    
    // 1.构造回零帧并发送
    ret = zdt_cmd_home(frame, (uint16_t)sizeof(frame), motor->addr,
                       mode, &flen);
    if (ret != ZDT_OK) {
        return ret;
    }
    return motor->pf_send(motor, frame, flen);
}

/**
 * @brief  扫描并解析本机返回帧；位置帧翻转符号后写 pos_pulse
 * @param  motor 电机对象
 * @param  data  返回帧缓冲
 * @param  len   字节数
 * @param  out   解析结果（事件 / 位置）
 * @retval ZDT_OK / ZDT_ERR_PARAM / ZDT_ERR_INIT / ZDT_ERR
 */
zdt_status_t zdt_motor_parse(zdt_motor_t *motor, const uint8_t *data,
                             uint16_t len, zdt_rx_t *out)
{
    uint16_t off = 0U;   /* 扫描偏移 */
    uint8_t found = 0U;  /* 是否命中本机帧 */

    // 0.参数合法性检查：对象、帧缓存、输出指针与实例化状态
    if ((motor == NULL) || (data == NULL) || (out == NULL)) {
        return ZDT_ERR_PARAM;
    }
    if (motor->is_inited == 0U) {
        return ZDT_ERR_INIT;
    }
    out->kind = ZDT_RX_NONE;

    // 1.扫描并解析属于本机的返回帧
    /* 扫描跳过前导杂字节，找到本机地址起始且解析成功的帧 */
    for (off = 0U; (off + ZDT_RX_MIN_FRAME) <= len; off++) {
        if (data[off] != motor->addr) {
            continue;
        }
        if ((zdt_decode(&data[off], (uint16_t)(len - off), out) == ZDT_OK) &&
            (out->addr == motor->addr)) {
            found = 1U;
            break;
        }
        out->kind = ZDT_RX_NONE; /* 本偏移不成帧，继续扫 */
    }

    // 2.检查是否命中本机有效帧
    if (found == 0U) {
        out->kind = ZDT_RX_NONE;
        return ZDT_ERR;
    }

    // 3.按默认方向更新位置脉冲
    /* 位置帧：协议符号按电机默认方向翻转后写入位置 */
    if (out->kind == ZDT_RX_POS) {
        if (motor->dir == ZDT_DIR_CCW) {
            out->pulse = -out->pulse;
        }
        motor->pos_pulse = out->pulse;
    }
    return ZDT_OK;
}

/**
 * @brief  实例化电机组对象并注入广播发送接口
 * @param  group   电机组对象
 * @param  pf_send 广播发送接口
 * @retval ZDT_OK / ZDT_ERR_PARAM
 */
zdt_status_t zdt_group_inst(zdt_group_t *group, zdt_grp_send_fn_t pf_send)
{
    uint8_t i = 0U; /* 槽位下标 */

    // 0.参数合法性检查：组对象与广播发送接口
    if ((group == NULL) || (pf_send == NULL)) {
        return ZDT_ERR_PARAM;
    }
    // 1.初始化组对象并清空槽位
    group->is_inited = 1U;
    group->count     = 0U;
    group->report_ms = 0U;
    group->pf_send   = pf_send;
    for (i = 0U; i < ZDT_GROUP_MAX; i++) {
        group->motors[i] = NULL;
    }
    return ZDT_OK;
}

/**
 * @brief  将单电机挂载到组的指定槽位
 * @param  group 电机组对象
 * @param  slot  槽位下标（< ZDT_GROUP_MAX）
 * @param  motor 已实例化电机
 * @retval ZDT_OK / ZDT_ERR_PARAM / ZDT_ERR_INIT
 */
zdt_status_t zdt_group_bind(zdt_group_t *group, uint16_t slot,
                            zdt_motor_t *motor)
{
    // 0.参数合法性检查：对象、实例化状态与槽位范围
    if ((group == NULL) || (motor == NULL)) {
        return ZDT_ERR_PARAM;
    }
    if (group->is_inited == 0U) {
        return ZDT_ERR_INIT;
    }
    if ((slot >= ZDT_GROUP_MAX) || (motor->is_inited == 0U)) {
        return ZDT_ERR_PARAM;
    }
    // 1.绑定电机；仅新占槽位时增加计数
    if (group->motors[slot] == NULL) { /* 新占槽位才计数 */
        group->count++;
    }
    group->motors[slot] = motor;
    return ZDT_OK;
}

/**
 * @brief  按各电机缓存速度拼一帧多机命令并广播
 * @param  group 电机组对象
 * @retval ZDT_OK / ZDT_ERR_PARAM / ZDT_ERR_INIT / ZDT_ERR_RES
 */
zdt_status_t zdt_group_speed(zdt_group_t *group)
{
    uint8_t buf[MULTI_BUF_SPEED]; /* 多机速度帧缓冲 */
    zdt_multi_t mb;               /* 拼帧器 */
    zdt_speed_t spd;              /* 单机速度参数 */
    uint16_t flen = 0U;           /* 整帧长度 */
    uint8_t i = 0U;               /* 槽位下标 */
    uint8_t n = 0U;               /* 已加入电机数 */
    zdt_status_t ret = ZDT_ERR;   /* 结果 */

    // 0.参数合法性检查：组对象与实例化状态
    if (group == NULL) {
        return ZDT_ERR_PARAM;
    }
    if (group->is_inited == 0U) {
        return ZDT_ERR_INIT;
    }
    // 1.初始化多机拼帧器
    ret = zdt_multi_init(&mb, buf, (uint16_t)sizeof(buf));
    if (ret != ZDT_OK) {
        return ret;
    }
    // 2.逐槽位追加已挂载电机的速度子帧
    for (i = 0U; i < ZDT_GROUP_MAX; i++) {
        if (group->motors[i] == NULL) {
            continue;
        }
        spd.dir   = pick_dir(group->motors[i]->dir,
                             group->motors[i]->speed < 0);
        spd.rpm   = speed_abs(group->motors[i]->speed);
        spd.accel = group->motors[i]->accel;
        ret = zdt_multi_add_speed(&mb, group->motors[i]->addr, &spd);
        if (ret != ZDT_OK) {
            return ret;
        }
        n++;
    }
    if (n == 0U) {
        return ZDT_ERR_RES;
    }
    // 3.收尾整帧并广播发送
    ret = zdt_multi_done(&mb, &flen);
    if (ret != ZDT_OK) {
        return ret;
    }
    return group->pf_send(buf, flen);
}

/**
 * @brief  配置组内多机定时上报位置周期
 * @param  group     电机组对象
 * @param  period_ms 上报周期 ms
 * @retval ZDT_OK / ZDT_ERR_PARAM / ZDT_ERR_INIT / ZDT_ERR_RES
 */
zdt_status_t zdt_group_report(zdt_group_t *group, uint16_t period_ms)
{
    uint8_t buf[MULTI_BUF_REPORT]; /* 多机上报帧缓冲 */
    uint8_t sub[ZDT_FRAME_MAX];    /* 单机上报子帧 */
    zdt_multi_t mb;                /* 拼帧器 */
    uint16_t flen = 0U;            /* 整帧长度 */
    uint16_t slen = 0U;            /* 子帧长度 */
    uint8_t i = 0U;                /* 槽位下标 */
    uint8_t n = 0U;                /* 已加入电机数 */
    zdt_status_t ret = ZDT_ERR;    /* 结果 */

    // 0.参数合法性检查：组对象与实例化状态
    if (group == NULL) {
        return ZDT_ERR_PARAM;
    }
    if (group->is_inited == 0U) {
        return ZDT_ERR_INIT;
    }
    // 1.初始化多机拼帧器
    ret = zdt_multi_init(&mb, buf, (uint16_t)sizeof(buf));
    if (ret != ZDT_OK) {
        return ret;
    }
    // 2.逐槽位追加已挂载电机的上报子帧
    for (i = 0U; i < ZDT_GROUP_MAX; i++) {
        if (group->motors[i] == NULL) {
            continue;
        }
        ret = zdt_cmd_report(sub, (uint16_t)sizeof(sub),
                             group->motors[i]->addr, period_ms, &slen);
        if (ret != ZDT_OK) {
            return ret;
        }
        ret = zdt_multi_add_raw(&mb, sub, slen);
        if (ret != ZDT_OK) {
            return ret;
        }
        n++;
    }
    if (n == 0U) {
        return ZDT_ERR_RES;
    }
    // 3.收尾整帧并广播发送；成功后回写周期
    ret = zdt_multi_done(&mb, &flen);
    if (ret != ZDT_OK) {
        return ret;
    }
    ret = group->pf_send(buf, flen);
    if (ret == ZDT_OK) {
        group->report_ms = period_ms; /* 发送成功才回写周期 */
    }
    return ret;
}

/**
 * @brief  多机命令读实时位置：逐台 0x36 子帧拼 00 AA 帧下发
 * @param  group 电机组对象
 * @retval ZDT_OK / ZDT_ERR_PARAM / ZDT_ERR_INIT / ZDT_ERR_RES
 * @note   裸广播 00 36 6B 仅 1 号电机回复，须走多机命令封装
 */
zdt_status_t zdt_group_read_pos(zdt_group_t *group)
{
    uint8_t buf[MULTI_BUF_RDPOS]; /* 多机读位置帧缓冲 */
    uint8_t sub[ZDT_FRAME_MAX];   /* 单机读位置子帧 */
    zdt_multi_t mb;               /* 拼帧器 */
    uint16_t flen = 0U;           /* 整帧长度 */
    uint16_t slen = 0U;           /* 子帧长度 */
    uint8_t i = 0U;               /* 槽位下标 */
    uint8_t n = 0U;               /* 已加入电机数 */
    zdt_status_t ret = ZDT_ERR;   /* 结果 */

    // 0.参数合法性检查：组对象与实例化状态
    if (group == NULL) {
        return ZDT_ERR_PARAM;
    }
    if (group->is_inited == 0U) {
        return ZDT_ERR_INIT;
    }
    // 1.初始化多机拼帧器
    ret = zdt_multi_init(&mb, buf, (uint16_t)sizeof(buf));
    if (ret != ZDT_OK) {
        return ret;
    }
    // 2.逐槽位追加已挂载电机的读位置子帧
    for (i = 0U; i < ZDT_GROUP_MAX; i++) {
        if (group->motors[i] == NULL) {
            continue;
        }
        ret = zdt_cmd_read_pos(sub, (uint16_t)sizeof(sub),
                               group->motors[i]->addr, &slen);
        if (ret != ZDT_OK) {
            return ret;
        }
        ret = zdt_multi_add_raw(&mb, sub, slen);
        if (ret != ZDT_OK) {
            return ret;
        }
        n++;
    }
    if (n == 0U) {
        return ZDT_ERR_RES;
    }
    // 3.收尾整帧并经总线发送
    ret = zdt_multi_done(&mb, &flen);
    if (ret != ZDT_OK) {
        return ret;
    }
    return group->pf_send(buf, flen);
}
