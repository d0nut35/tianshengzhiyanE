/**
 * @file    zdt_turntable_core.h
 * @brief   ZDT_X42S转盘电机平台无关协议Core。
 *
 * 本层只负责固定0x6B校验模式下的构帧与响应解析，不依赖HAL、DMA或RTOS。
 * X42S同时内置X和Emm固件，两者位置命令格式不同，因此调用者必须先通过
 * 0x1A只读命令确认固件类型，再选择对应构帧接口。
 */

#ifndef ZDT_TURNTABLE_CORE_H
#define ZDT_TURNTABLE_CORE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/** 当前驱动固定使用的协议校验字节。 */
#define ZDT_TURNTABLE_CHECK_BYTE          0x6BU
/** X/Emm两种固件命令帧的统一缓冲区上限。 */
#define ZDT_TURNTABLE_FRAME_MAX           18U
/** 当前已支持响应中最长一帧的字节数。 */
#define ZDT_TURNTABLE_RESPONSE_MAX         8U

/** Core、Service、Device和HAL adapter共用的状态码。 */
typedef enum {
    ZDT_TURNTABLE_OK = 0,
    ZDT_TURNTABLE_ERR_PARAM,
    ZDT_TURNTABLE_ERR_NOT_INIT,
    ZDT_TURNTABLE_ERR_STATE,
    ZDT_TURNTABLE_ERR_BUSY,
    ZDT_TURNTABLE_ERR_IO,
    ZDT_TURNTABLE_ERR_TIMEOUT,
    ZDT_TURNTABLE_ERR_QUEUE_FULL,
    ZDT_TURNTABLE_ERR_PROTOCOL,
    ZDT_TURNTABLE_ERR_DEVICE,
    ZDT_TURNTABLE_ERR_UNSUPPORTED,
} zdt_turntable_status_t;

/** X42S内部运行的协议固件类型。 */
typedef enum {
    ZDT_TURNTABLE_FIRMWARE_X = 0,
    ZDT_TURNTABLE_FIRMWARE_EMM = 1,
} zdt_turntable_firmware_t;

/** 电机命令方向；实际机械正反方向仍需实机标定。 */
typedef enum {
    ZDT_TURNTABLE_DIR_CW = 0,
    ZDT_TURNTABLE_DIR_CCW = 1,
} zdt_turntable_direction_t;

/** 位置命令相对/绝对参考系。 */
typedef enum {
    ZDT_TURNTABLE_POS_RELATIVE_LAST_TARGET = 0,
    ZDT_TURNTABLE_POS_ABSOLUTE_ZERO = 1,
    ZDT_TURNTABLE_POS_RELATIVE_CURRENT = 2,
} zdt_turntable_position_mode_t;

/** 已解析响应的语义类型。 */
typedef enum {
    ZDT_TURNTABLE_REPLY_ACK = 0,
    ZDT_TURNTABLE_REPLY_HOME_IDLE,
    ZDT_TURNTABLE_REPLY_PARAM_ERROR,
    ZDT_TURNTABLE_REPLY_FORMAT_ERROR,
    ZDT_TURNTABLE_REPLY_REACHED,
    ZDT_TURNTABLE_REPLY_OPTIONS,
    ZDT_TURNTABLE_REPLY_STATUS,
    ZDT_TURNTABLE_REPLY_POSITION,
    ZDT_TURNTABLE_REPLY_VERSION,
} zdt_turntable_reply_kind_t;

/**
 * @brief X和Emm位置命令的统一输入参数。
 * @note speed、acceleration和deceleration在两种固件中的单位不同；调用者
 *       必须按所选构帧接口填写，不能直接在X/Emm之间复用数值。
 */
typedef struct {
    zdt_turntable_direction_t direction;
    zdt_turntable_position_mode_t mode;
    uint16_t speed;
    uint16_t acceleration;
    uint16_t deceleration;
    uint32_t angle_0p1deg;
    uint8_t emm_acceleration;
} zdt_turntable_position_command_t;

/**
 * @brief 一帧完整设备返回的解析结果。
 * @note raw按值保存原始帧，不引用transport的DMA接收缓冲区。
 */
typedef struct {
    zdt_turntable_reply_kind_t kind;
    uint8_t address;
    uint8_t function;
    uint8_t raw[ZDT_TURNTABLE_RESPONSE_MAX];
    size_t raw_len;
    union {
        struct {
            uint16_t raw_flags;
            bool motor_step_angle_0p9deg;
            zdt_turntable_firmware_t firmware;
            bool closed_loop;
            bool positive_direction_ccw;
            bool buttons_locked;
            bool scaled_input;
            uint8_t parameter_lock_level;
        } options;
        struct {
            bool enabled;
            bool reached;
            bool stalled;
            bool stall_protected;
            bool left_limit_high;
            bool right_limit_high;
            bool power_loss_latched;
        } motor_status;
        struct {
            bool negative;
            uint32_t magnitude;
        } position;
        struct {
            uint16_t firmware_version;
            uint8_t hardware_series;
            uint8_t hardware_type;
            uint8_t hardware_version;
        } version;
    } data;
} zdt_turntable_response_t;

/**
 * @brief 构造读取固件/硬件版本命令0x1F。
 * @param address 非零设备地址。
 * @param frame 输出帧缓冲区。
 * @param capacity frame容量，至少3字节。
 * @param frame_len 输出有效帧长。
 * @return 构帧结果；OK只表示本地帧已生成。
 */
zdt_turntable_status_t zdt_turntable_build_read_version(
    uint8_t address, uint8_t *frame, size_t capacity, size_t *frame_len);

/**
 * @brief 构造读取选项参数命令0x1A，用于确认X/Emm与闭环状态。
 * @param address 非零设备地址。
 * @param frame 输出帧缓冲区。
 * @param capacity frame容量，至少3字节。
 * @param frame_len 输出有效帧长。
 * @return 构帧结果。
 */
zdt_turntable_status_t zdt_turntable_build_read_options(
    uint8_t address, uint8_t *frame, size_t capacity, size_t *frame_len);

/**
 * @brief 构造读取电机状态标志命令0x3A。
 * @param address 非零设备地址。
 * @param frame 输出帧缓冲区。
 * @param capacity frame容量，至少3字节。
 * @param frame_len 输出有效帧长。
 * @return 构帧结果。
 */
zdt_turntable_status_t zdt_turntable_build_read_status(
    uint8_t address, uint8_t *frame, size_t capacity, size_t *frame_len);

/**
 * @brief 构造读取实时位置命令0x36。
 * @param address 非零设备地址。
 * @param frame 输出帧缓冲区。
 * @param capacity frame容量，至少3字节。
 * @param frame_len 输出有效帧长。
 * @return 构帧结果。
 */
zdt_turntable_status_t zdt_turntable_build_read_position(
    uint8_t address, uint8_t *frame, size_t capacity, size_t *frame_len);

/**
 * @brief 构造X固件梯形加减速位置命令0xFD。
 * @param address 非零设备地址。
 * @param command 方向、位置模式、速度、加减速度和角度参数。
 * @param frame 输出帧缓冲区。
 * @param capacity frame容量，至少16字节。
 * @param frame_len 输出有效帧长。
 * @return 构帧结果。
 * @note angle_0p1deg单位为0.1度；若设备启用了继续缩小10倍输入，真实单位
 *       会变为0.01度，测试前必须通过0x1A确认scaled_input为false。
 */
zdt_turntable_status_t zdt_turntable_build_x_position(
    uint8_t address,
    const zdt_turntable_position_command_t *command,
    uint8_t *frame,
    size_t capacity,
    size_t *frame_len);

/**
 * @brief 构造Emm固件位置命令0xFD。
 * @param address 非零设备地址。
 * @param command 方向、位置模式、速度和Emm加速度档位。
 * @param pulses 位置脉冲数；角度换算依赖电机步距角和细分值。
 * @param frame 输出帧缓冲区。
 * @param capacity frame容量，至少13字节。
 * @param frame_len 输出有效帧长。
 * @return 构帧结果。
 */
zdt_turntable_status_t zdt_turntable_build_emm_position(
    uint8_t address,
    const zdt_turntable_position_command_t *command,
    uint32_t pulses,
    uint8_t *frame,
    size_t capacity,
    size_t *frame_len);

/**
 * @brief 构造立即停止命令0xFE。
 * @param address 非零设备地址。
 * @param frame 输出帧缓冲区。
 * @param capacity frame容量，至少5字节。
 * @param frame_len 输出有效帧长。
 * @return 构帧结果。
 */
zdt_turntable_status_t zdt_turntable_build_stop(
    uint8_t address, uint8_t *frame, size_t capacity, size_t *frame_len);

/**
 * @brief 解析一帧完整返回。
 * @param data 完整响应首地址。
 * @param len 响应字节数；必须与expected_function对应的手册帧长完全一致。
 * @param expected_address 当前事务期望的设备地址。
 * @param expected_function 当前事务期望的功能码。
 * @param response 输出解析结果；协议或参数错误时内容无效。
 * @return OK表示有效成功响应，ERR_DEVICE表示有效设备错误响应，其他值
 *         表示调用参数或协议格式错误。
 * @note 固定0x6B协议没有长度字段；transport必须按一次ReceiveToIdle事务
 *       提供完整单帧。本函数不在输入中扫描或拼接多帧。0x1A严格使用
 *       手册和2.01实机均已确认的5字节响应，不接受截断的4字节兼容格式。
 */
zdt_turntable_status_t zdt_turntable_parse_response(
    const uint8_t *data,
    size_t len,
    uint8_t expected_address,
    uint8_t expected_function,
    zdt_turntable_response_t *response);

#ifdef __cplusplus
}
#endif

#endif /* ZDT_TURNTABLE_CORE_H */
