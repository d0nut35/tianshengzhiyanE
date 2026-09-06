/** @file turn_bsp.c @brief ZDT_X42S协议与固定UART7 HAL BSP实现。 */

#include "turn_bsp.h"

#include <string.h>

#if TURN_BSP_HAL_ENABLE
#include <limits.h>
#include "usart.h"
#endif

#define ZDT_CODE_OPTIONS       0x1AU
#if !LICANG_RELEASE_MINIMAL
#define ZDT_CODE_VERSION       0x1FU
#define ZDT_CODE_REAL_POSITION 0x36U
#endif
#define ZDT_CODE_POSITION      0xFDU
#define ZDT_CODE_STOP          0xFEU
#define ZDT_CODE_STATUS        0x3AU

/**
 * @brief 以大端序写入16位协议字段。
 * @param dst 至少可写2字节的目标地址。
 * @param value 待编码数值。
 */
static void zdt_write_u16_be(uint8_t *dst, uint16_t value)
{
    dst[0] = (uint8_t)(value >> 8U);
    dst[1] = (uint8_t)value;
}

/**
 * @brief 以大端序写入32位协议字段。
 * @param dst 至少可写4字节的目标地址。
 * @param value 待编码数值。
 */
static void zdt_write_u32_be(uint8_t *dst, uint32_t value)
{
    dst[0] = (uint8_t)(value >> 24U);
    dst[1] = (uint8_t)(value >> 16U);
    dst[2] = (uint8_t)(value >> 8U);
    dst[3] = (uint8_t)value;
}

/**
 * @brief 从大端序4字节字段读取无符号数值。
 * @param src 至少包含4字节的源地址。
 * @return 解码后的32位数值。
 */
#if !LICANG_RELEASE_MINIMAL
static uint32_t zdt_read_u32_be(const uint8_t *src)
{
    return ((uint32_t)src[0] << 24U) |
           ((uint32_t)src[1] << 16U) |
           ((uint32_t)src[2] << 8U) |
           (uint32_t)src[3];
}
#endif

/**
 * @brief 从大端序2字节字段读取无符号数值。
 * @param src 至少包含2字节的源地址。
 * @return 解码后的16位数值。
 */
static uint16_t zdt_read_u16_be(const uint8_t *src)
{
    return (uint16_t)(((uint16_t)src[0] << 8U) | src[1]);
}

/**
 * @brief 构造“地址+功能码+0x6B”的公共只读命令。
 * @param address 非零设备地址。
 * @param function 查询功能码。
 * @param frame 输出缓冲区。
 * @param capacity 输出容量。
 * @param frame_len 输出有效长度。
 * @return 参数有效且成功构帧时返回OK。
 */
static zdt_turntable_status_t zdt_build_read(
    uint8_t address,
    uint8_t function,
    uint8_t *frame,
    size_t capacity,
    size_t *frame_len)
{
    if ((address == 0U) || (frame == NULL) || (frame_len == NULL) ||
        (capacity < 3U)) {
        return ZDT_TURNTABLE_ERR_PARAM;
    }
    frame[0] = address;
    frame[1] = function;
    frame[2] = ZDT_TURNTABLE_CHECK_BYTE;
    *frame_len = 3U;
    return ZDT_TURNTABLE_OK;
}

#if !LICANG_RELEASE_MINIMAL
/** @copydoc turn_version_frame() */
zdt_turntable_status_t turn_version_frame(
    uint8_t address, uint8_t *frame, size_t capacity, size_t *frame_len)
{
    return zdt_build_read(address, ZDT_CODE_VERSION, frame, capacity, frame_len);
}
#endif

/** @copydoc turn_options_frame() */
zdt_turntable_status_t turn_options_frame(
    uint8_t address, uint8_t *frame, size_t capacity, size_t *frame_len)
{
    return zdt_build_read(address, ZDT_CODE_OPTIONS, frame, capacity, frame_len);
}

/** @copydoc turn_status_frame() */
zdt_turntable_status_t turn_status_frame(
    uint8_t address, uint8_t *frame, size_t capacity, size_t *frame_len)
{
    return zdt_build_read(address, ZDT_CODE_STATUS, frame, capacity, frame_len);
}

#if !LICANG_RELEASE_MINIMAL
/** @copydoc turn_position_frame() */
zdt_turntable_status_t turn_position_frame(
    uint8_t address, uint8_t *frame, size_t capacity, size_t *frame_len)
{
    return zdt_build_read(
        address, ZDT_CODE_REAL_POSITION, frame, capacity, frame_len);
}
#endif

/**
 * @brief 校验位置命令中两种固件共用的字段范围。
 * @param command 待校验参数。
 * @return 非空且枚举合法时返回true；手册允许绝对位置命令使用零目标。
 * @note 速度上限由各固件构帧函数分别检查。
 */
static bool zdt_position_command_valid(
    const zdt_turntable_position_command_t *command)
{
    return (command != NULL) &&
           ((uint32_t)command->direction <= ZDT_TURNTABLE_DIR_CCW) &&
           ((uint32_t)command->mode <= ZDT_TURNTABLE_POS_RELATIVE_CURRENT);
}

#if !LICANG_RELEASE_MINIMAL
/** @copydoc turn_x_frame() */
zdt_turntable_status_t turn_x_frame(
    uint8_t address,
    const zdt_turntable_position_command_t *command,
    uint8_t *frame,
    size_t capacity,
    size_t *frame_len)
{
    if ((address == 0U) || !zdt_position_command_valid(command) ||
        (frame == NULL) || (frame_len == NULL) || (capacity < 16U) ||
        (command->speed > 30000U)) {
        return ZDT_TURNTABLE_ERR_PARAM;
    }
    frame[0] = address;
    frame[1] = ZDT_CODE_POSITION;
    frame[2] = (uint8_t)command->direction;
    zdt_write_u16_be(&frame[3], command->acceleration);
    zdt_write_u16_be(&frame[5], command->deceleration);
    zdt_write_u16_be(&frame[7], command->speed);
    zdt_write_u32_be(&frame[9], command->angle_0p1deg);
    frame[13] = (uint8_t)command->mode;
    frame[14] = 0U;
    frame[15] = ZDT_TURNTABLE_CHECK_BYTE;
    *frame_len = 16U;
    return ZDT_TURNTABLE_OK;
}
#endif

/** @copydoc turn_emm_frame() */
zdt_turntable_status_t turn_emm_frame(
    uint8_t address,
    const zdt_turntable_position_command_t *command,
    uint32_t pulses,
    uint8_t *frame,
    size_t capacity,
    size_t *frame_len)
{
    if ((address == 0U) || !zdt_position_command_valid(command) ||
        (frame == NULL) || (frame_len == NULL) || (capacity < 13U) ||
        (command->speed > 3000U)) {
        return ZDT_TURNTABLE_ERR_PARAM;
    }
    frame[0] = address;
    frame[1] = ZDT_CODE_POSITION;
    frame[2] = (uint8_t)command->direction;
    zdt_write_u16_be(&frame[3], command->speed);
    frame[5] = command->emm_acceleration;
    zdt_write_u32_be(&frame[6], pulses);
    frame[10] = (uint8_t)command->mode;
    frame[11] = 0U;
    frame[12] = ZDT_TURNTABLE_CHECK_BYTE;
    *frame_len = 13U;
    return ZDT_TURNTABLE_OK;
}

/** @copydoc turn_stop_frame() */
zdt_turntable_status_t turn_stop_frame(
    uint8_t address, uint8_t *frame, size_t capacity, size_t *frame_len)
{
    if ((address == 0U) || (frame == NULL) || (frame_len == NULL) ||
        (capacity < 5U)) {
        return ZDT_TURNTABLE_ERR_PARAM;
    }
    frame[0] = address;
    frame[1] = ZDT_CODE_STOP;
    frame[2] = 0x98U;
    frame[3] = 0U;
    frame[4] = ZDT_TURNTABLE_CHECK_BYTE;
    *frame_len = 5U;
    return ZDT_TURNTABLE_OK;
}

/** @copydoc turn_parse() */
zdt_turntable_status_t turn_parse(
    const uint8_t *data,
    size_t len,
    uint8_t expected_address,
    uint8_t expected_function,
    zdt_turntable_response_t *response)
{
    uint16_t flags;
    uint8_t value;

    if ((data == NULL) || (response == NULL)) {
        return ZDT_TURNTABLE_ERR_PARAM;
    }
    if ((len < 4U) || (len > ZDT_TURNTABLE_RESPONSE_MAX)) {
        /* UART交付的截断/超长内容属于线上协议错误，不是调用参数错误。 */
        return ZDT_TURNTABLE_ERR_PROTOCOL;
    }
    if ((data[0] != expected_address) || (data[1] != expected_function) ||
        (data[len - 1U] != ZDT_TURNTABLE_CHECK_BYTE)) {
        return ZDT_TURNTABLE_ERR_PROTOCOL;
    }
    (void)memset(response, 0, sizeof(*response));
    response->address = data[0];
    response->function = data[1];
#if !LICANG_RELEASE_MINIMAL
    response->raw_len = len;
    (void)memcpy(response->raw, data, len);
#endif

    if (expected_function == ZDT_CODE_OPTIONS) {
        /* 4字节E2/EE仍是合法设备错误；成功响应必须严格为5字节。 */
        if ((len == 4U) && ((data[2] == 0xE2U) || (data[2] == 0xEEU))) {
            response->kind = (data[2] == 0xE2U) ?
                ZDT_TURNTABLE_REPLY_PARAM_ERROR :
                ZDT_TURNTABLE_REPLY_FORMAT_ERROR;
            return ZDT_TURNTABLE_ERR_DEVICE;
        }
        if (len != 5U) {
            return ZDT_TURNTABLE_ERR_PROTOCOL;
        }
        flags = zdt_read_u16_be(&data[2]);
        response->kind = ZDT_TURNTABLE_REPLY_OPTIONS;
        response->data.options.raw_flags = flags;
        response->data.options.motor_step_angle_0p9deg =
            ((flags & 0x0001U) != 0U);
        response->data.options.firmware = ((flags & 0x0002U) != 0U) ?
            ZDT_TURNTABLE_FIRMWARE_EMM : ZDT_TURNTABLE_FIRMWARE_X;
        response->data.options.closed_loop = ((flags & 0x0004U) != 0U);
        response->data.options.positive_direction_ccw =
            ((flags & 0x0010U) != 0U);
        response->data.options.buttons_locked = ((flags & 0x0020U) != 0U);
        response->data.options.scaled_input = ((flags & 0x0080U) != 0U);
        response->data.options.parameter_lock_level =
            (uint8_t)((flags >> 8U) & 0x03U);
        return ZDT_TURNTABLE_OK;
    }

    if (expected_function == ZDT_CODE_STATUS) {
        if (len != 4U) {
            return ZDT_TURNTABLE_ERR_PROTOCOL;
        }
        value = data[2];
        /* bit6是保留位；E2/EE必须在状态位解释之前识别为设备错误。 */
        if ((value == 0xE2U) || (value == 0xEEU)) {
            response->kind = (value == 0xE2U) ?
                ZDT_TURNTABLE_REPLY_PARAM_ERROR :
                ZDT_TURNTABLE_REPLY_FORMAT_ERROR;
            return ZDT_TURNTABLE_ERR_DEVICE;
        }
        if ((value & 0x40U) != 0U) {
            return ZDT_TURNTABLE_ERR_PROTOCOL;
        }
        response->kind = ZDT_TURNTABLE_REPLY_STATUS;
        response->data.motor_status.enabled = ((value & 0x01U) != 0U);
        response->data.motor_status.reached = ((value & 0x02U) != 0U);
        response->data.motor_status.stalled = ((value & 0x04U) != 0U);
        response->data.motor_status.stall_protected = ((value & 0x08U) != 0U);
        response->data.motor_status.left_limit_high = ((value & 0x10U) != 0U);
        response->data.motor_status.right_limit_high = ((value & 0x20U) != 0U);
        response->data.motor_status.power_loss_latched = ((value & 0x80U) != 0U);
        return ZDT_TURNTABLE_OK;
    }

#if !LICANG_RELEASE_MINIMAL
    if (expected_function == ZDT_CODE_REAL_POSITION) {
        if ((len == 4U) && ((data[2] == 0xE2U) || (data[2] == 0xEEU))) {
            response->kind = (data[2] == 0xE2U) ?
                ZDT_TURNTABLE_REPLY_PARAM_ERROR :
                ZDT_TURNTABLE_REPLY_FORMAT_ERROR;
            return ZDT_TURNTABLE_ERR_DEVICE;
        }
        if ((len != 8U) || (data[2] > 1U)) {
            return ZDT_TURNTABLE_ERR_PROTOCOL;
        }
        response->kind = ZDT_TURNTABLE_REPLY_POSITION;
        response->data.position.negative = (data[2] != 0U);
        response->data.position.magnitude = zdt_read_u32_be(&data[3]);
        return ZDT_TURNTABLE_OK;
    }

    if (expected_function == ZDT_CODE_VERSION) {
        if ((len == 4U) && ((data[2] == 0xE2U) || (data[2] == 0xEEU))) {
            response->kind = (data[2] == 0xE2U) ?
                ZDT_TURNTABLE_REPLY_PARAM_ERROR :
                ZDT_TURNTABLE_REPLY_FORMAT_ERROR;
            return ZDT_TURNTABLE_ERR_DEVICE;
        }
        if (len != 7U) {
            return ZDT_TURNTABLE_ERR_PROTOCOL;
        }
        response->kind = ZDT_TURNTABLE_REPLY_VERSION;
        response->data.version.firmware_version =
            ((uint16_t)data[2] << 8U) | data[3];
        response->data.version.hardware_series = data[4] >> 4U;
        response->data.version.hardware_type = data[4] & 0x0FU;
        response->data.version.hardware_version = data[5];
        return ZDT_TURNTABLE_OK;
    }
#endif

    if ((expected_function == ZDT_CODE_POSITION) && (len == 4U)) {
        switch (data[2]) {
        case 0x02U: response->kind = ZDT_TURNTABLE_REPLY_ACK; return ZDT_TURNTABLE_OK;
        case 0xE2U: response->kind = ZDT_TURNTABLE_REPLY_PARAM_ERROR; return ZDT_TURNTABLE_ERR_DEVICE;
        case 0xEEU: response->kind = ZDT_TURNTABLE_REPLY_FORMAT_ERROR; return ZDT_TURNTABLE_ERR_DEVICE;
        case 0x9FU: response->kind = ZDT_TURNTABLE_REPLY_REACHED; return ZDT_TURNTABLE_OK;
        default: return ZDT_TURNTABLE_ERR_PROTOCOL;
        }
    }
    if ((expected_function == ZDT_CODE_STOP) && (len == 4U)) {
        switch (data[2]) {
        case 0x02U: response->kind = ZDT_TURNTABLE_REPLY_ACK; return ZDT_TURNTABLE_OK;
        case 0xE2U: response->kind = ZDT_TURNTABLE_REPLY_PARAM_ERROR; return ZDT_TURNTABLE_ERR_DEVICE;
        case 0xEEU: response->kind = ZDT_TURNTABLE_REPLY_FORMAT_ERROR; return ZDT_TURNTABLE_ERR_DEVICE;
        default: return ZDT_TURNTABLE_ERR_PROTOCOL;
        }
    }
    return ZDT_TURNTABLE_ERR_PROTOCOL;
}

#if TURN_BSP_HAL_ENABLE
static zdt_turntable_status_t zdt_map_hal(HAL_StatusTypeDef status)
{
    if (status == HAL_OK) return ZDT_TURNTABLE_OK;
    if (status == HAL_BUSY) return ZDT_TURNTABLE_ERR_BUSY;
    if (status == HAL_TIMEOUT) return ZDT_TURNTABLE_ERR_TIMEOUT;
    return ZDT_TURNTABLE_ERR_IO;
}

zdt_turntable_status_t turn_bsp_tx(
    void *ctx, const uint8_t *data, size_t len)
{
    (void)ctx;
    if ((data == NULL) || (len == 0U) || (len > UINT16_MAX)) {
        return ZDT_TURNTABLE_ERR_PARAM;
    }
    return zdt_map_hal(HAL_UART_Transmit_IT(&huart7, data, (uint16_t)len));
}

zdt_turntable_status_t turn_bsp_rx(
    void *ctx, uint8_t *data, size_t capacity)
{
    HAL_StatusTypeDef status;

    (void)ctx;
    if ((data == NULL) || (capacity == 0U) || (capacity > UINT16_MAX)) {
        return ZDT_TURNTABLE_ERR_PARAM;
    }
    status = HAL_UARTEx_ReceiveToIdle_DMA(
        &huart7, data, (uint16_t)capacity);
    if ((status == HAL_OK) && (huart7.hdmarx != NULL)) {
        __HAL_DMA_DISABLE_IT(huart7.hdmarx, DMA_IT_HT);
    }
    return zdt_map_hal(status);
}

zdt_turntable_status_t turn_bsp_abort(void *ctx)
{
    HAL_StatusTypeDef status;

    (void)ctx;
    status = HAL_UART_Abort(&huart7);
    if (status == HAL_OK) {
        __HAL_UART_CLEAR_PEFLAG(&huart7);
        HAL_NVIC_ClearPendingIRQ(UART7_IRQn);
        HAL_NVIC_ClearPendingIRQ(DMA1_Stream3_IRQn);
    }
    return zdt_map_hal(status);
}

bool turn_bsp_owns(const UART_HandleTypeDef *huart)
{
    return huart == &huart7;
}
#endif
