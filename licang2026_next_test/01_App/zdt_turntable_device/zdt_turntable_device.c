/** @file zdt_turntable_device.c @brief ZDT转盘语义接口实现。 */

#include "zdt_turntable_device.h"

#include <string.h>

/**
 * @brief 生成非零、单调递增的本地请求标识。
 * @param device 设备对象。
 * @return 新请求ID；uint32_t回绕时跳过0。
 */
static uint32_t zdt_next_id(zdt_turntable_device_t *device)
{
    device->next_request_id++;
    if (device->next_request_id == 0U) {
        device->next_request_id++;
    }
    return device->next_request_id;
}

/**
 * @brief 将已构造协议帧封装为Service事务并按值提交。
 * @param device 已初始化设备对象。
 * @param function 期望响应功能码。
 * @param frame 待复制协议帧。
 * @param frame_len 有效帧长。
 * @param done_cb 可选完成回调。
 * @param user_ctx 回调上下文。
 * @return Service入队结果。
 * @note request.frame在栈上创建，但submit会再次按值复制到Service队列。
 */
static zdt_turntable_status_t zdt_submit_frame(
    zdt_turntable_device_t *device,
    uint8_t function,
    const uint8_t *frame,
    size_t frame_len,
    zdt_turntable_done_fn_t done_cb,
    void *user_ctx)
{
    zdt_turntable_request_t request;

    if ((device == NULL) || !device->initialized || (frame == NULL) ||
        (frame_len > sizeof(request.frame))) {
        return ZDT_TURNTABLE_ERR_PARAM;
    }
    (void)memset(&request, 0, sizeof(request));
    request.request_id = zdt_next_id(device);
    (void)memcpy(request.frame, frame, frame_len);
    request.frame_len = frame_len;
    request.expected_address = device->config.address;
    request.expected_function = function;
    request.timeout_ms = device->config.timeout_ms;
    request.done_cb = done_cb;
    request.user_ctx = user_ctx;
    return device->submit_fn(device->submit_ctx, &request);
}

/** @copydoc zdt_turntable_device_init_with_submit() */
zdt_turntable_status_t zdt_turntable_device_init_with_submit(
    zdt_turntable_device_t *device,
    zdt_turntable_device_submit_fn_t submit_fn,
    void *submit_ctx,
    const zdt_turntable_device_config_t *config)
{
    if ((device == NULL) || (submit_fn == NULL) || (config == NULL) ||
        (config->address == 0U) || (config->timeout_ms == 0U) ||
        (config->emm_pulses_per_revolution == 0U)) {
        return ZDT_TURNTABLE_ERR_PARAM;
    }
    (void)memset(device, 0, sizeof(*device));
    device->submit_fn = submit_fn;
    device->submit_ctx = submit_ctx;
    device->config = *config;
    device->initialized = true;
    return ZDT_TURNTABLE_OK;
}

/**
 * @brief 处理0x1A查询完成，原子更新固件能力后转发用户回调。
 * @param ctx 设备对象。
 * @param request_id 原请求ID。
 * @param status 事务状态。
 * @param response 可选解析响应。
 * @note 失败不会覆盖上一次已确认的固件能力；pending状态始终释放。
 */
static void zdt_options_done(
    void *ctx,
    uint32_t request_id,
    zdt_turntable_status_t status,
    const zdt_turntable_response_t *response)
{
    zdt_turntable_device_t *device = (zdt_turntable_device_t *)ctx;
    zdt_turntable_done_fn_t user_done;
    void *user_ctx;

    if (device == NULL) {
        return;
    }
    user_done = device->options_done_cb;
    user_ctx = device->options_user_ctx;
    device->options_done_cb = NULL;
    device->options_user_ctx = NULL;
    device->options_pending = false;
    if ((status == ZDT_TURNTABLE_OK) &&
        (response != NULL) &&
        (response->kind == ZDT_TURNTABLE_REPLY_OPTIONS)) {
        device->firmware = response->data.options.firmware;
        device->closed_loop = response->data.options.closed_loop;
        device->scaled_input = response->data.options.scaled_input;
        device->firmware_known = true;
    }
    if (user_done != NULL) {
        user_done(user_ctx, request_id, status, response);
    }
}

/** @copydoc zdt_turntable_device_query_options() */
zdt_turntable_status_t zdt_turntable_device_query_options(
    zdt_turntable_device_t *device,
    zdt_turntable_done_fn_t done_cb,
    void *user_ctx)
{
    uint8_t frame[3];
    size_t frame_len;
    zdt_turntable_status_t status;

    if ((device == NULL) || !device->initialized) {
        return ZDT_TURNTABLE_ERR_NOT_INIT;
    }
    if (device->options_pending) {
        return ZDT_TURNTABLE_ERR_BUSY;
    }
    status = turn_options_frame(
        device->config.address, frame, sizeof(frame), &frame_len);
    if (status != ZDT_TURNTABLE_OK) {
        return status;
    }
    device->options_pending = true;
    device->options_done_cb = done_cb;
    device->options_user_ctx = user_ctx;
    status = zdt_submit_frame(
        device, 0x1AU, frame, frame_len, zdt_options_done, device);
    if (status != ZDT_TURNTABLE_OK) {
        /* 入队失败必须回滚代理回调，允许调用者稍后重试。 */
        device->options_pending = false;
        device->options_done_cb = NULL;
        device->options_user_ctx = NULL;
    }
    return status;
}

/** @copydoc zdt_turntable_device_query_status() */
zdt_turntable_status_t zdt_turntable_device_query_status(
    zdt_turntable_device_t *device,
    zdt_turntable_done_fn_t done_cb,
    void *user_ctx)
{
    uint8_t frame[3];
    size_t frame_len;
    zdt_turntable_status_t status;

    if ((device == NULL) || !device->initialized) {
        return ZDT_TURNTABLE_ERR_NOT_INIT;
    }
    status = turn_status_frame(
        device->config.address, frame, sizeof(frame), &frame_len);
    return (status == ZDT_TURNTABLE_OK) ?
        zdt_submit_frame(device, 0x3AU, frame, frame_len, done_cb, user_ctx) :
        status;
}

/** @copydoc zdt_turntable_device_query() */
#if !LICANG_RELEASE_MINIMAL
zdt_turntable_status_t zdt_turntable_device_query(
    zdt_turntable_device_t *device,
    uint8_t function,
    zdt_turntable_done_fn_t done_cb,
    void *user_ctx)
{
    uint8_t frame[3];
    size_t frame_len;
    zdt_turntable_status_t status;

    if ((device == NULL) || !device->initialized) {
        return ZDT_TURNTABLE_ERR_NOT_INIT;
    }
    if (function == 0x1FU) {
        status = turn_version_frame(
            device->config.address, frame, sizeof(frame), &frame_len);
    } else if (function == 0x3AU) {
        status = turn_status_frame(
            device->config.address, frame, sizeof(frame), &frame_len);
    } else if (function == 0x36U) {
        status = turn_position_frame(
            device->config.address, frame, sizeof(frame), &frame_len);
    } else {
        return ZDT_TURNTABLE_ERR_UNSUPPORTED;
    }
    return (status == ZDT_TURNTABLE_OK) ?
        zdt_submit_frame(device, function, frame, frame_len, done_cb, user_ctx) :
        status;
}
#endif

/** @copydoc zdt_turntable_device_move_emm_angle() */
zdt_turntable_status_t zdt_turntable_device_move_emm_angle(
    zdt_turntable_device_t *device,
    const zdt_turntable_position_command_t *command,
    zdt_turntable_done_fn_t done_cb,
    void *user_ctx)
{
    uint8_t frame[ZDT_TURNTABLE_FRAME_MAX];
    size_t frame_len;
    uint64_t pulse_numerator;
    uint64_t pulse_result;
    uint32_t pulses;
    zdt_turntable_status_t status;
    zdt_turntable_position_command_t adjusted;

    if ((device == NULL) || !device->initialized) {
        return ZDT_TURNTABLE_ERR_NOT_INIT;
    }
    if ((command == NULL) || !device->firmware_known ||
        !device->closed_loop ||
        (device->firmware != ZDT_TURNTABLE_FIRMWARE_EMM)) {
        return (command == NULL) ? ZDT_TURNTABLE_ERR_PARAM :
            ZDT_TURNTABLE_ERR_STATE;
    }
    adjusted = *command;
    if (device->scaled_input) {
        if (adjusted.speed > (3000U / 10U)) {
            return ZDT_TURNTABLE_ERR_PARAM;
        }
        adjusted.speed = (uint16_t)(adjusted.speed * 10U);
    }
    pulse_numerator = (uint64_t)command->angle_0p1deg *
                      device->config.emm_pulses_per_revolution;
    pulse_result = pulse_numerator / 3600U;
    if ((pulse_numerator % 3600U) >= 1800U) {
        pulse_result++;
    }
    if (pulse_result > UINT32_MAX) {
        return ZDT_TURNTABLE_ERR_PARAM;
    }
    pulses = (uint32_t)pulse_result;
    status = turn_emm_frame(
        device->config.address,
        &adjusted,
        pulses,
        frame,
        sizeof(frame),
        &frame_len);
    return (status == ZDT_TURNTABLE_OK) ?
        zdt_submit_frame(device, 0xFDU, frame, frame_len, done_cb, user_ctx) :
        status;
}

/** @copydoc zdt_turntable_device_move_angle() */
#if !LICANG_RELEASE_MINIMAL
zdt_turntable_status_t zdt_turntable_device_move_angle(
    zdt_turntable_device_t *device,
    const zdt_turntable_position_command_t *command,
    zdt_turntable_done_fn_t done_cb,
    void *user_ctx)
{
    uint8_t frame[ZDT_TURNTABLE_FRAME_MAX];
    size_t frame_len;
    uint64_t pulse_numerator;
    uint64_t pulse_result;
    uint32_t pulses;
    zdt_turntable_status_t status;
    zdt_turntable_position_command_t adjusted;

    if ((device == NULL) || !device->initialized) {
        return ZDT_TURNTABLE_ERR_NOT_INIT;
    }
    if (!device->firmware_known) {
        return ZDT_TURNTABLE_ERR_STATE;
    }
    if ((command == NULL) || !device->closed_loop) {
        return (command == NULL) ? ZDT_TURNTABLE_ERR_PARAM :
            ZDT_TURNTABLE_ERR_STATE;
    }
    adjusted = *command;
    if (device->firmware == ZDT_TURNTABLE_FIRMWARE_X) {
        if (device->scaled_input) {
            /* X固件缩小10倍输入时，保持上层0.1度语义不变。 */
            if (adjusted.angle_0p1deg > (UINT32_MAX / 10U)) {
                return ZDT_TURNTABLE_ERR_PARAM;
            }
            adjusted.angle_0p1deg *= 10U;
        }
        status = turn_x_frame(
            device->config.address, &adjusted, frame, sizeof(frame), &frame_len);
    } else {
        if (device->scaled_input) {
            /* Emm Scale使协议速度单位变为0.1RPM；上层接口继续使用整数RPM。 */
            if (adjusted.speed > (3000U / 10U)) {
                return ZDT_TURNTABLE_ERR_PARAM;
            }
            adjusted.speed = (uint16_t)(adjusted.speed * 10U);
        }
        /*
         * 四舍五入换算：0.1度 * 每圈脉冲 / 3600。先分解商和余数，
         * 避免在接近uint64_t上限时由“分子+1800”再次产生溢出。
         */
        pulse_numerator = (uint64_t)command->angle_0p1deg *
                          device->config.emm_pulses_per_revolution;
        pulse_result = pulse_numerator / 3600U;
        if ((pulse_numerator % 3600U) >= 1800U) {
            pulse_result++;
        }
        if (pulse_result > UINT32_MAX) {
            return ZDT_TURNTABLE_ERR_PARAM;
        }
        pulses = (uint32_t)pulse_result;
        status = turn_emm_frame(
            device->config.address,
            &adjusted,
            pulses,
            frame,
            sizeof(frame),
            &frame_len);
    }
    return (status == ZDT_TURNTABLE_OK) ?
        zdt_submit_frame(device, 0xFDU, frame, frame_len, done_cb, user_ctx) :
        status;
}
#endif

/** @copydoc zdt_turntable_device_stop() */
zdt_turntable_status_t zdt_turntable_device_stop(
    zdt_turntable_device_t *device,
    zdt_turntable_done_fn_t done_cb,
    void *user_ctx)
{
    uint8_t frame[5];
    size_t frame_len;
    zdt_turntable_status_t status;

    if ((device == NULL) || !device->initialized) {
        return ZDT_TURNTABLE_ERR_NOT_INIT;
    }
    status = turn_stop_frame(
        device->config.address, frame, sizeof(frame), &frame_len);
    return (status == ZDT_TURNTABLE_OK) ?
        zdt_submit_frame(device, 0xFEU, frame, frame_len, done_cb, user_ctx) :
        status;
}
