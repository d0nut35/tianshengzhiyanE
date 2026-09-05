/** @file zdt_turntable_device.h @brief 上层转盘电机语义接口。 */

#ifndef ZDT_TURNTABLE_DEVICE_H
#define ZDT_TURNTABLE_DEVICE_H

#ifdef __cplusplus
extern "C" {
#endif

#include "zdt_turntable_service.h"

#ifndef LICANG_RELEASE_MINIMAL
#define LICANG_RELEASE_MINIMAL 0
#endif

/** 设备地址、事务超时和Emm角度换算参数。 */
typedef struct {
    uint8_t address;
    uint32_t timeout_ms;
    uint32_t emm_pulses_per_revolution;
} zdt_turntable_device_config_t;

/**
 * @brief Device提交一笔ZDT事务的抽象接口。
 * @param submit_ctx 初始化时绑定的事务链路上下文。
 * @param request 待提交事务；实现必须在返回前复制请求和frame。
 * @return 提交结果。
 */
typedef zdt_turntable_status_t (*zdt_turntable_device_submit_fn_t)(
    void *submit_ctx,
    const zdt_turntable_request_t *request);

/**
 * @brief 转盘电机语义对象。
 *
 * firmware/closed_loop/scaled_input来自最近一次成功0x1A查询；在确认这些
 * 选项前禁止生成运动帧，避免用错误固件格式驱动真实电机。
 * scaled_input的协议含义随固件变化：X固件表示位置角度精度变为0.01度，
 * Emm固件表示速度精度变为0.1RPM，Device负责维持上层原有单位语义。
 */
typedef struct {
    bool initialized;
    bool firmware_known;
    zdt_turntable_firmware_t firmware;
    bool closed_loop;
    bool scaled_input;
    bool options_pending;
    zdt_turntable_device_submit_fn_t submit_fn;
    void *submit_ctx;
    zdt_turntable_device_config_t config;
    uint32_t next_request_id;
    zdt_turntable_done_fn_t options_done_cb;
    void *options_user_ctx;
} zdt_turntable_device_t;

/**
 * @brief 使用抽象事务提交函数初始化转盘Device。
 * @param device 待初始化对象。
 * @param submit_fn 已装配链路的非阻塞提交函数。
 * @param submit_ctx 原样传给submit_fn的上下文。
 * @param config 电机地址、超时和Emm每圈脉冲数。
 * @return 初始化结果。
 * @note 语义层无需知道复用通道、RTOS或HAL细节。
 */
zdt_turntable_status_t zdt_turntable_device_init_with_submit(
    zdt_turntable_device_t *device,
    zdt_turntable_device_submit_fn_t submit_fn,
    void *submit_ctx,
    const zdt_turntable_device_config_t *config);

/**
 * @brief 读取0x1A并在成功回调前缓存X/Emm固件类型。
 * @param device 已初始化设备对象。
 * @param done_cb 可选异步完成回调。
 * @param user_ctx 原样交给done_cb的调用者上下文。
 * @return OK仅表示查询已排队。
 * @note 同一设备只允许一笔选项查询处于等待或在途状态。
 */
zdt_turntable_status_t zdt_turntable_device_query_options(
    zdt_turntable_device_t *device,
    zdt_turntable_done_fn_t done_cb,
    void *user_ctx);

/** 查询正式任务用于到位和故障判断的0x3A状态。 */
zdt_turntable_status_t zdt_turntable_device_query_status(
    zdt_turntable_device_t *device,
    zdt_turntable_done_fn_t done_cb,
    void *user_ctx);

/**
 * @brief 查询版本、状态或实时位置，只接受0x1F/0x3A/0x36。
 * @param device 已初始化设备对象。
 * @param function 查询功能码。
 * @param done_cb 可选异步完成回调。
 * @param user_ctx 原样交给done_cb的调用者上下文。
 * @return OK仅表示查询已排队；不支持的功能码返回ERR_UNSUPPORTED。
 */
#if !LICANG_RELEASE_MINIMAL
zdt_turntable_status_t zdt_turntable_device_query(
    zdt_turntable_device_t *device,
    uint8_t function,
    zdt_turntable_done_fn_t done_cb,
    void *user_ctx);
#endif

/**
 * @brief 按已确认的Emm固件发送角度位置命令。
 * @note 正式任务只使用实机确认过的Emm路径，避免把X固件构帧带入固件。
 */
zdt_turntable_status_t zdt_turntable_device_move_emm_angle(
    zdt_turntable_device_t *device,
    const zdt_turntable_position_command_t *command,
    zdt_turntable_done_fn_t done_cb,
    void *user_ctx);

/**
 * @brief 按已识别固件发送固定角度位置命令。
 * @param device 已完成0x1A识别的设备对象。
 * @param command 统一角度、方向、模式和固件相关速度参数。
 * @param done_cb 可选异步完成回调。
 * @param user_ctx 原样交给done_cb的调用者上下文。
 * @return OK仅表示位置事务已排队。
 * @note 返回OK只表示请求排队；默认Response=Receive时完成回调的ACK也只代表
 *       电机接收命令，必须再查询0x3A/0x36或观察0x9F确认到位。
 */
#if !LICANG_RELEASE_MINIMAL
zdt_turntable_status_t zdt_turntable_device_move_angle(
    zdt_turntable_device_t *device,
    const zdt_turntable_position_command_t *command,
    zdt_turntable_done_fn_t done_cb,
    void *user_ctx);
#endif

/**
 * @brief 提交立即停止命令。
 * @param device 已初始化设备对象。
 * @param done_cb 可选异步完成回调。
 * @param user_ctx 原样交给done_cb的调用者上下文。
 * @return OK仅表示停止事务已排队。
 * @warning 本接口仍经过Service FIFO和UART应答事务，不是硬实时急停或安全回路；
 *          人身/机械风险必须依靠独立断电、驱动使能或硬件急停处理。
 */
zdt_turntable_status_t zdt_turntable_device_stop(
    zdt_turntable_device_t *device,
    zdt_turntable_done_fn_t done_cb,
    void *user_ctx);

#ifdef __cplusplus
}
#endif

#endif /* ZDT_TURNTABLE_DEVICE_H */
