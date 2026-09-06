/**
 * @file    arm_bsp.c
 * @brief   LSC16协议编解码与STM32 UART8 HAL/DMA实现。
 */

#include "arm_bsp.h"

#include <string.h>

#include "usart.h"

#define LSC16_FRAME_HEADER               0x55U
#define LSC16_CMD_SERVO_MOVE            0x03U
#define LSC16_CMD_ACTION_GROUP_RUN       0x06U
#define LSC16_CMD_ACTION_GROUP_STOP      0x07U
#define LSC16_CMD_ACTION_GROUP_COMPLETE  0x08U
#define LSC16_CMD_ACTION_GROUP_SPEED     0x0BU
#define LSC16_CMD_GET_BATTERY_VOLTAGE   0x0FU

/** 将HAL状态映射为机械臂模块状态。 */
static lsc16_status_t lsc16_map_hal(HAL_StatusTypeDef status)
{
    if (status == HAL_OK) return LSC16_OK;
    return (status == HAL_BUSY) ? LSC16_ERR_BUSY : LSC16_ERR_IO;
}

/** 通过固定UART8启动ReceiveToIdle DMA，并关闭半传输中断。 */
static lsc16_status_t lsc16_start_rx(lsc16_t *device)
{
    HAL_StatusTypeDef status;

    status = HAL_UARTEx_ReceiveToIdle_DMA(
        &huart8,
        device->dma_rx_buffer,
        (uint16_t)sizeof(device->dma_rx_buffer));
    if ((status == HAL_OK) && (huart8.hdmarx != NULL)) {
        __HAL_DMA_DISABLE_IT(huart8.hdmarx, DMA_IT_HT);
    }
    return lsc16_map_hal(status);
}

/**
 * @brief 计算RX环形缓冲区的下一个索引。
 * @param index 当前索引。
 * @return 回绕后的下一索引。
 */
static uint16_t lsc16_next_ring_index(uint16_t index)
{
    ++index;
    return (index >= LSC16_RX_RING_BUFFER_SIZE) ? 0U : index;
}

/**
 * @brief 在ISR中向上层发布轻量事件。
 * @param device LSC16 Core对象。
 * @param event TX完成、RX就绪或错误事件。
 * @warning 用户回调运行在ISR上下文，必须保持轻量且不能阻塞。
 */
static void lsc16_notify_isr(lsc16_t *device, lsc16_isr_event_t event)
{
    if (device->notify_cb != NULL) {
        device->notify_cb(device->notify_ctx, event);
    }
}

/**
 * @brief 重新挂接一次ReceiveToIdle DMA接收。
 * @param device LSC16 Core对象。
 * @return port层启动结果。
 */
static lsc16_status_t lsc16_restart_rx(lsc16_t *device)
{
    return (device == NULL) ? LSC16_ERR_PARAM : lsc16_start_rx(device);
}

/**
 * @brief 构造并异步发送一帧LSC16命令。
 * @param device LSC16 Core对象。
 * @param command 协议命令号。
 * @param parameters 参数数组；无参数时允许为NULL。
 * @param parameter_len 参数字节数。
 * @return 参数校验和DMA启动结果。
 * @note tx_busy在启动DMA前发布，失败时回滚，防止极短传输完成竞态。
 */
static lsc16_status_t lsc16_send_frame(
    lsc16_t *device,
    uint8_t command,
    const uint8_t *parameters,
    uint8_t parameter_count)
{
    size_t frame_size = (size_t)parameter_count + 4U;
    lsc16_status_t status;

    if ((device == NULL) || !device->initialized) {
        return (device == NULL) ? LSC16_ERR_PARAM : LSC16_ERR_NOT_INIT;
    }
    if ((frame_size > sizeof(device->tx_buffer)) ||
        ((parameter_count > 0U) && (parameters == NULL))) {
        return LSC16_ERR_PARAM;
    }
    if (device->tx_busy) {
        return LSC16_ERR_BUSY;
    }

    device->tx_buffer[0] = LSC16_FRAME_HEADER;
    device->tx_buffer[1] = LSC16_FRAME_HEADER;
    /* Length包含自身、Cmd和全部参数，不包含两个0x55帧头。 */
    device->tx_buffer[2] = (uint8_t)(parameter_count + 2U);
    device->tx_buffer[3] = command;
    if (parameter_count > 0U) {
        (void)memcpy(&device->tx_buffer[4], parameters, parameter_count);
    }

    /* 先置忙再启动异步发送，防止极短传输完成回调抢在状态发布之前。 */
    device->tx_busy = true;
    status = lsc16_map_hal(HAL_UART_Transmit_IT(
        &huart8,
        device->tx_buffer,
        (uint16_t)frame_size));
    if (status != LSC16_OK) {
        device->tx_busy = false;
    }
    return status;
}

/**
 * @brief 解释一帧完整控制板主动回报并更新事件位。
 * @param device LSC16 Core对象。
 * @param frame 已完成长度检查的帧。
 */
static void lsc16_handle_frame(lsc16_t *device, const uint8_t *frame)
{
    uint8_t length = frame[2];
    uint8_t command = frame[3];

    device->last_report.command = command;
    switch (command) {
    case LSC16_CMD_ACTION_GROUP_RUN:
        if (length == 5U) {
            device->last_report.action_group = frame[4];
            device->last_report.repeat_count =
                (uint16_t)frame[5] | ((uint16_t)frame[6] << 8);
            device->report_events |= LSC16_REPORT_EVENT_ACTION_STARTED;
        } else {
            device->report_events |= LSC16_REPORT_EVENT_INVALID_FRAME;
        }
        break;
    case LSC16_CMD_ACTION_GROUP_STOP:
        if (length == 2U) {
            device->report_events |= LSC16_REPORT_EVENT_ACTION_STOPPED;
        } else {
            device->report_events |= LSC16_REPORT_EVENT_INVALID_FRAME;
        }
        break;
    case LSC16_CMD_ACTION_GROUP_COMPLETE:
        if (length == 5U) {
            device->last_report.action_group = frame[4];
            device->last_report.repeat_count =
                (uint16_t)frame[5] | ((uint16_t)frame[6] << 8);
            device->report_events |= LSC16_REPORT_EVENT_ACTION_COMPLETED;
        } else {
            device->report_events |= LSC16_REPORT_EVENT_INVALID_FRAME;
        }
        break;
    case LSC16_CMD_GET_BATTERY_VOLTAGE:
        if (length == 4U) {
            device->last_report.battery_mv =
                (uint16_t)frame[4] | ((uint16_t)frame[5] << 8);
            device->report_events |= LSC16_REPORT_EVENT_BATTERY_UPDATED;
        } else {
            device->report_events |= LSC16_REPORT_EVENT_INVALID_FRAME;
        }
        break;
    default:
        /* 未知命令不等于坏帧：协议可能在新版控制板中继续扩展。 */
        break;
    }
}

/**
 * @brief 向55 55帧解析状态机投入一个字节。
 * @param device LSC16 Core对象。
 * @param data 新收到的字节。
 * @note 噪声、非法长度或坏帧会重新同步，不把无效内容上报Service。
 */
static void lsc16_parse_byte(lsc16_t *device, uint8_t data)
{
    uint16_t expected_size;

    if (device->rx_frame_count == 0U) {
        if (data == LSC16_FRAME_HEADER) {
            device->rx_frame[0] = data;
            device->rx_frame_count = 1U;
        }
        return;
    }
    if (device->rx_frame_count == 1U) {
        if (data == LSC16_FRAME_HEADER) {
            device->rx_frame[1] = data;
            device->rx_frame_count = 2U;
        } else {
            device->rx_frame_count = 0U;
        }
        return;
    }
    if (device->rx_frame_count == 2U) {
        if ((data < 2U) || (((uint16_t)data + 2U) > sizeof(device->rx_frame))) {
            device->report_events |= LSC16_REPORT_EVENT_INVALID_FRAME;
            device->rx_frame_count = (data == LSC16_FRAME_HEADER) ? 1U : 0U;
            return;
        }
    }

    device->rx_frame[device->rx_frame_count++] = data;
    expected_size = (uint16_t)device->rx_frame[2] + 2U;
    if (device->rx_frame_count == expected_size) {
        lsc16_handle_frame(device, device->rx_frame);
        device->rx_frame_count = 0U;
    }
}

/**
 * @brief 初始化LSC16协议Core并启动首次DMA接收。
 * @param device 待初始化Core对象。
 * @param port 已绑定硬件上下文的异步串口能力。
 * @return 初始化结果；首次RX失败时完整清空对象。
 */
lsc16_status_t lsc16_init(lsc16_t *device)
{
    lsc16_status_t status;

    if (device == NULL) {
        return LSC16_ERR_PARAM;
    }

    (void)memset(device, 0, sizeof(*device));
    device->initialized = true;
    status = lsc16_restart_rx(device);
    if (status != LSC16_OK) {
        (void)memset(device, 0, sizeof(*device));
    }
    return status;
}

/**
 * @brief 同步终止底层收发并反初始化Core。
 * @param device Core对象。
 * @return abort成功后返回LSC16_OK，否则保留状态供诊断。
 */
lsc16_status_t lsc16_deinit(lsc16_t *device)
{
    lsc16_status_t status;

    if (device == NULL) {
        return LSC16_ERR_PARAM;
    }
    if (!device->initialized) {
        return LSC16_ERR_NOT_INIT;
    }
    status = lsc16_map_hal(HAL_UART_Abort(&huart8));
    if (status == LSC16_OK) {
        (void)memset(device, 0, sizeof(*device));
    }
    return status;
}

/**
 * @brief 绑定或清除Core ISR轻量通知出口。
 * @param device Core对象。
 * @param notify_cb ISR事件回调；传NULL表示清除。
 * @param user_ctx 原样传给notify_cb的上下文。
 * @return 绑定结果。
 */
lsc16_status_t lsc16_bind_isr_notify(
    lsc16_t *device,
    lsc16_isr_notify_fn_t notify_cb,
    void *user_ctx)
{
    if ((device == NULL) || !device->initialized) {
        return (device == NULL) ? LSC16_ERR_PARAM : LSC16_ERR_NOT_INIT;
    }
    device->notify_cb = notify_cb;
    device->notify_ctx = user_ctx;
    return LSC16_OK;
}

/**
 * @brief 构造0x03帧并异步驱动1~16个舵机。
 * @param device Core对象。
 * @param targets 舵机目标数组。
 * @param servo_count 数组元素个数。
 * @param move_time_ms 共用运动时间，单位ms。
 * @return 参数校验和DMA启动结果。
 */
lsc16_status_t lsc16_move_servos(
    lsc16_t *device,
    const lsc16_servo_target_t *targets,
    uint8_t servo_count,
    uint16_t move_time_ms)
{
    uint8_t parameters[3U + (LSC16_SERVO_COUNT_MAX * 3U)];
    uint8_t i;
    uint8_t offset;

    if ((targets == NULL) || (servo_count == 0U) ||
        (servo_count > LSC16_SERVO_COUNT_MAX)) {
        return LSC16_ERR_PARAM;
    }
    parameters[0] = servo_count;
    parameters[1] = (uint8_t)(move_time_ms & 0xFFU);
    parameters[2] = (uint8_t)(move_time_ms >> 8);
    for (i = 0U; i < servo_count; ++i) {
        if ((targets[i].id > LSC16_SERVO_ID_MAX) ||
            (targets[i].position < LSC16_SERVO_POSITION_MIN) ||
            (targets[i].position > LSC16_SERVO_POSITION_MAX)) {
            return LSC16_ERR_PARAM;
        }
        offset = (uint8_t)(3U + (i * 3U));
        parameters[offset] = targets[i].id;
        parameters[offset + 1U] = (uint8_t)(targets[i].position & 0xFFU);
        parameters[offset + 2U] = (uint8_t)(targets[i].position >> 8);
    }
    return lsc16_send_frame(
        device,
        LSC16_CMD_SERVO_MOVE,
        parameters,
        (uint8_t)(3U + (servo_count * 3U)));
}

/**
 * @brief 构造0x06运行动作组命令。
 * @param device Core对象。
 * @param action_group 动作组编号。
 * @param repeat_count 重复次数，0表示循环执行。
 * @return DMA启动结果。
 */
lsc16_status_t lsc16_run_action_group(
    lsc16_t *device,
    uint8_t action_group,
    uint16_t repeat_count)
{
    uint8_t parameters[3] = {
        action_group,
        (uint8_t)(repeat_count & 0xFFU),
        (uint8_t)(repeat_count >> 8),
    };
    return lsc16_send_frame(device, LSC16_CMD_ACTION_GROUP_RUN, parameters, 3U);
}

/**
 * @brief 构造0x07停止动作组命令。
 * @param device Core对象。
 * @return DMA启动结果。
 */
lsc16_status_t lsc16_stop_action_group(lsc16_t *device)
{
    return lsc16_send_frame(device, LSC16_CMD_ACTION_GROUP_STOP, NULL, 0U);
}

/**
 * @brief 构造动作组速度设置命令。
 * @param device Core对象。
 * @param action_group 动作组编号，0xFF表示全部动作组。
 * @param speed_percent 速度百分比参数。
 * @return 参数校验和DMA启动结果。
 */
lsc16_status_t lsc16_set_action_group_speed(
    lsc16_t *device,
    uint8_t action_group,
    uint16_t speed_percent)
{
    uint8_t parameters[3] = {
        action_group,
        (uint8_t)(speed_percent & 0xFFU),
        (uint8_t)(speed_percent >> 8),
    };
    return lsc16_send_frame(device, LSC16_CMD_ACTION_GROUP_SPEED, parameters, 3U);
}

/**
 * @brief 构造0x0F电池电压查询命令。
 * @param device Core对象。
 * @return DMA启动结果。
 */
lsc16_status_t lsc16_request_battery_voltage(lsc16_t *device)
{
    return lsc16_send_frame(
        device,
        LSC16_CMD_GET_BATTERY_VOLTAGE,
        NULL,
        0U);
}

/**
 * @brief 在普通上下文消费环形缓冲并解析主动回报。
 * @param device Core对象。
 * @note 禁止在ISR中调用；ISR只负责把DMA字节搬入环形缓冲。
 */
void lsc16_process(lsc16_t *device)
{
    uint8_t data;

    if ((device == NULL) || !device->initialized) {
        return;
    }
    while (device->rx_tail != device->rx_head) {
        data = device->rx_ring[device->rx_tail];
        device->rx_tail = lsc16_next_ring_index(device->rx_tail);
        lsc16_parse_byte(device, data);
    }
}

/**
 * @brief 从UART错误中同步恢复Core并重新挂接接收。
 * @param device Core对象。
 * @return abort和RX重启结果。
 */
lsc16_status_t lsc16_recover(lsc16_t *device)
{
    lsc16_status_t status;

    if ((device == NULL) || !device->initialized) {
        return (device == NULL) ? LSC16_ERR_PARAM : LSC16_ERR_NOT_INIT;
    }
    status = lsc16_map_hal(HAL_UART_Abort(&huart8));
    if (status != LSC16_OK) {
        return status;
    }
    device->tx_busy = false;
    return lsc16_restart_rx(device);
}

/**
 * @brief 查询是否仍有DMA发送在途。
 * @param device Core对象。
 * @return 已初始化且TX活动时返回true。
 */
bool lsc16_is_tx_busy(const lsc16_t *device)
{
    return (device != NULL) && device->initialized && device->tx_busy;
}

/**
 * @brief 读取并清除累计的主动回报事件位。
 * @param device Core对象。
 * @return 本次取出的事件位掩码。
 */
uint32_t lsc16_take_report_events(lsc16_t *device)
{
    uint32_t events;

    if ((device == NULL) || !device->initialized) {
        return LSC16_REPORT_EVENT_NONE;
    }
    events = device->report_events;
    device->report_events = LSC16_REPORT_EVENT_NONE;
    return events;
}

/**
 * @brief 复制最近一次控制板主动回报。
 * @param device Core对象。
 * @param report 接收回报副本的输出对象。
 * @return 获取结果。
 */
lsc16_status_t lsc16_get_last_report(
    const lsc16_t *device,
    lsc16_report_t *report)
{
    if ((device == NULL) || (report == NULL)) {
        return LSC16_ERR_PARAM;
    }
    if (!device->initialized) {
        return LSC16_ERR_NOT_INIT;
    }
    *report = device->last_report;
    return LSC16_OK;
}

/**
 * @brief 处理底层TX DMA完成中断事件。
 * @param device Core对象。
 * @warning 仅由拥有UART8的HAL适配器在ISR中调用。
 */
bool lsc16_handle_tx_complete(
    lsc16_t *device,
    UART_HandleTypeDef *huart)
{
    if ((device == NULL) || !device->initialized || (huart != &huart8)) {
        return false;
    }
    device->tx_busy = false;
    lsc16_notify_isr(device, LSC16_ISR_EVENT_TX_COMPLETE);
    return true;
}

/**
 * @brief 搬运本次DMA接收字节并重新挂接ReceiveToIdle。
 * @param device Core对象。
 * @param rx_len 本次DMA有效字节数。
 * @warning ISR中只搬运字节和发布事件，不解析协议。
 */
bool lsc16_handle_rx_event(
    lsc16_t *device,
    UART_HandleTypeDef *huart,
    uint16_t rx_len)
{
    uint16_t i;
    uint16_t next_head;
    lsc16_status_t status;

    if ((device == NULL) || !device->initialized || (huart != &huart8)) {
        return false;
    }
    if (rx_len > sizeof(device->dma_rx_buffer)) {
        rx_len = (uint16_t)sizeof(device->dma_rx_buffer);
        ++device->rx_overflow_count;
    }
    for (i = 0U; i < rx_len; ++i) {
        next_head = lsc16_next_ring_index(device->rx_head);
        if (next_head == device->rx_tail) {
            ++device->rx_overflow_count;
            break;
        }
        device->rx_ring[device->rx_head] = device->dma_rx_buffer[i];
        device->rx_head = next_head;
    }

    /* Normal DMA在IDLE/TC事件后结束，必须立即续接下一段接收窗口。 */
    status = lsc16_restart_rx(device);
    if (status != LSC16_OK) {
        ++device->uart_error_count;
        lsc16_notify_isr(device, LSC16_ISR_EVENT_ERROR);
        return true;
    }
    lsc16_notify_isr(device, LSC16_ISR_EVENT_RX_READY);
    return true;
}

/**
 * @brief 记录UART错误并通知上层在普通上下文恢复。
 * @param device Core对象。
 * @warning ISR中不执行可能阻塞的abort。
 */
bool lsc16_handle_error(lsc16_t *device, UART_HandleTypeDef *huart)
{
    if ((device == NULL) || !device->initialized || (huart != &huart8)) {
        return false;
    }
    device->tx_busy = false;
    ++device->uart_error_count;
    /* HAL_UART_Abort可能触碰较多HAL状态，只登记错误，交给普通上下文恢复。 */
    lsc16_notify_isr(device, LSC16_ISR_EVENT_ERROR);
    return true;
}
