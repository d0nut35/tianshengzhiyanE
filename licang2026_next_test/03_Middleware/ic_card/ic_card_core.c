/**
 * @file    ic_card_core.c
 * @brief   M2940B-HA协议构帧、异步缓冲和响应解析实现。
 */

#include "ic_card_core.h"

#include <string.h>

#define IC_CARD_RESPONSE_PREFIX_SIZE 5U

/**
 * @brief 计算环形缓冲区的下一个索引。
 * @param index 当前索引。
 * @return 回绕后的下一索引。
 */
static uint16_t ic_card_next_ring_index(uint16_t index)
{
    ++index;
    return (index >= IC_CARD_RX_RING_BUFFER_SIZE) ? 0U : index;
}

/**
 * @brief 在ISR中向上层发布轻量事件。
 * @param device IC卡Core对象。
 * @param event 待发布事件。
 * @warning 回调运行在ISR上下文，必须保持轻量且不能阻塞。
 */
static void ic_card_notify_isr(ic_card_t *device, ic_card_isr_event_t event)
{
    if (device->notify_cb != NULL) {
        device->notify_cb(device->notify_ctx, event);
    }
}

/**
 * @brief 重新挂接一次ReceiveToIdle DMA接收。
 * @param device IC卡Core对象。
 * @return port层启动结果。
 * @note Normal DMA在每次IDLE/TC后停止，因此每个接收事件后必须重新挂接。
 */
static ic_card_status_t ic_card_restart_rx(ic_card_t *device)
{
    return device->port.rx_start(
        device->port.ctx,
        device->dma_rx_buffer,
        sizeof(device->dma_rx_buffer));
}

/**
 * @brief 判断字节是否是厂家协议允许的包类型。
 * @param packet_type 待检查字节。
 * @return 属于0x01~0x04时返回true。
 */
static bool ic_card_packet_type_is_valid(uint8_t packet_type)
{
    return ((packet_type >= IC_CARD_PACKET_CARD_OPERATION) &&
            (packet_type <= IC_CARD_PACKET_OTHER));
}

/**
 * @brief 计算厂家规定的逐字节异或取反校验。
 * @param data 待校验字节序列。
 * @param len_without_checksum 不包含校验字节的长度。
 * @return 计算出的单字节校验值。
 */
uint8_t ic_card_checksum(const uint8_t *data, size_t len_without_checksum)
{
    uint8_t value = 0U;
    size_t i;

    if ((data == NULL) && (len_without_checksum > 0U)) {
        return 0U;
    }
    for (i = 0U; i < len_without_checksum; ++i) {
        value ^= data[i];
    }
    return (uint8_t)(~value);
}

/**
 * @brief 构造厂家统一8字节命令帧。
 * @param packet_type 厂家包类型。
 * @param command 命令号。
 * @param address 设备地址。
 * @param parameter0 第一个参数。
 * @param parameter1 第二个参数。
 * @param parameter2 第三个参数。
 * @param frame 输出缓冲区。
 * @param capacity 输出容量。
 * @param frame_len 输出有效长度。
 * @return 构帧状态。
 */
static ic_card_status_t ic_card_build_command8(
    uint8_t packet_type,
    uint8_t command,
    uint8_t address,
    uint8_t parameter0,
    uint8_t parameter1,
    uint8_t parameter2,
    uint8_t *frame,
    size_t capacity,
    size_t *frame_len)
{
    if ((frame == NULL) || (frame_len == NULL) ||
        (capacity < IC_CARD_COMMAND_FRAME_SIZE) ||
        !ic_card_packet_type_is_valid(packet_type)) {
        return IC_CARD_ERR_PARAM;
    }
    frame[0] = packet_type;
    frame[1] = IC_CARD_COMMAND_FRAME_SIZE;
    frame[2] = command;
    frame[3] = address;
    frame[4] = parameter0;
    frame[5] = parameter1;
    frame[6] = parameter2;
    frame[7] = ic_card_checksum(frame, 7U);
    *frame_len = IC_CARD_COMMAND_FRAME_SIZE;
    return IC_CARD_OK;
}

/** @copydoc ic_card_build_read_block_key_a_frame() */
ic_card_status_t ic_card_build_read_block_key_a_frame(
    uint8_t address,
    uint8_t block,
    bool led_beep_prompt,
    uint8_t *frame,
    size_t capacity,
    size_t *frame_len)
{
    return ic_card_build_command8(
        IC_CARD_PACKET_CARD_OPERATION,
        IC_CARD_CMD_READ_BLOCK_KEY_A,
        address,
        block,
        led_beep_prompt ? 1U : 0U,
        0U,
        frame,
        capacity,
        frame_len);
}

/** @copydoc ic_card_build_query_frame() */
ic_card_status_t ic_card_build_query_frame(
    ic_card_command_t command,
    uint8_t address,
    uint8_t *frame,
    size_t capacity,
    size_t *frame_len)
{
    if ((command != IC_CARD_CMD_QUERY_ADDRESS) &&
        (command != IC_CARD_CMD_QUERY_WORK_MODE) &&
        (command != IC_CARD_CMD_QUERY_BEEPER) &&
        (command != IC_CARD_CMD_QUERY_AUTO_READ)) {
        return IC_CARD_ERR_UNSUPPORTED;
    }
    return ic_card_build_command8(
        IC_CARD_PACKET_QUERY,
        (uint8_t)command,
        (command == IC_CARD_CMD_QUERY_ADDRESS) ? 0U : address,
        0U,
        0U,
        0U,
        frame,
        capacity,
        frame_len);
}

/** @copydoc ic_card_parse_response_frame() */
ic_card_status_t ic_card_parse_response_frame(
    const uint8_t *frame,
    size_t frame_len,
    ic_card_response_t *response)
{
    size_t payload_len;

    if ((frame == NULL) || (response == NULL)) {
        return IC_CARD_ERR_PARAM;
    }
    if ((frame_len < 6U) || (frame_len > IC_CARD_FRAME_SIZE_MAX) ||
        (frame[1] != frame_len) ||
        !ic_card_packet_type_is_valid(frame[0]) ||
        (ic_card_checksum(frame, frame_len - 1U) != frame[frame_len - 1U])) {
        return IC_CARD_ERR_PROTOCOL;
    }
    payload_len = frame_len - 6U;
    (void)memset(response, 0, sizeof(*response));
    response->packet_type = frame[0];
    response->command = frame[2];
    response->address = frame[3];
    response->device_status = frame[4];
    response->payload_len = (uint8_t)payload_len;
    response->raw_len = (uint8_t)frame_len;
    if (payload_len > 0U) {
        (void)memcpy(response->payload, &frame[5], payload_len);
    }
    (void)memcpy(response->raw, frame, frame_len);
    return IC_CARD_OK;
}

/**
 * @brief 构造并异步发送统一8字节命令帧。
 * @param device IC卡Core对象。
 * @param packet_type 厂家包类型。
 * @param command 命令号。
 * @param address 设备地址。
 * @param parameter0 第一个参数字节。
 * @param parameter1 第二个参数字节。
 * @param parameter2 第三个参数字节。
 * @return DMA启动状态。
 * @note 厂家协议无固定帧头，先写完前7字节再统一计算校验，避免手写常量出错。
 */
static ic_card_status_t ic_card_send_command8(
    ic_card_t *device,
    uint8_t packet_type,
    uint8_t command,
    uint8_t address,
    uint8_t parameter0,
    uint8_t parameter1,
    uint8_t parameter2)
{
    ic_card_status_t status;
    size_t frame_len;

    if ((device == NULL) || !device->initialized) {
        return (device == NULL) ? IC_CARD_ERR_PARAM : IC_CARD_ERR_NOT_INIT;
    }
    if (device->tx_busy) {
        return IC_CARD_ERR_BUSY;
    }

    status = ic_card_build_command8(
        packet_type,
        command,
        address,
        parameter0,
        parameter1,
        parameter2,
        device->tx_buffer,
        sizeof(device->tx_buffer),
        &frame_len);
    if (status != IC_CARD_OK) {
        return status;
    }

    /* 先发布busy再启动DMA，防止极短传输的完成中断早于状态更新。 */
    device->tx_busy = true;
    status = device->port.tx_start(
        device->port.ctx,
        device->tx_buffer,
        frame_len);
    if (status != IC_CARD_OK) {
        device->tx_busy = false;
    }
    return status;
}

/**
 * @brief 提交一帧已经通过长度和校验检查的响应。
 * @param device IC卡Core对象。
 * @note 响应按值保存并递增序号，不保留对DMA缓冲区的引用。
 */
static void ic_card_commit_frame(ic_card_t *device)
{
    uint8_t frame_len = device->parse_buffer[1];

    if (ic_card_parse_response_frame(
            device->parse_buffer,
            frame_len,
            &device->last_response) == IC_CARD_OK) {
        ++device->response_sequence;
        if (device->response_sequence == 0U) {
            ++device->response_sequence;
        }
    }
}

/**
 * @brief 向无帧头协议拆包状态机投入一个字节。
 * @param device IC卡Core对象。
 * @param data 新收到的字节。
 * @note 只有包类型和长度合理才继续收集；校验失败时丢弃候选帧并重新同步。
 */
static void ic_card_parse_byte(ic_card_t *device, uint8_t data)
{
    if (device->parse_count == 0U) {
        if (ic_card_packet_type_is_valid(data)) {
            device->parse_buffer[0] = data;
            device->parse_count = 1U;
        }
        return;
    }

    if (device->parse_count == 1U) {
        if ((data < 6U) || (data > IC_CARD_FRAME_SIZE_MAX)) {
            ++device->invalid_frame_count;
            device->parse_count = 0U;
            device->expected_len = 0U;
            if (ic_card_packet_type_is_valid(data)) {
                device->parse_buffer[0] = data;
                device->parse_count = 1U;
            }
            return;
        }
        device->expected_len = data;
    }

    device->parse_buffer[device->parse_count++] = data;
    if (device->parse_count == device->expected_len) {
        if (ic_card_checksum(device->parse_buffer, device->expected_len - 1U) ==
            device->parse_buffer[device->expected_len - 1U]) {
            ic_card_commit_frame(device);
        } else {
            ++device->invalid_frame_count;
        }
        device->parse_count = 0U;
        device->expected_len = 0U;
    }
}

/**
 * @brief 初始化IC卡协议Core并启动首次DMA接收。
 * @param device 待初始化Core对象。
 * @param port 已绑定硬件上下文的异步串口能力。
 * @return 初始化结果；首次RX启动失败时完整清空对象。
 */
ic_card_status_t ic_card_init(ic_card_t *device, const ic_card_port_t *port)
{
    ic_card_status_t status;

    if ((device == NULL) || (port == NULL) || (port->tx_start == NULL) ||
        (port->rx_start == NULL) || (port->abort == NULL)) {
        return IC_CARD_ERR_PARAM;
    }
    if (device->initialized) {
        return IC_CARD_ERR_STATE;
    }

    (void)memset(device, 0, sizeof(*device));
    device->port = *port;
    device->initialized = true;
    status = ic_card_restart_rx(device);
    if (status != IC_CARD_OK) {
        (void)memset(device, 0, sizeof(*device));
    }
    return status;
}

/**
 * @brief 同步终止底层收发并反初始化Core。
 * @param device Core对象。
 * @return abort成功后返回IC_CARD_OK，否则保留对象供诊断。
 */
ic_card_status_t ic_card_deinit(ic_card_t *device)
{
    ic_card_status_t status;

    if (device == NULL) {
        return IC_CARD_ERR_PARAM;
    }
    if (!device->initialized) {
        return IC_CARD_ERR_NOT_INIT;
    }
    status = device->port.abort(device->port.ctx);
    if (status == IC_CARD_OK) {
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
ic_card_status_t ic_card_bind_isr_notify(
    ic_card_t *device,
    ic_card_isr_notify_fn_t notify_cb,
    void *user_ctx)
{
    if ((device == NULL) || !device->initialized) {
        return (device == NULL) ? IC_CARD_ERR_PARAM : IC_CARD_ERR_NOT_INIT;
    }
    device->notify_cb = notify_cb;
    device->notify_ctx = user_ctx;
    return IC_CARD_OK;
}

/**
 * @brief 使用模块内部Key A异步读取一个Mifare数据块。
 * @param device Core对象。
 * @param address 读卡器地址。
 * @param block 块号。
 * @param led_beep_prompt 是否请求成功提示。
 * @return 命令DMA启动状态。
 */
ic_card_status_t ic_card_read_block_key_a(
    ic_card_t *device,
    uint8_t address,
    uint8_t block,
    bool led_beep_prompt)
{
    return ic_card_send_command8(
        device,
        IC_CARD_PACKET_CARD_OPERATION,
        IC_CARD_CMD_READ_BLOCK_KEY_A,
        address,
        block,
        led_beep_prompt ? 1U : 0U,
        0U);
}

/**
 * @brief 异步发送一条受支持的参数查询命令。
 * @param device Core对象。
 * @param command B0/B1/B2/B8查询命令。
 * @param address 当前设备地址；B0会按手册自动发送地址0。
 * @return 命令DMA启动状态或不支持错误。
 */
ic_card_status_t ic_card_query(
    ic_card_t *device,
    ic_card_command_t command,
    uint8_t address)
{
    if ((command != IC_CARD_CMD_QUERY_ADDRESS) &&
        (command != IC_CARD_CMD_QUERY_WORK_MODE) &&
        (command != IC_CARD_CMD_QUERY_BEEPER) &&
        (command != IC_CARD_CMD_QUERY_AUTO_READ)) {
        return IC_CARD_ERR_UNSUPPORTED;
    }
    /* B0查询地址按手册要求address字段为0，其余查询使用当前设备地址。 */
    return ic_card_send_command8(
        device,
        IC_CARD_PACKET_QUERY,
        (uint8_t)command,
        (command == IC_CARD_CMD_QUERY_ADDRESS) ? 0U : address,
        0U,
        0U,
        0U);
}

/**
 * @brief 在普通上下文消费环形缓冲并推进流式拆包器。
 * @param device Core对象。
 * @note 禁止在ISR中调用；解析期间ISR可继续向环形缓冲写入新数据。
 */
void ic_card_process(ic_card_t *device)
{
    uint8_t data;

    if ((device == NULL) || !device->initialized) {
        return;
    }
    while (device->rx_tail != device->rx_head) {
        data = device->rx_ring[device->rx_tail];
        device->rx_tail = ic_card_next_ring_index(device->rx_tail);
        ic_card_parse_byte(device, data);
    }
}

/**
 * @brief 从UART错误或事务超时中同步恢复Core。
 * @param device Core对象。
 * @return abort并重新挂接RX的结果。
 * @note 清空未完成候选帧和旧RX字节，避免污染下一事务。
 */
ic_card_status_t ic_card_recover(ic_card_t *device)
{
    ic_card_status_t status;

    if ((device == NULL) || !device->initialized) {
        return (device == NULL) ? IC_CARD_ERR_PARAM : IC_CARD_ERR_NOT_INIT;
    }
    status = device->port.abort(device->port.ctx);
    if (status != IC_CARD_OK) {
        return status;
    }
    device->tx_busy = false;
    device->parse_count = 0U;
    device->expected_len = 0U;
    device->rx_tail = device->rx_head;
    return ic_card_restart_rx(device);
}

/**
 * @brief 查询Core是否仍有DMA发送在途。
 * @param device Core对象。
 * @return 已初始化且TX活动时返回true。
 */
bool ic_card_is_tx_busy(const ic_card_t *device)
{
    return (device != NULL) && device->initialized && device->tx_busy;
}

/**
 * @brief 按响应序号取得最近一帧新响应。
 * @param device Core对象。
 * @param last_sequence 调用者持有的上次序号；成功时更新。
 * @param response 接收响应副本的输出对象。
 * @return 存在新响应时返回true。
 */
bool ic_card_take_response(
    const ic_card_t *device,
    uint32_t *last_sequence,
    ic_card_response_t *response)
{
    if ((device == NULL) || (last_sequence == NULL) || (response == NULL) ||
        !device->initialized || (*last_sequence == device->response_sequence)) {
        return false;
    }
    *response = device->last_response;
    *last_sequence = device->response_sequence;
    return true;
}

/**
 * @brief 校验A3响应语义并提取16字节块数据。
 * @param response 已通过帧校验的响应。
 * @param expected_address 期望设备地址。
 * @param data 接收16字节数据的输出数组。
 * @return 成功、卡错误或协议错误。
 */
ic_card_status_t ic_card_extract_block_data(
    const ic_card_response_t *response,
    uint8_t expected_address,
    uint8_t data[IC_CARD_BLOCK_DATA_SIZE])
{
    if ((response == NULL) || (data == NULL)) {
        return IC_CARD_ERR_PARAM;
    }
    if ((response->packet_type != IC_CARD_PACKET_CARD_OPERATION) ||
        (response->command != IC_CARD_CMD_READ_BLOCK_KEY_A) ||
        (response->address != expected_address)) {
        return IC_CARD_ERR_PROTOCOL;
    }
    if (response->device_status != 0U) {
        return IC_CARD_ERR_CARD;
    }
    if (response->payload_len != IC_CARD_BLOCK_DATA_SIZE) {
        return IC_CARD_ERR_PROTOCOL;
    }
    (void)memcpy(data, response->payload, IC_CARD_BLOCK_DATA_SIZE);
    return IC_CARD_OK;
}

/**
 * @brief 处理底层TX DMA完成中断事件。
 * @param device Core对象。
 * @warning 仅由拥有该UART的HAL适配器在ISR中调用。
 */
void ic_card_on_tx_complete_isr(ic_card_t *device)
{
    if ((device == NULL) || !device->initialized) {
        return;
    }
    device->tx_busy = false;
    ic_card_notify_isr(device, IC_CARD_ISR_EVENT_TX_COMPLETE);
}

/**
 * @brief 搬运本次DMA接收字节并重新挂接ReceiveToIdle。
 * @param device Core对象。
 * @param rx_len DMA缓冲区中的有效字节数。
 * @warning 本函数运行在ISR中，只搬运字节和发布事件，不解析协议。
 */
void ic_card_on_rx_event_isr(ic_card_t *device, uint16_t rx_len)
{
    uint16_t i;
    uint16_t next_head;
    ic_card_status_t status;

    if ((device == NULL) || !device->initialized) {
        return;
    }
    if (rx_len > sizeof(device->dma_rx_buffer)) {
        rx_len = (uint16_t)sizeof(device->dma_rx_buffer);
        ++device->rx_overflow_count;
    }
    for (i = 0U; i < rx_len; ++i) {
        next_head = ic_card_next_ring_index(device->rx_head);
        if (next_head == device->rx_tail) {
            ++device->rx_overflow_count;
            break;
        }
        device->rx_ring[device->rx_head] = device->dma_rx_buffer[i];
        device->rx_head = next_head;
    }

    status = ic_card_restart_rx(device);
    if (status != IC_CARD_OK) {
        ++device->uart_error_count;
        ic_card_notify_isr(device, IC_CARD_ISR_EVENT_ERROR);
        return;
    }
    ic_card_notify_isr(device, IC_CARD_ISR_EVENT_RX_READY);
}

/**
 * @brief 记录UART错误并通知上层在普通上下文执行恢复。
 * @param device Core对象。
 * @warning ISR中不调用可能阻塞的abort。
 */
void ic_card_on_error_isr(ic_card_t *device)
{
    if ((device == NULL) || !device->initialized) {
        return;
    }
    device->tx_busy = false;
    ++device->uart_error_count;
    /* HAL abort不在ISR中执行，交给Service或裸机主循环完成恢复。 */
    ic_card_notify_isr(device, IC_CARD_ISR_EVENT_ERROR);
}
