/**
 * @file    mult_uart_stm32_hal.c
 * @brief   mult_uart Core 的 STM32F7 HAL 适配实现
 * @note    依赖方向固定为 Core -> port ops -> STM32 HAL；本文件不实现事务
 *          调度、协议解析或 RTOS 通知，只翻译硬件操作和 ISR 事件。
 */

#include "mult_uart_stm32_hal.h"

#include <limits.h>
#include <string.h>

#include "main.h"
#include "usart.h"

/**
 * @brief 同时写入A/B选择线，定义位于下方。
 * @param ctx HAL适配器上下文。
 * @param a_high A选择线目标电平。
 * @param b_high B选择线目标电平。
 * @return MULT_UART_OK表示写入成功。
 */
static mult_uart_status_t mult_uart_stm32_hal_write_select(
    void *ctx,
    bool a_high,
    bool b_high);
/**
 * @brief 写入EN物理电平，定义位于下方。
 * @param ctx HAL适配器上下文。
 * @param level_high EN引脚目标物理电平。
 * @return MULT_UART_OK表示写入成功。
 */
static mult_uart_status_t mult_uart_stm32_hal_write_enable(
    void *ctx,
    bool level_high);
/**
 * @brief 启动公共UART异步中断发送，定义位于下方。
 * @param ctx HAL适配器上下文。
 * @param data 待发送缓冲区。
 * @param len 发送长度。
 * @return 映射后的mult_uart状态。
 */
static mult_uart_status_t mult_uart_stm32_hal_start_tx_dma(
    void *ctx,
    const uint8_t *data,
    size_t len);
/**
 * @brief 启动公共UART ReceiveToIdle DMA接收，定义位于下方。
 * @param ctx HAL适配器上下文。
 * @param data DMA接收缓冲区。
 * @param capacity 缓冲区容量。
 * @return 映射后的mult_uart状态。
 */
static mult_uart_status_t mult_uart_stm32_hal_start_rx_dma(
    void *ctx,
    uint8_t *data,
    size_t capacity);
/**
 * @brief 同步中止公共UART异步发送和RX DMA，定义位于下方。
 * @param ctx HAL适配器上下文。
 * @return MULT_UART_OK表示旧传输和待处理IRQ已经清理。
 */
static mult_uart_status_t mult_uart_stm32_hal_abort_dma(void *ctx);

/**
 * @brief 使用有界CPU循环实现短微秒稳定等待，定义位于下方。
 * @param ctx 未使用的HAL适配器上下文。
 * @param delay_us 等待微秒数。
 */
static void mult_uart_stm32_hal_delay_us(void *ctx, uint32_t delay_us);

/** @brief 注入 Core 的唯一 STM32 HAL port 函数表。 */
static const mult_uart_port_ops_t mult_uart_stm32_hal_ops = {
    mult_uart_stm32_hal_write_select,
    mult_uart_stm32_hal_write_enable,
    mult_uart_stm32_hal_start_tx_dma,
    mult_uart_stm32_hal_start_rx_dma,
    mult_uart_stm32_hal_abort_dma,
    mult_uart_stm32_hal_delay_us,
};

/**
 * @brief 将 HAL 返回值收敛为 Core 状态。
 * @param status HAL接口返回值。
 * @return 对应的mult_uart状态码。
 * @note 具体 HAL 类型不能向上传播；未知 HAL 值按通用 I/O 错误处理，
 *       避免上层误把未识别状态当成成功。
 */
static mult_uart_status_t mult_uart_stm32_hal_map_status(
    HAL_StatusTypeDef status)
{
    switch (status) {
        case HAL_OK:
            return MULT_UART_OK;
        case HAL_BUSY:
            return MULT_UART_ERR_BUSY;
        case HAL_TIMEOUT:
            return MULT_UART_ERR_TIMEOUT;
        case HAL_ERROR:
        default:
            return MULT_UART_ERR_IO;
    }
}

/**
 * @brief 判断 pin mask 是否只描述一根 GPIO。
 * @param pin HAL GPIO位掩码。
 * @return 恰好只有一位置位时返回true。
 * @note HAL GPIO pin 使用位掩码；零值或多位置位都不能表示一根独立的
 *       A、B、EN 信号线。
 */
static bool mult_uart_stm32_hal_pin_is_valid(uint16_t pin)
{
    return (pin != 0U) && ((pin & (uint16_t)(pin - 1U)) == 0U);
}

/**
 * @brief 在 bind 提交 context 前集中校验硬件资源闭包。
 * @param config 待检查板级资源配置。
 * @return 所需UART、RX DMA和GPIO资源完整时返回true。
 *
 * RX DMA handle 只有在 Cube UART MSP 初始化并完成 __HAL_LINKDMA 后才有效，
 * 因此这里也能尽早发现“先 bind、后初始化外设”的错误顺序。TX使用UART
 * 中断，不要求hdmatx。
 */
static bool mult_uart_stm32_hal_config_is_valid(
    const mult_uart_stm32_hal_config_t *config)
{
    if ((config == NULL) || (config->huart == NULL) ||
        (config->huart->hdmarx == NULL) ||
        (config->select_port == NULL) ||
        (config->enable_port == NULL)) {
        return false;
    }

    if (!mult_uart_stm32_hal_pin_is_valid(config->select_a_pin) ||
        !mult_uart_stm32_hal_pin_is_valid(config->select_b_pin) ||
        !mult_uart_stm32_hal_pin_is_valid(config->enable_pin)) {
        return false;
    }

    return config->select_a_pin != config->select_b_pin;
}

/**
 * @brief 填写本工程正式公共端的固定资源描述。
 * @param config 接收资源映射的输出对象。
 *
 * 本函数只建立名称到资源的映射，不触碰寄存器，也不替代 Cube 生成的
 * MX_GPIO_Init()/MX_DMA_Init()/MX_UART7_Init()。
 */
void mult_uart_stm32_hal_make_uart7_config(
    mult_uart_stm32_hal_config_t *config)
{
    if (config == NULL) {
        return;
    }

    config->huart = &huart7;
    /* A/B必须同属一个GPIO port，才能用一次BSRR原子提交两根选择线。 */
    config->select_port = (M_A_GPIO_Port == M_B_GPIO_Port) ?
        M_A_GPIO_Port : NULL;
    config->select_a_pin = M_A_Pin;
    config->select_b_pin = M_B_Pin;
    config->enable_port = M_EN_GPIO_Port;
    config->enable_pin = M_EN_Pin;
    config->uart_irqn = UART7_IRQn;
    config->rx_dma_irqn = DMA1_Stream3_IRQn;
}

/**
 * @brief 把一个 Core bus 与一组已初始化的 STM32 资源绑定成 port。
 * @param adapter HAL适配器对象。
 * @param bus 平台无关Core总线对象。
 * @param config 板级资源配置。
 * @param port 接收port函数表的输出对象。
 * @return 绑定结果。
 *
 * adapter 同时保存向下访问硬件所需的 config 和 ISR 向上回到 Core 所需的
     * bus 指针。只有所有校验通过后才发布 bound 和输出 port，避免半绑定对象
     * 被 Core 使用。
 */
mult_uart_status_t mult_uart_stm32_hal_bind(
    mult_uart_stm32_hal_t *adapter,
    mult_uart_bus_t *bus,
    const mult_uart_stm32_hal_config_t *config,
    mult_uart_port_t *port)
{
    if ((adapter == NULL) || (bus == NULL) || (port == NULL) ||
        !mult_uart_stm32_hal_config_is_valid(config)) {
        return MULT_UART_ERR_PARAM;
    }

    memset(adapter, 0, sizeof(*adapter));
    adapter->bus = bus;
    adapter->config = *config;
    adapter->bound = true;

    port->ops = &mult_uart_stm32_hal_ops;
    port->ctx = adapter;

    return MULT_UART_OK;
}

/**
 * @brief 把逻辑 A/B 电平一次提交到选择 GPIO。
 * @param ctx HAL适配器上下文。
 * @param a_high A线目标电平。
 * @param b_high B线目标电平。
 * @return GPIO资源有效时返回MULT_UART_OK。
 *
 * PD9/PD10 位于同一 GPIO port，使用单次 BSRR 写可同时 set/reset 两根线，
 * 比两次 HAL_GPIO_WritePin 更少出现中间通道真值。主要隔离仍由 Core 的
 * break-before-switch 保证。
 */
static mult_uart_status_t mult_uart_stm32_hal_write_select(
    void *ctx,
    bool a_high,
    bool b_high)
{
    mult_uart_stm32_hal_t *adapter = (mult_uart_stm32_hal_t *)ctx;
    uint32_t set_mask = 0U;
    uint32_t reset_mask = 0U;

    if ((adapter == NULL) || !adapter->bound) {
        return MULT_UART_ERR_NOT_INIT;
    }

    set_mask |= a_high ? adapter->config.select_a_pin : 0U;
    reset_mask |= a_high ? 0U : adapter->config.select_a_pin;
    set_mask |= b_high ? adapter->config.select_b_pin : 0U;
    reset_mask |= b_high ? 0U : adapter->config.select_b_pin;

    /* 一次 BSRR 写同时提交 A/B，缩短中间通道出现的窗口。 */
    adapter->config.select_port->BSRR =
        set_mask | (reset_mask << 16U);
    return MULT_UART_OK;
}

/**
 * @brief 输出 EN 的实际引脚电平。
 * @param ctx HAL适配器上下文。
 * @param level_high 目标物理电平。
 * @return GPIO资源有效时返回MULT_UART_OK。
 * @note 本函数不解释低有效极性；Core 已根据 enable_active_low 把“使能/
 *       禁用”转换为 level_high，平台层只忠实写入物理电平。
 */
static mult_uart_status_t mult_uart_stm32_hal_write_enable(
    void *ctx,
    bool level_high)
{
    mult_uart_stm32_hal_t *adapter = (mult_uart_stm32_hal_t *)ctx;

    if ((adapter == NULL) || !adapter->bound) {
        return MULT_UART_ERR_NOT_INIT;
    }

    HAL_GPIO_WritePin(
        adapter->config.enable_port,
        adapter->config.enable_pin,
        level_high ? GPIO_PIN_SET : GPIO_PIN_RESET);
    return MULT_UART_OK;
}

/**
 * @brief 启动公共 UART7 的异步中断发送。
 * @param ctx HAL适配器上下文。
 * @param data 待发送数据。
 * @param len 数据长度。
 * @return HAL启动结果的统一映射。
 *
 * HAL 长度只有 16 位，必须在 size_t 窄化前检查上限。本工程 HAL 的发送
 * 参数本身是 const uint8_t *，因此无需丢弃 const。HAL中断发送仍会在函数
 * 返回后继续读取该缓冲区，其生命周期必须保持到TX complete/error/abort。
 * 函数名保留历史的_dma后缀，避免本轮扩大到Core接口重命名。
 */
static mult_uart_status_t mult_uart_stm32_hal_start_tx_dma(
    void *ctx,
    const uint8_t *data,
    size_t len)
{
    mult_uart_stm32_hal_t *adapter = (mult_uart_stm32_hal_t *)ctx;

    if ((adapter == NULL) || !adapter->bound) {
        return MULT_UART_ERR_NOT_INIT;
    }
    if ((data == NULL) || (len == 0U) || (len > UINT16_MAX)) {
        return MULT_UART_ERR_PARAM;
    }

    return mult_uart_stm32_hal_map_status(HAL_UART_Transmit_IT(
        adapter->config.huart,
        data,
        (uint16_t)len));
}

/**
 * @brief 启动公共 UART7 的 ReceiveToIdle Normal DMA 接收。
 * @param ctx HAL适配器上下文。
 * @param data 接收缓冲区。
 * @param capacity 缓冲区容量。
 * @return HAL启动结果的统一映射。
 *
 * Core 把一次 RxEvent 当作事务的 RX_COMPLETE，因此启动成功后关闭 DMA HT
 * 中断，只让 UART IDLE 或 DMA 满缓冲结束本次接收。Normal DMA 完成后是否
 * 重启由上层下一事务决定，adapter 不在 ISR 中擅自续接新缓冲区。
 */
static mult_uart_status_t mult_uart_stm32_hal_start_rx_dma(
    void *ctx,
    uint8_t *data,
    size_t capacity)
{
    mult_uart_stm32_hal_t *adapter = (mult_uart_stm32_hal_t *)ctx;
    HAL_StatusTypeDef status;

    if ((adapter == NULL) || !adapter->bound) {
        return MULT_UART_ERR_NOT_INIT;
    }
    if ((data == NULL) || (capacity == 0U) ||
        (capacity > UINT16_MAX)) {
        return MULT_UART_ERR_PARAM;
    }

    /*
     * CD4052在break-before-switch期间会断开RX通路，PE7可能出现
     * 短暂浮空/边沿，从而在UART中留下FE、NE或ORE标志。HAL官方
     * ReceiveToIdle实现明确指出：开始接收时若已有错误待处理，
     * 本次接收可能被立即终止。命令/应答总线在新事务前没有
     * 需要保留的旧字节，因此先清错误、IDLE并丢弃旧RDR数据。
     */
    __HAL_UART_CLEAR_FLAG(
        adapter->config.huart,
        UART_CLEAR_OREF | UART_CLEAR_NEF | UART_CLEAR_PEF |
            UART_CLEAR_FEF | UART_CLEAR_IDLEF);
    __HAL_UART_SEND_REQ(
        adapter->config.huart,
        UART_RXDATA_FLUSH_REQUEST);
    adapter->config.huart->ErrorCode = HAL_UART_ERROR_NONE;

    status = HAL_UARTEx_ReceiveToIdle_DMA(
        adapter->config.huart,
        data,
        (uint16_t)capacity);
    if (status != HAL_OK) {
        return mult_uart_stm32_hal_map_status(status);
    }

    /* Core 的 RX_COMPLETE 表示一帧结束，不能把 DMA 半满误报成完整帧。 */
    __HAL_DMA_DISABLE_IT(adapter->config.huart->hdmarx, DMA_IT_HT);
    return MULT_UART_OK;
}

/**
 * @brief 同步中止UART异步收发并清理可能污染下一事务的旧中断。
 * @param ctx HAL适配器上下文。
 * @return HAL abort结果的统一映射。
 *
 * Core 在调用本函数前已进入 aborting，期间到达的回调会被忽略。只有
 * HAL_UART_Abort成功后才清UART flags以及UART、RX DMA两个NVIC pending；
 * 若HAL abort失败则原样上报，使Core保持ERROR，而不是虚假恢复为IDLE。
 */
static mult_uart_status_t mult_uart_stm32_hal_abort_dma(void *ctx)
{
    mult_uart_stm32_hal_t *adapter = (mult_uart_stm32_hal_t *)ctx;
    HAL_StatusTypeDef status;

    if ((adapter == NULL) || !adapter->bound) {
        return MULT_UART_ERR_NOT_INIT;
    }

    status = HAL_UART_Abort(adapter->config.huart);
    if (status != HAL_OK) {
        return mult_uart_stm32_hal_map_status(status);
    }

    /*
     * HAL_UART_Abort 同步停UART收发并复位HAL状态；这里再清UART状态位和
     * NVIC pending，落实 Core 对“旧 ISR 不得污染下一事务”的更强契约。
     */
    __HAL_UART_CLEAR_PEFLAG(adapter->config.huart);
    HAL_NVIC_ClearPendingIRQ(adapter->config.uart_irqn);
    HAL_NVIC_ClearPendingIRQ(adapter->config.rx_dma_irqn);

    return MULT_UART_OK;
}

/**
 * @brief 使用有界CPU循环提供最短不少于请求值的微秒等待。
 * @param ctx HAL适配器上下文。
 * @param delay_us 等待微秒数。
 *
 * 逐微秒执行可避免 delay_us * cycles_per_us 的32位溢出。每次循环除NOP外
 * 还有递增和比较开销，所以只会比请求时间更长，不会更短。该实现
 * 不依赖调试器可能预先开启的DWT/CYCCNT，且循环上限始终有限，
 * 避免高优先级worker因计数器停止而永久占用CPU。
 */
static void mult_uart_stm32_hal_delay_us(void *ctx, uint32_t delay_us)
{
    mult_uart_stm32_hal_t *adapter = (mult_uart_stm32_hal_t *)ctx;
    uint32_t cycles_per_us;
    uint32_t spin;
    uint32_t i;

    if ((adapter == NULL) || !adapter->bound || (delay_us == 0U)) {
        return;
    }

    cycles_per_us = SystemCoreClock / 1000000U;
    if (cycles_per_us == 0U) {
        cycles_per_us = 1U;
    }

    /* 逐微秒等待避免 delay_us * cycles_per_us 的32位乘法溢出。 */
    for (i = 0U; i < delay_us; ++i) {
        for (spin = 0U; spin < cycles_per_us; ++spin) {
            __NOP();
        }
    }
}

/**
 * @brief 判断一个 HAL callback 是否属于本 adapter 绑定的物理 UART。
 * @param adapter HAL适配器对象。
 * @param huart 产生回调的UART句柄。
 * @return 句柄匹配且适配器已绑定时返回true。
 * @note HAL weak callback 为全局入口，必须先筛选 handle，避免把其他UART
 *       的事件错误上报给正式UART7复用总线。
 */
static bool mult_uart_stm32_hal_owns_uart(
    const mult_uart_stm32_hal_t *adapter,
    const UART_HandleTypeDef *huart)
{
    return (adapter != NULL) && adapter->bound &&
           (adapter->bus != NULL) &&
           (adapter->config.huart == huart);
}

/**
 * @brief 将匹配 UART 的最终 TX complete 事件转交 Core ISR 入口。
 * @param adapter HAL适配器对象。
 * @param huart 产生回调的UART句柄。
 * @retval true 表示该 UART 已被本 adapter 认领；false 允许统一分发器继续
 *         尝试UART8舵控板等其他消费者。
 */
bool mult_uart_stm32_hal_handle_tx_complete(
    mult_uart_stm32_hal_t *adapter,
    UART_HandleTypeDef *huart)
{
    if (!mult_uart_stm32_hal_owns_uart(adapter, huart)) {
        return false;
    }

    mult_uart_on_tx_complete_isr(adapter->bus);
    return true;
}

/**
 * @brief 将匹配 UART 的 IDLE/满缓冲有效长度转交 Core ISR 入口。
 * @param adapter HAL适配器对象。
 * @param huart 产生回调的UART句柄。
 * @param rx_len 本次有效字节数。
 * @return 匹配并转发时返回true。
 * @note handler 只翻译事件，不解析数据，也不在 ISR 中重启下一次接收。
 */
bool mult_uart_stm32_hal_handle_rx_event(
    mult_uart_stm32_hal_t *adapter,
    UART_HandleTypeDef *huart,
    uint16_t rx_len)
{
    if (!mult_uart_stm32_hal_owns_uart(adapter, huart)) {
        return false;
    }

    mult_uart_on_rx_complete_isr(adapter->bus, (size_t)rx_len);
    return true;
}

/**
 * @brief 将匹配 UART 的 HAL 原始 ErrorCode 转交 Core ISR 入口。
 * @param adapter HAL适配器对象。
 * @param huart 产生错误的UART句柄。
 * @return 匹配并转发时返回true。
 * @note Core 会结束两个活动方向并进入 ERROR；同步清理和恢复留给任务上下文
 *       随后调用 mult_uart_abort()，不能在错误 ISR 中阻塞 abort。
 */
bool mult_uart_stm32_hal_handle_error(
    mult_uart_stm32_hal_t *adapter,
    UART_HandleTypeDef *huart)
{
    if (!mult_uart_stm32_hal_owns_uart(adapter, huart)) {
        return false;
    }

    adapter->last_uart_error = huart->ErrorCode;
    adapter->uart_error_count++;
    mult_uart_on_error_isr(adapter->bus, huart->ErrorCode);
    return true;
}
