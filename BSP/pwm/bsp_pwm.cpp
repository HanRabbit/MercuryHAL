#include "bsp_pwm.h"

PWM *PWM::instances_[MAX_CNT] = {nullptr};
uint8_t PWM::count_ = 0;

/**
 * @brief PWM 构造函数
 * @param config PWM 配置结构体
 */
PWM::PWM(const Config &config) {
    htim = config.htim;
    channel = config.channel;
    period = config.period;
    dutyRatio = config.dutyRatio;
    callback = config.callback;

    tclk = selectTimerClock(htim);

    if (count_ < MAX_CNT) {
        instances_[count_++] = this;
    }
}

/**
 * @brief PWM 初始化函数，启动 PWM 输出并设置初始周期和占空比
 */
void PWM::init() {
    HAL_TIM_PWM_Start(htim, channel);

    setPeriod(period);
    setDuty(dutyRatio);
}

/**
 * @brief 启动 PWM 输出
 */
void PWM::start() const {
    HAL_TIM_PWM_Start(htim, channel);
}

/**
 * @brief 停止 PWM 输出
 */
void PWM::stop() const {
    HAL_TIM_PWM_Stop(htim, channel);
}

/**
 * @brief 启动 PWM DMA 输出，使用 DMA 传输占空比数据以实现动态调整
 * @param pData 指向占空比数据的指针，数据格式取决于定时器配置（通常为 CCR 寄存器值）
 * @param size 数据大小，单位为元素数量（例如 CCR 寄存器数量）
 */
void PWM::startDMA(const uint32_t *pData, const uint16_t size) const {
    HAL_TIM_PWM_Start_DMA(htim, channel, pData, size);
}

/**
 * @brief 停止 PWM DMA 输出
 */
void PWM::stopDMA() const {
    HAL_TIM_PWM_Stop_DMA(htim, channel);
}

/**
 * @brief 设置 PWM 周期，计算并更新定时器的 ARR 寄存器值以实现新的周期
 * @param sec 周期时间，单位为秒
 */
void PWM::setPeriod(const float sec) {
    period = sec;
    /** 计算 ARR 寄存器值，ARR = (tclk / (prescaler + 1)) * period - 1
     * 注意 STM32 定时器计数从 0 开始，因此需要减 1 来获得正确的周期 */
    const auto arr = static_cast<uint32_t>
                     (sec * (static_cast<double>(tclk) / static_cast<double>(htim->Init.Prescaler + 1))) - 1;
    __HAL_TIM_SetAutoreload(htim, arr);
}

/**
 * @brief 设置 PWM 占空比，计算并更新定时器的 CCR 寄存器值以实现新占空比
 * @param ratio 占空比，范围为 0.0 ~ 1.0
 */
void PWM::setDuty(float ratio) {
    if (ratio < 0.0f) ratio = 0.0f;
    if (ratio > 1.0f) ratio = 1.0f;

    dutyRatio = ratio;
    /**计算 CCR 寄存器值，CCR = ARR * dutyRatio */
    const auto ccr = static_cast<uint32_t>(ratio * static_cast<float>(htim->Instance->ARR));

    __HAL_TIM_SetCompare(htim, channel, ccr);
}

/**
 * @brief PWM 脉冲完成中断处理函数，遍历注册的 PWM 实例以找到匹配的定时器和通道，并调用对应的回调函数
 * @param htim 触发中断的定时器句柄指针
 */
void PWM::pulseFinishedHandler(const TIM_HandleTypeDef *htim) {
    for (uint8_t i = 0; i < count_; i++) {
        /* 检查当前实例是否与触发中断的定时器和通道匹配，注意 STM32 定时器通道使用位掩码表示 */
        if (PWM *pwm = instances_[i]; pwm->htim == htim &&
                                        htim->Channel ==
                                        (1U << (pwm->channel / 4))) {
            if (pwm->callback)
                pwm->callback(pwm);

            return;
        }
    }
}

/**
 * @brief 选择定时器时钟频率函数，根据定时器所在的总线（APB1 或 APB2）和时钟分频设置计算实际的定时器时钟频率
 * @param htim 定时器句柄指针，用于确定定时器所在的总线
 * @return 定时器时钟频率，单位为 Hz
 */
uint32_t PWM::selectTimerClock(TIM_HandleTypeDef *htim) {
#if defined(STM32H7)

    RCC_ClkInitTypeDef clk;
    uint32_t latency;

    HAL_RCC_GetClockConfig(&clk, &latency);

    /* APB1 TIM */
    /* STM32H7 系列中，APB1 总线上连接的定时器时钟频率可能是 PCLK1 的 2 倍，具体取决于 APB1 时钟分频器的设置 */
    if (const auto addr = reinterpret_cast<uintptr_t>(htim->Instance); (addr >= APB1PERIPH_BASE) &&
                                                                       (addr < APB1PERIPH_BASE + 0x8000)) {
        const uint32_t pclk = HAL_RCC_GetPCLK1Freq();

        return (clk.APB1CLKDivider ==
                RCC_HCLK_DIV1)
                   ? pclk
                   : pclk * 2;
    }

    /* APB2 TIM */
    const uint32_t pclk = HAL_RCC_GetPCLK2Freq();

    return (clk.APB2CLKDivider ==
            RCC_HCLK_DIV1)
               ? pclk
               : pclk * 2;

#else
    return HAL_RCC_GetPCLK1Freq();
#endif
}

extern "C"
void HAL_TIM_PWM_PulseFinishedCallback(
    TIM_HandleTypeDef *htim) {
    PWM::pulseFinishedHandler(htim);
}
