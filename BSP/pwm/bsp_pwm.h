#pragma once

#include "main.h"
#include <cstdint>

class PWM {
public:
    using Callback_t = void (*)(PWM *pwm);

    /* PWM 配置结构体 */
    struct Config {
        TIM_HandleTypeDef *htim;
        uint32_t channel;
        float period; // 秒
        float dutyRatio; // 0.0 ~ 1.0
        Callback_t callback;
    };

    /* PWM 构造函数 */
    explicit PWM(const Config &config);

    void init();

    void start() const;

    void stop() const;

    void startDMA(const uint32_t *pData, uint16_t size) const;

    void stopDMA() const;

    void setPeriod(float sec);

    void setDuty(float ratio);

    [[nodiscard]] float getPeriod() const { return period; }
    [[nodiscard]] float getDuty() const { return dutyRatio; }

    [[nodiscard]] TIM_HandleTypeDef *get_handle() const { return htim; }
    [[nodiscard]] uint32_t get_channel() const { return channel; }

    static void pulseFinishedHandler(const TIM_HandleTypeDef *htim);

private:
    static uint32_t selectTimerClock(TIM_HandleTypeDef *htim);

    TIM_HandleTypeDef *htim = nullptr;
    uint32_t channel = 0;
    float period = 0.001f;
    float dutyRatio = 0.5f;
    uint32_t tclk = 0;
    Callback_t callback = nullptr;

    /* 最大 PWM 设备注册数量 */
    static constexpr uint8_t MAX_CNT = 16;

    static PWM *instances_[MAX_CNT];
    static uint8_t count_;
};
