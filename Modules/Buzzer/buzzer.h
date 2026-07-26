#pragma once

#include "bsp_pwm.h"

/**
 * Simple PWM buzzer driver.
 * Pin/timer mapping is supplied via Config — no board hardcoding.
 */
class Buzzer {
public:
    struct Config {
        TIM_HandleTypeDef *htim = nullptr;
        uint32_t channel = 0;
        float periodSec = 0.08f;   /* tone period */
        float dutyRatio = 0.5f;
    };

    enum class State : uint8_t {
        Off = 0,
        On  = 1,
    };

    explicit Buzzer(const Config &config);

    void init() const;
    void setState(State on);
    [[nodiscard]] State getState() const { return state_; }

private:
    PWM pwm_;
    State state_ = State::Off;
};
