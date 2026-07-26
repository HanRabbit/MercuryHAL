#pragma once

#include "bsp_gpio.h"

#include <cstdint>

class LED {
public:
    enum class StatusPattern : uint8_t {
        Ok = 0,      /* N=2 blinks + pause */
        Active,      /* N=3 blinks + pause */
        NoDevice,    /* N=4 blinks + pause */
        Error,       /* fast continuous blink */
    };

    explicit LED(const GPIO *gpio, bool reverse = false);

    void on() const;
    void off() const;
    void toggle() const;

    /** Blink `count` times: on → off. */
    void blinkBurst(unsigned count, uint32_t onMs = 80U, uint32_t offMs = 120U) const;

    /** One on/off cycle (used for fast error blink). */
    void blinkOnce(uint32_t onMs = 100U, uint32_t offMs = 100U) const;

    /** Play a canned status pattern (blocks for the pattern duration). */
    void indicate(StatusPattern pattern, uint32_t pauseMs = 1000U) const;

private:
    GPIO gpio_;
    bool reverse_ = false;
};
