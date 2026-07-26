#include "led.h"

#include "cmsis_os2.h"

LED::LED(const GPIO *const gpio, const bool reverse)
    : gpio_(*gpio), reverse_(reverse) {
}

void LED::on() const {
    if (reverse_) {
        gpio_.reset();
    } else {
        gpio_.set();
    }
}

void LED::off() const {
    if (reverse_) {
        gpio_.set();
    } else {
        gpio_.reset();
    }
}

void LED::toggle() const {
    gpio_.togglePin();
}

void LED::blinkBurst(const unsigned count, const uint32_t onMs,
                     const uint32_t offMs) const {
    for (unsigned i = 0U; i < count; ++i) {
        on();
        osDelay(onMs);
        off();
        osDelay(offMs);
    }
}

void LED::blinkOnce(const uint32_t onMs, const uint32_t offMs) const {
    on();
    osDelay(onMs);
    off();
    osDelay(offMs);
}

void LED::indicate(const StatusPattern pattern, const uint32_t pauseMs) const {
    constexpr uint32_t kOnMs = 80U;
    constexpr uint32_t kOffMs = 120U;

    switch (pattern) {
    case StatusPattern::Error:
        blinkOnce(100U, 100U);
        break;
    case StatusPattern::NoDevice:
        blinkBurst(4U, kOnMs, kOffMs);
        osDelay(pauseMs);
        break;
    case StatusPattern::Active:
        blinkBurst(3U, kOnMs, kOffMs);
        osDelay(pauseMs);
        break;
    case StatusPattern::Ok:
    default:
        blinkBurst(2U, kOnMs, kOffMs);
        osDelay(pauseMs);
        break;
    }
}
