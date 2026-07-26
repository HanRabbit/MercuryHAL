#pragma once

#include "gpio.h"

/* GPIO 最大注册设备数量 */
#define GPIO_MAX_DEVICE 16

class GPIO {
public:
    using Callback_t = void (*)(GPIO *);

    GPIO(GPIO_TypeDef *GPIOx, uint16_t pin, Callback_t callback = nullptr);

    void setCallback(Callback_t cb);

    void togglePin() const;

    void set() const;

    void reset() const;

    [[nodiscard]] GPIO_PinState read() const;

    static void dispatchEXTI(uint16_t gpio_pin);

private:
    GPIO_TypeDef *GPIOx = nullptr; /* GPIO Port */
    uint16_t pin = 0; /* GPIO Pin */
    Callback_t callback = nullptr; /* GPIO EXTI 回调函数指针 */

    static GPIO *gpioInstances[GPIO_MAX_DEVICE]; /* GPIO 实例静态数组 */
    static uint8_t gpioInstanceCount; /* GPIO 实例计数 */
};
