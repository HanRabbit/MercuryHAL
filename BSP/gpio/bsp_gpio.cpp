#include "bsp_gpio.h"
#include "bsp_log.h"

/* GPIO 实例静态数组与计数（避免在 early runtime 使用堆） */
GPIO *GPIO::gpioInstances[GPIO_MAX_DEVICE] = {nullptr};
uint8_t GPIO::gpioInstanceCount = 0;

/**
 * @brief GPIO 构造函数
 * @param GPIOx GPIO 端口组
 * @param pin GPIO 端口号
 * @param callback GPIO EXTI 回调函数
 */
GPIO::GPIO(GPIO_TypeDef *GPIOx, const uint16_t pin,
           const Callback_t callback) : GPIOx(GPIOx), pin(pin), callback(callback) {
    if (gpioInstanceCount < GPIO_MAX_DEVICE) {
        gpioInstances[gpioInstanceCount++] = this;
    } else {
        /* 超出 GPIO 最大注册限制，记录错误并返回 */
        LOG_ERROR("[GPIO] Instance count exceeds maximum limit!");
    }
}

/**
 * @brief 设置 GPIO EXTI 回调函数
 * @param cb 回调函数指针
 */
void GPIO::setCallback(const Callback_t cb) {
    callback = cb;
}

/**
 * @brief 切换 GPIO 引脚状态
 */
void GPIO::togglePin() const {
    HAL_GPIO_TogglePin(GPIOx, pin);
}

/**
 * @brief 设置 GPIO 引脚为高电平
 */
void GPIO::set() const {
    HAL_GPIO_WritePin(GPIOx, pin, GPIO_PIN_SET);
}

/**
 * @brief 设置 GPIO 引脚为低电平
 */
void GPIO::reset() const {
    HAL_GPIO_WritePin(GPIOx, pin, GPIO_PIN_RESET);
}

/**
 * @brief 读取 GPIO 引脚状态
 * @return GPIO_PinState 当前引脚状态
 */
GPIO_PinState GPIO::read() const {
    return HAL_GPIO_ReadPin(GPIOx, pin);
}

/**
 * @brief 分发 GPIO EXTI 中断事件
 * @param gpio_pin 触发中断的 GPIO 引脚号
 */
void GPIO::dispatchEXTI(const uint16_t gpio_pin) {
    for (uint8_t i = 0; i < gpioInstanceCount; ++i) {
        if (GPIO *gpio = gpioInstances[i]; gpio && gpio->pin == gpio_pin && gpio->callback) {
            gpio->callback(gpio);
        }
    }
}

/* 中断服务函数一定要加 extern "C" */
extern "C" {
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    GPIO::dispatchEXTI(GPIO_Pin);
}
}
