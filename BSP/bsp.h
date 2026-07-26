#pragma once

/**
 * MercuryBSP umbrella header.
 *
 * Include only what you need in application code for faster builds.
 * This header pulls the portable core set. Optional peripherals
 * (FDCAN / I2C / USART / SPI / PWM / USB) are included when the
 * corresponding MERCURY_BSP_HAS_* macros are defined by CMake, or
 * you can include their headers directly.
 */

#include "bsp_dwt.h"
#include "bsp_gpio.h"
#include "bsp_log.h"

#if defined(MERCURY_BSP_HAS_IIC)
#include "bsp_iic.h"
#include "bsp_iic_slave.h"
#endif

#if defined(MERCURY_BSP_HAS_CAN)
#include "bsp_can.h"
#endif

#if defined(MERCURY_BSP_HAS_USART)
#include "bsp_usart.h"
#endif

#if defined(MERCURY_BSP_HAS_SPI)
#include "bsp_spi.h"
#endif

#if defined(MERCURY_BSP_HAS_PWM)
#include "bsp_pwm.h"
#endif

#if defined(MERCURY_BSP_HAS_USB)
#include "bsp_usb.h"
#endif

#include "SEGGER_RTT.h"

/**
 * Minimal portable BSP bring-up:
 *  - DWT cycle counter (pass HCLK in Hz)
 *  - SEGGER RTT for LOG_* / JScope
 *
 * Call once after HAL_Init() / SystemClock_Config().
 * Peripheral-specific init (FDCAN::init, USART::init, …) stays with the app.
 */
inline void MercuryBSP_Init(const uint32_t cpuFreqHz) {
    DWT_Time::init(cpuFreqHz);
    SEGGER_RTT_Init();
}
