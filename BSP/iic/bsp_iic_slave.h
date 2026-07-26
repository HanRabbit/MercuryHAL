#pragma once

#include "i2c.h"
#include <cstdint>

/**
 * Generic I2C slave listen engine (register pointer protocol).
 *
 * Master write: REG [, VALUE]
 * Master read:  returns value of current REG via RegReadFn
 *
 * Protocol binding (KEYS/FORCE etc.) stays in Application.
 */
class IIC_Slave {
public:
    using RegReadFn = uint8_t (*)(uint8_t reg);
    using RegWriteFn = void (*)(uint8_t reg, uint8_t value);

    explicit IIC_Slave(I2C_HandleTypeDef *hi2c);

    void setRegHandlers(RegReadFn readFn, RegWriteFn writeFn);

    HAL_StatusTypeDef startListen();

    void poll();

    [[nodiscard]] I2C_HandleTypeDef *getHandle() const;

    static IIC_Slave *getInstance(I2C_HandleTypeDef *hi2c);

    static void dispatchAddr(I2C_HandleTypeDef *hi2c, uint8_t transferDirection,
                             uint16_t addrMatchCode);

    static void dispatchSlaveRxCplt(I2C_HandleTypeDef *hi2c);

    static void dispatchSlaveTxCplt(I2C_HandleTypeDef *hi2c);

    static void dispatchListenCplt(I2C_HandleTypeDef *hi2c);

    static void dispatchError(I2C_HandleTypeDef *hi2c);

private:
    static constexpr uint8_t MAX_DEVICE = 2U;

    I2C_HandleTypeDef *m_hi2c = nullptr;
    RegReadFn m_readFn = nullptr;
    RegWriteFn m_writeFn = nullptr;

    uint8_t m_reg = 0U;
    uint8_t m_rxByte = 0U;
    uint8_t m_txByte = 0U;
    uint8_t m_rxLen = 0U;
    uint8_t m_rxBuf[2] = {};
    bool m_awaitingData = false;

    static IIC_Slave *s_instances[MAX_DEVICE];
    static uint8_t s_instanceCount;

    void onAddr(uint8_t transferDirection);

    void onRxCplt();

    void onListenCplt();

    void onError();

    void armListen();

    uint8_t readReg(uint8_t reg) const;

    void writeReg(uint8_t reg, uint8_t value) const;
};
