#include "bsp_iic_slave.h"

IIC_Slave *IIC_Slave::s_instances[MAX_DEVICE] = {};
uint8_t IIC_Slave::s_instanceCount = 0U;

IIC_Slave::IIC_Slave(I2C_HandleTypeDef *const hi2c) : m_hi2c(hi2c) {
    if (s_instanceCount < MAX_DEVICE) {
        s_instances[s_instanceCount++] = this;
    }
}

void IIC_Slave::setRegHandlers(const RegReadFn readFn, const RegWriteFn writeFn) {
    m_readFn = readFn;
    m_writeFn = writeFn;
}

HAL_StatusTypeDef IIC_Slave::startListen() {
    if (m_hi2c == nullptr) {
        return HAL_ERROR;
    }
    m_rxLen = 0U;
    m_awaitingData = false;
    return HAL_I2C_EnableListen_IT(m_hi2c);
}

void IIC_Slave::poll() {
    if (m_hi2c == nullptr) {
        return;
    }

    const uint32_t state = static_cast<uint32_t>(HAL_I2C_GetState(m_hi2c));
    if ((state & static_cast<uint32_t>(HAL_I2C_STATE_LISTEN)) != 0U) {
        return;
    }
    if (state == static_cast<uint32_t>(HAL_I2C_STATE_READY)) {
        armListen();
    }
}

I2C_HandleTypeDef *IIC_Slave::getHandle() const {
    return m_hi2c;
}

IIC_Slave *IIC_Slave::getInstance(I2C_HandleTypeDef *const hi2c) {
    for (uint8_t i = 0U; i < s_instanceCount; ++i) {
        if (s_instances[i] != nullptr && s_instances[i]->m_hi2c == hi2c) {
            return s_instances[i];
        }
    }
    return nullptr;
}

void IIC_Slave::armListen() {
    if (m_hi2c == nullptr) {
        return;
    }
    if (HAL_I2C_EnableListen_IT(m_hi2c) != HAL_OK) {
        (void) HAL_I2C_GetError(m_hi2c);
    }
}

uint8_t IIC_Slave::readReg(const uint8_t reg) const {
    if (m_readFn != nullptr) {
        return m_readFn(reg);
    }
    return 0U;
}

void IIC_Slave::writeReg(const uint8_t reg, const uint8_t value) const {
    if (m_writeFn != nullptr) {
        m_writeFn(reg, value);
    }
}

void IIC_Slave::onAddr(const uint8_t transferDirection) {
    if (transferDirection == I2C_DIRECTION_TRANSMIT) {
        m_rxLen = 0U;
        m_awaitingData = false;
        (void) HAL_I2C_Slave_Seq_Receive_IT(m_hi2c, &m_rxByte, 1U, I2C_FIRST_FRAME);
    } else {
        m_txByte = readReg(m_reg);
        (void) HAL_I2C_Slave_Seq_Transmit_IT(m_hi2c, &m_txByte, 1U, I2C_LAST_FRAME);
    }
}

void IIC_Slave::onRxCplt() {
    if (m_rxLen < sizeof(m_rxBuf)) {
        m_rxBuf[m_rxLen++] = m_rxByte;
    }

    if (m_rxLen == 1U) {
        m_reg = m_rxBuf[0];
        m_awaitingData = true;
        (void) HAL_I2C_Slave_Seq_Receive_IT(m_hi2c, &m_rxByte, 1U, I2C_NEXT_FRAME);
    } else if (m_rxLen >= 2U) {
        writeReg(m_reg, m_rxBuf[1]);
        m_awaitingData = false;
    }
}

void IIC_Slave::onListenCplt() {
    if (m_awaitingData && m_rxLen == 1U) {
        m_reg = m_rxBuf[0];
    }
    m_awaitingData = false;
    m_rxLen = 0U;
    armListen();
}

void IIC_Slave::onError() {
    m_awaitingData = false;
    m_rxLen = 0U;
    (void) HAL_I2C_GetError(m_hi2c);
    armListen();
}

void IIC_Slave::dispatchAddr(I2C_HandleTypeDef *const hi2c,
                             const uint8_t transferDirection,
                             const uint16_t addrMatchCode) {
    (void) addrMatchCode;
    IIC_Slave *const inst = getInstance(hi2c);
    if (inst != nullptr) {
        inst->onAddr(transferDirection);
    }
}

void IIC_Slave::dispatchSlaveRxCplt(I2C_HandleTypeDef *const hi2c) {
    IIC_Slave *const inst = getInstance(hi2c);
    if (inst != nullptr) {
        inst->onRxCplt();
    }
}

void IIC_Slave::dispatchSlaveTxCplt(I2C_HandleTypeDef *const hi2c) {
    (void) hi2c;
}

void IIC_Slave::dispatchListenCplt(I2C_HandleTypeDef *const hi2c) {
    IIC_Slave *const inst = getInstance(hi2c);
    if (inst != nullptr) {
        inst->onListenCplt();
    }
}

void IIC_Slave::dispatchError(I2C_HandleTypeDef *const hi2c) {
    IIC_Slave *const inst = getInstance(hi2c);
    if (inst != nullptr) {
        inst->onError();
    }
}

extern "C" void HAL_I2C_AddrCallback(I2C_HandleTypeDef *hi2c,
                                     uint8_t TransferDirection,
                                     uint16_t AddrMatchCode) {
    IIC_Slave::dispatchAddr(hi2c, TransferDirection, AddrMatchCode);
}

extern "C" void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c) {
    IIC_Slave::dispatchSlaveRxCplt(hi2c);
}

extern "C" void HAL_I2C_SlaveTxCpltCallback(I2C_HandleTypeDef *hi2c) {
    IIC_Slave::dispatchSlaveTxCplt(hi2c);
}

extern "C" void HAL_I2C_ListenCpltCallback(I2C_HandleTypeDef *hi2c) {
    IIC_Slave::dispatchListenCplt(hi2c);
}
