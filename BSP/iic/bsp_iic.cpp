#include "bsp_iic.h"
#include "bsp_dwt.h"
#include "bsp_iic_slave.h"
#include "bsp_log.h"

/* IIC 实例静态数组与计数（避免在 early runtime 使用堆） */
IIC *IIC::s_instances[IIC_MAX_DEVICE] = {nullptr};
uint8_t IIC::s_instanceCount = 0;

/**
 * @brief IIC 构造函数，自动注册到静态实例数组
 * @param hi2c HAL I2C 句柄指针
 */
IIC::IIC(I2C_HandleTypeDef *const hi2c) : m_hi2c(hi2c) {
    if (s_instanceCount < IIC_MAX_DEVICE) {
        s_instances[s_instanceCount++] = this;
    } else {
        LOG_ERROR("[IIC] Instance count exceeds maximum limit!");
    }
}

/**
 * @brief 设置工作模式
 * @param mode BLOCKING / INTERRUPT / DMA
 */
void IIC::setWorkMode(const WorkMode mode) {
    m_workMode = mode;
}

/**
 * @brief 获取当前工作模式
 * @return WorkMode 当前工作模式
 */
IIC::WorkMode IIC::getWorkMode() const {
    return m_workMode;
}

/**
 * @brief 主机发送数据
 * @param devAddr 7 位设备地址（HAL 左移格式）
 * @param pData 发送数据缓冲区
 * @param size 发送字节数
 * @param timeout 超时时间（ms，仅阻塞模式有效）
 * @return HAL status
 */
HAL_StatusTypeDef IIC::transmit(const uint16_t devAddr, uint8_t *const pData,
                                const uint16_t size, const uint32_t timeout) {
    if (m_hi2c == nullptr || pData == nullptr || size == 0) {
        return HAL_ERROR;
    }

    switch (m_workMode) {
    case WorkMode::BLOCKING:  return transmitBlocking(devAddr, pData, size, timeout);
    case WorkMode::INTERRUPT: return transmitIT(devAddr, pData, size);
    case WorkMode::DMA:       return transmitDMA(devAddr, pData, size);
    default:                  return HAL_ERROR;
    }
}

/** @brief 阻塞主机发送 */
HAL_StatusTypeDef IIC::transmitBlocking(const uint16_t devAddr, uint8_t *const pData,
                                        const uint16_t size, const uint32_t timeout) {
    return HAL_I2C_Master_Transmit(m_hi2c, devAddr, pData, size, timeout);
}

/** @brief 中断主机发送 */
HAL_StatusTypeDef IIC::transmitIT(const uint16_t devAddr, uint8_t *const pData,
                                  const uint16_t size) {
    return HAL_I2C_Master_Transmit_IT(m_hi2c, devAddr, pData, size);
}

/** @brief DMA 主机发送 */
HAL_StatusTypeDef IIC::transmitDMA(const uint16_t devAddr, uint8_t *const pData,
                                   const uint16_t size) {
    return HAL_I2C_Master_Transmit_DMA(m_hi2c, devAddr, pData, size);
}

/**
 * @brief 主机接收数据
 * @param devAddr 7 位设备地址（HAL 左移格式）
 * @param pData 接收数据缓冲区
 * @param size 接收字节数
 * @param timeout 超时时间（ms，仅阻塞模式有效）
 * @return HAL status
 */
HAL_StatusTypeDef IIC::receive(const uint16_t devAddr, uint8_t *const pData,
                               const uint16_t size, const uint32_t timeout) {
    if (m_hi2c == nullptr || pData == nullptr || size == 0) {
        return HAL_ERROR;
    }

    switch (m_workMode) {
    case WorkMode::BLOCKING:  return receiveBlocking(devAddr, pData, size, timeout);
    case WorkMode::INTERRUPT: return receiveIT(devAddr, pData, size);
    case WorkMode::DMA:       return receiveDMA(devAddr, pData, size);
    default:                  return HAL_ERROR;
    }
}

/** @brief 阻塞主机接收 */
HAL_StatusTypeDef IIC::receiveBlocking(const uint16_t devAddr, uint8_t *const pData,
                                       const uint16_t size, const uint32_t timeout) {
    return HAL_I2C_Master_Receive(m_hi2c, devAddr, pData, size, timeout);
}

/** @brief 中断主机接收 */
HAL_StatusTypeDef IIC::receiveIT(const uint16_t devAddr, uint8_t *const pData,
                                 const uint16_t size) {
    return HAL_I2C_Master_Receive_IT(m_hi2c, devAddr, pData, size);
}

/** @brief DMA 主机接收 */
HAL_StatusTypeDef IIC::receiveDMA(const uint16_t devAddr, uint8_t *const pData,
                                  const uint16_t size) {
    return HAL_I2C_Master_Receive_DMA(m_hi2c, devAddr, pData, size);
}

/**
 * @brief 向设备寄存器写入数据
 * @param devAddr 7 位设备地址（HAL 格式）
 * @param memAddr 设备内部寄存器地址
 * @param memAddrSize 寄存器地址宽度（I2C_MEMADD_SIZE_8BIT / I2C_MEMADD_SIZE_16BIT）
 * @param pData 写入数据缓冲区
 * @param size 写入字节数
 * @param timeout 超时时间（ms，仅阻塞模式有效）
 * @return HAL status
 */
HAL_StatusTypeDef IIC::accessMem(const uint16_t devAddr, const uint16_t memAddr,
                                 const uint16_t memAddrSize,
                                 uint8_t *const pData, const uint16_t size,
                                 const uint32_t timeout) {
    if (m_hi2c == nullptr || pData == nullptr || size == 0) {
        return HAL_ERROR;
    }

    switch (m_workMode) {
    case WorkMode::BLOCKING:
        return memWriteBlocking(devAddr, memAddr, memAddrSize, pData, size, timeout);
    case WorkMode::INTERRUPT:
        return memWriteIT(devAddr, memAddr, memAddrSize, pData, size);
    case WorkMode::DMA:
        return memWriteDMA(devAddr, memAddr, memAddrSize, pData, size);
    default:
        return HAL_ERROR;
    }
}

/** @brief 阻塞设备寄存器写入 */
HAL_StatusTypeDef IIC::memWriteBlocking(const uint16_t devAddr, const uint16_t memAddr,
                                        const uint16_t memAddrSize,
                                        uint8_t *const pData, const uint16_t size,
                                        const uint32_t timeout) {
    return HAL_I2C_Mem_Write(m_hi2c, devAddr, memAddr, memAddrSize,
                             pData, size, timeout);
}

/** @brief 中断设备寄存器写入 */
HAL_StatusTypeDef IIC::memWriteIT(const uint16_t devAddr, const uint16_t memAddr,
                                  const uint16_t memAddrSize,
                                  uint8_t *const pData, const uint16_t size) {
    return HAL_I2C_Mem_Write_IT(m_hi2c, devAddr, memAddr, memAddrSize,
                                pData, size);
}

/** @brief DMA 设备寄存器写入 */
HAL_StatusTypeDef IIC::memWriteDMA(const uint16_t devAddr, const uint16_t memAddr,
                                   const uint16_t memAddrSize,
                                   uint8_t *const pData, const uint16_t size) {
    return HAL_I2C_Mem_Write_DMA(m_hi2c, devAddr, memAddr, memAddrSize,
                                 pData, size);
}

/**
 * @brief 从设备寄存器读取数据
 * @param devAddr 7 位设备地址（HAL 格式）
 * @param memAddr 设备内部寄存器地址
 * @param memAddrSize 寄存器地址宽度（I2C_MEMADD_SIZE_8BIT / I2C_MEMADD_SIZE_16BIT）
 * @param pData 读取数据缓冲区
 * @param size 读取字节数
 * @param timeout 超时时间（ms，仅阻塞模式有效）
 * @return HAL status
 */
HAL_StatusTypeDef IIC::accessMemRead(const uint16_t devAddr, const uint16_t memAddr,
                                     const uint16_t memAddrSize,
                                     uint8_t *const pData, const uint16_t size,
                                     const uint32_t timeout) {
    if (m_hi2c == nullptr || pData == nullptr || size == 0) {
        return HAL_ERROR;
    }

    switch (m_workMode) {
    case WorkMode::BLOCKING:
        return memReadBlocking(devAddr, memAddr, memAddrSize, pData, size, timeout);
    case WorkMode::INTERRUPT:
        return memReadIT(devAddr, memAddr, memAddrSize, pData, size);
    case WorkMode::DMA:
        return memReadDMA(devAddr, memAddr, memAddrSize, pData, size);
    default:
        return HAL_ERROR;
    }
}

/** @brief 阻塞设备寄存器读取 */
HAL_StatusTypeDef IIC::memReadBlocking(const uint16_t devAddr, const uint16_t memAddr,
                                       const uint16_t memAddrSize,
                                       uint8_t *const pData, const uint16_t size,
                                       const uint32_t timeout) {
    return HAL_I2C_Mem_Read(m_hi2c, devAddr, memAddr, memAddrSize,
                            pData, size, timeout);
}

/** @brief 中断设备寄存器读取 */
HAL_StatusTypeDef IIC::memReadIT(const uint16_t devAddr, const uint16_t memAddr,
                                 const uint16_t memAddrSize,
                                 uint8_t *const pData, const uint16_t size) {
    return HAL_I2C_Mem_Read_IT(m_hi2c, devAddr, memAddr, memAddrSize,
                               pData, size);
}

/** @brief DMA 设备寄存器读取 */
HAL_StatusTypeDef IIC::memReadDMA(const uint16_t devAddr, const uint16_t memAddr,
                                  const uint16_t memAddrSize,
                                  uint8_t *const pData, const uint16_t size) {
    return HAL_I2C_Mem_Read_DMA(m_hi2c, devAddr, memAddr, memAddrSize,
                                pData, size);
}

/**
 * @brief 检测设备是否就绪
 * @param devAddr 7 位设备地址
 * @param trials 尝试次数
 * @param timeout 单次超时时间（ms）
 * @return HAL_OK 设备就绪，HAL_TIMEOUT / HAL_BUSY 设备未就绪
 */
HAL_StatusTypeDef IIC::isDeviceReady(const uint16_t devAddr, const uint32_t trials,
                                     const uint32_t timeout) {
    if (m_hi2c == nullptr) {
        return HAL_ERROR;
    }
    return HAL_I2C_IsDeviceReady(m_hi2c, devAddr, trials, timeout);
}

/**
 * @brief I2C 总线恢复：切换为 GPIO 开漏，补时钟释放卡住的从机，再重新初始化外设
 * @return HAL_OK 成功，HAL_ERROR 句柄无效或引脚未知
 *
 * @note 默认按本工程引脚映射：I2C1=PB6/PB7，I2C2=PB10/PB11。
 */
HAL_StatusTypeDef IIC::recoverBus() {
    if (m_hi2c == nullptr || m_hi2c->Instance == nullptr) {
        return HAL_ERROR;
    }

    GPIO_TypeDef *sclPort = nullptr;
    GPIO_TypeDef *sdaPort = nullptr;
    uint16_t sclPin = 0;
    uint16_t sdaPin = 0;

    if (m_hi2c->Instance == I2C1) {
        sclPort = GPIOB;
        sdaPort = GPIOB;
        sclPin = GPIO_PIN_6;
        sdaPin = GPIO_PIN_7;
    } else if (m_hi2c->Instance == I2C2) {
        sclPort = GPIOB;
        sdaPort = GPIOB;
        sclPin = GPIO_PIN_10;
        sdaPin = GPIO_PIN_11;
    } else {
        LOG_ERROR("[IIC] recoverBus: unsupported I2C instance");
        return HAL_ERROR;
    }

    (void)HAL_I2C_DeInit(m_hi2c);

    __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_InitTypeDef gpio = {0};
    gpio.Mode = GPIO_MODE_OUTPUT_OD;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;

    gpio.Pin = sclPin;
    HAL_GPIO_Init(sclPort, &gpio);
    gpio.Pin = sdaPin;
    HAL_GPIO_Init(sdaPort, &gpio);

    HAL_GPIO_WritePin(sclPort, sclPin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(sdaPort, sdaPin, GPIO_PIN_SET);
    DWT_Time::delay(0.001F);

    for (uint8_t i = 0; i < 9U; ++i) {
        if (HAL_GPIO_ReadPin(sdaPort, sdaPin) == GPIO_PIN_SET) {
            break;
        }
        HAL_GPIO_WritePin(sclPort, sclPin, GPIO_PIN_RESET);
        DWT_Time::delay(0.001F);
        HAL_GPIO_WritePin(sclPort, sclPin, GPIO_PIN_SET);
        DWT_Time::delay(0.001F);
    }

    HAL_GPIO_WritePin(sdaPort, sdaPin, GPIO_PIN_RESET);
    DWT_Time::delay(0.001F);
    HAL_GPIO_WritePin(sclPort, sclPin, GPIO_PIN_SET);
    DWT_Time::delay(0.001F);
    HAL_GPIO_WritePin(sdaPort, sdaPin, GPIO_PIN_SET);
    DWT_Time::delay(0.001F);

    if (HAL_I2C_Init(m_hi2c) != HAL_OK) {
        LOG_ERROR("[IIC] recoverBus: HAL_I2C_Init failed");
        return HAL_ERROR;
    }

    return HAL_OK;
}

/**
 * @brief 获取底层 HAL I2C 句柄
 * @return I2C_HandleTypeDef 指针
 */
I2C_HandleTypeDef *IIC::getHandle() const {
    return m_hi2c;
}

/**
 * @brief 获取 HAL 外设状态
 * @return HAL_I2C_StateTypeDef 当前状态
 */
HAL_I2C_StateTypeDef IIC::getState() const {
    if (m_hi2c == nullptr) {
        return HAL_I2C_STATE_RESET;
    }
    return HAL_I2C_GetState(m_hi2c);
}

/**
 * @brief 获取 HAL 错误码
 * @return uint32_t 错误码位掩码
 */
uint32_t IIC::getError() const {
    if (m_hi2c == nullptr) {
        return 0;
    }
    return HAL_I2C_GetError(m_hi2c);
}

/**
 * @brief 设置主机发送完成回调
 * @param cb 回调函数指针
 */
void IIC::setTxCpltCallback(const Callback_t cb) {
    m_txCpltCallback = cb;
}

/**
 * @brief 设置主机接收完成回调
 * @param cb 回调函数指针
 */
void IIC::setRxCpltCallback(const Callback_t cb) {
    m_rxCpltCallback = cb;
}

/**
 * @brief 设置设备寄存器写入完成回调
 * @param cb 回调函数指针
 */
void IIC::setMemTxCpltCallback(const Callback_t cb) {
    m_memTxCpltCallback = cb;
}

/**
 * @brief 设置设备寄存器读取完成回调
 * @param cb 回调函数指针
 */
void IIC::setMemRxCpltCallback(const Callback_t cb) {
    m_memRxCpltCallback = cb;
}

/**
 * @brief 设置 I2C 错误回调
 * @param cb 回调函数指针
 */
void IIC::setErrorCallback(const Callback_t cb) {
    m_errorCallback = cb;
}

/**
 * @brief 设置中止完成回调
 * @param cb 回调函数指针
 */
void IIC::setAbortCpltCallback(const Callback_t cb) {
    m_abortCpltCallback = cb;
}

/**
 * @brief 根据 HAL 句柄查找对应的 IIC 实例
 * @param hi2c HAL I2C 句柄
 * @return IIC 实例指针，未找到返回 nullptr
 */
IIC *IIC::getInstance(I2C_HandleTypeDef *const hi2c) {
    for (uint8_t i = 0; i < s_instanceCount; ++i) {
        if (s_instances[i] != nullptr && s_instances[i]->m_hi2c == hi2c) {
            return s_instances[i];
        }
    }
    return nullptr;
}

/**
 * @brief 分发主机发送完成事件到对应实例的回调
 * @param hi2c HAL I2C 句柄
 */
void IIC::dispatchMasterTxCplt(I2C_HandleTypeDef *const hi2c) {
    if (IIC *const instance = getInstance(hi2c);
        instance != nullptr && instance->m_txCpltCallback != nullptr) {
        instance->m_txCpltCallback(instance);
    }
}

/**
 * @brief 分发主机接收完成事件到对应实例的回调
 * @param hi2c HAL I2C 句柄
 */
void IIC::dispatchMasterRxCplt(I2C_HandleTypeDef *const hi2c) {
    if (IIC *const instance = getInstance(hi2c);
        instance != nullptr && instance->m_rxCpltCallback != nullptr) {
        instance->m_rxCpltCallback(instance);
    }
}

/**
 * @brief 分发设备寄存器写入完成事件到对应实例的回调
 * @param hi2c HAL I2C 句柄
 */
void IIC::dispatchMemTxCplt(I2C_HandleTypeDef *const hi2c) {
    if (IIC *const instance = getInstance(hi2c);
        instance != nullptr && instance->m_memTxCpltCallback != nullptr) {
        instance->m_memTxCpltCallback(instance);
    }
}

/**
 * @brief 分发设备寄存器读取完成事件到对应实例的回调
 * @param hi2c HAL I2C 句柄
 */
void IIC::dispatchMemRxCplt(I2C_HandleTypeDef *const hi2c) {
    if (IIC *const instance = getInstance(hi2c);
        instance != nullptr && instance->m_memRxCpltCallback != nullptr) {
        instance->m_memRxCpltCallback(instance);
    }
}

/**
 * @brief 分发 I2C 错误事件到对应实例的回调
 * @param hi2c HAL I2C 句柄
 */
void IIC::dispatchError(I2C_HandleTypeDef *const hi2c) {
    if (IIC *const instance = getInstance(hi2c);
        instance != nullptr && instance->m_errorCallback != nullptr) {
        instance->m_errorCallback(instance);
    }
}

/**
 * @brief 分发中止完成事件到对应实例的回调
 * @param hi2c HAL I2C 句柄
 */
void IIC::dispatchAbortCplt(I2C_HandleTypeDef *const hi2c) {
    if (IIC *const instance = getInstance(hi2c);
        instance != nullptr && instance->m_abortCpltCallback != nullptr) {
        instance->m_abortCpltCallback(instance);
    }
}

/* 中断服务函数一定要加 extern "C" */
extern "C" {

void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c) {
    IIC::dispatchMasterTxCplt(hi2c);
}

void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c) {
    IIC::dispatchMasterRxCplt(hi2c);
}

void HAL_I2C_MemTxCpltCallback(I2C_HandleTypeDef *hi2c) {
    IIC::dispatchMemTxCplt(hi2c);
}

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c) {
    IIC::dispatchMemRxCplt(hi2c);
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c) {
    IIC::dispatchError(hi2c);
    IIC_Slave::dispatchError(hi2c);
}

void HAL_I2C_AbortCpltCallback(I2C_HandleTypeDef *hi2c) {
    IIC::dispatchAbortCplt(hi2c);
}

} /* extern "C" */
