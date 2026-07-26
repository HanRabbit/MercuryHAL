#pragma once

#include "i2c.h"

/* IIC 最大注册设备数量 */
#define IIC_MAX_DEVICE 4

class IIC {
public:
    using Callback_t = void (*)(IIC *);

    /* 工作模式 */
    enum class WorkMode : uint8_t {
        BLOCKING  = 0, /* 阻塞模式 */
        INTERRUPT = 1, /* 中断模式 */
        DMA       = 2  /* DMA 模式 */
    };

    IIC(I2C_HandleTypeDef *hi2c);

    /* 工作模式设置 */
    void setWorkMode(WorkMode mode);
    [[nodiscard]] WorkMode getWorkMode() const;

    /* 主机发送 */
    HAL_StatusTypeDef transmit(uint16_t devAddr, uint8_t *pData,
                               uint16_t size, uint32_t timeout = 100);

    /* 主机接收 */
    HAL_StatusTypeDef receive(uint16_t devAddr, uint8_t *pData,
                              uint16_t size, uint32_t timeout = 100);

    /* 设备寄存器写入 */
    HAL_StatusTypeDef accessMem(uint16_t devAddr, uint16_t memAddr,
                                uint16_t memAddrSize,
                                uint8_t *pData, uint16_t size,
                                uint32_t timeout = 100);

    /* 设备寄存器读取 */
    HAL_StatusTypeDef accessMemRead(uint16_t devAddr, uint16_t memAddr,
                                    uint16_t memAddrSize,
                                    uint8_t *pData, uint16_t size,
                                    uint32_t timeout = 100);

    /* 设备就绪检测 */
    HAL_StatusTypeDef isDeviceReady(uint16_t devAddr, uint32_t trials = 2,
                                    uint32_t timeout = 100);

    /* I2C 总线恢复（时钟拉伸卡死后释放从机） */
    HAL_StatusTypeDef recoverBus();

    /* 获取底层 HAL 句柄 */
    [[nodiscard]] I2C_HandleTypeDef *getHandle() const;

    /* 获取 HAL 状态 */
    [[nodiscard]] HAL_I2C_StateTypeDef getState() const;

    /* 获取 HAL 错误码 */
    [[nodiscard]] uint32_t getError() const;

    /* 回调设置 */
    void setTxCpltCallback(Callback_t cb);
    void setRxCpltCallback(Callback_t cb);
    void setMemTxCpltCallback(Callback_t cb);
    void setMemRxCpltCallback(Callback_t cb);
    void setErrorCallback(Callback_t cb);
    void setAbortCpltCallback(Callback_t cb);

    /* 根据 HAL 句柄查找 IIC 实例 */
    static IIC *getInstance(I2C_HandleTypeDef *hi2c);

    /* HAL 回调分发 */
    static void dispatchMasterTxCplt(I2C_HandleTypeDef *hi2c);
    static void dispatchMasterRxCplt(I2C_HandleTypeDef *hi2c);
    static void dispatchMemTxCplt(I2C_HandleTypeDef *hi2c);
    static void dispatchMemRxCplt(I2C_HandleTypeDef *hi2c);
    static void dispatchError(I2C_HandleTypeDef *hi2c);
    static void dispatchAbortCplt(I2C_HandleTypeDef *hi2c);

private:
    I2C_HandleTypeDef *m_hi2c = nullptr; /* HAL I2C 句柄 */
    WorkMode m_workMode = WorkMode::BLOCKING; /* 当前工作模式 */

    Callback_t m_txCpltCallback = nullptr; /* 发送完成回调 */
    Callback_t m_rxCpltCallback = nullptr; /* 接收完成回调 */
    Callback_t m_memTxCpltCallback = nullptr; /* 内存写入完成回调 */
    Callback_t m_memRxCpltCallback = nullptr; /* 内存读取完成回调 */
    Callback_t m_errorCallback = nullptr; /* 错误回调 */
    Callback_t m_abortCpltCallback = nullptr; /* 中止完成回调 */

    static IIC *s_instances[IIC_MAX_DEVICE]; /* IIC 实例静态数组 */
    static uint8_t s_instanceCount; /* IIC 实例计数 */

    /* 工作模式分发函数 */
    HAL_StatusTypeDef transmitBlocking(uint16_t devAddr, uint8_t *pData,
                                       uint16_t size, uint32_t timeout);
    HAL_StatusTypeDef transmitIT(uint16_t devAddr, uint8_t *pData,
                                 uint16_t size);
    HAL_StatusTypeDef transmitDMA(uint16_t devAddr, uint8_t *pData,
                                  uint16_t size);

    HAL_StatusTypeDef receiveBlocking(uint16_t devAddr, uint8_t *pData,
                                      uint16_t size, uint32_t timeout);
    HAL_StatusTypeDef receiveIT(uint16_t devAddr, uint8_t *pData,
                                uint16_t size);
    HAL_StatusTypeDef receiveDMA(uint16_t devAddr, uint8_t *pData,
                                 uint16_t size);

    HAL_StatusTypeDef memWriteBlocking(uint16_t devAddr, uint16_t memAddr,
                                       uint16_t memAddrSize,
                                       uint8_t *pData, uint16_t size,
                                       uint32_t timeout);
    HAL_StatusTypeDef memWriteIT(uint16_t devAddr, uint16_t memAddr,
                                 uint16_t memAddrSize,
                                 uint8_t *pData, uint16_t size);
    HAL_StatusTypeDef memWriteDMA(uint16_t devAddr, uint16_t memAddr,
                                  uint16_t memAddrSize,
                                  uint8_t *pData, uint16_t size);

    HAL_StatusTypeDef memReadBlocking(uint16_t devAddr, uint16_t memAddr,
                                      uint16_t memAddrSize,
                                      uint8_t *pData, uint16_t size,
                                      uint32_t timeout);
    HAL_StatusTypeDef memReadIT(uint16_t devAddr, uint16_t memAddr,
                                uint16_t memAddrSize,
                                uint8_t *pData, uint16_t size);
    HAL_StatusTypeDef memReadDMA(uint16_t devAddr, uint16_t memAddr,
                                 uint16_t memAddrSize,
                                 uint8_t *pData, uint16_t size);
};
