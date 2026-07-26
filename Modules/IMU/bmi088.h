#pragma once

#include "bsp_spi.h"
#include "bsp_gpio.h"

class BMI088 {
public:
    struct Config {
        SPI *spi;

        GPIO *accCs;
        GPIO *gyroCs;

        GPIO *accInt;
        GPIO *gyroInt;
    };

    struct Accel {
        float x;
        float y;
        float z;
    };

    struct Gyro {
        float x;
        float y;
        float z;
    };

    struct Data {
        Accel accel;
        Gyro gyro;
        float temperature;
    };

    explicit BMI088(const Config &config);

    bool init() const;

    void update();

    [[nodiscard]]
    const Data &getData() const {
        return data;
    }

    /* 获取调试计数器 */
    struct DebugCounters {
        uint32_t accelIrq;
        uint32_t gyroIrq;
        uint32_t accelRead;
        uint32_t gyroRead;
    };

    [[nodiscard]]
    DebugCounters getDebugCounters() const {
        return {accelIrqCount, gyroIrqCount, accelReadCount, gyroReadCount};
    }

    /* SPI DMA 收发完成回调，由全局 HAL_SPI_TxRxCpltCallback 转发 */
    void onSpiTxRxComplete(SPI_HandleTypeDef *hspi);

    /* SPI 错误回调，释放总线并复位状态，避免一次错误导致后续读取永久阻塞 */
    void onSpiError(SPI_HandleTypeDef *hspi);

    static BMI088 *getInstance() {
        return instance;
    }

private:
    Config config;

    Data data{};

    volatile bool accelUpdateFlag = false;
    volatile bool gyroUpdateFlag = false;

    /* 调试计数器：统计各数据源实际读取次数和中断次数 */
    volatile uint32_t accelReadCount = 0;
    volatile uint32_t gyroReadCount = 0;
    volatile uint32_t accelIrqCount = 0;
    volatile uint32_t gyroIrqCount = 0;

    /* 当前正在进行的 DMA 读取目标 */
    enum class Reading : uint8_t {
        None,
        Accel,
        Gyro,
        Temp,
    };

    volatile Reading reading = Reading::None;

    /*
     * DMA 缓冲区必须常驻内存（不能放在栈上，否则异步传输期间栈帧已失效），
     * 且需位于 DMA1 可访问的内存域。BMI088 为堆对象，位于 RAM_D1(0x24000000)，满足要求。
     */
    uint8_t txBuf[8]{};
    uint8_t rxBuf[8]{};

    /*
     * 温度更新缓慢（BMI088 温度寄存器约每 1.28s 刷新一次），无需每周期都读。
     * tempCounter 在每次 update() 累加，达到 tempReadInterval 时将 tempReadFlag
     * 置位；实际读取在加速度计 DMA 完成后链式发起，避免因陀螺仪持续占用 SPI 而饥饿。
     */
    static constexpr uint16_t tempReadInterval = 1000;
    uint16_t tempCounter = 0;
    volatile bool tempReadFlag = false;

    static void accelIRQ(GPIO *gpio);

    static void gyroIRQ(GPIO *gpio);

    /* 通过 DMA 发起一次读取 */
    void startAccelRead();

    void startGyroRead();

    void startTempRead();

    /* 阻塞式寄存器访问，仅用于初始化阶段（不依赖中断，可在关中断时工作） */
    [[nodiscard]] uint8_t readAccelReg(uint8_t reg) const;

    void writeAccelReg(uint8_t reg, uint8_t value) const;

    [[nodiscard]] uint8_t readGyroReg(uint8_t reg) const;

    void writeGyroReg(uint8_t reg, uint8_t value) const;

    static BMI088 *instance;
};
