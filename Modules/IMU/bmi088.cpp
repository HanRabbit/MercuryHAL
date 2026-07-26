#include "bmi088.h"
#include "bsp_dwt.h"

BMI088 *BMI088::instance = nullptr;

/**
 * @brief BMI088 构造函数，拷贝配置并注册中断回调函数
 * @param config BMI088 配置结构体，包含 SPI 外设、片选 GPIO 和中断 GPIO 的配置
 */
BMI088::BMI088(const Config &config) : config(config) {
    instance = this;
    if (config.accInt)
        config.accInt->setCallback(accelIRQ);
    if (config.gyroInt)
        config.gyroInt->setCallback(gyroIRQ);
}

/**
 * @brief 加速度计中断回调函数，设置加速度计数据更新标志并统计中断次数
 * @param gpio 指向触发中断的 GPIO 实例的指针，函数内部会根据该实例获取中断源并进行处理
 */
void BMI088::accelIRQ(GPIO *) {
    if (instance) {
        instance->accelUpdateFlag = true;
        instance->accelIrqCount++;
    }
}

/**
 * @brief 陀螺仪中断回调函数，设置陀螺仪数据更新标志并统计中断次数
 * @param gpio 指向触发中断的 GPIO 实例的指针，函数内部会根据该实例获取中断源并进行处理
 */
void BMI088::gyroIRQ(GPIO *) {
    if (instance) {
        instance->gyroUpdateFlag = true;
        instance->gyroIrqCount++;
    }
}

/**
 * @brief 读取加速度计寄存器的值，使用 SPI 进行通信
 * @param reg 待读取的寄存器地址
 * @return 寄存器的值
 */
uint8_t BMI088::readAccelReg(const uint8_t reg) const {
    /* 加速度计 SPI 读：地址字节后返回 1 个 dummy 字节，真实数据在其后 */
    const uint8_t tx[3] = {
        static_cast<uint8_t>(reg | 0x80),
        0x55,
        0x55,
    };

    uint8_t rx[3]{};
    config.accCs->reset();
    config.spi->transmitReceive(tx, rx, 3);
    config.accCs->set();

    return rx[2];
}

/**
 * @brief 向加速度计寄存器写入值，使用 SPI 进行通信
 * @param reg 待写入的寄存器地址
 * @param value 待写入的值
 */
void BMI088::writeAccelReg(const uint8_t reg, const uint8_t value) const {
    const uint8_t tx[2] = {
        static_cast<uint8_t>(reg & 0x7F),
        value
    };

    config.accCs->reset();
    config.spi->transmit(tx, 2);
    config.accCs->set();
}

/**
 * @brief 读取陀螺仪寄存器的值，使用 SPI 进行通信
 * @param reg 待读取的寄存器地址
 * @return 寄存器的值
 */
uint8_t BMI088::readGyroReg(const uint8_t reg) const {
    const uint8_t tx[2] = {
        static_cast<uint8_t>(reg | 0x80),
        0x55
    };

    uint8_t rx[2]{};
    config.gyroCs->reset();
    config.spi->transmitReceive(tx, rx, 2);
    config.gyroCs->set();

    return rx[1];
}

/**
 * @brief 向陀螺仪寄存器写入值，使用 SPI 进行通信
 * @param reg 待写入的寄存器地址
 * @param value 待写入的值
 */
void BMI088::writeGyroReg(const uint8_t reg, const uint8_t value) const {
    const uint8_t tx[2] = {
        static_cast<uint8_t>(reg & 0x7F),
        value
    };

    config.gyroCs->reset();
    config.spi->transmit(tx, 2);
    config.gyroCs->set();
}

/**
 * @brief BMI088 初始化函数，配置加速度计和陀螺仪的寄存器参数
 * @return 初始化是否成功
 */
bool BMI088::init() const {
    /*
     * 初始化阶段使用阻塞轮询 SPI + DWT 延时：
     * 该阶段在 Robot::init() 的 __disable_irq() 区间内运行，
     * HAL_Delay/osDelay 依赖中断会死锁，DWT 延时基于 CYCCNT 自旋不依赖中断。
     */

    /* 加速度计软复位 */
    writeAccelReg(0x7E, 0xB6);
    DWT_Time::delay(0.05f);
    /* 软复位后加速度计回到 I2C 模式，需一次 SPI 读把它切回 SPI 模式 */
    (void) readAccelReg(0x00);
    DWT_Time::delay(0.001f);

    writeAccelReg(0x7C, 0x00); /* ACC_PWR_CONF: active（退出挂起） */
    DWT_Time::delay(0.01f);
    writeAccelReg(0x7D, 0x04); /* ACC_PWR_CTRL: accelerometer on */
    DWT_Time::delay(0.05f);

    writeAccelReg(0x40, 0xAB); /* ACC_CONF: ODR 800Hz, OSR normal */
    DWT_Time::delay(0.001f);
    writeAccelReg(0x41, 0x03); /* ACC_RANGE: ±24g（与 24/32768 量程系数对应） */
    DWT_Time::delay(0.001f);

    /* 加速度计数据就绪中断 → INT1（推挽、高有效，匹配 EXTI 上升沿触发） */
    writeAccelReg(0x53, 0x0A); /* INT1_IO_CTRL: int1 输出使能 | 推挽 | 高有效 */
    DWT_Time::delay(0.001f);
    writeAccelReg(0x58, 0x04); /* INT1_INT2_MAP_DATA: 数据就绪映射到 INT1 */
    DWT_Time::delay(0.001f);

    /* 陀螺仪软复位并配置 */
    writeGyroReg(0x14, 0xB6);
    DWT_Time::delay(0.05f);
    writeGyroReg(0x0F, 0x00); /* GYRO_RANGE: ±2000 dps（与 2000/32768 量程系数对应） */
    writeGyroReg(0x10, 0x02); /* GYRO_BANDWIDTH: ODR 1000Hz / 滤波 116Hz */
    writeGyroReg(0x11, 0x00); /* GYRO_LPM1: normal mode */
    DWT_Time::delay(0.01f);

    /* 陀螺仪数据就绪中断 → INT3（推挽、高有效，匹配 EXTI 上升沿触发） */
    writeGyroReg(0x15, 0x80); /* GYRO_INT_CTRL: 使能新数据中断 */
    DWT_Time::delay(0.001f);
    writeGyroReg(0x16, 0x01); /* INT3_INT4_IO_CONF: int3 推挽 | 高有效 */
    DWT_Time::delay(0.001f);
    writeGyroReg(0x18, 0x01); /* INT3_INT4_IO_MAP: 数据就绪映射到 INT3 */
    DWT_Time::delay(0.01f);

    return true;
}

/**
 * @brief BMI088 数据更新函数，需在任务循环中周期调用。
 *        根据中断标志发起 SPI DMA 读取，并在 DMA 完成回调中解析数据。
 */
void BMI088::update() {
    /* 上一次 DMA 读取尚未完成则跳过，避免重入 SPI/DMA */
    if (reading != Reading::None) {
        return;
    }

    /* 温度计数器：仅负责定时置位 flag，实际读取在加速度计完成回调中链式发起 */
    if (++tempCounter >= tempReadInterval) {
        tempCounter = 0;
        tempReadFlag = true;
    }

    /* 陀螺仪优先（角速度对控制更关键） */
    if (gyroUpdateFlag) {
        gyroUpdateFlag = false;
        gyroReadCount++;
        startGyroRead();
        return;
    }

    if (accelUpdateFlag) {
        accelUpdateFlag = false;
        accelReadCount++;
        startAccelRead();
    }
}

/**
 * @brief 发起一次加速度计 SPI DMA 读取，读取 6 字节加速度数据
 *        读取完成后会在 DMA 回调中解析数据并更新 data.accel
 */
void BMI088::startAccelRead() {
    reading = Reading::Accel;

    /* 地址字节 + 1 dummy + 6 数据字节 = 8 字节 */
    txBuf[0] = 0x12 | 0x80; /* ACC_X_LSB，读 */
    for (uint8_t i = 1; i < 8; ++i)
        txBuf[i] = 0x55;

    config.accCs->reset();
    if (config.spi->transmitReceiveDMA(txBuf, rxBuf, 8) != HAL_OK) {
        config.accCs->set();
        reading = Reading::None;
    }
}

/**
 * @brief 发起一次陀螺仪 SPI DMA 读取，读取 6 字节角速度数据
 *        读取完成后会在 DMA 回调中解析数据并更新 data.gyro
 */
void BMI088::startGyroRead() {
    reading = Reading::Gyro;

    /* 陀螺仪无 dummy 字节：地址字节 + 6 数据字节 = 7 字节 */
    txBuf[0] = 0x02 | 0x80; /* RATE_X_LSB，读 */
    for (uint8_t i = 1; i < 7; ++i)
        txBuf[i] = 0x55;

    config.gyroCs->reset();
    if (config.spi->transmitReceiveDMA(txBuf, rxBuf, 7) != HAL_OK) {
        config.gyroCs->set();
        reading = Reading::None;
    }
}

/**
 * @brief 发起一次温度 SPI DMA 读取，读取 2 字节温度数据
 *        读取完成后会在 DMA 回调中解析数据并更新 data.temperature
 */
void BMI088::startTempRead() {
    reading = Reading::Temp;

    /* 温度寄存器位于加速度计地址空间：地址字节 + 1 dummy + 2 数据字节 = 4 字节 */
    txBuf[0] = 0x22 | 0x80; /* TEMP_MSB，读 */
    for (uint8_t i = 1; i < 4; ++i)
        txBuf[i] = 0x55;

    config.accCs->reset();
    if (config.spi->transmitReceiveDMA(txBuf, rxBuf, 4) != HAL_OK) {
        config.accCs->set();
        reading = Reading::None;
    }
}

/**
 * @brief SPI DMA 传输完成回调函数，解析接收到的数据并更新对应的成员变量
 * @param hspi 指向触发回调的 SPI_HandleTypeDef 实例的指针
 */
void BMI088::onSpiTxRxComplete(SPI_HandleTypeDef *hspi) {
    if (hspi != config.spi->getHandle()) {
        return;
    }

    switch (reading) {
        case Reading::Accel: {
            config.accCs->set();

            /* rxBuf[0]=地址相位, rxBuf[1]=dummy, rxBuf[2..7]=6 数据字节 */
            const auto ax = static_cast<int16_t>((rxBuf[3] << 8) | rxBuf[2]);
            const auto ay = static_cast<int16_t>((rxBuf[5] << 8) | rxBuf[4]);
            const auto az = static_cast<int16_t>((rxBuf[7] << 8) | rxBuf[6]);

            constexpr float scale = 24.0f / 32768.0f;
            data.accel.x = static_cast<float>(ax) * scale;
            data.accel.y = static_cast<float>(ay) * scale;
            data.accel.z = static_cast<float>(az) * scale;

            reading = Reading::None;

            /* 链式读取：加速度计完成后发起低优先级的温度读取 */
            if (tempReadFlag) {
                tempReadFlag = false;
                startTempRead();
            }
            break;
        }

        case Reading::Gyro: {
            config.gyroCs->set();

            /* rxBuf[0]=地址相位, rxBuf[1..6]=6 数据字节 */
            const auto gx = static_cast<int16_t>((rxBuf[2] << 8) | rxBuf[1]);
            const auto gy = static_cast<int16_t>((rxBuf[4] << 8) | rxBuf[3]);
            const auto gz = static_cast<int16_t>((rxBuf[6] << 8) | rxBuf[5]);

            constexpr float scale = 2000.0f / 32768.0f;
            data.gyro.x = static_cast<float>(gx) * scale;
            data.gyro.y = static_cast<float>(gy) * scale;
            data.gyro.z = static_cast<float>(gz) * scale;

            reading = Reading::None;

            /*
             * 链式读取：陀螺仪 DMA 完成后立即检查加速度计是否有待处理的数据就绪标志。
             * 若不做此处理，由于陀螺仪 ODR（1000Hz）≥ update() 调用频率，
             * update() 中的陀螺仪优先逻辑会始终抢占加速度计，导致加速度计数据几乎
             * 无法得到服务（饥饿问题）。在此处链式启动可保证加速度计以其自身 ODR
             * （800Hz）正常更新，不依赖 update() 调度频率。
             */
            if (accelUpdateFlag) {
                accelUpdateFlag = false;
                accelReadCount++;
                startAccelRead();
            }
            break;
        }

        case Reading::Temp: {
            config.accCs->set();

            /*
             * rxBuf[0]=地址相位, rxBuf[1]=dummy, rxBuf[2]=TEMP_MSB, rxBuf[3]=TEMP_LSB。
             * 温度为 11 位有符号值：高 8 位在 MSB，低 3 位在 LSB 的高 3 位。
             */
            const uint16_t rawTemp = (static_cast<uint16_t>(rxBuf[2]) << 3) | (rxBuf[3] >> 5);
            auto t = static_cast<int16_t>(rawTemp);
            if (t > 1023)
                t -= 2048;

            /* 分辨率 0.125°C/LSB，偏置 23°C */
            data.temperature = static_cast<float>(t) * 0.125f + 23.0f;

            reading = Reading::None;
            break;
        }

        default:
            break;
    }
}

/**
 * @brief SPI 错误回调函数，拉高片选并复位状态，避免一次错误导致后续读取永久阻塞
 * @param hspi 指向触发回调的 SPI_HandleTypeDef 实例的指针
 */
void BMI088::onSpiError(SPI_HandleTypeDef *hspi) {
    if (hspi != config.spi->getHandle()) {
        return;
    }

    /* 出错时拉高两片 CS 并复位状态，保证下一轮读取能正常发起 */
    config.accCs->set();
    config.gyroCs->set();
    reading = Reading::None;
}

extern "C" void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi) {
    if (BMI088 *inst = BMI088::getInstance())
        inst->onSpiTxRxComplete(hspi);
}

extern "C" void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi) {
    if (BMI088 *inst = BMI088::getInstance())
        inst->onSpiError(hspi);
}
