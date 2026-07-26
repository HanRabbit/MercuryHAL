#pragma once

#include "bsp_can.h"
#include "bmi088.h"
#include "arm_math.h"  /* consumer must provide CMSIS-DSP include path */

/* 超核惯导 IMU 默认 CAN ID */
#define HiIMU_CAN_RX_ACC_ID 0x188
#define HiIMU_CAN_RX_GYRO_ID 0x288
#define HiIMU_CAN_RX_ANGLE_ID 0x388

class IMU {
public:
    /* IMU 类型，用户在 Config 中指定以注册不同的 IMU */
    enum class Type {
        HiIMU_CAN, /* 超核惯导，CAN 通信 */
        BMI088_SPI, /* BMI088，SPI 通信 */
    };

    /* IMU 配置结构体 */
    struct Config {
        /* IMU 类型，默认使用超核惯导 CAN */
        Type type = Type::HiIMU_CAN;

        /* ---- 超核惯导 CAN 配置（type == HiIMU_CAN 时生效）---- */
        FDCAN_HandleTypeDef *hfdcan{};
        /* IMU 发送的不同数据包的 CAN_ID */
        uint16_t rx_acc_id = HiIMU_CAN_RX_ACC_ID;
        uint16_t rx_gyro_id = HiIMU_CAN_RX_GYRO_ID;
        uint16_t rx_angle_id = HiIMU_CAN_RX_ANGLE_ID;

        /* ---- BMI088 SPI 配置（type == BMI088_SPI 时生效）---- */
        BMI088::Config bmi088{};
    };

    struct accel_data_t {
        float x;
        float y;
        float z;
    };

    struct gyro_data_t {
        float x;
        float y;
        float z;
    };

    struct angle_data_t {
        float roll;
        float pitch;
        float yaw;
        float total_yaw;
        float total_yaw_init_offset;
    };

    struct quaternion_data_t {
        float x;
        float y;
        float z;
        float w;
    };

    struct imu_data_t {
        accel_data_t accel;
        gyro_data_t gyro;
        angle_data_t angle;
        quaternion_data_t quaternion;
        float temperature;
    };

    FDCAN::Config fdcan_init_config[3]{}; /* IMU 的 FDCAN 配置（仅 HiIMU_CAN 使用）*/
    bool imu_init_done = false; /* IMU 初始化完成标志 */
    imu_data_t data{}; /* IMU 数据结构体实例 */
    Config initConfig; /* IMU 初始化配置 */

    explicit IMU(const Config &config);

    void init();

    /**
     * @brief IMU 数据更新函数，需在任务循环中周期调用。
     *        对于 BMI088（轮询读取）会在此处读取最新数据并写入统一数据结构；
     *        对于超核惯导（中断回调驱动）则无需处理，此处为空操作。
     */
    void update();

private:
    float q_data[4];
    float dq_data[4];
    float omega_data[16];
    arm_matrix_instance_f32 q;
    arm_matrix_instance_f32 dq;
    arm_matrix_instance_f32 Omega;

    static void decode(FDCAN *fdcan);
    static IMU *instance;

    BMI088 *bmi088 = nullptr; /* BMI088 驱动实例（仅 BMI088_SPI 使用）*/
};
