#include "imu.h"
#include <cstring>
#include <cmath>
#include "bsp_dwt.h"
#include "bsp_log.h"
#include "canopen_parser.h"
#include "hipnuc_can_common.h"
#include "general_def.h"
#include "user_lib.h"

static hipnuc_can_frame_t imu_frame;
static can_sensor_data_t imu_data;

IMU *IMU::instance = nullptr;

/**
 * @brief IMU 构造函数，根据配置中的 type 字段注册对应类型的 IMU
 * @param config IMU 配置结构体，包含 IMU 类型以及对应通信外设的配置。
 *               当 type 为 HiIMU_CAN 时，使用 hfdcan 与各数据包 CAN ID 初始化三个 FDCAN 配置，
 *               用于注册三个 FDCAN 实例以接收加速度、陀螺仪和角度数据；
 *               当 type 为 BMI088_SPI 时，使用 bmi088 子配置创建 BMI088 驱动实例。
 */
IMU::IMU(const Config &config) {
    instance = this;
    initConfig = config;

    switch (config.type) {
        case Type::HiIMU_CAN:
            fdcan_init_config[0] = {
                .hfdcan = config.hfdcan,
                .tx_id = 0, /* IMU 不发送数据，因此 TX ID 设置为 0 */
                .rx_id = config.rx_acc_id, /* 加速度数据的接收 ID */
                .callback = &IMU::decode, /* 接收加速度数据时调用 decode 函数进行解码处理 */
            };
            fdcan_init_config[1] = {
                .hfdcan = config.hfdcan,
                .tx_id = 0,
                .rx_id = config.rx_gyro_id, /* 陀螺仪数据的接收 ID */
                .callback = &IMU::decode,
            };
            fdcan_init_config[2] = {
                .hfdcan = config.hfdcan,
                .tx_id = 0,
                .rx_id = config.rx_angle_id, /* 角度数据的接收 ID */
                .callback = &IMU::decode,
            };
            break;

        case Type::BMI088_SPI:
            bmi088 = new BMI088(config.bmi088);
            break;
    }
}

/**
 * @brief IMU 初始化函数，根据 IMU 类型初始化对应的通信外设。
 *        HiIMU_CAN 注册三个 FDCAN 实例；BMI088_SPI 完成传感器寄存器配置。
 */
void IMU::init() {
    LOG_INFO("[IMU] Initializing IMU...");

    /* 初始化 IMU 四元数 */
    data.quaternion = {
        .x = 0.0f,
        .y = 0.0f,
        .z = 0.0f,
        .w = 1.0f,
    };
    q_data[0] = data.quaternion.x;
    q_data[1] = data.quaternion.y;
    q_data[2] = data.quaternion.z;
    q_data[3] = data.quaternion.w;

    switch (initConfig.type) {
        case Type::HiIMU_CAN: {
            /* FDCAN 初始化 */
            static FDCAN fdcan[3] = {
                FDCAN(fdcan_init_config[0]),
                FDCAN(fdcan_init_config[1]),
                FDCAN(fdcan_init_config[2]),
            };
            break;
        }

        case Type::BMI088_SPI:
            if (bmi088 != nullptr) {
                bmi088->init();
                imu_init_done = true;
            }
            break;
    }

    /* 初始化 IMU 的四元数矩阵 */
    arm_mat_init_f32(&q, 4, 1, q_data);
    arm_mat_init_f32(&dq, 4, 1, dq_data);
    arm_mat_init_f32(&Omega, 4, 4, omega_data);

    LOG_INFO("[IMU] Initialized.");
}

/**
 * @brief IMU 数据更新函数，需在任务循环中周期调用。
 *        对于 BMI088（轮询读取）会在此处读取最新数据并写入统一数据结构；
 *        对于超核惯导（中断回调驱动）则无需处理，此处为空操作。
 */
void IMU::update() {
    if (initConfig.type != Type::BMI088_SPI || bmi088 == nullptr) {
        return;
    }
    static uint64_t last_time = DWT_Time::getTimeline_us();

    bmi088->update();
    const BMI088::Data &raw = bmi088->getData();

    /* BMI088 输出加速度单位为 g，角速度单位为 deg/s */
    data.accel.x = raw.accel.x;
    data.accel.y = raw.accel.y;
    data.accel.z = raw.accel.z;

    data.gyro.x = raw.gyro.x * DEGREE_2_RAD;
    data.gyro.y = raw.gyro.y * DEGREE_2_RAD;
    data.gyro.z = raw.gyro.z * DEGREE_2_RAD;

    /* 四元数运动学矩阵 Omega (标量在后 [x,y,z,w] 约定, 与 q_data 存储/欧拉角提取一致)
     * dq = 0.5 * Omega * q, q=[x,y,z,w]^T */
    omega_data[0] = 0;
    omega_data[1] = data.gyro.z;
    omega_data[2] = -data.gyro.y;
    omega_data[3] = data.gyro.x;

    omega_data[4] = -data.gyro.z;
    omega_data[5] = 0;
    omega_data[6] = data.gyro.x;
    omega_data[7] = data.gyro.y;

    omega_data[8] = data.gyro.y;
    omega_data[9] = -data.gyro.x;
    omega_data[10] = 0;
    omega_data[11] = data.gyro.z;

    omega_data[12] = -data.gyro.x;
    omega_data[13] = -data.gyro.y;
    omega_data[14] = -data.gyro.z;
    omega_data[15] = 0;

    const uint64_t now = DWT_Time::getTimeline_us();
    const float dt = static_cast<float>(now - last_time) * 1e-6f;
    last_time = now;

    if (arm_mat_mult_f32(&Omega, &q, &dq) != ARM_MATH_SUCCESS) {
        return;
    }

    for (int i = 0; i < 4; i++) {
        q_data[i] += 0.5f * dq_data[i] * dt;
    }

    float norm2;
    float norm;
    arm_dot_prod_f32(q_data, q_data, 4, &norm2);
    if (norm2 > 0.0f && arm_sqrt_f32(norm2, &norm) == ARM_MATH_SUCCESS && norm > 0.0f) {
        arm_scale_f32(q_data, 1.0f / norm, q_data, 4);
    } else {
        q_data[0] = 0.0f;
        q_data[1] = 0.0f;
        q_data[2] = 0.0f;
        q_data[3] = 1.0f;
    }

    data.quaternion.x = q_data[0];
    data.quaternion.y = q_data[1];
    data.quaternion.z = q_data[2];
    data.quaternion.w = q_data[3];

    /* 四元数转欧拉角 (ZYX / yaw-pitch-roll 顺序), 单位: rad */
    const float qx = data.quaternion.x;
    const float qy = data.quaternion.y;
    const float qz = data.quaternion.z;
    const float qw = data.quaternion.w;

    /* roll (绕 X 轴) */
    const float sinr_cosp = 2.0f * (qw * qx + qy * qz);
    const float cosr_cosp = 1.0f - 2.0f * (qx * qx + qy * qy);
    data.angle.roll = atan2f(sinr_cosp, cosr_cosp);

    /* pitch (绕 Y 轴); 万向锁处理: sinp 限幅到 [-1,1] 防止 asinf 返回 NaN */
    float sinp = 2.0f * (qw * qy - qz * qx);
    if (sinp > 1.0f) sinp = 1.0f;
    else if (sinp < -1.0f) sinp = -1.0f;
    data.angle.pitch = asinf(sinp);

    /* yaw (绕 Z 轴) */
    const float siny_cosp = 2.0f * (qw * qz + qx * qy);
    const float cosy_cosp = 1.0f - 2.0f * (qy * qy + qz * qz);
    data.angle.yaw = atan2f(siny_cosp, cosy_cosp);

    /* Yaw 连续展开, 得到不受 ±π 跳变影响的累积角 */
    static float last_yaw = 0.0f;
    data.angle.total_yaw = angle_unwrap_update(data.angle.yaw, &last_yaw, &data.angle.total_yaw);

    data.temperature = raw.temperature;
}

/**
 * @brief IMU 数据解码函数，作为 FDCAN 接收回调函数被调用，解析接收到的 CAN 数据帧并更新 IMU 数据结构体中的加速度、陀螺仪和角度数据，同时进行单位转换和角度连续展开处理以获得累积的 Yaw 角度值
 * @param fdcan 指向触发回调的 FDCAN 实例的指针，函数内部会根据该实例获取接收到的数据帧并进行解析处理，更新 IMU 的数据结构体以供后续使用
 */
void IMU::decode(FDCAN *fdcan) {
    if (instance == nullptr) {
        return;
    }

    static float last_yaw = 0.0f;

    imu_frame.can_dlc = 6;
    imu_frame.can_id = fdcan->getRxId();
    memcpy(&imu_frame.data, fdcan->getRxBuff(), 6);
    canopen_parse_frame(&imu_frame, &imu_data);

    /*
     * 此时解析之后的数据单位为 加速度：mG，角速度：0.1deg/s，欧拉角：0.01°
     * 需要将加速度转换为 m/s^2，角速度转换为 rad/s，欧拉角转换为 rad
     * */

    /* 将加速度从 mG 转换为 m/s^2 */
    instance->data.accel.x = imu_data.acc_x * 0.00980665f;
    instance->data.accel.y = imu_data.acc_y * 0.00980665f;
    instance->data.accel.z = imu_data.acc_z * 0.00980665f;

    /* 将角速度从 0.1deg/s 转换为 rad/s */
    instance->data.gyro.x = imu_data.gyr_x * DEGREE_2_RAD;
    instance->data.gyro.y = imu_data.gyr_y * DEGREE_2_RAD;
    instance->data.gyro.z = imu_data.gyr_z * DEGREE_2_RAD;

    instance->data.angle.pitch = imu_data.pitch;
    instance->data.angle.roll = imu_data.roll;

    /* IMU 的 Yaw 值以第一次初始化时的值为基准置 0 */
    if (!instance->imu_init_done && instance->data.angle.yaw != 0.0f) {
        instance->data.angle.total_yaw_init_offset = instance->data.angle.yaw;
        last_yaw = instance->data.angle.yaw;
        instance->imu_init_done = true;
        LOG_INFO("[IMU] Successfully initialized with offset: %f.", instance->data.angle.total_yaw_init_offset);
    }
    instance->data.angle.yaw = imu_data.imu_yaw - instance->data.angle.total_yaw_init_offset;

    /* 使用角度连续展开获得 Yaw 轴角度的累积值，由于超核惯导的 IMU 已经内置了拓展卡尔曼滤波，因此可以直接使用读到的角度作为准确值 */
    instance->data.angle.total_yaw = angle_unwrap_update(instance->data.angle.yaw, &last_yaw,
                                                         &instance->data.angle.total_yaw);
}
