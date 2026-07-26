#pragma once

#include "bsp_can.h"
#include "bsp_pwm.h"
#include "daemon.h"
#include "motor_def.h"

#define DJI_MOTOR_SPEED_CONF 0.9f   /* 速度环低通滤波系数 */
#define DJI_MOTOR_CURRENT_CONF 0.9f    /* 电流环低通滤波系数 */

#define DJI_ECD_RAD_COEF (2.0f * 3.1415926f / 8192.0f)  // 编码器读数转换为弧度的系数

class DJI_Motor {
public:
    /* 电机初始化配置 */
    struct Config {
        DJI_Motor_Type_t type;
        uint8_t id;
        Motor_Controller_Init_s controller_param_init_config;
        Motor_Control_Settings_s controller_settings_init_config;
        FDCAN::Config can_init_config;
        float vel_LPF_RC;
    };

    DJI_Motor_Measure_t measure{}; // 电机测量数据
    Motor_Control_Settings_s motor_settings{}; // 电机设置
    Motor_Controller_s motor_controller; // 电机控制器

    FDCAN *motor_can_instance{}; // 电机 CAN 实例

    /* 分组发送 */
    uint8_t sender_group{};
    uint8_t message_num{};

    Config init_config{};

    DJI_Motor_Type_t motor_type; // 电机类型
    Motor_Working_Type_e stop_flag; // 电机启停标志

    Daemon *daemon{};
    uint32_t feed_cnt{};
    float dt{};

    static constexpr uint8_t MAX_CNT = 16;
    static DJI_Motor *instances_[MAX_CNT];
    static uint8_t count_;

    explicit DJI_Motor(const Config &config);
    void init();
    void senderGrouping();
    void stop();
    void setFeed(CloseLoop_Type_e loop, Feedback_Source_e type);
    void enable();
    void setOuterLoop(CloseLoop_Type_e outer_loop);
    void setRef(float ref);

    static void lostCallback(void *motor_ptr);
    static void decode(FDCAN *can_instance);
    static void control();
};
