#pragma once

#include <cstdint>

#include "controller.h"

typedef struct {
    float pos_rad; /* 弧度制的电机单圈角度 */
    float pos_total_rad; /* 弧度制的电机多圈角度 */
    float vel_rad; /* 弧度制的电机角速度 */
    float vel_rad_filtered; /* 经过低通滤波的电机角速度 */
    uint16_t ecd; /* 电机单圈编码器值 */
    uint16_t last_ecd; /* 上次记录的电机单圈编码器值 */
    int32_t total_round; /* 总圈数 */
    int16_t current; /* 电机当前电流 */
    uint8_t temperature; /* 电机当前温度 */
} DJI_Motor_Measure_t;

/* 电机枚举类型 */
typedef enum {
    GM6020 = 0,
    M3508P19,
    M3508,
    M2006,
} DJI_Motor_Type_t;

typedef enum {
    MOTOR_STOP = 0,
    MOTOR_ENABLED
} Motor_Working_Type_e;

typedef enum {
    OPEN_LOOP = 0b0000,
    CURRENT_LOOP = 0b0001,
    SPEED_LOOP = 0b0010,
    POSITION_LOOP = 0b0100,

    SPEED_AND_CURRENT_LOOP = 0b0011,
    ANGLE_AND_SPEED_LOOP = 0b0110,
    ALL_THREE_LOOP = 0b0111,
} CloseLoop_Type_e;

typedef enum {
    FEEDFORWARD_NONE = 0b00,
    CURRENT_FEEDFORWARD = 0b01, // 包含电流前馈
    SPEED_FEEDFORWARD = 0b10, // 包含速度前馈
    CURRENT_AND_SPEED_FEEDFORWARD = CURRENT_FEEDFORWARD | SPEED_FEEDFORWARD,
} Feedforward_Type_e;

typedef enum {
    MOTOR_FEED = 0,
    OTHER_FEED
} Feedback_Source_e;

typedef enum {
    MOTOR_DIRECTION_NORMAL = 0,
    MOTOR_DIRECTION_REVERSE
} Motor_Reverse_Flag_e;

typedef enum {
    FEEDBACK_DIRECTION_NORMAL = 0,
    FEEDBACK_DIRECTION_REVERSE = 1
} Feedback_Reverse_Flag_e;

typedef struct {
    CloseLoop_Type_e outer_loop_type; // 最外层闭环控制模式
    CloseLoop_Type_e close_loop_type; // 串级闭环控制
    Motor_Reverse_Flag_e motor_reverse_flag; // 电机反转标志
    Feedback_Reverse_Flag_e feedback_reverse_flag; // 反馈反转标志
    Feedback_Source_e angle_feedback_source; // 角度反馈源
    Feedback_Source_e speed_feedback_source; // 速度反馈源
    Feedforward_Type_e feedforward_flag; // 前馈控制标志
} Motor_Control_Settings_s;

struct Motor_Controller_s {
    float *other_angle_feedback_ptr{}; // 其他角度反馈指针
    float *other_speed_feedback_ptr{}; // 其他速度反馈指针
    float *speed_feedforward_ptr{}; // 速度前馈指针
    float *current_feedforward_ptr{}; // 电流前馈指针

    Controller *PID_current{};
    Controller *PID_speed{};
    Controller *PID_angle{};

    float pid_ref{}; // 串级闭环参数
};

typedef struct {
    float *other_angle_feedback_ptr; // 其他角度反馈指针，电机使用的是 total_rad
    float *other_speed_feedback_ptr; // 其他速度反馈指针，单位为 rad per sec

    float *speed_feedforward_ptr; // 速度前馈指针
    float *current_feedforward_ptr; // 电流前馈指针

    Controller::Config PID_current;
    Controller::Config PID_speed;
    Controller::Config PID_angle;
} Motor_Controller_Init_s;

