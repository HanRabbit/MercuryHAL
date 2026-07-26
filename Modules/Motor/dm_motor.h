#pragma once

#include <cstdint>

#include "bsp_can.h"
#include "daemon.h"

/*
 * 达妙 DM-J4310-2EC V1.2 减速电机驱动
 *
 * 与 DJI 电机不同, 达妙电机的闭环控制(位置/速度/电流环)全部运行在电机自带
 * 的驱动器内部, 主控只需按对应模式下发目标指令(位置/速度/力矩/Kp/Kd 等),
 * 电机即会自行完成控制. 因此本驱动不做串级 PID, 只负责:
 *   1. 按所选模式打包并周期发送控制帧;
 *   2. 解析电机反馈帧(位置/速度/扭矩/温度/状态), 并做多圈角度计算;
 *   3. 使能/失能/保存零点/清除错误等控制命令;
 *   4. 通过 Daemon 做电机在线检测(喂狗).
 *
 * @attention 使用前需先通过达妙上位机(调试助手)完成电机侧编码器校准、参数标定、
 *            输出轴编码器校准, 并配置好 控制模式(CTRL_MODE)、CAN_ID(ESC_ID)、
 *            反馈 ID(MST_ID) 以及 位置/速度/扭矩映射范围(PMAX/VMAX/TMAX).
 *            本驱动的 Config.mode 必须与电机内部配置的 CTRL_MODE 保持一致.
 */

#define DM_MOTOR_KP_MIN 0.0f    /* MIT 模式 Kp 下限 */
#define DM_MOTOR_KP_MAX 500.0f  /* MIT 模式 Kp 上限 */
#define DM_MOTOR_KD_MIN 0.0f    /* MIT 模式 Kd 下限 */
#define DM_MOTOR_KD_MAX 5.0f    /* MIT 模式 Kd 上限 */

/* 力位混控模式限速/限流放大系数 */
#define DM_FORCE_POS_VEL_SCALE 100.0f     /* v_des 放大 100 倍 */
#define DM_FORCE_POS_CUR_SCALE 10000.0f   /* i_des 放大 10000 倍 */
#define DM_FORCE_POS_MAX_RAW 10000         /* 放大后的最大值 */

/* 控制命令帧数据段最后一字节(D[7]) */
#define DM_CMD_ENABLE 0xFC      /* 使能 */
#define DM_CMD_DISABLE 0xFD     /* 失能 */
#define DM_CMD_SAVE_ZERO 0xFE   /* 保存位置零点 */
#define DM_CMD_CLEAR_ERROR 0xFB /* 清除错误 */

/*
 * 控制模式枚举.
 * @note 枚举值即为控制帧 ID 相对 CAN_ID 的偏移(offset = value << 8):
 *       MIT->0x000, 位置速度->0x100, 速度->0x200, 力位混控->0x300.
 *       对应电机 CTRL_MODE 寄存器(0x0A)取值分别为 1/2/3/4.
 */
typedef enum {
    DM_MODE_MIT = 0,       /* MIT 模式    (CTRL_MODE=1) */
    DM_MODE_POS_VEL = 1,   /* 位置速度模式 (CTRL_MODE=2) */
    DM_MODE_VEL = 2,       /* 速度模式    (CTRL_MODE=3) */
    DM_MODE_FORCE_POS = 3, /* 力位混控模式 (CTRL_MODE=4) */
} DM_Motor_Mode_e;

/* 电机状态/错误码, 取自反馈帧 D[0] 的高 4 位(ERR) */
typedef enum {
    DM_STATE_DISABLE = 0x0,        /* 失能 */
    DM_STATE_ENABLE = 0x1,         /* 使能 */
    DM_STATE_OVER_VOLTAGE = 0x8,   /* 超压 */
    DM_STATE_UNDER_VOLTAGE = 0x9,  /* 欠压 */
    DM_STATE_OVER_CURRENT = 0xA,   /* 过电流 */
    DM_STATE_MOS_OVER_TEMP = 0xB,  /* MOS 过温 */
    DM_STATE_COIL_OVER_TEMP = 0xC, /* 电机线圈过温 */
    DM_STATE_COMM_LOST = 0xD,      /* 通讯丢失 */
    DM_STATE_OVERLOAD = 0xE,       /* 过载 */
} DM_Motor_State_e;

/* 电机反馈测量数据 */
typedef struct {
    uint8_t id;             /* 反馈的控制器 ID(取 CAN_ID 低 8 位) */
    uint8_t state;          /* 状态/错误码, 见 DM_Motor_State_e */
    float pos_rad;          /* 输出轴单圈位置, 弧度制 */
    float vel_rad;          /* 输出轴角速度, 弧度制 */
    float torque;           /* 电机扭矩, 单位 N·m */
    float vel_rad_filtered; /* 经低通滤波的角速度 */
    uint8_t t_mos;          /* 驱动板 MOS 平均温度, 单位 ℃ */
    uint8_t t_rotor;        /* 电机线圈平均温度, 单位 ℃ */

    float last_pos_rad;   /* 上次单圈位置, 用于多圈计算 */
    int32_t total_round;  /* 总圈数 */
    float total_pos_rad;  /* 多圈累计位置, 弧度制 */
} DM_Motor_Measure_t;

class DM_Motor {
public:
    /* 电机初始化配置 */
    struct Config {
        DM_Motor_Mode_e mode;          /* 控制模式(需与电机 CTRL_MODE 一致) */
        uint16_t can_id;               /* 电机 CAN ID(ESC_ID), 建议 < 16 */
        uint16_t master_id;            /* 反馈帧 ID(MST_ID) */
        float p_max;                   /* 位置映射范围(rad), 对应寄存器 PMAX */
        float v_max;                   /* 速度映射范围(rad/s), 对应寄存器 VMAX */
        float t_max;                   /* 扭矩映射范围(N·m), 对应寄存器 TMAX */
        FDCAN::Config can_init_config; /* CAN 配置, 仅需填写 hfdcan */
        float vel_LPF_RC;              /* 速度低通滤波时间常数(0 表示不滤波) */
        bool debug = false;            /* 调试: USB CDC <-> 电机调试串口透传 */
        UART_HandleTypeDef *debug_huart = nullptr; /* debug=true 时必填 */
    };

    DM_Motor_Measure_t measure{}; /* 电机测量数据 */
    DM_Motor_Mode_e mode{};       /* 电机控制模式 */

    FDCAN *motor_can_instance{}; /* 电机 CAN 实例 */
    Config init_config{};        /* 初始化配置 */

    bool enabled{}; /* 使能标志 */

    /* 控制设定值(由对应模式的 setXXX() 写入, control() 中打包发送) */
    float p_des{}; /* 位置给定(rad) */
    float v_des{}; /* 速度给定(rad/s); 力位混控下为限速幅值 */
    float kp{};    /* MIT 位置比例系数 */
    float kd{};    /* MIT 位置微分系数 */
    float t_ff{};  /* MIT 转矩前馈(N·m) */
    float i_des{}; /* 力位混控电流限定标幺值[0,1] */

    Daemon *daemon{};   /* 在线检测(喂狗) */
    uint32_t feed_cnt{}; /* DWT 计数, 用于计算 dt */
    float dt{};          /* 两次反馈时间间隔 */

    static constexpr uint8_t MAX_CNT = 16;
    static DM_Motor *instances_[MAX_CNT];
    static uint8_t count_;

    explicit DM_Motor(const Config &config);
    void init();

    /* 控制命令 */
    void enable();     /* 使能 */
    void disable();    /* 失能 */
    void saveZero();   /* 保存当前输出轴位置为零点 */
    void clearError(); /* 清除错误 */

    /* 各模式设定值写入(仅使用与当前模式匹配的接口) */
    void setMIT(float pos, float vel, float kp, float kd, float torque); /* MIT 模式 */
    void setPosVel(float pos, float vel);                               /* 位置速度模式 */
    void setVel(float vel);                                             /* 速度模式 */
    void setForcePos(float pos, float vel, float cur);                  /* 力位混控模式 */

    static void decode(FDCAN *can_instance);
    static void lostCallback(void *motor_ptr);
    static void control();

private:
    /* 浮点数与定点数之间的线性映射(达妙 CAN 协议) */
    static int float_to_uint(float x, float x_min, float x_max, int bits);
    static float uint_to_float(int x_int, float x_min, float x_max, int bits);

    void sendCommand(uint8_t cmd); /* 发送控制命令帧(使能/失能/零点/清错) */
    void packAndSend();            /* 按模式打包并发送控制帧 */
};
