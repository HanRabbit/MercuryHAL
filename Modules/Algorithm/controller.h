#pragma once

#include "bsp_dwt.h"

class Controller {
    float measure{};
    float last_measure{};
    float err{};
    float last_err{};
    float last_ITerm{};

    float Pout{};
    float Iout{};
    float Dout{};
    float ITerm{};

    float last_output{};
    float last_Dout{};

    float ref{};

    uint32_t DWT_CNT{};
    float dt{};

    void f_trapezoid_integral();
    void f_changing_integration_rate();
    void f_integral_limit();
    void f_derivative_on_measurement();
    void f_derivative_filter();
    void f_output_filter();
    void f_output_limit();
    void f_error_handle();

public:
    /* PID 改进功能枚举 */
    enum PID_Improvement_t {
        PID_IMPROVE_NONE = 0b00000000, // 0000 0000
        PID_Integral_Limit = 0b00000001, // 0000 0001
        PID_Derivative_On_Measurement = 0b00000010, // 0000 0010
        PID_Trapezoid_Integral = 0b00000100, // 0000 0100
        PID_Proportional_On_Measurement = 0b00001000, // 0000 1000
        PID_OutputFilter = 0b00010000, // 0001 0000
        PID_ChangingIntegrationRate = 0b00100000, // 0010 0000
        PID_DerivativeFilter = 0b01000000, // 0100 0000
        PID_ErrorHandle = 0b10000000, // 1000 0000
    };

    /* PID 错误类型枚举 */
    enum ErrorType_t {
        ERROR_NONE = 0,
        ERROR_MOTOR_BLOCKED /* 电机堵转 */
    };

    /* PID 错误处理 */
    struct ErrorHandler_t {
        uint32_t err_count;
        ErrorType_t err_type;
    };

    /* PID 初始化配置结构体 */
    struct Config {
        // basic parameter
        float Kp;
        float Ki;
        float Kd;
        float max_out; // 输出限幅
        float dead_band; // 死区

        // improve parameter
        PID_Improvement_t imp; // PID改进功能选择
        float integral_limit; // 积分限幅
        float CoefA; // AB为变速积分参数,变速积分实际上就引入了积分分离
        float CoefB; // ITerm = Err*((A-abs(err)+B)/A)  when B<|err|<A+B
        float output_LPF_RC; // RC = 1/omegac
        float derivative_LPF_RC;
    };

    float Kp{};
    float Ki{};
    float Kd{};
    float max_out{};
    float dead_band{};
    float output{};

    PID_Improvement_t imp = PID_IMPROVE_NONE;
    float integral_limit{}; // 积分限幅
    float CoefA{}; // 变速积分 For Changing Integral
    float CoefB{}; // 变速积分 ITerm = Err*((A-abs(err)+B)/A)  when B<|err|<A+B
    float output_LPF_RC{}; // 输出滤波器 RC = 1/omegac
    float derivative_LPF_RC{}; // 微分滤波器系数

    ErrorHandler_t error_handler{};
    Config init_config{};

    explicit Controller(const Config &config);
    void init();
    float calculate(float pid_measure, float pid_ref);
};
