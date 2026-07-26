#include "controller.h"
#include <cmath>
#include "bsp_dwt.h"

Controller::Controller(const Config &config) {
    this->init_config = config;
    Kp = init_config.Kp;
    Ki = init_config.Ki;
    Kd = init_config.Kd;
    max_out = init_config.max_out;
    dead_band = init_config.dead_band;
    imp = init_config.imp;
    integral_limit = init_config.integral_limit;
    CoefA = init_config.CoefA;
    CoefB = init_config.CoefB;
    output_LPF_RC = init_config.output_LPF_RC;
    derivative_LPF_RC = init_config.derivative_LPF_RC;
}

void Controller::init() {
    DWT_Time::getDeltaT(&DWT_CNT);
}

/**
 * @brief 计算 PID 输出函数，接受当前测量值和参考输入作为参数，函数内部会根据 PID 算法计算 P、I、D 三项的输出，并根据配置的改进功能进行相应的处理，最终返回计算得到的控制输出值
 * @param pid_measure 当前测量值
 * @param pid_ref 参考输入值
 * @return 计算得到的控制输出值，根据 PID 算法和改进功能进行处理后的结果
 */
float Controller::calculate(const float pid_measure, const float pid_ref) {
    // 堵转检测
    if (imp & PID_ErrorHandle)
        f_error_handle();

    dt = DWT_Time::getDeltaT(&DWT_CNT); // 获取两次pid计算的时间间隔,用于积分和微分

    // 保存上次的测量值和误差,计算当前error
    measure = pid_measure;
    ref = pid_ref;
    err = ref - measure;

    // 如果在死区外,则计算PID
    if (fabsf(err) > dead_band) {
        // 基本的pid计算,使用位置式
        Pout = Kp * err;
        ITerm = Ki * err * dt;
        Dout = Kd * (err - last_err) / dt;

        // 梯形积分
        if (imp & PID_Trapezoid_Integral)
            f_trapezoid_integral();
        // 变速积分
        if (imp & PID_ChangingIntegrationRate)
            f_changing_integration_rate();
        // 微分先行
        if (imp & PID_Derivative_On_Measurement)
            f_derivative_on_measurement();
        // 微分滤波器
        if (imp & PID_DerivativeFilter)
            f_derivative_filter();
        // 积分限幅
        if (imp & PID_Integral_Limit)
            f_integral_limit();

        Iout += ITerm; // 累加积分
        output = Pout + Iout + Dout; // 计算输出

        // 输出滤波
        if (imp & PID_OutputFilter)
            f_output_filter();

        // 输出限幅
        f_output_limit();
    } else // 进入死区, 则清空积分和输出
    {
        output = 0;
        ITerm = 0;
        Iout = 0;
    }

    // 保存当前数据,用于下次计算
    last_measure = measure;
    last_output = output;
    last_Dout = Dout;
    last_err = err;
    last_ITerm = ITerm;

    return output;
}

/**
 * @brief 梯形积分计算函数，使用梯形法则计算积分项的值，函数内部会根据当前误差和上一次误差的平均值乘以积分增益和时间增量来计算积分项的输出
 */
void Controller::f_trapezoid_integral() {
    /* 计算梯形的面积,(上底 + 下底) * 高 / 2 */
    ITerm = Ki * ((err + last_err) / 2) * dt;
}

/**
 * @brief 变速积分计算函数，根据当前误差的绝对值调整积分项的增益，以实现积分分离的效果，函数内部会根据误差的大小与配置的 CoefA 和 CoefB 参数进行比较，并相应地调整积分项的输出，以避免积分过度累积导致系统不稳定
 */
void Controller::f_changing_integration_rate() {
    if (err * Iout > 0) {
        /* 积分呈累积趋势 */
        if (fabsf(err) <= CoefB)
            return; // Full integral
        if (fabsf(err) <= (CoefA + CoefB))
            ITerm *= (CoefA - fabsf(err) + CoefB) / CoefA;
        else // 最大阈值,不使用积分
            ITerm = 0;
    }
}

/**
 * @brief 积分限幅函数，限制积分项的输出以防止积分过度累积导致系统不稳定，函数内部会根据当前输出和积分项的值与配置的最大输出和积分限幅进行比较，并相应地调整积分项的输出，以确保系统在安全范围内运行
 */
void Controller::f_integral_limit() {
    static float temp_Output, temp_Iout;
    temp_Iout = Iout + ITerm;
    temp_Output = Pout + Iout + Dout;
    if (fabsf(temp_Output) > max_out) {
        if (err * Iout > 0) // 积分却还在累积
        {
            ITerm = 0; // 当前积分项置零
        }
    }
    if (temp_Iout > integral_limit) {
        ITerm = 0;
        Iout = integral_limit;
    }
    if (temp_Iout < -integral_limit) {
        ITerm = 0;
        Iout = -integral_limit;
    }
}

/**
 * @brief 微分项计算函数，使用测量值的变化率来计算微分项的输出，函数内部会根据当前测量值与上一次测量值的差值除以时间增量来计算微分项的输出，以提供对系统变化趋势的预测和响应能力
 */
void Controller::f_derivative_on_measurement() {
    Dout = Kd * (last_measure - measure) / dt;
}

/**
 * @brief 微分项滤波函数，使用低通滤波器对微分项的输出进行滤波，以减少高频噪声对系统的影响，函数内部会根据当前微分项的输出和上一次微分项的输出以及配置的滤波器时间常数来计算滤波后的微分项输出，以提高系统的稳定性和响应性能
 */
void Controller::f_derivative_filter() {
    Dout = Dout * dt / (derivative_LPF_RC + dt) +
           last_Dout * derivative_LPF_RC / (derivative_LPF_RC + dt);
}

/**
 * @brief 输出滤波函数，使用低通滤波器对输出进行滤波，以减少高频噪声对系统的影响
 */
void Controller::f_output_filter() {
    output = output * dt / (output_LPF_RC + dt) +
             last_output * output_LPF_RC / (output_LPF_RC + dt);
}

/**
 * @brief 输出限幅函数，限制控制输出的范围以防止过度控制导致系统不稳定，函数内部会根据当前输出与配置的最大输出进行比较，并相应地调整输出值，以确保系统在安全范围内运行
 */
void Controller::f_output_limit() {
    if (output > max_out) {
        output = max_out;
    }
    if (output < -(max_out)) {
        output = -(max_out);
    }
}

/**
 * @brief 错误处理函数，检测系统是否存在电机堵转等异常情况，并进行相应的错误计数和类型记录，以便后续进行故障诊断和处理
 */
void Controller::f_error_handle() {
    /*Motor Blocked Handle*/
    if (fabsf(output) < max_out * 0.001f || fabsf(ref) < 0.0001f)
        return;

    if ((fabsf(ref - measure) / fabsf(ref)) > 0.95f) {
        // Motor blocked counting
        error_handler.err_count++;
    } else {
        error_handler.err_count = 0;
    }

    if (error_handler.err_count > 500) {
        // Motor blocked over 1000times
        error_handler.err_type = ERROR_MOTOR_BLOCKED;
    }
}
