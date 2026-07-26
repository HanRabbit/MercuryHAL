#pragma once

#include <cstdint>
#include "arm_math.h"

/**
 * @brief 拓展卡尔曼滤波器 (Extended Kalman Filter, EKF)
 *
 * 适用于非线性系统的状态估计，使用方式与本目录下的 Controller(PID) 同构：
 * 通过 Config 配置状态维数、观测维数、噪声协方差与初值，并以 4 个函数指针提供
 * 描述系统的非线性模型 f/h 及其雅可比 F/H；构造后调用 init()，随后在任务循环中
 * 周期调用 update() 即可获得最新的状态估计。
 *
 * 系统模型：
 *     x_k = f(x_{k-1}) + w,   w ~ N(0, Q)   // 状态方程（非线性）
 *     z_k = h(x_k)     + v,   v ~ N(0, R)   // 观测方程（非线性）
 *
 * 递推：
 *     预测:  x̂⁻ = f(x̂)            P⁻ = F P Fᵀ + Q
 *     校正:  K  = P⁻ Hᵀ (H P⁻ Hᵀ + R)⁻¹
 *            x̂  = x̂⁻ + K (z − h(x̂⁻))
 *            P  = (I − K H) P⁻
 *
 * 矩阵运算基于 ARM CMSIS-DSP（arm_math.h），全程单精度浮点。
 */
class EKF {
public:
    /**
     * @brief 状态转移函数 f(x)：由上一时刻状态预测当前状态
     * @param x      输入，上一时刻状态估计 (n×1)
     * @param x_pred 输出，预测得到的当前状态 (n×1)
     * @param dt     两次预测之间的时间间隔（s，由滤波器内部用 DWT 计算后传入）
     */
    using StateFunc = void (*)(const float *x, float *x_pred, float dt);

    /**
     * @brief 状态转移雅可比 F = ∂f/∂x，在 x 处求值
     * @param x  输入，求值点状态 (n×1)
     * @param F  输出，雅可比矩阵 (n×n，行优先存储)
     * @param dt 两次预测之间的时间间隔（s）
     */
    using StateJacobianFunc = void (*)(const float *x, float *F, float dt);

    /**
     * @brief 观测函数 h(x)：由状态推算理论观测量
     * @param x      输入，当前预测状态 (n×1)
     * @param z_pred 输出，预测观测量 (m×1)
     */
    using MeasureFunc = void (*)(const float *x, float *z_pred);

    /**
     * @brief 观测雅可比 H = ∂h/∂x，在 x 处求值
     * @param x 输入，求值点状态 (n×1)
     * @param H 输出，雅可比矩阵 (m×n，行优先存储)
     */
    using MeasureJacobianFunc = void (*)(const float *x, float *H);

    /* EKF 初始化配置结构体 */
    struct Config {
        uint16_t state_dim; // 状态维数 n
        uint16_t measure_dim; // 观测维数 m

        StateFunc f; // 状态转移函数 f(x)
        StateJacobianFunc jacobian_f; // 状态转移雅可比 F = ∂f/∂x
        MeasureFunc h; // 观测函数 h(x)
        MeasureJacobianFunc jacobian_h; // 观测雅可比 H = ∂h/∂x

        const float *x0; // 初始状态 (n×1)，为空则置零
        const float *P0; // 初始估计误差协方差 (n×n，行优先)，为空则置单位阵
        const float *Q; // 过程噪声协方差 (n×n，行优先)
        const float *R; // 观测噪声协方差 (m×m，行优先)
    };

    Config init_config{}; // EKF 初始化配置

    explicit EKF(const Config &config);

    ~EKF();

    void init();

    /**
     * @brief 预测步（时间更新）：x̂⁻ = f(x̂)，P⁻ = F P Fᵀ + Q
     */
    void predict();

    /**
     * @brief 校正步（量测更新）：用观测 z 修正预测，更新状态与协方差
     * @param z 当前观测量 (m×1)
     */
    void correct(const float *z);

    /**
     * @brief 滤波一步：依次执行 predict() 与 correct()
     * @param z 当前观测量 (m×1)
     * @return 指向最新状态估计的指针 (n×1)
     */
    const float *update(const float *z);

    /**
     * @brief 获取当前状态估计向量
     * @return 指向状态估计的指针 (n×1)
     */
    const float *state() const;

    /**
     * @brief 获取当前状态估计的某一分量
     * @param i 分量下标
     * @return 第 i 个状态分量
     */
    float getState(uint16_t i) const;

private:
    uint16_t n{}; // 状态维数
    uint16_t m{}; // 观测维数

    StateFunc f{}; // 状态转移函数 f(x)
    StateJacobianFunc jacobian_f{}; // 状态转移雅可比 F
    MeasureFunc h{}; // 观测函数 h(x)
    MeasureJacobianFunc jacobian_h{}; // 观测雅可比 H

    uint32_t DWT_CNT{};
    float dt{};

    /* 滤波器矩阵（均为行优先存储） */
    arm_matrix_instance_f32 xhat{}; // x̂   状态估计 (n×1)
    arm_matrix_instance_f32 xhatminus{}; // x̂⁻  预测状态 (n×1)
    arm_matrix_instance_f32 z_pred{}; // h(x̂⁻) 预测观测 (m×1)
    arm_matrix_instance_f32 innovation{}; // y = z − h(x̂⁻) 新息 (m×1)

    arm_matrix_instance_f32 F{}; // 状态转移雅可比 (n×n)
    arm_matrix_instance_f32 Ft{}; // Fᵀ (n×n)
    arm_matrix_instance_f32 H{}; // 观测雅可比 (m×n)
    arm_matrix_instance_f32 Ht{}; // Hᵀ (n×m)

    arm_matrix_instance_f32 P{}; // 估计误差协方差 (n×n)
    arm_matrix_instance_f32 Pminus{}; // 预测误差协方差 (n×n)
    arm_matrix_instance_f32 Q{}; // 过程噪声协方差 (n×n)
    arm_matrix_instance_f32 R{}; // 观测噪声协方差 (m×m)
    arm_matrix_instance_f32 K{}; // 卡尔曼增益 (n×m)

    arm_matrix_instance_f32 S{}; // 新息协方差 (m×m)
    arm_matrix_instance_f32 Sinv{}; // S⁻¹ (m×m)
    arm_matrix_instance_f32 eye{}; // 单位阵 (n×n)

    /* 中间运算缓存 */
    arm_matrix_instance_f32 tmp_nn{}; // (n×n)
    arm_matrix_instance_f32 tmp_nn2{}; // (n×n)
    arm_matrix_instance_f32 tmp_mn{}; // (m×n)
    arm_matrix_instance_f32 tmp_nm{}; // (n×m)
    arm_matrix_instance_f32 tmp_mm{}; // (m×m)
    arm_matrix_instance_f32 tmp_n1{}; // (n×1)

    static void createMatrix(arm_matrix_instance_f32 &mat, uint16_t rows, uint16_t cols);
};
