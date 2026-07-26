#include "ekf.h"
#include <cstring>
#include "bsp_dwt.h"

/**
 * @brief 构造函数，拷贝配置、申请各矩阵缓存并写入初值。状态维数 n 与观测维数 m
 *        来自配置；Q/R/x0/P0 在此处一次性拷入内部缓存，构造完成后不再依赖外部指针
 * @param config EKF 初始化配置结构体
 */
EKF::EKF(const Config &config) {
    this->init_config = config;
    n = config.state_dim;
    m = config.measure_dim;
    f = config.f;
    jacobian_f = config.jacobian_f;
    h = config.h;
    jacobian_h = config.jacobian_h;

    // 申请各矩阵缓存（行优先存储，零初始化）
    createMatrix(xhat, n, 1);
    createMatrix(xhatminus, n, 1);
    createMatrix(z_pred, m, 1);
    createMatrix(innovation, m, 1);
    createMatrix(F, n, n);
    createMatrix(Ft, n, n);
    createMatrix(H, m, n);
    createMatrix(Ht, n, m);
    createMatrix(P, n, n);
    createMatrix(Pminus, n, n);
    createMatrix(Q, n, n);
    createMatrix(R, m, m);
    createMatrix(K, n, m);
    createMatrix(S, m, m);
    createMatrix(Sinv, m, m);
    createMatrix(eye, n, n);
    createMatrix(tmp_nn, n, n);
    createMatrix(tmp_nn2, n, n);
    createMatrix(tmp_mn, m, n);
    createMatrix(tmp_nm, n, m);
    createMatrix(tmp_mm, m, m);
    createMatrix(tmp_n1, n, 1);

    // 构造单位阵
    for (uint16_t i = 0; i < n; i++)
        eye.pData[i * n + i] = 1.0f;

    // 写入噪声协方差与初值
    if (init_config.Q)
        memcpy(Q.pData, init_config.Q, sizeof(float) * n * n);
    if (init_config.R)
        memcpy(R.pData, init_config.R, sizeof(float) * m * m);
    if (init_config.x0)
        memcpy(xhat.pData, init_config.x0, sizeof(float) * n); // 否则保持零

    if (init_config.P0) {
        memcpy(P.pData, init_config.P0, sizeof(float) * n * n);
    } else {
        // 默认初始协方差取单位阵
        for (uint16_t i = 0; i < n; i++)
            P.pData[i * n + i] = 1.0f;
    }
}

/**
 * @brief 析构函数，释放所有矩阵缓存
 */
EKF::~EKF() {
    arm_matrix_instance_f32 *mats[] = {
        &xhat, &xhatminus, &z_pred, &innovation,
        &F, &Ft, &H, &Ht,
        &P, &Pminus, &Q, &R, &K,
        &S, &Sinv, &eye,
        &tmp_nn, &tmp_nn2, &tmp_mn, &tmp_nm, &tmp_mm, &tmp_n1,
    };
    for (auto *mat: mats)
        delete[] mat->pData;
}

/**
 * @brief 申请并初始化一个行优先存储的浮点矩阵，缓存零初始化
 * @param mat  待初始化的矩阵实例
 * @param rows 行数
 * @param cols 列数
 */
void EKF::createMatrix(arm_matrix_instance_f32 &mat, const uint16_t rows, const uint16_t cols) {
    arm_mat_init_f32(&mat, rows, cols, new float[rows * cols]{});
}

/**
 * @brief 初始化，播种 DWT 计数作为首次预测的时间基准
 */
void EKF::init() {
    DWT_Time::getDeltaT(&DWT_CNT);
}

/**
 * @brief 预测步（时间更新）。利用状态转移函数 f 与其雅可比 F，由上一时刻估计推算
 *        当前时刻的预测状态 x̂⁻ 及预测误差协方差 P⁻ = F P Fᵀ + Q。时间间隔 dt 由
 *        DWT 实时计算并传入用户模型
 */
void EKF::predict() {
    if (!f || !jacobian_f)
        return;

    dt = DWT_Time::getDeltaT(&DWT_CNT); // 获取两次预测的时间间隔

    // 状态预测: x̂⁻ = f(x̂)
    f(xhat.pData, xhatminus.pData, dt);
    // 状态转移雅可比: F = ∂f/∂x |_{x̂}
    jacobian_f(xhat.pData, F.pData, dt);

    // 协方差预测: P⁻ = F P Fᵀ + Q
    arm_mat_trans_f32(&F, &Ft);
    arm_mat_mult_f32(&F, &P, &tmp_nn); // F P
    arm_mat_mult_f32(&tmp_nn, &Ft, &tmp_nn2); // F P Fᵀ
    arm_mat_add_f32(&tmp_nn2, &Q, &Pminus); // + Q
}

/**
 * @brief 校正步（量测更新）。利用观测 z 修正预测：计算卡尔曼增益 K，更新状态估计
 *        x̂ 与估计误差协方差 P。当新息协方差 S 奇异（不可逆）时放弃本次校正，保留
 *        预测值以保证数值稳定
 * @param z 当前观测量 (m×1)
 */
void EKF::correct(const float *z) {
    if (!h || !jacobian_h)
        return;

    // 观测预测: ẑ = h(x̂⁻)，观测雅可比: H = ∂h/∂x |_{x̂⁻}
    h(xhatminus.pData, z_pred.pData);
    jacobian_h(xhatminus.pData, H.pData);
    arm_mat_trans_f32(&H, &Ht);

    // 新息协方差: S = H P⁻ Hᵀ + R
    arm_mat_mult_f32(&H, &Pminus, &tmp_mn); // H P⁻
    arm_mat_mult_f32(&tmp_mn, &Ht, &tmp_mm); // H P⁻ Hᵀ
    arm_mat_add_f32(&tmp_mm, &R, &S); // + R

    // 卡尔曼增益: K = P⁻ Hᵀ S⁻¹（S 奇异时保留预测值并返回）
    if (arm_mat_inverse_f32(&S, &Sinv) != ARM_MATH_SUCCESS) {
        memcpy(xhat.pData, xhatminus.pData, sizeof(float) * n);
        memcpy(P.pData, Pminus.pData, sizeof(float) * n * n);
        return;
    }
    arm_mat_mult_f32(&Pminus, &Ht, &tmp_nm); // P⁻ Hᵀ
    arm_mat_mult_f32(&tmp_nm, &Sinv, &K); // K = P⁻ Hᵀ S⁻¹

    // 状态校正: x̂ = x̂⁻ + K (z − ẑ)
    arm_matrix_instance_f32 zmat;
    arm_mat_init_f32(&zmat, m, 1, const_cast<float *>(z));
    arm_mat_sub_f32(&zmat, &z_pred, &innovation); // y = z − ẑ
    arm_mat_mult_f32(&K, &innovation, &tmp_n1); // K y
    arm_mat_add_f32(&xhatminus, &tmp_n1, &xhat); // x̂⁻ + K y

    // 协方差校正: P = (I − K H) P⁻
    arm_mat_mult_f32(&K, &H, &tmp_nn); // K H
    arm_mat_sub_f32(&eye, &tmp_nn, &tmp_nn2); // I − K H
    arm_mat_mult_f32(&tmp_nn2, &Pminus, &P); // (I − K H) P⁻
}

/**
 * @brief 滤波一步，依次执行预测与校正
 * @param z 当前观测量 (m×1)
 * @return 指向最新状态估计的指针 (n×1)
 */
const float *EKF::update(const float *z) {
    predict();
    correct(z);
    return xhat.pData;
}

/**
 * @brief 获取当前状态估计向量
 * @return 指向状态估计的指针 (n×1)
 */
const float *EKF::state() const {
    return xhat.pData;
}

/**
 * @brief 获取当前状态估计的某一分量
 * @param i 分量下标
 * @return 第 i 个状态分量
 */
float EKF::getState(const uint16_t i) const {
    return xhat.pData[i];
}
