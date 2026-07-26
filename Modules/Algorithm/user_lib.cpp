#include "user_lib.h"
#include <cmath>

/**
 * @brief 计算两个角度之间的差值，结果在(-π, π]范围内
 * @param target 目标角度（单位：rad）
 * @param current 当前角度（单位：rad）
 * @return 角度差值（单位：rad）
 */
float angle_diff_rad(const float target, const float current) {
    float diff = fmodf(static_cast<float>(target - current + M_PI), 2.0f * M_PI);

    if (diff < 0)
        diff += 2.0f * M_PI;

    return diff - static_cast<float>(M_PI);
}

/**
 * @brief 使用角度连续展开（unwrap）更新未包装的角度值
 * @param meas_wrapped 当前测量的包装角度（单位：rad，范围：-π到π）
 * @param last_wrapped 指向上一次测量的包装角度的指针
 * @param unwrapped 指向未包装角度的指针
 * @return 更新后的未包装角度值（单位：rad）
 */
float angle_unwrap_update(const float meas_wrapped, float *last_wrapped, float *unwrapped) {
    const float delta = angle_diff_rad(meas_wrapped, *last_wrapped);
    *unwrapped += delta;
    *last_wrapped = meas_wrapped;
    return *unwrapped;
}
