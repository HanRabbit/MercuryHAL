#ifndef MIAO_BOARD_USER_LIB_H
#define MIAO_BOARD_USER_LIB_H

float angle_diff_rad(float target, float current);
float angle_unwrap_update(float meas_wrapped, float *last_wrapped, float *unwrapped);

#endif //MIAO_BOARD_USER_LIB_H
