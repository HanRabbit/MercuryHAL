#pragma once
#include "daemon.h"
#include "dji_motor.h"
#include "dm_motor.h"

/**
 * @brief 电机任务函数,在该函数中周期调用DJI_Motor::control()函数以实现对电机的实时控制
 *
 * @note 该函数需要在电机任务中周期调用,以实现对电机的实时控制
 */
inline void Motor_Task() {
    Daemon::task(); /* 喂狗/掉线检测 (电机、视觉等共用) */
    DJI_Motor::control();
    DM_Motor::control();
}
