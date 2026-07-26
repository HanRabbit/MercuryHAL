#include "dji_motor.h"

#include "arm_math.h"
#include "bsp_dwt.h"
#include "bsp_log.h"
#include "general_def.h"

#include <cstring>
#include <new>

DJI_Motor *DJI_Motor::instances_[MAX_CNT] = {nullptr};
uint8_t DJI_Motor::count_ = 0;

/**
 * DJI group TX: up to kMaxBus buses × 3 TX IDs (0x1FF / 0x200 / 0x2FF).
 * Buses are discovered from motor Config.hfdcan — no board-level hfdcan1/2.
 *
 * C610(m2006)/C620(m3508): tx 0x1FF / 0x200
 * GM6020:                  tx 0x1FF / 0x2FF
 * Feedback rx: GM6020 0x204+id ; C610/C620 0x200+id
 */
namespace {
constexpr uint8_t kMaxBus = 4U;
constexpr uint8_t kGroupsPerBus = 3U;
constexpr uint8_t kMaxSenders = kMaxBus * kGroupsPerBus;
constexpr uint16_t kGroupTxId[kGroupsPerBus] = {0x1FFU, 0x200U, 0x2FFU};

FDCAN_HandleTypeDef *s_buses[kMaxBus] = {};
uint8_t s_busCount = 0U;

alignas(FDCAN) unsigned char s_senderStorage[kMaxSenders][sizeof(FDCAN)];
FDCAN *s_senders[kMaxSenders] = {};
uint8_t s_senderEnable[kMaxSenders] = {};

uint8_t findOrRegisterBus(FDCAN_HandleTypeDef *const h) {
    for (uint8_t i = 0U; i < s_busCount; ++i) {
        if (s_buses[i] == h) {
            return i;
        }
    }
    if (h == nullptr || s_busCount >= kMaxBus) {
        LOG_ERROR("[DJI Motor] FDCAN bus register failed (null or >%u buses)",
                  static_cast<unsigned>(kMaxBus));
        for (;;) {
        }
    }
    const uint8_t bus = s_busCount++;
    s_buses[bus] = h;
    for (uint8_t g = 0U; g < kGroupsPerBus; ++g) {
        const uint8_t idx = static_cast<uint8_t>(bus * kGroupsPerBus + g);
        s_senders[idx] = new (s_senderStorage[idx]) FDCAN({
            .hfdcan = h,
            .tx_id = kGroupTxId[g],
            .rx_id = 0U,
            .callback = nullptr,
            .owner = nullptr,
        });
    }
    return bus;
}

FDCAN *senderAt(const uint8_t group) {
    return s_senders[group];
}
} // namespace

/**
 * @brief DJI_Motor类的构造函数,用于初始化电机实例的配置参数,注册CAN接收回调函数以及将电机实例添加到静态实例数组中
 *
 * @param config 电机配置参数,包括电机类型,控制器设置,CAN初始化配置等
 */
DJI_Motor::DJI_Motor(const Config &config) {
    init_config = config;
    motor_type = config.type;
    motor_settings = config.controller_settings_init_config;
    stop_flag = MOTOR_STOP;

    motor_controller.other_angle_feedback_ptr = config.controller_param_init_config.other_angle_feedback_ptr;
    motor_controller.other_speed_feedback_ptr = config.controller_param_init_config.other_speed_feedback_ptr;
    motor_controller.speed_feedforward_ptr = config.controller_param_init_config.speed_feedforward_ptr;
    motor_controller.current_feedforward_ptr = config.controller_param_init_config.current_feedforward_ptr;

    /* 检查是否超出电机注册最大数量数量限制 */
    if (count_ < MAX_CNT) {
        instances_[count_++] = this;
    } else {
        for (;;)
            LOG_ERROR("[DJI Motor] Instance count exceeds maximum limit!");
    }

    init_config.can_init_config.owner = this;
}

/** * @brief 初始化电机,包括PID控制器的初始化,电机分组发送的设置,CAN接收回调函数的注册以及电机喂狗的设置
 *
 * @note 该函数需要在创建电机实例后调用,才能使能电机并开始接收反馈数据
 */
void DJI_Motor::init() {
    motor_controller.PID_current = new Controller(init_config.controller_param_init_config.PID_current);
    motor_controller.PID_speed = new Controller(init_config.controller_param_init_config.PID_speed);
    motor_controller.PID_angle = new Controller(init_config.controller_param_init_config.PID_angle);

    motor_controller.PID_current->init();
    motor_controller.PID_speed->init();
    motor_controller.PID_angle->init();

    /* 电机分组发送 */
    senderGrouping();

    init_config.can_init_config.callback = decode;
    motor_can_instance = new FDCAN(init_config.can_init_config);

    /* 电机喂狗 */
    daemon = new Daemon({
        .reload_count = 2, // 设置超过 20ms 之后为超时，触发回调函数
        .init_count = 0,
        .callback = lostCallback,
        .owner = this
    });

    /* 使能电机 */
    enable();
}

/**
 * @brief 解析电机反馈数据,并进行多圈角度计算
 *
 * @param can_instance 接收电机反馈数据的CAN实例
 */
void DJI_Motor::decode(FDCAN *can_instance) {
    const uint8_t *rx_buffer = can_instance->getRxBuff();
    auto *motor = static_cast<DJI_Motor *>(can_instance->getOwner());
    motor->dt = DWT_Time::getDeltaT(&motor->feed_cnt);

    /* 解析电机反馈数据 */
    motor->measure.last_ecd = motor->measure.ecd;
    motor->measure.ecd = (static_cast<uint16_t>(rx_buffer[0])) << 8 | rx_buffer[1];
    motor->measure.pos_rad = DJI_ECD_RAD_COEF * static_cast<float>(motor->measure.ecd);
    motor->measure.vel_rad = (1.0f - DJI_MOTOR_SPEED_CONF) * motor->measure.vel_rad +
                             RPM_2_RAD_PER_SEC * DJI_MOTOR_SPEED_CONF * static_cast<float>(static_cast<int16_t>(
                                 rx_buffer[2] << 8 | rx_buffer[3]));
    motor->measure.current = static_cast<int16_t>(
        (1.0f - DJI_MOTOR_CURRENT_CONF) * static_cast<float>(motor->measure.current) +
        DJI_MOTOR_CURRENT_CONF * static_cast<float>(static_cast<int16_t>(
            rx_buffer[4] << 8 | rx_buffer[5])));
    motor->measure.temperature = rx_buffer[6];

    motor->measure.vel_rad_filtered = motor->measure.vel_rad * motor->dt / (motor->init_config.vel_LPF_RC + motor->dt)
    + motor->measure.vel_rad_filtered * motor->init_config.vel_LPF_RC / (motor->init_config.vel_LPF_RC + motor->dt);

    /* 多圈角度计算 */
    if (motor->measure.ecd - motor->measure.last_ecd > 4096)
        motor->measure.total_round--;
    else if (motor->measure.ecd - motor->measure.last_ecd < -4096)
        motor->measure.total_round++;
    motor->measure.pos_total_rad = motor->measure.total_round * 2.0f * PI + motor->measure.pos_rad;
}

/**
 * @brief 将电机按照发送ID进行分组,每组四个电机,并检查是否存在ID冲突
 *
 * @note 发送ID和接收ID的关系: 发送ID = 0x1FF/0x200/0x2FF, 接收ID = 发送ID + 1
 */
void DJI_Motor::senderGrouping() {
    const uint8_t motor_id = static_cast<uint8_t>(init_config.can_init_config.tx_id - 1U);
    const uint8_t bus = findOrRegisterBus(init_config.can_init_config.hfdcan);
    uint8_t motor_send_num = 0U;
    uint8_t group_in_bus = 0U; /* 0=0x1FF, 1=0x200, 2=0x2FF */

    switch (motor_type) {
        case M3508:
        case M3508P19:
        case M2006: {
            init_config.can_init_config.rx_id = static_cast<uint16_t>(0x200U + motor_id + 1U);
            if (motor_id < 4U) {
                motor_send_num = motor_id;
                group_in_bus = 1U; /* 0x200 */
            } else {
                motor_send_num = static_cast<uint8_t>(motor_id - 4U);
                group_in_bus = 0U; /* 0x1FF */
            }
            break;
        }
        case GM6020: {
            init_config.can_init_config.rx_id = static_cast<uint16_t>(0x204U + motor_id + 1U);
            if (motor_id < 4U) {
                motor_send_num = motor_id;
                group_in_bus = 0U; /* 0x1FF */
            } else {
                motor_send_num = static_cast<uint8_t>(motor_id - 4U);
                group_in_bus = 2U; /* 0x2FF */
            }
            break;
        }
        default: {
            LOG_ERROR("[DJI Motor] Unknown motor type!");
            for (;;) {
            }
        }
    }

    sender_group = static_cast<uint8_t>(bus * kGroupsPerBus + group_in_bus);
    message_num = motor_send_num;
    s_senderEnable[sender_group] = 1U;

    for (uint8_t i = 0U; i < count_; ++i) {
        const DJI_Motor *const motor = instances_[i];
        if (motor == this || motor->motor_can_instance == nullptr) {
            continue;
        }
        if (motor->motor_can_instance->getHandle() == init_config.can_init_config.hfdcan
            && motor->motor_can_instance->getRxId() == init_config.can_init_config.rx_id) {
            LOG_ERROR("[DJI Motor] bus%u ID 0x%03X conflict!",
                      static_cast<unsigned>(bus),
                      static_cast<unsigned>(init_config.can_init_config.tx_id));
            for (;;) {
            }
        }
    }
}

/**
 * @brief DJI电机丢失回调函数，当电机长时间未收到反馈时触发。
 *
 * @param motor_ptr 指向 Motor 类的指针，表示丢失的电机实例。
 */
void DJI_Motor::lostCallback(void *motor_ptr) {
    const DJI_Motor *const motor = static_cast<DJI_Motor *>(motor_ptr);
    LOG_WARNING("[DJI Motor] motor rx ID 0x%03X lost (hfdcan=%p)!",
                static_cast<unsigned>(motor->motor_can_instance->getRxId()),
                static_cast<void *>(motor->motor_can_instance->getHandle()));
}


/**
 * @brief 设置电机PID控制的反馈源,位置环和速度环可以分别设置
 *
 * @param loop 位置环或速度环
 * @param type 反馈源类型,可以是电机自带的反馈或者外部传感器的反馈
 */
void DJI_Motor::setFeed(const CloseLoop_Type_e loop, Feedback_Source_e type) {
    if (loop == POSITION_LOOP)
        motor_settings.angle_feedback_source = type;
    else if (loop == SPEED_LOOP)
        motor_settings.speed_feedback_source = type;
    else
        LOG_ERROR("[DJI Motor] Unknown loop type!");
}

/**
 * @brief 停止电机,设置停止标志位,在控制函数中会根据该标志位决定是否发送控制命令
 */
void DJI_Motor::stop() {
    stop_flag = MOTOR_STOP;
}

/**
 * @brief 使能电机,设置使能标志位,在控制函数中会根据该标志位决定是否发送控制命令
 */
void DJI_Motor::enable() {
    stop_flag = MOTOR_ENABLED;
}

/**
 * @brief 设置PID控制的外环类型,可以是位置环或者速度环
 *
 * @param outer_loop 外环类型
 */
void DJI_Motor::setOuterLoop(const CloseLoop_Type_e outer_loop) {
    motor_settings.outer_loop_type = outer_loop;
}

/**
 * @brief 设置PID控制的参考值,根据外环类型的不同,该参考值可以是位置或者速度
 *
 * @param ref 参考值
 */
void DJI_Motor::setRef(const float ref) {
    motor_controller.pid_ref = ref;
}

/**
 * @brief 电机控制函数,遍历所有电机实例,根据其配置进行PID计算并设置发送报文的值,最后根据发送标志位决定是否发送报文
 *
 * @note 该函数需要在电机任务中周期调用,以实现对电机的实时控制
 */
void DJI_Motor::control() {
    float pid_measure;

    // 遍历所有电机实例,进行串级PID的计算并设置发送报文的值
    for (size_t i = 0; i < count_; ++i) {
        // 减小访存开销,先保存指针引用
        DJI_Motor *motor = instances_[i];
        const Motor_Control_Settings_s *motor_setting = &motor->motor_settings;
        Motor_Controller_s *motor_controller = &motor->motor_controller;
        const DJI_Motor_Measure_t *measure = &motor->measure;

        float pid_ref = motor_controller->pid_ref; // 保存设定值,防止motor_controller->pid_ref在计算过程中被修改
        if (motor_setting->motor_reverse_flag == MOTOR_DIRECTION_REVERSE)
            pid_ref *= -1; // 设置反转

        // pid_ref会顺次通过被启用的闭环充当数据的载体
        // 计算位置环,只有启用位置环且外层闭环为位置时会计算速度环输出
        if ((motor_setting->close_loop_type & POSITION_LOOP) && motor_setting->outer_loop_type == POSITION_LOOP) {
            if (motor_setting->angle_feedback_source == OTHER_FEED)
                pid_measure = *motor_controller->other_angle_feedback_ptr;
            else
                pid_measure = measure->pos_total_rad; // MOTOR_FEED,对total rad闭环,防止在边界处出现突跃
            // 更新pid_ref进入下一个环
            pid_ref = motor_controller->PID_angle->calculate(pid_measure, pid_ref);
            // 计算位置环,但不更新pid_ref,因为位置环的输出不直接作为下一个环的设定值,而是作为速度环的测量值
        }

        // 计算速度环,(外层闭环为速度或位置)且(启用速度环)时会计算速度环
        if ((motor_setting->close_loop_type & SPEED_LOOP) && (
                motor_setting->outer_loop_type & (POSITION_LOOP | SPEED_LOOP))) {
            if (motor_setting->feedforward_flag & SPEED_FEEDFORWARD)
                pid_ref += *motor_controller->speed_feedforward_ptr;

            if (motor_setting->speed_feedback_source == OTHER_FEED)
                pid_measure = *motor_controller->other_speed_feedback_ptr;
            else // MOTOR_FEED
                pid_measure = measure->vel_rad_filtered;
            // 更新pid_ref进入下一个环
            pid_ref = motor_controller->PID_speed->calculate(pid_measure, pid_ref);
        }

        // 计算电流环,目前只要启用了电流环就计算,不管外层闭环是什么,并且电流只有电机自身传感器的反馈
        if (motor_setting->feedforward_flag & CURRENT_FEEDFORWARD)
            pid_ref += *motor_controller->current_feedforward_ptr;
        if (motor_setting->close_loop_type & CURRENT_LOOP) {
            pid_ref = motor_controller->PID_current->calculate(measure->current, pid_ref);
        }

        if (motor_setting->feedback_reverse_flag == FEEDBACK_DIRECTION_REVERSE)
            pid_ref *= -1;

        // 获取最终输出
        const auto set = static_cast<int16_t>(pid_ref);

        // 分组填入发送数据
        const uint8_t group = motor->sender_group;
        const uint8_t num = motor->message_num;
        senderAt(group)->getTxBuff()[2 * num] = static_cast<uint8_t>(set >> 8); // 低八位
        senderAt(group)->getTxBuff()[2 * num + 1] = static_cast<uint8_t>(set & 0x00ff); // 高八位

        // 若该电机处于停止状态,直接将buff置零
        if (motor->stop_flag == MOTOR_STOP)
            memset(senderAt(group)->getTxBuff() + 2 * num, 0, 2u);
    }

    const uint8_t sender_total = static_cast<uint8_t>(s_busCount * kGroupsPerBus);
    for (uint8_t i = 0U; i < sender_total; ++i) {
        if (s_senderEnable[i] && s_senders[i] != nullptr) {
            if (!s_senders[i]->transmit()) {
                LOG_ERROR("[DJI Motor] Failed to send transmission on group %u!",
                          static_cast<unsigned>(i));
            }
        }
    }
}
