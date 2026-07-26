#include "dm_motor.h"

#include <cstring>

#include "bsp_dwt.h"
#include "bsp_log.h"
#include "bsp_usart.h"
#include "bsp_usb.h"

DM_Motor *DM_Motor::instances_[MAX_CNT] = {nullptr};
uint8_t DM_Motor::count_ = 0;

namespace {
void controlDebug(USART *usart, uint16_t size);
void debugToMotor(uint16_t len);

USB *s_debugUsb = nullptr;
USART *s_debugUsart = nullptr;
bool s_debugLinkReady = false;

void ensureDebugLink(UART_HandleTypeDef *const huart) {
    if (s_debugLinkReady) {
        return;
    }
    if (huart == nullptr) {
        LOG_ERROR("[DM Motor] debug=true requires Config.debug_huart");
        return;
    }
    static USB usb;
    static USART usart(huart, USART_RX_BUFFER_MAX, controlDebug);
    s_debugUsb = &usb;
    s_debugUsart = &usart;
    (void)usb.init(debugToMotor, nullptr);
    usart.init();
    s_debugLinkReady = true;
    LOG_INFO("[DM Motor] debug link: USB CDC <-> UART ready");
}
} // namespace

/**
 * @brief DM_Motor 构造函数, 初始化配置并注册实例
 *
 * @param config 电机配置参数, 包括控制模式、CAN_ID、反馈 ID、映射范围以及 CAN 配置等
 *
 * @note 控制帧发送 ID = (mode << 8) + can_id, 反馈接收 ID = master_id.
 *       由于达妙驱动只校验帧 ID 的低 8 位(高 3 位忽略), 故各模式的偏移
 *       (0x100/0x200/0x300) 与命令帧均能被同一电机正确接收.
 */
DM_Motor::DM_Motor(const Config &config) {
    init_config = config;
    mode = config.mode;

    /* 计算控制帧发送 ID 与反馈接收 ID */
    init_config.can_init_config.tx_id =
            static_cast<uint16_t>(static_cast<uint16_t>(config.mode) << 8) + config.can_id;
    init_config.can_init_config.rx_id = config.master_id;
    init_config.can_init_config.owner = this;

    if (init_config.debug) {
        /* Optional debug bridge for Damiao PC tools — inject UART via Config. */
        ensureDebugLink(init_config.debug_huart);
    }

    /* 检查是否超出电机注册最大数量限制 */
    if (count_ < MAX_CNT) {
        instances_[count_++] = this;
    } else {
        for (;;)
            LOG_ERROR("[DM Motor] Instance count exceeds maximum limit!");
    }
}

/**
 * @brief 初始化电机, 创建喂狗与 CAN 实例, 并做反馈 ID 冲突检测
 *
 * @note 需在创建电机实例后调用. 先创建 Daemon 再创建 CAN 实例, 以保证
 *       CAN 接收回调触发时 Daemon 已存在.
 */
void DM_Motor::init() {
    /* 电机喂狗: 超过 reload_count 个 Daemon::task() 周期未收到反馈即判定离线 */
    daemon = new Daemon({
        .reload_count = 5,
        .init_count = 0,
        .callback = lostCallback,
        .owner = this
    });

    /* 注册 CAN 接收回调 */
    init_config.can_init_config.callback = decode;
    motor_can_instance = new FDCAN(init_config.can_init_config);

    /* 反馈 ID 冲突检测(同一总线上 master_id 不可重复) */
    for (uint8_t i = 0; i < count_; i++) {
        const DM_Motor *other = instances_[i];
        if (other == this || other->motor_can_instance == nullptr)
            continue;
        if (other->motor_can_instance->getHandle() == init_config.can_init_config.hfdcan
            && other->motor_can_instance->getRxId() == init_config.can_init_config.rx_id) {
            LOG_ERROR("[DM Motor] feedback ID 0x%03X conflict (hfdcan=%p)!",
                      static_cast<unsigned>(init_config.can_init_config.rx_id),
                      static_cast<void *>(init_config.can_init_config.hfdcan));
            for (;;) {}
        }
    }
}

/**
 * @brief 解析电机反馈帧, 更新测量数据并进行多圈角度计算
 *
 * @param can_instance 接收电机反馈数据的 CAN 实例
 *
 * @note 反馈帧格式(各模式统一):
 *       D[0]=ID|ERR<<4, D[1..2]=POS[15:0], D[3]=VEL[11:4],
 *       D[4]=VEL[3:0]|T[11:8], D[5]=T[7:0], D[6]=T_MOS, D[7]=T_Rotor.
 *       POS 为 16 位, VEL/T 为 12 位, 均按线性映射还原为浮点.
 */
void DM_Motor::decode(FDCAN *can_instance) {
    const uint8_t *rx = can_instance->getRxBuff();
    auto *motor = static_cast<DM_Motor *>(can_instance->getOwner());

    motor->dt = DWT_Time::getDeltaT(&motor->feed_cnt);
    motor->daemon->reload(); /* 收到反馈即喂狗 */

    DM_Motor_Measure_t &m = motor->measure;
    const Config &cfg = motor->init_config;

    /* 拆分状态与定点数据 */
    m.id = rx[0] & 0x0F;
    m.state = rx[0] >> 4;
    const int p_int = rx[1] << 8 | rx[2];
    const int v_int = rx[3] << 4 | rx[4] >> 4;
    const int t_int = (rx[4] & 0x0F) << 8 | rx[5];

    /* 线性映射还原为浮点 */
    m.last_pos_rad = m.pos_rad;
    m.pos_rad = uint_to_float(p_int, -cfg.p_max, cfg.p_max, 16);
    m.vel_rad = uint_to_float(v_int, -cfg.v_max, cfg.v_max, 12);
    m.torque = uint_to_float(t_int, -cfg.t_max, cfg.t_max, 12);
    m.t_mos = rx[6];
    m.t_rotor = rx[7];

    /* 速度低通滤波 */
    m.vel_rad_filtered = m.vel_rad * motor->dt / (cfg.vel_LPF_RC + motor->dt)
                         + m.vel_rad_filtered * cfg.vel_LPF_RC / (cfg.vel_LPF_RC + motor->dt);

    /* 多圈角度计算: 位置在 [-p_max, p_max] 内回绕, 跨越即累加圈数 */
    if (m.pos_rad - m.last_pos_rad < -cfg.p_max)
        m.total_round++;
    else if (m.pos_rad - m.last_pos_rad > cfg.p_max)
        m.total_round--;
    m.total_pos_rad = static_cast<float>(m.total_round) * 2.0f * cfg.p_max + m.pos_rad;
}

/**
 * @brief 电机丢失回调, 长时间未收到反馈时触发
 *
 * @param motor_ptr 指向丢失的 DM_Motor 实例
 *
 * @note 失能状态下电机不发送反馈属正常现象, 故不告警.
 */
void DM_Motor::lostCallback(void *motor_ptr) {
    const DM_Motor *const motor = static_cast<DM_Motor *>(motor_ptr);
    if (!motor->enabled) {
        return;
    }
    LOG_WARNING("[DM Motor] feedback ID 0x%03X lost (hfdcan=%p)!",
                static_cast<unsigned>(motor->motor_can_instance->getRxId()),
                static_cast<void *>(motor->motor_can_instance->getHandle()));
}

/**
 * @brief 发送控制命令帧(使能/失能/保存零点/清除错误)
 *
 * @param cmd 命令字节, 填入数据段 D[7], 其余 7 字节为 0xFF
 */
void DM_Motor::sendCommand(const uint8_t cmd) {
    uint8_t *tx = motor_can_instance->getTxBuff();
    memset(tx, 0xFF, 7);
    tx[7] = cmd;
    motor_can_instance->setDLC(8);
    if (!motor_can_instance->transmit())
        LOG_ERROR("[DM Motor] Failed to send command 0x%02X!", cmd);
}

/**
 * @brief 使能电机, 上电自检后需先使能才能进行控制
 */
void DM_Motor::enable() {
    sendCommand(DM_CMD_ENABLE);
    /* 使能时喂狗, 为首帧反馈预留完整宽限期; 先喂狗再置位, 避免与中断中的 Daemon::task() 竞争 */
    daemon->reload();
    enabled = true;
}

/**
 * @brief 失能电机
 */
void DM_Motor::disable() {
    sendCommand(DM_CMD_DISABLE);
    enabled = false;
}

/**
 * @brief 保存当前输出轴位置为零点, 并将位置给定清零
 */
void DM_Motor::saveZero() {
    sendCommand(DM_CMD_SAVE_ZERO);
    p_des = 0.0f;
}

/**
 * @brief 清除电机错误(过热等)
 */
void DM_Motor::clearError() {
    sendCommand(DM_CMD_CLEAR_ERROR);
}

/**
 * @brief 设置 MIT 模式控制量
 *
 * @param pos    位置给定(rad)
 * @param vel    速度给定(rad/s)
 * @param kp     位置比例系数, 范围 [0,500]
 * @param kd     位置微分系数, 范围 [0,5]
 * @param torque 转矩前馈(N·m)
 *
 * @note 对位置进行控制时, kd 不能为 0, 否则会造成电机震荡甚至失控.
 */
void DM_Motor::setMIT(const float pos, const float vel, const float kp, const float kd, const float torque) {
    p_des = pos;
    v_des = vel;
    this->kp = kp;
    this->kd = kd;
    t_ff = torque;
}

/**
 * @brief 设置位置速度模式控制量
 *
 * @param pos 目标位置(rad)
 * @param vel 梯形加减速下的匀速段速度(rad/s)
 */
void DM_Motor::setPosVel(const float pos, const float vel) {
    p_des = pos;
    v_des = vel;
}

/**
 * @brief 设置速度模式控制量
 *
 * @param vel 目标速度(rad/s)
 */
void DM_Motor::setVel(const float vel) {
    v_des = vel;
}

/**
 * @brief 设置力位混控模式控制量
 *
 * @param pos 目标位置(rad)
 * @param vel 限速幅值(rad/s), 范围 0~100
 * @param cur 电流限定标幺值, 范围 0~1.0(实际电流/最大相电流)
 */
void DM_Motor::setForcePos(const float pos, const float vel, const float cur) {
    p_des = pos;
    v_des = vel;
    i_des = cur;
}

/**
 * @brief 按当前模式打包控制帧并发送
 *
 * @note 位置速度/速度/力位混控模式下的浮点数据均为低位在前(小端),
 *       与 Cortex-M 内存布局一致, 故可直接内存拷贝.
 */
void DM_Motor::packAndSend() {
    uint8_t *tx = motor_can_instance->getTxBuff();
    const Config &cfg = init_config;

    switch (mode) {
        case DM_MODE_MIT: {
            const int p = float_to_uint(p_des, -cfg.p_max, cfg.p_max, 16);
            const int v = float_to_uint(v_des, -cfg.v_max, cfg.v_max, 12);
            const int kp_int = float_to_uint(kp, DM_MOTOR_KP_MIN, DM_MOTOR_KP_MAX, 12);
            const int kd_int = float_to_uint(kd, DM_MOTOR_KD_MIN, DM_MOTOR_KD_MAX, 12);
            const int t = float_to_uint(t_ff, -cfg.t_max, cfg.t_max, 12);

            tx[0] = p >> 8;
            tx[1] = p & 0xFF;
            tx[2] = v >> 4;
            tx[3] = (v & 0x0F) << 4 | kp_int >> 8;
            tx[4] = kp_int & 0xFF;
            tx[5] = kd_int >> 4;
            tx[6] = (kd_int & 0x0F) << 4 | t >> 8;
            tx[7] = t & 0xFF;
            motor_can_instance->setDLC(8);
            break;
        }
        case DM_MODE_POS_VEL: {
            memcpy(&tx[0], &p_des, 4);
            memcpy(&tx[4], &v_des, 4);
            motor_can_instance->setDLC(8);
            break;
        }
        case DM_MODE_VEL: {
            memcpy(&tx[0], &v_des, 4);
            motor_can_instance->setDLC(4);
            break;
        }
        case DM_MODE_FORCE_POS: {
            /* 限速与限流做幅值截断后放大为无符号 16 位 */
            float vel = v_des;
            float cur = i_des;
            if (vel < 0.0f) vel = 0.0f;
            if (cur < 0.0f) cur = 0.0f;
            int v_raw = static_cast<int>(vel * DM_FORCE_POS_VEL_SCALE);
            int i_raw = static_cast<int>(cur * DM_FORCE_POS_CUR_SCALE);
            if (v_raw > DM_FORCE_POS_MAX_RAW) v_raw = DM_FORCE_POS_MAX_RAW;
            if (i_raw > DM_FORCE_POS_MAX_RAW) i_raw = DM_FORCE_POS_MAX_RAW;

            memcpy(&tx[0], &p_des, 4);
            tx[4] = v_raw & 0xFF;
            tx[5] = v_raw >> 8;
            tx[6] = i_raw & 0xFF;
            tx[7] = i_raw >> 8;
            motor_can_instance->setDLC(8);
            break;
        }
    }

    if (!motor_can_instance->transmit())
        LOG_ERROR("[DM Motor] Failed to send control frame!");
}

/**
 * @brief 电机控制函数, 遍历所有已使能的电机实例并发送其控制帧
 *
 * @note 需在电机任务中周期调用. 达妙电机反馈为问询式, 每收到一帧控制指令
 *       即回复一帧反馈, 故周期发送控制帧同时也维持了反馈与喂狗.
 */
void DM_Motor::control() {
    for (uint8_t i = 0; i < count_; i++) {
        DM_Motor *motor = instances_[i];
        if (!motor->enabled)
            continue;
        motor->packAndSend();
    }
}

/**
 * @brief 浮点数转无符号定点数(线性映射), 用于打包控制帧
 *
 * @param x     待转换的浮点值
 * @param x_min 映射区间下限
 * @param x_max 映射区间上限
 * @param bits  定点数位数(16 或 12)
 * @return 映射后的定点数, 范围 [0, 2^bits - 1]
 */
int DM_Motor::float_to_uint(float x, const float x_min, const float x_max, const int bits) {
    const float span = x_max - x_min;
    if (x > x_max) x = x_max;
    else if (x < x_min) x = x_min;
    return static_cast<int>((x - x_min) * static_cast<float>((1 << bits) - 1) / span);
}

/**
 * @brief 无符号定点数转浮点数(线性映射), 用于解析反馈帧
 *
 * @param x_int 待转换的定点值
 * @param x_min 映射区间下限
 * @param x_max 映射区间上限
 * @param bits  定点数位数(16 或 12)
 * @return 映射后的浮点值
 */
float DM_Motor::uint_to_float(const int x_int, const float x_min, const float x_max, const int bits) {
    const float span = x_max - x_min;
    return static_cast<float>(x_int) * span / static_cast<float>((1 << bits) - 1) + x_min;
}

namespace {
void controlDebug(USART *usart, uint16_t size) {
    if (usart == nullptr) {
        return;
    }
    (void)USB::transmit(usart->getRxBuff(), size);
}

void debugToMotor(uint16_t len) {
    if (s_debugUsb == nullptr || s_debugUsart == nullptr || len == 0U) {
        return;
    }
    const uint8_t *const buff = s_debugUsb->getBuff();
    uint16_t offset = 0U;
    while (offset < len) {
        const uint16_t remain = static_cast<uint16_t>(len - offset);
        const uint8_t chunk = remain > 255U ? 255U : static_cast<uint8_t>(remain);
        s_debugUsart->send(buff + offset, chunk, USART::TransferMode::DMA);
        offset = static_cast<uint16_t>(offset + chunk);
    }
}
} // namespace
