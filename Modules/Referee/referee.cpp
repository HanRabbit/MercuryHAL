#include "referee.h"

#include <cstring>

#include "bsp_log.h"
#include "cmsis_os2.h"

Referee *Referee::instance_ = nullptr;

/* 裁判系统串口接收缓冲区大小(官方帧最大长度不超过此值) */
static constexpr uint16_t RX_BUFFER_SIZE = 255;

/* RoboMaster 裁判系统官方 CRC8 表 */
static constexpr uint8_t CRC8_TABLE[256] = {
    0x00, 0x5E, 0xBC, 0xE2, 0x61, 0x3F, 0xDD, 0x83, 0xC2, 0x9C, 0x7E, 0x20, 0xA3, 0xFD, 0x1F, 0x41,
    0x9D, 0xC3, 0x21, 0x7F, 0xFC, 0xA2, 0x40, 0x1E, 0x5F, 0x01, 0xE3, 0xBD, 0x3E, 0x60, 0x82, 0xDC,
    0x23, 0x7D, 0x9F, 0xC1, 0x42, 0x1C, 0xFE, 0xA0, 0xE1, 0xBF, 0x5D, 0x03, 0x80, 0xDE, 0x3C, 0x62,
    0xBE, 0xE0, 0x02, 0x5C, 0xDF, 0x81, 0x63, 0x3D, 0x7C, 0x22, 0xC0, 0x9E, 0x1D, 0x43, 0xA1, 0xFF,
    0x46, 0x18, 0xFA, 0xA4, 0x27, 0x79, 0x9B, 0xC5, 0x84, 0xDA, 0x38, 0x66, 0xE5, 0xBB, 0x59, 0x07,
    0xDB, 0x85, 0x67, 0x39, 0xBA, 0xE4, 0x06, 0x58, 0x19, 0x47, 0xA5, 0xFB, 0x78, 0x26, 0xC4, 0x9A,
    0x65, 0x3B, 0xD9, 0x87, 0x04, 0x5A, 0xB8, 0xE6, 0xA7, 0xF9, 0x1B, 0x45, 0xC6, 0x98, 0x7A, 0x24,
    0xF8, 0xA6, 0x44, 0x1A, 0x99, 0xC7, 0x25, 0x7B, 0x3A, 0x64, 0x86, 0xD8, 0x5B, 0x05, 0xE7, 0xB9,
    0x8C, 0xD2, 0x30, 0x6E, 0xED, 0xB3, 0x51, 0x0F, 0x4E, 0x10, 0xF2, 0xAC, 0x2F, 0x71, 0x93, 0xCD,
    0x11, 0x4F, 0xAD, 0xF3, 0x70, 0x2E, 0xCC, 0x92, 0xD3, 0x8D, 0x6F, 0x31, 0xB2, 0xEC, 0x0E, 0x50,
    0xAF, 0xF1, 0x13, 0x4D, 0xCE, 0x90, 0x72, 0x2C, 0x6D, 0x33, 0xD1, 0x8F, 0x0C, 0x52, 0xB0, 0xEE,
    0x32, 0x6C, 0x8E, 0xD0, 0x53, 0x0D, 0xEF, 0xB1, 0xF0, 0xAE, 0x4C, 0x12, 0x91, 0xCF, 0x2D, 0x73,
    0xCA, 0x94, 0x76, 0x28, 0xAB, 0xF5, 0x17, 0x49, 0x08, 0x56, 0xB4, 0xEA, 0x69, 0x37, 0xD5, 0x8B,
    0x57, 0x09, 0xEB, 0xB5, 0x36, 0x68, 0x8A, 0xD4, 0x95, 0xCB, 0x29, 0x77, 0xF4, 0xAA, 0x48, 0x16,
    0xE9, 0xB7, 0x55, 0x0B, 0x88, 0xD6, 0x34, 0x6A, 0x2B, 0x75, 0x97, 0xC9, 0x4A, 0x14, 0xF6, 0xA8,
    0x74, 0x2A, 0xC8, 0x96, 0x15, 0x4B, 0xA9, 0xF7, 0xB6, 0xE8, 0x0A, 0x54, 0xD7, 0x89, 0x6B, 0x35,
};

/* RoboMaster 裁判系统官方 CRC16 表 (CRC-16/CCITT 反射, 多项式 0x8408) */
static constexpr uint16_t CRC16_TABLE[256] = {
    0x0000, 0x1189, 0x2312, 0x329B, 0x4624, 0x57AD, 0x6536, 0x74BF,
    0x8C48, 0x9DC1, 0xAF5A, 0xBED3, 0xCA6C, 0xDBE5, 0xE97E, 0xF8F7,
    0x1081, 0x0108, 0x3393, 0x221A, 0x56A5, 0x472C, 0x75B7, 0x643E,
    0x9CC9, 0x8D40, 0xBFDB, 0xAE52, 0xDAED, 0xCB64, 0xF9FF, 0xE876,
    0x2102, 0x308B, 0x0210, 0x1399, 0x6726, 0x76AF, 0x4434, 0x55BD,
    0xAD4A, 0xBCC3, 0x8E58, 0x9FD1, 0xEB6E, 0xFAE7, 0xC87C, 0xD9F5,
    0x3183, 0x200A, 0x1291, 0x0318, 0x77A7, 0x662E, 0x54B5, 0x453C,
    0xBDCB, 0xAC42, 0x9ED9, 0x8F50, 0xFBEF, 0xEA66, 0xD8FD, 0xC974,
    0x4204, 0x538D, 0x6116, 0x709F, 0x0420, 0x15A9, 0x2732, 0x36BB,
    0xCE4C, 0xDFC5, 0xED5E, 0xFCD7, 0x8868, 0x99E1, 0xAB7A, 0xBAF3,
    0x5285, 0x430C, 0x7197, 0x601E, 0x14A1, 0x0528, 0x37B3, 0x263A,
    0xDECD, 0xCF44, 0xFDDF, 0xEC56, 0x98E9, 0x8960, 0xBBFB, 0xAA72,
    0x6306, 0x728F, 0x4014, 0x519D, 0x2522, 0x34AB, 0x0630, 0x17B9,
    0xEF4E, 0xFEC7, 0xCC5C, 0xDDD5, 0xA96A, 0xB8E3, 0x8A78, 0x9BF1,
    0x7387, 0x620E, 0x5095, 0x411C, 0x35A3, 0x242A, 0x16B1, 0x0738,
    0xFFCF, 0xEE46, 0xDCDD, 0xCD54, 0xB9EB, 0xA862, 0x9AF9, 0x8B70,
    0x8408, 0x9581, 0xA71A, 0xB693, 0xC22C, 0xD3A5, 0xE13E, 0xF0B7,
    0x0840, 0x19C9, 0x2B52, 0x3ADB, 0x4E64, 0x5FED, 0x6D76, 0x7CFF,
    0x9489, 0x8500, 0xB79B, 0xA612, 0xD2AD, 0xC324, 0xF1BF, 0xE036,
    0x18C1, 0x0948, 0x3BD3, 0x2A5A, 0x5EE5, 0x4F6C, 0x7DF7, 0x6C7E,
    0xA50A, 0xB483, 0x8618, 0x9791, 0xE32E, 0xF2A7, 0xC03C, 0xD1B5,
    0x2942, 0x38CB, 0x0A50, 0x1BD9, 0x6F66, 0x7EEF, 0x4C74, 0x5DFD,
    0xB58B, 0xA402, 0x9699, 0x8710, 0xF3AF, 0xE226, 0xD0BD, 0xC134,
    0x39C3, 0x284A, 0x1AD1, 0x0B58, 0x7FE7, 0x6E6E, 0x5CF5, 0x4D7C,
    0xC60C, 0xD785, 0xE51E, 0xF497, 0x8028, 0x91A1, 0xA33A, 0xB2B3,
    0x4A44, 0x5BCD, 0x6956, 0x78DF, 0x0C60, 0x1DE9, 0x2F72, 0x3EFB,
    0xD68D, 0xC704, 0xF59F, 0xE416, 0x90A9, 0x8120, 0xB3BB, 0xA232,
    0x5AC5, 0x4B4C, 0x79D7, 0x685E, 0x1CE1, 0x0D68, 0x3FF3, 0x2E7A,
    0xE70E, 0xF687, 0xC41C, 0xD595, 0xA12A, 0xB0A3, 0x8238, 0x93B1,
    0x6B46, 0x7ACF, 0x4854, 0x59DD, 0x2D62, 0x3CEB, 0x0E70, 0x1FF9,
    0xF78F, 0xE606, 0xD49D, 0xC514, 0xB1AB, 0xA022, 0x92B9, 0x8330,
    0x7BC7, 0x6A4E, 0x58D5, 0x495C, 0x3DE3, 0x2C6A, 0x1EF1, 0x0F78,
};

/**
 * @brief Referee 构造函数, 保存配置并注册单例指针
 * @param config Referee 配置结构体, 包含串口句柄与 Daemon 超时参数
 */
Referee::Referee(const Config &config) {
    instance_ = this;
    config_ = config;
}

/**
 * @brief 初始化裁判系统串口(注册 USART 实例并启动空闲中断 + DMA 接收)与掉线检测 Daemon
 */
void Referee::init() {
    if (config_.huart == nullptr) {
        LOG_ERROR("[Referee] Invalid config: huart is null");
        return;
    }

    LOG_INFO("[Referee] Initializing referee system UART...");

    usart_ = new USART(config_.huart, RX_BUFFER_SIZE, rxCallback);
    usart_->init();

    daemon_ = new Daemon({
        .reload_count = config_.reload_count,
        .init_count = config_.init_count,
        .callback = lostCallback,
        .owner = this,
    });

    LOG_INFO("[Referee] Initialized.");
}

/**
 * @brief 发送一帧数据到裁判系统
 * @param data 待发送数据首地址
 * @param len 待发送长度(字节)
 * @note 裁判系统上行 CMD 数据频率上限 10Hz, 发送后阻塞延时限速(与旧框架 RefereeSend 语义一致),
 *       因此只能在任务上下文中调用, 不可在中断中调用
 */
void Referee::send(const uint8_t *data, const uint16_t len) const {
    if (usart_ == nullptr || data == nullptr || len == 0 || len > UINT8_MAX) {
        return;
    }

    usart_->send(data, static_cast<uint8_t>(len), USART::TransferMode::DMA);
    osDelay(115);
}

/**
 * @brief 裁判系统链路是否在线
 */
bool Referee::isOnline() const {
    return daemon_ != nullptr && daemon_->isOnline();
}

/**
 * @brief 计算 CRC8(查表)
 */
uint8_t Referee::getCrc8(const uint8_t *msg, uint16_t len, uint8_t crc8) {
    if (msg == nullptr) {
        return CRC8_INIT;
    }

    while (len--) {
        const uint8_t index = crc8 ^ *msg++;
        crc8 = CRC8_TABLE[index];
    }
    return crc8;
}

/**
 * @brief 校验 CRC8, len 含末尾 1 字节校验值
 */
bool Referee::verifyCrc8(const uint8_t *msg, const uint16_t len) {
    if (msg == nullptr || len <= 2) {
        return false;
    }

    return getCrc8(msg, len - 1, CRC8_INIT) == msg[len - 1];
}

/**
 * @brief 计算并追加 CRC8 到 msg[len-1]
 */
void Referee::appendCrc8(uint8_t *msg, const uint16_t len) {
    if (msg == nullptr || len <= 2) {
        return;
    }

    msg[len - 1] = getCrc8(msg, len - 1, CRC8_INIT);
}

/**
 * @brief 计算 CRC16(查表)
 */
uint16_t Referee::getCrc16(const uint8_t *msg, uint32_t len, uint16_t crc16) {
    if (msg == nullptr) {
        return CRC16_INIT;
    }

    while (len--) {
        const uint8_t data = *msg++;
        crc16 = (crc16 >> 8) ^ CRC16_TABLE[(crc16 ^ data) & 0xFF];
    }
    return crc16;
}

/**
 * @brief 校验 CRC16, len 含末尾 2 字节校验值(小端)
 */
bool Referee::verifyCrc16(const uint8_t *msg, const uint32_t len) {
    if (msg == nullptr || len <= 2) {
        return false;
    }

    const uint16_t expected = getCrc16(msg, len - 2, CRC16_INIT);
    return (expected & 0xFF) == msg[len - 2] && (expected >> 8 & 0xFF) == msg[len - 1];
}

/**
 * @brief 计算并追加 CRC16 到 msg[len-2..len-1](小端)
 */
void Referee::appendCrc16(uint8_t *msg, const uint32_t len) {
    if (msg == nullptr || len <= 2) {
        return;
    }

    const uint16_t crc16 = getCrc16(msg, len - 2, CRC16_INIT);
    msg[len - 2] = static_cast<uint8_t>(crc16 & 0xFF);
    msg[len - 1] = static_cast<uint8_t>(crc16 >> 8 & 0xFF);
}

/**
 * @brief 解析接收缓冲区中的字节流, 迭代处理一次接收中粘连的多帧
 *        单帧格式: 帧头(SOF 0xA5 + 数据长 2B + 包序号 1B + CRC8) + cmd_id(2B) + 数据段 + CRC16(2B)。
 *        帧头 CRC8 与整帧 CRC16 均校验通过后才按 cmd_id 拷贝数据, 并对越界读取做保护。
 * @param buff 接收缓冲区首地址
 * @param max_len 缓冲区可解析的最大长度
 */
void Referee::parseFrame(const uint8_t *buff, const uint16_t max_len) {
    if (buff == nullptr) {
        return;
    }

    /* 一帧最小长度: 帧头 + cmd_id + CRC16(数据段为空) */
    constexpr uint16_t min_frame_len = LEN_HEADER + LEN_CMDID + LEN_TAIL;

    uint16_t offset = 0;
    while (offset + min_frame_len <= max_len) {
        const uint8_t *frame = buff + offset;

        /* 帧头起始字节与 CRC8 校验, 失败则认为后续无有效帧 */
        if (frame[SOF] != REFEREE_SOF) {
            break;
        }
        if (!verifyCrc8(frame, LEN_HEADER)) {
            break;
        }

        /* 从帧头取数据段长度并做越界保护, 再做整帧 CRC16 校验 */
        const auto data_len = static_cast<uint16_t>(frame[DATA_LENGTH] | frame[DATA_LENGTH + 1] << 8);
        const auto frame_len = static_cast<uint16_t>(min_frame_len + data_len);
        if (offset + frame_len > max_len) {
            break;
        }
        if (!verifyCrc16(frame, frame_len)) {
            break;
        }

        const auto cmd_id = static_cast<uint16_t>(frame[CMD_ID_Offset] | frame[CMD_ID_Offset + 1] << 8);
        const uint8_t *data = frame + DATA_Offset;

        /* 按命令码将数据段拷贝到对应结构体(拷贝长度为协议规定长度) */
        data_.cmd_id = cmd_id;
        switch (cmd_id) {
            case ID_game_state: /* 0x0001 */
                memcpy(&data_.game_state, data, LEN_game_state);
                break;
            case ID_game_result: /* 0x0002 */
                memcpy(&data_.game_result, data, LEN_game_result);
                break;
            case ID_game_robot_survivors: /* 0x0003 */
                memcpy(&data_.game_robot_hp, data, LEN_game_robot_HP);
                break;
            case ID_event_data: /* 0x0101 */
                memcpy(&data_.event_data, data, LEN_event_data);
                break;
            case ID_supply_projectile_action: /* 0x0102 */
                memcpy(&data_.supply_projectile_action, data, LEN_supply_projectile_action);
                break;
            case ID_game_robot_state: /* 0x0201 */
                memcpy(&data_.game_robot_state, data, LEN_game_robot_state);
                updateRobotId();
                break;
            case ID_power_heat_data: /* 0x0202 */
                memcpy(&data_.power_heat_data, data, LEN_power_heat_data);
                break;
            case ID_game_robot_pos: /* 0x0203 */
                memcpy(&data_.game_robot_pos, data, LEN_game_robot_pos);
                break;
            case ID_buff_musk: /* 0x0204 */
                memcpy(&data_.buff_musk, data, LEN_buff_musk);
                break;
            case ID_aerial_robot_energy: /* 0x0205 */
                memcpy(&data_.aerial_robot_energy, data, LEN_aerial_robot_energy);
                break;
            case ID_robot_hurt: /* 0x0206 */
                memcpy(&data_.robot_hurt, data, LEN_robot_hurt);
                break;
            case ID_shoot_data: /* 0x0207 */
                memcpy(&data_.shoot_data, data, LEN_shoot_data);
                break;
            case ID_student_interactive: /* 0x0301 */
                memcpy(&data_.receive_data, data, LEN_receive_data);
                break;
            default:
                break;
        }

        /* 成功解析一帧才喂狗, 避免噪声数据维持在线状态 */
        if (daemon_ != nullptr) {
            daemon_->reload();
        }

        offset += frame_len;
    }
}

/**
 * @brief 根据 0x0201 机器人状态数据推导机器人颜色 / 客户端 ID
 *        robot_id 1~7 为红方, 大于 7(101~107) 为蓝方, 客户端 ID = 0x0100 + robot_id
 */
void Referee::updateRobotId() {
    data_.id.robot_color = data_.game_robot_state.robot_id > 7 ? Robot_Blue : Robot_Red;
    data_.id.robot_id = data_.game_robot_state.robot_id;
    data_.id.client_id = 0x0100 + data_.id.robot_id;
    data_.id.receiver_robot_id = 0;
}

/**
 * @brief USART 接收完成静态回调, 解析本次收到的裁判系统数据
 * @param usart 触发回调的 USART 实例指针
 * @param size 本次接收的数据长度
 */
void Referee::rxCallback(USART *usart, uint16_t size) {
    if (instance_ == nullptr || usart == nullptr) {
        return;
    }

    instance_->parseFrame(usart->getRxBuff(), size);
}

/**
 * @brief 裁判系统掉线回调, 重启串口接收 DMA 以尝试恢复
 * @param owner Referee 实例指针
 */
void Referee::lostCallback(void *owner) {
    auto *self = static_cast<Referee *>(owner);
    if (self == nullptr || self->usart_ == nullptr) {
        return;
    }

    self->usart_->init();
    LOG_WARNING("[Referee] Referee system offline, restarting UART RX");
}
