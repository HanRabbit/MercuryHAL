#include "vision.h"

#include <cstring>

#include "bsp_log.h"
#include "bsp_usb.h"

Vision *Vision::instance_ = nullptr;

/* RoboMaster / DJI 裁判系统标准 CRC16 表 (CRC-16/CCITT 反射, 多项式 0x8408) */
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
 * @brief Vision 构造函数, 保存配置并初始化发送帧默认字段
 * @param config Vision 配置结构体, 包含 Daemon 超时与掉线回调
 */
Vision::Vision(const Config &config) {
    instance_ = this;
    config_ = config;

    std::memset(&tx_data_, 0, sizeof(tx_data_));
    std::memset(&tx_packet_, 0, sizeof(tx_packet_));
    std::memset(&rx_data_, 0, sizeof(rx_data_));
    tx_data_.head[0] = FRAME_HEAD0;
    tx_data_.head[1] = FRAME_HEAD1;
    tx_data_.q[0] = 1.0f; /* 默认单位四元数 wxyz */
}

/**
 * @brief 初始化 USB CDC 与在线检测 Daemon
 * @note 仅通过 BSP USB 封装注册收发回调, 不直接操作底层 CDC / USB Device
 */
void Vision::init() {
    if (inited_) {
        return;
    }

    LOG_INFO("[Vision] Initializing USB CDC vision link...");

    rx_buff_ = usb_.init(rxCallback, txCallback);
    if (rx_buff_ == nullptr) {
        LOG_ERROR("[Vision] USB CDC init failed, rx buffer is null");
        return;
    }

    daemon_ = new Daemon({
        .reload_count = config_.reload_count,
        .init_count = config_.init_count,
        .callback = lostCallback,
        .owner = this,
    });

    inited_ = true;
    LOG_INFO("[Vision] Initialized, rx buffer at %p", rx_buff_);
}

/**
 * @brief 整体覆盖发送帧内容 (不含 head / crc)
 * @param data 发送帧数据, head 与 crc16 字段会被忽略并在 send() 中重写
 */
void Vision::setTx(const GimbalToVision &data) {
    tx_data_ = data;
    tx_data_.head[0] = FRAME_HEAD0;
    tx_data_.head[1] = FRAME_HEAD1;
}

/**
 * @brief 按字段设置发送帧内容
 */
void Vision::setTx(const TxMode mode,
                   const float q_wxyz[4],
                   const float yaw,
                   const float yaw_vel,
                   const float pitch,
                   const float pitch_vel,
                   const float bullet_speed,
                   const uint16_t bullet_count) {
    tx_data_.mode = static_cast<uint8_t>(mode);
    if (q_wxyz != nullptr) {
        tx_data_.q[0] = q_wxyz[0];
        tx_data_.q[1] = q_wxyz[1];
        tx_data_.q[2] = q_wxyz[2];
        tx_data_.q[3] = q_wxyz[3];
    }
    tx_data_.yaw = yaw;
    tx_data_.yaw_vel = yaw_vel;
    tx_data_.pitch = pitch;
    tx_data_.pitch_vel = pitch_vel;
    tx_data_.bullet_speed = bullet_speed;
    tx_data_.bullet_count = bullet_count;
}

/**
 * @brief 组装帧头 / CRC 并通过 USB CDC 发送
 * @return true 已提交发送; false 未初始化或上一次发送尚未完成
 */
bool Vision::send() {
    if (!inited_ || rx_buff_ == nullptr) {
        return false;
    }
    if (tx_busy_) {
        return false;
    }

    tx_data_.head[0] = FRAME_HEAD0;
    tx_data_.head[1] = FRAME_HEAD1;
    tx_data_.crc16 = getCrc16(reinterpret_cast<const uint8_t *>(&tx_data_),
                              sizeof(tx_data_) - sizeof(tx_data_.crc16));

    /* 快照到独立发送缓冲, 避免 setTx 在 USB 传输过程中覆盖正在发送的数据 */
    tx_packet_ = tx_data_;

    if (!USB::transmit(reinterpret_cast<uint8_t *>(&tx_packet_), sizeof(tx_packet_))) {
        return false;
    }

    tx_busy_ = true;
    return true;
}

/**
 * @brief 视觉链路是否在线
 */
bool Vision::isOnline() const {
    return daemon_ != nullptr && daemon_->isOnline();
}

/**
 * @brief CRC16 查表计算, len 不含 crc16 字段
 * @param data 数据指针
 * @param len  参与校验的字节数
 * @return CRC16 结果
 */
uint16_t Vision::getCrc16(const uint8_t *data, const uint32_t len) {
    uint16_t crc = CRC16_INIT;
    if (data == nullptr) {
        return crc;
    }

    for (uint32_t i = 0; i < len; ++i) {
        crc = (crc >> 8) ^ CRC16_TABLE[(crc ^ data[i]) & 0xFF];
    }
    return crc;
}

/**
 * @brief CRC16 整帧校验, len 含 crc16 字段; 整帧校验结果应为 0
 * @param data 整帧数据指针
 * @param len  整帧长度 (含 crc16)
 * @return true 校验通过
 */
bool Vision::checkCrc16(const uint8_t *data, const uint32_t len) {
    if (data == nullptr || len < 2) {
        return false;
    }
    return getCrc16(data, len) == 0;
}

/**
 * @brief USB 接收回调入口, 将本次 CDC 数据流喂入帧解析状态机
 * @param len 本次接收字节数
 */
void Vision::handleRx(const uint16_t len) {
    if (rx_buff_ == nullptr || len == 0) {
        return;
    }

    for (uint16_t i = 0; i < len; ++i) {
        feedRxByte(rx_buff_[i]);
    }
}

/**
 * @brief 逐字节推入接收流, 查找帧头并尝试定长解析
 * @param byte 新接收字节
 */
void Vision::feedRxByte(const uint8_t byte) {
    /* 流缓冲区将满时, 丢弃最旧字节, 保留尾部以继续同步帧头 */
    if (rx_stream_len_ >= RX_STREAM_SIZE) {
        std::memmove(rx_stream_, rx_stream_ + 1, RX_STREAM_SIZE - 1);
        rx_stream_len_ = RX_STREAM_SIZE - 1;
    }

    rx_stream_[rx_stream_len_++] = byte;

    /* 同步到 'S''P' 帧头 */
    while (rx_stream_len_ >= 2) {
        if (rx_stream_[0] == FRAME_HEAD0 && rx_stream_[1] == FRAME_HEAD1) {
            break;
        }
        std::memmove(rx_stream_, rx_stream_ + 1, rx_stream_len_ - 1);
        --rx_stream_len_;
    }

    while (tryParseRxFrame()) {
        /* 连续粘包时继续解析 */
    }
}

/**
 * @brief 在已对齐帧头的接收流中尝试解析一帧 VisionToGimbal
 * @return true 成功解析并消费一帧, 可继续尝试下一帧
 */
bool Vision::tryParseRxFrame() {
    constexpr uint16_t frame_len = sizeof(VisionToGimbal);

    if (rx_stream_len_ < 2) {
        return false;
    }
    if (rx_stream_[0] != FRAME_HEAD0 || rx_stream_[1] != FRAME_HEAD1) {
        return false;
    }
    if (rx_stream_len_ < frame_len) {
        return false;
    }

    VisionToGimbal frame{};
    std::memcpy(&frame, rx_stream_, frame_len);

    if (!checkCrc16(reinterpret_cast<const uint8_t *>(&frame), frame_len)) {
        /* CRC 失败: 丢弃当前帧头首字节, 重新同步 */
        std::memmove(rx_stream_, rx_stream_ + 1, rx_stream_len_ - 1);
        --rx_stream_len_;
        return true;
    }

    rx_data_ = frame;
    if (daemon_ != nullptr) {
        daemon_->reload();
    }

    /* 消费已解析帧 */
    const uint16_t remain = static_cast<uint16_t>(rx_stream_len_ - frame_len);
    if (remain > 0) {
        std::memmove(rx_stream_, rx_stream_ + frame_len, remain);
    }
    rx_stream_len_ = remain;
    return rx_stream_len_ >= frame_len;
}

/**
 * @brief USB CDC 接收完成静态回调
 * @param len 本次接收长度
 */
void Vision::rxCallback(const uint16_t len) {
    if (instance_ != nullptr) {
        instance_->handleRx(len);
    }
}

/**
 * @brief USB CDC 发送完成静态回调
 * @param len 本次发送长度 (未使用)
 */
void Vision::txCallback(const uint16_t /*len*/) {
    if (instance_ != nullptr) {
        instance_->tx_busy_ = false;
    }
}

/**
 * @brief 视觉链路超时掉线回调, 清空控制指令并转发用户回调
 * @param owner Vision 实例指针
 */
void Vision::lostCallback(void *owner) {
    auto *self = static_cast<Vision *>(owner);
    if (self == nullptr) {
        return;
    }

    /* 掉线后清零控制指令, 避免沿用过期开火/角度目标 */
    std::memset(&self->rx_data_, 0, sizeof(self->rx_data_));
    self->rx_data_.head[0] = FRAME_HEAD0;
    self->rx_data_.head[1] = FRAME_HEAD1;

    LOG_WARNING("[Vision] Vision link offline");

    if (self->config_.lost_callback != nullptr) {
        self->config_.lost_callback(self->config_.owner);
    }
}
