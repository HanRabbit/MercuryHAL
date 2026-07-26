#pragma once

#include <cstdint>
#include "bsp_usb.h"
#include "daemon.h"

/**
 * @brief 下位机与视觉端的 USB CDC 通信模块
 *
 * 物理层: USB CDC 虚拟串口, 8N1、无流控; CDC 下波特率被忽略。
 * 帧格式: 定长、__attribute__((packed)) 紧凑无填充、小端。
 * 帧头固定 'S''P' (0x53 0x50), 帧尾 CRC16(CRC-16/CCITT 反射, 多项式 0x8408, 与 RM 裁判系统一致)。
 * 双向独立、各自定时发送(非请求-应答)。
 *
 * 注意两个方向的 mode 语义不同:
 * - TX (GimbalToVision): 当前档位 (空闲/自瞄/小符/大符)
 * - RX (VisionToGimbal): 控制/开火指令
 */
class Vision {
public:
    /* 下位机 → 视觉: 当前档位 */
    enum class TxMode : uint8_t {
        Idle = 0,       /* 空闲 */
        AutoAim = 1,    /* 自瞄 */
        SmallRune = 2,  /* 小符 */
        LargeRune = 3,  /* 大符 */
    };

    /* 视觉 → 下位机: 控制/开火指令 */
    enum class RxMode : uint8_t {
        None = 0,           /* 不控制 */
        ControlOnly = 1,    /* 控制云台但不开火 */
        ControlAndFire = 2, /* 控制云台且开火 */
    };

    /**
     * @brief 下位机 → 视觉 定长帧 (43 字节)
     */
    struct __attribute__((packed)) GimbalToVision {
        uint8_t head[2];      /* 固定 'S','P' */
        uint8_t mode;         /* TxMode */
        float q[4];           /* 姿态四元数, wxyz 顺序 */
        float yaw;            /* 云台 yaw (rad) */
        float yaw_vel;        /* yaw 角速度 */
        float pitch;          /* 云台 pitch (rad) */
        float pitch_vel;      /* pitch 角速度 */
        float bullet_speed;   /* 弹速 (m/s) */
        uint16_t bullet_count;/* 子弹累计发射次数 */
        uint16_t crc16;       /* CRC16 校验 */
    };
    static_assert(sizeof(GimbalToVision) == 43, "GimbalToVision size must be 43");

    /**
     * @brief 视觉 → 下位机 定长帧 (29 字节)
     */
    struct __attribute__((packed)) VisionToGimbal {
        uint8_t head[2];      /* 固定 'S','P' */
        uint8_t mode;         /* RxMode */
        float yaw;            /* 目标 yaw (rad) */
        float yaw_vel;        /* yaw 期望角速度 */
        float yaw_acc;        /* yaw 期望角加速度 */
        float pitch;          /* 目标 pitch (rad) */
        float pitch_vel;      /* pitch 期望角速度 */
        float pitch_acc;      /* pitch 期望角加速度 */
        uint16_t crc16;       /* CRC16 校验 */
    };
    static_assert(sizeof(VisionToGimbal) == 29, "VisionToGimbal size must be 29");

    struct Config {
        /* Daemon 超时计数; 以调用 reload 的周期为基准, 0 表示使用默认值 100 */
        uint16_t reload_count = 100;
        uint16_t init_count = 0;
        Daemon::Callback lost_callback = nullptr;
        void *owner = nullptr;
    };

    explicit Vision(const Config &config);

    /**
     * @brief 初始化 USB CDC 并注册收发回调, 同时创建在线检测 Daemon
     */
    void init();

    /**
     * @brief 填充发送帧字段 (不含 head / crc, 由 send() 统一写入)
     */
    void setTx(const GimbalToVision &data);

    /**
     * @brief 按字段设置发送帧内容 (不含 head / crc)
     */
    void setTx(TxMode mode,
               const float q_wxyz[4],
               float yaw,
               float yaw_vel,
               float pitch,
               float pitch_vel,
               float bullet_speed,
               uint16_t bullet_count);

    /**
     * @brief 组装帧头与 CRC 并通过 USB CDC 发送到视觉端
     * @return true 已提交发送, false 发送繁忙或未初始化
     */
    bool send();

    /**
     * @brief 获取最近一次通过 CRC 校验的视觉指令帧
     */
    [[nodiscard]] const VisionToGimbal &getRx() const { return rx_data_; }

    /**
     * @brief 获取待发送/最近一次组装的下位机状态帧
     */
    [[nodiscard]] const GimbalToVision &getTx() const { return tx_data_; }

    /**
     * @brief 视觉链路是否在线 (由 Daemon 超时判定)
     */
    [[nodiscard]] bool isOnline() const;

    /**
     * @brief CRC16 查表计算, len 不含 crc16 字段
     */
    static uint16_t getCrc16(const uint8_t *data, uint32_t len);

    /**
     * @brief CRC16 整帧校验, len 含 crc16 字段; 通过时返回 true
     */
    static bool checkCrc16(const uint8_t *data, uint32_t len);

private:
    static constexpr uint8_t FRAME_HEAD0 = 'S';
    static constexpr uint8_t FRAME_HEAD1 = 'P';
    static constexpr uint16_t CRC16_INIT = 0xFFFF;
    static constexpr uint16_t RX_STREAM_SIZE = 128;

    void handleRx(uint16_t len);
    void feedRxByte(uint8_t byte);
    bool tryParseRxFrame();

    static void rxCallback(uint16_t len);
    static void txCallback(uint16_t len);
    static void lostCallback(void *owner);

    Config config_{};
    USB usb_{};
    uint8_t *rx_buff_ = nullptr;
    Daemon *daemon_ = nullptr;

    GimbalToVision tx_data_{};
    GimbalToVision tx_packet_{}; /* 提交给 USB 的发送快照, 发送完成前不可覆盖 */
    VisionToGimbal rx_data_{};

    uint8_t rx_stream_[RX_STREAM_SIZE]{};
    uint16_t rx_stream_len_ = 0;

    volatile bool tx_busy_ = false;
    bool inited_ = false;

    static Vision *instance_;
};
