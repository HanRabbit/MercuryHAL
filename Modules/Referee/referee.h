#pragma once

#include <cstdint>
#include "bsp_usart.h"
#include "daemon.h"
#include "referee_protocol.h"

/**
 * @brief RoboMaster 裁判系统串口通信模块
 *
 * 物理层: 串口, 115200-8N1。帧格式为官方协议(见 referee_protocol.h):
 *   帧头(5B, 含 CRC8) + cmd_id(2B) + 数据段(nB) + 帧尾 CRC16(2B)。
 * 接收: 经 USART 空闲中断 + DMA 收帧, 在回调中校验 CRC 并按 cmd_id 分发到 data_。
 * 发送: 供 RefereeUI 组好的交互帧经此发送(裁判系统上行限速 10Hz)。
 * 掉线检测: 复用 Daemon, 一段时间无有效数据则重启串口接收。
 */
class Referee {
public:
    /* 由 0x0201 机器人状态数据推导出的 ID 信息, 供 UI / 车间通信使用 */
    struct id_t {
        uint8_t robot_color; /* 机器人颜色: 0 红方, 1 蓝方 */
        uint16_t robot_id; /* 本机器人 ID */
        uint16_t client_id; /* 本机器人对应的操作手客户端 ID */
        uint16_t receiver_robot_id; /* 车间通信接收者 ID, 须与本机同色 */
    };

    /* 裁判系统接收数据汇总(按 cmd_id 分类存储) */
    struct data_t {
        id_t id;
        uint16_t cmd_id; /* 最近一次成功解析的命令码 */

        ext_game_state_t game_state; /* 0x0001 比赛状态 */
        ext_game_result_t game_result; /* 0x0002 比赛结果 */
        ext_game_robot_HP_t game_robot_hp; /* 0x0003 机器人血量 */
        ext_event_data_t event_data; /* 0x0101 场地事件 */
        ext_supply_projectile_action_t supply_projectile_action; /* 0x0102 补给站动作 */
        ext_game_robot_state_t game_robot_state; /* 0x0201 机器人状态 */
        ext_power_heat_data_t power_heat_data; /* 0x0202 功率热量 */
        ext_game_robot_pos_t game_robot_pos; /* 0x0203 机器人位置 */
        ext_buff_musk_t buff_musk; /* 0x0204 机器人增益 */
        aerial_robot_energy_t aerial_robot_energy; /* 0x0205 空中机器人能量 */
        ext_robot_hurt_t robot_hurt; /* 0x0206 伤害状态 */
        ext_shoot_data_t shoot_data; /* 0x0207 实时射击 */
        Communicate_ReceiveData_t receive_data; /* 0x0301 车间交互数据 */
    };

    /* 裁判系统配置结构体 */
    struct Config {
        UART_HandleTypeDef *huart = nullptr; /* 裁判系统串口句柄 */
        uint16_t reload_count = 30; /* Daemon 超时计数, 以 Daemon::task 周期为基准(~0.3s) */
        uint16_t init_count = 0; /* Daemon 初始计数 */
    };

    explicit Referee(const Config &config);

    /**
     * @brief 初始化裁判系统串口(启动接收 DMA)并注册掉线检测 Daemon
     */
    void init();

    /**
     * @brief 发送一帧数据到裁判系统, 供 RefereeUI 调用
     * @param data 待发送数据首地址
     * @param len 待发送长度(字节), 裁判交互帧最大约 120 字节
     * @note 裁判系统上行 CMD 数据频率上限 10Hz, 发送后阻塞延时以限速
     */
    void send(const uint8_t *data, uint16_t len) const;

    /**
     * @brief 获取裁判系统接收数据
     */
    [[nodiscard]] const data_t &getData() const { return data_; }

    /**
     * @brief 裁判系统链路是否在线(由 Daemon 超时判定)
     */
    [[nodiscard]] bool isOnline() const;

    /**
     * @brief 获取单例指针, 供 RefereeUI 及数据消费方取用
     */
    static Referee *getInstance() { return instance_; }

    /* ---- 官方 CRC 校验(表驱动), 供解析与 UI 组帧共用 ---- */

    /**
     * @brief 计算 CRC8
     * @param msg 数据指针
     * @param len 参与计算的字节数
     * @param crc8 初始值(整帧计算时传 CRC8_INIT)
     */
    static uint8_t getCrc8(const uint8_t *msg, uint16_t len, uint8_t crc8);

    /**
     * @brief 校验 CRC8, len 含末尾 1 字节校验值
     */
    static bool verifyCrc8(const uint8_t *msg, uint16_t len);

    /**
     * @brief 计算并追加 CRC8 到 msg[len-1]
     */
    static void appendCrc8(uint8_t *msg, uint16_t len);

    /**
     * @brief 计算 CRC16
     * @param msg 数据指针
     * @param len 参与计算的字节数
     * @param crc16 初始值(整帧计算时传 CRC16_INIT)
     */
    static uint16_t getCrc16(const uint8_t *msg, uint32_t len, uint16_t crc16);

    /**
     * @brief 校验 CRC16, len 含末尾 2 字节校验值
     */
    static bool verifyCrc16(const uint8_t *msg, uint32_t len);

    /**
     * @brief 计算并追加 CRC16 到 msg[len-2..len-1](小端)
     */
    static void appendCrc16(uint8_t *msg, uint32_t len);

    static constexpr uint8_t CRC8_INIT = 0xFF;
    static constexpr uint16_t CRC16_INIT = 0xFFFF;

private:
    /**
     * @brief 解析接收缓冲区中的一段字节流, 迭代处理粘连的多帧
     * @param buff 接收缓冲区首地址
     * @param max_len 缓冲区可解析的最大长度(用于越界保护)
     */
    void parseFrame(const uint8_t *buff, uint16_t max_len);

    /**
     * @brief 根据 0x0201 机器人状态数据推导机器人 ID / 客户端 ID
     */
    void updateRobotId();

    /* USART 接收完成回调: 喂狗并解析数据 */
    static void rxCallback(USART *usart, uint16_t size);

    /* Daemon 掉线回调: 重启串口接收 DMA */
    static void lostCallback(void *owner);

    Config config_{};
    USART *usart_ = nullptr;
    Daemon *daemon_ = nullptr;
    data_t data_{};

    static Referee *instance_;
};
