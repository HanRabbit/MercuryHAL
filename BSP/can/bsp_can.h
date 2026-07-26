#pragma once

#include "main.h"
#include <cstdint>

/* FDCAN 设备最大注册数量 */
#define FDCAN_MAX_DEVICE 16

class FDCAN {
public:
    using Callback_t = void (*)(FDCAN *can);

    /* FDCAN 配置结构体 */
    struct Config {
        FDCAN_HandleTypeDef *hfdcan;
        uint16_t tx_id;
        uint16_t rx_id;
        Callback_t callback;
        void *owner;
    };

    explicit FDCAN(const Config &config);

    static void init();

    [[nodiscard]] bool transmit(float timeout = 1.0f) const;

    void setDLC(uint8_t len);

    uint16_t tx_id = 0;
    uint16_t rx_id = 0;

    uint8_t *getTxBuff() { return tx_buff; }
    uint8_t *getRxBuff() { return rx_buff; }
    [[nodiscard]] uint8_t getRxLen() const { return rx_len; }
    [[nodiscard]] uint16_t getTxId() const { return tx_id; }
    [[nodiscard]] uint16_t getRxId() const { return rx_id; }
    [[nodiscard]] void *getOwner() const { return owner; }

    [[nodiscard]] FDCAN_HandleTypeDef *getHandle() const {
        return fdcan_handle;
    }

    /* HAL 回调分发入口 */
    static void fifoHandler(FDCAN_HandleTypeDef *hfdcan, uint32_t fifo);

private:
    FDCAN_HandleTypeDef *fdcan_handle = {};
    FDCAN_TxHeaderTypeDef tx_header = {};
    uint32_t tx_mailbox = 0;

    Callback_t callback = nullptr;

    uint8_t tx_buff[8] = {};
    uint8_t rx_buff[8] = {};
    uint8_t rx_len = 0;

    void *owner = nullptr;

    static FDCAN *canInstances[FDCAN_MAX_DEVICE];
    static uint8_t canInstanceCount;

    static void addFilter(FDCAN_HandleTypeDef *hfdcan);
};
