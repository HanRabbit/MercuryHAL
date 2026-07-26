#include "bsp_can.h"
#include "bsp_log.h"
#include "bsp_dwt.h"

#include <cstring>

/* FDCAN 实例静态数组与计数（避免在 early runtime 使用堆） */
FDCAN *FDCAN::canInstances[FDCAN_MAX_DEVICE];
uint8_t FDCAN::canInstanceCount = 0;

/**
 * @brief FDCAN 构造函数
 * @param config FDCAN 配置结构体
 */
FDCAN::FDCAN(const Config &config) {
    if (canInstanceCount >= FDCAN_MAX_DEVICE) {
        LOG_ERROR("[FDCAN] Instance count exceeds maximum limit!");
        return;
    }

    fdcan_handle = config.hfdcan;
    tx_id = config.tx_id;
    rx_id = config.rx_id;
    callback = config.callback;
    owner = config.owner;
    tx_header.Identifier = tx_id;
    tx_header.IdType = FDCAN_STANDARD_ID;
    tx_header.TxFrameType = FDCAN_DATA_FRAME;
    tx_header.DataLength = FDCAN_DLC_BYTES_8;
    tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    tx_header.BitRateSwitch = FDCAN_BRS_OFF;
    tx_header.FDFormat = FDCAN_CLASSIC_CAN;
    tx_header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    tx_header.MessageMarker = 0;

    canInstances[canInstanceCount++] = this;
}

/**
 * @brief Start every unique FDCAN handle referenced by registered instances.
 * @note  Construct all FDCAN objects first, then call FDCAN::init() once.
 */
void FDCAN::init() {
    FDCAN_HandleTypeDef *unique[FDCAN_MAX_DEVICE] = {};
    uint8_t uniqueCount = 0U;

    for (uint8_t i = 0U; i < canInstanceCount; ++i) {
        FDCAN_HandleTypeDef *const h = canInstances[i]->fdcan_handle;
        if (h == nullptr) {
            continue;
        }
        bool seen = false;
        for (uint8_t j = 0U; j < uniqueCount; ++j) {
            if (unique[j] == h) {
                seen = true;
                break;
            }
        }
        if (seen) {
            continue;
        }
        unique[uniqueCount++] = h;
    }

    for (uint8_t i = 0U; i < uniqueCount; ++i) {
        FDCAN_HandleTypeDef *const h = unique[i];
        addFilter(h);
        if (HAL_FDCAN_Start(h) != HAL_OK) {
            LOG_ERROR("[FDCAN] Start failed, err=0x%lx",
                      static_cast<unsigned long>(h->ErrorCode));
            continue;
        }
        (void)HAL_FDCAN_ActivateNotification(
            h,
            FDCAN_IT_RX_FIFO0_NEW_MESSAGE | FDCAN_IT_RX_FIFO1_NEW_MESSAGE,
            0U);
    }
}

/**
 * @brief 设置 FDCAN 数据长度代码（DLC），限制最大长度为 8 字节
 * @param len 待设置的数据长度
 */
void FDCAN::setDLC(const uint8_t len) {
    if (len > 8) {
        LOG_ERROR("[FDCAN] Data length exceeds maximum of 8 bytes!");
        return;
    }
    tx_header.DataLength = static_cast<uint32_t>(len);
}

/**
 * @brief FDCAN 数据发送函数，将数据添加到发送队列
 * @param timeout 发送超时时间，单位为秒
 * @return 发送是否成功
 */
bool FDCAN::transmit(const float timeout) const {
    static uint32_t busy_cnt;
    static volatile float wait_time __attribute__((unused));
    const float dwt_start = DWT_Time::getTimeline_ms();

    /* 等待 Tx FIFO 有空位 */
    while (HAL_FDCAN_GetTxFifoFreeLevel(fdcan_handle) == 0) {
        if ((DWT_Time::getTimeline_ms() - dwt_start) > timeout) {
            busy_cnt++;
            LOG_ERROR("[bsp:fdcan] FDCANTransmit timeout, busy_cnt = %lu", busy_cnt);
            return false;
        }
    }

    wait_time = DWT_Time::getTimeline_ms() - dwt_start;

    /* 发送报文 */
    if (HAL_FDCAN_AddMessageToTxFifoQ(fdcan_handle, &tx_header, tx_buff) != HAL_OK) {
        FDCAN_ProtocolStatusTypeDef ps;
        HAL_FDCAN_GetProtocolStatus(fdcan_handle, &ps);
        LOG_ERROR("PS: LEC=%ld BO=%d EP=%d", ps.LastErrorCode, ps.BusOff, ps.ErrorPassive);
        return false;
    }
    return true;
}

/**
 * @brief FDCAN 添加过滤器函数，配置接收过滤器以匹配指定 ID 的消息
 * @param hfdcan FDCAN 句柄
 */
void FDCAN::addFilter(FDCAN_HandleTypeDef *hfdcan) {
    FDCAN_FilterTypeDef sFilter;

    sFilter.IdType = FDCAN_STANDARD_ID;
    sFilter.FilterIndex = 0;
    sFilter.FilterType = FDCAN_FILTER_MASK;
    sFilter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;

    sFilter.FilterID1 = 0x000;
    sFilter.FilterID2 = 0x000;

    HAL_FDCAN_ConfigFilter(hfdcan, &sFilter);
}

extern "C"
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs) {
    if (RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE)
        FDCAN::fifoHandler(hfdcan, FDCAN_RX_FIFO0);
}

extern "C"
void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo1ITs) {
    if (RxFifo1ITs & FDCAN_IT_RX_FIFO1_NEW_MESSAGE)
        FDCAN::fifoHandler(hfdcan, FDCAN_RX_FIFO1);
}

/**
 * @brief FDCAN 接收中断处理函数，从接收 FIFO 中读取消息并分发给对应实例的回调函数
 * @param hfdcan 触发中断的 FDCAN 句柄指针
 * @param fifo 触发中断的 FIFO 号（FDCAN_RX_FIFO0 或 FDCAN_RX_FIFO1）
 */
void FDCAN::fifoHandler(FDCAN_HandleTypeDef *hfdcan, uint32_t fifo) {
    FDCAN_RxHeaderTypeDef rxHeader;
    uint8_t data[8];

    /* 轮询读取 FIFO 中的消息，直到 FIFO 为空 */
    while (HAL_FDCAN_GetRxFifoFillLevel(hfdcan, fifo) > 0) {
        HAL_FDCAN_GetRxMessage(hfdcan, fifo, &rxHeader, data);

        for (uint8_t i = 0; i < canInstanceCount; i++) {
            FDCAN *_fdcan = canInstances[i];
            if (_fdcan->fdcan_handle == hfdcan &&
                _fdcan->rx_id == rxHeader.Identifier) {
                memcpy(_fdcan->rx_buff, data, 8);
                if (_fdcan->callback)
                    _fdcan->callback(_fdcan);
                return;
            }
        }
    }
}
