#pragma once

#include "main.h"

class SPI {
public:
    explicit SPI(SPI_HandleTypeDef *hspi);

    HAL_StatusTypeDef transmit(
        const uint8_t *tx_data,
        uint16_t size,
        uint32_t timeout = HAL_MAX_DELAY
    ) const;

    HAL_StatusTypeDef receive(
        uint8_t *rx_data,
        uint16_t size,
        uint32_t timeout = HAL_MAX_DELAY
    ) const;

    HAL_StatusTypeDef transmitReceive(
        const uint8_t *tx_data,
        uint8_t *rx_data,
        uint16_t size,
        uint32_t timeout = HAL_MAX_DELAY
    ) const;

    /* DMA（非阻塞）收发，传输完成后触发 HAL_SPI_TxRxCpltCallback */
    HAL_StatusTypeDef transmitReceiveDMA(
        const uint8_t *tx_data,
        uint8_t *rx_data,
        uint16_t size
    ) const;

    /* DMA（非阻塞）发送，传输完成后触发 HAL_SPI_TxCpltCallback */
    HAL_StatusTypeDef transmitDMA(
        const uint8_t *tx_data,
        uint16_t size
    ) const;

    [[nodiscard]] SPI_HandleTypeDef *getHandle() const {
        return hspi_;
    }

private:
    SPI_HandleTypeDef *hspi_;
};
