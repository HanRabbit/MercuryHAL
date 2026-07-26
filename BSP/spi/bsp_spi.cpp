#include "bsp_spi.h"

SPI::SPI(SPI_HandleTypeDef *hspi) : hspi_(hspi) {
}

/**
 * @brief 通过 SPI 发送数据
 * @param tx_data 指向要发送的数据缓冲区的指针
 * @param size 要发送的数据大小（以字节为单位）
 * @param timeout 超时时间，单位为毫秒，默认为 HAL_MAX_DELAY 表示无限等待
 * @return HAL_StatusTypeDef SPI 传输状态，HAL_OK 表示成功，其他值表示错误
 */
HAL_StatusTypeDef SPI::transmit(
    const uint8_t *tx_data,
    const uint16_t size, const uint32_t timeout) const {
    return HAL_SPI_Transmit(hspi_, tx_data, size, timeout);
}

/**
 * @brief 通过 SPI 接收数据
 * @param rx_data 指向接收数据缓冲区的指针，接收到的数据将存储在该缓冲区中
 * @param size 要接收的数据大小（以字节为单位）
 * @param timeout 超时时间，单位为毫秒，默认为 HAL_MAX_DELAY 表示无限等待
 * @return HAL_StatusTypeDef SPI 接收状态，HAL_OK 表示成功，其他值表示错误
 */
HAL_StatusTypeDef SPI::receive(
    uint8_t *rx_data,
    const uint16_t size,
    const uint32_t timeout
) const {
    return HAL_SPI_Receive(hspi_, rx_data, size, timeout);
}

/**
 * @brief 通过 SPI 同时发送和接收数据
 * @param tx_data 指向要发送的数据缓冲区的指针
 * @param rx_data 指向接收数据缓冲区的指针，接收到的数据将存储在该缓冲区中
 * @param size 要发送和接收的数据大小（以字节为单位）
 * @param timeout 超时时间，单位为毫秒，默认为 HAL_MAX_DELAY 表示无限等待
 * @return HAL_StatusTypeDef SPI 传输状态，HAL_OK 表示成功，其他值表示错误
 */
HAL_StatusTypeDef SPI::transmitReceive(
    const uint8_t *tx_data,
    uint8_t *rx_data,
    const uint16_t size,
    const uint32_t timeout
) const {
    return HAL_SPI_TransmitReceive(hspi_, tx_data, rx_data, size, timeout);
}

/**
 * @brief 通过 SPI 以 DMA 方式同时发送和接收数据（非阻塞）
 * @param tx_data 指向要发送的数据缓冲区的指针，缓冲区在传输完成前必须保持有效
 * @param rx_data 指向接收数据缓冲区的指针，缓冲区在传输完成前必须保持有效
 * @param size 要发送和接收的数据大小（以字节为单位）
 * @return HAL_StatusTypeDef 传输启动状态，HAL_OK 表示 DMA 已成功启动
 * @note 函数立即返回，实际数据在传输完成后由 HAL_SPI_TxRxCpltCallback 处理；
 *       缓冲区不可放在栈上，且需位于 DMA 可访问的内存域（本工程使用 RAM_D1）
 */
HAL_StatusTypeDef SPI::transmitReceiveDMA(
    const uint8_t *tx_data,
    uint8_t *rx_data,
    const uint16_t size
) const {
    return HAL_SPI_TransmitReceive_DMA(hspi_, const_cast<uint8_t *>(tx_data), rx_data, size);
}

/**
 * @brief 通过 SPI 以 DMA 方式发送数据（非阻塞）
 * @param tx_data 指向要发送的数据缓冲区的指针，缓冲区在传输完成前必须保持有效
 * @param size 要发送的数据大小（以字节为单位）
 * @return HAL_StatusTypeDef 传输启动状态，HAL_OK 表示 DMA 已成功启动
 * @note 函数立即返回，传输完成后触发 HAL_SPI_TxCpltCallback
 */
HAL_StatusTypeDef SPI::transmitDMA(
    const uint8_t *tx_data,
    const uint16_t size
) const {
    return HAL_SPI_Transmit_DMA(hspi_, const_cast<uint8_t *>(tx_data), size);
}
