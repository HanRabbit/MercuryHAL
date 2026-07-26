#include "bsp_usart.h"
#include "bsp_log.h"
#include <cstring>

/* USART 实例静态数组与计数（避免在 early runtime 使用堆）*/
USART *USART::usartInstances[USART_MAX_DEVICE] = {nullptr};
uint8_t USART::usartInstanceCount = 0;

/**
 * @brief USART 构造函数
 * @param usart_handle USART 句柄指针
 * @param recv_buff_size USART 接收缓冲区大小
 * @param usart_callback USART 接收回调函数指针
 */
USART::USART(UART_HandleTypeDef *usart_handle, const uint16_t recv_buff_size, const Callback_t usart_callback) {
    this->usart_handle = usart_handle;
    this->recv_buff_size = recv_buff_size;
    this->usart_callback = usart_callback;

    if (usartInstanceCount < USART_MAX_DEVICE) {
        /* 确保同一 USART 句柄不会被重复注册 */
        for (uint8_t i = 0; i < usartInstanceCount; i++) {
            if (usartInstances[i]->getUartHandle() == usart_handle) {
                LOG_ERROR("[USART] USART instance with the same handle already exists!");
                return;
            }
        }
        usartInstances[usartInstanceCount++] = this;
    } else {
        /* 超出 USART 最大注册限制，记录错误并返回 */
        LOG_ERROR("[USART] Instance count exceeds maximum limit!");
    }
}

/**
 * @brief USART 初始化函数，启动接收 DMA 并配置相关中断
 */
void USART::init() {
    HAL_UARTEx_ReceiveToIdle_DMA(usart_handle, recv_buff, recv_buff_size); /* 启动 USART 接收 DMA，接收完成或空闲时触发中断 */
    __HAL_DMA_DISABLE_IT(usart_handle->hdmarx, DMA_IT_HT); /* 禁止半传输中断，避免在接收过程中被频繁触发 */
}

/**
 * @brief USART 发送函数，根据传输模式选择不同的发送方式
 * @param buff 发送数据缓冲区指针
 * @param len 发送数据长度
 * @param mode 传输模式（Blocking、IT、DMA）
 */
void USART::send(const uint8_t *buff, const uint8_t len, const TransferMode mode) const {
    switch (mode) {
        case TransferMode::Blocking:
            HAL_UART_Transmit(usart_handle, buff, len, HAL_MAX_DELAY);
            break;
        case TransferMode::IT:
            HAL_UART_Transmit_IT(usart_handle, buff, len);
            break;
        case TransferMode::DMA:
            HAL_UART_Transmit_DMA(usart_handle, buff, len);
            break;
        default:
            LOG_ERROR("[USART] Invalid transfer mode!");
            break;
    }
}

/**
 * @brief USART 就绪状态检查函数，判断当前是否可以进行新的发送操作
 * @return true 如果 USART 当前不处于发送忙碌状态，false 否则
 */
bool USART::isReady() const {
    return !(usart_handle->gState & HAL_UART_STATE_BUSY_TX);
}

/**
 * @brief 获取 USART 接收缓冲区指针，供外部访问接收数据
 * @return uint8_t* USART 接收缓冲区指针
 */
uint8_t *USART::getRxBuff() {
    return recv_buff;
}

/**
 * @brief USART 接收事件处理函数，供 HAL UART 接收完成或空闲中断调用
 * @param usart_handle 触发事件的 USART 句柄指针
 * @param size 本次接收的数据长度（不包括空闲时的 DMA 传输剩余数据）
 */
void USART::RxEventHandler(UART_HandleTypeDef *usart_handle, uint16_t size) {
    for (size_t i = 0; i < usartInstanceCount; i++) {
        if (usartInstances[i]->getUartHandle() == usart_handle) {
            if (usartInstances[i]->usart_callback) {
                usartInstances[i]->usart_callback(usartInstances[i], size);
            }

            HAL_UARTEx_ReceiveToIdle_DMA(usart_handle, usartInstances[i]->recv_buff, usartInstances[i]->recv_buff_size);
            /* 重新启动 USART 接收 DMA，准备下一次接收 */
            __HAL_DMA_DISABLE_IT(usart_handle->hdmarx, DMA_IT_HT);

            return;
        }
    }
}

/**
 * @brief USART 错误事件处理函数，供 HAL UART 错误中断调用
 * @param usart_handle 触发事件的 USART 句柄指针
 */
void USART::ErrorHandler(UART_HandleTypeDef *usart_handle) {
    for (size_t i = 0; i < usartInstanceCount; i++) {
        if (usartInstances[i]->getUartHandle() == usart_handle) {
            HAL_UARTEx_ReceiveToIdle_DMA(usart_handle, usartInstances[i]->recv_buff, usartInstances[i]->recv_buff_size);
            /* 重新启动 USART 接收 DMA，准备下一次接收 */
            __HAL_DMA_DISABLE_IT(usart_handle->hdmarx, DMA_IT_HT);

            LOG_ERROR("[USART] USART[%d] error occurred!", i);
            return;
        }
    }
}

extern "C"
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
    USART::RxEventHandler(huart, Size);
}

extern "C"
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
    USART::ErrorHandler(huart);
}
