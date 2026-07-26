#pragma once

#include "main.h"
#include <cstdint>

/* 串口最大允许注册设备数量 */
#define USART_MAX_DEVICE 8

#define USART_RX_BUFFER_MAX 256

class USART {
public:
    enum class TransferMode : uint8_t {
        Blocking = 0,
        IT,
        DMA
    };

    using Callback_t = void (*)(USART *usart, uint16_t size);

    USART(UART_HandleTypeDef *usart_handle, uint16_t recv_buff_size, Callback_t usart_callback = nullptr);

    void init();

    void send(const uint8_t *buff, uint8_t len, TransferMode mode = TransferMode::DMA) const;

    [[nodiscard]] bool isReady() const;

    uint8_t *getRxBuff();

    /* 获取接收缓冲区长度 */
    [[nodiscard]] uint16_t getRxBuffLen() const { return recv_buff_size; };

    /* 获得 usart 句柄指针 */
    [[nodiscard]] UART_HandleTypeDef *getUartHandle() const { return usart_handle; }

    /* HAL 回调分发入口 */
    static void RxEventHandler(UART_HandleTypeDef *usart_handle, uint16_t size);

    static void ErrorHandler(UART_HandleTypeDef *usart_handle);

private:
    UART_HandleTypeDef *usart_handle = {}; /* USART 句柄指针 */
    uint16_t recv_buff_size = {}; /* USART 接收缓冲区大小 */
    Callback_t usart_callback = {}; /* USART 接收回调函数指针 */
    uint8_t recv_buff[USART_RX_BUFFER_MAX] = {}; /* USART 接收缓冲区 */

    static USART *usartInstances[USART_MAX_DEVICE]; /* USART 实例静态数组 */
    static uint8_t usartInstanceCount; /* USART 实例计数 */
};
