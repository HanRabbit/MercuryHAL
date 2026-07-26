#include "bsp_usb.h"
#include "bsp_log.h"

/**
 * @brief USB 初始化函数，调用底层 CDC 初始化函数并传入接收和发送回调函数指针
 * @param rxCallback 指向 USB 数据接收回调函数的指针，当主机发送数据时会调用该函数
 * @param txCallback 指向 USB 数据发送完成回调函数的指针，当数据成功发送到主机后会调用该函数
 * @return 指向 USB CDC 接收缓冲区的指针，主机发送的数据将存储在该缓冲区中
 */
uint8_t *USB::init(const USB_Callback rxCallback, const USB_Callback txCallback) {
    rx_buff = CDC_Init(rxCallback, txCallback);
    LOG_INFO("[USB] Initialized with RX buffer at address %p", rx_buff);
    return rx_buff;
}

/**
 * @brief USB 数据发送函数，调用底层 CDC 传输函数将数据发送到主机
 * @param txBuff 指向要发送数据的缓冲区指针
 * @param len 要发送的数据长度，单位为字节
 * @return true 已成功提交发送; false 发送繁忙或失败
 */
bool USB::transmit(uint8_t *txBuff, const uint16_t len) {
    return CDC_Transmit_HS(txBuff, len) == USBD_OK;
}
