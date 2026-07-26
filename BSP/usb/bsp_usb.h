#pragma once

#include "usbd_cdc_if.h"

class USB {
public:
    uint8_t *init(USB_Callback rxCallback, USB_Callback txCallback);
    [[nodiscard]]uint8_t *getBuff() const { return rx_buff; }
    static bool transmit(uint8_t *txBuff, uint16_t len);
private:
    /* USB CDC 接收缓冲区 */
    uint8_t *rx_buff = nullptr;
};
