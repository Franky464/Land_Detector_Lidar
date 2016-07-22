#pragma once

#include "AP_HAL_Linux.h"

#include "UARTDriver.h"


class Linux::SPIUARTDriver : public Linux::UARTDriver {
public:
    SPIUARTDriver();
    void begin(uint32_t b, uint16_t rxS, uint16_t txS);
    void _timer_tick(void);

protected:
    int _write_fd(const uint8_t *buf, uint16_t n);
    int _read_fd(uint8_t *buf, uint16_t n);

    AP_HAL::OwnPtr<AP_HAL::SPIDevice> _dev;

    uint8_t *_buffer;

    uint32_t _last_update_timestamp;

    bool _external;
};
