#pragma once

#include <SITL/SITL.h>
#include "SIM_SerialDevice.h"
#include <AP_HAL/utility/DataRateLimit.h>

namespace SITL {

class ELRS : public SerialDevice {
public:
    ELRS(const uint8_t portNumber, HALSITL::SITL_State_Common *sitl_state);

    void begin(uint32_t baud, uint16_t rxS, uint16_t txS, const char* path) override;

    void uart_timer_tick() override;

    uint32_t device_baud() const override { return 460800; }

    void update();

private:
    void sendQueuedData();

    struct {
        mavlink_message_t rxmsg;
        mavlink_status_t status;
    } mavlink;

    uint8_t mavlink_parse_char_helper(uint8_t c, mavlink_message_t* r_message, mavlink_status_t* r_mavlink_status);

    ByteBuffer mavlinkInputBuffer;
    ByteBuffer mavlinkOutputBuffer;

    DataRateLimit input_limit;
    DataRateLimit output_limit;

    uint32_t lastSentFlowCtrl;
    HALSITL::UARTDriver *uart;

    const uint8_t this_system_id;
    const uint8_t this_component_id;

    // Air data rate limits in bytes per second
    const float input_data_rate;
    const float output_data_rate;

};

}
