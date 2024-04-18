#include <AP_HAL/AP_HAL.h>

// Only support ELRS simulation in SITL (not Sim on Hardware)
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL

#include "SIM_ELRS.h"
#include <SITL/SITL.h>

#include <AP_HAL_SITL/AP_HAL_SITL.h>
#include <AP_HAL_SITL/UARTDriver.h>

#include "include/mavlink/v2.0/all/mavlink.h"

#define MAV_FTP_OPCODE_OPENFILERO 4

// Example command: -A --serial2=sim:ELRS:tcp:3

// Baud rate must be set correctly
// param set SERIAL2_BAUD 460

using namespace SITL;

ELRS::ELRS(const uint8_t portNumber, HALSITL::SITL_State_Common *sitl_state) :
    // Mirror typical ELRS UART buffer sizes
    SerialDevice::SerialDevice(64, 128),
    // Mirror MAVLink buffer sizes
    mavlinkInputBuffer(2048),
    mavlinkOutputBuffer(2048),
    // Typical setup is about 500 B /s
    input_data_rate(500*5),
    output_data_rate(500*5),
    // 255 is typically used by the GCS, for RC override to work in ArduPilot `SYSID_MYGCS` must be set to this value (255 is the default)
    this_system_id(255),
    // Strictly this is not a valid source component ID
    this_component_id(MAV_COMPONENT::MAV_COMP_ID_ALL)
{
    // Create a UART to talk to the GCS as normal
    uart = new HALSITL::UARTDriver(portNumber, (HALSITL::SITL_State*)sitl_state);
    if (uart == nullptr) {
        AP_HAL::panic("ELRS uart nullptr");
        return;
    }
}

// Called when begin is called on main serial port
void ELRS::begin(uint32_t baud, uint16_t rxS, uint16_t txS, const char* path)
{
    if (uart == nullptr) {
        AP_HAL::panic("ELRS uart nullptr");
        return;
    }

    // Start of path must be "sim:ELRS:" since we exist
    // Offset by those 9 characters for child uart
    uart->begin(baud, rxS, txS, path + 9);

    // Call through to base class method
    SerialDevice::begin(baud, rxS, txS, nullptr);
}

void ELRS::uart_timer_tick()
{
    update();
}

void ELRS::update()
{
    if (uart == nullptr) {
        AP_HAL::panic("ELRS uart nullptr");
        return;
    }
    uart->_timer_tick();

    // Read from AP into radio
    const uint32_t input_space = mavlinkInputBuffer.space();
    if (input_space > 0) {
        uint8_t buf[input_space];
        ssize_t len = read_from_autopilot((char*)buf, input_space);
        mavlinkInputBuffer.write(buf, len);
    }

    // Send from radio to GCS
    const uint32_t send_bytes = input_limit.max_bytes(input_data_rate);
    if (send_bytes > 0) {
        uint8_t buf[send_bytes];
        const uint32_t len = mavlinkInputBuffer.read(buf, send_bytes);
        uart->write(buf, len);
    }

    // Incoming data from GCS to radio
    const uint32_t receive_bytes = output_limit.max_bytes(output_data_rate);
    if (receive_bytes > 0) {
        uint8_t buf[receive_bytes];
        const uint32_t len = uart->read(buf, receive_bytes);
        mavlinkOutputBuffer.write(buf, len);
    }

    // Write from radio to AP
    sendQueuedData();
}

// Function to behave like MAVLink libs `mavlink_parse_char` but use local buffer
uint8_t ELRS::mavlink_parse_char_helper(uint8_t c, mavlink_message_t* r_message, mavlink_status_t* r_mavlink_status)
{
    uint8_t msg_received = mavlink_frame_char_buffer(&mavlink.rxmsg, &mavlink.status, c, r_message, r_mavlink_status);
    if ((msg_received == MAVLINK_FRAMING_BAD_CRC) || (msg_received == MAVLINK_FRAMING_BAD_SIGNATURE)) {
        return 0;
    }
    return msg_received;
}

// Send incoming data to AP, this is a re-implementation of the ELRS function found here:
// https://github.com/ExpressLRS/ExpressLRS/blob/0d31863f34ca16a036e94a9c2a56038ae56c7f9e/src/src/rx-serial/SerialMavlink.cpp#L78
void ELRS::sendQueuedData()
{

    // Send radio messages at 100Hz
    const uint32_t now = AP_HAL::millis();
    if ((now - lastSentFlowCtrl) > 10) {
        lastSentFlowCtrl = now; 

        // Space remaining as a percentage.
        const uint8_t percentage_remaining = (mavlinkInputBuffer.space() * 100) / mavlinkInputBuffer.get_size();

        // Populate radio status packet
        const mavlink_radio_status_t radio_status {
            rxerrors: 0,
            fixed: 0,
            rssi: UINT8_MAX, // Unknown
            remrssi: UINT8_MAX, // Unknown
            txbuf: percentage_remaining,
            noise: UINT8_MAX, // Unknown
            remnoise: UINT8_MAX, // Unknown
        };

        uint8_t buf[MAVLINK_MSG_ID_RADIO_STATUS_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES];
        mavlink_message_t msg;
        mavlink_msg_radio_status_encode(this_system_id, this_component_id, &msg, &radio_status);
        uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
        write_to_autopilot((char*)buf, len);
    }

    // Read one byte at a time until were done
    while (true) {
        uint8_t c;
        if (!mavlinkOutputBuffer.read_byte(&c)) {
            break;
        }

        mavlink_message_t msg;
        mavlink_status_t status;

        // Try parse a mavlink message
        if (mavlink_parse_char_helper(c, &msg, &status)) {
            // Message decoded successfully

            // Forward message to the UART
            uint8_t buf[MAVLINK_MAX_PACKET_LEN];
            uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
            uint16_t written = write_to_autopilot((char*)buf, len);
            if ((written != uint16_t(-1)) && (len != written)) {
                ::fprintf(stderr, "Failed to write full msg, wanted %u achieved %u (msg id: %u)\n", len, written, msg.msgid);
            }
        }
    }

}

#endif // CONFIG_HAL_BOARD == HAL_BOARD_SITL
