#include "AP_GPSParser.h"



AP_GPSParser::AP_GPSParser() {

}

void AP_GPSParser::setup_uart(AP_HAL::UARTDriver *uart_param, const char *name){
this->uart = uart_param;
    if (uart == nullptr) {
        // that UART doesn't exist on this platform
        return;
    }
    uart->begin(57600);
}

void AP_GPSParser::setup(){
    hal.scheduler->delay(1000); //Ensure that the uartA can be initialized

    setup_uart(hal.serial(3), "SERIAL3");  // 1st GPS
}


void AP_GPSParser::test_uart(AP_HAL::UARTDriver *uart_param, const char *name)
{
    if (uart_param == nullptr) {
        // that UART doesn't exist on this platform
        return;
    }
    uart_param->printf("Hello on UART %s at %.3f seconds\n",
                 name, (double)(AP_HAL::millis() * 0.001f));
                 uart_param->write("kalundborg er en skøge");
}


void AP_GPSParser::process() {

    test_uart(hal.serial(3), "SERIAL3");

    // also do a raw printf() on some platforms, which prints to the
    // debug console
    hal.console->printf("Hello on debug console at %.3f seconds\n", (double)(AP_HAL::millis() * 0.001f));

    hal.scheduler->delay(1000);


/*
    uint32_t key = 123; // Replace with your desired key
    
    while (uart->available_locked(key) > 0) {
        uint8_t received_byte;
        ssize_t bytesRead = uart->read_locked(&received_byte, 1, key);
        double last_run_time = AP_HAL::millis();

        // 100hz loop
        if (AP_HAL::millis() - last_run_time < 20) {
            hal.scheduler->delay(10);
            continue;
        }

        if (bytesRead > 0) {
            if (parseMavlinkByte(received_byte)) {
                if (processMavlinkMessage(mavlink_buffer, mavlink_buffer_index)) {
                    // Process the GPS data, e.g., update the gps_data structure
                }
                mavlink_buffer_index = 0;
            }
        }
    }
    */
}

bool AP_GPSParser::parseMavlinkByte(uint8_t byte) {
    if (mavlink_buffer_index < sizeof(mavlink_buffer)) {
        mavlink_buffer[mavlink_buffer_index] = byte;
        mavlink_buffer_index++;
        if (mavlink_buffer_index >= 6 + mavlink_buffer[1]) {
            return true;
        }
    }
    return false;
}

bool AP_GPSParser::processMavlinkMessage(const uint8_t* buffer, uint16_t length) {
    // Implement your logic to parse the MAVLink message and update gps_data
    // Return true if GPS data is successfully extracted, otherwise false
    return false; // Replace with your parsing logic
  
}
 
