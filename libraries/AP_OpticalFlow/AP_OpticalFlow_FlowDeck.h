#pragma once

#include "AP_OpticalFlow_config.h"

#if AP_OPTICALFLOW_FLOWDECK_ENABLED

#include "AP_OpticalFlow_Backend.h"
#include <AP_HAL/utility/OwnPtr.h>

class AP_OpticalFlow_FlowDeck : public OpticalFlow_backend
{
public:
    /// constructor
    AP_OpticalFlow_FlowDeck(const char *devname, AP_OpticalFlow &_frontend);

    // read latest values from sensor and fill in x,y and totals.
    void update() override;

    // detect if the sensor is available
    static AP_OpticalFlow_FlowDeck *detect(const char *devname, AP_OpticalFlow &_frontend);

private:
    AP_HAL::OwnPtr<AP_HAL::SPIDevice> _dev;

    static const uint8_t REG_ID = 0x00;         // ID register
    static const uint8_t REG_ID_INV = 0x5F;     // Inverse ID register
    static const uint8_t REG_MOTION = 0x02;     // Motion register (not sure)
    static const uint8_t REG_QUALITY = 0x07;     // Quality register (not sure)
    static const uint8_t REG_MOTION_BURST = 0x16;  // PMW3901 motion burst; streams the whole report

    // read a register from the sensor
    uint8_t reg_read(uint8_t reg);
    
    // write a register to the sensor
    void reg_write(uint8_t reg, uint8_t value);
    
    // Initialize the sensor registers
    void initRegisters();
    
    // setup sensor
    bool setup_sensor();
    
    // read latest sensor data
    void timer();

    // read raw motion data
    void read_motion_count(int16_t *delta_x, int16_t *delta_y);

    // read raw motion data, quality and status at the same time
    bool read_motion_burst(int16_t &delta_x, int16_t &delta_y, uint8_t &quality, uint8_t &motion);
    
    // For Camera Use
    void enable_frame_buffer();
    void read_frame_buffer(char *FBuffer);
    
    // Activate LED
    void setLED(bool ledOn);
    
    struct Accumulator {
        Vector2f flow_sum;
        Vector2f gyro_integral;
        float dt;
        uint32_t quality_sum;
        uint16_t sample_count;
    } accumulator;

    struct Diagnostics {
        uint32_t read_count;
        uint32_t accepted_count;
        uint32_t spi_fail_count;
        uint32_t motion_reject_count;
        uint32_t delta_reject_count;
        uint32_t squal_reject_count;
        uint32_t gap_reject_count;
        uint32_t publish_count;
    } diagnostics;

    void log_diagnostics();

    uint32_t last_flow_us;            // timestamp of last flow reading
    uint32_t last_diagnostics_ms;     // timestamp of last diagnostic log message
};

#endif // AP_OPTICALFLOW_FLOWDECK_ENABLED