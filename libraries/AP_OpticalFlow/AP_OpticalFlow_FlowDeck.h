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

    // initialise the sensor
    void init() override;

    // read latest values from sensor and fill in x,y and totals.
    void update() override;

    // detect if the sensor is available
    static AP_OpticalFlow_FlowDeck *detect(const char *devname, AP_OpticalFlow &_frontend);

private:
    AP_HAL::OwnPtr<AP_HAL::SPIDevice> _dev;

    // Register definitions for your sensor
    static const uint8_t REG_ID = 0x00;         // ID register
    static const uint8_t REG_ID_INV = 0x5F;     // Inverse ID register
    static const uint8_t REG_MOTION = 0x02;     // Motion register (not sure)
    static const uint8_t REG_QUALITY = 0x07;     // Quality register (not sure)

    // to count init retries
    uint8_t _init_retries = 0; // Initialize to 0
    
    // Initialization status flag
    bool is_initialized = false;
    
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
    
    // For Camera Use
    void enable_frame_buffer();
    void read_frame_buffer(char *FBuffer);
    
    // Activate LED
    void setLED(bool ledOn);
    
    // Variables to store sensor state
    uint32_t last_flow_us;            // timestamp of last flow reading
    uint32_t last_update_ms;          // system time of last update
    Vector2f gyro_sum;                // sum of gyro sensor values since last frame
    uint16_t gyro_sum_count;          // number of gyro samples in sum

    // Define maximum retries
    static const uint8_t MAX_INIT_RETRIES = 10;
};

#endif // AP_OPTICALFLOW_FLOWDECK_ENABLED