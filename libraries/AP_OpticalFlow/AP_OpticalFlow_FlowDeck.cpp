/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "AP_OpticalFlow_FlowDeck.h"
#include <AP_RangeFinder/AP_RangeFinder.h>

#if AP_OPTICALFLOW_FLOWDECK_ENABLED
 
#include <AP_HAL/AP_HAL.h>
#include <AP_AHRS/AP_AHRS.h>
#include <utility>
#include <stdio.h>
 
extern const AP_HAL::HAL& hal;
 
// --- Constructor ---
AP_OpticalFlow_FlowDeck::AP_OpticalFlow_FlowDeck(const char *devname, AP_OpticalFlow &_frontend) :
    OpticalFlow_backend(_frontend),
    last_flow_us(0),
    last_update_ms(0),
    gyro_sum(0,0),
    gyro_sum_count(0)
{
    _dev = std::move(hal.spi->get_device(devname));
}
 
// --- Detect the sensor ---
AP_OpticalFlow_FlowDeck *AP_OpticalFlow_FlowDeck::detect(const char *devname, AP_OpticalFlow &_frontend)
{
    hal.console->printf("FlowDeck::detect START\n"); // DEBUG
    AP_OpticalFlow_FlowDeck *sensor = new AP_OpticalFlow_FlowDeck(devname, _frontend);
    if (!sensor) {
        return nullptr;
    }
    if (!sensor->setup_sensor()) {
        delete sensor;
        return nullptr;
    }
    return sensor;
}
 
// --- Setup the device ---
bool AP_OpticalFlow_FlowDeck::setup_sensor()
{
    hal.console->printf("FlowDeck::setup_sensor START\n"); // DEBUG
    if (!_dev) {
        hal.console->printf("FlowDeck: FAILED to get SPI device\n"); // DEBUG
        return false;
    }
    hal.console->printf("FlowDeck: Got SPI device OK\n"); // DEBUG
 
    // Get semaphore (threading)
    WITH_SEMAPHORE(_dev->get_semaphore());

    // --- Reset Sensor ---
    _dev->set_chip_select(false);  // HIGH (inactive)
    hal.scheduler->delay(40);  // Brief delay

    hal.console->printf("FlowDeck: Resetting sensor...\n"); // DEBUG
    // Reset sequence by toggling CS: HIGH->LOW->HIGH
    _dev->set_chip_select(false);  // HIGH (inactive)
    hal.scheduler->delay(2);  // Brief delay
    
    _dev->set_chip_select(true);   // LOW (active)
    hal.scheduler->delay(2);  // Brief delay
    
    _dev->set_chip_select(false);  // HIGH (inactive)
    hal.scheduler->delay(2);  // Brief delay

    hal.scheduler->delay(200);  // Brief delay

    // Power on reset
    reg_write(0x3A, 0x5A);
    hal.scheduler->delay(5);  // delay
    hal.console->printf("FlowDeck: Power on reset sent.\n"); // DEBUG
    // --- End of Reset Sequence ---

    // --- ID Check with Retries ---
    hal.console->printf("FlowDeck: Checking ID (will retry up to 5 times)...\n");   // DEBUG
    hal.console->flush();
    uint8_t id = 0;
    uint8_t id_inv = 0;
    bool id_ok = false;
    for (int i = 0; i < 5; i++) { // Loop up to 5 times
        id = reg_read(REG_ID);         // Read register 0x00
        id_inv = reg_read(REG_ID_INV); // Read register 0x5F
        hal.console->printf("FlowDeck: Attempt %d: Read ID=0x%02X, InvID=0x%02X\n", i + 1, id, id_inv);
        hal.console->flush();

        if (id == 0x49 && id_inv == 0xB6) { // Check for expected values
            id_ok = true;
            break; // Success, exit the loop
        }

        // If failed, wait briefly before retrying
        hal.scheduler->delay(5);
    }

    // Check if ID was successful after retries
    if (!id_ok) {
         hal.console->printf("FlowDeck: ID check FAILED after multiple attempts!\n");
         hal.console->flush();
         return false; // Exit setup if ID check failed
    }
    hal.console->printf("FlowDeck: ID check OK\n");
    hal.console->flush();
    // --- End of ID Check ---
 
    // Register periodic callback for sensor reading (every 10ms = 100Hz)
    hal.console->printf("FlowDeck: Registering periodic callback...\n"); // DEBUG
    bool registered = _dev->register_periodic_callback(10000, FUNCTOR_BIND_MEMBER(&AP_OpticalFlow_FlowDeck::timer, void));
    if (!registered) {
        hal.console->printf("FlowDeck: FAILED to register periodic callback\n"); // DEBUG
    } else {
        hal.console->printf("FlowDeck: Periodic callback registered OK\n"); // DEBUG
    }

    hal.console->printf("FlowDeck: Init Registers\n"); // DEBUG
 
    // --- Initialize sensor with required configuration ---
    // Write registers for improved performance
    initRegisters();

    // Reading the motion registers one time
    reg_read(0x02);
    reg_read(0x03);
    reg_read(0x04);
    reg_read(0x05);
    reg_read(0x06);
    hal.scheduler->delay(1);  // delay
    // --- End of Sensor Initialization

    hal.console->printf("FlowDeck: Turn on LED\n"); // DEBUG

    // Turn on LED
    setLED(true);

    hal.console->printf("FlowDeck: Setup Done!\n"); // DEBUG
     
    return true;
}
 
// --- Read register from sensor ---
uint8_t AP_OpticalFlow_FlowDeck::reg_read(uint8_t reg)
{
    uint8_t value_read = 0;

    bool success = _dev->read_registers(reg, &value_read, 1);   // MSB = 1

    if (!success) {
        hal.console->printf("Failed to read register 0x%02X\n", reg); // Log which register failed
        return 0;
    }

    return value_read;
}
 
// --- Write register to sensor ---
void AP_OpticalFlow_FlowDeck::reg_write(uint8_t reg, uint8_t value)
{
    uint8_t write_address = reg | 0x80; // MSB = 0
    
    bool success = _dev->write_register(write_address, value);

    if (!success) {
        hal.console->printf("Failed to write 0x%02X to register 0x%02X\n", value, reg);
    }
    
    hal.scheduler->delay_microseconds(50);  // Add delay in-between writes

}

// --- Enable Frame Buffer (For Camera Use) ---
void AP_OpticalFlow_FlowDeck::enable_frame_buffer()
{
    reg_write(0x7F, 0x07);  //Magic frame readout registers
    reg_write(0x41, 0x1D);
    reg_write(0x4C, 0x00);
    reg_write(0x7F, 0x08);
    reg_write(0x6A, 0x38);
    reg_write(0x7F, 0x00);
    reg_write(0x55, 0x04);
    reg_write(0x40, 0x80);
    reg_write(0x4D, 0x11);
  
    reg_write(0x70, 0x00);   //More magic? 
    reg_write(0x58, 0xFF);
  
    int temp, check;
  
    do { // keep reading
       temp = reg_read(0x58); // the status register
       check = temp>>6; // rightshift 6 bits so only top two stay 
    } while(check == 0x03); // while bits aren't set denoting ready state
    hal.scheduler->delay_microseconds(50);
}

// --- Read Frame Buffer (For Camera Use) ---
void AP_OpticalFlow_FlowDeck::read_frame_buffer(char *FBuffer)
{
    int count = 0;
    uint8_t a; //temp value for reading register
    uint8_t b; //temp value for second register
    uint8_t hold; //holding value for checking bits
    uint8_t mask = 0x0c; //mask to take bits 2 and 3 from b
    uint8_t pixel = 0; //temp holding value for pixel
  
    for (int ii = 0; ii < 1225; ii++) { //for 1 frame of 1225 pixels (35*35)
        do { 
        //if data is either invalid status
        //check status bits 6 and 7
        //if 01 move upper 6 bits into temp value
        //if 00 or 11, reread
        //else lower 2 bits into temp value
        a = reg_read(0x58); //read register
        hold = a >> 6; //right shift to leave top two bits for ease of check.
        } while((hold == 0x03) || (hold == 0x00));
      
        if (hold == 0x01) { //if data is upper 6 bits
            b = reg_read(0x58); //read next set to get lower 2 bits
            pixel = a; //set pixel to a
            pixel = pixel << 2; //push left to 7:2
            pixel += (b & mask); //set lower 2 from b to 1:0
            FBuffer[count++] = pixel; //put temp value in fbuffer array
            //delayMicroseconds(100);
        }
    }
    reg_write(0x70, 0x00);   //More magic? 
    reg_write(0x58, 0xFF);
  
    int temp, check; 
  
    do { //keep reading and testing
        temp = reg_read(0x58); //read status register
        check = temp>>6; //rightshift 6 bits so only top two stay 
    } while(check == 0x03); //while bits aren't set denoting ready state
}
 
// --- Read X,Y motion counts ---
void AP_OpticalFlow_FlowDeck::read_motion_count(int16_t *delta_x, int16_t *delta_y)
{
    reg_read(REG_MOTION);  // Motion Detection register
    *delta_x = ((int16_t)reg_read(0x04) << 8) | reg_read(0x03);
    hal.scheduler->delay_microseconds(50);
    *delta_y = ((int16_t)reg_read(0x06) << 8) | reg_read(0x05);
    hal.scheduler->delay_microseconds(50);
}
 
// --- Update Measurement (Called by the scheduler at regular intervals) ---
void AP_OpticalFlow_FlowDeck::timer()
{
    // hal.console->printf("FlowDeck: timer() called.\n"); // DEBUG (will very noisy)
    
    uint8_t motion = reg_read(REG_MOTION);

    // hal.console->printf("FlowDeck: motion=0x%02X\n", motion); // DEBUG
     
    // Only process if motion detected
    if (motion == 0xB0) {      // 0xB0 indicates valid motion data detected by the sensor
        // hal.console->printf("FlowDeck: motion detected!"); // DEBUG
        
        int16_t delta_x = 0;
        int16_t delta_y = 0;
        read_motion_count(&delta_x, &delta_y);

        //hal.console->printf("FlowDeck Update: dx=%d dy=%d\n", delta_x, delta_y); // DEBUG
         
        // Get surface quality
        uint8_t quality = reg_read(REG_QUALITY);
         
        // Get delta time since last successful reading
        const uint32_t now = AP_HAL::micros();
        float dt = (now - last_flow_us) * 1.0e-6f;
        last_flow_us = now;

        // Skip if dt is too small or too large
        if (!is_positive(dt) || dt > 0.5f) {
            return;
        }
         
        // Get gyro data for sensor compensation
        const Vector3f &gyro = AP::ahrs().get_gyro();
        gyro_sum.x += gyro.x;
        gyro_sum.y += gyro.y;
        gyro_sum_count++;
         
        // Apply sensor-specific constants (from Kalman filter)
        const float Npix = 30.0f;              // Number of pixels in field of view
        const float thetapix = radians(4.2f);  // Aperture angle per pixel in radians
        const float omegaFactor = 1.25f;       // Gyro compensation factor
        
        // Convert from sensor frame to flow rates
        // Note: We invert the calculation from the Kalman filter
        //       to go from pixel movement to flow rates
        
        // Prepare optical flow data
        struct AP_OpticalFlow::OpticalFlow_state state;
        
        // Set quality
        state.surface_quality = quality;
        
        // Calculate flow rates applying optical flow scaling and sensor characteristics
        const Vector2f flowScaler = _flowScaler();
        float flowScaleFactorX = 1.0f + 0.001f * flowScaler.x;
        float flowScaleFactorY = 1.0f + 0.001f * flowScaler.y;
        
        // Average body rates from accumulated gyro data
        float omegax_b = 0.0f;
        float omegay_b = 0.0f;
        if (gyro_sum_count > 0) {
            omegax_b = gyro_sum.x / gyro_sum_count;
            omegay_b = gyro_sum.y / gyro_sum_count;
        }
        
        // Height estimate - use rangefinder if available, otherwise use EKF height
        float height_estimate = 1.0f;  // Default if no height source available
        const auto *rangefinder = AP::rangefinder();
        if (rangefinder && rangefinder->status_orient(ROTATION_PITCH_270) == RangeFinder::Status::Good) {
            height_estimate = rangefinder->distance_orient(ROTATION_PITCH_270);
        } else {
            float hagl_meters = 0.0f; // Variable to store the result
            // Call get_hagl (Quality of measuerement) and check if the returned value is valid
            if (AP::ahrs().get_hagl(hagl_meters)) {
                height_estimate = hagl_meters; // Use valid HAGL estimate
            }
            // else: Keep the default height_estimate (1.0f) if get_hagl fails
        }
        
        // Ensure height is at least 0.1m to avoid division by zero
        height_estimate = MAX(height_estimate, 0.1f);
        
        // Convert raw pixel counts to flow rates
        // Based on the reverse of the Kalman filter prediction
        // We're solving for dx_g and dy_g using the measured pixel change
        
        // Get R[2][2] which is cosine of roll * cosine of pitch
        // This comes from the rotation matrix, in simple form:
        float R22 = cosf(AP::ahrs().get_roll()) * cosf(AP::ahrs().get_pitch());
        
        // Calculate flowRate from pixel movement (the inverse of the Kalman prediction)
        // We're solving for velocity given the pixel movement
        float flow_x = (float)delta_x * (thetapix / (Npix * dt)) * height_estimate / R22;
        float flow_y = (float)delta_y * (thetapix / (Npix * dt)) * height_estimate / R22;
        
        // Apply gyro compensation
        flow_x += omegaFactor * omegay_b;
        flow_y -= omegaFactor * omegax_b;
        
        // Scale by configured factors
        flow_x *= flowScaleFactorX;
        flow_y *= flowScaleFactorY;
        
        // Assign to state
        state.flowRate.x = flow_x;
        state.flowRate.y = flow_y;
        
        // Copy average body rate
        state.bodyRate.x = omegax_b;
        state.bodyRate.y = omegay_b;
        
        // Apply yaw correction to the flow rate
        _applyYaw(state.flowRate);
        
        // Update frontend
        _update_frontend(state);
        
        // Reset gyro sum
        gyro_sum.zero();
        gyro_sum_count = 0;
    }
}
 
// --- Update ---
void AP_OpticalFlow_FlowDeck::update()
{
    uint32_t now = AP_HAL::millis();
    if (now - last_update_ms < 100) {  // Limit updates to 10Hz (Necessary?)
        return;
    }
    last_update_ms = now;
}

// --- Init ---
void AP_OpticalFlow_FlowDeck::init()
{
    setup_sensor();
}
 
// --- Initialize the sensor registers (for performance) ---
void AP_OpticalFlow_FlowDeck::initRegisters()
{
    reg_write(0x7F, 0x00);
    reg_write(0x61, 0xAD);
    reg_write(0x7F, 0x03);
    reg_write(0x40, 0x00);
    reg_write(0x7F, 0x05);
    reg_write(0x41, 0xB3);
    reg_write(0x43, 0xF1);
    reg_write(0x45, 0x14);
    reg_write(0x5B, 0x32);
    reg_write(0x5F, 0x34);
    reg_write(0x7B, 0x08);
    reg_write(0x7F, 0x06);
    reg_write(0x44, 0x1B);
    reg_write(0x40, 0xBF);
    reg_write(0x4E, 0x3F);
    reg_write(0x7F, 0x08);
    reg_write(0x65, 0x20);
    reg_write(0x6A, 0x18);
    reg_write(0x7F, 0x09);
    reg_write(0x4F, 0xAF);
    reg_write(0x5F, 0x40);
    reg_write(0x48, 0x80);
    reg_write(0x49, 0x80);
    reg_write(0x57, 0x77);
    reg_write(0x60, 0x78);
    reg_write(0x61, 0x78);
    reg_write(0x62, 0x08);
    reg_write(0x63, 0x50);
    reg_write(0x7F, 0x0A);
    reg_write(0x45, 0x60);
    reg_write(0x7F, 0x00);
    reg_write(0x4D, 0x11);
    reg_write(0x55, 0x80);
    reg_write(0x74, 0x1F);
    reg_write(0x75, 0x1F);
    reg_write(0x4A, 0x78);
    reg_write(0x4B, 0x78);
    reg_write(0x44, 0x08);
    reg_write(0x45, 0x50);
    reg_write(0x64, 0xFF);
    reg_write(0x65, 0x1F);
    reg_write(0x7F, 0x14);
    reg_write(0x65, 0x67);
    reg_write(0x66, 0x08);
    reg_write(0x63, 0x70);
    reg_write(0x7F, 0x15);
    reg_write(0x48, 0x48);
    reg_write(0x7F, 0x07);
    reg_write(0x41, 0x0D);
    reg_write(0x43, 0x14);
    reg_write(0x4B, 0x0E);
    reg_write(0x45, 0x0F);
    reg_write(0x44, 0x42);
    reg_write(0x4C, 0x80);
    reg_write(0x7F, 0x10);
    reg_write(0x5B, 0x02);
    reg_write(0x7F, 0x07);
    reg_write(0x40, 0x41);
    reg_write(0x70, 0x00);

    hal.scheduler->delay(50); // delay 10ms

    reg_write(0x32, 0x44);
    reg_write(0x7F, 0x07);
    reg_write(0x40, 0x40);
    reg_write(0x7F, 0x06);
    reg_write(0x62, 0xF0);
    reg_write(0x63, 0x00);
    reg_write(0x7F, 0x0D);
    reg_write(0x48, 0xC0);
    reg_write(0x6F, 0xD5);
    reg_write(0x7F, 0x00);
    reg_write(0x5B, 0xA0);
    reg_write(0x4E, 0xA8);
    reg_write(0x5A, 0x50);
    reg_write(0x40, 0x80);

    reg_write(0x7F, 0x00);
    reg_write(0x5A, 0x10);
    reg_write(0x54, 0x00);
}

// --- Set LED (Not Working - Wrong register?) ---
void AP_OpticalFlow_FlowDeck::setLED(bool ledOn)
{
    reg_write(0x7f, 0x14);
    hal.scheduler->delay(5);
    reg_write(0x6f, ledOn ? 0x1c : 0x00);
    hal.scheduler->delay(5);
    reg_write(0x7f, 0x00);
    hal.scheduler->delay(5);
}
 
#endif // AP_OPTICALFLOW_FLOWDECK_ENABLED
