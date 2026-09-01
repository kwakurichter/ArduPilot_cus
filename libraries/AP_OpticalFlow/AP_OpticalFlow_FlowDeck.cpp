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

#if AP_OPTICALFLOW_FLOWDECK_ENABLED
 
#include <AP_HAL/AP_HAL.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_Logger/AP_Logger.h>
#include <utility>
#include <GCS_MAVLink/GCS.h>

#define FLOWDECK_MOTION_VALID 0xB0
 
extern const AP_HAL::HAL& hal;
 
// --- Constructor ---
AP_OpticalFlow_FlowDeck::AP_OpticalFlow_FlowDeck(const char *devname, AP_OpticalFlow &_frontend) :
    OpticalFlow_backend(_frontend),
    accumulator{},
    diagnostics{},
    last_flow_us(0),
    last_diagnostics_ms(0)
{
    _dev = std::move(hal.spi->get_device(devname));
}
 
// --- Detect the sensor ---
AP_OpticalFlow_FlowDeck *AP_OpticalFlow_FlowDeck::detect(const char *devname, AP_OpticalFlow &_frontend)
{
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
    if (!_dev) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "FlowDeck: SPI device not found");
        return false;
    }
 
    // Get semaphore (threading)
    WITH_SEMAPHORE(_dev->get_semaphore());

    // --- Reset Sensor ---
    _dev->set_chip_select(false);  // HIGH (inactive)
    hal.scheduler->delay(40);

    // Reset sequence by toggling CS: HIGH->LOW->HIGH
    _dev->set_chip_select(false);  // HIGH (inactive)
    hal.scheduler->delay(2);
    
    _dev->set_chip_select(true);   // LOW (active)
    hal.scheduler->delay(2);
    
    _dev->set_chip_select(false);  // HIGH (inactive)
    hal.scheduler->delay(2);

    hal.scheduler->delay(200);

    // Power on reset
    reg_write(0x3A, 0x5A);
    hal.scheduler->delay(5);  // delay
    
    // --- ID Check with Retries ---
    
    uint8_t id = 0;
    uint8_t id_inv = 0;
    bool id_ok = false;
    for (int i = 0; i < 10; i++) {
        id = reg_read(REG_ID);         // Read register 0x00
        id_inv = reg_read(REG_ID_INV); // Read register 0x5F

        if (id == 0x49 && id_inv == 0xB6) { // Check for expected values
            id_ok = true;
            break; // Success, exit the loop
        }

        // If failed, wait briefly before retrying
        hal.scheduler->delay(5);
    }

    // Check if ID was successful after retries
    if (!id_ok) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "FlowDeck: sensor ID check failed");
        return false;
    }

    // --- Initialize sensor with required configuration ---
    initRegisters();    // Write registers for improved performance

    // Reading the motion registers one time
    reg_read(0x02);
    reg_read(0x03);
    reg_read(0x04);
    reg_read(0x05);
    reg_read(0x06);
    hal.scheduler->delay(1);

    // Turn on LED
    setLED(true);   // Not Working?

    // Register only after configuration is complete so the callback cannot access a sensor that is still being reset or programmed.
    if (_dev->register_periodic_callback(10000, FUNCTOR_BIND_MEMBER(&AP_OpticalFlow_FlowDeck::timer, void)) == nullptr) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "FlowDeck: callback registration failed");
        return false;
    }

    return true;
}
 
// --- Read register from sensor ---
uint8_t AP_OpticalFlow_FlowDeck::reg_read(uint8_t reg)
{
    uint8_t value_read = 0;

    if (!_dev->read_registers(reg, &value_read, 1)) {
        return 0;
    }

    return value_read;
}
 
// --- Write register to sensor ---
void AP_OpticalFlow_FlowDeck::reg_write(uint8_t reg, uint8_t value)
{
    uint8_t write_address = reg | 0x80; // MSB = 0
    _dev->write_register(write_address, value);
    hal.scheduler->delay_microseconds(50);  // Add delay inbetween writes
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
    //reg_read(REG_MOTION);  // Motion Detection register
    *delta_x = ((int16_t)reg_read(0x04) << 8) | reg_read(0x03);
    hal.scheduler->delay_microseconds(50);
    *delta_y = ((int16_t)reg_read(0x06) << 8) | reg_read(0x05);
    hal.scheduler->delay_microseconds(50);
}

// --- Read X,Y motion counts, quality all at the same time ---
/*
  Read the PMW3901 motion report as a burst.

    0      motion
    1      observation
    2-3    delta_x, little endian
    4-5    delta_y
    6      squal
    7      rawdata_sum
    8      max_raw
    9      min_raw
    10-11  shutter
 */
bool AP_OpticalFlow_FlowDeck::read_motion_burst(int16_t &delta_x, int16_t &delta_y, uint8_t &quality, uint8_t &motion)
{
    struct PACKED {
        uint8_t motion;
        uint8_t observation;
        int16_t delta_x;
        int16_t delta_y;
        uint8_t squal;
        uint8_t rawdata_sum;
        uint8_t max_raw;
        uint8_t min_raw;
        uint8_t shutter_upper;
        uint8_t shutter_lower;
    } burst;

    // the read length below is sizeof(burst); if packing ever changed we would silently clock out the wrong number of bytes
    static_assert(sizeof(burst) == 12, "PMW3901 motion burst must be 12 bytes");

    delta_x = 0;
    delta_y = 0;
    quality = 0;
    motion = 0;

    // Hold CS across both transfers; the burst aborts if it is released early.
    if (!_dev->set_chip_select(true)) {
        return false;
    }

    uint8_t reg = REG_MOTION_BURST;
    bool ok = _dev->transfer(&reg, 1, nullptr, 0);
    if (ok) {
        // t_SRAD_MOTBR: the sensor needs this long before the report streams out
        hal.scheduler->delay_microseconds(150);
        ok = _dev->transfer(nullptr, 0, (uint8_t *)&burst, sizeof(burst));
    }

    _dev->set_chip_select(false);

    if (!ok) {
        return false;
    }

    delta_x = burst.delta_x;
    delta_y = burst.delta_y;
    quality = burst.squal;
    motion = burst.motion;

    return true;
}

// --- Update Measurement ---
void AP_OpticalFlow_FlowDeck::timer()
{
    const uint32_t now_us = AP_HAL::micros();
    if (last_flow_us == 0) {
        last_flow_us = now_us;
        return;
    }
    const uint32_t elapsed_us = now_us - last_flow_us;
    last_flow_us = now_us;

    if (elapsed_us == 0 || elapsed_us > 500000U) {
        WITH_SEMAPHORE(_sem);
        diagnostics.gap_reject_count++;
        return;
    }
    const float dt = elapsed_us * 1.0e-6f;

    int16_t delta_x = 0;
    int16_t delta_y = 0;
    uint8_t quality = 0;
    uint8_t motion = 0;
    const bool read_ok = read_motion_burst(delta_x, delta_y, quality, motion);

    WITH_SEMAPHORE(_sem);
    diagnostics.read_count++;
    if (!read_ok) {
        diagnostics.spi_fail_count++;
        return;
    }

    const int16_t delta_max = _flowdeck_delta_max();
    const int32_t abs_delta_x = delta_x < 0 ? -int32_t(delta_x) : int32_t(delta_x);
    const int32_t abs_delta_y = delta_y < 0 ? -int32_t(delta_y) : int32_t(delta_y);
    if (delta_max > 0 && (abs_delta_x >= delta_max || abs_delta_y >= delta_max)) {
        diagnostics.delta_reject_count++;
        return;
    }
    if (quality < _flowdeck_squal_min()) {
        diagnostics.squal_reject_count++;
        return;
    }
    if (motion != FLOWDECK_MOTION_VALID) {
        diagnostics.motion_reject_count++;
        return;
    }

    const Vector3f &gyro = AP::ahrs().get_gyro();
    accumulator.flow_sum.x += delta_x;
    accumulator.flow_sum.y += delta_y;
    accumulator.gyro_integral.x += gyro.x * dt;
    accumulator.gyro_integral.y += gyro.y * dt;
    accumulator.dt += dt;
    accumulator.quality_sum += quality;
    accumulator.sample_count++;
    diagnostics.accepted_count++;
}
 
// --- Update ---
void AP_OpticalFlow_FlowDeck::update()
{
    log_diagnostics();

    Accumulator data {};
    {
        WITH_SEMAPHORE(_sem);
        if (accumulator.dt < 0.1f || accumulator.sample_count == 0) {
            return;
        }
        data = accumulator;
        accumulator = {};
    }

    struct AP_OpticalFlow::OpticalFlow_state state = {};

    // 1. Calculate Scaler
    const Vector2f flowScaler = _flowScaler();
    const float flowScaleFactorX = 1.0f + 0.001f * flowScaler.x;
    const float flowScaleFactorY = 1.0f + 0.001f * flowScaler.y;

    // 2. Calculate Flow Rate (Velocity)
    // Velocity = (Accumulated Pixels * Scaling) / Accumulated Time
    // Invert X/Y here to match frame
    const float pixel_scaling = _flowdeck_scale();
    float flow_x_rad = -data.flow_sum.x * pixel_scaling;
    float flow_y_rad = -data.flow_sum.y * pixel_scaling;

    // 3. Apply the Parameter Scaler
    flow_x_rad *= flowScaleFactorX;
    flow_y_rad *= flowScaleFactorY;    

    state.flowRate.x = flow_x_rad / data.dt;
    state.flowRate.y = flow_y_rad / data.dt;

    // Calculate body rate over the same integration window as the flow data.
    state.bodyRate = data.gyro_integral / data.dt;

    state.surface_quality = constrain_int16(data.quality_sum / data.sample_count, 0, 255);

    // 6. Final Processing
    _applyYaw(state.flowRate);
    _update_frontend(state);

    WITH_SEMAPHORE(_sem);
    diagnostics.publish_count++;
}

void AP_OpticalFlow_FlowDeck::log_diagnostics()
{
    const uint32_t now_ms = AP_HAL::millis();
    if (now_ms - last_diagnostics_ms < 1000U) {
        return;
    }
    last_diagnostics_ms = now_ms;

    Diagnostics data {};
    {
        WITH_SEMAPHORE(_sem);
        data = diagnostics;
        diagnostics = {};
    }

#if HAL_LOGGING_ENABLED
    // @LoggerMessage: OFD
    // @Description: Crazyflie FlowDeck sample diagnostics accumulated since the previous message
    // @Field: TimeUS: Time since system startup
    // @Field: Read: Motion burst read attempts
    // @Field: Good: Accepted samples
    // @Field: SPI: Failed SPI burst reads
    // @Field: Mot: Samples rejected by the motion status gate
    // @Field: Dlt: Samples rejected by the raw delta gate
    // @Field: SQ: Samples rejected by the surface quality gate
    // @Field: Gap: Samples rejected due to an invalid time interval
    // @Field: Pub: Optical flow windows published to the frontend
    AP::logger().Write(
        "OFD",
        "TimeUS,Read,Good,SPI,Mot,Dlt,SQ,Gap,Pub",
        "s--------",
        "F--------",
        "QIIIIIIII",
        AP_HAL::micros64(),
        data.read_count,
        data.accepted_count,
        data.spi_fail_count,
        data.motion_reject_count,
        data.delta_reject_count,
        data.squal_reject_count,
        data.gap_reject_count,
        data.publish_count);
#endif
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