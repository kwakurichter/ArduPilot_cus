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
#include <AP_Math/crc.h>
#include <utility>
#include <stdio.h>
#include <GCS_MAVLink/GCS.h>

#define FLOWDECK_PIXEL_SCALING      (4.2e-3)
 
extern const AP_HAL::HAL& hal;
 
// --- Constructor ---
AP_OpticalFlow_FlowDeck::AP_OpticalFlow_FlowDeck(const char *devname, AP_OpticalFlow &_frontend) :
    OpticalFlow_backend(_frontend),
    last_flow_us(0),
    last_update_ms(0),
    gyro_sum(0,0),
    gyro_sum_count(0),
    flow_sum(0,0),
    flow_dt(0),
    qual_sum(0)
{
    _dev = std::move(hal.spi->get_device(devname));
}
 
// --- Detect the sensor ---
AP_OpticalFlow_FlowDeck *AP_OpticalFlow_FlowDeck::detect(const char *devname, AP_OpticalFlow &_frontend)
{
    //gcs().send_text(MAV_SEVERITY_ALERT, "FlowDeck::detect START\n"); // DEBUG
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
    //gcs().send_text(MAV_SEVERITY_ALERT, "FlowDeck::setup_sensor START\n"); // DEBUG
    if (!_dev) {
        gcs().send_text(MAV_SEVERITY_ALERT, "FlowDeck: FAILED to get SPI device\n"); // DEBUG
        return false;
    }
    //gcs().send_text(MAV_SEVERITY_ALERT, "FlowDeck: Got SPI device OK\n"); // DEBUG
 
    // Get semaphore (threading)
    WITH_SEMAPHORE(_dev->get_semaphore());

    // --- Reset Sensor ---
    _dev->set_chip_select(false);  // HIGH (inactive)
    hal.scheduler->delay(40);

    //gcs().send_text(MAV_SEVERITY_ALERT, "FlowDeck: Resetting sensor...\n"); // DEBUG

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
    
    //gcs().send_text(MAV_SEVERITY_ALERT, "FlowDeck: Power on reset sent.\n"); // DEBUG
    
    // --- End of Reset Sequence ---

    // --- ID Check with Retries ---

    //gcs().send_text(MAV_SEVERITY_ALERT, "FlowDeck: Checking ID (will retry up to 10 times)...\n"); // DEBUG
    
    uint8_t id = 0;
    uint8_t id_inv = 0;
    bool id_ok = false;
    for (int i = 0; i < 10; i++) { // Loop up to 5 times
        id = reg_read(REG_ID);         // Read register 0x00
        id_inv = reg_read(REG_ID_INV); // Read register 0x5F

        //gcs().send_text(MAV_SEVERITY_ALERT, "FlowDeck: Attempt %d: Read ID=0x%02X, InvID=0x%02X\n", i + 1, id, id_inv); // DEBUG

        if (id == 0x49 && id_inv == 0xB6) { // Check for expected values
            id_ok = true;
            break; // Success, exit the loop
        }

        // If failed, wait briefly before retrying
        hal.scheduler->delay(5);
    }

    // Check if ID was successful after retries
    if (!id_ok) {
         gcs().send_text(MAV_SEVERITY_ALERT, "FlowDeck: ID check FAILED after multiple attempts!\n"); // DEBUG
         return false; // Exit setup if ID check failed
    }

    //gcs().send_text(MAV_SEVERITY_ALERT, "FlowDeck: ID check OK\n"); // DEBUG

    // --- End of ID Check ---
 
    // Register periodic callback for sensor reading (every 10ms = 100Hz)

    //gcs().send_text(MAV_SEVERITY_ALERT, "FlowDeck: Registering periodic callback...\n"); // DEBUG

    bool registered = _dev->register_periodic_callback(10000, FUNCTOR_BIND_MEMBER(&AP_OpticalFlow_FlowDeck::timer, void));
    if (!registered) {
        gcs().send_text(MAV_SEVERITY_ALERT, "FlowDeck: FAILED to register periodic callback\n"); // DEBUG
    } else {
        //gcs().send_text(MAV_SEVERITY_ALERT, "FlowDeck: Periodic callback registered OK\n"); // DEBUG
    }

    //gcs().send_text(MAV_SEVERITY_ALERT, "FlowDeck: Init Registers\n"); // DEBUG
 
    // --- Initialize sensor with required configuration ---
    initRegisters();    // Write registers for improved performance

    // Reading the motion registers one time
    reg_read(0x02);
    reg_read(0x03);
    reg_read(0x04);
    reg_read(0x05);
    reg_read(0x06);
    hal.scheduler->delay(1);  // delay
    // --- End of Sensor Initialization

    //gcs().send_text(MAV_SEVERITY_ALERT, "FlowDeck: Turn on LED\n"); // DEBUG

    // Turn on LED
    setLED(true);   // Not Working?

    gcs().send_text(MAV_SEVERITY_ALERT, "FlowDeck: Setup Done!\n"); // DEBUG
     
    return true;
}
 
// --- Read register from sensor ---
uint8_t AP_OpticalFlow_FlowDeck::reg_read(uint8_t reg)
{
    uint8_t value_read = 0;

    bool success = _dev->read_registers(reg, &value_read, 1);   // MSB = 1

    if (!success) {
        // gcs().send_text(MAV_SEVERITY_DEBUG, "Failed to read register 0x%02X\n", reg); // DEBUG
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
        // gcs().send_text(MAV_SEVERITY_DEBUG, "Failed to write 0x%02X to register 0x%02X\n", value, reg); // DEBUG
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
    //reg_read(REG_MOTION);  // Motion Detection register
    *delta_x = ((int16_t)reg_read(0x04) << 8) | reg_read(0x03);
    hal.scheduler->delay_microseconds(50);
    *delta_y = ((int16_t)reg_read(0x06) << 8) | reg_read(0x05);
    hal.scheduler->delay_microseconds(50);
}

// --- Read X,Y motion counts, quality all at the same time ---
/*
  Read the PMW3901 motion report as a burst.

  Reading the fields individually costs six SPI transactions, each with its
  own chip-select assertion and a t_SRR delay between them, and the periodic
  callback holds the SPI bus semaphore for all of it. Burst mode streams the
  whole report under one chip-select, so this is two transfers and one delay
  on a bus shared with the SD card and the Loco deck.

  Layout streamed from REG_MOTION_BURST, per the PMW3901 datasheet and
  matching AP_OpticalFlow_Pixart's MotionBurst struct:

    0      motion
    1      observation
    2-3    delta_x, little endian
    4-5    delta_y
    6      squal
    7      rawdata_sum
    8      max_raw
    9      min_raw
    10-11  shutter

  Note reads send the bare register address; only writes set the MSB. That is
  the PixArt convention, and this driver never calls set_read_flag(), so the
  HAL leaves the address alone.
 */
bool AP_OpticalFlow_FlowDeck::read_motion_burst(int16_t &delta_x, int16_t &delta_y, uint8_t &quality)
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

    // the read length below is sizeof(burst); if packing ever changed we would
    // silently clock out the wrong number of bytes
    static_assert(sizeof(burst) == 12, "PMW3901 motion burst must be 12 bytes");

    delta_x = 0;
    delta_y = 0;
    quality = 0;

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

    /*
      Deltas are reported whether or not the motion bit is set: a still frame
      is a genuine zero-displacement sample, and dropping it would leave the
      integration in timer() accumulating dt without the matching flow.
     */
    delta_x = burst.delta_x;
    delta_y = burst.delta_y;
    quality = burst.squal;

    return true;
}

// --- Update Measurement ---
void AP_OpticalFlow_FlowDeck::timer()
{
    // 1. Calculate dt for this specific sample
    uint32_t now_us = AP_HAL::micros();
    float dt = (now_us - last_flow_us) * 1.0e-6f;

    // Sanity check dt
    if (dt > 0.5f) { // Reset if too much time has passed
        last_flow_us = now_us;
        return;
    }

    // 2. Perform Burst Read
    int16_t delta_x = 0;
    int16_t delta_y = 0;
    uint8_t quality = 0;

    if (read_motion_burst(delta_x, delta_y, quality)) {
        // Successful read
        last_flow_us = now_us;

        // 3. Accumulate Raw Data
        // We only accumulate if the data is valid, or we can accumulate everything and filter in update(). Accumulating everything is safer for integration.
        flow_sum.x += delta_x;
        flow_sum.y += delta_y;
        flow_dt += dt;
        qual_sum += quality;    // Store quality for reporting
        
        // Accumulate Gyro Data for compensation
        const Vector3f &gyro = AP::ahrs().get_gyro();
        gyro_sum.x += gyro.x;
        gyro_sum.y += gyro.y;
        gyro_sum_count++;
        
    } else {
        gcs().send_text(MAV_SEVERITY_DEBUG, "FlowDeck: Burst Read Failed\n"); // DEBUG
    }
}
 
// --- Update ---
void AP_OpticalFlow_FlowDeck::update()
{
    // Return if no sufficient time has accumulated to avoid singularity or noise
    if (flow_dt < 1.0e-1f) { // wait for at least 100ms of data
        return;
    }

    struct AP_OpticalFlow::OpticalFlow_state state = {};

    // 1. Calculate Scaler
    const Vector2f flowScaler = _flowScaler();
    float flowScaleFactorX = 1.0f + 0.001f * flowScaler.x;
    float flowScaleFactorY = 1.0f + 0.001f * flowScaler.y;

    // 2. Calculate Flow Rate (Velocity)
    // Velocity = (Accumulated Pixels * Scaling) / Accumulated Time
    // Invert X/Y here to match frame
    float flow_x_rad = (float)(-flow_sum.x) * FLOWDECK_PIXEL_SCALING;
    float flow_y_rad = (float)(-flow_sum.y) * FLOWDECK_PIXEL_SCALING;

    // 3. Apply the Parameter Scaler
    flow_x_rad *= flowScaleFactorX;
    flow_y_rad *= flowScaleFactorY;    

    state.flowRate.x = flow_x_rad / flow_dt;
    state.flowRate.y = flow_y_rad / flow_dt;

    // 4. Calculate Body Rate (Average Gyro)
    if (gyro_sum_count > 0) {
        state.bodyRate.x = gyro_sum.x / gyro_sum_count;
        state.bodyRate.y = gyro_sum.y / gyro_sum_count;
    } else {
        state.bodyRate.zero();
    }

    // 5. Surface Quality
    state.surface_quality = constrain_int16((qual_sum / gyro_sum_count), 0, 255);

    // 6. Final Processing
    _applyYaw(state.flowRate);
    _update_frontend(state);

    // 7. Reset Accumulators
    flow_sum.x = 0;
    flow_sum.y = 0;
    flow_dt = 0;
    qual_sum = 0;
    gyro_sum.zero();
    gyro_sum_count = 0;
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