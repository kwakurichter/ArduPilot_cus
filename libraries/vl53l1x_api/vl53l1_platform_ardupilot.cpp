/**
 * @file  vl53l1_platform_ardupilot.cpp
 * @brief Platform implementation for VL53L1 API on ArduPilot HAL
 */

 #include "vl53l1_platform.h" // Include the header above
 #include <AP_HAL/AP_HAL.h> // Access to hal global
 #include <stdlib.h>       // For malloc/free
 #include <string.h>       // For memcpy
 #include <AP_HAL/I2CDevice.h>
 
 // Ensure ST API error codes are available
 #include "vl53l1_error_codes.h"
 #include "vl53l1_platform_user_data.h"   // defines VL53L1_Dev_t
 
 static AP_HAL::I2CDevice *g_i2c_dev = nullptr;

 extern const AP_HAL::HAL& hal;

 extern "C" void VL53L1_set_aphal_device(AP_HAL::I2CDevice*);

 /* helper macro now just returns the static variable */
 #define STDEV_GET_APHAL_DEV(Dev) (g_i2c_dev)

 /* ------------------------------------------------------------------ *
 *  Public setter so the backend can pass us its AP_HAL::I2CDevice*
 * ------------------------------------------------------------------ */
 extern "C" void VL53L1_set_aphal_device(AP_HAL::I2CDevice *dev)
 {
     g_i2c_dev = dev;
 }

 // Define default chunk size if not provided by user defines header
 #ifndef VL53L1_COMMS_CHUNK_SIZE
 #define VL53L1_COMMS_CHUNK_SIZE   64 // A reasonable default
 #endif
 
 VL53L1_Error VL53L1_WriteMulti(VL53L1_DEV Dev, uint16_t index, uint8_t *pdata, uint32_t count)
 {
     AP_HAL::I2CDevice *i2c_dev = STDEV_GET_APHAL_DEV(Dev);
     if (!i2c_dev) {
         return VL53L1_ERROR_INVALID_PARAMS; // Device handle not setup in Dev->Data
     }
 
     bool success = true;
     uint32_t write_size;
     uint32_t current_offset = 0;
 
     // Handle potential multi-part writes if count exceeds buffer size limitations
     while (count > 0 && success) {
         // Calculate size for this chunk: max buffer size minus 2 bytes for address
         write_size = (count > (VL53L1_COMMS_CHUNK_SIZE - 2)) ? (VL53L1_COMMS_CHUNK_SIZE - 2) : count;
 
         // Allocate temporary buffer: 2 bytes for address + data chunk
         uint8_t *buffer = (uint8_t *)malloc(write_size + 2);
         if (!buffer) {
             // hal.console->printf("VL53L1X: Malloc failed in WriteMulti\n"); // Optional debug
             return VL53L1_ERROR_PLATFORM_SPECIFIC_START; // Indicate memory allocation error
         }
 
         // Prepare buffer: [ADDR_MSB] [ADDR_LSB] [DATA_0] ... [DATA_N]
         uint16_t current_index = index + current_offset;
         buffer[0] = (uint8_t)(current_index >> 8);   // Address MSB
         buffer[1] = (uint8_t)(current_index & 0xFF); // Address LSB
         memcpy(buffer + 2, pdata + current_offset, write_size);
 
         // Use ArduPilot I2C transfer (write only)
         if (!i2c_dev->transfer(buffer, write_size + 2, nullptr, 0)) {
             // hal.console->printf("VL53L1X: I2C write failed at index 0x%X\n", current_index); // DEBUG
             success = false;
         }
 
         free(buffer);
         count -= write_size;
         current_offset += write_size;
     }
 
     return success ? VL53L1_ERROR_NONE : VL53L1_ERROR_CONTROL_INTERFACE;
 }
 
 
 VL53L1_Error VL53L1_ReadMulti(VL53L1_DEV Dev, uint16_t index, uint8_t *pdata, uint32_t count)
 {
     AP_HAL::I2CDevice *i2c_dev = STDEV_GET_APHAL_DEV(Dev);
     if (!i2c_dev) {
         return VL53L1_ERROR_INVALID_PARAMS;
     }
     if (count > VL53L1_COMMS_CHUNK_SIZE) {
         // hal.console->printf("VL53L1X: ReadMulti count %lu exceeds chunk size %d\n", count, VL53L1_COMMS_CHUNK_SIZE);
         // return VL53L1_ERROR_INVALID_PARAMS; // Or handle chunking
     }
 
     // VL53L1X requires 16-bit register address write before read
     uint8_t reg_addr_bytes[2] = { (uint8_t)(index >> 8), (uint8_t)(index & 0xFF) };
 
     // Use ArduPilot I2C transfer (write address, then read data)
     bool success = i2c_dev->transfer(reg_addr_bytes, 2, pdata, count);
 
     // if (!success) {
     //     hal.console->printf("VL53L1X: I2C read failed at index 0x%X\n", index); // DEBUG
     // }
 
     return success ? VL53L1_ERROR_NONE : VL53L1_ERROR_CONTROL_INTERFACE;
 }
 
 
 VL53L1_Error VL53L1_WrByte(VL53L1_DEV Dev, uint16_t index, uint8_t data)
 {
     return VL53L1_WriteMulti(Dev, index, &data, 1);
 }
 
 
 VL53L1_Error VL53L1_WrWord(VL53L1_DEV Dev, uint16_t index, uint16_t data)
 {
     uint8_t buffer[2];
 
     buffer[0] = (uint8_t)(data >> 8);   // MSB First
     buffer[1] = (uint8_t)(data & 0x00FF);
     return VL53L1_WriteMulti(Dev, index, buffer, 2);
 }
 
 
 VL53L1_Error VL53L1_WrDWord(VL53L1_DEV Dev, uint16_t index, uint32_t data)
 {
     uint8_t buffer[4];
 
     buffer[0] = (uint8_t)(data >> 24); /*!< MSB first */
     buffer[1] = (uint8_t)((data & 0x00FF0000) >> 16);
     buffer[2] = (uint8_t)((data & 0x0000FF00) >> 8);
     buffer[3] = (uint8_t)(data & 0x000000FF); /*!< LSB last */
     return VL53L1_WriteMulti(Dev, index, buffer, 4);
 }
 
 
 //VL53L1_Error VL53L1_RdByte(VL53L1_DEV Dev, uint16_t index, uint8_t *pdata)
 //{
 //    return VL53L1_ReadMulti(Dev, index, pdata, 1);
 //}

 VL53L1_Error VL53L1_RdByte(VL53L1_DEV Dev, uint16_t index, uint8_t *pdata)
 {
     AP_HAL::I2CDevice *i2c_dev = STDEV_GET_APHAL_DEV(Dev);
     if (!i2c_dev) {
         // hal.console->printf("VL53L1X: RdByte - No I2C device handle!\n");   // DEBUG
         return VL53L1_ERROR_INVALID_PARAMS;
     }

     uint8_t reg_addr_bytes[2] = { (uint8_t)(index >> 8), (uint8_t)(index & 0xFF) };
     bool success = i2c_dev->transfer(reg_addr_bytes, 2, pdata, 1);

     // Add debug print here?
     if (index == 0x00E5) { // Register polled during boot
       hal.console->printf("VL53L1X: RdByte(0x%04X) -> success=%d, data=0x%02X\n", index, (int)success, (unsigned)*pdata);
     } else if (!success) {
       hal.console->printf("VL53L1X: RdByte(0x%04X) -> FAILED\n", index);
     }
     // End debug print?

     return success ? VL53L1_ERROR_NONE : VL53L1_ERROR_CONTROL_INTERFACE;
 }
 
 
 VL53L1_Error VL53L1_RdWord(VL53L1_DEV Dev, uint16_t index, uint16_t *pdata)
 {
     uint8_t buffer[2];
     VL53L1_Error status = VL53L1_ReadMulti(Dev, index, buffer, 2);
 
     if (status == VL53L1_ERROR_NONE) {
         // Recast to uint16_t is safe due to size and standard layout
         *pdata = (uint16_t)(((uint16_t)buffer[0] << 8) | (uint16_t)buffer[1]);
     }
     return status;
 }
 
 
 VL53L1_Error VL53L1_RdDWord(VL53L1_DEV Dev, uint16_t index, uint32_t *pdata)
 {
     uint8_t buffer[4];
     VL53L1_Error status = VL53L1_ReadMulti(Dev, index, buffer, 4);
 
     if (status == VL53L1_ERROR_NONE) {
         // Recast to uint32_t is safe due to size and standard layout
         *pdata = ((uint32_t)buffer[0] << 24) | ((uint32_t)buffer[1] << 16) | ((uint32_t)buffer[2] << 8) | (uint32_t)buffer[3];
     }
     return status;
 }
 
 
 VL53L1_Error VL53L1_WaitUs(VL53L1_DEV Dev, int32_t wait_us)
 {
     // Parameter Dev is unused for ArduPilot implementation but part of ST API signature
     (void)Dev; // Mark as unused to avoid compiler warnings
 
     if (wait_us < 0) {
         return VL53L1_ERROR_INVALID_PARAMS; // Cannot wait for negative time
     }
     hal.scheduler->delay_microseconds((uint32_t)wait_us);
     return VL53L1_ERROR_NONE;
 }
 
 
 VL53L1_Error VL53L1_WaitMs(VL53L1_DEV Dev, int32_t wait_ms)
 {
     // Parameter Dev is unused for ArduPilot implementation
     (void)Dev;
 
     if (wait_ms < 0) {
          return VL53L1_ERROR_INVALID_PARAMS;
     }
     hal.scheduler->delay((uint32_t)wait_ms);
     return VL53L1_ERROR_NONE;
 }

 VL53L1_Error VL53L1_WaitValueMaskEx(
     VL53L1_DEV    Dev,
     uint32_t      timeout_ms,
     uint16_t      index,
     uint8_t       value,
     uint8_t       mask,
     uint32_t      poll_delay_ms)
 {
     VL53L1_Error status = VL53L1_ERROR_NONE;
     uint32_t start_time_ms = AP_HAL_millis(); // Get current time from ArduPilot HAL
     uint32_t current_time_ms = 0;
     uint8_t byte_value = 0;
     uint8_t new_value = 0;
     uint8_t done = 0;

     // Ensure poll_delay_ms is not less than 1ms to prevent busy-waiting with 0 delay
     if (poll_delay_ms < 1) {
        poll_delay_ms = 1;
     }

     do {
        // Read the byte
        status = VL53L1_RdByte(Dev, index, &byte_value);
        if (status != VL53L1_ERROR_NONE) {
            // Propagate I2C read error
            goto timeout_error;
        }

        // Check the masked value
        new_value = byte_value & mask;
        if (new_value == value) {
            done = 1; // Value matches, exit loop
            break;
        }

        // Check for timeout
        current_time_ms = AP_HAL_millis();
        if (timeout_ms > 0 && (current_time_ms - start_time_ms) > timeout_ms) {
            status = VL53L1_ERROR_TIME_OUT;
            goto timeout_error;
        }

        // Wait specified delay
        if (poll_delay_ms > 0) {
             VL53L1_WaitMs(Dev, poll_delay_ms);
        }

     } while (!done);

 timeout_error: // Label for potential error exit
    // Log timeout or read errors
    // if (status == VL53L1_ERROR_TIME_OUT) {
    //    hal.console->printf("VL53L1X: WaitValueMaskEx timeout index=0x%X mask=0x%X value=0x%X\n", index, mask, value);
    // }

     return status;
 }

 extern "C" {

    // C wrapper that pure‑C code can call
    uint32_t AP_HAL_millis(void)
    {
        return (uint32_t)AP_HAL::millis();
    }
        
 } // extern "C"
    
 // uint32_t VL53L