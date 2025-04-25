#ifndef _VL53L1_PLATFORM_ARDUPILOT_H_
#define _VL53L1_PLATFORM_ARDUPILOT_H_

// Include base ST platform header first (defines prototypes & VL53L1_Dev_t etc.)
#include "vl53l1_platform.h"

// Include necessary ArduPilot HAL headers
#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/I2CDevice.h>

// Define any platform-specific configurations if needed
#include "vl53l1_platform_user_defines.h" // For VL53L1_COMMS_CHUNK_SIZE etc.


#ifdef __cplusplus
extern "C" {
#endif

/*
 * The functions below are implementations of the prototypes defined in vl53l1_platform.h
 * required by the ST API core code. They are implemented in
 * vl53l1_platform_ardupilot.cpp using ArduPilot HAL.
 */

VL53L1_Error VL53L1_WriteMulti(VL53L1_DEV Dev, uint16_t index, uint8_t *pdata, uint32_t count);
VL53L1_Error VL53L1_ReadMulti(VL53L1_DEV Dev, uint16_t index, uint8_t *pdata, uint32_t count);
VL53L1_Error VL53L1_WrByte(VL53L1_DEV Dev, uint16_t index, uint8_t data);
VL53L1_Error VL53L1_WrWord(VL53L1_DEV Dev, uint16_t index, uint16_t data);
VL53L1_Error VL53L1_WrDWord(VL53L1_DEV Dev, uint16_t index, uint32_t data);
VL53L1_Error VL53L1_RdByte(VL53L1_DEV Dev, uint16_t index, uint8_t *pdata);
VL53L1_Error VL53L1_RdWord(VL53L1_DEV Dev, uint16_t index, uint16_t *pdata);
VL53L1_Error VL53L1_RdDWord(VL53L1_DEV Dev, uint16_t index, uint32_t *pdata);
VL53L1_Error VL53L1_WaitUs(VL53L1_DEV Dev, int32_t wait_us);
VL53L1_Error VL53L1_WaitMs(VL53L1_DEV Dev, int32_t wait_ms);


#ifdef __cplusplus
} // extern "C"
#endif

#endif // _VL53L1_PLATFORM_ARDUPILOT_H_