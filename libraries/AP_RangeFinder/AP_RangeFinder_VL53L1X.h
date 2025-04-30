#pragma once

#include "AP_RangeFinder_config.h"

#if AP_RANGEFINDER_VL53L1X_ENABLED

#include "AP_RangeFinder.h"
#include "AP_RangeFinder_Backend.h"

#include <AP_HAL/I2CDevice.h>
#include <AP_HAL/utility/sparse-endian.h>

// --- Include ST API Headers ---
// Wrap C headers in extern "C" when included from C++
extern "C" {
  //#include "vl53l1_platform.h" // Should include platform specifics like VL53L1_DevData_t
  //#include "vl53l1_types.h"
  //#include "vl53l1_error_codes.h"
  //#include "vl53l1_ll_device.h" // Provides VL53L1_LLDriverData_t definition
  //#include "vl53l1_def.h"       // Provides enums like VL53L1_DistanceModes, VL53L1_RangeStatus
  //#include "vl53l1_api.h"       // Provides high-level API function prototypes
  #include "vl53l1x_api/platform/inc/vl53l1_platform.h"
  //#include "vl53l1x_api/core/inc/vl53l1_types.h"
  #include "vl53l1x_api/core/inc/vl53l1_error_codes.h"
  #include "vl53l1x_api/core/inc/vl53l1_ll_device.h"
  #include "vl53l1x_api/core/inc/vl53l1_def.h"
  //#include "vl53l1x_api/core/inc/vl53l1_api_core.h" // <-- ADD THIS LINE
  #include "vl53l1x_api/core/inc/vl53l1_api.h"
}
// --- End ST API Headers ---


class AP_RangeFinder_VL53L1X : public AP_RangeFinder_Backend
{

public:
    // Constructor
    AP_RangeFinder_VL53L1X(RangeFinder::RangeFinder_State &_state, AP_RangeFinder_Params &_params, AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev);

    // Static detection function
    static AP_RangeFinder_Backend *detect(RangeFinder::RangeFinder_State &_state, AP_RangeFinder_Params &_params, AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev);

    // Update state method (called periodically)
    void update(void) override;

protected:
    // Get MAVLink sensor type
    virtual MAV_DISTANCE_SENSOR _get_mav_distance_sensor_type() const override {
        return MAV_DISTANCE_SENSOR_LASER;
    }

private:
    // ArduPilot I2C device handle
    AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev;

    /* Full ST device wrapper – contains LL driver data plus user fields */
    VL53L1_Dev_t st_dev;

    /* Convenience alias passed to ST API functions */
    VL53L1_DEV st_dev_ptr = &st_dev;

    // Initialization status flag
    bool is_initialized = false;

    // to count init retries
    uint8_t _init_retries = 0; // Initialize to 0

    // Internal initialization function
    bool init();

    // Timer function (optional, likely unused)
    void timer();

    // Define maximum retries
    static const uint8_t MAX_INIT_RETRIES = 10;

};

#endif // AP_RANGEFINDER_VL53L1X_ENABLED