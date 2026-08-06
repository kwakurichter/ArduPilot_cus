#pragma once

#include "AP_RangeFinder_config.h"

#if AP_RANGEFINDER_FLOWDECK_ENABLED

#include "AP_RangeFinder.h"
#include "AP_RangeFinder_Backend_I2C.h"

#include <AP_HAL/I2CDevice.h>
#include <AP_HAL/utility/sparse-endian.h>

// --- Include ST API Headers ---
extern "C" {
  #include "vl53l1x_api/platform/inc/vl53l1_platform.h"
  #include "vl53l1x_api/core/inc/vl53l1_error_codes.h"
  #include "vl53l1x_api/core/inc/vl53l1_ll_device.h"
  #include "vl53l1x_api/core/inc/vl53l1_def.h"
  #include "vl53l1x_api/core/inc/vl53l1_api.h"
}


class AP_RangeFinder_FlowDeck : public AP_RangeFinder_Backend_I2C
{

public:
    // Static detection function
    static AP_RangeFinder_Backend *detect(RangeFinder::RangeFinder_State &_state,
                                          AP_RangeFinder_Params &_params,
                                          class AP_HAL::I2CDevice &dev) {
        // this will free the object if configuration fails:
        return configure(NEW_NOTHROW AP_RangeFinder_FlowDeck(_state, _params, dev));
    }

    // Update state method
    void update(void) override;

protected:
    // Get MAVLink sensor type
    virtual MAV_DISTANCE_SENSOR _get_mav_distance_sensor_type() const override {
        return MAV_DISTANCE_SENSOR_LASER;
    }

private:
    // constructor; `dev` is owned by the base class
    using AP_RangeFinder_Backend_I2C::AP_RangeFinder_Backend_I2C;

    /* Full ST device wrapper – contains LL driver data plus user fields */
    VL53L1_Dev_t st_dev;

    /* Convenience alias passed to ST API functions */
    VL53L1_DEV st_dev_ptr = &st_dev;

    bool is_initialized = false;    // Initialization status flag

    bool init() override;

    void timer();

};

#endif // AP_RANGEFINDER_FLOWDECK_ENABLED