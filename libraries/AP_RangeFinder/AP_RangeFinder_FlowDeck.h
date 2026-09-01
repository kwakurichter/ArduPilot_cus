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

    /*
      Runs on the I2C bus thread. The ST API talks to the sensor over several
      multi-register transfers per sample, which is milliseconds of blocking
      I2C - far past the 100us the read_rangefinder scheduler slot allows, so
      it must not happen on the main thread.
     */
    void timer();

    // one I2C read cycle; called only from timer()
    void sample();

    // sample handed from timer() to update(), guarded by the backend semaphore
    struct PendingSample {
        float distance_m;
        int8_t quality_pct;
        uint32_t time_ms;
        bool valid;
    } _pending_sample {};

    bool _new_sample = false;
    bool _sensor_lost = false;      // timer() gave up; update() reports it

    // inter-measurement period, chosen from the distance mode in init()
    uint32_t _measurement_period_ms = 0;

};

#endif // AP_RANGEFINDER_FLOWDECK_ENABLED
