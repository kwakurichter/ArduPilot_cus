#include "AP_RangeFinder_FlowDeck.h"

#if AP_RANGEFINDER_FLOWDECK_ENABLED

#include <utility>
#include <AP_HAL/AP_HAL.h>
#include <AP_Logger/AP_Logger.h>
#include <stdio.h> // For snprintf if logging details
#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL& hal;

// --- Configuration Constants ---
#define SENSOR_TIMEOUT_MS 500   // Timeout for considering the sensor disconnected if no valid readings (milliseconds)


extern "C" void VL53L1_set_aphal_device(AP_HAL::I2CDevice*);

/* Initialise sensor using ST API calls.
   Called by AP_RangeFinder_Backend_I2C::configure(), which deletes us on failure. */
bool AP_RangeFinder_FlowDeck::init()
{
    VL53L1_Error st_status = VL53L1_ERROR_NONE;

    // Zero initialize the ST LL driver data structure
    memset(&st_dev, 0, sizeof(st_dev));

    // Setup the platform data within the ST device structure
    // This links the generic ST API to ArduPilot HAL implementations
    VL53L1_set_aphal_device(&dev);

    // Replace st_ll_data accesses:
    st_dev_ptr->i2c_slave_address = params.address.get();

    st_dev_ptr->comms_type = 1; // 1 = I2C

    st_dev_ptr->comms_speed_khz = 400; // Default I2C speed

    WITH_SEMAPHORE(dev.get_semaphore()); // Ensure exclusive I2C access during init

    // -- Ensure sensor is booted --

    //gcs().send_text(MAV_SEVERITY_DEBUG, "VL53L1X: Waiting for device boot...\n"); //DEBUG

    st_status = VL53L1_WaitDeviceBooted(st_dev_ptr);
    if (st_status != VL53L1_ERROR_NONE) {
        gcs().send_text(MAV_SEVERITY_ALERT, "VL53L1X: WaitDeviceBooted failed (%d)\n", (int)st_status); //DEBUG
        return false;
    }

    // -- Data initialization --

    //gcs().send_text(MAV_SEVERITY_DEBUG, "VL53L1X: Performing DataInit...\n"); //DEBUG

    st_status = VL53L1_DataInit(st_dev_ptr);
    if (st_status != VL53L1_ERROR_NONE) {
        gcs().send_text(MAV_SEVERITY_ALERT, "VL53L1X: DataInit failed (%d)\n", (int)st_status); //DEBUG
        return false;
    }

    // -- Static initialization (applies base config) --
    
    //gcs().send_text(MAV_SEVERITY_DEBUG, "VL53L1X: Performing StaticInit...\n"); //DEBUG
    
    st_status = VL53L1_StaticInit(st_dev_ptr);
    if (st_status != VL53L1_ERROR_NONE) {
        gcs().send_text(MAV_SEVERITY_ALERT, "VL53L1X: StaticInit failed (%d)\n", (int)st_status); //DEBUG
        return false;
    }

    // --- Configure Ranging Parameters ---
    VL53L1_DistanceModes mode_to_set;
    uint32_t VL53L1X_TIMING_BUDGET_US;
    uint32_t VL53L1X_INTER_MEASUREMENT_MS;
    
    // Check the type parameter associated *with this instance*
    uint8_t mode = params.distance_mode.get();

    if (mode == 0) {
        mode_to_set = VL53L1_DISTANCEMODE_SHORT;
        VL53L1X_TIMING_BUDGET_US = 20000;            // 20ms timing budget
       //gcs().send_text(MAV_SEVERITY_DEBUG, "VL53L1X: Setting Distance Mode to Short...\n"); //DEBUG
    } else if (mode == 2) {
        mode_to_set = VL53L1_DISTANCEMODE_LONG;
        VL53L1X_TIMING_BUDGET_US = 140000;            // 140ms timing budget
        //gcs().send_text(MAV_SEVERITY_DEBUG, "VL53L1X: Setting Distance Mode to Long...\n"); //DEBUG    
    } else {
        // Default to Medium for other VL53L1X types,
        mode_to_set = VL53L1_DISTANCEMODE_MEDIUM;
        VL53L1X_TIMING_BUDGET_US = 25000;            // 25ms timing budget
        //gcs().send_text(MAV_SEVERITY_DEBUG, "VL53L1X: Setting Distance Mode to Medium (Default)...\n"); //DEBUG
    }

    VL53L1X_INTER_MEASUREMENT_MS = ((VL53L1X_TIMING_BUDGET_US / 1000) + 5); // Timing budget + min 4 ms
    _measurement_period_ms = VL53L1X_INTER_MEASUREMENT_MS;

    st_status = VL53L1_SetDistanceMode(st_dev_ptr, mode_to_set);
    if (st_status != VL53L1_ERROR_NONE) {
        gcs().send_text(MAV_SEVERITY_ALERT, "VL53L1X: SetDistanceMode failed (%d)\n", (int)st_status); //DEBUG
        return false;
    }

    //gcs().send_text(MAV_SEVERITY_DEBUG, "VL53L1X: Setting Timing Budget to %u us...\n", (unsigned)VL53L1X_TIMING_BUDGET_US); //DEBUG

    st_status = VL53L1_SetMeasurementTimingBudgetMicroSeconds(st_dev_ptr, VL53L1X_TIMING_BUDGET_US); // 25ms
    if (st_status != VL53L1_ERROR_NONE) {
        gcs().send_text(MAV_SEVERITY_ALERT, "VL53L1X: SetMeasurementTimingBudgetMicroSeconds failed (%d)\n", (int)st_status); //DEBUG
        return false;
    }

    // -- Set inter-measurement period (match timing budget for continuous mode) --
    
    //gcs().send_text(MAV_SEVERITY_DEBUG, "VL53L1X: Setting Inter-Measurement Period to %u ms...\n", (unsigned)VL53L1X_INTER_MEASUREMENT_MS); //DEBUG
    
    st_status = VL53L1_SetInterMeasurementPeriodMilliSeconds(st_dev_ptr, VL53L1X_INTER_MEASUREMENT_MS);
     if (st_status != VL53L1_ERROR_NONE) {
        gcs().send_text(MAV_SEVERITY_ALERT, "VL53L1X: SetInterMeasurementPeriodMilliSeconds failed (%d)\n", (int)st_status); //DEBUG
        return false;
    }

    // --- Start Measurement ---
    
    //gcs().send_text(MAV_SEVERITY_DEBUG, "VL53L1X: Starting Measurement...\n"); //DEBUG
    
    st_status = VL53L1_StartMeasurement(st_dev_ptr);
    if (st_status != VL53L1_ERROR_NONE) {
        gcs().send_text(MAV_SEVERITY_ALERT, "VL53L1X: StartMeasurement failed (%d)\n", (int)st_status); //DEBUG
        return false;
    }

    is_initialized = true;
    set_status(RangeFinder::Status::Good); // Set initial status

    gcs().send_text(MAV_SEVERITY_DEBUG, "VL53L1X: Initialization complete.\n"); //DEBUG

    /*
      Poll on the bus thread at the sensor's own measurement cadence. Doing
      this from update() instead would put several millisecond-scale I2C
      transfers inside a 100us scheduler slot on the main thread.
     */
    dev.register_periodic_callback(_measurement_period_ms * 1000U,
                                   FUNCTOR_BIND_MEMBER(&AP_RangeFinder_FlowDeck::timer, void));

    return true;
}

/*
  Read one sample. Runs on the I2C bus thread via the periodic callback, so
  the several millisecond-scale ST API transfers below stay off the main loop.
  Only the decoded result is handed to update(), under _sem.
 */
void AP_RangeFinder_FlowDeck::sample(void)
{
    VL53L1_Error st_status = VL53L1_ERROR_NONE;
    uint8_t data_ready = 0;
    VL53L1_RangingMeasurementData_t measurement_data;

    {
        WITH_SEMAPHORE(dev.get_semaphore());
        st_status = VL53L1_GetMeasurementDataReady(st_dev_ptr, &data_ready);
        if (st_status != VL53L1_ERROR_NONE) {
            WITH_SEMAPHORE(_sem);
            _sensor_lost = true;
            return;
        }
        if (!data_ready) {
            return;     // sample still integrating
        }

        st_status = VL53L1_GetRangingMeasurementData(st_dev_ptr, &measurement_data);
        if (st_status == VL53L1_ERROR_NONE) {
            // re-arm immediately so the next integration overlaps our processing
            st_status = VL53L1_ClearInterruptAndStartMeasurement(st_dev_ptr);
        } else {
            // try to unstick the sensor even though this read failed
            VL53L1_ClearInterruptAndStartMeasurement(st_dev_ptr);
        }
        if (st_status != VL53L1_ERROR_NONE) {
            WITH_SEMAPHORE(_sem);
            _sensor_lost = true;
            return;
        }
    }

    const bool valid =
        measurement_data.RangeStatus == VL53L1_RANGESTATUS_RANGE_VALID ||
        measurement_data.RangeStatus == VL53L1_RANGESTATUS_RANGE_VALID_MIN_RANGE_CLIPPED ||
        measurement_data.RangeStatus == VL53L1_RANGESTATUS_RANGE_VALID_NO_WRAP_CHECK_FAIL;

    if (!valid) {
        return;     // sensor reported a bad sample; update() times it out
    }

    // Signal quality from the reported sigma: lower sigma is a better fix.
    const float min_sigma_mm = 5.0f;
    const float max_sigma_mm = 50.0f;
    const float sigma_mm = (float)measurement_data.SigmaMilliMeter;

    int8_t quality;
    if (measurement_data.SigmaMilliMeter == 0) {
        quality = RangeFinder::SIGNAL_QUALITY_UNKNOWN;
    } else if (sigma_mm < min_sigma_mm) {
        quality = 100;
    } else if (sigma_mm > max_sigma_mm) {
        quality = 0;
    } else {
        quality = (int8_t)constrain_int16(100 * (1.0f - (sigma_mm - min_sigma_mm) / (max_sigma_mm - min_sigma_mm)), 0, 100);
    }

    WITH_SEMAPHORE(_sem);
    _distance_m   = measurement_data.RangeMilliMeter * 0.001f;
    _quality_pct  = quality;
    _new_sample   = true;
    _sensor_lost  = false;      // a good read clears an earlier I2C failure
}

/*
  Publish whatever the bus thread last read. Main thread, no I2C here.
 */
void AP_RangeFinder_FlowDeck::update(void)
{
    if (!is_initialized) {
        return;
    }

    bool got_sample = false;
    bool lost = false;
    {
        WITH_SEMAPHORE(_sem);
        if (_new_sample) {
            state.distance_m         = _distance_m;
            state.signal_quality_pct = _quality_pct;
            state.last_reading_ms    = AP_HAL::millis();
            _new_sample = false;
            got_sample = true;
        }
        lost = _sensor_lost;
    }

    if (got_sample) {
        update_status();
        if (state.status != RangeFinder::Status::Good) {
            set_status(RangeFinder::Status::Good);
        }
        return;
    }

    const uint32_t since_ms = AP_HAL::millis() - state.last_reading_ms;

    // Sustained I2C failure means the deck is gone, not just a dropped sample.
    if (lost && since_ms > SENSOR_TIMEOUT_MS * 2) {
        set_status(RangeFinder::Status::NotConnected);
        state.signal_quality_pct = 0;
        return;
    }

    if (since_ms > SENSOR_TIMEOUT_MS) {
        set_status(RangeFinder::Status::NoData);
        state.signal_quality_pct = 0;
    }
}

void AP_RangeFinder_FlowDeck::timer(void)
{
    sample();
}

#endif // AP_RANGEFINDER_FLOWDECK_ENABLED