#include "AP_RangeFinder_VL53L1X.h"

#if AP_RANGEFINDER_VL53L1X_ENABLED

#include <utility>
#include <AP_HAL/AP_HAL.h>
#include <AP_Logger/AP_Logger.h>
#include <stdio.h> // For snprintf if logging details

extern const AP_HAL::HAL& hal;

// --- Configuration Constants (Based on Crazyflie zranger2.c) ---
// Define desired configuration constants matching Crazyflie's setup
#define VL53L1X_DISTANCE_MODE       VL53L1_DISTANCEMODE_MEDIUM  // Medium range mode
#define VL53L1X_TIMING_BUDGET_US    25000                       // 25ms timing budget
//#define VL53L1X_INTER_MEASUREMENT_MS VL53L1X_TIMING_BUDGET_US/1000 // Match timing budget for back-to-back reads (~40Hz)
#define VL53L1X_INTER_MEASUREMENT_MS (VL53L1X_TIMING_BUDGET_US/1000 + 5) // DEBUG --increase timng budget
// Timeout for waiting for data ready after triggering
#define DATA_READY_TIMEOUT_MS 100
// Timeout for considering sensor disconnected if no valid readings
#define SENSOR_TIMEOUT_MS 500


extern "C" void VL53L1_set_aphal_device(AP_HAL::I2CDevice*);

// Constructor
AP_RangeFinder_VL53L1X::AP_RangeFinder_VL53L1X(RangeFinder::RangeFinder_State &_state_in, AP_RangeFinder_Params &_params_in, AP_HAL::OwnPtr<AP_HAL::I2CDevice> _dev)
    : AP_RangeFinder_Backend(_state_in, _params_in), dev(std::move(_dev))
{
    // Zero initialize the ST LL driver data structure
    memset(&st_dev, 0, sizeof(st_dev));

    // Setup the platform data within the ST device structure
    VL53L1_set_aphal_device(dev.get());     // links generic ST API to ArduPilot HAL

    // Setup i2c
    st_dev_ptr->i2c_slave_address = _params_in.address.get();

    st_dev_ptr->comms_type = 1; // 1 = I2C

    st_dev_ptr->comms_speed_khz = 400; // Default I2C speed
}

/* Static detect function */
AP_RangeFinder_Backend *AP_RangeFinder_VL53L1X::detect(RangeFinder::RangeFinder_State &_state, AP_RangeFinder_Params &_params, AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev)
{
    if (!dev) {
        return nullptr;
    }
    AP_RangeFinder_VL53L1X *sensor = new AP_RangeFinder_VL53L1X(_state, _params, std::move(dev));
    if (!sensor) {
        hal.console->printf("VL53L1X: Failed to allocate sensor object\n");
        return nullptr;
    }
    if (!sensor->init()) {
        // Init function print detailed errors
        delete sensor;
        return nullptr;
    }
    hal.console->printf("VL53L1X: Detected and initialized successfully\n");
    return sensor;
}

/* Initialise sensor using ST API calls (from zranger2) */
bool AP_RangeFinder_VL53L1X::init()
{
    VL53L1_Error st_status = VL53L1_ERROR_NONE;

    WITH_SEMAPHORE(dev->get_semaphore()); // Ensure exclusive I2C access during init

    // Ensure sensor is booted
    hal.console->printf("VL53L1X: Waiting for device boot...\n");
    st_status = VL53L1_WaitDeviceBooted(st_dev_ptr);
    if (st_status != VL53L1_ERROR_NONE) {
        hal.console->printf("VL53L1X: WaitDeviceBooted failed (%d)\n", (int)st_status);
        return false;
    }

    // Data initialization
    hal.console->printf("VL53L1X: Performing DataInit...\n");
    st_status = VL53L1_DataInit(st_dev_ptr);
    if (st_status != VL53L1_ERROR_NONE) {
        hal.console->printf("VL53L1X: DataInit failed (%d)\n", (int)st_status);
        return false;
    }

    // Static initialization (applies base config)
    hal.console->printf("VL53L1X: Performing StaticInit...\n");
    st_status = VL53L1_StaticInit(st_dev_ptr);
    if (st_status != VL53L1_ERROR_NONE) {
        hal.console->printf("VL53L1X: StaticInit failed (%d)\n", (int)st_status);
        return false;
    }

    // --- Configure Ranging Parameters ---
    VL53L1_DistanceModes mode_to_set;
    // Check the type parameter associated with this instnce
    RangeFinder::Type rf_type = (RangeFinder::Type)params.type.get();

    if (rf_type == RangeFinder::Type::VL53L1X_Short) {
        mode_to_set = VL53L1_DISTANCEMODE_SHORT;
        hal.console->printf("VL53L1X: Setting Distance Mode to Short...\n");
    } else {
        // Default to Medium for now
        // Should change in future so can be selected in QGC (e.g., RNGFNDx_MODE)
        mode_to_set = VL53L1_DISTANCEMODE_MEDIUM;
        hal.console->printf("VL53L1X: Setting Distance Mode to Medium (Default)...\n");
    }

    st_status = VL53L1_SetDistanceMode(st_dev_ptr, mode_to_set);
    if (st_status != VL53L1_ERROR_NONE) {
        hal.console->printf("VL53L1X: SetDistanceMode failed (%d)\n", (int)st_status);
        return false;
    }

    hal.console->printf("VL53L1X: Setting Timing Budget to %u us...\n", (unsigned)VL53L1X_TIMING_BUDGET_US);
    st_status = VL53L1_SetMeasurementTimingBudgetMicroSeconds(st_dev_ptr, VL53L1X_TIMING_BUDGET_US); // 25ms
    if (st_status != VL53L1_ERROR_NONE) {
        hal.console->printf("VL53L1X: SetMeasurementTimingBudgetMicroSeconds failed (%d)\n", (int)st_status);
        return false;
    }

    // Set inter-measurement period (match timing budget for continuous mode)
    hal.console->printf("VL53L1X: Setting Inter-Measurement Period to %u ms...\n", (unsigned)VL53L1X_INTER_MEASUREMENT_MS);
    st_status = VL53L1_SetInterMeasurementPeriodMilliSeconds(st_dev_ptr, VL53L1X_INTER_MEASUREMENT_MS);
     if (st_status != VL53L1_ERROR_NONE) {
        hal.console->printf("VL53L1X: SetInterMeasurementPeriodMilliSeconds failed (%d)\n", (int)st_status);
        return false;
    }

    // --- Start Measurement ---
    hal.console->printf("VL53L1X: Starting Measurement...\n");
    st_status = VL53L1_StartMeasurement(st_dev_ptr);
    if (st_status != VL53L1_ERROR_NONE) {
        hal.console->printf("VL53L1X: StartMeasurement failed (%d)\n", (int)st_status);
        return false;
    }

    is_initialized = true;
    set_status(RangeFinder::Status::Good); // Set initial status
    hal.console->printf("VL53L1X: Initialization complete.\n");
    return true;
}

/* Update state (called periodically to read measurements) */
void AP_RangeFinder_VL53L1X::update(void)
{
    if (!is_initialized) {
        // Attempt re-initialization periodically if desired, up to a maximum count
        static uint32_t last_init_attempt_ms = 0;
    
        // Check if status indicates a need for re-init, enough time has passed and we haven't exceeded the maximum retry count
        if ((state.status == RangeFinder::Status::NotConnected || state.status == RangeFinder::Status::NoData) &&
            (_init_retries < MAX_INIT_RETRIES) &&
            (AP_HAL::millis() - last_init_attempt_ms > 5000)) { // Retry every 5s
    
            last_init_attempt_ms = AP_HAL::millis();
            _init_retries++; // <<< Increment retry counter before attempting
    
            hal.console->printf("VL53L1X: Re-attempting initialization (%u/%u)...\n", (unsigned)_init_retries, (unsigned)MAX_INIT_RETRIES); // Updated log
    
            // Clear potential stale status before retrying init
            set_status(RangeFinder::Status::NotConnected);
            init(); // Attempt to re-initialize
    
            // If init() succeeded, is_initialized will be true, and this block won't run next time.
            // If init() failed, is_initialized remains false, and we might retry later if count < max.
    
        } else if (_init_retries >= MAX_INIT_RETRIES && state.status != RangeFinder::Status::NotConnected) {
             // Once max retries are hit, ensure status stays NotConnected
             // This prevents it flip-flopping if it was temporarily NoData.
             set_status(RangeFinder::Status::NotConnected);
        }
    
        // If not initialized (either failed init or max retries reached), do nothing more in update
        return;
    }
    

    VL53L1_Error st_status = VL53L1_ERROR_NONE;
    uint8_t data_ready = 0;
    VL53L1_RangingMeasurementData_t measurement_data;
    bool read_ok = false;
    //uint32_t check_start_ms = AP_HAL::millis();

    // Non-blocking check if data is ready
    {
        WITH_SEMAPHORE(dev->get_semaphore());
        st_status = VL53L1_GetMeasurementDataReady(st_dev_ptr, &data_ready);
    }

    // Handle I2C error during check
    if (st_status != VL53L1_ERROR_NONE) {
        // Log periodically
        static uint32_t last_comm_fail_ms = 0;
        if (AP_HAL::millis() - last_comm_fail_ms > 2000) {
            hal.console->printf("VL53L1X: GetMeasurementDataReady failed (%d)\n", (int)st_status);
            last_comm_fail_ms = AP_HAL::millis();
        }
        // If communication fails consistently, sensor is likely disconnected
        if (AP_HAL::millis() - state.last_reading_ms > SENSOR_TIMEOUT_MS * 2) {
             set_status(RangeFinder::Status::NotConnected);
             is_initialized = false; // Force re-init attempt next time
        } else {
            set_status(RangeFinder::Status::NoData);    //error
        }
        return;
    }

    if (!data_ready) {
        // No new data yet. Check for timeout if we previously had good readings
        if ((state.status == RangeFinder::Status::Good) &&
            (AP_HAL::millis() - state.last_reading_ms > SENSOR_TIMEOUT_MS)) {
            set_status(RangeFinder::Status::NoData);
            state.signal_quality_pct = 0;
        }
        return; // No new data
    }

    // Data is ready, know get the measurement data
    {
        WITH_SEMAPHORE(dev->get_semaphore());
        st_status = VL53L1_GetRangingMeasurementData(st_dev_ptr, &measurement_data);

        if (st_status == VL53L1_ERROR_NONE) {
            // Clear interrupt and start next measurement immediately after successful read
            // Note: Crazyflie uses Stop/Start, but ClearInterrupt should work for continuous mode
            st_status = VL53L1_ClearInterruptAndStartMeasurement(st_dev_ptr);
            if (st_status == VL53L1_ERROR_NONE) {
                read_ok = true; // Mark as successful read and trigger
            } else {
                hal.console->printf("VL53L1X: ClearInterruptAndStartMeasurement failed (%d)\n", (int)st_status);
                // if failed, continue processing the data, but flag init state
                 is_initialized = false;
                 // Use NoData to indicate a problem preventing valid data flow
                 //set_status(RangeFinder::Status::Error);
                 set_status(RangeFinder::Status::NoData);
            }
        } else {
             hal.console->printf("VL53L1X: GetRangingMeasurementData failed (%d)\n", (int)st_status);
             // Attempt to clear interrupt anyway to potentially recover state
             VL53L1_ClearInterruptAndStartMeasurement(st_dev_ptr);
        }
    } // Semaphore released


    if (read_ok) {
        // Process valid measurement data
        // Check RangeStatus based on ST API definitions (vl53l1_def.h)
        if (measurement_data.RangeStatus == VL53L1_RANGESTATUS_RANGE_VALID ||
            measurement_data.RangeStatus == VL53L1_RANGESTATUS_RANGE_VALID_MIN_RANGE_CLIPPED ||
            measurement_data.RangeStatus == VL53L1_RANGESTATUS_RANGE_VALID_NO_WRAP_CHECK_FAIL)
        {
            state.distance_m = measurement_data.RangeMilliMeter * 0.001f;
            state.last_reading_ms = AP_HAL::millis();

            // Calculate signal quality
            // Check vl53l1_def.h or API src for SigmaMilliMeter format
            const float min_sigma_mm = 5.0f;  // Lower sigma = better quality
            const float max_sigma_mm = 50.0f;
            float sigma_mm = (float)measurement_data.SigmaMilliMeter;

            if (measurement_data.SigmaMilliMeter == 0) { // Check for zero sigma
                state.signal_quality_pct = RangeFinder::SIGNAL_QUALITY_UNKNOWN; // Or 100?
            } else if (sigma_mm < min_sigma_mm) {
                state.signal_quality_pct = 100;
            } else if (sigma_mm > max_sigma_mm) {
                 state.signal_quality_pct = 0;
            } else {
                state.signal_quality_pct = 100 * (1.0f - (sigma_mm - min_sigma_mm) / (max_sigma_mm - min_sigma_mm));
            }
            state.signal_quality_pct = constrain_int16(state.signal_quality_pct, 0, 100);

            // Update ArduPilot status based on distance and limits
            update_status();

            // If status wasbad, mark as Good now
            if (state.status != RangeFinder::Status::Good) {
                 set_status(RangeFinder::Status::Good);
            }

        } else {
            // Measurement reported an error status by the sensor
             static uint32_t last_err_log_ms = 0;
             if (AP_HAL::millis() - last_err_log_ms > 2000) {
                  hal.console->printf("VL53L1X: Invalid measurement status: %u\n", measurement_data.RangeStatus);
                  last_err_log_ms = AP_HAL::millis();
             }
            set_status(RangeFinder::Status::NoData); // Report NoData for transient sensor errors
            state.signal_quality_pct = 0;
        }

    } else {
        // Read failed or ClearInterrupt failed
        if (state.status == RangeFinder::Status::Good || state.status == RangeFinder::Status::NoData) {
            // If we were previously working, mark as NoData or Error
            if (AP_HAL::millis() - state.last_reading_ms > SENSOR_TIMEOUT_MS) {
                 set_status(RangeFinder::Status::NoData);
                 state.signal_quality_pct = 0;
            }
        }
        // If ClearInterrupt failed, is_initialized is false, will likely become NotConnected soon
    }
}

/* Timer function - called periodically */
void AP_RangeFinder_VL53L1X::timer(void)
{
    // empty
}

#endif // AP_RANGEFINDER_VL53L1X_ENABLED