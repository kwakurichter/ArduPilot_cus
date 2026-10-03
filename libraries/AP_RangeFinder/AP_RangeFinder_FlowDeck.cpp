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

    // Setup the platform data within the ST device structure (links the generic ST API to ArduPilot HAL implementations)
    VL53L1_set_aphal_device(&dev);

    // Replace st_ll_data accesses:
    st_dev_ptr->i2c_slave_address = params.address.get();

    st_dev_ptr->comms_type = 1; // 1 = I2C

    st_dev_ptr->comms_speed_khz = 400; // Default I2C speed

    WITH_SEMAPHORE(dev.get_semaphore()); // Ensure exclusive I2C access during init

    // -- Ensure sensor is booted --

    st_status = VL53L1_WaitDeviceBooted(st_dev_ptr);
    if (st_status != VL53L1_ERROR_NONE) {
        gcs().send_text(MAV_SEVERITY_ALERT, "VL53L1X: WaitDeviceBooted failed (%d)\n", (int)st_status); //DEBUG
        return false;
    }

    // -- Data initialization --

    st_status = VL53L1_DataInit(st_dev_ptr);
    if (st_status != VL53L1_ERROR_NONE) {
        gcs().send_text(MAV_SEVERITY_ALERT, "VL53L1X: DataInit failed (%d)\n", (int)st_status); //DEBUG
        return false;
    }

    // -- Static initialization (applies base config) --
    
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
    } else if (mode == 2) {
        mode_to_set = VL53L1_DISTANCEMODE_LONG;
        VL53L1X_TIMING_BUDGET_US = 140000;            // 140ms timing budget
    } else {
        // Default to Medium for other VL53L1X types,
        mode_to_set = VL53L1_DISTANCEMODE_MEDIUM;
        VL53L1X_TIMING_BUDGET_US = 25000;            // 25ms timing budget
    }

    VL53L1X_INTER_MEASUREMENT_MS = ((VL53L1X_TIMING_BUDGET_US / 1000) + 5); // Timing budget + min 4 ms
    _measurement_period_ms = VL53L1X_INTER_MEASUREMENT_MS;

    st_status = VL53L1_SetDistanceMode(st_dev_ptr, mode_to_set);
    if (st_status != VL53L1_ERROR_NONE) {
        gcs().send_text(MAV_SEVERITY_ALERT, "VL53L1X: SetDistanceMode failed (%d)\n", (int)st_status); //DEBUG
        return false;
    }

    st_status = VL53L1_SetMeasurementTimingBudgetMicroSeconds(st_dev_ptr, VL53L1X_TIMING_BUDGET_US); // 25ms
    if (st_status != VL53L1_ERROR_NONE) {
        gcs().send_text(MAV_SEVERITY_ALERT, "VL53L1X: SetMeasurementTimingBudgetMicroSeconds failed (%d)\n", (int)st_status); //DEBUG
        return false;
    }

    // -- Set inter-measurement period (match timing budget for continuous mode) --
    
    st_status = VL53L1_SetInterMeasurementPeriodMilliSeconds(st_dev_ptr, VL53L1X_INTER_MEASUREMENT_MS);
     if (st_status != VL53L1_ERROR_NONE) {
        gcs().send_text(MAV_SEVERITY_ALERT, "VL53L1X: SetInterMeasurementPeriodMilliSeconds failed (%d)\n", (int)st_status); //DEBUG
        return false;
    }

    // --- Start Measurement ---
    
    st_status = VL53L1_StartMeasurement(st_dev_ptr);
    if (st_status != VL53L1_ERROR_NONE) {
        gcs().send_text(MAV_SEVERITY_ALERT, "VL53L1X: StartMeasurement failed (%d)\n", (int)st_status); //DEBUG
        return false;
    }

    _last_sample_ms = AP_HAL::millis();
    _last_recovery_ms = _last_sample_ms;

    // --- Register periodic callback for sampling ---
    if (dev.register_periodic_callback(_measurement_period_ms * 1000U,
                                       FUNCTOR_BIND_MEMBER(&AP_RangeFinder_FlowDeck::timer, void)) == nullptr) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "VL53L1X: callback registration failed");
        return false;
    }

    is_initialized = true;
    set_status(RangeFinder::Status::NoData);

    return true;
}

/*
  Read one sample. Runs on the I2C bus thread via the periodic callback, so
  the several millisecond ST API transfers below stay off the main loop.
  Only the decoded result is handed to update(), under _sem.
 */
void AP_RangeFinder_FlowDeck::sample(void)
{
#if HAL_LOGGING_ENABLED && AP_RANGEFINDER_FLOWDECK_DIAGNOSTICS_ENABLED
    const uint32_t start_us = AP_HAL::micros();
    {
        WITH_SEMAPHORE(_sem);
        _diagnostics.polls++;
        _diagnostics.start_ms = AP_HAL::millis();
        _diagnostics.stage = 1;
    }
#endif
    read_sample();
    recover_if_needed();
#if HAL_LOGGING_ENABLED && AP_RANGEFINDER_FLOWDECK_DIAGNOSTICS_ENABLED
    WITH_SEMAPHORE(_sem);
    _diagnostics.done++;
    _diagnostics.stage = 0;
    _diagnostics.duration_us = AP_HAL::micros() - start_us;
    _diagnostics.max_duration_us = MAX(_diagnostics.max_duration_us, _diagnostics.duration_us);
#endif
}

void AP_RangeFinder_FlowDeck::read_sample(void)
{
    VL53L1_Error st_status = VL53L1_ERROR_NONE;
    uint8_t data_ready = 0;
    VL53L1_RangingMeasurementData_t measurement_data {};

    {
        WITH_SEMAPHORE(dev.get_semaphore());
#if HAL_LOGGING_ENABLED && AP_RANGEFINDER_FLOWDECK_DIAGNOSTICS_ENABLED
        {
            WITH_SEMAPHORE(_sem);
            _diagnostics.stage = 2;
        }
#endif
        st_status = VL53L1_GetMeasurementDataReady(st_dev_ptr, &data_ready);
#if HAL_LOGGING_ENABLED && AP_RANGEFINDER_FLOWDECK_DIAGNOSTICS_ENABLED
        {
            WITH_SEMAPHORE(_sem);
            if (st_status != VL53L1_ERROR_NONE) {
                _diagnostics.ready_errors++;
                _diagnostics.ready_error = st_status;
            } else if (!data_ready) {
                _diagnostics.waiting++;
            }
        }
#endif
        if (st_status != VL53L1_ERROR_NONE) {
            WITH_SEMAPHORE(_sem);
            _sensor_lost = true;
            return;
        }
        if (!data_ready) {
            return;     // sample still integrating
        }

#if HAL_LOGGING_ENABLED && AP_RANGEFINDER_FLOWDECK_DIAGNOSTICS_ENABLED
        {
            WITH_SEMAPHORE(_sem);
            _diagnostics.stage = 3;
        }
#endif
        st_status = VL53L1_GetRangingMeasurementData(st_dev_ptr, &measurement_data);
#if HAL_LOGGING_ENABLED && AP_RANGEFINDER_FLOWDECK_DIAGNOSTICS_ENABLED
        {
            WITH_SEMAPHORE(_sem);
            _diagnostics.stage = 4;
            if (st_status != VL53L1_ERROR_NONE) {
                _diagnostics.read_errors++;
                _diagnostics.read_error = st_status;
            } else {
                _diagnostics.have_sample = true;
                _diagnostics.sample_ms = AP_HAL::millis();
                _diagnostics.range_status = measurement_data.RangeStatus;
                _diagnostics.stream = measurement_data.StreamCount;
                _diagnostics.distance_mm = measurement_data.RangeMilliMeter;
                _diagnostics.sigma = measurement_data.SigmaMilliMeter;
                _diagnostics.signal = measurement_data.SignalRateRtnMegaCps;
                _diagnostics.ambient = measurement_data.AmbientRateRtnMegaCps;
            }
        }
#endif
        // Re-arm after every read attempt, including failed reads, as before.
        const VL53L1_Error clear_status = VL53L1_ClearInterruptAndStartMeasurement(st_dev_ptr);
#if HAL_LOGGING_ENABLED && AP_RANGEFINDER_FLOWDECK_DIAGNOSTICS_ENABLED
        {
            WITH_SEMAPHORE(_sem);
            _diagnostics.stage = 5;
            if (clear_status != VL53L1_ERROR_NONE) {
                _diagnostics.clear_errors++;
                _diagnostics.clear_error = clear_status;
            }
        }
#endif
        if (st_status == VL53L1_ERROR_NONE) {
            st_status = clear_status;
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

    // Signal quality from the reported sigma: lower sigma is a better fix.
    const float min_sigma_mm = 5.0f;
    const float max_sigma_mm = 50.0f;
    const float sigma_mm = measurement_data.SigmaMilliMeter * (1.0f / 65536.0f);

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
#if HAL_LOGGING_ENABLED && AP_RANGEFINDER_FLOWDECK_DIAGNOSTICS_ENABLED
    if (valid) {
        _diagnostics.good++;
    } else {
        _diagnostics.bad++;
    }
#endif
    _pending_sample.distance_m = measurement_data.RangeMilliMeter * 0.001f;
    _pending_sample.quality_pct = quality;
    _last_sample_ms = AP_HAL::millis();
    _pending_sample.time_ms = _last_sample_ms;
#if HAL_LOGGING_ENABLED && AP_RANGEFINDER_FLOWDECK_DIAGNOSTICS_ENABLED
    if (_recovery_diagnostics.waiting) {
        _recovery_diagnostics.fresh++;
        _recovery_diagnostics.latency_ms = _last_sample_ms - _recovery_diagnostics.end_ms;
        _recovery_diagnostics.waiting = false;
    }
#endif
    _pending_sample.valid = valid;
    _new_sample = true;
    _sensor_lost = false;       // a completed read clears an earlier I2C failure
}

// Runs only on the bus callback thread, after the ordinary read attempt.
void AP_RangeFinder_FlowDeck::recover_if_needed()
{
    const uint32_t now_ms = AP_HAL::millis();
    if (now_ms - _last_sample_ms < RECOVERY_INTERVAL_MS ||
        now_ms - _last_recovery_ms < RECOVERY_INTERVAL_MS) {
        return;
    }

    WITH_SEMAPHORE(dev.get_semaphore());
#if HAL_LOGGING_ENABLED && AP_RANGEFINDER_FLOWDECK_DIAGNOSTICS_ENABLED
    const uint32_t start_us = AP_HAL::micros();
#endif
    {
        WITH_SEMAPHORE(_sem);
        // Do not publish a pending pre-recovery measurement as fresh.
        _new_sample = false;
#if HAL_LOGGING_ENABLED && AP_RANGEFINDER_FLOWDECK_DIAGNOSTICS_ENABLED
        _diagnostics.stage = 6;
        _recovery_diagnostics.attempts++;
        _recovery_diagnostics.waiting = false;
        _recovery_diagnostics.start_status = 127;
#endif
    }

    // StartMeasurement requires the ST PAL to be idle, so stop first.
    const VL53L1_Error stop_status = VL53L1_StopMeasurement(st_dev_ptr);
    VL53L1_Error start_status = VL53L1_ERROR_NONE;
#if HAL_LOGGING_ENABLED && AP_RANGEFINDER_FLOWDECK_DIAGNOSTICS_ENABLED
    {
        WITH_SEMAPHORE(_sem);
        _recovery_diagnostics.stop_status = stop_status;
        if (stop_status != VL53L1_ERROR_NONE) {
            _recovery_diagnostics.stop_errors++;
        } else {
            _diagnostics.stage = 7;
        }
    }
#endif
    if (stop_status == VL53L1_ERROR_NONE) {
        start_status = VL53L1_StartMeasurement(st_dev_ptr);
    }
    _last_recovery_ms = AP_HAL::millis();

    WITH_SEMAPHORE(_sem);
    _sensor_lost = stop_status != VL53L1_ERROR_NONE || start_status != VL53L1_ERROR_NONE;
    // Neither timestamp for measurement freshness is advanced by a restart.
    // update() continues to report stale data until a new sample is received.
#if HAL_LOGGING_ENABLED && AP_RANGEFINDER_FLOWDECK_DIAGNOSTICS_ENABLED
    _recovery_diagnostics.completed++;
    _recovery_diagnostics.end_ms = _last_recovery_ms;
    _recovery_diagnostics.duration_us = AP_HAL::micros() - start_us;
    if (stop_status == VL53L1_ERROR_NONE) {
        _recovery_diagnostics.start_status = start_status;
        if (start_status != VL53L1_ERROR_NONE) {
            _recovery_diagnostics.start_errors++;
        }
    }
    _recovery_diagnostics.waiting = !_sensor_lost;
#endif
}

/*
  Publish whatever the bus thread last read. Main thread (no I2C here).
 */
void AP_RangeFinder_FlowDeck::update(void)
{
    if (!is_initialized) {
        return;
    }

#if HAL_LOGGING_ENABLED && AP_RANGEFINDER_FLOWDECK_DIAGNOSTICS_ENABLED
    log_diagnostics();
#endif
    PendingSample sample {};
    bool got_sample = false;
    bool lost = false;
    {
        WITH_SEMAPHORE(_sem);
        if (_new_sample) {
            sample = _pending_sample;
            _new_sample = false;
            got_sample = true;
        }
        lost = _sensor_lost;
    }

    if (got_sample) {
        state.distance_m = sample.distance_m;
        state.last_reading_ms = sample.time_ms;
        if (sample.valid) {
            state.signal_quality_pct = sample.quality_pct;
            update_status();
        } else {
            state.signal_quality_pct = 0;
            set_status(RangeFinder::Status::NoData);
        }
        return;
    }

    const uint32_t since_ms = AP_HAL::millis() - state.last_reading_ms;

    if (lost) {
        // Stop presenting the previous range as current as soon as the bus thread reports an error. Escalate to disconnected if it persists.
        set_status(since_ms > SENSOR_TIMEOUT_MS * 2 ?
                   RangeFinder::Status::NotConnected :
                   RangeFinder::Status::NoData);
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


#if HAL_LOGGING_ENABLED && AP_RANGEFINDER_FLOWDECK_DIAGNOSTICS_ENABLED
void AP_RangeFinder_FlowDeck::log_diagnostics()
{
    if (params.flowdeck_log.get() == 0) {
        return;
    }
    if (AP_HAL::millis() - _last_diagnostics_ms < 1000U) {
        return;
    }
    Diagnostics d;
    RecoveryDiagnostics recovery;
    uint64_t now_us;
    uint32_t now_ms;
    {
        WITH_SEMAPHORE(_sem);
        d = _diagnostics;
        recovery = _recovery_diagnostics;
        now_us = AP_HAL::micros64();
        now_ms = AP_HAL::millis();
    }
    _last_diagnostics_ms = now_ms;
    // @LoggerMessage: RFD
    // @Description: FlowDeck rangefinder cumulative callback and result counters
    // @Field: TimeUS: Time since boot
    // @Field: Addr: I2C address
    // @Field: Poll: Callbacks started
    // @Field: Done: Callbacks completed
    // @Field: Wait: Successful ready checks reporting not ready
    // @Field: Good: Valid results handed to main thread
    // @Field: Bad: Invalid range-status results handed to main thread
    // @Field: RErr: Ready-check API errors
    // @Field: DErr: Measurement-read API errors
    // @Field: CErr: Clear-interrupt/restart API errors
    // @Field: Max: Maximum callback duration since boot
    AP::logger().Write("RFD", "TimeUS,Addr,Poll,Done,Wait,Good,Bad,RErr,DErr,CErr,Max",
                       "s---------s", "F---------F", "QBIIIIIIIII",
                       now_us, uint8_t(params.address.get()), d.polls, d.done, d.waiting,
                       d.good, d.bad, d.ready_errors, d.read_errors, d.clear_errors, d.max_duration_us);
    // @LoggerMessage: RFS
    // @Description: FlowDeck rangefinder callback stage, sticky API errors and last successful read (even if restart failed)
    // @Field: TimeUS: Time since boot
    // @Field: Addr: I2C address
    // @Field: St: 0 idle, 1 bus lock, 2 ready check, 3 measurement read, 4 clear/restart, 5 processing, 6 recovery stop, 7 recovery start
    // @Field: Age: Time since callback start, UINT32_MAX before first callback
    // @Field: Smp: Time since successful measurement read, UINT32_MAX before first read
    // @Field: Dur: Last completed callback duration
    // @Field: RE: Last nonzero ready-check API error, zero if none
    // @Field: DE: Last nonzero measurement-read API error, zero if none
    // @Field: CE: Last nonzero clear/restart API error, zero if none
    // @Field: RS: Last raw ST range status, 255 before first successful read
    // @Field: SC: Last ST stream count
    // @Field: Dist: Last raw distance in millimeters
    // @Field: Sig: Last raw ST sigma in 16.16 millimeters
    // @Field: SR: Last raw ST signal rate in 16.16 MCPS
    // @Field: Amb: Last raw ST ambient rate in 16.16 MCPS
    AP::logger().Write("RFS", "TimeUS,Addr,St,Age,Smp,Dur,RE,DE,CE,RS,SC,Dist,Sig,SR,Amb",
                       "s--sss---------", "F--CCF---------", "QBBIIIbbbBBhIII",
                       now_us, uint8_t(params.address.get()), d.stage,
                       d.polls == 0 ? UINT32_MAX : now_ms - d.start_ms,
                       d.have_sample ? now_ms - d.sample_ms : UINT32_MAX, d.duration_us,
                       d.ready_error, d.read_error, d.clear_error, d.range_status, d.stream,
                       d.distance_mm, d.sigma, d.signal, d.ambient);
    // @LoggerMessage: RFR
    // @Description: FlowDeck rangefinder timeout recovery attempts and fresh-result confirmation
    // @Field: TimeUS: Time since boot
    // @Field: Addr: I2C address
    // @Field: N: Cumulative stop/start recovery attempts
    // @Field: Done: Recovery attempts whose API calls have returned
    // @Field: Fresh: Attempts followed by a new measurement and successful re-arm, regardless of range validity
    // @Field: SErr: Cumulative stop API failures
    // @Field: RErr: Cumulative start API failures
    // @Field: Stop: Latest stop API result, zero also before first attempt
    // @Field: Start: Latest start API result, 127 if not attempted
    // @Field: Wait: Waiting for a fresh result after successful stop/start
    // @Field: Age: Time since latest recovery completed, UINT32_MAX before first completion
    // @Field: Lat: Time from recovery completion to first subsequent result, retained until next confirmed recovery
    // @Field: Dur: Duration of the last completed recovery
    AP::logger().Write("RFR", "TimeUS,Addr,N,Done,Fresh,SErr,RErr,Stop,Start,Wait,Age,Lat,Dur",
                       "s---------sss", "F---------CCF", "QBIIIIIbbBIII",
                       now_us, uint8_t(params.address.get()), recovery.attempts, recovery.completed, recovery.fresh,
                       recovery.stop_errors, recovery.start_errors, recovery.stop_status, recovery.start_status,
                       uint8_t(recovery.waiting), recovery.completed == 0 ? UINT32_MAX : now_ms - recovery.end_ms,
                       recovery.latency_ms, recovery.duration_us);
}
#endif // HAL_LOGGING_ENABLED && AP_RANGEFINDER_FLOWDECK_DIAGNOSTICS_ENABLED

#endif // AP_RANGEFINDER_FLOWDECK_ENABLED
