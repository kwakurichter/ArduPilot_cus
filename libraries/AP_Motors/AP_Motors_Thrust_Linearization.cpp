
#include "AP_Motors_Thrust_Linearization.h"
#include "AP_Motors_Class.h"
#include <AP_BattMonitor/AP_BattMonitor.h>
#include <AP_Baro/AP_Baro.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_Vehicle/AP_Vehicle_Type.h>

#define AP_MOTORS_BATT_VOLT_FILT_HZ 0.5 // battery voltage filtered at 0.5hz

#if AP_MOTORS_THRUST_CUBIC_ENABLED
#include "AP_Motors_Thrust_Cubic.h"
static const AP_Motors_Thrust_Cubic cubic_model(AP_MOTORS_THRUST_CUBIC_PROFILE);
#endif

#if APM_BUILD_TYPE(APM_BUILD_UNKNOWN)
    // Example does not instantiate baro so cannot do density compensation
    #define AP_MOTORS_DENSITY_COMP 0
#else
    #ifndef AP_MOTORS_DENSITY_COMP
        #define AP_MOTORS_DENSITY_COMP 1
    #endif
#endif

#if APM_BUILD_TYPE(APM_BUILD_Heli)
    // defaults to no linearisation to not break users existing setups
    #define THRST_LIN_THST_EXPO_DEFAULT     0.0f   // set to 0 for linear and 1 for second order approximation
    #define THRST_LIN_SPIN_MIN_DEFAULT      0.0f   // throttle out ratio which produces the minimum thrust.  (i.e. 0 ~ 1 ) of the full throttle range
    #define THRST_LIN_SPIN_MAX_DEFAULT      1.0f   // throttle out ratio which produces the maximum thrust.  (i.e. 0 ~ 1 ) of the full throttle range
    #define THRST_LIN_BAT_VOLT_MAX_DEFAULT  0.0f    // voltage limiting max default
    #define THRST_LIN_BAT_VOLT_MIN_DEFAULT  0.0f    // voltage limiting min default (voltage dropping below this level will have no effect)
#else
    #define THRST_LIN_THST_EXPO_DEFAULT     0.65f   // set to 0 for linear and 1 for second order approximation
    #define THRST_LIN_SPIN_MIN_DEFAULT      0.15f   // throttle out ratio which produces the minimum thrust.  (i.e. 0 ~ 1 ) of the full throttle range
    #define THRST_LIN_SPIN_MAX_DEFAULT      0.95f   // throttle out ratio which produces the maximum thrust.  (i.e. 0 ~ 1 ) of the full throttle range
    #define THRST_LIN_BAT_VOLT_MAX_DEFAULT  0.0f    // voltage limiting max default
    #define THRST_LIN_BAT_VOLT_MIN_DEFAULT  0.0f    // voltage limiting min default (voltage dropping below this level will have no effect)
#endif


extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo Thrust_Linearization::var_info[] = {

    // @Param: THST_EXPO
    // @DisplayName: Thrust Curve Expo
    // @Description: motor thrust curve exponent (0.0 for linear to 1.0 for second order curve)
    // @Range: -1 1
    // @User: Standard
    AP_GROUPINFO("THST_EXPO", 1, Thrust_Linearization,  curve_expo, THRST_LIN_THST_EXPO_DEFAULT),

    // @Param: SPIN_MIN
    // @DisplayName: Motor Spin minimum
    // @Description: Point at which the thrust starts expressed as a number from 0 to 1 in the entire output range.  Should be higher than MOT_SPIN_ARM.
    // @Values: 0.0:Low, 0.15:Default, 0.3:High
    // @Range: 0.0 0.3
    // @User: Standard
    AP_GROUPINFO("SPIN_MIN", 2, Thrust_Linearization,  spin_min, THRST_LIN_SPIN_MIN_DEFAULT),

    // @Param: SPIN_MAX
    // @DisplayName: Motor Spin maximum
    // @Description: Point at which the thrust saturates expressed as a number from 0 to 1 in the entire output range
    // @Values: 0.9:Low, 0.95:Default, 1.0:High
    // @Range: 0.9 1.0
    // @User: Standard
    AP_GROUPINFO("SPIN_MAX", 3, Thrust_Linearization,  spin_max, THRST_LIN_SPIN_MAX_DEFAULT),

    // @Param: BAT_IDX
    // @DisplayName: Battery compensation index
    // @Description: Which battery monitor should be used for doing compensation
    // @Values: 0:First battery, 1:Second battery
    // @Range: 0 15
    // @User: Standard
    AP_GROUPINFO("BAT_IDX", 4, Thrust_Linearization, batt_idx, 0),

    // @Param: BAT_V_MAX
    // @DisplayName: Battery voltage compensation maximum voltage
    // @Description: Battery voltage compensation maximum voltage (voltage above this will have no additional scaling effect on thrust).  Recommend 4.2 * cell count, 0 = Disabled
    // @Range: 6 53
    // @Units: V
    // @User: Standard
    AP_GROUPINFO("BAT_V_MAX", 5, Thrust_Linearization, batt_voltage_max, THRST_LIN_BAT_VOLT_MAX_DEFAULT),

    // @Param: BAT_V_MIN
    // @DisplayName: Battery voltage compensation minimum voltage
    // @Description: Battery voltage compensation minimum voltage (voltage below this will have no additional scaling effect on thrust).  Recommend 3.3 * cell count, 0 = Disabled
    // @Range: 6 42
    // @Units: V
    // @User: Standard
    AP_GROUPINFO("BAT_V_MIN", 6, Thrust_Linearization, batt_voltage_min, THRST_LIN_BAT_VOLT_MIN_DEFAULT),

    AP_GROUPEND
};

Thrust_Linearization::Thrust_Linearization(AP_Motors& _motors) :
    lift_max(1.0),
    motors(_motors)
{
    // setup battery voltage filtering
    batt_voltage_filt.set_cutoff_frequency(AP_MOTORS_BATT_VOLT_FILT_HZ);
    batt_voltage_filt.reset(1.0);

#if AP_MOTORS_THRUST_CUBIC_ENABLED
    // Equivalent to approximately alpha=0.01 at 1 kHz, independent of loop rate.
    thrust_model.set(0);
    cubic_voltage_filt.set_cutoff_frequency(1.6f);
    cubic_voltage_filt.reset(cubic_model.reference_voltage());
    cubic_voltage_ready = false;
#endif

#if APM_BUILD_TYPE(APM_BUILD_Heli)
    AP_Param::setup_object_defaults(this, var_info);
#endif
}

// converts desired thrust to linearized actuator output in a range of 0~1
float Thrust_Linearization::thrust_to_actuator(float thrust_in) const
{
    thrust_in = constrain_float(thrust_in, 0.0, 1.0);
    return spin_min + (spin_max - spin_min) * apply_thrust_curve_and_volt_scaling(thrust_in);
}

// inverse of above, tested with AP_Motors/examples/expo_inverse_test
// used to calculate equivelent motor throttle level to direct ouput, used in tailsitter transtions
float Thrust_Linearization::actuator_to_thrust(float actuator) const
{
    actuator = (actuator - spin_min) /  (spin_max - spin_min);
    return constrain_float(remove_thrust_curve_and_volt_scaling(actuator), 0.0, 1.0);
}

// apply_thrust_curve_and_volt_scaling - returns throttle in the range 0 ~ 1
float Thrust_Linearization::apply_thrust_curve_and_volt_scaling(float thrust) const
{
#if AP_MOTORS_THRUST_CUBIC_ENABLED
    if (cubic_selected() && cubic_model.valid() && spin_max > spin_min) {
        // Undo the mixer's available-thrust normalization exactly once.
        const float force = constrain_float(thrust, 0.0f, 1.0f) * lift_max * cubic_model.maximum_thrust();
        const float actuator = cubic_model.actuator(force, cubic_voltage_filt.get(), spin_min, spin_max);
        // The caller applies the spin range; the calibrated output is already absolute.
        return (actuator - spin_min) / (spin_max - spin_min);
    }
#endif
    float battery_scale = 1.0;
    if (is_positive(batt_voltage_filt.get())) {
        battery_scale = 1.0 / batt_voltage_filt.get();
    }
    // apply thrust curve - domain -1.0 to 1.0, range -1.0 to 1.0
    float thrust_curve_expo = constrain_float(curve_expo, -1.0, 1.0);
    if (is_zero(thrust_curve_expo)) {
        // zero expo means linear, avoid floating point exception for small values
        return lift_max * thrust * battery_scale;
    }
    float throttle_ratio = ((thrust_curve_expo - 1.0) + safe_sqrt((1.0 - thrust_curve_expo) * (1.0 - thrust_curve_expo) + 4.0 * thrust_curve_expo * lift_max * thrust)) / (2.0 * thrust_curve_expo);
    return constrain_float(throttle_ratio * battery_scale, 0.0, 1.0);
}

// inverse of above, tested with AP_Motors/examples/expo_inverse_test
// used to calculate equivelent motor throttle level to direct ouput, used in tailsitter transtions
float Thrust_Linearization::remove_thrust_curve_and_volt_scaling(float throttle) const
{
#if AP_MOTORS_THRUST_CUBIC_ENABLED
    if (cubic_selected() && cubic_model.valid() && is_positive(lift_max)) {
        const float actuator = spin_min + constrain_float(throttle, 0.0f, 1.0f) * (spin_max - spin_min);
        return constrain_float(cubic_model.force(actuator, cubic_voltage_filt.get()) /
                               (lift_max * cubic_model.maximum_thrust()), 0.0f, 1.0f);
    }
#endif
    float battery_scale = 1.0;
    if (is_positive(batt_voltage_filt.get())) {
        battery_scale = 1.0 / batt_voltage_filt.get();
    }
    // apply thrust curve - domain -1.0 to 1.0, range -1.0 to 1.0
    float thrust_curve_expo = constrain_float(curve_expo, -1.0, 1.0);
    if (is_zero(thrust_curve_expo)) {
        // zero expo means linear, avoid floating point exception for small values
        return  throttle / (lift_max * battery_scale);
    }
    float thrust = ((throttle / battery_scale) * (2.0 * thrust_curve_expo)) - (thrust_curve_expo - 1.0);
    thrust = (thrust * thrust) - ((1.0 - thrust_curve_expo) * (1.0 - thrust_curve_expo));
    thrust /=  4.0 * thrust_curve_expo * lift_max;
    return constrain_float(thrust, 0.0, 1.0);
}

// update_lift_max from battery voltage - used for voltage compensation
void Thrust_Linearization::update_lift_max_from_batt_voltage()
{
#if AP_MOTORS_THRUST_CUBIC_ENABLED
    if (cubic_selected() && cubic_model.valid()) {
#if AP_BATTERY_ENABLED
        update_cubic_voltage(AP::battery().voltage(batt_idx), AP::battery().healthy(batt_idx));
#else
        update_cubic_voltage(0.0f, false);
#endif
        return;
    }
#endif
#if AP_BATTERY_ENABLED
    // sanity check battery_voltage_min is not too small
    // if disabled or misconfigured exit immediately
    float _batt_voltage = motors.has_option(AP_Motors::MotorOptions::BATT_RAW_VOLTAGE) ? AP::battery().voltage(batt_idx) : AP::battery().voltage_resting_estimate(batt_idx);

    if ((batt_voltage_max <= 0) || (batt_voltage_min >= batt_voltage_max) || (_batt_voltage < 0.25 * batt_voltage_min)) {
        batt_voltage_filt.reset(1.0);
        lift_max = 1.0;
        return;
    }

    batt_voltage_min.set(MAX(batt_voltage_min, batt_voltage_max * 0.6));

    // constrain resting voltage estimate (resting voltage is actual voltage with sag removed based on current draw and resistance)
    _batt_voltage = constrain_float(_batt_voltage, batt_voltage_min, batt_voltage_max);

    if (!motors.has_option(AP_Motors::MotorOptions::BATT_RAW_VOLTAGE)) {
        // filter at 0.5 Hz
        batt_voltage_filt.apply(_batt_voltage / batt_voltage_max, motors.get_dt_s());
    } else {
        // reset is equivalent to no filtering
        batt_voltage_filt.reset(_batt_voltage / batt_voltage_max);
    }

    // calculate lift max
    float thrust_curve_expo = constrain_float(curve_expo, -1.0, 1.0);
    lift_max = batt_voltage_filt.get() * (1 - thrust_curve_expo) + thrust_curve_expo * batt_voltage_filt.get() * batt_voltage_filt.get();
#endif
}

#if AP_MOTORS_THRUST_CUBIC_ENABLED
bool Thrust_Linearization::cubic_configuration_valid() const
{
    if (thrust_model == 0) {
        return true;
    }
    if (!cubic_selected() || !cubic_model.valid() || !motors.is_digital_pwm_type() ||
        !isfinite(spin_min) || !isfinite(spin_max) || is_negative(spin_min) ||
        spin_max > 1.0f || spin_max <= spin_min ||
        !isfinite(batt_voltage_min) || !isfinite(batt_voltage_max) ||
        !is_positive(batt_voltage_min) || batt_voltage_min >= batt_voltage_max ||
        batt_voltage_max > cubic_model.reference_voltage()) {
        return false;
    }
    // Keep the entire running range inside the measured portion of the curve.
    if (cubic_model.force(spin_min, batt_voltage_min) < cubic_model.minimum_thrust() ||
        cubic_model.force(spin_min, batt_voltage_max) >= cubic_model.maximum_thrust()) {
        return false;
    }
#if AP_BATTERY_ENABLED
    const float voltage = AP::battery().voltage(batt_idx);
    return AP::battery().healthy(batt_idx) && isfinite(voltage) && voltage >= batt_voltage_min &&
           voltage <= 1.25f * cubic_model.reference_voltage();
#else
    return false;
#endif
}

void Thrust_Linearization::update_cubic_voltage(float voltage, bool healthy)
{
    if (healthy && isfinite(voltage) &&
        voltage >= 0.5f * cubic_model.reference_voltage() &&
        voltage <= 1.25f * cubic_model.reference_voltage() &&
        is_positive(batt_voltage_min) && batt_voltage_min < batt_voltage_max &&
        batt_voltage_max <= cubic_model.reference_voltage()) {
        const float bounded_voltage = constrain_float(voltage, batt_voltage_min, batt_voltage_max);
        if (!cubic_voltage_ready) {
            cubic_voltage_filt.reset(bounded_voltage);
            cubic_voltage_ready = true;
        } else {
            cubic_voltage_filt.apply(bounded_voltage, motors.get_dt_s());
        }
    }
    // Retain the last usable voltage during telemetry loss; battery failsafes
    // remain responsible for the vehicle's response, without switching models.
    lift_max = cubic_model.available_thrust(cubic_voltage_filt.get(), spin_max) / cubic_model.maximum_thrust();
}
#endif // AP_MOTORS_THRUST_CUBIC_ENABLED

// return gain scheduling gain based on voltage and air density
float Thrust_Linearization::get_compensation_gain() const
{
    // avoid divide by zero
    if (get_lift_max() <= 0.0) {
        return 1.0;
    }

    float ret = 1.0 / get_lift_max();

#if AP_MOTORS_DENSITY_COMP == 1
    // air density ratio is increasing in density / decreasing in altitude
    const float air_density_ratio = AP::ahrs().get_air_density_ratio();
    if (air_density_ratio > 0.3 && air_density_ratio < 1.5) {
        ret *= 1.0 / constrain_float(air_density_ratio, 0.5, 1.25);
    }
#endif
    return ret;
}
