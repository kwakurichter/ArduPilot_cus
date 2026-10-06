#pragma once

#include <AP_Math/AP_Math.h>

// Calibrated force (N) as a polynomial of effective motor voltage (V).
// The output mapping is actuator = scale * motor_voltage / battery_voltage + offset.
class AP_Motors_Thrust_Cubic {
public:
    struct Profile {
        float c0, c1, c2, c3;
        float thrust_min;
        float thrust_max;
        float voltage_max;
        float scale;
        float offset;
    };

    explicit constexpr AP_Motors_Thrust_Cubic(const Profile &profile) : _profile(profile) {}

    bool valid() const
    {
        // Non-negative derivative coefficients give a unique inverse over V >= 0.
        return isfinite(_profile.c0) && isfinite(_profile.c1) && isfinite(_profile.c2) && isfinite(_profile.c3) &&
               is_positive(_profile.c1) && !is_negative(_profile.c2) && !is_negative(_profile.c3) &&
               isfinite(_profile.thrust_min) && isfinite(_profile.thrust_max) &&
               is_positive(_profile.thrust_min) && _profile.thrust_max > _profile.thrust_min &&
               isfinite(_profile.voltage_max) && is_positive(_profile.voltage_max) &&
               isfinite(_profile.scale) && is_positive(_profile.scale) &&
               isfinite(_profile.offset) && !is_positive(_profile.offset);
    }

    float force(float actuator, float battery_voltage) const
    {
        const float voltage = MAX(0.0f, (actuator - _profile.offset) / _profile.scale * battery_voltage);
        return MAX(0.0f, _profile.c0 + voltage * (_profile.c1 + voltage * (_profile.c2 + voltage * _profile.c3)));
    }

    float actuator(float thrust, float battery_voltage, float spin_min, float spin_max) const
    {
        if (!isfinite(thrust) || !isfinite(battery_voltage) || !is_positive(battery_voltage)) {
            return spin_min;
        }
        if (thrust <= force(spin_min, battery_voltage)) {
            return spin_min;
        }
        if (thrust >= force(spin_max, battery_voltage)) {
            return spin_max;
        }
        // Fixed work, no cube roots, and less than one DShot count of numerical error.
        float low = spin_min;
        float high = spin_max;
        for (uint8_t i = 0; i < 16; i++) {
            const float mid = 0.5f * (low + high);
            if (force(mid, battery_voltage) < thrust) {
                low = mid;
            } else {
                high = mid;
            }
        }
        return 0.5f * (low + high);
    }

    float available_thrust(float battery_voltage, float spin_max) const
    {
        return MIN(_profile.thrust_max, force(spin_max, battery_voltage));
    }

    float minimum_thrust() const { return _profile.thrust_min; }
    float maximum_thrust() const { return _profile.thrust_max; }
    float reference_voltage() const { return _profile.voltage_max; }

private:
    const Profile _profile;
};
