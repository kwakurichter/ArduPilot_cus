/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
/*
  Simulated Crazyflie aid sensors for the cooperative estimator; see SIM_CoopSensors.h.

  Every physics step [a, b] holds the truth at b constant over the step (the step is about
  1 ms, the shortest support 4 ms) and integrates it into whatever sensor windows the step
  overlaps; a window that ends inside the step is split there, like a tick that straddles an
  IMU sub-interval.
 */
#include "SIM_CoopSensors.h"

#if AP_SIM_COOP_SENSORS_ENABLED

#include "SIM_Aircraft.h"

using namespace SITL;

namespace {

// Sensor windows (us), as the thesis simulator.
constexpr uint32_t FLOW_WIDTH_US[3] {10000, 20000, 30000};
constexpr uint32_t FLOW_NO_MOTION_EVERY = 5;     // report 3 of every 5 has no motion
constexpr uint32_t FLOW_NO_MOTION_PHASE = 3;
constexpr uint8_t FLOW_QUALITY = 90;
constexpr double FLOW_COUNT_SD = 0.5;

constexpr uint32_t TOF_PERIOD_US = 25000;
constexpr uint32_t TOF_BUDGET_US = 20000;
constexpr double TOF_SD_M = 0.015;
constexpr double TOF_MIN_M = 0.04;                // VL53L1X minimum range
constexpr float TOF_MAX_TILT_COS = 0.5f;          // beyond 60 degrees: signal fail

constexpr uint32_t BARO_PERIOD_US = 40000;
constexpr uint32_t BARO_CONVERSION_US[2][2] {{2000, 6000}, {22000, 26000}};
constexpr double BARO_SD_PA = 1.0606601717798212;  // 1.5 / sqrt(2): two conversions averaged
constexpr double SCALE_HEIGHT_M = 8434.5;
constexpr float BARO_TEMPERATURE_C = 38.0f;

constexpr uint32_t MAG_PERIOD_US = 10000;
constexpr double MAG_SD_RAD = 0.008;

// Flow / ToF mount: 25 mm ahead of and 15 mm below the body origin, axes along body FRD.
const Vector3f MOUNT_FRD {0.025f, 0.0f, 0.015f};

// Truth calibration of the simulated PMW3901 (rad/count, sensor axes): the defaults of
// COOP_FLOW_C11..C22 and the thesis simulator's synthetic counts_to_angle.
constexpr double FLOW_C[2][2] {{0.001, 0.00002}, {0.0, 0.0011}};

// Thesis reference pressure (Pa) and per-agent baro bias (Pa) at time t (s).
double reference_pressure(double t_s)
{
    return 101325.0 + 0.8 * sin(0.7 * t_s);
}
double pressure_bias(int agent, double t_s)
{
    return (agent * 2 - 1) * 1.5 + 0.1 * sin(1.3 * t_s);
}

}  // namespace

const AP_Param::GroupInfo CoopSensors::var_info[] = {
    // @Param: ENABLE
    // @DisplayName: Cooperative aid sensors enable
    // @Description: Simulate the Crazyflie aid sensors (PMW3901 flow, VL53L1X ToF, BMP388 baro, MMC5983MA mag) with the thesis simulator's timing and noise models. Their SITL drivers (FLOW_TYPE 101, RNGFNDx_TYPE 101, and the baro and compass, which replace the regular SITL ones) hand every reading to AP_CoopBridge.
    // @Values: 0:Disabled,1:Enabled
    // @User: Advanced
    // @RebootRequired: True
    AP_GROUPINFO_FLAGS("ENABLE", 1, CoopSensors, _enable, 0, AP_PARAM_FLAG_ENABLE),

    // @Param: AGENT
    // @DisplayName: Cooperative agent index
    // @Description: Agent index in the thesis models: selects the baro bias, (2 i - 1) 1.5 Pa, and the noise stream.
    // @Range: 0 7
    // @User: Advanced
    AP_GROUPINFO("AGENT", 2, CoopSensors, _agent, 0),

    // @Param: SEED
    // @DisplayName: Cooperative sensor noise seed
    // @Description: Seed of the sensor noise; the agent index is mixed in, so vehicles with the same seed draw independent noise.
    // @User: Advanced
    AP_GROUPINFO("SEED", 3, CoopSensors, _seed, 7),

    // @Param: TOF_MAX
    // @DisplayName: ToF maximum range
    // @Description: Longest range the simulated VL53L1X reports; beyond it the reading has range status 2 (signal fail).
    // @Units: m
    // @Range: 1 4
    // @User: Advanced
    AP_GROUPINFO("TOF_MAX", 4, CoopSensors, _tof_max_m, 4.0f),

    // @Param: MD_X0
    // @DisplayName: Magnetic disturbance band start
    // @Description: Eastern start of the band, relative to home, inside which the magnetometer direction is bent about the body x axis (scenario S5).
    // @Units: m
    // @User: Advanced
    AP_GROUPINFO("MD_X0", 5, CoopSensors, _magd_x0, 3.0f),

    // @Param: MD_X1
    // @DisplayName: Magnetic disturbance band end
    // @Description: Eastern end of the magnetic disturbance band, relative to home.
    // @Units: m
    // @User: Advanced
    AP_GROUPINFO("MD_X1", 6, CoopSensors, _magd_x1, 6.0f),

    // @Param: MD_DEG
    // @DisplayName: Magnetic disturbance peak
    // @Description: Rotation of the field direction at the band centre (a half sine across the band), and a field norm raised by 30 % at the peak. 0 disables the disturbance.
    // @Units: deg
    // @Range: 0 90
    // @User: Advanced
    AP_GROUPINFO("MD_DEG", 7, CoopSensors, _magd_deg, 0.0f),

    AP_GROUPEND
};

CoopSensors::CoopSensors()
{
    AP_Param::setup_object_defaults(this, var_info);
}

uint64_t CoopSensors::next_random()
{
    // splitmix64: small, fast and reproducible on every host
    uint64_t z = (_rng += 0x9E3779B97F4A7C15ULL);
    z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ULL;
    z = (z ^ (z >> 27)) * 0x94D049BB133111EBULL;
    return z ^ (z >> 31);
}

double CoopSensors::gauss()
{
    // Box-Muller on two uniforms in (0, 1]
    const double u1 = (double(next_random() >> 11) + 1.0) * (1.0 / 9007199254740993.0);
    const double u2 = double(next_random() >> 11) * (1.0 / 9007199254740992.0);
    return sqrt(-2.0 * log(u1)) * cos(2.0 * M_PI * u2);
}

bool CoopSensors::depth(const Truth &x, double &d) const
{
    // sensor origin and beam (body down axis) in NED; the floor is D = 0
    const Vector3f mount_ned = x.dcm * MOUNT_FRD;
    const double sensor_d = x.p_ned.z + mount_ned.z;
    const float beam_d = x.dcm.c.z;     // NED down component of the body z axis
    if (beam_d < TOF_MAX_TILT_COS) {
        return false;
    }
    d = -sensor_d / beam_d;
    return d > TOF_MIN_M;
}

bool CoopSensors::optical_rate(const Truth &x, Vector2d &rate) const
{
    double d;
    if (!depth(x, d)) {
        return false;
    }
    // sensor axes = body FRD axes: velocity of the sensor and rotation rate in them
    const Vector3f v = x.dcm.mul_transpose(x.v_ned) + (x.gyro_frd % MOUNT_FRD);
    const Vector3f &w = x.gyro_frd;
    rate.x = -v.x / d - w.y;
    rate.y = -v.y / d + w.x;
    return true;
}

double CoopSensors::pressure(const Truth &x) const
{
    const double z_up = -x.p_ned.z;
    return reference_pressure(x.t_s) * exp(-z_up / SCALE_HEIGHT_M) + pressure_bias(_agent.get(), x.t_s);
}

void CoopSensors::update(const Aircraft &aircraft, uint64_t now_us)
{
    if (!enabled()) {
        return;
    }
    if (!_rng_seeded) {
        _rng = (uint64_t(uint32_t(_seed.get())) << 32) ^ (uint64_t(uint8_t(_agent.get())) * 0x2545F4914F6CDD1DULL);
        _rng_seeded = true;
    }
    Truth x;
    x.t_s = now_us * 1.0e-6;
    x.p_ned = aircraft.get_position_relhome();
    x.v_ned = aircraft.get_velocity_ef();
    x.gyro_frd = aircraft.get_gyro();
    x.dcm = aircraft.get_dcm();
    x.field_frd = aircraft.get_mag_field_bf();

    if (!_started) {
        // the first step only sets the clock; sensors start on their next grid point
        _started = true;
        _last_us = now_us;
        _flow_start_us = ((now_us + FLOW_WIDTH_US[0] - 1) / FLOW_WIDTH_US[0]) * FLOW_WIDTH_US[0];
        _flow_end_us = _flow_start_us + FLOW_WIDTH_US[0];
        _flow_packet = 0;
        _flow_integral.zero();
        _flow_gyro.zero();
        _flow_ok = true;
        return;
    }
    if (now_us <= _last_us) {
        return;
    }
    const uint64_t a = _last_us;
    _last_us = now_us;
    update_flow(x, a, now_us);
    update_tof(x, a, now_us);
    update_baro(x, a, now_us);
    update_mag(x, a, now_us);
}

void CoopSensors::update_flow(const Truth &x, uint64_t a, uint64_t b)
{
    Vector2d rate;
    const bool ok = optical_rate(x, rate);
    const Vector2d gyro(x.gyro_frd.x, x.gyro_frd.y);
    uint64_t cur = MAX(a, _flow_start_us);
    while (cur < b) {
        const uint64_t seg = MIN(b, _flow_end_us);
        if (seg > cur) {
            const double dt = (seg - cur) * 1.0e-6;
            _flow_integral += rate * dt;
            _flow_gyro += gyro * dt;
            _flow_ok = _flow_ok && ok;
        }
        cur = seg;
        if (seg < _flow_end_us) {
            break;
        }
        // report complete
        Flow r {};
        r.t_start_us = _flow_start_us;
        r.t_end_us = _flow_end_us;
        const double width_s = (_flow_end_us - _flow_start_us) * 1.0e-6;
        r.body_rate = Vector2f(_flow_gyro.x / width_s, _flow_gyro.y / width_s);
        const bool motion = (_flow_packet % FLOW_NO_MOTION_EVERY) != FLOW_NO_MOTION_PHASE;
        r.valid = motion && _flow_ok;
        if (r.valid) {
            // counts = C^-1 * angle + noise, rounded (C upper triangular)
            const double c11 = FLOW_C[0][0], c12 = FLOW_C[0][1], c22 = FLOW_C[1][1];
            const double ny = _flow_integral.y / c22;
            const double nx = (_flow_integral.x - c12 * ny) / c11;
            for (uint8_t i = 0; i < 2; i++) {
                const double n = (i == 0 ? nx : ny) + FLOW_COUNT_SD * gauss();
                r.counts[i] = int16_t(constrain_float(roundf(float(n)), -32767, 32767));
            }
            r.angle.x = FLOW_C[0][0] * r.counts[0] + FLOW_C[0][1] * r.counts[1];
            r.angle.y = FLOW_C[1][0] * r.counts[0] + FLOW_C[1][1] * r.counts[1];
            r.quality = FLOW_QUALITY;
        }
        if (!_flow_q.push(r)) {
            // backend not consuming (FLOW_TYPE not set): keep only the newest reports
            Flow old;
            UNUSED_RESULT(_flow_q.pop(old));
            _flow_q.push(r);
        }
        // next report
        _flow_packet++;
        _flow_start_us = _flow_end_us;
        _flow_end_us = _flow_start_us + FLOW_WIDTH_US[_flow_packet % 3];
        _flow_integral.zero();
        _flow_gyro.zero();
        _flow_ok = true;
    }
}

void CoopSensors::update_tof(const Truth &x, uint64_t a, uint64_t b)
{
    double d = 0;
    bool ok = depth(x, d) && d <= _tof_max_m.get();
    uint64_t cur = a;
    while (cur < b) {
        const uint64_t window_start = (cur / TOF_PERIOD_US) * TOF_PERIOD_US;
        const uint64_t window_end = window_start + TOF_BUDGET_US;
        if (cur >= window_end) {
            // between windows
            cur = MIN(b, window_start + TOF_PERIOD_US);
            continue;
        }
        if (cur == window_start) {
            // a window is only reported if it was integrated from its start
            _tof_open = true;
            _tof_integral = 0;
            _tof_ok = true;
        }
        const uint64_t seg = MIN(b, window_end);
        _tof_integral += d * (seg - cur) * 1.0e-6;
        _tof_ok = _tof_ok && ok;
        cur = seg;
        if (seg == window_end && _tof_open) {
            _tof_open = false;
            Tof r {};
            r.t_start_us = window_start;
            r.t_end_us = window_end;
            if (_tof_ok) {
                r.range_m = _tof_integral / (TOF_BUDGET_US * 1.0e-6) + TOF_SD_M * gauss();
                r.status = 0;
            } else {
                r.range_m = 0;
                r.status = 2;
            }
            if (!_tof_q.push(r)) {
                Tof old;
                UNUSED_RESULT(_tof_q.pop(old));
                _tof_q.push(r);
            }
        }
    }
}

void CoopSensors::update_baro(const Truth &x, uint64_t a, uint64_t b)
{
    const double p = pressure(x);
    uint64_t cur = a;
    while (cur < b) {
        const uint64_t period_start = (cur / BARO_PERIOD_US) * BARO_PERIOD_US;
        const uint64_t offset = cur - period_start;
        // the conversion that covers ``offset``, or the next one
        uint8_t i = 0;
        while (i < 2 && offset >= BARO_CONVERSION_US[i][1]) {
            i++;
        }
        if (i == 2) {
            cur = MIN(b, period_start + BARO_PERIOD_US);
            continue;
        }
        const uint64_t conv_start = period_start + BARO_CONVERSION_US[i][0];
        const uint64_t conv_end = period_start + BARO_CONVERSION_US[i][1];
        if (cur < conv_start) {
            cur = MIN(b, conv_start);
            continue;
        }
        if (cur == conv_start) {
            // the reading needs both conversions integrated from their starts
            _baro_open[i] = (i == 0) || _baro_open[0];
            _baro_integral[i] = 0;
        }
        const uint64_t seg = MIN(b, conv_end);
        _baro_integral[i] += p * (seg - cur) * 1.0e-6;
        cur = seg;
        if (seg == conv_end && i == 1 && _baro_open[1]) {
            _baro_open[0] = _baro_open[1] = false;
            const double width_s = (BARO_CONVERSION_US[0][1] - BARO_CONVERSION_US[0][0]) * 1.0e-6;
            Baro r {};
            r.t_end_us = conv_end;
            // equal conversions: the midpoint is halfway between their centres
            r.mid_before_end_us = BARO_CONVERSION_US[1][1] -
                0.25 * (BARO_CONVERSION_US[0][0] + BARO_CONVERSION_US[0][1] + BARO_CONVERSION_US[1][0] + BARO_CONVERSION_US[1][1]);
            r.pressure_pa = 0.5 * (_baro_integral[0] + _baro_integral[1]) / width_s + BARO_SD_PA * gauss();
            r.temperature_c = BARO_TEMPERATURE_C;
            if (!_baro_q.push(r)) {
                Baro old;
                UNUSED_RESULT(_baro_q.pop(old));
                _baro_q.push(r);
            }
        }
    }
}

void CoopSensors::update_mag(const Truth &x, uint64_t a, uint64_t b)
{
    // one sample per grid point in (a, b], with the truth at b
    for (uint64_t t = (a / MAG_PERIOD_US + 1) * MAG_PERIOD_US; t <= b; t += MAG_PERIOD_US) {
        const float norm = x.field_frd.length();
        if (!is_positive(norm)) {
            continue;
        }
        Vector3d dir(x.field_frd.x / norm, x.field_frd.y / norm, x.field_frd.z / norm);
        // tangent noise: two unit vectors orthogonal to the direction
        Vector3d e1 = (fabs(dir.x) < 0.9 ? Vector3d(1, 0, 0) : Vector3d(0, 1, 0)) % dir;
        e1.normalize();
        const Vector3d e2 = dir % e1;
        dir += e1 * (MAG_SD_RAD * gauss()) + e2 * (MAG_SD_RAD * gauss());
        dir.normalize();
        double ratio = 1.0;
        const float east = x.p_ned.y;
        if (is_positive(_magd_deg.get()) && _magd_x1 > _magd_x0 && east >= _magd_x0 && east <= _magd_x1) {
            // smooth bump: zero at the band edges, the peak rotation about body x at the centre
            const double weight = sin(M_PI * (east - _magd_x0) / (_magd_x1 - _magd_x0));
            const double angle = radians(_magd_deg.get()) * weight;
            const double c = cos(angle), s = sin(angle);
            dir = Vector3d(dir.x, c * dir.y - s * dir.z, s * dir.y + c * dir.z);
            ratio = 1.0 + 0.3 * weight;
        }
        Mag r;
        r.capture_us = t;
        r.field_frd = Vector3f(dir.x, dir.y, dir.z) * float(norm * ratio);
        if (!_mag_q.push(r)) {
            Mag old;
            UNUSED_RESULT(_mag_q.pop(old));
            _mag_q.push(r);
        }
    }
}

#endif  // AP_SIM_COOP_SENSORS_ENABLED
