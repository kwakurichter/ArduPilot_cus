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
  Simulated Crazyflie aid sensors for the cooperative estimator (SIM_COOP_*).

  The physics truth of SITL is turned into per-reading measurements with the timing, noise
  and failure models of the thesis simulator (thesis repo: scripts/decentralized/reference/
  synthetic.py generate(), scripts/coop/scenarios.py), so SITL runs exercise the companion
  with the same statistics as the P2-P4 simulations:

    PMW3901 flow   reports over 10, 20, 30 ms (cycling); every 5th report has no motion;
                   counts = C^-1 * (integral of the optical rate) + N(0, 0.5), rounded;
                   SQUAL 90 (0 without a usable surface)
    VL53L1X ToF    every 25 ms, the range averaged over a 20 ms timing budget, + N(0, 0.015 m)
    BMP388 baro    every 40 ms, two 4 ms conversions (at +2 and +22 ms) averaged,
                   p = p_ref(t) exp(-z / H) + bias(agent, t) + N(0, 1.5 / sqrt(2) Pa)
    MMC5983MA mag  every 10 ms, the truth field direction with N(0, 0.008 rad) per tangent axis;
                   optionally bent inside an east band (scenario S5, SIM_COOP_MD_*)
    camera         stand-in for the HM01B0 marker frontend, off by default: sparse fixes of the
                   body position in the map (SIM_COOP_CAM_HZ, _SD, _LAT), released after the
                   detection latency; AP_CoopBridge sends them as COOP_CAMERA (position only)

  Readings are produced on the absolute FC-time grid (multiples of the period), so they line
  up with the IMU sub-intervals and knots. The SITL driver backends (AP_OpticalFlow_SITL_Coop,
  AP_RangeFinder_SITL_Coop, AP_Baro_SITL_Coop, AP_Compass_SITL_Coop) pop them from the queues
  below, feed their frontends (and so the EKF) and hand them to AP_CoopBridge exactly as the
  hardware drivers do.

  Frames: the companion's map is east-north-up with the floor at z = 0; this vehicle's home is at
  (SIM_COOP_MAP_E, SIM_COOP_MAP_N, 0) in it, so vehicles started at different homes share one
  map. Readings in the body frame (flow, ToF, mag) do not depend on the offset. The flow and ToF
  sensors look down from 25 mm ahead of and 15 mm below the body origin (the thesis "DOWN"
  mount), with their x/y axes along the body's FRD x/y axes.
 */
#pragma once

#include "SIM_config.h"

#if AP_SIM_COOP_SENSORS_ENABLED

#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>
#include <AP_HAL/utility/RingBuffer.h>

namespace SITL {

class Aircraft;

class CoopSensors {
public:
    CoopSensors();

    static const struct AP_Param::GroupInfo var_info[];

    bool enabled() const { return _enable.get() != 0; }

    // Advance the sensors to the end of one physics step at FC time ``now_us``.
    void update(const Aircraft &aircraft, uint64_t now_us);

    // One PMW3901 report (support [t_start_us, t_end_us]).
    struct Flow {
        uint64_t t_start_us, t_end_us;
        int16_t counts[2];       // sensor x, y
        Vector2f angle;          // C * counts (rad, sensor axes): what a calibrated driver sees
        Vector2f body_rate;      // mean body FRD x/y rate over the support (rad/s), for the EKF
        uint8_t quality;         // SQUAL
        bool valid;              // false: no motion report
    };
    // One VL53L1X ranging over the timing budget [t_start_us, t_end_us].
    struct Tof {
        uint64_t t_start_us, t_end_us;
        float range_m;
        uint8_t status;          // 0 valid, 2 signal fail (surface too close, too far or tilted)
    };
    // One BMP388 reading (two conversions).
    struct Baro {
        uint64_t t_end_us;
        float mid_before_end_us;
        float pressure_pa;
        float temperature_c;
    };
    // One camera stand-in fix: body pose in the map (FLU to ENU) at the capture.
    struct Camera {
        uint64_t capture_us, receipt_us;
        float q[4];              // w, x, y, z
        Vector3f position_m;     // map ENU, with noise
        float sigma_m;           // position noise per axis
    };
    // Rotation standard deviation the bridge sends with a position-only fix (rad): large
    // enough that the rotation carries no information.
    static constexpr float CAMERA_ROTATION_SD = 3.0f;

    // One MMC5983MA sample.
    struct Mag {
        uint64_t capture_us;
        Vector3f field_frd;      // milligauss, body FRD
    };

    // Readings in completion order (single consumer each: the matching SITL backend).
    bool pop_flow(Flow &r) { return _flow_q.pop(r); }
    bool pop_tof(Tof &r) { return _tof_q.pop(r); }
    bool pop_baro(Baro &r) { return _baro_q.pop(r); }
    bool pop_mag(Mag &r) { return _mag_q.pop(r); }
    bool pop_camera(Camera &r) { return _camera_q.pop(r); }

private:
    // Truth at one instant, in the frames the models use.
    struct Truth {
        double t_s;              // FC time (s)
        Vector3d p_ned;          // body origin relative to home
        Vector3f v_ned;
        Vector3f gyro_frd;       // rad/s
        Matrix3f dcm;            // body FRD to NED
        Vector3f field_frd;      // milligauss
    };
    // Sensor depth to the floor along the sensor axis, false if not measurable.
    bool depth(const Truth &x, double &d) const;
    // Optical rate (rad/s) in the sensor axes, as hardware_models.optical_rate.
    bool optical_rate(const Truth &x, Vector2d &rate) const;
    // Static pressure at the body origin (Pa), without noise.
    double pressure(const Truth &x) const;

    void update_flow(const Truth &x, uint64_t a_us, uint64_t b_us);
    void update_tof(const Truth &x, uint64_t a_us, uint64_t b_us);
    void update_baro(const Truth &x, uint64_t a_us, uint64_t b_us);
    void update_mag(const Truth &x, uint64_t a_us, uint64_t b_us);
    void update_camera(const Truth &x, uint64_t a_us, uint64_t b_us);

    // Deterministic Gaussian noise (seeded by SIM_COOP_SEED and the agent).
    double gauss();
    uint64_t next_random();

    // Parameters
    AP_Int8 _enable;
    AP_Int8 _agent;              // agent index of the thesis models (pressure bias)
    AP_Int32 _seed;
    AP_Float _tof_max_m;
    AP_Float _magd_x0, _magd_x1; // east band of the magnetic disturbance (m from home)
    AP_Float _magd_deg;          // peak rotation at the band centre, 0 = off
    AP_Float _map_e, _map_n;     // this vehicle's home in the shared map (m)
    AP_Float _cam_hz;            // camera stand-in fix rate, 0 = off
    AP_Float _cam_sd;            // its position noise per axis (m)
    AP_Float _cam_lat_ms;        // its detection latency (ms)

    // State
    bool _started;
    uint64_t _last_us;
    uint64_t _rng;
    bool _rng_seeded;

    // flow: the report being integrated
    uint64_t _flow_start_us, _flow_end_us;
    uint32_t _flow_packet;
    Vector2d _flow_integral;     // integral of the optical rate (rad)
    Vector2d _flow_gyro;         // integral of the body x/y rate (rad)
    bool _flow_ok;               // surface usable throughout

    // ToF: the window being averaged (when inside one)
    double _tof_integral;        // integral of the depth (m s)
    bool _tof_ok;                // surface measurable throughout
    bool _tof_open;              // integrating a window from its start

    // baro: the two conversion averages of the current 40 ms period
    double _baro_integral[2];    // integral of the pressure over each conversion (Pa s)
    bool _baro_open[2];          // conversion integrated from its start (and the first one)

    ObjectBuffer<Flow> _flow_q{8};
    ObjectBuffer<Tof> _tof_q{4};
    ObjectBuffer<Baro> _baro_q{4};
    ObjectBuffer<Mag> _mag_q{16};
    ObjectBuffer<Camera> _camera_q{4};

    // camera fixes captured but not yet released (detection latency)
    static constexpr uint8_t MAX_PENDING_CAMERA = 4;
    Camera _camera_pending[MAX_PENDING_CAMERA];
    uint8_t _n_camera_pending;
};

}  // namespace SITL

#endif  // AP_SIM_COOP_SENSORS_ENABLED
