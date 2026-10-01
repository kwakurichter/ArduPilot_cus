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
  First stage of the two-stage IMU preintegration (COOP_IMU_DELTA).

  The companion preintegrates the IMU between its 100 ms knots. The FC cannot afford the full
  per-sample recurrence (it also propagates a 15x15 covariance), so it only integrates the mean
  over short sub-intervals and sends one increment per sub-interval; the companion composes them
  and adds the covariance (thesis repo: scripts/coop/frontends/two_stage.py is the reference,
  embedded/core/src/imu_second_stage.cpp the companion side).

  Per sub-interval [t0, t0 + SUB_US) this class integrates, raw (no bias correction) and in the
  sub-interval's start frame:

      dp <- dp + dv * dt + 1/2 * dR * delta_v * dt
      dv <- dv + dR * delta_v
      dR <- dR * Exp(delta_theta)

  where delta_theta and delta_v are the INS delta angle and delta velocity of one main-loop
  tick and dt its length. The rates are held constant within a tick, so a tick that straddles a
  sub-interval boundary is split in proportion to time. The grid is absolute (t mod SUB_US = 0
  on the FC clock), so it lines up with the knots; time before the first grid point is dropped.

  Pure arithmetic, no HAL access, so it can be unit tested (tests/test_imu_stage.cpp).
 */
#pragma once

#include "AP_CoopBridge_config.h"

#if AP_COOPBRIDGE_ENABLED

#include <AP_Math/AP_Math.h>

class AP_CoopBridge_ImuStage
{
public:
    static constexpr uint32_t SUB_US = AP_COOPBRIDGE_IMU_SUB_US;

    // One completed sub-interval, the contents of COOP_IMU_DELTA.
    struct Sub {
        uint64_t t0_us;         // start of the sub-interval (FC time, a multiple of SUB_US)
        uint16_t duration_us;   // always SUB_US
        Vector3f theta;         // rotation vector Log(dR), rad
        Vector3f velocity;      // dv in the start frame, m/s (gravity not removed)
        Vector3f position;      // dp in the start frame, m (gravity not removed)
        uint16_t samples;       // IMU samples (accel + gyro) integrated, rounded
        uint8_t flags;          // OR of the flags of every tick that contributed
    };

    AP_CoopBridge_ImuStage() { reset(); }

    // Forget everything; the next push() starts a new grid.
    void reset();

    // Integrate one tick [t_start_us, t_end_us] with its delta angle and delta velocity
    // (already in the frame the companion expects). ``samples`` is the number of IMU samples
    // in the tick and ``flags`` its COOP_SAMPLE_FLAGS; both are shared out in proportion to
    // time like the deltas. A tick that does not start where the previous one ended restarts
    // the grid (nothing is sent across the discontinuity).
    //
    // Writes the sub-intervals this tick completes to ``out`` (at most ``capacity``) and
    // returns how many were completed. If that exceeds ``capacity`` (a tick longer than
    // capacity sub-intervals), the extra ones are lost and the return value tells the caller.
    uint16_t push(uint64_t t_start_us, uint64_t t_end_us, const Vector3f &delta_theta,
                 const Vector3f &delta_velocity, float samples, uint8_t flags,
                 Sub *out, uint8_t capacity);

    // True once a grid point has been reached and a sub-interval is being integrated.
    bool running() const { return _running; }

private:
    // Add the part of a tick that lies inside the current sub-interval.
    void integrate(const Vector3f &delta_theta, const Vector3f &delta_velocity, float dt_s);
    // Start a new sub-interval at _sub_start_us with zero increments.
    void begin_sub();

    bool _have_last;          // _last_end_us is valid
    bool _running;            // inside a sub-interval (a grid point has been reached)
    uint64_t _last_end_us;    // end of the previous tick
    uint64_t _sub_start_us;   // start of the current sub-interval
    Quaternion _rotation;     // dR, start frame to current body frame
    Vector3f _velocity;       // dv
    Vector3f _position;       // dp
    float _samples;           // fractional sample count of the current sub-interval
    uint8_t _flags;           // flags of the current sub-interval
};

#endif  // AP_COOPBRIDGE_ENABLED
