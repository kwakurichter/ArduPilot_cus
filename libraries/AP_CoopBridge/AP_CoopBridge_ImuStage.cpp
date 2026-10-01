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
#include "AP_CoopBridge_ImuStage.h"

#if AP_COOPBRIDGE_ENABLED

void AP_CoopBridge_ImuStage::reset()
{
    _have_last = false;
    _running = false;
    _last_end_us = 0;
    _sub_start_us = 0;
    begin_sub();
}

void AP_CoopBridge_ImuStage::begin_sub()
{
    _rotation.initialise();
    _velocity.zero();
    _position.zero();
    _samples = 0;
    _flags = 0;
}

void AP_CoopBridge_ImuStage::integrate(const Vector3f &delta_theta, const Vector3f &delta_velocity, float dt_s)
{
    // position first: it uses the velocity and rotation at the start of the piece
    const Vector3f dv_start = _rotation * delta_velocity;
    _position += _velocity * dt_s + dv_start * (0.5f * dt_s);
    _velocity += dv_start;
    _rotation.rotate(delta_theta);  // dR * Exp(delta_theta)
    _rotation.normalize();
}

uint16_t AP_CoopBridge_ImuStage::push(uint64_t t_start_us, uint64_t t_end_us, const Vector3f &delta_theta,
                                     const Vector3f &delta_velocity, float samples, uint8_t flags,
                                     Sub *out, uint8_t capacity)
{
    if (t_end_us <= t_start_us) {
        return 0;
    }
    if (!_have_last || t_start_us != _last_end_us) {
        // first tick, or a discontinuity: drop the partial sub-interval and wait for the
        // next grid point
        _running = false;
        begin_sub();
    }
    _have_last = true;
    _last_end_us = t_end_us;

    const float tick_us = float(t_end_us - t_start_us);
    uint16_t completed = 0;
    uint64_t a = t_start_us;
    while (a < t_end_us) {
        if (!_running) {
            // skip to the first grid point at or after a
            const uint64_t grid = ((a + SUB_US - 1) / SUB_US) * SUB_US;
            if (grid >= t_end_us) {
                break;
            }
            a = grid;
            _sub_start_us = grid;
            _running = true;
            begin_sub();
        }
        // the piece [a, b] of this tick that lies in the current sub-interval
        const uint64_t boundary = _sub_start_us + SUB_US;
        const uint64_t b = MIN(t_end_us, boundary);
        const float share = float(b - a) / tick_us;
        integrate(delta_theta * share, delta_velocity * share, float(b - a) * 1.0e-6f);
        _samples += samples * share;
        _flags |= flags;
        a = b;
        if (b == boundary) {
            if (completed < capacity) {
                Sub &s = out[completed];
                s.t0_us = _sub_start_us;
                s.duration_us = SUB_US;
                _rotation.to_axis_angle(s.theta);
                s.velocity = _velocity;
                s.position = _position;
                s.samples = uint16_t(MIN(roundf(_samples), float(UINT16_MAX)));
                s.flags = _flags;
            }
            completed++;
            _sub_start_us = boundary;
            begin_sub();
        }
    }
    return completed;
}

#endif  // AP_COOPBRIDGE_ENABLED
