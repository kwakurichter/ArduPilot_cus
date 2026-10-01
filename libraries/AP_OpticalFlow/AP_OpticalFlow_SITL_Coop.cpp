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
#include "AP_OpticalFlow_SITL_Coop.h"

#if AP_OPTICALFLOW_SITL_COOP_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <SITL/SITL.h>
#include <AP_CoopBridge/AP_CoopBridge.h>

extern const AP_HAL::HAL& hal;

void AP_OpticalFlow_SITL_Coop::init()
{
    hal.scheduler->register_timer_process(FUNCTOR_BIND_MEMBER(&AP_OpticalFlow_SITL_Coop::timer, void));
}

void AP_OpticalFlow_SITL_Coop::timer()
{
    SITL::SIM *sitl = AP::sitl();
    if (sitl == nullptr) {
        return;
    }
    SITL::CoopSensors::Flow f;
    while (sitl->models.coop_sensors.pop_flow(f)) {
        const float duration_s = (f.t_end_us - f.t_start_us) * 1.0e-6f;
#if AP_COOPBRIDGE_ENABLED
        AP_CoopBridge *bridge = AP::coopbridge();
        if (bridge != nullptr) {
            AP_CoopReading::Flow r {};
            r.t_end_us = f.t_end_us;
            r.mid_before_end_us = 0.5f * (f.t_end_us - f.t_start_us);
            r.duration_s = duration_s;
            r.delta_counts[0] = f.counts[0];
            r.delta_counts[1] = f.counts[1];
            r.quality = f.quality;
            r.valid = f.valid;
            bridge->push_flow(r);
        }
#endif
        // ArduPilot's flow rate is the rotation rate about the sensor x and y axes; the
        // sensor's angle is the image motion along them: rate = (angle_y, -angle_x) / T
        AP_OpticalFlow::OpticalFlow_state state {};
        state.surface_quality = f.valid ? f.quality : 0;
        if (f.valid && is_positive(duration_s)) {
            state.flowRate = Vector2f(f.angle.y, -f.angle.x) / duration_s;
        }
        state.bodyRate = f.body_rate;
        WITH_SEMAPHORE(_sem);
        _latest = state;
        _have_latest = true;
    }
}

void AP_OpticalFlow_SITL_Coop::update()
{
    AP_OpticalFlow::OpticalFlow_state state;
    {
        WITH_SEMAPHORE(_sem);
        if (!_have_latest) {
            return;
        }
        state = _latest;
        _have_latest = false;
    }
    _applyYaw(state.flowRate);
    _update_frontend(state);
}

#endif  // AP_OPTICALFLOW_SITL_COOP_ENABLED
