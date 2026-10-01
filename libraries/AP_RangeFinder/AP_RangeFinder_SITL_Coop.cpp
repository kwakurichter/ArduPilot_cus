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
#include "AP_RangeFinder_SITL_Coop.h"

#if AP_RANGEFINDER_SIM_COOP_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <SITL/SITL.h>
#include <AP_CoopBridge/AP_CoopBridge.h>

extern const AP_HAL::HAL& hal;

AP_RangeFinder_SITL_Coop::AP_RangeFinder_SITL_Coop(RangeFinder::RangeFinder_State &_state, AP_RangeFinder_Params &_params) :
    AP_RangeFinder_Backend(_state, _params)
{
    hal.scheduler->register_timer_process(FUNCTOR_BIND_MEMBER(&AP_RangeFinder_SITL_Coop::timer, void));
}

void AP_RangeFinder_SITL_Coop::timer()
{
    SITL::SIM *sitl = AP::sitl();
    if (sitl == nullptr) {
        return;
    }
    SITL::CoopSensors::Tof t;
    while (sitl->models.coop_sensors.pop_tof(t)) {
#if AP_COOPBRIDGE_ENABLED
        AP_CoopBridge *bridge = AP::coopbridge();
        if (bridge != nullptr) {
            AP_CoopReading::Tof r {};
            r.t_end_us = t.t_end_us;
            r.mid_before_end_us = 0.5f * (t.t_end_us - t.t_start_us);
            r.range_m = t.range_m;
            r.range_status = t.status;
            r.valid = true;     // the driver read a ranging; its status says if it is usable
            bridge->push_tof(r);
        }
#endif
        WITH_SEMAPHORE(_sem);
        _latest_m = t.range_m;
        _latest_status = t.status;
        _have_latest = true;
    }
}

void AP_RangeFinder_SITL_Coop::update()
{
    WITH_SEMAPHORE(_sem);
    if (!_have_latest) {
        if (AP_HAL::millis() - state.last_reading_ms > 200) {
            set_status(RangeFinder::Status::NoData);
        }
        return;
    }
    _have_latest = false;
    state.last_reading_ms = AP_HAL::millis();
    if (_latest_status != 0) {
        // signal fail: too close, too far or too tilted
        state.distance_m = 0;
        set_status(RangeFinder::Status::NoData);
        return;
    }
    state.distance_m = MAX(0.0f, _latest_m);
    update_status();
}

#endif  // AP_RANGEFINDER_SIM_COOP_ENABLED
