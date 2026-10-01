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
#include "AP_Baro_SITL_Coop.h"

#if AP_BARO_SITL_COOP_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <SITL/SITL.h>
#include <AP_CoopBridge/AP_CoopBridge.h>

extern const AP_HAL::HAL& hal;

AP_Baro_SITL_Coop::AP_Baro_SITL_Coop(AP_Baro &baro) :
    AP_Baro_Backend(baro)
{
    _instance = _frontend.register_sensor();
    // a BMP388 at its I2C address on a simulated bus
    set_bus_id(_instance, AP_HAL::Device::make_bus_id(AP_HAL::Device::BUS_TYPE_SITL, 1, 0x77, DEVTYPE_BARO_BMP388));
    hal.scheduler->register_timer_process(FUNCTOR_BIND_MEMBER(&AP_Baro_SITL_Coop::timer, void));
}

void AP_Baro_SITL_Coop::timer()
{
    SITL::SIM *sitl = AP::sitl();
    if (sitl == nullptr) {
        return;
    }
    SITL::CoopSensors::Baro b;
    while (sitl->models.coop_sensors.pop_baro(b)) {
#if AP_COOPBRIDGE_ENABLED
        AP_CoopBridge *bridge = AP::coopbridge();
        if (bridge != nullptr) {
            AP_CoopReading::Baro r {};
            r.t_end_us = b.t_end_us;
            r.mid_before_end_us = b.mid_before_end_us;
            r.pressure_pa = b.pressure_pa;
            r.temperature_c = b.temperature_c;
            r.valid = true;
            bridge->push_baro(r);
        }
#endif
        WITH_SEMAPHORE(_sem);
        _latest_pa = b.pressure_pa;
        _latest_c = b.temperature_c;
        _have_latest = true;
    }
}

void AP_Baro_SITL_Coop::update()
{
    WITH_SEMAPHORE(_sem);
    if (!_have_latest) {
        return;
    }
    _copy_to_frontend(_instance, _latest_pa, _latest_c);
    _have_latest = false;
}

#endif  // AP_BARO_SITL_COOP_ENABLED
