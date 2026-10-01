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
#include "AP_Compass_SITL_Coop.h"

#if AP_COMPASS_SITL_COOP_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <SITL/SITL.h>
#include <AP_CoopBridge/AP_CoopBridge.h>

extern const AP_HAL::HAL& hal;

AP_Compass_SITL_Coop::AP_Compass_SITL_Coop()
{
    SITL::SIM *sitl = AP::sitl();
    if (sitl == nullptr) {
        return;
    }
    // an MMC5983MA at its I2C address on a simulated bus
    const uint32_t dev_id = AP_HAL::Device::make_bus_id(AP_HAL::Device::BUS_TYPE_SITL, 1, 0x30, DEVTYPE_MMC5983);
    if (!register_compass(dev_id)) {
        return;
    }
    save_dev_id();      // always comes up configured, like the SITL compasses
    set_rotation(ROTATION_NONE);
    set_external(true);
    _offsets = sitl->mag_ofs[0].get();
    if (_compass.get_offsets(instance).is_zero()) {
        _compass.set_offsets(instance, _offsets);
    }
    _registered = true;
    hal.scheduler->register_timer_process(FUNCTOR_BIND_MEMBER(&AP_Compass_SITL_Coop::timer, void));
}

void AP_Compass_SITL_Coop::timer()
{
    SITL::SIM *sitl = AP::sitl();
    if (sitl == nullptr || !_registered) {
        return;
    }
    SITL::CoopSensors::Mag m;
    while (sitl->models.coop_sensors.pop_mag(m)) {
#if AP_COOPBRIDGE_ENABLED
        AP_CoopBridge *bridge = AP::coopbridge();
        if (bridge != nullptr) {
            AP_CoopReading::Mag r {};
            r.capture_us = m.capture_us;
            r.end_after_capture_us = 0;     // instantaneous sample
            r.field_frd = m.field_frd;
            r.valid = true;
            bridge->push_mag(r);
        }
#endif
        // the raw sensor field carries the hard-iron error that COMPASS_OFS compensates
        // (ArduPilot adds the offsets); the frontend rotates by the board orientation, so
        // undo it here as the SITL compasses do
        Vector3f raw = m.field_frd - _offsets;
        raw.rotate(get_board_orientation());
        accumulate_sample(raw, 10);
    }
}

void AP_Compass_SITL_Coop::read()
{
    drain_accumulated_samples(nullptr);
}

#endif  // AP_COMPASS_SITL_COOP_ENABLED
