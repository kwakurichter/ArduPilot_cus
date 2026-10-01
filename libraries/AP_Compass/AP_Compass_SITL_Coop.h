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
  SITL MMC5983MA for the cooperative estimator: replaces the regular SITL compasses when
  SIM_COOP_ENABLE is set.

  Samples come from SITL's cooperative sensor model (SIM_COOP_*, libraries/SITL/
  SIM_CoopSensors.h): the truth body field every 10 ms with the thesis simulator's direction
  noise and optional disturbance band. Each sample is handed to AP_CoopBridge (calibrated, as
  the bridge expects) and fed to the frontend with a fixed hard-iron offset (SIM_MAG1_OFS) that
  the frontend's COMPASS_OFS removes, so the compass counts as calibrated.
 */
#pragma once

#include "AP_Compass_config.h"

#if AP_COMPASS_SITL_COOP_ENABLED

#include "AP_Compass_Backend.h"

class AP_Compass_SITL_Coop : public AP_Compass_Backend
{
public:
    AP_Compass_SITL_Coop();

    // publish the samples accumulated since the last call
    void read() override;

private:
    // timer: pop completed samples, hand them to AP_CoopBridge and accumulate them
    void timer();

    bool _registered;
    Vector3f _offsets;          // simulated hard-iron offset (mGauss)
};

#endif  // AP_COMPASS_SITL_COOP_ENABLED
