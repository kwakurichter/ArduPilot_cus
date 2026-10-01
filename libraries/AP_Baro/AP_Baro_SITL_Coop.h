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
  SITL BMP388 for the cooperative estimator: replaces the regular SITL barometers when
  SIM_COOP_ENABLE is set.

  Readings come from SITL's cooperative sensor model (SIM_COOP_*, libraries/SITL/
  SIM_CoopSensors.h): every 40 ms, two 4 ms conversions averaged, with the thesis
  simulator's reference-pressure drift and per-agent bias. Each reading is handed to
  AP_CoopBridge as it completes, as the hardware driver does, and copied to the frontend.
 */
#pragma once

#include "AP_Baro_Backend.h"

#if AP_BARO_SITL_COOP_ENABLED

class AP_Baro_SITL_Coop : public AP_Baro_Backend
{
public:
    AP_Baro_SITL_Coop(AP_Baro &baro);

    // give the frontend the newest reading
    void update() override;

private:
    // timer: pop completed readings, hand them to AP_CoopBridge, keep the newest for update()
    void timer();

    uint8_t _instance;
    bool _have_latest;
    float _latest_pa;
    float _latest_c;
};

#endif  // AP_BARO_SITL_COOP_ENABLED
