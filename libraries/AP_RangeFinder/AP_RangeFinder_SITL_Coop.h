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
  SITL VL53L1X (Crazyflie FlowDeck) for the cooperative estimator, RNGFNDx_TYPE 101.

  Rangings come from SITL's cooperative sensor model (SIM_COOP_*, libraries/SITL/
  SIM_CoopSensors.h): one range averaged over the 20 ms timing budget every 25 ms, with its
  range status. Each ranging is handed to AP_CoopBridge as it completes, as the hardware
  driver does; the frontend (EKF, terrain following) gets the valid ones.
 */
#pragma once

#include "AP_RangeFinder_config.h"

#if AP_RANGEFINDER_SIM_COOP_ENABLED

#include "AP_RangeFinder_Backend.h"

class AP_RangeFinder_SITL_Coop : public AP_RangeFinder_Backend
{
public:
    AP_RangeFinder_SITL_Coop(RangeFinder::RangeFinder_State &_state, AP_RangeFinder_Params &_params);

    // give the frontend the newest ranging
    void update() override;

protected:
    MAV_DISTANCE_SENSOR _get_mav_distance_sensor_type() const override {
        return MAV_DISTANCE_SENSOR_LASER;
    }

private:
    // timer: pop completed rangings, hand them to AP_CoopBridge, keep the newest for update()
    void timer();

    bool _have_latest;
    float _latest_m;
    uint8_t _latest_status;
};

#endif  // AP_RANGEFINDER_SIM_COOP_ENABLED
