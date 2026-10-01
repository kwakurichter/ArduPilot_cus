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
  SITL PMW3901 (Crazyflie FlowDeck) for the cooperative estimator, FLOW_TYPE 101.

  Reports come from SITL's cooperative sensor model (SIM_COOP_*, libraries/SITL/
  SIM_CoopSensors.h) with the PMW3901's per-report support, counts and SQUAL. Like the
  hardware driver, each report is handed to AP_CoopBridge as it completes and the frontend
  (and so the EKF) gets the flow rate of the report.
 */
#pragma once

#include "AP_OpticalFlow_config.h"

#if AP_OPTICALFLOW_SITL_COOP_ENABLED

#include "AP_OpticalFlow.h"

class AP_OpticalFlow_SITL_Coop : public OpticalFlow_backend
{
public:
    using OpticalFlow_backend::OpticalFlow_backend;

    // start draining the sensor model at 1 kHz
    void init() override;

    // give the frontend the newest report
    void update() override;

private:
    // timer: pop completed reports, hand them to AP_CoopBridge, keep the newest for update()
    void timer();

    AP_OpticalFlow::OpticalFlow_state _latest;
    bool _have_latest;
};

#endif  // AP_OPTICALFLOW_SITL_COOP_ENABLED
