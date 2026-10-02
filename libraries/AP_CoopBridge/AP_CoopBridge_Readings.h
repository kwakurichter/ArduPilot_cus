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
  Per-reading records that sensor drivers hand to AP_CoopBridge.

  The cooperative estimator models every reading over the time it was actually acquired
  (its support), so a driver reports each reading once, as it completes, with:

    t_end_us            end of the support (FC time, AP_HAL::micros64)
    mid_before_end_us   end minus the width-weighted midpoint of the support, the time the
                        estimator evaluates the reading at (several spans, e.g. two baro
                        conversions, are combined by width)

  and the raw value in the driver's own terms. The bridge applies the calibrations the FC
  owns (flow counts to angle), the noise parameters (COOP_*_SD), the body frame change for
  the magnetometer, and sends one COOP_* message per reading. No gating: the companion
  applies the gates, so the simulation and the firmware share one rejection logic.

  The same records come from the hardware drivers (PMW3901 FlowDeck, VL53L1X, BMP388,
  MMC5983MA) and from their SITL counterparts, so SITL exercises the bridge unchanged.
 */
#pragma once

#include <stdint.h>
#include <AP_Math/vector3.h>

namespace AP_CoopReading {

// One PMW3901 report: motion counts accumulated over the support.
struct Flow {
    uint64_t t_end_us;
    float mid_before_end_us;
    float duration_s;           // total support length
    int16_t delta_counts[2];    // sensor x, y
    uint8_t quality;            // SQUAL
    bool valid;                 // false: no motion report (the counts are meaningless)
};

// One VL53L1X ranging: the range averaged over the timing budget.
struct Tof {
    uint64_t t_end_us;
    float mid_before_end_us;
    float range_m;              // along the sensor axis
    uint8_t range_status;       // VL53L1X range status, 0 = valid
    bool valid;                 // false: the driver could not read a range at all
};

// One BMP388 pressure: one or more conversions.
struct Baro {
    uint64_t t_end_us;
    float mid_before_end_us;
    float pressure_pa;
    float temperature_c;
    bool valid;
};

// One fix of the marker frontend: the body pose in the map at the exposure, already converted
// from the camera frame (body FLU to map ENU, the estimator's frames), with its 6x6 covariance
// in the right tangent [rotation, translation] as the upper triangle, row by row. A source
// that only knows the position (the SITL stand-in) sends a very large rotation variance.
struct Camera {
    uint64_t capture_us;
    uint32_t receipt_after_capture_us;  // when the fix became available on the FC
    uint32_t end_after_capture_us;      // end of the exposure
    float q[4];                 // w, x, y, z
    Vector3f position_m;
    float covariance[21];
    uint8_t marker_id;
    bool valid;
};

// One MMC5983MA sample: the calibrated field in the body frame (FRD, as AP_Compass), any
// unit as long as COOP_MAG_NORM uses the same (AP_Compass uses milligauss).
struct Mag {
    uint64_t capture_us;
    uint16_t end_after_capture_us;
    Vector3f field_frd;
    bool valid;
};

}  // namespace AP_CoopReading
