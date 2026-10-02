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
  Mesh clock of a follower: the offset that turns this vehicle's micros64() into the
  reference vehicle's clock, estimated from the reference's SYNC broadcasts.

  Each SYNC carries the reference's mesh time at transmission, ref_us; the follower notes its
  own clock at reception, local_us. Then d = local_us - ref_us = delay - offset, where the
  delay (transmit queue, radio, receive servicing) is never negative. The smallest d of the
  last ``window`` samples has the least delay, so the target offset is

      target = latency - min(d)

  with ``latency`` the fixed part of the delay that the minimum cannot see (_SYNC_LAT). The
  applied offset follows the target at a bounded rate (_SYNC_SLEW, ppm) so mesh time stays
  monotonic and smooth. It jumps only to the first estimate and, where the caller allows it
  (a vehicle on the ground), to errors above ``step_us``.

  Clock drift biases the minimum by at most drift x window period (20 ppm over 8 samples at
  2 Hz: 80 us). No dependencies beyond stdint, so it is unit-tested on the host.
 */
#pragma once

#include <stdint.h>

#ifndef AP_SWARMMESH_SYNC_WINDOW_MAX
#define AP_SWARMMESH_SYNC_WINDOW_MAX 16
#endif

class AP_SwarmMesh_TimeSync
{
public:
    struct Config {
        uint8_t  window = 8;          // samples in the minimum (1..AP_SWARMMESH_SYNC_WINDOW_MAX)
        uint32_t slew_ppm = 500;      // largest rate of change of the applied offset
        int32_t  latency_us = 0;      // fixed delay added to every estimate
        int64_t  step_us = 20000;     // when stepping is allowed, step on errors above this
        uint32_t timeout_us = 3000000;// no sample for this long: not locked
    };

    // One SYNC from reference ``ref_id`` (its mesh time ``ref_us`` at transmission), received
    // at local time ``local_us``. A sample from another reference than the current one starts
    // a new window (the caller decides which reference to follow).
    void sample(const Config &cfg, uint8_t ref_id, uint64_t ref_us, uint64_t local_us);

    // Move the applied offset towards the target, at local time ``local_us``. ``may_step``:
    // the offset may jump (vehicle on the ground). The first target is always taken at once.
    void update(const Config &cfg, uint64_t local_us, bool may_step);

    // Forget the reference (window, lock); the applied offset is kept.
    void reset();

    int64_t  offset_us() const { return _applied; }
    int64_t  target_us() const { return _target; }
    bool     locked(const Config &cfg, uint64_t local_us) const;
    uint8_t  reference() const { return _ref; }     // 0 = none yet
    uint64_t last_sample_us() const { return _last_sample_us; }
    uint32_t samples() const { return _samples; }
    uint32_t steps() const { return _steps; }

private:
    int64_t  _d[AP_SWARMMESH_SYNC_WINDOW_MAX] {};
    uint8_t  _n = 0;            // samples held
    uint8_t  _next = 0;         // ring position of the next sample
    uint8_t  _ref = 0;
    bool     _have_target = false;
    bool     _applied_once = false; // the applied offset has followed a target
    int64_t  _target = 0;
    int64_t  _applied = 0;
    uint64_t _last_sample_us = 0;
    uint64_t _last_update_us = 0;
    uint32_t _samples = 0;
    uint32_t _steps = 0;
};
