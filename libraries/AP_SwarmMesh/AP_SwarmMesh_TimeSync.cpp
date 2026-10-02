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
  Mesh clock offset estimator; see AP_SwarmMesh_TimeSync.h.
 */
#include "AP_SwarmMesh_TimeSync.h"

void AP_SwarmMesh_TimeSync::reset()
{
    _n = 0;
    _next = 0;
    _ref = 0;
    _have_target = false;
}

void AP_SwarmMesh_TimeSync::sample(const Config &cfg, uint8_t ref_id, uint64_t ref_us, uint64_t local_us)
{
    if (ref_id != _ref) {
        reset();
        _ref = ref_id;
    }
    uint8_t window = cfg.window;
    if (window < 1) {
        window = 1;
    } else if (window > AP_SWARMMESH_SYNC_WINDOW_MAX) {
        window = AP_SWARMMESH_SYNC_WINDOW_MAX;
    }
    if (_n > window || _next >= window) {
        // the window shrank: start it again
        _n = 0;
        _next = 0;
    }
    _d[_next] = (int64_t)(local_us - ref_us);
    _next = (_next + 1) % window;
    if (_n < window) {
        _n++;
    }
    int64_t d_min = _d[0];
    for (uint8_t i = 1; i < _n; i++) {
        if (_d[i] < d_min) {
            d_min = _d[i];
        }
    }
    _target = (int64_t)cfg.latency_us - d_min;
    _have_target = true;
    _last_sample_us = local_us;
    _samples++;
}

void AP_SwarmMesh_TimeSync::update(const Config &cfg, uint64_t local_us, bool may_step)
{
    const uint64_t dt_us = (_last_update_us != 0 && local_us > _last_update_us) ? local_us - _last_update_us : 0;
    _last_update_us = local_us;
    if (!_have_target) {
        return;
    }
    const int64_t error = _target - _applied;
    const int64_t abs_error = error < 0 ? -error : error;
    if (!_applied_once || (may_step && abs_error > cfg.step_us)) {
        _applied = _target;
        _applied_once = true;
        _steps++;
        return;
    }
    const int64_t max_change = (int64_t)(dt_us * cfg.slew_ppm / 1000000U);
    if (abs_error <= max_change) {
        _applied = _target;
    } else {
        _applied += error > 0 ? max_change : -max_change;
    }
}

bool AP_SwarmMesh_TimeSync::locked(const Config &cfg, uint64_t local_us) const
{
    return _have_target && _applied_once && local_us - _last_sample_us < cfg.timeout_us;
}
