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

#include "AP_Ranging_Backend.h"

#if AP_RANGING_ENABLED

// base class constructor.
AP_Ranging_Backend::AP_Ranging_Backend(AP_Ranging &frontend) : _frontend(frontend)
{
    // NOTE: a backend sets up its own transport in its own constructor/init.
}

// set the measured range to a node in meters
void AP_Ranging_Backend::set_node_distance(uint8_t node_instance, float distance)
{
    // sanity check instance
    if (node_instance >= AP_RANGING_MAX_NODES) {
        return;
    }

    // grow the node count as new nodes appear
    if (node_instance >= _frontend.num_nodes) {
        _frontend.num_nodes = node_instance + 1;
    }

    _frontend.node_state[node_instance].distance_update_ms = AP_HAL::millis();
    _frontend.node_state[node_instance].distance = distance;
    _frontend.node_state[node_instance].healthy = true;
}

#endif  // AP_RANGING_ENABLED
