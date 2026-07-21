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

// this node's configured UWB address (RNG_NODE_ID)
uint8_t AP_Ranging_Backend::get_node_id() const
{
    return (uint8_t)_frontend._node_id.get();
}

// debug verbosity (RNG_DEBUG)
int8_t AP_Ranging_Backend::get_debug() const
{
    return _frontend._debug.get();
}

// record a measured range (meters) to the peer identified by node_id
void AP_Ranging_Backend::set_node_distance(uint8_t node_id, float distance)
{
    const uint32_t now = AP_HAL::millis();

    // find the stable slot already assigned to this node id
    for (uint8_t i = 0; i < _frontend.num_nodes; i++) {
        if (_frontend.node_state[i].id == node_id) {
            _frontend.node_state[i].distance = distance;
            _frontend.node_state[i].distance_update_ms = now;
            _frontend.node_state[i].healthy = true;
            return;
        }
    }

    // first time we've seen this node: claim the next slot. Slots are never reassigned, so a node keeps its slot for the life of the library, which
    // keeps log/report column ordering stable.
    if (_frontend.num_nodes >= AP_RANGING_MAX_NODES) {
        return;   // node table full
    }
    const uint8_t i = _frontend.num_nodes++;
    _frontend.node_state[i].id = node_id;
    _frontend.node_state[i].distance = distance;
    _frontend.node_state[i].distance_update_ms = now;
    _frontend.node_state[i].healthy = true;
}

#endif  // AP_RANGING_ENABLED
