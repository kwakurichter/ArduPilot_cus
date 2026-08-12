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
#include <GCS_MAVLink/GCS.h>

#if AP_RANGING_ENABLED

// base class constructor.
AP_Ranging_Backend::AP_Ranging_Backend(AP_Ranging &frontend) : _frontend(frontend)
{
    // NOTE: a backend sets up its own transport in its own constructor/init.
}

// Get parameters from the frontend
uint8_t AP_Ranging_Backend::get_node_id() const    { return gcs().sysid_this_mav(); }
int8_t AP_Ranging_Backend::get_debug() const       { return _frontend._debug.get(); }
uint8_t AP_Ranging_Backend::get_peer_id(uint8_t n) const
{
    if (n >= AP_RANGING_MAX_NODES) {
        return 0;
    }
    return (uint8_t)constrain_int16(_frontend._peer_id[n].get(), 0, 255);
}
uint16_t AP_Ranging_Backend::get_ant_delay() const { return (uint16_t)_frontend._ant_delay.get(); }
uint16_t AP_Ranging_Backend::get_poll_ms() const   { return (uint16_t)_frontend._poll_ms.get(); }
uint8_t  AP_Ranging_Backend::get_channel() const   { return (uint8_t)_frontend._channel.get(); }
uint16_t AP_Ranging_Backend::get_reply_us() const  { return (uint16_t)_frontend._reply_us.get(); }
uint16_t AP_Ranging_Backend::get_xchg_ms() const   { return (uint16_t)_frontend._xchg_ms.get(); }

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
