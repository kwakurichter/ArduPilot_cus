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
#include <AP_Logger/AP_Logger.h>
#include <AP_SwarmMesh/AP_SwarmMesh_config.h>
#if AP_SWARMMESH_ENABLED
#include <AP_SwarmMesh/AP_SwarmMesh.h>
#endif

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
bool     AP_Ranging_Backend::knot_schedule() const { return _frontend._sched.get() == 1; }
uint32_t AP_Ranging_Backend::get_slot_us() const   { return (uint32_t)MAX(_frontend._slot_us.get(), 0); }

uint64_t AP_Ranging_Backend::mesh_now_us()
{
#if AP_SWARMMESH_ENABLED
    return AP_SwarmMesh::mesh_micros64();
#else
    return AP_HAL::micros64();
#endif
}

bool AP_Ranging_Backend::scheduled_slot(uint64_t now_us, Slot &slot) const
{
    // roster: this node and the configured peers, without repeats, sorted ascending
    const uint8_t self = get_node_id();
    uint8_t ids[AP_RANGING_MAX_NODES + 1];
    uint8_t n = 0;
    ids[n++] = self;
    for (uint8_t i = 0; i < AP_RANGING_MAX_NODES; i++) {
        const uint8_t id = get_peer_id(i);
        bool seen = (id == 0);
        for (uint8_t j = 0; j < n && !seen; j++) {
            seen = (ids[j] == id);
        }
        if (!seen) {
            ids[n++] = id;
        }
    }
    if (n < 2) {
        return false;
    }
    for (uint8_t i = 1; i < n; i++) {
        for (uint8_t j = i; j > 0 && ids[j - 1] > ids[j]; j--) {
            const uint8_t t = ids[j];
            ids[j] = ids[j - 1];
            ids[j - 1] = t;
        }
    }

    // Circle method over m = n (+ 1 bye if n is odd) positions: position 0 stays, the others
    // turn by r = k mod (m - 1): rotated[1 + j] = position 1 + (j - r) mod (m - 1). Pair i is
    // (rotated[i], rotated[m - 1 - i]); position n (if present) is the bye.
    const uint32_t period = uint32_t(now_us / AP_RANGING_KNOT_US);
    const uint8_t m = n + (n & 1);
    const uint8_t r = uint8_t(period % (m - 1));
    auto rotated = [m, r](uint8_t pos) -> uint8_t {
        if (pos == 0) {
            return 0;
        }
        const uint8_t j = pos - 1;
        return 1 + uint8_t((j + (m - 1) - r) % (m - 1));
    };
    uint8_t s = 0;
    for (uint8_t i = 0; i < m / 2; i++) {
        const uint8_t a = rotated(i);
        const uint8_t b = rotated(m - 1 - i);
        if (a >= n || b >= n) {
            continue;   // the bye: no slot
        }
        if (ids[a] == self || ids[b] == self) {
            const uint8_t peer = ids[a] == self ? ids[b] : ids[a];
            slot.period = period;
            slot.start_us = uint64_t(period) * AP_RANGING_KNOT_US +
                            uint64_t(MAX(_frontend._slot0_us.get(), 0)) + uint64_t(s) * get_slot_us();
            slot.peer = peer;
            // lower id initiates in even periods, higher in odd ones
            slot.initiator = ((period & 1U) == 0) == (self < peer);
            return true;
        }
        s++;
    }
    return false;
}

void AP_Ranging_Backend::record_exchange(AP_Ranging::Exchange &e)
{
    e.variance_m2 = sq(_frontend._sigma.get());
    if (!_frontend._exchanges.push(e)) {
        _frontend._exchanges_lost++;
    }
    set_node_distance(e.initiator == get_node_id() ? e.responder : e.initiator, e.range_m);

#if HAL_LOGGING_ENABLED
    // @LoggerMessage: TWRX
    // @Description: One completed UWB two-way-ranging exchange, recorded at both ends (the responder computes the range; the initiator learns it from the REPORT frame in the knot-aligned schedule)
    // @Field: TimeUS: Time of the log write
    // @Field: Start: Exchange start (POLL on air at the initiator, POLL received at the responder)
    // @Field: Dur: Start to the end of this node's part of the exchange
    // @Field: I: MAV_SYSID of the initiator
    // @Field: R: MAV_SYSID of the responder
    // @Field: Rng: Range
    // @Field: RxP: Total receive power of the FINAL frame (0 if not measured)
    // @Field: FpP: First-path power of the FINAL frame (0 if not measured)
    AP::logger().Write("TWRX", "TimeUS,Start,Dur,I,R,Rng,RxP,FpP", "sss--m--", "FFF--0--", "QQIBBfff",
                       AP_HAL::micros64(), e.start_us, e.duration_us, e.initiator, e.responder,
                       e.range_m, e.power_valid ? e.rx_power_dbm : 0.0f,
                       e.power_valid ? e.fp_power_dbm : 0.0f);
#endif
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
