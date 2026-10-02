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
#pragma once

#include "AP_Ranging.h"

#if AP_RANGING_ENABLED

#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>
#include <AP_HAL/AP_HAL.h>

// Base class for a ranging backend. A concrete backend owns its own transport
// (e.g. a DW1000 over SPI) and the TWR message schedule, and reports each
// measured range back to the frontend via set_node_distance().
class AP_Ranging_Backend
{
public:
    // constructor
    AP_Ranging_Backend(AP_Ranging &frontend);

    virtual ~AP_Ranging_Backend(void) {}

    // return true if the sensor is basically healthy (we are receiving data)
    virtual bool healthy() = 0;

    // update - run the ranging state machine and publish new ranges
    virtual void update() = 0;

    // record a measured range (meters) to the peer identified by node_id.
    void set_node_distance(uint8_t node_id, float distance);

    /*
      Record one completed exchange: queue it for AP_CoopBridge (COOP_UWB), log it (TWRX)
      and update the peer table. ``e.variance_m2`` is filled in here (RNG_SIGMA^2). Any
      thread; one producer.
     */
    void record_exchange(AP_Ranging::Exchange &e);

    /*
      Knot-aligned pair schedule (RNG_SCHED = 1). In period k = floor(t / knot) the roster
      (this node and RNG_PEER_n: nonzero, distinct, sorted) is split into disjoint pairs by
      round k of the circle method, as the thesis simulator's round_robin_pairs (for an odd
      roster the pair with the bye is dropped, and the slots of the remaining pairs follow
      on without a gap). Fills ``slot`` with this node's exchange of the period of
      ``now_us``; false if it sits out (odd roster) or the roster is too short.
     */
    struct Slot {
        uint32_t period;             // k
        uint64_t start_us;           // knot + RNG_SLOT0_US + s * RNG_SLOT_US
        uint8_t  peer;               // MAV_SYSID of the other end
        bool     initiator;          // this node sends the POLL
    };
    bool scheduled_slot(uint64_t now_us, Slot &slot) const;

    // The schedule runs on mesh time (AP_SwarmMesh::mesh_micros64(), the swarm's common clock;
    // micros64() without a mesh), so the pairs agree on the periods and slots. Exchange
    // records stay in micros64(); AP_CoopBridge converts them.
    static uint64_t mesh_now_us();

protected:

    // this node's configured UWB address (RNG_NODE_ID)
    uint8_t get_node_id() const;

    // debug verbosity (RNG_DEBUG); >0 means debug output is enabled
    int8_t get_debug() const;

    // MAV_SYSID of peer slot n from the RNG_PEER_n roster, 0 if unused
    // MAV_SYSID of peer slot n from the RNG_PEER_n roster, 0 if unused
    uint8_t get_peer_id(uint8_t n) const;

    // DW1000 antenna delay in device ticks (RNG_ANT_DLY)
    uint16_t get_ant_delay() const;

    // tuning parameters
    uint16_t get_poll_ms() const;   // RNG_POLL_MS  - base poll cadence
    uint8_t  get_channel() const;   // RNG_CHAN     - UWB RF channel
    uint16_t get_reply_us() const;  // RNG_REPLY_US - TWR reply delay
    uint16_t get_xchg_ms() const;   // RNG_XCHG_MS  - exchange timeout
    bool     knot_schedule() const; // RNG_SCHED == 1
    uint32_t get_slot_us() const;   // RNG_SLOT_US  - slot length

    // reference to the owning frontend
    AP_Ranging &_frontend;
};

#endif  // AP_RANGING_ENABLED
