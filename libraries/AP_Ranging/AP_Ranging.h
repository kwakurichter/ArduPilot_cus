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

#include "AP_Ranging_config.h"

#if AP_RANGING_ENABLED

#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>
#include <AP_HAL/utility/RingBuffer.h>

class AP_Ranging_Backend;

class AP_Ranging
{
public:
    friend class AP_Ranging_Backend;

    AP_Ranging();

    CLASS_NO_COPY(AP_Ranging);

    // get singleton instance
    static AP_Ranging *get_singleton() { return _singleton; }

    // ranging backend types (used by the _TYPE parameter)
    enum class Type : uint8_t {
        None   = 0,
        DW1000 = 1,
        SITL   = 100,  // SITL instances ranging each other (AP_Ranging_SITL)
    };

    /*
      One completed DS-TWR exchange this vehicle took part in, recorded at both ends: by the
      responder when the FINAL arrives (it computes the range) and, in the knot-aligned
      schedule, by the initiator when the responder's REPORT arrives. Times are FC time.
      AP_CoopBridge sends each one to the companion as COOP_UWB.
     */
    struct Exchange {
        uint64_t start_us;           // exchange start: the POLL on air (initiator) or received (responder)
        uint32_t duration_us;        // start to the end of this vehicle's part (FINAL or REPORT received)
        float    range_m;            // antenna-delay-corrected range
        float    variance_m2;        // RNG_SIGMA^2
        float    rx_power_dbm;       // total receive power of the FINAL frame (at the responder)
        float    fp_power_dbm;       // first-path power of the FINAL frame
        uint8_t  initiator;          // MAV_SYSID of the initiator
        uint8_t  responder;          // MAV_SYSID of the responder
        bool     power_valid;        // the two powers are measured
    };

    struct NodeState {
        uint16_t id;                 // unique id of the peer node
        bool     healthy;            // true if we have a recent range to this node
        float    distance;           // measured range to the node (meters)
        uint32_t distance_update_ms; // system time of the last range update
    };

    // initialise the ranging backend selected by the _TYPE parameter
    void init(void);

    // return true if ranging is enabled
    bool enabled(void) const;

    // return true if the sensor is basically healthy (we are receiving data)
    bool healthy(void) const;

    // update state of all nodes. Should be called often from the main loop.
    void update(void);

    // return the number of peer nodes currently tracked
    uint8_t count() const;

    // return all data for a single node
    bool get_node_data(uint8_t node_instance, struct NodeState& state) const;

    // return an individual node's id
    uint16_t node_id(uint8_t node_instance) const;

    // return node health
    bool node_healthy(uint8_t node_instance) const;

    // return measured range to a node in meters
    float node_distance(uint8_t node_instance) const;

    // return last range update time from a node in milliseconds
    uint32_t node_last_update_ms(uint8_t node_instance) const;

    // Take the oldest completed exchange (single consumer: AP_CoopBridge, main thread).
    bool pop_exchange(Exchange &e) { return _exchanges.pop(e); }

    // exchanges lost because nobody took them from the ring
    uint32_t exchanges_lost() const { return _exchanges_lost; }

    static const struct AP_Param::GroupInfo var_info[];

    // a method for vehicles to call to make onboard log messages
    void log();

    /*
      Emit the peer table to the companion computer as a MAVLink TUNNEL on the
      port named by RNG_FWD_PORT. Rate limited internally and deliberately not
      tied to the log bitmask: forwarding is a live data path, and coupling it
      to logging means it silently stops when the bitmask is trimmed.
     */
    void send_tunnel();

private:

    static AP_Ranging *_singleton;

    // check if the device is ready
    bool device_ready(void) const;

    // parameters
    AP_Enum<Type> _type;
    AP_Int8       _debug;     // debug verbosity (RNG_DEBUG): 0=off
    AP_Int16      _peer_id[AP_RANGING_MAX_NODES]; // peers to range against (RNG_PEER_n), 0 = unused
    AP_Int16      _ant_delay; // DW1000 antenna delay, device ticks (RNG_ANT_DLY)
    AP_Int16      _poll_ms;   // base poll cadence, ms (RNG_POLL_MS)
    AP_Int8       _channel;   // UWB RF channel (RNG_CHAN)
    AP_Int16      _reply_us;  // TWR reply delay, us (RNG_REPLY_US)
    AP_Int16      _xchg_ms;   // exchange timeout, ms (RNG_XCHG_MS)
    AP_Int8       _fwd_port;  // serial port to forward the peer table to (RNG_FWD_PORT), -1 off
    AP_Int8       _sched;     // 1 = knot-aligned pair schedule (RNG_SCHED), 0 = jittered polling
    AP_Int32      _slot0_us;  // first slot's start after the knot, us (RNG_SLOT0_US)
    AP_Int32      _slot_us;   // slot length, us (RNG_SLOT_US)
    AP_Float      _sigma;     // range standard deviation reported with each exchange, m (RNG_SIGMA)

    // completed exchanges (producer: the backend's thread, consumer: AP_CoopBridge)
    ObjectBuffer<Exchange> _exchanges{AP_RANGING_EXCHANGE_RING};
    uint32_t _exchanges_lost;

    // resolved once per RNG_FWD_PORT change rather than per send
    int8_t   _fwd_port_resolved = -2;   // -2 == never resolved
    uint8_t  _fwd_chan = UINT8_MAX;     // UINT8_MAX == port is not a MAVLink port
    uint32_t _last_tunnel_ms;

    // backend driver
    AP_Ranging_Backend *_driver;

    // per-node data
    uint8_t num_nodes = 0;
    NodeState node_state[AP_RANGING_MAX_NODES];
};

namespace AP {
    AP_Ranging *ranging();
};

#endif  // AP_RANGING_ENABLED
