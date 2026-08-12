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
        // TODO: add SITL
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
