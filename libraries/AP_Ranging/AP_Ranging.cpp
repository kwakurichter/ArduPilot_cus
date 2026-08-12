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

#include "AP_Ranging.h"

#if AP_RANGING_ENABLED

#include "AP_Ranging_Backend.h"
#include "AP_Ranging_DW1000.h"

#include <AP_Logger/AP_Logger.h>
#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL &hal;

// table of user settable parameters
const AP_Param::GroupInfo AP_Ranging::var_info[] = {

    // @Param: _TYPE
    // @DisplayName: UWB ranging device type
    // @Description: What type of UWB ranging device is connected
    // @Values: 0:None,1:DW1000
    // @User: Advanced
    AP_GROUPINFO_FLAGS("_TYPE", 0, AP_Ranging, _type, 0, AP_PARAM_FLAG_ENABLE),

    // index 1 was NODE_ID; this node's id is MAV_SYSID now

    // @Param: _DEBUG
    // @DisplayName: UWB ranging debug output
    // @Description: Enables verbose ranging debug messages over MAVLink.
    // @Values: 0:Disabled,1:Enabled
    // @User: Advanced
    AP_GROUPINFO("_DEBUG", 2, AP_Ranging, _debug, 0),

    // index 3 was NUM_NODES; the peer roster below replaced it

    // @Param: _PEER_1
    // @DisplayName: Ranging peer 1 system id
    // @Description: MAV_SYSID of a peer to range against. 0 leaves the slot unused. Only configured peers are polled, so ids need not be contiguous. A peer keeps its reporting slot for the life of the vehicle.
    // @Range: 0 255
    // @User: Standard
    AP_GROUPINFO("_PEER_1", 10, AP_Ranging, _peer_id[0], 0),

    // @Param: _PEER_2
    // @DisplayName: Ranging peer 2 system id
    // @Description: MAV_SYSID of a peer to range against. 0 leaves the slot unused. Only configured peers are polled, so ids need not be contiguous. A peer keeps its reporting slot for the life of the vehicle.
    // @Range: 0 255
    // @User: Standard
    AP_GROUPINFO("_PEER_2", 11, AP_Ranging, _peer_id[1], 0),

    // @Param: _PEER_3
    // @DisplayName: Ranging peer 3 system id
    // @Description: MAV_SYSID of a peer to range against. 0 leaves the slot unused. Only configured peers are polled, so ids need not be contiguous. A peer keeps its reporting slot for the life of the vehicle.
    // @Range: 0 255
    // @User: Standard
    AP_GROUPINFO("_PEER_3", 12, AP_Ranging, _peer_id[2], 0),

    // @Param: _PEER_4
    // @DisplayName: Ranging peer 4 system id
    // @Description: MAV_SYSID of a peer to range against. 0 leaves the slot unused. Only configured peers are polled, so ids need not be contiguous. A peer keeps its reporting slot for the life of the vehicle.
    // @Range: 0 255
    // @User: Standard
    AP_GROUPINFO("_PEER_4", 13, AP_Ranging, _peer_id[3], 0),

    // @Param: _ANT_DLY
    // @DisplayName: UWB antenna delay
    // @Description: DW1000 antenna delay in device time units (~15.65ps each), applied to both TX and RX. Calibrate at a known distance: increasing this REDUCES the reported range. 0 gives a large positive offset; ~16384 is the typical DWM1000 value.
    // @Range: 0 32767
    // @User: Advanced
    // @RebootRequired: True
    AP_GROUPINFO("_ANT_DLY", 4, AP_Ranging, _ant_delay, 16384),

    // @Param: _POLL_MS
    // @DisplayName: UWB poll period
    // @Description: Base interval between initiating a ranging exchange to each neighbour. Random jitter is added on top to de-synchronise nodes.
    // @Range: 10 1000
    // @Units: ms
    // @User: Advanced
    AP_GROUPINFO("_POLL_MS", 5, AP_Ranging, _poll_ms, 50),

    // @Param: _CHAN
    // @DisplayName: UWB RF channel
    // @Description: DW1000 UWB channel. All nodes on the network must use the same channel.
    // @Values: 1:Ch1,2:Ch2,3:Ch3,4:Ch4,5:Ch5,7:Ch7
    // @User: Advanced
    // @RebootRequired: True
    AP_GROUPINFO("_CHAN", 6, AP_Ranging, _channel, 2),

    // @Param: _REPLY_US
    // @DisplayName: UWB TWR reply delay
    // @Description: Delay before sending each TWR reply. Must exceed the servicing latency (~1ms)
    // @Range: 1500 20000
    // @Units: us
    // @User: Advanced
    AP_GROUPINFO("_REPLY_US", 7, AP_Ranging, _reply_us, 3000),

    // @Param: _XCHG_MS
    // @DisplayName: UWB exchange timeout
    // @Description: Abort a stalled ranging exchange after this long and retry on the next cycle.
    // @Range: 5 200
    // @Units: ms
    // @User: Advanced
    AP_GROUPINFO("_XCHG_MS", 8, AP_Ranging, _xchg_ms, 30),

    // @Param: _FWD_PORT
    // @DisplayName: Ranging forward serial port
    // @Description: Serial port number that the UWB peer table is forwarded to as a MAVLink TUNNEL message, so a companion computer can consume ranges without parsing logs. Slots are stable, so a given peer keeps its slot index for the life of the vehicle and can be tracked across messages. The port must already be configured as a MAVLink port. -1 disables forwarding.
    // @Range: -1 9
    // @User: Advanced
    AP_GROUPINFO("_FWD_PORT", 9, AP_Ranging, _fwd_port, -1),

    AP_GROUPEND
};

AP_Ranging::AP_Ranging()
{
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    if (_singleton != nullptr) {
        AP_HAL::panic("AP_Ranging must be singleton");
    }
#endif
    _singleton = this;
    AP_Param::setup_object_defaults(this, var_info);
}

// initialise the AP_Ranging class
void AP_Ranging::init(void)
{
    if (_driver != nullptr) {
        // init called a 2nd time?
        return;
    }

    // create backend
    switch ((Type)_type) {
#if AP_RANGING_DW1000_ENABLED
    case Type::DW1000:
        _driver = NEW_NOTHROW AP_Ranging_DW1000(*this);
        break;
#endif
    case Type::None:
        break;
    }
}

// return true if ranging is enabled
bool AP_Ranging::enabled(void) const
{
    return (_type != Type::None);
}

// return true if the sensor is basically healthy (we are receiving data)
bool AP_Ranging::healthy(void) const
{
    if (!device_ready()) {
        return false;
    }
    return _driver->healthy();
}

// update state. This should be called often from the main loop.
void AP_Ranging::update(void)
{
    if (!device_ready()) {
        return;
    }
    _driver->update();
    send_tunnel();
}

// return the number of peer nodes currently tracked
uint8_t AP_Ranging::count() const
{
    if (!device_ready()) {
        return 0;
    }
    return num_nodes;
}

// return all data for a single node
bool AP_Ranging::get_node_data(uint8_t node_instance, struct NodeState& state) const
{
    if (!device_ready() || node_instance >= num_nodes) {
        return false;
    }
    state = node_state[node_instance];
    return true;
}

// return an individual node's id
uint16_t AP_Ranging::node_id(uint8_t node_instance) const
{
    if (node_instance >= num_nodes) {
        return 0;
    }
    return node_state[node_instance].id;
}

// return node health
bool AP_Ranging::node_healthy(uint8_t node_instance) const
{
    if (node_instance >= num_nodes) {
        return false;
    }
    // a node is only healthy if it also has a recent range
    if (AP_HAL::millis() - node_state[node_instance].distance_update_ms > AP_RANGING_TIMEOUT_MS) {
        return false;
    }
    return node_state[node_instance].healthy;
}

// return measured range to a node in meters
float AP_Ranging::node_distance(uint8_t node_instance) const
{
    if (node_instance >= num_nodes || !node_healthy(node_instance)) {
        return 0.0f;
    }
    return node_state[node_instance].distance;
}

// return last range update time from a node in milliseconds
uint32_t AP_Ranging::node_last_update_ms(uint8_t node_instance) const
{
    if (_type == Type::None || node_instance >= num_nodes) {
        return 0;
    }
    return node_state[node_instance].distance_update_ms;
}

// check if the device is ready
bool AP_Ranging::device_ready(void) const
{
    return ((_driver != nullptr) && (_type != Type::None));
}

#if HAL_LOGGING_ENABLED
// Write UWB ranging data
void AP_Ranging::log()
{
    if (!enabled()) {
        return;
    }

    // gather each fixed slot's node id + last measured range into locals (avoid
    // taking the address of packed struct members), and set the health bit if
    // that node's range is still recent. Ordering is stable because a node id
    // keeps the same slot (see set_node_distance()). The log carries 4 slots.
    const uint8_t LOG_SLOTS = 4;
    uint8_t id[LOG_SLOTS] = {};
    float   d[LOG_SLOTS] = {};
    uint8_t health = 0;
    const uint8_t n = MIN(num_nodes, LOG_SLOTS);
    for (uint8_t i = 0; i < n; i++) {
        id[i] = (uint8_t)node_state[i].id;
        d[i]  = node_state[i].distance;   // raw last range; validity is in Hlth
        if (node_healthy(i)) {
            health |= (1U << i);
        }
    }

    const struct log_Ranging pkt {
        LOG_PACKET_HEADER_INIT(LOG_RANGING_MSG),
        time_us : AP_HAL::micros64(),
        count   : num_nodes,
        health  : health,
        id0 : id[0], id1 : id[1], id2 : id[2], id3 : id[3],
        dist0 : d[0], dist1 : d[1], dist2 : d[2], dist3 : d[3],
    };
    AP::logger().WriteBlock(&pkt, sizeof(pkt));
}
#endif


/*
  Forward the peer table to a companion computer as a MAVLink TUNNEL.

  TUNNEL rather than a purpose-built message because a fork cannot add to the
  common dialect without both ends carrying generated headers, and rather than
  DISTANCE_SENSOR because that means "obstacle at this orientation" and is
  consumed by ArduPilot's own proximity and avoidance code - peer ranges would
  be read as obstacles on a link the flight controller also parses.

  The slot index is carried explicitly rather than implied by array position.
  A node keeps its slot for the life of the vehicle, so the companion can
  difference a peer's range over time, and an explicit index keeps that true
  even if the payload is ever reordered or truncated.
 */
void AP_Ranging::send_tunnel()
{
#if HAL_GCS_ENABLED
    const int8_t port = _fwd_port.get();
    if (port < 0 || !device_ready()) {
        return;
    }

    // 10Hz, matching the log cadence; the ranging cycle is slower than this
    const uint32_t now_ms = AP_HAL::millis();
    if (now_ms - _last_tunnel_ms < 100) {
        return;
    }
    _last_tunnel_ms = now_ms;

    // resolve the port to a channel once, and again only when the parameter moves
    if (port != _fwd_port_resolved) {
        _fwd_port_resolved = port;
        _fwd_chan = gcs().get_channel_from_port_number((uint8_t)port);
    }
    if (_fwd_chan == UINT8_MAX) {
        return;     // not configured as a MAVLink port
    }

    struct PACKED PeerSlot {
        uint8_t slot;        // stable slot index, not implied by position
        uint8_t peer_id;     // 0 when the slot is unused
        uint8_t healthy;     // 1 if this range is recent
        uint8_t reserved;
        float   range_m;
    };
    struct PACKED Payload {
        uint8_t  version;    // bump if the layout below ever changes
        uint8_t  node_id;    // this vehicle's MAV_SYSID
        uint8_t  count;      // slots populated
        uint8_t  reserved;
        PeerSlot peer[AP_RANGING_MAX_NODES];
    } payload {};

    payload.version = 1;
    payload.node_id = gcs().sysid_this_mav();
    payload.count   = num_nodes;

    for (uint8_t i = 0; i < AP_RANGING_MAX_NODES; i++) {
        payload.peer[i].slot = i;
        if (i < num_nodes) {
            payload.peer[i].peer_id = (uint8_t)node_state[i].id;
            payload.peer[i].healthy = node_healthy(i) ? 1 : 0;
            payload.peer[i].range_m = node_state[i].distance;
        }
    }

    static_assert(sizeof(Payload) <= MAVLINK_MSG_TUNNEL_FIELD_PAYLOAD_LEN,
                  "ranging tunnel payload does not fit a TUNNEL message");

    if (!HAVE_PAYLOAD_SPACE((mavlink_channel_t)_fwd_chan, TUNNEL)) {
        return;
    }

    uint8_t buf[MAVLINK_MSG_TUNNEL_FIELD_PAYLOAD_LEN] {};
    memcpy(buf, &payload, sizeof(payload));

    mavlink_msg_tunnel_send((mavlink_channel_t)_fwd_chan,
                            0, 0,               // broadcast: any listener on this port
                            AP_RANGING_TUNNEL_PAYLOAD_TYPE,
                            sizeof(payload),
                            buf);
#endif  // HAL_GCS_ENABLED
}

// singleton instance
AP_Ranging *AP_Ranging::_singleton;

namespace AP {

AP_Ranging *ranging()
{
    return AP_Ranging::get_singleton();
}

}

#endif  // AP_RANGING_ENABLED
