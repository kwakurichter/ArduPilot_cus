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

extern const AP_HAL::HAL &hal;

// table of user settable parameters
const AP_Param::GroupInfo AP_Ranging::var_info[] = {

    // @Param: _TYPE
    // @DisplayName: UWB ranging device type
    // @Description: What type of UWB ranging device is connected
    // @Values: 0:None,1:DW1000
    // @User: Advanced
    AP_GROUPINFO_FLAGS("_TYPE", 0, AP_Ranging, _type, 0, AP_PARAM_FLAG_ENABLE),

    // @Param: _NODE_ID
    // @DisplayName: UWB node address
    // @Description: This vehicle's UWB node address. Each ranging node on the network must have a unique id.
    // @Range: 0 255
    // @User: Advanced
    AP_GROUPINFO("_NODE_ID", 1, AP_Ranging, _node_id, 0),

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

    const struct log_Ranging pkt{
        LOG_PACKET_HEADER_INIT(LOG_RANGING_MSG),
        time_us         : AP_HAL::micros64(),
        health          : (uint8_t)healthy(),
        count           : (uint8_t)count(),
        dist0           : node_distance(0),
        dist1           : node_distance(1),
        dist2           : node_distance(2),
        dist3           : node_distance(3),
    };
    AP::logger().WriteBlock(&pkt, sizeof(pkt));
}
#endif

// singleton instance
AP_Ranging *AP_Ranging::_singleton;

namespace AP {

AP_Ranging *ranging()
{
    return AP_Ranging::get_singleton();
}

}

#endif  // AP_RANGING_ENABLED
