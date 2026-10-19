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
    void set_node_distance(uint8_t node_id, float distance, uint32_t time_ms);
    void set_anchor_distance(uint8_t id, float distance, uint32_t time_ms);

protected:

    AP_Ranging::Mode get_mode() const { return _frontend.mode(); }
    uint16_t get_network_id() const { return (uint16_t)_frontend._network_id.get(); }
    bool anchor_enabled(uint8_t i) const { return _frontend.anchor(i).enabled; }

    // this vehicle's MAV_SYSID
    uint8_t get_node_id() const;

    // diagnostic logging (RNG_DEBUG); >0 enables health and timestamp logs
    int8_t get_debug() const;
#if AP_RANGING_RF_DIAGNOSTICS_ENABLED
    uint8_t get_rf_diag_mask() const { return uint8_t(_frontend._rf_diag_mask.get()); }
#endif

    // MAV_SYSID of peer slot n from the RNG_PEER_n roster, 0 if unused
    uint8_t get_peer_id(uint8_t n) const;

    // DW1000 antenna delay in device ticks (RNG_ANT_DLY)
    uint16_t get_ant_delay() const;

    // tuning parameters
    uint16_t get_poll_ms() const;   // RNG_POLL_MS  - base poll cadence
    uint8_t  get_channel() const;   // RNG_CHAN     - UWB RF channel
    uint16_t get_reply_us() const;  // RNG_REPLY_US - TWR reply delay
    uint16_t get_xchg_ms() const;   // RNG_XCHG_MS  - exchange timeout

    // reference to the owning frontend
    AP_Ranging &_frontend;
};

#endif  // AP_RANGING_ENABLED
