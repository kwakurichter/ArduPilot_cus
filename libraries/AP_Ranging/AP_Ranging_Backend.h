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

    // set the measured range to a node in meters. Called by the backend as
    // each TWR exchange completes.
    void set_node_distance(uint8_t node_instance, float distance);

protected:

    // reference to the owning frontend
    AP_Ranging &_frontend;
};

#endif  // AP_RANGING_ENABLED
