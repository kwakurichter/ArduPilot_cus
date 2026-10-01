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
/*
  AP_CoopBridge: the FC end of the cooperative-estimation link to a companion computer.

  The companion (a Teensy 4.0 next to the Crazyflie 2.1's STM32F405) runs the cooperative
  estimator. The FC feeds it over a MAVLink2 serial port with the arduswarm_coop messages
  (modules/mavlink/message_definitions/v1.0/arduswarm_coop.xml):

    COOP_IMU_DELTA    100 Hz  first stage of the IMU preintegration (AP_CoopBridge_ImuStage)
    COOP_LINK_STATUS    1 Hz  health of the cooperative path

  Still to come (thesis repo, embedded/ardupilot/required_changes.md): the per-reading aid
  messages (COOP_FLOW/TOF/BARO/MAG, COOP_UWB), and COOP_PEER, which is handed to
  AP_SwarmMesh's critical class. The companion's replies (ODOMETRY for ExternalNav, DEBUG_VECT
  rows for the Lua avoidance) use the existing GCS handlers.

  The port is an ordinary MAVLink2 port (SERIALn_PROTOCOL = 2) selected by COOP_PORT; the
  bridge sends on its channel with the vehicle's system and component ID. GCS streams on that
  port should be turned off (SRn_* = 0) to leave the bandwidth to the bridge.

  Frames: the INS works in body FRD, the estimator in body FLU (x forward, y left, z up), so
  vectors are sent as (x, -y, -z). Times are FC microseconds (AP_HAL::micros64).
 */
#pragma once

#include "AP_CoopBridge_config.h"

#if AP_COOPBRIDGE_ENABLED

#include <AP_Param/AP_Param.h>
#include <GCS_MAVLink/GCS_MAVLink.h>
#include "AP_CoopBridge_ImuStage.h"

class AP_CoopBridge
{
public:
    AP_CoopBridge();

    CLASS_NO_COPY(AP_CoopBridge);

    static AP_CoopBridge *get_singleton() { return _singleton; }

    static const struct AP_Param::GroupInfo var_info[];

    // Called once at startup, after the INS is initialised.
    void init();

    // Called every main-loop tick, right after the INS update: integrates the tick's delta
    // angle and velocity and sends every sub-interval it completes.
    void update_fast();

    // Called at a low rate (scheduler, 10 Hz): link status and logging once a second.
    void update();

    // Counters for COOP_LINK_STATUS, the COOP log message and tests.
    struct Stats {
        uint32_t imu_sent;      // COOP_IMU_DELTA sent
        uint32_t imu_dropped;   // completed sub-intervals not sent (no space on the port)
        uint32_t imu_gaps;      // ticks flagged COOP_SAMPLE_GAP (missing or late INS data)
        uint32_t imu_clipped;   // ticks flagged COOP_SAMPLE_CLIPPED
        // (all zero at startup: the object is a static member of the vehicle's parameters)
    };
    const Stats &stats() const { return _stats; }

private:
    static AP_CoopBridge *_singleton;

    // Resolve COOP_PORT to a MAVLink channel (cached until the parameter changes). False if
    // the bridge is disabled or the port is not a MAVLink port.
    bool resolve_channel();
    // Send one completed sub-interval as COOP_IMU_DELTA.
    void send_imu(const AP_CoopBridge_ImuStage::Sub &sub);
    // Send COOP_LINK_STATUS.
    void send_link_status(uint64_t now_us);
#if HAL_LOGGING_ENABLED
    // Write the COOP log message (counters, once a second).
    void log_stats(uint64_t now_us);
#endif

    // Parameters
    AP_Int8 _port;              // serial port number of the companion, -1 = disabled

    // Port resolution
    int8_t _port_resolved = -2; // value of _port the channel was resolved for
    uint8_t _chan = UINT8_MAX;  // MAVLink channel of the port, UINT8_MAX = none

    // IMU first stage
    AP_CoopBridge_ImuStage _imu;
    bool _imu_locked;           // _imu_instance chosen (cleared by init())
    uint8_t _imu_instance;      // INS instance used for both gyro and accel
    uint64_t _last_tick_us;     // end of the previous tick, 0 before the first
    uint32_t _last_clip_count;  // accel clip count at the previous tick

    // Low-rate work
    uint64_t _last_status_us;   // last COOP_LINK_STATUS
    Stats _stats;
};

namespace AP {
    AP_CoopBridge *coopbridge();
};

#endif  // AP_COOPBRIDGE_ENABLED
