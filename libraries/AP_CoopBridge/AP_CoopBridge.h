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
    COOP_FLOW,        per     one message per sensor reading, handed over by the drivers
    COOP_TOF,         reading (push_flow() etc., AP_CoopBridge_Readings.h)
    COOP_BARO, COOP_MAG, COOP_GLOBAL_FIX (sparse global fixes: motion capture through
                      ATT_POS_MOCAP, push_mocap(); SITL: the SIM_COOP_FIX stand-in)
    COOP_UWB          per     every DS-TWR exchange this vehicle took part in, taken from
                      exchange AP_Ranging (both ends with RNG_SCHED = 1)
    COOP_LINK_STATUS    1 Hz  health of the cooperative path

  COOP_PEER, the companion's cooperative messages, goes both ways: from the companion it is
  handed to AP_SwarmMesh's critical class (handle_coop_peer()), and AP_SwarmMesh writes the
  peers' frames for this vehicle straight to the companion port (P2P_FWD_PORT = COOP_PORT,
  P2P_FWD_MASK bit 1). The companion's replies (ODOMETRY for ExternalNav, DEBUG_VECT rows for
  the Lua avoidance) use the existing GCS handlers.

  The port is an ordinary MAVLink2 port (SERIALn_PROTOCOL = 2) selected by COOP_PORT; the
  bridge sends on its channel with the vehicle's system and component ID. GCS streams on that
  port should be turned off (SRn_* = 0) to leave the bandwidth to the bridge.

  Global fixes (docs/plans/p5_global_fix_plan.md in the thesis repository): with
  COOP_FIX_SRC = 1, GCS_MAVLINK::handle_att_pos_mocap hands ATT_POS_MOCAP to push_mocap()
  instead of AP_VisualOdom, so EKF3's ExternalNav carries only the companion's ODOMETRY.
  The capture time is ATT_POS_MOCAP.time_usec when the sender synchronised it to this FC's
  micros64() (COOP_FIX_TIME = 1; swarm_hub does it with TIMESYNC), else the arrival minus
  COOP_FIX_DELAY. Fixes are thinned to COOP_FIX_HZ, knot-aligned: the first fix captured at
  or after a 100 ms knot that is due one, and (COOP_FIX_SEL = 1) only if it reaches the
  companion before the next knot (requirement R-FIX-1). COOP_FIX_HOLD delays every fix, to
  emulate a slow frontend.

  Frames: the INS works in body FRD, the estimator in body FLU (x forward, y left, z up), so
  vectors are sent as (x, -y, -z). Times are mesh time (AP_SwarmMesh::mesh_micros64(): the
  FC's micros64() plus the offset to the swarm's reference clock), so they agree across
  vehicles; driver stamps are converted when sent.
 */
#pragma once

#include "AP_CoopBridge_config.h"

#if AP_COOPBRIDGE_ENABLED

#include <AP_Param/AP_Param.h>
#include <GCS_MAVLink/GCS_MAVLink.h>
#include <AP_HAL/utility/RingBuffer.h>
#include "AP_CoopBridge_ImuStage.h"
#include "AP_CoopBridge_Readings.h"
#include <AP_Ranging/AP_Ranging_config.h>
#if AP_RANGING_ENABLED
#include <AP_Ranging/AP_Ranging.h>
#endif

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

    // Hand over one sensor reading (any thread; one producer per sensor type). Queued and
    // sent from update_fast(), before the IMU sub-intervals of the same tick, so a reading
    // reaches the companion ahead of the IMU data that moves its clock past the reading.
    // Returns false if the queue is full or the bridge is disabled (the reading is lost).
    bool push_flow(const AP_CoopReading::Flow &r);
    bool push_tof(const AP_CoopReading::Tof &r);
    bool push_baro(const AP_CoopReading::Baro &r);
    bool push_mag(const AP_CoopReading::Mag &r);
    bool push_fix(const AP_CoopReading::GlobalFix &r);

    // ATT_POS_MOCAP for this vehicle (body FRD in NED), received at micros64() ``arrival_us``.
    // Converted, timed, selected and sent as COOP_GLOBAL_FIX (see the file comment).
    void push_mocap(const mavlink_att_pos_mocap_t &m, uint64_t arrival_us);
    // True when ATT_POS_MOCAP belongs to the bridge rather than AP_VisualOdom (COOP_FIX_SRC).
    bool takes_mocap() const { return _port.get() >= 0 && _fix_src.get() == 1; }

    // COOP_PEER received on MAVLink channel ``chan`` (GCS_MAVLINK::handle_message): if it
    // came from the companion port, hand the frame to AP_SwarmMesh's critical class.
    void handle_coop_peer(mavlink_channel_t chan, const mavlink_message_t &msg);

    // Counters for COOP_LINK_STATUS, the COOP log message and tests.
    struct Stats {
        uint32_t imu_sent;      // COOP_IMU_DELTA sent
        uint32_t imu_dropped;   // completed sub-intervals not sent (no space on the port)
        uint32_t imu_gaps;      // ticks flagged COOP_SAMPLE_GAP (missing or late INS data)
        uint32_t imu_clipped;   // ticks flagged COOP_SAMPLE_CLIPPED
        uint32_t aids_sent;     // COOP_FLOW/TOF/BARO/MAG/GLOBAL_FIX/UWB sent
        uint32_t aids_dropped;  // readings lost (queue full or no space on the port)
        uint32_t peer_in;       // COOP_PEER frames from the companion
        uint32_t peer_rejected; // of them, not taken by the mesh (not running or dropped)
        uint32_t fix_in;        // mocap poses handed to push_mocap()
        uint32_t fix_thinned;   // of them, not due (COOP_FIX_HZ) or the hold queue was full
        uint32_t fix_late;      // dropped by R-FIX-1 selection (would reach the next knot)
        uint32_t fix_time_fallback; // capture time from arrival - COOP_FIX_DELAY
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
    // Send every queued reading (flow, ToF, baro, mag, global fix) and UWB exchange.
    void send_aids();
    void send_flow(const AP_CoopReading::Flow &r);
    void send_tof(const AP_CoopReading::Tof &r);
    void send_baro(const AP_CoopReading::Baro &r);
    void send_mag(const AP_CoopReading::Mag &r);
    void send_global_fix(const AP_CoopReading::GlobalFix &r);
    // Release held mocap fixes whose COOP_FIX_HOLD has passed (R-FIX-1 check, then send).
    void release_mocap(uint64_t now_us);
#if AP_RANGING_ENABLED
    void send_uwb(const AP_Ranging::Exchange &e);
#endif
#if AP_SIM_COOP_SENSORS_ENABLED
    // SITL stand-in for motion capture: the SIM_COOP_FIX poses, through push_mocap()
    void sitl_fixes();
#endif
    // Send COOP_LINK_STATUS.
    void send_link_status(uint64_t now_us);
#if HAL_LOGGING_ENABLED
    // Write the COOP log message (counters, once a second).
    void log_stats(uint64_t now_us);
#endif

    // Parameters
    AP_Int8 _port;              // serial port number of the companion, -1 = disabled
    AP_Float _flow_c[4];        // flow counts to angle (rad/count), row-major 2x2
    AP_Float _flow_sd;          // flow count noise (counts), quantisation added
    AP_Float _tof_sd;           // ToF range noise (m)
    AP_Float _baro_sd;          // baro pressure noise per reading (Pa)
    AP_Float _mag_sd;           // magnetometer direction noise per axis (rad)
    AP_Float _mag_norm;         // surveyed field strength (driver units), 0 = no norm check
    AP_Int8 _fix_src;           // 1: ATT_POS_MOCAP goes to the bridge, not AP_VisualOdom
    AP_Float _fix_hz;           // global fixes forwarded per second (knot-aligned), 0 = none
    AP_Int8 _fix_dof;           // 6, 4 or 3
    AP_Int8 _fix_time;          // 1: time_usec is the capture in micros64()
    AP_Int16 _fix_delay_ms;     // fallback arrival minus capture
    AP_Float _fix_p_sd;         // position standard deviation (m)
    AP_Float _fix_r_sd;         // attitude / yaw standard deviation (deg)
    AP_Int8 _fix_sel;           // 1: R-FIX-1 selection (drop fixes that would be late)
    AP_Int16 _fix_hold_ms;      // extra hold before forwarding

    // Readings waiting for update_fast(), one queue per sensor (single producer each)
    ObjectBuffer<AP_CoopReading::Flow> _flow_q{8};
    ObjectBuffer<AP_CoopReading::Tof> _tof_q{4};
    ObjectBuffer<AP_CoopReading::Baro> _baro_q{4};
    ObjectBuffer<AP_CoopReading::Mag> _mag_q{8};
    ObjectBuffer<AP_CoopReading::GlobalFix> _fix_q{4};

    // Mocap fixes waiting for COOP_FIX_HOLD, oldest first, and the selection state.
    static constexpr uint8_t FIX_HOLD_MAX = 4;
    struct HeldFix {
        AP_CoopReading::GlobalFix fix;
        uint64_t release_us;    // micros64()
    };
    HeldFix _fix_held[FIX_HOLD_MAX];
    uint8_t _n_fix_held;
    uint64_t _fix_next_knot;    // first knot (index, mesh time / 100 ms) due a fix

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
