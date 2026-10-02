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

#include "AP_SwarmMesh_config.h"

#if AP_SWARMMESH_ENABLED
#include <AP_Common/AP_Common.h>
#include <AP_Common/Location.h>
#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>
#include <AP_Filesystem/AP_Filesystem_config.h>
#include <GCS_MAVLink/GCS_MAVLink.h>
#include <AP_HAL/Semaphores.h>
#include "AP_SwarmMesh_TimeSync.h"

class AP_SwarmMesh_Backend;

// Counters of the critical class (COOP_PEER), cumulative except queued / queued_max.
struct SwarmCriticalStats {
    uint32_t tx;            // frames put on the mesh (own and relayed)
    uint32_t rx;            // frames for this vehicle received from the mesh
    uint32_t tx_dropped;    // dropped here: queue full or deadline expired
    uint32_t fwd_dropped;   // received but not written to the companion port (full)
    uint8_t  queued;        // waiting now
    uint8_t  queued_max;    // most waiting at once since the maximum was restarted
};

#if AP_SWARMMESH_COORD_ENABLED
/*
  Coordination state published by a Lua script or a companion computer, and read back for each peer.
  The library assigns no meaning to any of these values beyond carrying them (what a role or a task
  id means is entirely up to your automated scripts).

  Fields are flat scalars because this is bound directly into Lua as a userdata.
 */
struct SwarmCoordState {
    uint8_t role;
    uint8_t task_id;
    uint8_t formation_slot;
    uint8_t priority;
    int32_t target_lat;             // degE7
    int32_t target_lng;             // degE7
    int32_t target_alt_mm;          // mm, AMSL
    int16_t target_vel_NED[3];      // cm/s
    int16_t target_accel_NED[3];    // cm/s/s
    uint8_t user[AP_SWARMMESH_COORD_USER_MAX];  // opaque to the library
    uint8_t user_len;               // number of valid bytes in user[]
};
#endif  // AP_SWARMMESH_COORD_ENABLED

class AP_SwarmMesh
{
public:
    friend class AP_SwarmMesh_Backend;

    AP_SwarmMesh();

    /* Do not allow copies */
    CLASS_NO_COPY(AP_SwarmMesh);

    // get singleton instance
    static AP_SwarmMesh *get_singleton() { return _singleton; }

    // external hardware backend types (used by _TYPE parameter)
    enum class Type : uint8_t {
        None    = 0,
        Serial  = 1,
        Syslink = 2,
#if AP_SWARMMESH_SITL_ENABLED
        SITL    = 10
#endif
    };

    // Bit positions in PeerState.freshness, one per tracked message type. Mirrors AP_SwarmMesh_Backend::LogMsg
    // TODO: single assignment governs both dataflash logging and type freshness.
    enum class MsgFresh : uint8_t {
        HEARTBEAT                  = 0,
        SYS_STATUS                 = 1,
        GLOBAL_POSITION_INT        = 2,
        LOCAL_POSITION_NED         = 3,
        POSITION_TARGET_GLOBAL_INT = 4,
        EXTENDED_SYS_STATE         = 5,
        ATTITUDE                   = 6,
        EKF_STATUS_REPORT          = 7,
        SCALED_IMU                 = 8,
#if AP_SWARMMESH_COORD_ENABLED
        COORDINATION               = 9,
#endif
        NUM_TYPES                  // keep last
    };
    static constexpr uint8_t NUM_FRESH_TYPES = (uint8_t)MsgFresh::NUM_TYPES;

    // The AP_SwarmMesh structure is filled in by the backend driver
    struct PeerState {
        // Peer identity
        uint8_t  sysid;         // unique ID of original peer
        uint8_t  vehicle_type;  // 0: copter, 1: plane, 2: sub, 3: blimp, 4: rover
        uint8_t  prev_id;       // ID of peer which forwarded message
        // Liveness / Link quality
        uint16_t last_seq;      // for dedup ring buffer
        uint32_t seq_seen_mask; // bitmask of the 32 seq numbers behind last_seq
        uint8_t  rssi;          // signal strength
        uint16_t rx_count;      // received message count
        uint16_t drop_count;    // dropped message count
        uint32_t last_heard_ms[NUM_FRESH_TYPES];    // type freshness. last_heard_ms[t] is the local time of the last received message of type t; freshness has bit t set while that is within the type's budget.
        uint32_t freshness;     // bitmask of fresh message types (bit = MsgFresh). freshness == 0 means the peer is dead/stale.
        // Kinematic state
        Vector3f local_pos_NED; // offset from origin [x, y, z] in meters
        Vector3l global_pos;    // GPS [lat (degE7), lon (degE7), alt (mm)]
        int16_t  velocity[3];   // actual velocity [x, y, z] NED, cm/s. From LOCAL_POSITION_NED or GLOBAL_POSITION_INT; global overrides local when both arrive in the same TX cycle
        int16_t  accel[3];      // actual acceleration [x, y, z] body frame, mG, EKF bias removed. From SCALED_IMU
        float    pos_horiz_variance; // EKF horizontal position variance, from EKF_STATUS_REPORT
        float    pos_vert_variance;  // EKF vertical position variance, from EKF_STATUS_REPORT
        float    vel_variance;       // EKF velocity variance, from EKF_STATUS_REPORT
        Vector3f attitude;      // [pitch, roll, yaw] in rads
        // Vehicle state
        uint8_t  mode;
        bool     armed_state;   // true: armed, false: disarmed
        uint8_t  landed_state;  // MAV_LANDED_STATE: 0 undefined, 1 on ground, 2 in air, 3 takeoff, 4 landing
        uint32_t failsafe_flags; // bitmask of unhealthy sensors (present & enabled & !health), MAV_SYS_STATUS_SENSOR bits
        uint16_t battery_voltage;
        uint8_t  health_flags;
        // Coordination state
        uint8_t  role;
        uint8_t  task_id;
        uint8_t  formation_slot;
        Vector3l target_pos;    // [lat (degE7), lon (degE7), alt (mm)]
        int16_t  target_velocity[3]; // commanded velocity [x, y, z] NED, cm/s, from POSITION_TARGET_GLOBAL_INT
        int16_t  target_accel[3];    // commanded acceleration [x, y, z] NED, cm/s/s, from POSITION_TARGET_GLOBAL_INT
        uint8_t  priority;
#if AP_SWARMMESH_COORD_ENABLED
        uint8_t  coord_user[AP_SWARMMESH_COORD_USER_MAX]; // script defined payload from this peer's coordination basket
        uint8_t  coord_user_len;     // valid bytes in coord_user. A longer basket than we can hold is truncated to this
#endif
    };

    // initialise
    void init(void);

    // return true if mesh feature is enabled
    bool enabled(void) const;

    // return true if mesh is basically healthy (we are receiving data)
    bool healthy(void) const;

    // update state of all peers
    void update(void);

    // return number of known peers
    uint8_t count() const;

    // return data for a specific peer by index
    bool get_peer_data(uint8_t peer_id, struct PeerState& state) const;

    // return the sysid held at a peer table index, or 0 if that index is not filled
    uint8_t get_peer_sysid(uint8_t index) const;

    // fill loc with peer's last global pos, returns false if the peer is unknown or its entry is stale.
    bool get_peer_location(Location& loc, uint8_t peer_sysid) const;

    // fill vel_ned with the peer's last vel, returns false if the peer is unknown or its entry is stale.
    bool get_peer_velocity_NED(Vector3f& vel_ned, uint8_t peer_sysid) const;

#if AP_SWARMMESH_COORD_ENABLED
    // publish our own coordination state to the swarm. It is broadcast at _SR_COORD Hz until it is replaced by another call. Returns false if the mesh is not running.
    bool set_coord_state(const SwarmCoordState &state);

    // fill state with the coordination state a peer last published. Returns false if the peer is unknown or has not published within the freshness budget.
    bool get_peer_coord_state(SwarmCoordState &state, uint8_t peer_sysid) const;

    // handle a TUNNEL received on an ordinary telemetry link, so a companion computer can publish coordination state the same way a script does. Returns true if it was ours.
    bool handle_tunnel(const mavlink_message_t &msg);
#endif

    // Send one MAVLink2 frame (COOP_PEER from the companion) in the critical class: ahead
    // of all telemetry, never rate limited, retried until ``deadline_ms`` (0 = none), to
    // ``dest_id`` (0 = broadcast). False if the mesh is not running or it was dropped.
    bool send_critical(const uint8_t *frame, uint16_t len, uint8_t dest_id, uint16_t deadline_ms);

    // Critical class counters (COOP_LINK_STATUS); false if the mesh is not running.
    bool get_critical_stats(SwarmCriticalStats &stats, bool restart_max);

    /*
      Mesh clock. A disarmed vehicle with _SR_SYNC > 0 is a reference: it broadcasts its
      mesh time in SWARMMESH_TYPE_SYNC packets. Every other vehicle follows the lowest system
      ID it hears and keeps an offset to it (AP_SwarmMesh_TimeSync), so that mesh time is the
      same on every vehicle. Mesh time is micros64() plus that offset; before any reference
      has been heard, and without a mesh, it is micros64(). Any thread.

      The cooperative estimator's time stamps (AP_CoopBridge) and the UWB schedule
      (AP_Ranging) use it. Followers should be synced before they arm: in flight the offset
      only slews (_SYNC_SLEW), on the ground it may step.
     */
    static uint64_t mesh_micros64();
    // a local micros64() time in mesh time (with the current offset)
    static uint64_t to_mesh_us(uint64_t local_us);
    // offset from micros64() to mesh time (us)
    int64_t mesh_offset_us() const;

    struct SyncStatus {
        bool     is_reference;  // broadcasting (disarmed and _SR_SYNC > 0)
        bool     locked;        // following a reference heard within the timeout
        uint8_t  reference;     // system ID followed, 0 = none
        int64_t  offset_us;     // applied offset
        int64_t  target_us;     // the estimator's current target
        uint32_t samples;       // SYNC packets used
    };
    bool get_sync_status(SyncStatus &status) const;

    // _FWD_MASK bits: which received frames go to _FWD_PORT
    static constexpr uint8_t FWD_TELEMETRY = 1U << 0;
    static constexpr uint8_t FWD_CRITICAL  = 1U << 1;

    static const struct AP_Param::GroupInfo var_info[];

    // a method for vehicles to call to make onboard log messages:
    void log();

    // number of SR stream buckets (must match the SR_* param entries in var_info); the
    // clock sync bucket (_SR_SYNC) is always the last
#if AP_SWARMMESH_COORD_ENABLED
    static constexpr uint8_t NUM_BUCKETS = 5;
#else
    static constexpr uint8_t NUM_BUCKETS = 4;
#endif
    static constexpr uint8_t SYNC_BUCKET = NUM_BUCKETS - 1;

private:

    // return true if driver is instantiated and type is not None
    bool device_ready(void) const;

    // find an existing peer entry by sysid, or allocate a new one.
    // returns nullptr if the table is full and the peer is not already present.
    PeerState *find_or_alloc_peer(uint8_t peer_sysid);

    // find an existing peer entry by sysid without allocating. returns nullptr if absent.
    const PeerState *find_peer_by_sysid(uint8_t peer_sysid) const;

    // returns true if peer_sysid is allowed by the neighbourhood filter.
    // when all _PEER_* slots are 0 (default), every sysid is allowed.
    bool peer_is_allowed(uint8_t peer_sysid) const;

#if AP_FILESYSTEM_FILE_WRITING_ENABLED
    // periodically rewrite the on-disk peer-table snapshot (filled + fresh entries only)
    void save_peer_snapshot();

    // called once from init(): restore the peer table from the on-disk snapshot, if one exists and is valid. Restored peers always start with freshness == false.
    void load_peer_snapshot();
#endif

    // periodically deletes stale peer table entries
    void prune_peer_table();

    // mesh clock (backend side): true while this vehicle broadcasts SYNC
    bool sync_is_reference() const;
    // a SYNC from ``ref_id`` with its mesh time ``ref_us``, received at local ``local_us``
    void sync_sample(uint8_t ref_id, uint64_t ref_us, uint64_t local_us);
    // advance the applied offset (main loop)
    void sync_update();
    AP_SwarmMesh_TimeSync::Config sync_config() const;

    static AP_SwarmMesh *_singleton;

    // parameters
    AP_Enum<Type> _type;
    AP_Int8  stream_rate[NUM_BUCKETS];  // SR_POSITION, SR_EXT_STAT, SR_EXTRA1 (Hz; 0 = disabled)
    AP_Int16 swarm_size;
    AP_Int8  destination_id;
    AP_Int8  ttl;
    AP_Int8  hardware_mask;
    AP_Int16 log_rate_hz;  // max combined rate (Hz) of RX dataflash log writes; 0 disables
    AP_Int32 log_mask;     // bitmask of which RX message types are logged (see AP_SwarmMesh_Serial::LogMsg)
#if AP_FILESYSTEM_FILE_WRITING_ENABLED
    AP_Int8  save_rate_hz; // rate (Hz) at which the on-disk peer snapshot is rewritten; 0 disables
#endif
    AP_Int8  prune_timeout;
    AP_Int8  peer_filter[AP_SWARMMESH_MAX_PEER_FILTERS]; // neighbourhood allowlist. 0 = slot unused
    AP_Int8  fwd_port;     // serial port number received peer MAVLink is forwarded to. -1 disables
    AP_Int8  fwd_mask;     // FWD_TELEMETRY | FWD_CRITICAL: which received frames are forwarded
    AP_Int8  critical_depth; // critical queue slots (1..AP_SWARMMESH_CRITICAL_DEPTH_MAX)
    AP_Int32 sync_latency_us; // fixed SYNC delay the minimum filter cannot see (us)
    AP_Int16 sync_slew_ppm;   // largest rate of change of the mesh offset in flight (ppm)

    // mesh clock: the estimator (main thread) and its applied offset for any thread
    AP_SwarmMesh_TimeSync _sync;
    mutable HAL_Semaphore _sync_sem;
    int64_t _sync_offset_us;
    bool    _sync_was_locked;

    // external references
    AP_SwarmMesh_Backend *_driver;

#if AP_SWARMMESH_COORD_ENABLED
    // coordination state we publish, as last set by a script or companion computer. nothing is broadcast until it has been set at least once.
    SwarmCoordState _coord_state;
    bool _coord_state_set;
#endif

    // individual peer data
    uint8_t num_peers = 0;
    PeerState peer_state[AP_SWARMMESH_MAX_PEERS];

#if AP_FILESYSTEM_FILE_WRITING_ENABLED
    uint32_t _last_save_ms;     // last time the peer snapshot was written
    bool     _save_dir_checked; // true once we've attempted to create AP_SWARMMESH_PEER_DIR
    bool     _save_dir_ok;      // true if the directory exists/was created successfully
#endif
    uint32_t _last_check_ms;    // last time the peer table was checked for pruning
};

namespace AP {
    AP_SwarmMesh *swarmmesh();
};

#endif  // AP_SWARMMESH_ENABLED
