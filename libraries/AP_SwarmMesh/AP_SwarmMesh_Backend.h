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

#include "AP_SwarmMesh.h"

#if AP_SWARMMESH_ENABLED

#include "AP_SwarmMesh_packet.h"
#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include <GCS_MAVLink/GCS_MAVLink.h>
#include <AP_AHRS/AP_AHRS_config.h>

class AP_SwarmMesh_Backend
{
public:
    AP_SwarmMesh_Backend(AP_SwarmMesh &frontend);

    // return true if the transport is connected and recently receiving
    virtual bool healthy();

    // run one tick: drain RX bytes, update freshness, send TX streams
    virtual void update();

    // write backend performance counters to the onboard log
    virtual void log_stats();

    // Send one MAVLink2 frame in the critical class (see SWARMMESH_TYPE_CRITICAL): at once
    // if nothing critical is waiting and the transport takes it, else queued (the oldest
    // frame is dropped when the queue is full). ``deadline_ms`` (0 = none) bounds how long it
    // may wait here and is carried in the header for the relays. False if it was dropped.
    bool send_critical(const uint8_t *frame, uint16_t len, uint8_t dest_id, uint16_t deadline_ms);

    // Counters of the critical class (SwarmCriticalStats; the queue maximum restarts when
    // ``restart_max`` is set).
    using CriticalStats = SwarmCriticalStats;
    void get_critical_stats(CriticalStats &stats, bool restart_max);

protected:

    // returns true if the transport is ready to send/receive
    virtual bool transport_ready() const = 0;

    // number of bytes available to read from the transport
    virtual uint32_t transport_available() = 0;

    // read one byte (-1 if none)
    virtual int16_t transport_read() = 0;

    // number of bytes that can be written without blocking
    virtual uint32_t transport_txspace() = 0;

    // write len bytes (for SITL this must be a single complete packet); false if the
    // transport refused them (nothing was written)
    virtual bool transport_write(const uint8_t *buf, uint16_t len) = 0;

    // Datagram transports call this with every datagram as it arrives (any thread), so a
    // SYNC packet's receive time is its arrival, not the next update() tick. Others leave the
    // stamp to process time.
    void note_datagram(const uint8_t *data, uint16_t len, uint64_t arrival_us);

    // ---- frontend accessors ----
    uint8_t frontend_sysid() const;
    uint8_t frontend_dest_id() const;
    uint8_t frontend_ttl() const;
    AP_SwarmMesh::PeerState *frontend_peerstate(uint8_t peer_sysid);
    uint8_t frontend_peer_count() const;
    AP_SwarmMesh::PeerState *frontend_peer_at(uint8_t index);
    uint8_t frontend_sr_rate(uint8_t bucket) const;
    bool frontend_uses_full() const;
    uint16_t frontend_log_rate_hz() const;
    uint32_t frontend_log_mask() const;
    int8_t   frontend_fwd_port() const;
    uint8_t  frontend_fwd_mask() const;
    uint8_t  frontend_critical_depth() const;

    AP_SwarmMesh &_frontend;

private:

    // TX stream buckets (order must match SR_* param indices in AP_SwarmMesh::var_info)
    enum class Bucket : uint8_t {
        POSITION = 0,   // GLOBAL_POSITION_INT, LOCAL_POSITION_NED
        EXT_STAT = 1,   // SYS_STATUS, NAV_CONTROLLER_OUTPUT, POSITION_TARGET_GLOBAL_INT, MISSION_CURRENT
        EXTRA1   = 2,   // ATTITUDE, EKF_STATUS_REPORT, SCALED_IMU
#if AP_SWARMMESH_COORD_ENABLED
        COORD    = 3,   // TUNNEL carrying the coordination basket
#endif
        SYNC     = AP_SwarmMesh::SYNC_BUCKET,  // mesh clock (SWARMMESH_TYPE_SYNC), reference only
    };

    // RX message types gated by the _LOG_MASK param (bits 9-31 reserved)
    enum class LogMsg : uint32_t {
        HEARTBEAT                   = 1U << 0,
        SYS_STATUS                  = 1U << 1,
        GLOBAL_POSITION_INT         = 1U << 2,
        LOCAL_POSITION_NED          = 1U << 3,
        POSITION_TARGET_GLOBAL_INT  = 1U << 4,
        EXTENDED_SYS_STATE          = 1U << 5,
        ATTITUDE                    = 1U << 6,
        EKF_STATUS_REPORT           = 1U << 7,
        SCALED_IMU                  = 1U << 8,
#if AP_SWARMMESH_COORD_ENABLED
        COORDINATION                = 1U << 9,
#endif
    };

    // RX parser state machine
    enum class ParseState : uint8_t {
        WAIT_SYNC1,
        WAIT_SYNC2,
        HEADER,
        PAYLOAD,
    } _state = ParseState::WAIT_SYNC1;

    // parser working buffer
    uint8_t  _msgbuf[SWARMMESH_MSG_BUF_MAX];
    uint16_t _msg_len;
    uint8_t  _payload_len;
    uint8_t  _crc;
    uint8_t  _type;

    // persistent MAVLink byte-level parser state
    mavlink_message_t _mavlink_rxmsg;
    mavlink_status_t  _mavlink_rx_status;

    // TX sequence / counters
    uint16_t _tx_seq;
    uint16_t _tx_fwd;
    uint16_t _tx_dropped;
    uint16_t _crc_fail;
    uint16_t _stale;
    uint16_t _ttl;
    uint16_t _dedup;
    uint16_t _dropped;

    // timers
    uint32_t _last_rx_ms;
    uint32_t _last_bucket_ms[AP_SwarmMesh::NUM_BUCKETS];
    uint32_t _last_heartbeat_ms;
    uint32_t _last_log_ms;

    // parser
    bool parse_byte(uint8_t b);

    // packet processing
    void process_packet();
    void handle_mavlink(const mavlink_message_t &msg, AP_SwarmMesh::PeerState &ps);
    bool log_rate_ok();

    // type freshness: map a MAVLink msgid to a MsgFresh bit (-1 if untracked), and stamp that type's last_heard time + set its freshness bit.
    static int8_t fresh_bit_for_msgid(uint32_t msgid);
    void mark_fresh(AP_SwarmMesh::PeerState &ps, uint32_t msgid);

    // true only if we have GPS UTC. Used to gate the deadline/staleness check.
    bool have_synced_utc(uint64_t &usec) const;

    // critical class: a ring of complete mesh packets (header + frame) waiting to be sent
    struct CriticalSlot {
        uint32_t queued_ms;     // when it was queued (deadline reference)
        uint16_t deadline_ms;   // 0 = none
        uint16_t len;           // header + payload
        uint8_t  pkt[SWARMMESH_MSG_BUF_MAX];
    };
    CriticalSlot *_crit;        // allocated on first use, COOP depth slots
    uint8_t  _crit_depth;
    uint8_t  _crit_head;        // oldest
    uint8_t  _crit_count;
    CriticalStats _crit_stats;
    // queue a complete packet (dropping the oldest when full)
    void critical_push(const uint8_t *pkt, uint16_t len, uint16_t deadline_ms);
    // send up to ``max_frames`` queued packets, oldest first; expired ones are dropped
    void critical_drain(uint8_t max_frames);
    // handle a received critical packet (after the duplicate and TTL checks)
    void process_critical(const p2p_header_t &hdr);

    // mesh clock: broadcast our mesh time (reference), and take a received SYNC (follower)
    void send_sync();
    void process_sync(const p2p_header_t &hdr);
    // arrival times of the last few SYNC datagrams (note_datagram), by origin and sequence
    struct SyncStamp {
        uint64_t arrival_us;
        uint16_t seq;
        uint8_t  origin;
    };
    static constexpr uint8_t SYNC_STAMPS = 4;
    SyncStamp _sync_stamps[SYNC_STAMPS];
    uint8_t   _sync_stamp_next;
    HAL_Semaphore _sync_stamp_sem;
    uint32_t  _sync_tx;

    // fill the header's checksum (sum of every header byte before it)
    static void set_header_crc(p2p_header_t &hdr);

#if HAL_GCS_ENABLED
    // forward a received peer MAVLink frame, unmodified, to the companion serial port;
    // false if the port is not configured or had no room
    bool forward_to_port(const uint8_t *frame, uint16_t len);

    // cached resolution of the _FWD_PORT parameter to a MAVLink channel
    int8_t   _fwd_port_resolved = -2;  // last port value resolved (-2 == never resolved)
    uint8_t  _fwd_chan = UINT8_MAX;    // resolved channel, UINT8_MAX if port is not MAVLink
    uint16_t _fwd_dropped;             // frames dropped because the port had no space
#endif

    // TX path
    void send_mavlink(uint8_t dest_id, const mavlink_message_t *msg, uint16_t deadline_ms, uint8_t ttl);
    void forward_mavlink(uint8_t id, uint8_t dest_id, const uint8_t *payload, uint16_t deadline_ms, uint8_t ttl, uint8_t payload_len, uint8_t flags, uint64_t origin_time, uint16_t seq);
    void send_stream(Bucket bucket);
    // MAVLink messages
    void send_heartbeat();
#if AP_AHRS_ENABLED
    void send_global_position_int();
    void send_local_position();
    void send_attitude();
    void send_ekf_status_report();
    void send_scaled_imu();
#endif
    void send_sys_status();
    void send_nav_controller_output();
    void send_position_target_global_int();
    void send_extended_sys_state();
#if AP_SWARMMESH_COORD_ENABLED
    void send_coordination();
    void handle_coordination(const mavlink_message_t &msg, AP_SwarmMesh::PeerState &ps);
#endif
};

#endif  // AP_SWARMMESH_ENABLED
