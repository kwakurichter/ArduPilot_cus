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

/// @file	GCS_MAVLink.cpp

/*
This provides some support code and variables for MAVLink enabled sketches

*/

#include "GCS_config.h"

#if HAL_MAVLINK_BINDINGS_ENABLED

#include "GCS.h"
#include "GCS_MAVLink.h"

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>

#ifdef HAL_CF21
#include "RadioBuffer.h"
#include <AP_Common/ExpandingString.h>

#pragma pack(push, 1)
typedef struct {
    uint8_t  stx;          // 0xA8 for mission-state packet
    uint8_t  peer_id;      // source id
    uint16_t seq;          // sequence number for dedupe
    uint8_t  st;           // mission state/event
    uint16_t val;          // optional value (wp idx, substate, etc.)
    uint32_t time_ms;      // timestamp
    uint16_t res_0;        // reserved
    uint16_t res_1;        // reserved
    uint8_t  c0;           // Fletcher-8
    uint8_t  c1;           // Fletcher-8
} p2p_mstate_v1_t;
#pragma pack(pop)

static_assert(sizeof(p2p_mstate_v1_t) <= 17, "p2p packet must be 17 bytes");

#pragma pack(push, 1)
typedef struct {
    uint8_t  stx;           // 0xA7
    uint8_t  peer_id;       // who sent it
    uint32_t time_boot_ms;  // copied from MAVLink ATTITUDE
    int16_t  roll_cd;       // roll  in centi-deg
    int16_t  pitch_cd;      // pitch in centi-deg
    int16_t  yaw_cd;        // yaw   in centi-deg (wrap to [-18000, +18000])
    int16_t  rollrate_cds;  // rollspeed  in centi-deg/s
    int16_t  pitchrate_cds; // pitchspeed in centi-deg/s
    int16_t  yawrate_cds;   // yawspeed   in centi-deg/s
    int16_t  res_0;         // reserved
    int16_t  res_1;         // reserved
    uint8_t  c0;            // Fletcher-8
    uint8_t  c1;            // Fletcher-8
} p2p_att_v1_t;
#pragma pack(pop)

static_assert(sizeof(p2p_att_v1_t) == 24, "p2p packet must be 24 bytes");

#pragma pack(push, 1)
typedef struct {
    uint8_t  stx;           // 0xA9
    uint8_t  peer_id;       // who sent it
    uint32_t time_boot_ms;  // copied from MAVLink (32)
    int16_t  x_pos;         // x-position in cm
    int16_t  y_pos;         // y-position in cm
    int16_t  z_pos;         // z-position in cm
    int16_t  x_vel;         // x-velocity in cm/s
    int16_t  y_vel;         // y-velocity in cm/s
    int16_t  z_vel;         // z-velocity in cm/s
    int16_t  res_0;         // reserved
    int16_t  res_1;         // reserved
    uint8_t  c0;            // Fletcher-8
    uint8_t  c1;            // Fletcher-8
} p2p_pos_v1_t;
#pragma pack(pop)

static_assert(sizeof(p2p_pos_v1_t) == 24, "p2p packet must be 24 bytes");

#pragma pack(push, 1)
typedef struct {
    uint8_t  stx;           // 0xAA
    uint8_t  peer_id;       // who sent it
    uint32_t time_boot_ms;  // AP_HAL::millis() at send time
    uint16_t res_0;         // reserved
    uint8_t  c0;            // Fletcher-8
    uint8_t  c1;            // Fletcher-8
} p2p_rssi_v1_t;
#pragma pack(pop)

static_assert(sizeof(p2p_rssi_v1_t) == 10, "p2p_rssi_v1_t must be 10 bytes");

static uint16_t g_syslink_message_id_counter = 0; // For Crazyflie Syslink Packet ID

static HAL_Semaphore g_mstate_sem;
static bool g_mstate_pending = false;
static p2p_mstate_v1_t g_mstate_pkt{};
#endif

extern const AP_HAL::HAL& hal;

#ifdef MAVLINK_SEPARATE_HELPERS
// Shut up warnings about missing declarations; TODO: should be fixed on
// mavlink/pymavlink project for when MAVLINK_SEPARATE_HELPERS is defined
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmissing-declarations"
#include "include/mavlink/v2.0/mavlink_helpers.h"
#pragma GCC diagnostic pop
#endif

mavlink_message_t* mavlink_get_channel_buffer(uint8_t chan) {
#if HAL_GCS_ENABLED
    GCS_MAVLINK *link = gcs().chan(chan);
    if (link == nullptr) {
        return nullptr;
    }
    return link->channel_buffer();
#else
    return nullptr;
#endif
}

mavlink_status_t* mavlink_get_channel_status(uint8_t chan) {
#if HAL_GCS_ENABLED
    GCS_MAVLINK *link = gcs().chan(chan);
    if (link == nullptr) {
        return nullptr;
    }
    return link->channel_status();
#else
    return nullptr;
#endif
}

#endif // HAL_MAVLINK_BINDINGS_ENABLED

#if HAL_GCS_ENABLED

AP_HAL::UARTDriver	*mavlink_comm_port[MAVLINK_COMM_NUM_BUFFERS];
bool gcs_alternative_active[MAVLINK_COMM_NUM_BUFFERS];

// per-channel lock
static HAL_Semaphore chan_locks[MAVLINK_COMM_NUM_BUFFERS];
static bool chan_discard[MAVLINK_COMM_NUM_BUFFERS];

mavlink_system_t mavlink_system = {7,1};

// routing table
MAVLink_routing GCS_MAVLINK::routing;

GCS_MAVLINK *GCS_MAVLINK::find_by_mavtype_and_compid(uint8_t mav_type, uint8_t compid, uint8_t &sysid) {
    mavlink_channel_t channel;
    if (!routing.find_by_mavtype_and_compid(mav_type, compid, sysid, channel)) {
        return nullptr;
    }
    return gcs().chan(channel);
}

// set a channel as private. Private channels get sent heartbeats, but
// don't get broadcast packets or forwarded packets
void GCS_MAVLINK::set_channel_private(mavlink_channel_t _chan)
{
    const uint8_t mask = (1U<<(unsigned)_chan);
    mavlink_private |= mask;
}

// return a MAVLink parameter type given a AP_Param type
MAV_PARAM_TYPE GCS_MAVLINK::mav_param_type(enum ap_var_type t)
{
    if (t == AP_PARAM_INT8) {
	    return MAV_PARAM_TYPE_INT8;
    }
    if (t == AP_PARAM_INT16) {
	    return MAV_PARAM_TYPE_INT16;
    }
    if (t == AP_PARAM_INT32) {
	    return MAV_PARAM_TYPE_INT32;
    }
    // treat any others as float
    return MAV_PARAM_TYPE_REAL32;
}


/// Check for available transmit space on the nominated MAVLink channel
///
/// @param chan		Channel to check
/// @returns		Number of bytes available
uint16_t comm_get_txspace(mavlink_channel_t chan)
{
    GCS_MAVLINK *link = gcs().chan(chan);
    if (link == nullptr) {
        return 0;
    }
    return link->txspace();
}

#ifdef HAL_CF21
static inline float wrap_pi(float a) {
    while (a >  M_PI) a -= 2.0f*M_PI;
    while (a < -M_PI) a += 2.0f*M_PI;
    return a;
}

static inline int16_t to_cdeg(float radians)
{
    // radians -> centi-degrees
    const float cd = radians * (180.0f / (float)M_PI) * 100.0f;
    const int32_t v = (int32_t)lrintf(cd);
    return (int16_t)constrain_int32(v, -32768, 32767);
}

static inline int16_t to_cdeg_per_s(float rad_s)
{
    const float cds = rad_s * (180.0f / (float)M_PI) * 100.0f;
    const int32_t v = (int32_t)lrintf(cds);
    return (int16_t)constrain_int32(v, -32768, 32767);
}

static inline int16_t to_cm(float metres)
{
    const int32_t v = (int32_t)lrintf(metres * 100.0f);
    return (int16_t)constrain_int32(v, -32768, 32767);
}

static inline int16_t to_cms(float m_per_s)
{
    const int32_t v = (int32_t)lrintf(m_per_s * 100.0f);
    return (int16_t)constrain_int32(v, -32768, 32767);
}

static inline void fletcher8(const uint8_t *buf, uint8_t len, uint8_t *c0, uint8_t *c1)
{
    uint8_t a = 0, b = 0;
    for (uint8_t i = 0; i < len; i++) {
        a += buf[i];
        b += a;
    }
    *c0 = a;
    *c1 = b;
}

static inline bool fletcher8_ok(const uint8_t *buf, uint8_t len_with_crc)
{
    if (len_with_crc < 3) return false;
    uint8_t c0, c1;
    fletcher8(buf, len_with_crc - 2, &c0, &c1);
    return (c0 == buf[len_with_crc - 2]) && (c1 == buf[len_with_crc - 1]);
}

static void build_p2p_att_packet(p2p_att_v1_t &pkt, uint8_t peer_id, const mavlink_attitude_t &att, uint16_t res_0, uint16_t res_1)
{
    pkt.stx = 0xA7;
    pkt.peer_id = peer_id;
    pkt.time_boot_ms = att.time_boot_ms;

    pkt.roll_cd  = to_cdeg(att.roll);
    pkt.pitch_cd = to_cdeg(att.pitch);
    pkt.yaw_cd   = to_cdeg(wrap_pi(att.yaw));           // keep yaw in [-180, 180]

    pkt.rollrate_cds  = to_cdeg_per_s(att.rollspeed);
    pkt.pitchrate_cds = to_cdeg_per_s(att.pitchspeed);
    pkt.yawrate_cds   = to_cdeg_per_s(att.yawspeed);

    pkt.res_0 = res_0;
    pkt.res_1 = res_1;

    fletcher8((const uint8_t*)&pkt, sizeof(pkt) - 2, &pkt.c0, &pkt.c1);
}

static void build_p2p_pos_packet(p2p_pos_v1_t &pkt, uint8_t peer_id, const mavlink_local_position_ned_t &pos, uint16_t res_0, uint16_t res_1)
{
    pkt.stx = 0xA9;
    pkt.peer_id = peer_id;
    pkt.time_boot_ms = pos.time_boot_ms;

    pkt.x_pos = to_cm(pos.x);
    pkt.y_pos = to_cm(pos.y);
    pkt.z_pos = to_cm(pos.z);

    pkt.x_vel = to_cms(pos.vx);
    pkt.y_vel = to_cms(pos.vy);
    pkt.z_vel = to_cms(pos.vz);

    pkt.res_0 = res_0;
    pkt.res_1 = res_1;    

    fletcher8((const uint8_t*)&pkt, sizeof(pkt) - 2, &pkt.c0, &pkt.c1);
}

void p2p_queue_mission_state(uint8_t src_id, uint16_t seq, uint8_t st, uint16_t val, uint32_t time_ms, uint16_t res_0, uint16_t res_1)
{
    p2p_mstate_v1_t pkt{};
    pkt.stx     = 0xA8;
    pkt.peer_id = src_id;
    pkt.seq     = seq;
    pkt.st      = st;
    pkt.val     = val;
    pkt.time_ms = time_ms;
    pkt.res_0   = res_0;
    pkt.res_1   = res_1;    

    fletcher8((const uint8_t*)&pkt, sizeof(pkt) - 2, &pkt.c0, &pkt.c1);

    WITH_SEMAPHORE(g_mstate_sem);
    g_mstate_pkt = pkt;
    g_mstate_pending = true;   // flag for COMM_2 pipeline
}

static void p2p_send_broadcast_payload(const uint8_t *payload, uint8_t payload_len)
{
    if (payload_len > 58) {
        return; // must fit (60 - 2 CRTP header)
    }

    // 1) CRTP header + payload
    uint8_t p2p_packet[58];
    uint8_t p2p_idx = 0;
    p2p_packet[p2p_idx++] = 0xFF;
    p2p_packet[p2p_idx++] = 0x80 | (0 & 0x0F); // Port 0
    memcpy(&p2p_packet[p2p_idx], payload, payload_len);
    p2p_idx += payload_len;

    // 2) Syslink header + CRTP payload
    uint8_t syslink_packet[64];
    uint8_t syslink_idx = 0;
    syslink_packet[syslink_idx++] = 0xBC;
    syslink_packet[syslink_idx++] = 0xCF;
    syslink_packet[syslink_idx++] = 0x0A;      // P2P Broadcast
    syslink_packet[syslink_idx++] = p2p_idx;   // length
    memcpy(&syslink_packet[syslink_idx], p2p_packet, p2p_idx);
    syslink_idx += p2p_idx;

    // 3) Fletcher-8 checksum
    uint8_t c0 = 0, c1 = 0;
    for (uint8_t j = 2; j < syslink_idx; j++) {
        c0 += syslink_packet[j];
        c1 += c0;
    }
    syslink_packet[syslink_idx++] = c0;
    syslink_packet[syslink_idx++] = c1;

    // 4) push
    RadioPacketBuffer::get_instance().push(syslink_packet, syslink_idx);
}

static void p2p_flush_pending_mission_state()
{
    p2p_mstate_v1_t pkt;
    {
        WITH_SEMAPHORE(g_mstate_sem);
        if (!g_mstate_pending) {
            return;
        }
        pkt = g_mstate_pkt;
        g_mstate_pending = false;
    }

    // only send if the radio buffer has space for 1 packet
    if (RadioPacketBuffer::get_instance().free_space() < 1) {
        // retry later? reset pending here instead of dropping
        return;
    }

    p2p_send_broadcast_payload((const uint8_t*)&pkt, sizeof(pkt));
}

static uint8_t get_peer_id_from_param()
{
    // cache lookup
    static AP_Param *p = nullptr;
    static enum ap_var_type t = AP_PARAM_NONE;

    if (p == nullptr) {
        p = AP_Param::find("CF_PEER_ID", &t, nullptr);
    }
    if (p == nullptr) {
        return 21; // fallback
    }

    int32_t v = 0;
    switch (t) {
    case AP_PARAM_INT8:
        v = ((AP_Int8*)p)->get();
        break;
    case AP_PARAM_INT16:
        v = ((AP_Int16*)p)->get();
        break;
    case AP_PARAM_INT32:
        v = ((AP_Int32*)p)->get();
        break;
    default:
        return 21; // wrong type -> fallback
    }

    // clamp to byte
    if (v < 0)   v = 0;
    if (v > 255) v = 255;

    return (uint8_t)v;
}

static uint32_t get_p2p_stream_from_param()
{
    // cache lookup
    static AP_Param *p = nullptr;
    static enum ap_var_type t = AP_PARAM_NONE;

    if (p == nullptr) {
        p = AP_Param::find("CF_P2P_STREAM", &t, nullptr);
    }
    if (p == nullptr) {
        return 0; // fallback
    }

    int32_t v = 0;
    switch (t) {
    case AP_PARAM_INT8:
        v = ((AP_Int8*)p)->get();
        break;
    case AP_PARAM_INT16:
        v = ((AP_Int16*)p)->get();
        break;
    case AP_PARAM_INT32:
        v = ((AP_Int32*)p)->get();
        break;
    default:
        return 0; // wrong type -> fallback
    }

    // clamp to byte
    if (v < 0)   v = 0;
    if (v > 2147483647) v = 2147483647;

    return (uint32_t)v;
}

static uint8_t get_rssi_hz_from_param()
{
    // cache lookup
    static AP_Param *p = nullptr;
    static enum ap_var_type t = AP_PARAM_NONE;

    if (p == nullptr) {
        p = AP_Param::find("CF_RSSI_HZ", &t, nullptr);
    }
    if (p == nullptr) {
        return 0; // fallback
    }

    int32_t v = 0;
    switch (t) {
    case AP_PARAM_INT8:
        v = ((AP_Int8*)p)->get();
        break;
    case AP_PARAM_INT16:
        v = ((AP_Int16*)p)->get();
        break;
    case AP_PARAM_INT32:
        v = ((AP_Int32*)p)->get();
        break;
    default:
        return 0; // wrong type -> fallback
    }

    // clamp to 50 Hz
    if (v < 0)   v = 0;
    if (v > 50) v = 50;

    return (uint8_t)v;
}

enum : uint32_t {
    P2P_TX_ATTITUDE      = 1U << 0,
    P2P_TX_MISSION_STATE = 1U << 1,
    P2P_TX_POSITION      = 1U << 2,
    P2P_TX_RSSI          = 1U << 3,
};

static void p2p_send_rssi_beacon()
{
    if (RadioPacketBuffer::get_instance().free_space() < 1) {
        return;
    }
    p2p_rssi_v1_t pkt{};
    pkt.stx          = 0xAA;
    pkt.peer_id      = get_peer_id_from_param();
    pkt.time_boot_ms = AP_HAL::millis();
    pkt.res_0        = 0;
    fletcher8((const uint8_t*)&pkt, sizeof(pkt) - 2, &pkt.c0, &pkt.c1);
    p2p_send_broadcast_payload((const uint8_t*)&pkt, sizeof(pkt));
}
#endif

/*
  send a buffer out a MAVLink channel
 */
void comm_send_buffer(mavlink_channel_t chan, const uint8_t *buf, uint8_t len)
{
    if (!valid_channel(chan) || mavlink_comm_port[chan] == nullptr || chan_discard[chan]) {
        return;
    }
#ifdef HAL_CF21
    // This logic is for the nRF radio channel (MAVLINK_COMM_2)
    if (chan == MAVLINK_COMM_2) {
        // get P2P stream bitmask
        const uint32_t P2P_MASK = get_p2p_stream_from_param();
        bool p2p_att = false;
        bool p2p_pos = false;

        if (P2P_MASK & P2P_TX_ATTITUDE) {
            p2p_att = true;
        }
        if (P2P_MASK & P2P_TX_POSITION) {
            p2p_pos = true;
        }        
        if (P2P_MASK & P2P_TX_MISSION_STATE) {
            p2p_flush_pending_mission_state();
        }
        if (P2P_MASK & P2P_TX_RSSI) {
            const uint8_t hz = get_rssi_hz_from_param();
            if (hz > 0) {
                static uint32_t last_rssi_ms = 0;
                const uint32_t now_ms = AP_HAL::millis();
                if (now_ms - last_rssi_ms >= (1000u / hz)) {
                    last_rssi_ms = now_ms;
                    p2p_send_rssi_beacon();
                }
            }
        }

        // --- START P2P REASSEMBLY & INTERCEPTION LOGIC ---

        // Static buffer to reassemble MAVLink chunks
        static uint8_t p2p_mavlink_buf[MAVLINK_MAX_PACKET_LEN];
        static uint16_t p2p_mavlink_idx = 0;
        static uint16_t expected_mavlink_len = 0;

        // Append incoming data to our reassembly buffer
        if ((p2p_mavlink_idx + len) <= MAVLINK_MAX_PACKET_LEN) {
            memcpy(&p2p_mavlink_buf[p2p_mavlink_idx], buf, len);
            p2p_mavlink_idx += len;
        } else {
            // Buffer overflow, something is wrong. Reset.
            p2p_mavlink_idx = 0;
            expected_mavlink_len = 0;
            return;
        }

        // Check if we have enough data for a MAVLink header
        if (p2p_mavlink_idx >= 2) {
            if (expected_mavlink_len == 0) {
                // Determine total expected length from the MAVLink header
                if (p2p_mavlink_buf[0] == MAVLINK_STX) { // MAVLink 2
                    expected_mavlink_len = p2p_mavlink_buf[1] + 12; // Payload len + 12 bytes overhead
                } else if (p2p_mavlink_buf[0] == MAVLINK_STX_MAVLINK1) { // MAVLink 1
                    expected_mavlink_len = p2p_mavlink_buf[1] + 8; // Payload len + 8 bytes overhead
                }
            }

            // Do we have the complete packet yet?
            if (expected_mavlink_len > 0 && p2p_mavlink_idx >= expected_mavlink_len) {
                // We have a full MAVLink packet in p2p_mavlink_buf
                bool is_p2p_message = false;

                // 1) Parse the full frame we already buffered
                mavlink_message_t in_msg {};
                mavlink_status_t  st {};
                bool parsed = false;                

                // Feed the already-complete packet into the MAVLink parser
                for (uint16_t i = 0; i < expected_mavlink_len; i++) {
                    if (mavlink_parse_char(MAVLINK_COMM_0, p2p_mavlink_buf[i], &in_msg, &st)) {
                        parsed = true;
                        break;
                    }
                }

                if (parsed) {
                    const uint32_t msgid = in_msg.msgid; // works for MAVLink1 and MAVLink2
                    const uint8_t PEER_ID = get_peer_id_from_param();
                    if (msgid == MAVLINK_MSG_ID_ATTITUDE) {
                        // Decode MAVLink ATTITUDE payload
                        mavlink_attitude_t att {};
                        mavlink_msg_attitude_decode(&in_msg, &att);

                        // Build 24-byte custom packet
                        p2p_att_v1_t pkt {};
                        build_p2p_att_packet(pkt, PEER_ID, att, 0, 0);

                        // Overwrite the reassembly buffer with our custom packet bytes
                        static_assert(sizeof(p2p_att_v1_t) == 24, "p2p_att_v1_t must be 24 bytes");
                        memcpy(p2p_mavlink_buf, &pkt, sizeof(pkt));

                        expected_mavlink_len = sizeof(pkt);
                        p2p_mavlink_idx      = expected_mavlink_len;

                        if (p2p_att) {
                            is_p2p_message = true;
                        } else {
                            is_p2p_message = false;
                        }
                    }
                    if (msgid == MAVLINK_MSG_ID_LOCAL_POSITION_NED) {
                        // Decode MAVLink POSITION payload
                        mavlink_local_position_ned_t pos {};
                        mavlink_msg_local_position_ned_decode(&in_msg, &pos);

                        // Build 24-byte custom packet
                        p2p_pos_v1_t pkt {};
                        build_p2p_pos_packet(pkt, PEER_ID, pos, 0, 0);

                        // Overwrite the reassembly buffer with our custom packet bytes
                        static_assert(sizeof(p2p_pos_v1_t) == 24, "p2p_pos_v1_t must be 24 bytes");
                        memcpy(p2p_mavlink_buf, &pkt, sizeof(pkt));

                        expected_mavlink_len = sizeof(pkt);
                        p2p_mavlink_idx      = expected_mavlink_len;

                        if (p2p_pos) {
                            is_p2p_message = true;
                        } else {
                            is_p2p_message = false;
                        }
                    }
                    // Add more message types                    
                }                         

                if (is_p2p_message) {
                    // This is a heartbeat, let's wrap it for P2P
                    uint8_t p2p_packet[58]; // Buffer for the P2P packet
                    uint8_t p2p_idx = 0;

                    // 1. CRTP Header for P2P
                    p2p_packet[p2p_idx++] = 0xff;
                    p2p_packet[p2p_idx++] = 0x80 | (0 & 0x0f); // Port 0

                    // 2. Copy the MAVLink heartbeat payload
                    memcpy(&p2p_packet[p2p_idx], p2p_mavlink_buf, expected_mavlink_len);
                    p2p_idx += expected_mavlink_len;

                    // 3. Syslink Header
                    uint8_t syslink_packet[64];
                    uint8_t syslink_idx = 0;
                    syslink_packet[syslink_idx++] = 0xBC;
                    syslink_packet[syslink_idx++] = 0xCF;
                    syslink_packet[syslink_idx++] = 0x0A; // TYPE = P2P Broadcast
                    syslink_packet[syslink_idx++] = p2p_idx; // LENGTH

                    // 4. Copy CRTP-wrapped MAVLink packet
                    memcpy(&syslink_packet[syslink_idx], p2p_packet, p2p_idx);
                    syslink_idx += p2p_idx;

                    // 5. Fletcher-8 Checksum
                    uint8_t c0=0, c1=0;
                    for (uint8_t j = 2; j < syslink_idx; j++) {
                        c0 += syslink_packet[j];
                        c1 += c0;
                    }
                    syslink_packet[syslink_idx++] = c0;
                    syslink_packet[syslink_idx++] = c1;

                    // 6. Push to Radio Buffer
                    RadioPacketBuffer::get_instance().push(syslink_packet, syslink_idx);

                    // -- DEBUG --
                    //ExpandingString hex_dump;
                    //hex_dump.printf("P2P Sent(%u): ", syslink_idx);
                    //for (uint8_t i = 0; i < syslink_idx; i++) {
                    //    hex_dump.printf("%02X ", syslink_packet[i]);
                    //}
                    //gcs().send_text(MAV_SEVERITY_DEBUG, "%s", hex_dump.get_string());
                    // -- DEBUG --

                } else {    // regular telemetry nRF51 path
                    // Define the chunk size for fragmentation
                    static const int MAV_CHUNK = 24;

                    // Calculate how many fragments this MAVLink message will be split into
                    uint8_t total_syslink_fragments = (expected_mavlink_len + MAV_CHUNK - 1) / MAV_CHUNK;

                    // --- ATOMIC BUFFERING LOGIC ---
                    // Check if the radio buffer has enough space for ALL fragments of this message
                    if (RadioPacketBuffer::get_instance().free_space() < total_syslink_fragments) {
                        // Not enough space for the entire message, drop it.
                        // gcs().send_text(MAV_SEVERITY_WARNING, "Radio buffer full, MAVLink msg dropped!");
                        return;
                    }

                    // If we get here, there is enough space. Proceed with fragmentation and buffering.
                    uint16_t syslink_fragmentation_full_id = g_syslink_message_id_counter++;
                    uint8_t offset = 0;

                    while (offset < expected_mavlink_len)
                    {
                        uint8_t this_len = std::min((uint16_t)MAV_CHUNK, (uint16_t)(expected_mavlink_len - offset));
                        uint8_t length_field = 6 + this_len; // 6B fragment header + data
                        uint8_t packet[36]; // Buffer for one fragment
                        uint8_t idx = 0;

                        // 1) Syslink header
                        packet[idx++] = 0xBC;
                        packet[idx++] = 0xCF;
                        packet[idx++] = 0x0C; // TYPE = Radio MAVLink
                        packet[idx++] = length_field;

                        // 2) Fragment header
                        packet[idx++] = uint8_t(syslink_fragmentation_full_id & 0xFF);
                        packet[idx++] = uint8_t(syslink_fragmentation_full_id >> 8);
                        packet[idx++] = uint8_t(expected_mavlink_len & 0xFF);
                        packet[idx++] = uint8_t(expected_mavlink_len >> 8);
                        packet[idx++] = total_syslink_fragments;
                        packet[idx++] = uint8_t(offset / MAV_CHUNK);

                        // 3) Payload slice
                        if (this_len > 0) {
                            memcpy(packet + idx, p2p_mavlink_buf + offset, this_len);
                        }
                        idx += this_len;
                        
                        // 4) Fletcher-8 checksum
                        uint8_t c0=0, c1=0;
                        for (uint8_t j = 2; j < idx; j++) {
                            c0 += packet[j];
                            c1 += c0;
                        }
                        packet[idx++] = c0;
                        packet[idx++] = c1;

                        // 5) Push the fragment to the buffer. We've already confirmed space exists.
                        RadioPacketBuffer::get_instance().push(packet, idx);

                        offset += this_len;
                    }
                }

                // Reset the buffer for the next message
                p2p_mavlink_idx = 0;
                expected_mavlink_len = 0;
            }
        }
        // --- END P2P REASSEMBLY & INTERCEPTION LOGIC ---
        return; // we handle all MAVLINK_COMM_2 traffic inside this block now.
    }
#endif        
#if HAL_HIGH_LATENCY2_ENABLED
    // if it's a disabled high latency channel, don't send
    GCS_MAVLINK *link = gcs().chan(chan);
    if (link->is_high_latency_link && !gcs().get_high_latency_status()) {
        return;
    }
#endif
    if (gcs_alternative_active[chan]) {
        // an alternative protocol is active
        return;
    }
    const size_t written = mavlink_comm_port[chan]->write(buf, len);
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    if (written < len && !mavlink_comm_port[chan]->is_write_locked()) {
        AP_HAL::panic("Short write on UART: %lu < %u", (unsigned long)written, len);
    }
#else
    (void)written;
#endif
}

/*
  lock a channel for send
  if there is insufficient space to send size bytes then all bytes
  written to the channel by the mavlink library will be discarded
  while the lock is held.
 */
void comm_send_lock(mavlink_channel_t chan_m, uint16_t size)
{
    const uint8_t chan = uint8_t(chan_m);
    chan_locks[chan].take_blocking();
    if (mavlink_comm_port[chan]->txspace() < size) {
        chan_discard[chan] = true;
        gcs_out_of_space_to_send(chan_m);
    }
}

/*
  unlock a channel
 */
void comm_send_unlock(mavlink_channel_t chan_m)
{
    const uint8_t chan = uint8_t(chan_m);
    chan_discard[chan] = false;
    chan_locks[chan].give();
}

/*
  return reference to GCS channel lock, allowing for
  HAVE_PAYLOAD_SPACE() to be run with a locked channel
 */
HAL_Semaphore &comm_chan_lock(mavlink_channel_t chan)
{
    return chan_locks[uint8_t(chan)];
}

#endif  // HAL_GCS_ENABLED
