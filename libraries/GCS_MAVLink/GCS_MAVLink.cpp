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
#include "RadioBuffer.h"
#include <AP_Common/ExpandingString.h>

static uint16_t g_syslink_message_id_counter = 0; // For Crazyflie Syslink Packet ID

extern const AP_HAL::HAL& hal;

bool g_syslink_ready = false; // The flag to indicate NRF is ready

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

/*
  send a buffer out a MAVLink channel
 */
void comm_send_buffer(mavlink_channel_t chan, const uint8_t *buf, uint16_t len)
{
    if (!valid_channel(chan) || mavlink_comm_port[chan] == nullptr || chan_discard[chan]) {
        return;
    }

    // This logic is for the nRF radio channel (MAVLINK_COMM_2)
    if (chan == MAVLINK_COMM_2) {
        // --- START P2P REASSEMBLY & INTERCEPTION LOGIC ---

        // Static buffer to reassemble MAVLink chunks
        static uint8_t p2p_mavlink_buf[MAVLINK_MAX_PACKET_LEN];
        static uint8_t p2p_mavlink_idx = 0;
        static uint8_t expected_mavlink_len = 0;

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
                bool is_heartbeat = false;
                if (p2p_mavlink_buf[0] == MAVLINK_STX && p2p_mavlink_buf[7] == 0 && p2p_mavlink_buf[8] == 0 && p2p_mavlink_buf[9] == 0) {
                    is_heartbeat = true;
                } else if (p2p_mavlink_buf[0] == MAVLINK_STX_MAVLINK1 && p2p_mavlink_buf[5] == MAVLINK_MSG_ID_HEARTBEAT) {
                    is_heartbeat = true;
                }

                if (is_heartbeat) {
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
                    //gcs().send_text(MAV_SEVERITY_ALERT, "%s", hex_dump.get_string());
                    // -- DEBUG --

                    // Heartbeat sent via P2P, so we skip the normal GCS path
                    // return;
                } else {
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
                        packet[idx++] = 0x0B; // TYPE = Radio MAVLink
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
                        // We ignore the return value as we've pre-checked the space.
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
        return; // Important: We handle all MAVLINK_COMM_2 traffic inside this block now.
    }

    // For all other MAVLink channels, use the regular send
    mavlink_comm_port[chan]->write(buf, len);
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
