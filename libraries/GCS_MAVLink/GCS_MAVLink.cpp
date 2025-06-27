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
#include "mavlink_fragment.h"
#include "RadioBuffer.h"

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

    // 1) Only touch the port we care about
    if (chan == MAVLINK_COMM_2) {
        //mavlink_comm_port[chan]->write("\n>> nRF port (COMM_2) <<<\n"); // DEBUG
        
        // pick a chunk size so that after adding ~12B header+2B checksum
        // we stay ≤ 36 bytes total
        static const int MAV_CHUNK = 24;
        uint8_t offset = 0;

        // This full_id is for Syslink's own fragmentation of the current buf,
        // if buf itself needs to be split into multiple Syslink frames.
        uint16_t syslink_fragmentation_full_id = g_syslink_message_id_counter++;

        // Calculate total Syslink fragments needed for *this current buf*
        uint8_t total_syslink_fragments_for_chunk = (len + MAV_CHUNK - 1) / MAV_CHUNK;
        if (total_syslink_fragments_for_chunk == 0 && len > 0) { // Ensure at least 1 fragment if data exists
            total_syslink_fragments_for_chunk = 1;
        }
        else if (len == 0) { // No data to send

            if (len == 0) return; // Optionally, handle zero-length chunks if they are not expected
        }

        while (offset < len)
        {
            uint8_t this_len = std::min((uint16_t)MAV_CHUNK, (uint16_t)(len - offset));
            uint8_t length_field = 6 + this_len; // 6 for Syslink frag header + data part length
            // allocate packet buffer on the stack
            uint8_t packet[36];
            uint8_t idx = 0;

            //gcs().send_text(MAV_SEVERITY_ALERT, "DBG frag: len=%u off=%u this_len=%u L=%u", (unsigned)len, (unsigned)offset, (unsigned)this_len, (unsigned)length_field); // DEBUG

            // 1) Syslink header
            packet[idx++] = 0xBC;
            packet[idx++] = 0xCF;
            packet[idx++] = 0x0B;            // TYPE = Radio MAVLink (Raw = 0x00)
            packet[idx++] = length_field; // fragment header (6 B) + data

            // 2) Fragment header
            packet[idx++] = uint8_t(syslink_fragmentation_full_id & 0xFF);   // LSB of ID for this chunk's Syslink fragmentation session
            packet[idx++] = uint8_t(syslink_fragmentation_full_id >> 8);     // MSB
            packet[idx++] = uint8_t(len & 0xFF);                             // LSB of total length of current chunk_buf
            packet[idx++] = uint8_t(len >> 8);                               // MSB
            packet[idx++] = total_syslink_fragments_for_chunk;               // Total Syslink fragments for this chunk_buf
            packet[idx++] = uint8_t(offset / MAV_CHUNK);                     // Sequence number of this Syslink fragment (0-indexed)

            // 3) Payload slice
            if (this_len > 0) { // Only copy if there's data for this fragment
                memcpy(packet + idx, buf + offset, this_len);
            }
            idx += this_len;

            // 4) Fletcher-8 over TYPE..data
            uint8_t c0=0, c1=0;
            for (uint8_t j = 2; j < idx; j++) { // Start from TYPE field (index 2)
                c0 += packet[j];
                c1 += c0;
            }
            packet[idx++] = c0;
            packet[idx++] = c1;

            // 5) write it immediately, while 'packet' is still in scope
            //mavlink_comm_port[chan]->write(packet, idx);

            const bool nrf_is_ready = (hal.gpio->read(54) == 0);
            
            // We can send immediately if the nRF is ready AND the buffer is empty (to maintain correct order)
            if (g_syslink_ready && nrf_is_ready && RadioPacketBuffer::get_instance().is_empty()) {
                gcs().send_text(MAV_SEVERITY_DEBUG, "COMM_SEND: Using comm_send path for chan %d", (int)chan); // DEBUG
                mavlink_comm_port[chan]->write(packet, idx);
                g_syslink_ready = false;    // Add to reset the flag?
            } else {
                // Otherwise, the nRF is busy or there are older packets waiting. Buffer this packet.
                if (!RadioPacketBuffer::get_instance().push(packet, idx)) {
                    // Buffer is full. This packet is dropped.
                    gcs().send_text(MAV_SEVERITY_DEBUG, "Radio buffer full, packet dropped!\n"); // DEBUG
                }
            }

            offset += this_len;
            if (len == 0) break; // If original chunk was 0 length, send one empty syslink packet and exit

        }

        return;  // skip the normal send
    }
    // otherwise the regular MAVLink send…
    mavlink_comm_port[chan]->write(buf, len);
}

void comm_send_buffer_old(mavlink_channel_t chan, const uint8_t *buf, uint8_t len)
{
    if (!valid_channel(chan) || mavlink_comm_port[chan] == nullptr || chan_discard[chan]) {
        return;
    }
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
