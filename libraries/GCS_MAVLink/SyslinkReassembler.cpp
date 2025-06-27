// SyslinkReassembler.cpp
#include "SyslinkReassembler.h"
#include <AP_HAL/AP_HAL.h> // For AP_HAL::millis()
#include "GCS.h" // For GCS_SEND_TEXT
#include <AP_Common/ExpandingString.h>

bool SyslinkToMAVLinkReassembler::check_fletcher8(const uint8_t* data, size_t len_for_check, uint8_t crc0_expected, uint8_t crc1_expected) {
    uint8_t c0 = 0, c1 = 0;
    for (size_t i = 0; i < len_for_check; ++i) {
        c0 += data[i];
        c1 += c0;
    }
    return (c0 == crc0_expected) && (c1 == crc1_expected);
}

void SyslinkToMAVLinkReassembler::reset_parser_state() {
    state = ParseState::WAIT_SYNC1;
    current_syslink_frame_buffer.clear();
    syslink_payload_bytes_expected = 0;
}

bool SyslinkToMAVLinkReassembler::process_byte(uint8_t c, std::function<void(uint8_t mav_byte)> mavlink_byte_pusher) {
    bool consumed_by_syslink = true; // Assume consumed initially

    current_syslink_frame_buffer.push_back(c);

    switch (state) {
        case ParseState::WAIT_SYNC1:
            if (c == SYSLINK_SYNC1) {
                // We've already added 'c', so buffer starts with SYNC1
                gcs().send_text(MAV_SEVERITY_DEBUG, "Syslink(1): SYNC detected\n"); // DEBUG
                state = ParseState::WAIT_SYNC2;
            } else {
                reset_parser_state(); // Not a start, reset
                consumed_by_syslink = false; // Let ArduPilot's main parser try it
            }
            break;

        case ParseState::WAIT_SYNC2:
            if (c == SYSLINK_SYNC2 && current_syslink_frame_buffer.size() == 2 && current_syslink_frame_buffer[0] == SYSLINK_SYNC1) {
                gcs().send_text(MAV_SEVERITY_DEBUG, "Syslink(1): SYNC detected\n"); // DEBUG
                state = ParseState::READ_TYPE_LENGTH;
            } else { // Sequencing error or not SYNC2
                reset_parser_state();
                // Check if 'c' itself is a new SYNC1
                if (c == SYSLINK_SYNC1) {
                    current_syslink_frame_buffer.push_back(c);
                    state = ParseState::WAIT_SYNC2;
                } else {
                    consumed_by_syslink = false;
                }
            }
            break;

        case ParseState::READ_TYPE_LENGTH:
            if (current_syslink_frame_buffer.size() == 3) { // Byte for TYPE received
                syslink_type_byte = c;
                if ((syslink_type_byte != EXPECTED_SYSLINK_TYPE_MAVLINK) && (syslink_type_byte != EXPECTED_SYSLINK_TYPE_RADIO)) {
                    gcs().send_text(MAV_SEVERITY_ALERT, "Syslink: Bad Type %u\n", c); // DEBUG
                    reset_parser_state(); // Invalid type
                    // 'c' was consumed as part of an invalid Syslink header
                }
            } else if (current_syslink_frame_buffer.size() == 4) { // Byte for LENGTH_FIELD received
                syslink_length_field = c;
                if (syslink_length_field < 1) { // We only do a basic sanity check here. A payload must have at least 1 byte for the CRTP header. The more specific length check for MAVLink packets is moved to after we've checked the port.
                    gcs().send_text(MAV_SEVERITY_ALERT, "Syslink: Length %u too small\n", c); // DEBUG
                    reset_parser_state(); // Invalid length
                } else {
                    syslink_payload_bytes_expected = syslink_length_field + 2; // data_slice + CRC
                    gcs().send_text(MAV_SEVERITY_DEBUG, "Syslink(1): HDR TYPE=%u LEN=%u\n", syslink_type_byte, syslink_length_field); // DEBUG
                    state = ParseState::WAIT_CRTP_HEADER;
                }
            }
            break;

        case ParseState::WAIT_CRTP_HEADER:
        // This is the 5th byte, which is the first byte of the Syslink payload (the CRTP header)
        {
            const uint8_t port = (c >> 4) & 0x0F;
            //if (port == 0) { // 0 is the CONSOLE port
            if (port == 0 || port == 15 || port == 11) { // temporarily accept 15 for debugging purposes
                // Payload must contain at least a 1-byte CRTP header and a 6-byte MAVLink fragment header.
                if (syslink_length_field < 7) {
                    gcs().send_text(MAV_SEVERITY_ALERT, "Syslink: MAVLink fragment length %u is too small\n", syslink_length_field); // DEBUG
                    reset_parser_state();
                }
                else {
                    // Length is valid for MAVLink, proceed to read the payload
                    gcs().send_text(MAV_SEVERITY_DEBUG, "Syslink: Parsing packet for CRTP Port: %u\n", port);   // DEBUG
                    state = ParseState::READ_PAYLOAD_AND_CRC;
                }
            } else {
                // This is for a different CRTP port, so we discard the packet
                gcs().send_text(MAV_SEVERITY_ALERT, "Syslink: Discarding packet for CRTP Port: %u\n", port);    // DEBUG
                reset_parser_state();
            }
        }
            break;

        case ParseState::READ_PAYLOAD_AND_CRC:
            // current_syslink_frame_buffer now holds SYNC1, SYNC2, TYPE, LENGTH_FIELD, and 'c' is the next byte.
            // We expect syslink_length_field bytes of data, then 2 CRC bytes.
            // Size of buffer when full: 4 (header) + syslink_length_field + 2 (CRC)
            if (current_syslink_frame_buffer.size() == (size_t)(4 + syslink_length_field + 2)) {
                // Full Syslink frame potentially received
                const uint8_t* frame_ptr = current_syslink_frame_buffer.data();
                // Data for CRC check starts at TYPE field (index 2)
                // Length of data for CRC = 1 (TYPE) + 1 (LENGTH_FIELD) + syslink_length_field
                size_t crc_check_len = 2 + syslink_length_field;
                uint8_t crc0_expected = frame_ptr[2 + crc_check_len];
                uint8_t crc1_expected = frame_ptr[2 + crc_check_len + 1];

                if (check_fletcher8(&frame_ptr[2], crc_check_len, crc0_expected, crc1_expected)) {
                    // CRC OK. Extract fragment and process
                    // The actual fragment data starts after SYNC1, SYNC2, TYPE, LENGTH_FIELD
                    //gcs().send_text(MAV_SEVERITY_DEBUG, "Syslink(1): Frame CRC OK\n"); // DEBUG

                    // Create an ExpandingString to build the hex dump of the full packet
                    ExpandingString full_packet_hex_dump;
                    full_packet_hex_dump.printf("Syslink Full Pkt OK: ");
                    for (const uint8_t byte_val : current_syslink_frame_buffer) {
                        full_packet_hex_dump.printf("%02X ", byte_val);
                    }
                    gcs().send_text(MAV_SEVERITY_DEBUG, "%s", full_packet_hex_dump.get_string()); // DEBUG

                    handle_complete_syslink_fragment(&frame_ptr[4], syslink_length_field, mavlink_byte_pusher);
                } else {
                    gcs().send_text(MAV_SEVERITY_ALERT, "Syslink(1): Frame CRC FAIL!\n"); // DEBUG
                }
                reset_parser_state(); // Done with this frame
            }
            // If not yet full, just keep consuming bytes in this state.
            break;

        default:
            reset_parser_state();
            consumed_by_syslink = false; // Should not happen
            break;
    }
    return consumed_by_syslink;
}


void SyslinkToMAVLinkReassembler::handle_complete_syslink_fragment(
    const uint8_t* frag_data_start, // Points to [TYPE, LENGTH_FIELD, FRAG_HDR(6), MAVLINK_SLICE(...)]
    uint8_t original_syslink_length_field, // This is the value of the LENGTH_FIELD from Syslink header
    std::function<void(uint8_t mav_byte)> mavlink_byte_pusher) {

    gcs().send_text(MAV_SEVERITY_DEBUG, "Syslink(1): Entered handle_frag for LEN=%d\n", original_syslink_length_field); // DEBUG
    gcs().send_text(MAV_SEVERITY_DEBUG, "Syslink(1): RawFragHdrBytes: %02X %02X %02X %02X %02X %02X\n", frag_data_start[0], frag_data_start[1], frag_data_start[2], frag_data_start[3], frag_data_start[4], frag_data_start[5]); // DEBUG

    // Skip TYPE and LENGTH_FIELD to get to fragment_header
    const uint8_t* fragment_header = frag_data_start + 1; // The MAVLink fragment header now starts *after* the 1-byte CRTP header

    uint16_t full_id = fragment_header[0] | (fragment_header[1] << 8);
    gcs().send_text(MAV_SEVERITY_DEBUG, "Debug: full_id = 0x%04X (from %02X %02X)", (unsigned)full_id, fragment_header[0], fragment_header[1]); // DEBUG
    uint16_t original_mav_len = fragment_header[2] | (fragment_header[3] << 8);
    gcs().send_text(MAV_SEVERITY_DEBUG, "Debug: original_mav_len = %u (from %02X %02X)", (unsigned)original_mav_len, fragment_header[2], fragment_header[3]); // DEBUG
    uint8_t total_frags = fragment_header[4];
    gcs().send_text(MAV_SEVERITY_DEBUG, "Debug: total_frags = %u (from %02X)", (unsigned)total_frags, fragment_header[4]); // DEBUG
    uint8_t seq = fragment_header[5];
    gcs().send_text(MAV_SEVERITY_DEBUG, "Debug: seq = %u (from %02X)", (unsigned)seq, fragment_header[5]); // DEBUG

    const uint8_t* mavlink_slice_ptr = fragment_header + 6;
    int mavlink_slice_len = original_syslink_length_field - 7; // 1 is the CRTP header, 6 is size of fragment header

    if (mavlink_slice_len < 0) {
        gcs().send_text(MAV_SEVERITY_ALERT, "Syslink(1): Invalid slice len %d\n", mavlink_slice_len); // DEBUG
        return;
    }

    gcs().send_text(MAV_SEVERITY_DEBUG, "Syslink(1): Parsed Frag ID=0x%04X, Seq=%u/%u, OrigMAVLen=%u, SliceLen=%d\n", (unsigned)full_id, (unsigned)seq, (unsigned)total_frags,
    (unsigned)original_mav_len, mavlink_slice_len); // DEBUG

    FragmentBuffer& buf = reassembly_buffers[full_id]; // Creates if not exists
    if (buf.frags.empty()) { // First fragment for this ID
        buf.original_len = original_mav_len;
        buf.total_frags = total_frags;
    } else {
        if (buf.original_len != original_mav_len || buf.total_frags != total_frags) {
            gcs().send_text(MAV_SEVERITY_ALERT, "Syslink(1): Inconsistent header for ID %u\n", full_id); // DEBUG
            reassembly_buffers.erase(full_id);
            // Potentially start new if this is a valid first fragment
            // For now, just discard and wait for a clean sequence for this full_id
            return;
        }
    }
    buf.last_active_ms = AP_HAL::millis();

    if (buf.frags.find(seq) == buf.frags.end()) { // Store if new
        buf.frags[seq] = uint8_t_vector(mavlink_slice_ptr, mavlink_slice_ptr + mavlink_slice_len);
        gcs().send_text(MAV_SEVERITY_DEBUG, "Syslink(1): Frag RX ID=%u, Seq=%u/%u, SliceLen=%d\n", full_id, seq, total_frags, mavlink_slice_len); // DEBUG
    } else {
        // Duplicate fragment, ignore or log
    }

    if (buf.frags.size() == total_frags) { // Check if all fragments are present
        bool complete_and_ordered = true;
        for (uint8_t i = 0; i < total_frags; ++i) {
            if (buf.frags.find(i) == buf.frags.end()) {
                complete_and_ordered = false;
                break;
            }
        }

        if (complete_and_ordered) {
            uint8_t_vector reassembled_mavlink_msg;
            for (uint8_t i = 0; i < total_frags; ++i) {
                reassembled_mavlink_msg.insert(reassembled_mavlink_msg.end(), buf.frags[i].begin(), buf.frags[i].end());
            }

            if (reassembled_mavlink_msg.size() == original_mav_len) {
                // Success! Push each byte of the reassembled MAVLink message.
                for (uint8_t mav_byte : reassembled_mavlink_msg) {
                    mavlink_byte_pusher(mav_byte);
                }
                gcs().send_text(MAV_SEVERITY_DEBUG, "Syslink(1): MAVLink Reassembled! ID=%u, TotalLen=%u\n", full_id, original_mav_len); // DEBUG
            } else {
                gcs().send_text(MAV_SEVERITY_ALERT, "Syslink(1): Reassembled len mismatch for ID %u\n", full_id); // DEBUG
            }
        } else {
            gcs().send_text(MAV_SEVERITY_ALERT, "Syslink(1): Missing frags for ID %u on completion check\n", full_id); // DEBUG
        }
        reassembly_buffers.erase(full_id); // Clean up this assembly buffer
    }
    cleanup_stale_reassembly_buffers(); // Call this periodically or after operations
}

void SyslinkToMAVLinkReassembler::cleanup_stale_reassembly_buffers() {
    // Iterate through reassembly_buffers and remove entries older than a timeout
    // (5 seconds) to prevent memory leaks from incomplete transmissions.
    const uint32_t now = AP_HAL::millis();
    const uint32_t timeout_ms = 5000; // 5 seconds
    for (auto it = reassembly_buffers.begin(); it != reassembly_buffers.end(); /* no increment */) {
        if (now - it->second.last_active_ms > timeout_ms) {
            // gcs().send_text(MAV_SEVERITY_DEBUG, "Syslink: Timeout for full_id %u", it->first); // DEBUG
            it = reassembly_buffers.erase(it);
        } else {
            ++it;
        }
    }
}