#include "mavlink_fragment.h"
#include <algorithm>
#include <string.h>

#define MAVLINK_STX_V1 0xFE
#define MAVLINK_STX_V2 0xFD

//—extract and compare the MAVLink message ID— DEPRECIATED
uint32_t is_target_msg(const uint8_t *buf, uint16_t len) { // Changed len to uint16_t for consistency
    uint32_t extracted_msgid = UINT32_MAX; // Default to invalid

    if (buf == nullptr) return UINT32_MAX;

    if (buf[0] == MAVLINK_STX_V1) {
        if (len < 8) return UINT32_MAX; // MAVLink v1 min frame length is 8 (for header + empty payload + CRC)
        // MSGID is at buf[5] for MAVLink v1
        extracted_msgid = buf[5];
    } else if (buf[0] == MAVLINK_STX_V2) {
        if (len < 12) return UINT32_MAX; // MAVLink v2 min frame length is 12
        // MSGID (3 bytes) is at buf[7], buf[8], buf[9] for MAVLink v2
        extracted_msgid = uint32_t(buf[7]) | (uint32_t(buf[8]) << 8) | (uint32_t(buf[9]) << 16);
    } else {
        // Did not start with a known MAVLink STX byte
        return UINT32_MAX;
    }
    return extracted_msgid;
}

//—slice the raw MAVLink bytes into equal-sized chunks— DEPRECIATED
std::vector<Fragment> fragment_buffer(const uint8_t *buf, uint8_t total_len, uint8_t max_chunk) {
    std::vector<Fragment> out;
    uint8_t offset = 0;
    while (offset < total_len) {
        uint8_t this_len = std::min<uint8_t>(max_chunk, total_len - offset);
        out.push_back({ buf + offset, this_len });
        offset += this_len;
    }
    return out;
}

//—wrap each MAVLink fragment in your 12-byte syslink + fragment header— DEPRECIATED
std::vector<Fragment> wrap_with_syslink(const std::vector<Fragment> &mavfrags,
                                        uint16_t full_msg_id,
                                        uint16_t original_len)
{
    std::vector<Fragment> out;
    uint8_t total = mavfrags.size();
    for (uint8_t i = 0; i < total; i++) {
        // build a small buffer on the stack for the headers + data
        uint8_t tmp[64];
        uint8_t idx = 0;

        // Syslink START
        tmp[idx++] = 0xBC;
        tmp[idx++] = 0xCF;
        // TYPE
        //tmp[idx++] = 0x0B;  // = MAVLink
        tmp[idx++] = 0x00;  // = Radio Raw
        // LENGTH = fragment header + data
        tmp[idx++] = uint8_t(6 + mavfrags[i].len);

        // Fragment header
        tmp[idx++] = uint8_t(full_msg_id & 0xFF);
        tmp[idx++] = uint8_t(full_msg_id >> 8);
        tmp[idx++] = uint8_t(original_len & 0xFF);
        tmp[idx++] = uint8_t(original_len >> 8);
        tmp[idx++] = total;
        tmp[idx++] = i;

        // copy the MAVLink bytes
        memcpy(&tmp[idx], mavfrags[i].ptr, mavfrags[i].len);
        idx += mavfrags[i].len;

        // compute Fletcher-8 over TYPE, LEN, fragment header & data
        uint8_t c0 = 0, c1 = 0;
        for (uint8_t j = 2; j < idx; j++) {
            c0 = (c0 + tmp[j]) & 0xFF;
            c1 = (c1 + c0)     & 0xFF;
        }
        tmp[idx++] = c0;
        tmp[idx++] = c1;

        out.push_back({ tmp, idx });
    }
    return out;
}