#include "mavlink_fragment.h"
#include <algorithm>

bool is_your_msg(const uint8_t *buf, uint8_t len) {
    // …extract and compare msgid…
}

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