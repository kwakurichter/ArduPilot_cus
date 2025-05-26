#pragma once
#include <vector>
#include <cstdint>

struct Fragment { const uint8_t *ptr; uint8_t len; };

bool is_target_msg(const uint8_t *buf, uint8_t len);
std::vector<Fragment> fragment_buffer(const uint8_t *buf, uint8_t total_len, uint8_t max_chunk);
std::vector<Fragment> wrap_with_syslink(const std::vector<Fragment> &mavfrags, uint16_t full_msg_id, uint16_t original_len);