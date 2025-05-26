#pragma once
#include <vector>
#include <cstdint>

struct Fragment { const uint8_t *ptr; uint8_t len; };

bool is_your_msg(const uint8_t *buf, uint8_t len);
std::vector<Fragment> fragment_buffer(const uint8_t *buf, uint8_t total_len, uint8_t max_chunk);