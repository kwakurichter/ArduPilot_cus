#pragma once

#include <stdint.h>

// Diagnostic timing of the intervals accepted by the driver. These times do
// not include unknown sensor exposure/readout delay and do not affect fusion.
class AP_OpticalFlow_FlowDeck_Timing {
public:
    void add_sample(uint32_t end_us, uint32_t dt_us)
    {
        if (dt_us == 0) {
            return;
        }
        if (_accepted_us == 0) {
            _start_us = end_us - dt_us;
        }
        _end_us = end_us;
        const uint32_t relative_end_us = end_us - _start_us;
        // Twice the interval midpoint, weighted by its duration. Relative
        // unsigned times preserve precision across a micros() rollover.
        _weighted_midpoint += uint64_t(dt_us) * (2ULL * relative_end_us - dt_us);
        _accepted_us += dt_us;
    }

    uint32_t accepted_us() const { return _accepted_us; }
    uint32_t span_us() const { return _end_us - _start_us; }
    uint32_t last_age_us(uint32_t now_us) const { return now_us - _end_us; }
    uint32_t mean_age_us(uint32_t now_us) const
    {
        if (_accepted_us == 0) {
            return 0;
        }
        const uint32_t mean_offset_us = _weighted_midpoint / (2ULL * _accepted_us);
        return (now_us - _start_us) - mean_offset_us;
    }

private:
    uint64_t _weighted_midpoint = 0;
    uint32_t _start_us = 0;
    uint32_t _end_us = 0;
    uint32_t _accepted_us = 0;
};

// Bounded raw-status histogram. No interpretation of a status as valid or
// no-motion is made here; samples are counted before delta/quality/status gates.
class AP_OpticalFlow_FlowDeck_MotionStats {
public:
    struct Entry {
        uint16_t count;
        uint16_t nonzero;
        uint8_t status;
    } entries[8] {};
    uint16_t overflow_count = 0;
    uint16_t overflow_nonzero = 0;

    void add(uint8_t status, bool nonzero)
    {
        for (auto &entry : entries) {
            if (entry.count == 0 || entry.status == status) {
                entry.status = status;
                entry.count++;
                entry.nonzero += nonzero;
                return;
            }
        }
        overflow_count++;
        overflow_nonzero += nonzero;
    }
};
