#pragma once

#include <stdint.h>

// Caller holds the backend semaphore for both push and pop. No allocation or
// logger calls occur on the SPI callback; overflow drops the newest record.
class AP_OpticalFlow_FlowDeck_RawLog {
public:
    enum class Reason : uint8_t {
        Accepted = 0,
        InvalidInterval = 1,
        SpiFailure = 2,
        DeltaRejected = 3,
        QualityRejected = 4,
        MotionRejected = 5,
    };

    struct Sample {
        uint64_t time_us;
        uint32_t dt_us;
        uint32_t read_start_us;
        uint32_t read_end_us;
        uint32_t gyro_us;
        int16_t delta_x;
        int16_t delta_y;
        uint8_t quality;
        uint8_t motion;
        Reason reason;
        float gyro_x;
        float gyro_y;
        float gyro_z;
        uint32_t sequence;
        uint32_t dropped;
    };

    void push(Sample sample)
    {
        sample.sequence = ++_sequence;
        if (_count == CAPACITY) {
            _dropped++;
            return;
        }
        sample.dropped = _dropped;
        _samples[(_head + _count) % CAPACITY] = sample;
        _count++;
    }

    bool pop(Sample &sample)
    {
        if (_count == 0) {
            return false;
        }
        sample = _samples[_head];
        _head = (_head + 1) % CAPACITY;
        _count--;
        return true;
    }

    // Discard queued records when disabled, retaining diagnostic counters.
    void clear()
    {
        _head = 0;
        _count = 0;
    }

    static constexpr uint8_t CAPACITY = 16;

private:
    Sample _samples[CAPACITY] {};
    uint32_t _sequence = 0;
    uint32_t _dropped = 0;
    uint8_t _head = 0;
    uint8_t _count = 0;
};
