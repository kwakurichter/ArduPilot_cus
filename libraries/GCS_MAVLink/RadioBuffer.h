#ifndef RADIO_BUFFER_H
#define RADIO_BUFFER_H

#include <AP_HAL/AP_HAL.h>
#include "GCS_MAVLink.h"
// #include <GCS_MAVLink.h> // For MAVLINK_MAX_PACKET_LEN

// A structure to hold a single complete Syslink-wrapped packet
struct RadioPacket {
    uint16_t len;
    // A single Syslink packet can be up to 64 bytes
    uint8_t buf[64];
};

// The size of the ring buffer (how many packets it can hold)
#define RADIO_BUFFER_SIZE 16

bool is_nrf_radio_channel(mavlink_channel_t chan);

// A thread-safe ring buffer for radio packets
class RadioPacketBuffer {
public:
    // Singleton accessor: provides a global access point to the single instance
    //static RadioPacketBuffer& get_instance() {
    //    static RadioPacketBuffer instance;
    //    return instance;
    //}

    // Change singleton accessor to return a pointer
    static RadioPacketBuffer* get_singleton() {
        return _singleton;
    }

    // The drain function is now a public member method
    //void drain_task(mavlink_channel_t chan);
    void drain_task();

    // Tries to add a packet to the buffer.
    // Returns flse if the buffer is full (packet is dropped)
    bool push(const uint8_t* pkt_buf, uint8_t pkt_len);

    // Tries to retrieve a packet from the buffer.
    // Returns false if the buffer is empty
    bool pop(RadioPacket& packet);

    // Checks if the buffer is empty
    bool is_empty();

    void register_scheduler_task(mavlink_channel_t chan);

private:
    // Make the constructor private to enforce the singleton pattern
    //RadioPacketBuffer() {}
    //RadioPacketBuffer() : _chan(MAVLINK_COMM_0), _is_registered(false) {} // Initialize members
    HAL_Semaphore sem; // Semaphore for thread-safe access
    RadioPacket buffer[RADIO_BUFFER_SIZE];
    volatile uint8_t head = 0;
    volatile uint8_t tail = 0;
    volatile uint8_t count = 0;

    // Add member variables to store state
    mavlink_channel_t _chan;
    bool _is_registered;

    static RadioPacketBuffer* _singleton;

    uint32_t _last_drain_ms;
};

// Declare the function that will be called by the scheduler to drain the buffer
//void drain_radio_buffer();

#endif // RADIO_BUFFER_H