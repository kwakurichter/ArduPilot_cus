#ifndef RADIO_BUFFER_H
#define RADIO_BUFFER_H

#include <AP_HAL/AP_HAL.h>
// #include <GCS_MAVLink.h> // For MAVLINK_MAX_PACKET_LEN

// A structure to hold a single complete Syslink-wrapped packet
struct RadioPacket {
    uint16_t len;
    // A single Syslink packet can be up to 64 bytes
    uint8_t buf[64];
};

// The size of the ring buffer (how many packets it can hold)
#define RADIO_BUFFER_SIZE 200

// A thread-safe ring buffer for radio packets
class RadioPacketBuffer {
public:
    // Singleton accessor: provides a global access point to the single instance
    static RadioPacketBuffer& get_instance() {
        static RadioPacketBuffer instance;
        return instance;
    }

    // The drain function is now a public member method
    void drain_task();

    // Tries to add a packet to the buffer.
    // Returns flse if the buffer is full (packet is dropped)
    bool push(const uint8_t* pkt_buf, uint8_t pkt_len);

    // Tries to retrieve a packet from the buffer.
    // Returns false if the buffer is empty
    bool pop(RadioPacket& packet);

    // Checks if the buffer is empty
    bool is_empty();

    void register_scheduler_task();

    uint8_t free_space();

private:
    // Make the constructor private to enforce the singleton pattern
    RadioPacketBuffer() {}
    HAL_Semaphore sem; // Semaphore for thread-safe access
    RadioPacket buffer[RADIO_BUFFER_SIZE];
    volatile uint8_t head = 0;
    volatile uint8_t tail = 0;
    volatile uint8_t count = 0;
};

#endif // RADIO_BUFFER_H