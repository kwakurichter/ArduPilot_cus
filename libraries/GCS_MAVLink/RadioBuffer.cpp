#include <AP_HAL/AP_HAL.h>
#include <AP_Scheduler/AP_Scheduler.h>
#include "RadioBuffer.h"
#include "GCS_MAVLink.h"
#include <string.h> // For memcpy
#include "GCS.h"

extern const AP_HAL::HAL& hal;

// To access the global mavlink_comm_port array declared in GCS_MAVLink.cpp
extern AP_HAL::UARTDriver* mavlink_comm_port[MAVLINK_COMM_NUM_BUFFERS];

// Create a single, static instance of our buffer
//static RadioPacketBuffer g_radio_buffer;

// Tries to add a packet to the buffer.
bool RadioPacketBuffer::push(const uint8_t* pkt_buf, uint8_t pkt_len) {
    WITH_SEMAPHORE(sem); // Automatically takes and gives the semaphore

    if (count >= RADIO_BUFFER_SIZE) {
        // Buffer is full, packet will be dropped (tail droped)
        return false;
    }

    if (pkt_len > sizeof(buffer[tail].buf)) {
        // Packet is too large for our struct buffer, cannot store
        return false;
    }

    // Copy the packet data into the buffer at the current tail position
    buffer[tail].len = pkt_len;
    memcpy(buffer[tail].buf, pkt_buf, pkt_len);

    // Advance the tail index, wrapping around if necessary
    tail = (tail + 1) % RADIO_BUFFER_SIZE;
    count++;

    return true;
}

// Tries to retrieve a packet from the buffer
bool RadioPacketBuffer::pop(RadioPacket& packet) {
    WITH_SEMAPHORE(sem); // Automatically takes and gives the semaphore

    if (count == 0) {
        // Buffer is empty
        return false;
    }

    // Copy the packet from the head of the buffer to the provided packet struct
    packet.len = buffer[head].len;
    memcpy(packet.buf, buffer[head].buf, buffer[head].len);

    // Advance the head index, wrapping around if necessary
    head = (head + 1) % RADIO_BUFFER_SIZE;
    count--;

    return true;
}

// Checks if the buffer is empty
bool RadioPacketBuffer::is_empty() {
    WITH_SEMAPHORE(sem);
    return count == 0;
}

// =================================================================
// DRAIN BUFFER FUNCTION
// =================================================================
// This function will be registered as a timer process to run at high frequency
// Its job is to send queued packets when the nRF radio is ready
//void drain_radio_buffer() {
    // Check if the nRF radio is ready to receive data (RTS line is low)
//    const bool nrf_is_ready = (hal.gpio->read(HAL_GPIO_PIN_NRF_FLOW_CTRL) == 0);

    // If the nRF is not ready, can't send anything. Exit now
//    if (!nrf_is_ready) {
//        return;
//    }

//    RadioPacket packet_to_send;
    // Attempt to get a packet from our buffer
//    if (g_radio_buffer.pop(packet_to_send)) {
        // The buffer had a packet, and the nRF is ready. Send it now
        // We send to MAVLINK_COMM_1
//        if (mavlink_comm_port[MAVLINK_COMM_1] != nullptr) {
//            mavlink_comm_port[MAVLINK_COMM_1]->write(packet_to_send.buf, packet_to_send.len);
//        }
//    }
//}

void RadioPacketBuffer::register_scheduler_task()
{
    // Use a static bool to ensure we only ever register this task once
    static bool is_registered = false;
    if (is_registered) {
        return;
    }

    hal.scheduler->register_timer_process(FUNCTOR_BIND_MEMBER(&RadioPacketBuffer::drain_task, void));
    is_registered = true;
}


void RadioPacketBuffer::drain_task() {

    static uint32_t last_print_ms = 0;
    uint32_t now_ms = AP_HAL::millis();
    if (now_ms - last_print_ms > 2000) { // Print only every 2 seconds
        gcs().send_text(MAV_SEVERITY_DEBUG, "DRAIN_TASK: Running...");  // DEBUG
        last_print_ms = now_ms;
    }

    // Don't send anything until the handshake is complete.
    if (!g_syslink_ready) {
        return;
    }
    
    const bool nrf_is_ready = (hal.gpio->read(54) == 0);

    // Don't send anything until the nrf is ready
    if (!nrf_is_ready) {
        return;
    }

    RadioPacket packet_to_send;
    // Call the pop() method on this instance
    if (this->pop(packet_to_send)) {
        if (mavlink_comm_port[MAVLINK_COMM_2] != nullptr) {
            gcs().send_text(MAV_SEVERITY_DEBUG, "COMM_SEND: Using Drain path for chan %d", (int)MAVLINK_COMM_2); // DEBUG
            mavlink_comm_port[MAVLINK_COMM_2]->write(packet_to_send.buf, packet_to_send.len);
            g_syslink_ready = false;    // Add to reset the flag?
        }
    }
}

uint8_t RadioPacketBuffer::free_space() {
    WITH_SEMAPHORE(sem);
    return RADIO_BUFFER_SIZE - count;
}