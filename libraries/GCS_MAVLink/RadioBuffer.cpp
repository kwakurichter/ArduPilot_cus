#include <AP_HAL/AP_HAL.h>
#include <AP_Param/AP_Param.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include <AP_Scheduler/AP_Scheduler.h>
#include "RadioBuffer.h"
#include "GCS_MAVLink.h"
#include <string.h> // For memcpy
#include "GCS.h"

RadioPacketBuffer* RadioPacketBuffer::_singleton = nullptr;

extern const AP_HAL::HAL& hal;

// To access the global mavlink_comm_port array declared in GCS_MAVLink.cpp
extern AP_HAL::UARTDriver* mavlink_comm_port[MAVLINK_COMM_NUM_BUFFERS];

// Create a single, static instance of our buffer
//static RadioPacketBuffer g_radio_buffer;

// Helper function to check if a MAVLink channel corresponds to the NRF port
//bool is_nrf_radio_channel(mavlink_channel_t chan)
//{
    // Declare variables to hold the output from the find() function.
//    ap_var_type param_type;
//    uint16_t param_flags; // We don't use this variable, but the function requires a valid pointer.
    // --- 1. Find the parameter by its string name ---
    // The find() method is called with just the parameter name.
    // Replace "NRF_PORT" with the actual 16-character (max) name you gave your parameter.
//    AP_Param *p = AP_Param::find("NRF_PORT", &param_type, &param_flags);

//    int8_t nrf_port_number = 9;

    // --- 2. Check if parameter was found AND has the correct type ---
//    if (p != nullptr) {
        // Parameter was found and is the correct type (AP_Int8).
        // Now, cast the generic AP_Param pointer to the specific AP_Int8 pointer and get its value.
//        nrf_port_number = ((AP_Int8 *)p)->get();
//    } else {
        // The parameter either doesn't exist or is not the type we expect (e.g., AP_Int8).
        // In either case, we can't use it, so we'll treat Syslink as disabled.
//        return false;
//    }
    
    // If the parameter is set to -1 (or any negative value), Syslink is disabled.
//    if (nrf_port_number > 6) {
//        return false;
//    }

    // --- 3. Get the UART driver pointer for the channel we are currently processing ---
//    AP_HAL::UARTDriver *current_channel_driver = mavlink_comm_port[chan];
//    if (current_channel_driver == nullptr) {
//        return false;
//    }

    // --- 4. Get the UART driver pointer for the user-configured NRF port number ---
    // The get_port() method on AP_SerialManager takes the logical port number (e.g., 0 for SERIAL0, 1 for SERIAL1).
//    AP_HAL::UARTDriver *nrf_port_driver = AP::serialmanager().get_serial_by_id(nrf_port_number);
//    if (nrf_port_driver == nullptr) {
        // This could happen if the user sets NRF_PORT to a port that doesn't exist or isn't enabled.
//        return false;
//    }

    // --- 5. Compare the pointers. If they are the same, this is the correct port ---
//    return (current_channel_driver == nrf_port_driver);
//}

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

void RadioPacketBuffer::register_scheduler_task(mavlink_channel_t chan)
{
    // Use a static bool to ensure we only ever register this task once
    static bool is_registered = false;
    if (is_registered) {
        return;
    }

    // Store the channel as a member variable
    this->_chan = chan;

    hal.scheduler->register_timer_process(FUNCTOR_BIND_MEMBER(&RadioPacketBuffer::drain_task, void));
    is_registered = true;
}

//void RadioPacketBuffer::register_scheduler_task(mavlink_channel_t chan)
//{
    // Use a member variable to ensure this is only done once for the singleton
//    if (_is_registered) {
//        return;
//    }

//    _chan = chan; // Store the channel

    // --- Start of new code ---

    // Manually create the Functor object instead of using the macro.
    // This explicitly tells the compiler the types involved.
//    Functor<void> functor = Functor<void>::bind<RadioPacketBuffer, &RadioPacketBuffer::drain_task>(this);

    // Register the created functor.
//    hal.scheduler->register_timer_process(functor);

    // --- End of new code ---

//    _is_registered = true;
//}


//void RadioPacketBuffer::drain_task(mavlink_channel_t chan) {
void RadioPacketBuffer::drain_task() {

    // Note: We can't use gcs().send_text() here because this task runs too fast
    // and would flood the connection. We'll add a temporary, heavily-throttled print.
    //static uint32_t last_print_ms = 0;
    //uint32_t now_ms = AP_HAL::millis();
    //if (now_ms - last_print_ms > 2000) { // Print only every 2 seconds
    //    gcs().send_text(MAV_SEVERITY_CRITICAL, "DRAIN_TASK: Running...");
    //    last_print_ms = now_ms;
    //}

    // Limit the drain task to run at a maximum of 200Hz (every 5ms)
    const uint32_t now = AP_HAL::millis();
    if (now - _last_drain_ms < 5) {
        return;
    }
    _last_drain_ms = now;

    const bool nrf_is_ready = (hal.gpio->read(54) == 0);

    if (!nrf_is_ready) {
        return;
    }

    RadioPacket packet_to_send;
    // Call the pop() method on this instance
    if (this->pop(packet_to_send)) {
        if (mavlink_comm_port[this->_chan] != nullptr) {
            mavlink_comm_port[this->_chan]->write(packet_to_send.buf, packet_to_send.len);
        }
    }
}