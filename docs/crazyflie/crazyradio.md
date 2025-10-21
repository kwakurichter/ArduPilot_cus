# Crazyradio Guide
## Communication in ArduPilot
ArduPilot’s communication stack is built around the MAVLink protocol which is a lightweight, header-based, binary framing system that defines every telemetry packet, command, and parameter exchange between the drone and Ground Control Station (GCS) or companion computers.

On startup, each enabled serial, USB or radio interface is instantiated as a “COMM_PORT” with its own send and receive buffers managed by AP_HAL (Hardware Abstraction) layer. 

When the drone wants to send data (e.g. attitude, GPS, sensor readings), it calls into mavlink_msg_x_send() functions to serialize fields into a MAVLink frame (including sequence number, system/component IDs, length, and CRC), then hands that frame to the communication driver.

Incoming MAVLink data are fed into mavlink_parse_char(), which reconstructs messages, checks integrity, and sends them to the appropriate handler (parameter manager, mission planner, or scripting engine). 

By decoupling message definition (in an XML) from the transport layer, ArduPilot can easily support multiple MAVLink versions, throttle bandwidth per-port, and automatically fragment large payloads, ensuring robust communication across a wide range of links and bandwidth constraints.

## Communication on Crazyflie
Bitcraze’s Crazyflie communication pipeline layers a lightweight radio transport (CRTP) on top of an onboard routing protocol (Syslink) to shuttle data between the GCS and the STM32 flight MCU.

On the PC side, the Crazyflie Python library takes high-level data (joystick setpoints, parameter reads, commands, etc.) and wraps it into a CRTP packet (two-byte sync, a one-byte port ID, a one-byte length, plus payload).

That frame is sent over 2.4 GHz radio and picked up by the secondary NRF51 MCU onboard the Crazyflie. The NRF then parses the CRTP header and payload; if the packet’s port indicates it’s for the flight controller, the NRF then encloses the payload in a Syslink frame (sync bytes 0xBC, 0xCF, a type, and length byte) and writes it out on its UART to the STM32.

The STM32’s Syslink parser strips off the Syslink framing and hands the original payload to the appropriate places.

Telemetry and status from the STM32 take the reverse path: the STM32 encodes data into Syslink frames, sends them over UART to the NRF51, the NRF unwraps Syslink and repackages the raw payload into CRTP packets on the appropriate port, and transmits them back to the GCS.

This two-stage framing separates radio transport from onboard routing, minimizing overhead while ensuring reliable, low-latency communication.

## Custom Communication Pipeline
Our goal is to bridge ArduPilot’s MAVLink stack with Bitcraze’s CRTP/Syslink radio—using only the Crazyflie’s onboard NRF51—to avoid extra radio hardware, saving weight and power.

The pipeline must remain invisible to existing MAVLink streams and must not interfere with the NRF51’s built-in tasks (power management, OTA flashing, etc.).

### How It Works
1. **Initialization**
- On boot, the STM32 sends Syslink-framed configuration packets (radio channel, data rate, address) over UART to the idle NRF51.
- The NRF51 applies these settings, then indicates link readiness by setting its Clear to Send (CTS) pin low.

2. **Uplink: STM32 → NRF51 → GCS**
- ArduPilot serializes telemetry into MAVLink frames.
- If a frame exceeds the CRTP payload limit (31 bytes total), it’s split into fragments, each prepended with a small header (message ID, total length, fragment count & index).
- Fragments are wrapped in Syslink and buffered until CTS is low and the link is open.
- The NRF51 strips the Syslink envelope, attaches a 1-byte CRTP header (port ID + length), and queues fragments for 2.4 GHz transmission.

3. **Downlink: GCS → NRF51 → STM32**
- The GCS (via Crazyradio PA) takes in MAVLink data (heartbeats, initialization commands, etc.) and general commands from the user.
- It then checks if the MAVLink messages exceed the 31 byte CRTP limit and fragments (if needed), wraps them in CRTP, and sends them to the NRF51.
- The NRF51 unwraps CRTP into Syslink frames and forwards them to the STM32 over UART.
- The STM32’s Syslink parser re-builds the MAVLink packet and feeds it into ArduPilot’s MAVLink parser.

By layering CRTP (radio transport) over Syslink (onboard routing), we keep transport details separate from flight-controller logic—minimizing overhead while ensuring robust, low-latency MAVLink communication.

## Modifying the Firmware
To get the radio working, we need to modify the legacy ArduPilot firmware. 

Note that this solution is still in active development and will eventually be coalesced into a more traditional driver structure to match existing radio communication in ArduPilot.

***Modify the Send Function***

We start by modifying the function which sends all bytes from the STM32. We are essentially intercepting the MAVLink stream on the NRF51 serial port right before it is sent over UART, and replacing the raw MAVLink data with our custom protocol:
- In your development environment, navigate to the GCS_MAVLink file:
```
path\...\libraries\GCS_MAVLink\GCS_MAVLink.cpp
```
- Near the middle of the file, find the comm_send_buffer function:
```
void comm_send_buffer(mavlink_channel_t chan, const uint8_t *buf, uint8_t len)
...
```
- Replace the contents of this function with the following:
```
void comm_send_buffer(mavlink_channel_t chan, const uint8_t *buf, uint16_t len)
{
    if (!valid_channel(chan) || mavlink_comm_port[chan] == nullptr || chan_discard[chan]) {
        return;
    }

    // This logic is for the nRF radio channel (MAVLINK_COMM_2)
    if (chan == MAVLINK_COMM_2) {
        // --- START P2P REASSEMBLY & INTERCEPTION LOGIC ---

        // Static buffer to reassemble MAVLink chunks
        static uint8_t p2p_mavlink_buf[MAVLINK_MAX_PACKET_LEN];
        static uint8_t p2p_mavlink_idx = 0;
        static uint8_t expected_mavlink_len = 0;

        // Append incoming data to our reassembly buffer
        if ((p2p_mavlink_idx + len) <= MAVLINK_MAX_PACKET_LEN) {
            memcpy(&p2p_mavlink_buf[p2p_mavlink_idx], buf, len);
            p2p_mavlink_idx += len;
        } else {
            // Buffer overflow, something is wrong. Reset.
            p2p_mavlink_idx = 0;
            expected_mavlink_len = 0;
            return;
        }

        // Check if we have enough data for a MAVLink header
        if (p2p_mavlink_idx >= 2) {
            if (expected_mavlink_len == 0) {
                // Determine total expected length from the MAVLink header
                if (p2p_mavlink_buf[0] == MAVLINK_STX) { // MAVLink 2
                    expected_mavlink_len = p2p_mavlink_buf[1] + 12; // Payload len + 12 bytes overhead
                } else if (p2p_mavlink_buf[0] == MAVLINK_STX_MAVLINK1) { // MAVLink 1
                    expected_mavlink_len = p2p_mavlink_buf[1] + 8; // Payload len + 8 bytes overhead
                }
            }

            // Do we have the complete packet yet?
            if (expected_mavlink_len > 0 && p2p_mavlink_idx >= expected_mavlink_len) {
                // We have a full MAVLink packet in p2p_mavlink_buf
                bool is_p2p_message = false;

                // Check for MAVLink v2
                if (p2p_mavlink_buf[0] == MAVLINK_STX) {
                    uint32_t msg_id = p2p_mavlink_buf[7] | (p2p_mavlink_buf[8] << 8) | (p2p_mavlink_buf[9] << 16);
                    if (msg_id == MAVLINK_MSG_ID_HEARTBEAT || msg_id == MAVLINK_MSG_ID_ATTITUDE) {
                        // Define a system and component ID for the Peer.
                        // This identifies the Peer as the source of the MAVLink message.
                        p2p_mavlink_buf[5] = 1;                   // System ID:
                        p2p_mavlink_buf[6] = MAV_COMP_ID_USER1;   // Component ID: IMU

                        is_p2p_message = true;
                    }
                }
                // Check for MAVLink v1
                else if (p2p_mavlink_buf[0] == MAVLINK_STX_MAVLINK1) {
                    uint8_t msg_id = p2p_mavlink_buf[5];
                    if (msg_id == MAVLINK_MSG_ID_HEARTBEAT || msg_id == MAVLINK_MSG_ID_ATTITUDE) {
                        // Define a system and component ID for the Peer.
                        // This identifies the Peer as the source of the MAVLink message.
                        p2p_mavlink_buf[3] = 1;                   // System ID:
                        p2p_mavlink_buf[4] = MAV_COMP_ID_USER1;   // Component ID: IMU                        
                        
                        is_p2p_message = true;
                    }
                }                                

                if (is_p2p_message) {
                    // This is a heartbeat, let's wrap it for P2P
                    uint8_t p2p_packet[58]; // Buffer for the P2P packet
                    uint8_t p2p_idx = 0;

                    // 1. CRTP Header for P2P
                    p2p_packet[p2p_idx++] = 0xff;
                    p2p_packet[p2p_idx++] = 0x80 | (0 & 0x0f); // Port 0

                    // 2. Copy the MAVLink heartbeat payload
                    memcpy(&p2p_packet[p2p_idx], p2p_mavlink_buf, expected_mavlink_len);
                    p2p_idx += expected_mavlink_len;

                    // 3. Syslink Header
                    uint8_t syslink_packet[64];
                    uint8_t syslink_idx = 0;
                    syslink_packet[syslink_idx++] = 0xBC;
                    syslink_packet[syslink_idx++] = 0xCF;
                    syslink_packet[syslink_idx++] = 0x0A; // TYPE = P2P Broadcast
                    syslink_packet[syslink_idx++] = p2p_idx; // LENGTH

                    // 4. Copy CRTP-wrapped MAVLink packet
                    memcpy(&syslink_packet[syslink_idx], p2p_packet, p2p_idx);
                    syslink_idx += p2p_idx;

                    // 5. Fletcher-8 Checksum
                    uint8_t c0=0, c1=0;
                    for (uint8_t j = 2; j < syslink_idx; j++) {
                        c0 += syslink_packet[j];
                        c1 += c0;
                    }
                    syslink_packet[syslink_idx++] = c0;
                    syslink_packet[syslink_idx++] = c1;

                    // 6. Push to Radio Buffer
                    RadioPacketBuffer::get_instance().push(syslink_packet, syslink_idx);

                    // -- DEBUG --
                    //ExpandingString hex_dump;
                    //hex_dump.printf("P2P Sent(%u): ", syslink_idx);
                    //for (uint8_t i = 0; i < syslink_idx; i++) {
                    //    hex_dump.printf("%02X ", syslink_packet[i]);
                    //}
                    //gcs().send_text(MAV_SEVERITY_ALERT, "%s", hex_dump.get_string());
                    // -- DEBUG --

                    // Heartbeat sent via P2P, so we skip the normal GCS path
                    // return;
                } else {
                    // Define the chunk size for fragmentation
                    static const int MAV_CHUNK = 24;

                    // Calculate how many fragments this MAVLink message will be split into
                    uint8_t total_syslink_fragments = (expected_mavlink_len + MAV_CHUNK - 1) / MAV_CHUNK;

                    // --- ATOMIC BUFFERING LOGIC ---
                    // Check if the radio buffer has enough space for ALL fragments of this message
                    if (RadioPacketBuffer::get_instance().free_space() < total_syslink_fragments) {
                        // Not enough space for the entire message, drop it.
                        // gcs().send_text(MAV_SEVERITY_WARNING, "Radio buffer full, MAVLink msg dropped!");
                        return;
                    }

                    // If we get here, there is enough space. Proceed with fragmentation and buffering.
                    uint16_t syslink_fragmentation_full_id = g_syslink_message_id_counter++;
                    uint8_t offset = 0;

                    while (offset < expected_mavlink_len)
                    {
                        uint8_t this_len = std::min((uint16_t)MAV_CHUNK, (uint16_t)(expected_mavlink_len - offset));
                        uint8_t length_field = 6 + this_len; // 6B fragment header + data
                        uint8_t packet[36]; // Buffer for one fragment
                        uint8_t idx = 0;

                        // 1) Syslink header
                        packet[idx++] = 0xBC;
                        packet[idx++] = 0xCF;
                        packet[idx++] = 0x0B; // TYPE = Radio MAVLink
                        packet[idx++] = length_field;

                        // 2) Fragment header
                        packet[idx++] = uint8_t(syslink_fragmentation_full_id & 0xFF);
                        packet[idx++] = uint8_t(syslink_fragmentation_full_id >> 8);
                        packet[idx++] = uint8_t(expected_mavlink_len & 0xFF);
                        packet[idx++] = uint8_t(expected_mavlink_len >> 8);
                        packet[idx++] = total_syslink_fragments;
                        packet[idx++] = uint8_t(offset / MAV_CHUNK);

                        // 3) Payload slice
                        if (this_len > 0) {
                            memcpy(packet + idx, p2p_mavlink_buf + offset, this_len);
                        }
                        idx += this_len;
                        
                        // 4) Fletcher-8 checksum
                        uint8_t c0=0, c1=0;
                        for (uint8_t j = 2; j < idx; j++) {
                            c0 += packet[j];
                            c1 += c0;
                        }
                        packet[idx++] = c0;
                        packet[idx++] = c1;

                        // 5) Push the fragment to the buffer. We've already confirmed space exists.
                        // We ignore the return value as we've pre-checked the space.
                        RadioPacketBuffer::get_instance().push(packet, idx);

                        offset += this_len;
                    }
                }

                // Reset the buffer for the next message
                p2p_mavlink_idx = 0;
                expected_mavlink_len = 0;
            }
        }
        // --- END P2P REASSEMBLY & INTERCEPTION LOGIC ---
        return; // Important: We handle all MAVLINK_COMM_2 traffic inside this block now.
    }

    // For all other MAVLink channels, use the regular send
    mavlink_comm_port[chan]->write(buf, len);
}
```

This function now replaces the MAVLink stream on the NRF serial port with the custom protocol, while leaving the other serial ports untouched.

We also need to add the following at the top of the file for later:
- Under the includes at the top, add the following:
```
#include "GCS.h"
#include "GCS_MAVLink.h"
#include "RadioBuffer.h"		<-- Add this
```
- Also, add the following variables:
```
bool g_syslink_ready = false; // The flag to indicate NRF is ready
static uint16_t g_syslink_message_id_counter = 0; // For Crazyflie Syslink Packet ID
```

Next, we need to modify the corresponding header file:
- Navigate to the GCS_MAVLink header file:
```
path\...\libraries\GCS_MAVLink\GCS_MAVLink.h
```
- At the top, add the following includes:
```
#include <vector>
#include <cstdint>
```
- Add the definition for the flag variable from earlier:
```
extern bool g_syslink_ready;    // Indicates when NRF is ready to receive
```

***Add New Drivers***

The next step is to add the new driver files which handle processing the outgoing Syslink stream ([RadioBuffer.cpp](../../libraries/GCS_MAVLink/RadioBuffer.cpp) and [RadioBuffer.h](../../libraries/GCS_MAVLink/RadioBuffer.h)) and the incoming Syslink stream ([SyslinkReassembler.cpp](../../libraries/GCS_MAVLink/SyslinkReassembler.cpp) and [SyslinkReassembler.h](../../libraries/GCS_MAVLink/SyslinkReassembler.h)):
- Navigate to the GCS_MAVLink folder:
```
path\...\ardupilot\libraries\GCS_MAVLink
```
- Add the attached files to this directory:
```
path\...\ardupilot\libraries\GCS_MAVLink\RadioBuffer.cpp
path\...\ardupilot\libraries\GCS_MAVLink\RadioBuffer.h
path\...\ardupilot\libraries\GCS_MAVLink\SyslinkReassembler.cpp
path\...\ardupilot\libraries\GCS_MAVLink\SyslinkReassembler.h
```

***Modify the Main Functions***

The next step is to modify the main GCS implementation in ArduPilot:
- Head to the GCS_Common file:
```
path\...\ardupilot\libraries\GCS_MAVLink\GCS_Common.cpp
```
- In the includes at the top, add the following:
```
#include "GCS_config.h"

 #if HAL_GCS_ENABLED
 
 #include "GCS.h"
 #include "SyslinkReassembler.h"	<-- ADD THIS
 #include "RadioBuffer.h"		    <-- ADD THIS
 #include <AP_HAL/AP_HAL.h>         <-- ADD THIS
```
- Add this line under the includes:
```
static SyslinkToMAVLinkReassembler s_syslink_reassembler_for_comm1;
```
- Next, find the following definition:
```
GCS_MAVLINK::GCS_MAVLINK(GCS_MAVLINK_Parameters &parameters,
                          AP_HAL::UARTDriver &uart)
 {
    _port = &uart;
 
    streamRates = parameters.streamRates;
 }
```
- Underneath the definiton, add the following functions:
```
 // Helper function to calculate the Fletcher-8 checksum used by Syslink
static void calculate_fletcher8(const uint8_t *data, uint8_t len, uint8_t &ck_a, uint8_t &ck_b)
{
    uint8_t c0 = 0;
    uint8_t c1 = 0;

    for (uint8_t i = 0; i < len; i++) {
        c0 += data[i];
        c1 += c0;
    }

    ck_a = c0;
    ck_b = c1;
}

static void send_packet_blocking(AP_HAL::UARTDriver* port, const uint8_t* data, uint8_t size)
{
    if (port == nullptr) {
        return;
    }

    port->write(data, size);

    // Wait until the software transmit buffer is empty, using the tx_pending() method.
    // This indicates the hardware has taken all the data for transmission.
    // We add a 100ms timeout to prevent the system from hanging forever.
    const uint32_t start_ms = AP_HAL::millis();
    while (port->tx_pending()) {
        if (AP_HAL::millis() - start_ms > 100) {
            // gcs().send_text(MAV_SEVERITY_WARNING, "NRF: UART TX timeout");
            break;
        }
        hal.scheduler->delay(1);
    }
}
 
 static void send_syslink_config_packets(AP_HAL::UARTDriver* port)
{
    if (port == nullptr) {
        return;
    }

    gcs().send_text(MAV_SEVERITY_ALERT, "NRF_INIT: Sending config packets..."); // DEBUG

    uint8_t ck_a, ck_b;

    // --- Packet 1: Set Radio Channel to 80 (0x50) ---
    const uint8_t packet1_data[] = { 0x01, 0x01, 0x50 }; // Type, Length, Data
    calculate_fletcher8(packet1_data, sizeof(packet1_data), ck_a, ck_b);
    const uint8_t packet1[] = { 0xBC, 0xCF, packet1_data[0], packet1_data[1], packet1_data[2], ck_a, ck_b };
    //port->write(packet1, sizeof(packet1));
    //hal.scheduler->delay(5); // Small delay for the NRF to process
    send_packet_blocking(port, packet1, sizeof(packet1));

    // --- Packet 2: Set Data Rate to 2M (0x02) ---
    const uint8_t packet2_data[] = { 0x02, 0x01, 0x02 }; // Type, Length, Data
    calculate_fletcher8(packet2_data, sizeof(packet2_data), ck_a, ck_b);
    const uint8_t packet2[] = { 0xBC, 0xCF, packet2_data[0], packet2_data[1], packet2_data[2], ck_a, ck_b };
    //port->write(packet2, sizeof(packet2));
    //hal.scheduler->delay(5);
    send_packet_blocking(port, packet2, sizeof(packet2));

    // --- Packet 3: Set Radio Address to E7E7E7E701 (user configurble) ---
    // The Crazyflie firmware expects the 5-byte address in little-endian byte order.
    // So, 0xE7E7E7E701 is sent as {0x01, 0xE7, 0xE7, 0xE7, 0xE7}.
    const uint8_t packet3_data[] = { 0x05, 0x05, 0x01, 0xE7, 0xE7, 0xE7, 0xE7 }; // Type, Length, Data
    calculate_fletcher8(packet3_data, sizeof(packet3_data), ck_a, ck_b);
    const uint8_t packet3[] = { 0xBC, 0xCF, packet3_data[0], packet3_data[1], packet3_data[2], packet3_data[3], packet3_data[4], packet3_data[5], packet3_data[6], ck_a, ck_b };
    //port->write(packet3, sizeof(packet3));
    //hal.scheduler->delay(5);
    send_packet_blocking(port, packet3, sizeof(packet3));

    // --- Packet 4: Set Radio Power to +6dBm (0x06) (Optional) ---
    //const uint8_t packet4_data[] = { 0x07, 0x01, 0x06 }; // Type, Length, Data
    //calculate_fletcher8(packet4_data, sizeof(packet4_data), ck_a, ck_b);
    //const uint8_t packet4[] = { 0xBC, 0xCF, packet4_data[0], packet4_data[1], packet4_data[2], ck_a, ck_b };
    //port->write(packet4, sizeof(packet4));
    //hal.scheduler->delay(5);

    gcs().send_text(MAV_SEVERITY_ALERT, "NRF_INIT: Config packets sent to port: %u", (int)port);  // DEBUG
}
```

These functions act as helpers to send the initial configuration packets that must be sent to the NRF for it to begin forwarding GCS data. 

Along with these definitions we need to modify the init function:
- Find the init function:
```
bool GCS_MAVLINK::init(uint8_t instance)
...
```
- Under this block of code:
```
     // and init the gcs instance
 
     // whether this port is considered "private" is stored on the uart
     // rather than in our own parameters:
     if (uartstate->option_enabled(AP_HAL::UARTDriver::OPTION_MAVLINK_NO_FORWARD)) {
         set_channel_private(chan);
     }
```
- Add the following:
```
     if (chan == MAVLINK_COMM_2) {
        RadioPacketBuffer::get_instance().register_scheduler_task();    // register the drain task to run in parallel
        gcs().send_text(MAV_SEVERITY_DEBUG, "INIT: Registered drain task for chan %d", (int)instance);

        // Send the initial config packets required by the NRF firmware
        send_syslink_config_packets(_port);
   
     }
```

This registers a new task on a separate thread which is responsible for managing the buffer which is used to send packets. This task regularly checks if the send conditions we discussed earlier (flow control and initial activation flag) are true, which indicates the NRF is ready to receive a packet. If these conditions are met, this task will send a packet over serial. 

By registering this task on a separate thread, we reduce latency by ensuring packets are not being bottlenecked by the main GCS loop.

Next, we need to modify the function which handles received data.
- Find the update_receive function:
```
void
GCS_MAVLINK::update_receive(uint32_t max_time_us)
...
```

- Replace function with the following:
```
 void
 GCS_MAVLINK::update_receive(uint32_t max_time_us)
 {
     // do absolutely nothing if we are locked
     if (locked()) {
         return;
     }
 
     // receive new packets
     mavlink_message_t msg;
     mavlink_status_t status;
     uint32_t tstart_us = AP_HAL::micros();
     uint32_t now_ms = AP_HAL::millis();
 
     status.packet_rx_drop_count = 0;
 
     //uint32_t now_ms = AP_HAL::millis(); // Ensure now_ms is available
 
     const uint16_t nbytes = _port->available();
     for (uint16_t i=0; i<nbytes; i++)
     {
         const uint8_t c = (uint8_t)_port->read();
         const uint32_t protocol_timeout = 4000;
         bool byte_handled_by_syslink = false;
 
         // --- BEGIN SYSLINK PRE-PROCESSING FOR MAVLINK_COMM_2 ---
         if (chan == MAVLINK_COMM_2) {
             //gcs().send_text(MAV_SEVERITY_DEBUG, "COMM_2 RAW RX: 0x%02X", (unsigned)c);  // DEBUG

             // Define the callback that SyslinkReassembler will use to push MAVLink bytes
             auto mavlink_byte_pusher_lambda = 
                 [&](uint8_t mav_byte) { // Captures needed variables by reference
                 const uint8_t framing = mavlink_frame_char_buffer(channel_buffer(), channel_status(), mav_byte, &msg, &status);
                 if (framing == MAVLINK_FRAMING_OK) {
                     // gcs().send_text(MAV_SEVERITY_DEBUG, "MSGID: %.2u Received\n", msg.msgid);    // DEBUG 

                     // This is the first successful packet from the NRF. The handshake is complete.
                     if (!g_syslink_ready) {
                        g_syslink_ready = true;
                        gcs().send_text(MAV_SEVERITY_DEBUG, "NRF_INIT: Syslink ready!");    // DEBUG
                     }
                    
                     // DEBUG - Not working?
                     //hal.gpio->write(11, 1); // Turn ON LED_GREEN_L (PC1, pin 11)
                     //hal.scheduler->delay_microseconds(1000); // Wait 1000 microseconds (1ms)
                     //hal.gpio->write(11, 0); // Immediately turn OFF for a quick flash
                     //gcs().send_text(MAV_SEVERITY_DEBUG, "Syslink(1)->MAV: Decoded MAVLink MSG ID %u\n", msg.msgid); 
                     // DEBUG

                     hal.util->persistent_data.last_mavlink_msgid = msg.msgid;
                     packetReceived(status, msg); // Process the MAVLink packet

                     // Update Battery Data sent from NRF51
                     if (msg.msgid == MAVLINK_MSG_ID_BATTERY_STATUS) {
                        // gcs().send_text(MAV_SEVERITY_CRITICAL, "Battery status received\n");    // DEBUG 

                        // Get a reference to the main battery monitor object
                        AP_BattMonitor &battery_mon = AP::battery();                        
                        
                        // 1. Decode the incoming MAVLink message
                        mavlink_battery_status_t batt_status;
                        mavlink_msg_battery_status_decode(&msg, &batt_status);

                        // 2. Create and populate the state struct that the scripting backend expects
                        BattMonitorScript_State script_state{};                 

                        // Voltage: MAVLink is in mV, struct expects V
                        script_state.voltage = batt_status.voltages[0] / 1000.0f;        

                        // Temperature: MAVLink is in cdegC, struct expects degC
                        if (batt_status.temperature != INT16_MAX) {
                            script_state.temperature = batt_status.temperature / 100.0f;
                        } else {
                            script_state.temperature = NAN;
                        }

                        // Correctly copy only the available voltage data (10 cells)
                        memcpy(script_state.cell_voltages, batt_status.voltages, sizeof(batt_status.voltages));

                        // Also copy the extended voltage data (cells 11-14)
                        memcpy(&script_state.cell_voltages[10], batt_status.voltages_ext, sizeof(batt_status.voltages_ext));

                        script_state.cell_count = 1;

                        // Set other fields to "unknown"
                        script_state.current_amps = NAN;
                        script_state.consumed_mah = NAN;
                        script_state.capacity_remaining_pct = UINT8_MAX;
                        script_state.consumed_wh = NAN;
                        script_state.cycle_count = UINT16_MAX;

                        // Set health status
                        script_state.healthy = true;                    
                        
                        // 3. Call the handler to inject the data into the battery monitor system
                        battery_mon.handle_scripting(0, script_state);

                        gcs().send_text(MAV_SEVERITY_DEBUG, "VBAT: %.2f V, %.2f C", (double)script_state.voltage, (double)script_state.temperature);    // DEBUG  
                     }
 
                     gcs_alternative_active[chan] = false; // MAVLink is active
                     alternative.last_mavlink_ms = now_ms; // Update MAVLink activity timestamp
                     hal.util->persistent_data.last_mavlink_msgid = 0;
                 }
                 #if AP_SCRIPTING_ENABLED
                 else if (framing == MAVLINK_FRAMING_BAD_CRC) {
                     AP_Scripting* scripting = AP_Scripting::get_singleton();
                     if (scripting != nullptr) {
                         scripting->handle_message(msg, chan);
                     }
                 }
                 #endif
             };
             auto p2p_packet_handler_lambda =
                [&](const uint8_t* payload, uint8_t len) {
                // Check if the received P2P payload is a heartbeat

                if (payload[0] == MAVLINK_STX) {
                    uint32_t msg_id = payload[7] | (payload[8] << 8) | (payload[9] << 16);

                    if (msg_id == MAVLINK_MSG_ID_HEARTBEAT) {
                        //gcs().send_text(MAV_SEVERITY_DEBUG, "P2P Heartbeat Received, forwarding to AI Deck...");    // DEBUG                        
                    }
                    if (msg_id == MAVLINK_MSG_ID_ATTITUDE) {
                        //gcs().send_text(MAV_SEVERITY_DEBUG, "P2P Attitude Received, forwarding to AI Deck...");    // DEBUG                        
                    }                    
                    // --- FORWARDING LOGIC ---
                    // Check if the target MAVLink port for the AI Deck is valid and initialized
                    if (mavlink_comm_port[MAVLINK_COMM_1] != nullptr) {
                        // Forward the raw MAVLink message directly to the AI Deck's serial port.
                        mavlink_comm_port[MAVLINK_COMM_1]->write(payload, len);
                    }
                }
             };                
             
             byte_handled_by_syslink = s_syslink_reassembler_for_comm1.process_byte(c, mavlink_byte_pusher_lambda, p2p_packet_handler_lambda);
         }
         // --- END SYSLINK PRE-PROCESSING ---  
         
         if (byte_handled_by_syslink) {
             // If Syslink logic (on MAVLINK_COMM_1) consumed or processed the byte 'c',
             // skip the default MAVLink/alternative protocol handling for this byte.
             // The mavlink_byte_pusher_lambda already updated alternative.last_mavlink_ms
             // if MAVLink was successfully generated from Syslink.
             continue;
         }        
         
         // Original MAVLink / alternative protocol handling for byte 'c'
         // This part runs if 'chan' is not MAVLINK_COMM_1, or if Syslink didn't consume the byte.
         bool parsed_packet_std = false; // Use a different variable name
         if (alternative.handler &&
             now_ms - alternative.last_mavlink_ms > protocol_timeout) {
             if (alternative.handler(c, mavlink_comm_port[chan])) {
                 alternative.last_alternate_ms = now_ms;
                 gcs_alternative_active[chan] = true;
             }
             if (now_ms - alternative.last_alternate_ms <= protocol_timeout) {
                 continue;
             }
         }
 
         const uint8_t framing_std = mavlink_frame_char_buffer(channel_buffer(), channel_status(), c, &msg, &status);
         if (framing_std == MAVLINK_FRAMING_OK) {
             hal.util->persistent_data.last_mavlink_msgid = msg.msgid;
             packetReceived(status, msg);
             parsed_packet_std = true;
             gcs_alternative_active[chan] = false;
             alternative.last_mavlink_ms = now_ms;
             hal.util->persistent_data.last_mavlink_msgid = 0;
         }
         #if AP_SCRIPTING_ENABLED
         else if (framing_std == MAVLINK_FRAMING_BAD_CRC) {
             AP_Scripting *scripting = AP_Scripting::get_singleton();
             if (scripting != nullptr) {
                 scripting->handle_message(msg, chan);
             }
         }
         #endif
 
         if (parsed_packet_std || i % 100 == 0) {
             if (AP_HAL::micros() - tstart_us > max_time_us) { // tstart_us needs to be defined earlier
                 break;
             }
         }
     }
 
     const uint32_t tnow = AP_HAL::millis();
 
     // send a timesync message every 10 seconds; this is for data
     // collection purposes
```

This block checks if the byte it has received over serial was sent by the NRF. If it was, the byte is linked to the SyslinkReassembler we defined previously which reassembles full MAVLink messages. 

Once the full MAVLink message has been reassembled, it is pushed back to this update_receive function. If the message passes verification, the init activation flag is set to true and the message is passed along to be further processed.

Similarly, P2P (Peer-to-Peer) packets which are received from the NRF51 are parsed and sent to the AI Deck for processing.

## Flashing the NRF51
The next step to getting the Crazyradio working in ArduPilot is to flash the NRF51 with a custom version of the legacy Bitcraze firmware. This step should be completed first before compiling and flashing the custom ArduPilot firmware.

Please reference the [Flashing the NRF Guide](flashing_the_nrf.md).

## Compiling & Flashing to the Crazyflie
Before using the new Crazyradio in ArduPilot, we need to compile the custom firmware and flash it the Crazyflie. For detailed flashing instructions, please reference the [Compiling & Flashing Guide](compiling_and_flashing.md).

If you attempt to compile your firmware and get a build failed error: 
```
Build failed -> task in 'bin/arducopter' failed (exit status 1)
```
Chances are you have exceeded the memory limit. Please reference the [Freeing up Memory Guide](freeing_up_memory.md) for detailed instructions on how to reduce the build size.

## Testing and Using Crazyradio
Once you have successfully flashed your custom firmware with Crazyradio, using the custom protocol is a bit more involved than the legacy options. First, we need to change all of the serial protocols to MAVLink2 by changing the following parameters in your GCS of choice:
```
param set SERIAL1_PROTOCOL 2
param set SERIAL2_PROTOCOL 2
param set SERIAL3_PROTOCOL 2
```
Next, we need to change the baud rate of the NRF serial port to match what the NRF is expecting. Set the following parameter:
```
param set SERIAL2_BAUD 1000
```
After changing these parameters and saving them to memory, restart the system by either cutting power to the drone directly or sending a reboot command through MAVLink.

Once the system has restarted, the communication link should now be active. Connect to your drone from the GCS by referencing the [Custom GCS Guide](custom_gcs_guide.md).
