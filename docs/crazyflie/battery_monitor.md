# Battery Monitor Setup
## Background
An important parameter to keep track of during autonomous flight is the battery status of the drone. The Crazyflie 2.1 uses a small, single cell 250 mah LiPo battery to power the entire system. As a result, the Crazyflie 2.1 can only fly for around 10 to 15 minutes off of a full charge. 

This means we need to carefully monitor the state of the battery to avoid crashes during an autonomous mission.

## Implementation
Most flight controllers monitor the state of the battery directly or through a external power management unit such as an ESC. The Crazyflie 2.1 delagates all of the power management tasks to its secondary MCU (Micro-Controller Unit), the NRF51.

In the Bitcraze firmware, the NRF51 polls the battery at regular intervals for state updates (i.e. voltage, temperature, charging status, etc.) and then sends the data over serial to the main STM32 flight controller over serial.

For ArduSwarm we adopt a similar strategy, using the MAVLink protocol rather than Bitcraze's proprietary protocols.

### How It Works
The battery monitor works by using the communication infrastructure developed for the [CrazyRadio](crazyradio.md) implementation. If you have not already enabled CrazyRadio on your drone, please follow that guide before proceeding.

1. **NRF51 Battery Status**
- On boot, the NRF51 begins polling the battery for state updates as a backend task.
- The NRF51 firmware provides access to these state updates on the frontend through a user API.

2. **Uplink: STM32 → NRF51 → GCS**
- The NRF51 requests the current voltage, temperature, and charging state, then serializes the battery status into MAVLink frames.
- If a frame exceeds the Syslink payload limit (64 bytes total), it’s split into fragments, each prepended with a small header (message ID, total length, fragment count & index).
- Fragments are wrapped in Syslink and CRTP and are buffered until the serial port is open.
- The STM32 strips the Syslink and CRTP envelope, reassembles the full MAVLink frames (if necessary), and passes the full frames to the MAVLink parser.
- Once parsed, battery status messages update the battery state by using the battery scripting driver.

## Modifying the Firmware
To get the battery monitor working, we need to modify the legacy ArduPilot firmware as well as the NRF51 firmware.

Note that this solution is still in active development and will eventually be coalesced into a more traditional driver structure to match existing battery monitors in ArduPilot.

### Modify the HWDEF file
Start by enabling battery monitor scripting in the hardware definition file:

- In your development environment, navigate to the Crazyflie hwdef file:
```
path\...\libraries\AP_HAL_ChibiOS\hwdef\crazyflie2\hwdef.dat
```
- Near the bottom of the file, find the line that minimizes the ArduPilot features:
```
include ../include/minimize_features.inc
```
- Underneath this line, include the following line: 
```
define AP_BATTERY_SCRIPTING_ENABLED 1       #Battery Monitor Support
```
This tells the compiler to enable the battery scripting driver in our ArduPilot build regardless of any conditional statements.

### Modify the Receive Function

Next, we modify the function which receives all bytes from the NRF51. We need to intercept the MAVLink stream on the NRF51 serial port right before it is parsed by the MAVLink parser. If the message that was received is a battery status message, we update the internal battery state.
- In your development environment, navigate to the GCS_Common file:
```
path\...\libraries\GCS_MAVLink\GCS_Common.cpp
```
- Find the update_receive() function:
```
void GCS_MAVLINK::update_receive(uint32_t max_time_us)
...
```
- Replace the contents of this function with the following:

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

                        // gcs().send_text(MAV_SEVERITY_DEBUG, "VBAT: %.2f V, %.2f C", (double)script_state.voltage, (double)script_state.temperature);    // DEBUG  
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
```

This function now decodes battery status messages sent by the NRF51 and passes the data to the scripting battery monitor struct. The battery monitor backend will use this struct to update the internal battery state periodically.

## Compiling & Flashing to the Crazyflie
Before using the new battery monitor driver in ArduPilot, we need to compile the custom firmware and flash it the Crazyflie.

For detailed flashing instructions, please reference the [Compiling & Flashing Guide](compiling_and_flashing.md).

If you attempt to compile your firmware and get a build failed error:
```
Build failed -> task in 'bin/arducopter' failed (exit status 1)
```
Chances are you have exceeded the memory limit. Please reference the [Freeing up Memory Guide](freeing_up_memory.md) for detailed instructions on how to reduce the build size.

## Testing and Using Optical Flow
Once you have successfully flashed both the NRF51 and the STM32 with the custom firmware, we are ready to enable the battery monitor in ArduPilot.

Start by changing the following parameters upon startup of your drone:

```
SERIAL1_PROTOCOL 2
SERIAL1_BAUD     115
SERIAL2_PROTOCOL 2
SERIAL2_BAUD     1000
SERIAL3_PROTOCOL 2
BATT_MONITOR     29
```

Changing the serial protocol and baudrate of port 2 to MAVLink and 1M respectively allows the STM32 to communicate with the NRF51. Changing the battery monitor parameter to 29 enables the scripting driver.

After changing the parameter values and saving them to memory, restart the system by either cutting power to the drone directly or sending a reboot command through MAVLink.

The battery monitor should now initialize properly upon powering the system. In your Ground Control Station of choice (ie. QGroundControl, MavProxy, etc.), verify the sensor is working properly by monitoring the live feed of the battery state:

![Battery Monitor](images/battery_monitor/battery-monitor.png)

Note that the current implementation has a bug which causes the voltage to report incorrectly in the GCS as shown in the above photo. For now, the correct voltage can be seen in the drone's console feed through a recurring print statement. Future iterations will fix this.