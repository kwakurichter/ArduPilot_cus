# Flashing the NRF Guide
## What does the NRF do?
The Crazyflie 2.1 pairs its STM32 flight controller MCU with a Nordic nRF51822 (NRF51) as a dedicated “board controller”. The nRF51 handles all 2.4 GHz radio transport (CRTP), framing and forwarding packets via Syslink to the STM32, so the flight controller stays focused on control loops.

It also manages the power system (battery measurement, regulator enable/disable, and safe power-up/down sequencing) and provides the over-the-air (and USB-bootloader) flashing interface for both itself and the STM32.

By offloading radio, power management, and bootloading tasks to this low-power SoC, the Crazyflie achieves reliable telemetry and firmware updates without burdening the main flight processor.

## Why Flash the NRF?
The main reason for flashing the NRF is to add ArduPilot communication functionality. In essence, we want to modify the NRF firmware slightly to allow the NRF to act as a telemetry forwarding radio without touching the critical underlying functionality discussed previously. 

## Setting up the Development Environment
To modify the legacy Bitcraze NRF firmware, start by setting up a local development environment on your PC. Follow the guide in the link below for instructions:

https://www.bitcraze.io/documentation/repository/crazyflie2-nrf-firmware/master/development/starting_development/

## Modify the NRF Firmware
***Modify the Syslink Implementation***

We start by modifying the Syslink protocol implementation. 
- Navigate to the syslink file:
```
path\...\crazyflie2-nrf-firmware\src\syslink.c
```
- Find the existing send function:
```
bool syslinkSend(struct syslinkPacket *packet)
...
```
- Under this, add the following new send function:
```
// For ArduPilot Implementation (uses the buffered UART send)
bool syslinkSend_buffered(struct syslinkPacket *packet)
{
  // This is part of the original logic to wait for the STM32 to talk first.
  if (!isSyslinkActive)
  {
    return false;
  }

  // 1. Create a temporary buffer to hold the entire packet frame.
  //    Size = 2 (START) + 1 (TYPE) + 1 (LEN) + max_data + 2 (CKSUM)
  uint8_t frame_buffer[SYSLINK_MTU + 6];
  uint16_t frame_len = 0;
  
  // 2. Initialize checksum variables
  uint8_t cksum_a=0;
  uint8_t cksum_b=0;
  
  // 3. Assemble the packet header
  frame_buffer[frame_len++] = START_BYTE1;
  frame_buffer[frame_len++] = START_BYTE2;

  frame_buffer[frame_len++] = packet->type;
  cksum_a += packet->type;
  cksum_b += cksum_a;

  frame_buffer[frame_len++] = packet->length;
  cksum_a += packet->length;
  cksum_b += cksum_a;

  // 4. Copy the data payload and update checksum at the same time
  for (int i = 0; i < packet->length; i++)
  {
    frame_buffer[frame_len++] = packet->data[i];
    cksum_a += packet->data[i];
    cksum_b += cksum_a;
  }

  // 5. Add the calculated checksum to the end of the frame
  frame_buffer[frame_len++] = cksum_a;
  frame_buffer[frame_len++] = cksum_b;

  // 6. Send the entire assembled frame in one non-blocking call
  uart_buffered_send(frame_buffer, frame_len);

  return true;
}
```
This new send function takes packets destined for the STM and wraps them in Syslink protocol.

Next, we need to update the corresponding header file:
- Navigate to:
```
path\...\crazyflie2-nrf-firmware\src\syslink.h
```
- Under the legacy send function definition, add:
```
/**
 * Send syslink packet.
 * Will only send if link is first activated by a packet beeing received.
 *
 * @param packet  Syslink packet containing data to send.
 */
bool syslinkSend(struct syslinkPacket *packet);

// -- ADD THIS --
/**
 * Send syslink packet that uses the new buffered UART send.
 * For ArduPilot Implementation.
 *
 * @param packet  Syslink packet containing data to send.
 */
bool syslinkSend_buffered(struct syslinkPacket *packet);
// -- ADD THIS --
```
- We also need to add a new Syslink packet type:
```
#define SYSLINK_RADIO_P2P           0x08
#define SYSLINK_RADIO_P2P_ACK       0x09
#define SYSLINK_RADIO_P2P_BROADCAST 0x0A
#define SYSLINK_RADIO_MAVLINK       0x0B <-- ADD THIS
```

***Modify the Serial Implementation***

- Navigate to the uart file:
```
path\...\crazyflie2-nrf-firmware\src\uart.c
```
- At the beginning of the file, add the following:
```
#define Q_LENGTH 128

// ADD THIS
#define UART_TX_BUFFER_SIZE 256

static volatile uint8_t uart_tx_buffer[UART_TX_BUFFER_SIZE];
static volatile uint16_t uart_tx_head = 0;
static volatile uint16_t uart_tx_tail = 0;
// -- ADD THIS --
```
- Next, find the existing handler function:
```
void UART0_IRQHandler()
...
```
- Replace this function with the updated function below:
```
void UART0_IRQHandler()
{
  // --- Existing RX handling code ---
  if (NRF_UART0->EVENTS_RXDRDY) {
    int nhead = head+1;

    if (NRF_UART0->ERRORSRC) {
      uartError = NRF_UART0->ERRORSRC;
      NRF_UART0->ERRORSRC = 0xFF;

      uartErrorCount++;
    }

    NRF_UART0->EVENTS_RXDRDY = 0;

    // Check if the queue is not full
    if (nhead >= Q_LENGTH) nhead = 0;
    if (nhead == tail) {
      dummy = NRF_UART0->RXD; //Read anyway to avoid hw overflow
      dropped++;
      return;
    }

    // Push data in queue
    rxq[head++] = NRF_UART0->RXD;
    if (head >= Q_LENGTH) head = 0;
  }
  // --- New TX handling code ---
  if (NRF_UART0->EVENTS_TXDRDY) {
    NRF_UART0->EVENTS_TXDRDY = 0;

    if (uart_tx_head != uart_tx_tail) {
      // If there is data in our new buffer, send the next byte
      NRF_UART0->TXD = uart_tx_buffer[uart_tx_tail];
      uart_tx_tail = (uart_tx_tail + 1) % UART_TX_BUFFER_SIZE;
    } else {
      // Buffer is empty, disable the TX interrupt until more data is added
      NRF_UART0->INTENCLR = UART_INTENCLR_TXDRDY_Msk;
    }
  }
}
```
This function defines a new buffer where data received from the STM over serial is stored.

Next, we need to define a new send function.
- Under the existing send function:
```
void uartSend(char* data, int len)
...
```
- Add the following:
```
// For Ardupilot Implementation (Non-Blocking send)
void uart_buffered_send(const uint8_t *data, uint16_t length) {
  // 1) Snapshot head/tail & compute free space
  uint16_t head = uart_tx_head;
  uint16_t tail = uart_tx_tail;
  uint16_t free_space;

  if (head >= tail) {
      free_space = UART_TX_BUFFER_SIZE - (head - tail) - 1;
  } else {
      free_space = (tail - head) - 1;
  }

  // Drop whole packet if it won't fit
  if (length > free_space) {
      // uart_packets_dropped++;
      return;
  }

  // Remember if we were idle (so we know to prime the pump)
  bool was_idle = (head == tail);

  // 2) Disable all IRQs to protect head/tail
  __disable_irq();

  // 3) Queue the data
  for (uint16_t i = 0; i < length; i++) {
      uart_tx_buffer[head] = data[i];
      head = (head + 1) % UART_TX_BUFFER_SIZE;
  }
  uart_tx_head = head;

  // 4) If buffer was empty before, kick off the first byte immediately
  if (was_idle) {
      // Clear any stale TXDRDY event
      NRF_UART0->EVENTS_TXDRDY = 0;
      // Send first byte
      NRF_UART0->TXD = uart_tx_buffer[uart_tx_tail];
      // Advance tail
      uart_tx_tail = (uart_tx_tail + 1) % UART_TX_BUFFER_SIZE;
  }

  // 5) Enable the TXDRDY interrupt so the ISR will send the rest
  NRF_UART0->INTENSET = UART_INTENSET_TXDRDY_Msk;

  // 6) Re-enable IRQs
  __enable_irq();
}
```
This new send fucntion disables all interupts while handling packets to avoid corruption. It pushes data being held in the buffer we defined earlier to the STM. Also, the function checks if the system was previously idle.

Next, we need to update the uart header file to match the changes we made to the implementation.
- Navigate to the header:
```
path\...\crazyflie2-nrf-firmware\src\uart.h
```
- Under the legacy send definition, add the following:
```
void uartSend(char* data, int len);

void uart_buffered_send(const uint8_t *data, uint16_t length); <-- ADD THIS
```


***Modify the main loop***

- Navigate to the main loop file:
```
path\...\crazyflie2-nrf-firmware\src\main.c
```
Under the defines at the top, add the following:
```
#ifdef BLE
int volatile bleEnabled = 1;
#else
int volatile bleEnabled = 0;
#endif

// -- ADD THIS --
typedef struct {
  uint8_t size;
  uint8_t data[63];
  uint8_t rssi;
  bool broadcast;
} SafeRxPacket;
// -- ADD THIS --

static SafeRxPacket safePacket; <-- ADD THIS
```
This new structure will allow us to store packets received by the radio in seperate flash to avoid race conditions.

We now need to change any functions which used the legacy structure "esbPacket" with this new structure.
- Find the below functions and replace their arguments:
```
static void handleRadioCmd(struct esbPacket_s * packet); <-- REPLACE THIS

static void handleRadioCmd(const SafeRxPacket* packet); <-- WITH THIS
```

```
static void handleBootloaderCmd(struct esbPacket_s *packet); <-- REPLACE THIS

static void handleBootloaderCmd(const SafeRxPacket* packet); <-- WITH THIS
```

- Add the following flag:
```
static bool debugProbeReceivedRate = false;
static bool bleIsDisabled = false;  <-- ADD THIS
```

We need to update the mainloop() next.
- Find the following if statement in the mainloop function:
```
void mainloop()
...
...
if ((esbReceived == false) && esbIsRxPacket())
    {
...
...
    }
    handleSyslinkEvents(syslinkReceive(&slRxPacket));
    sendDataToStmOverSyslink();
```
- Replace it with the following:
```
if (esbIsRxPacket())
{
  // A packet is available in the radio's receive queue.
  // We must immediately copy it to our safe, local buffer before the interrupt handler or radio DMA can overwrite it with an ACK packet.
  // To prevent the RADIO_IRQHandler from corrupting our data during the copy, we disable all interrupts, perform the copy, and then immediately re-enable them. This makes the copy atomic.
  
  __disable_irq(); // Disable all interrupts
  
  // Step 1: Get the packet from the radio's receive queue.
  EsbPacket* packet = esbGetRxPacket();

  // 2. Perform a copy into our safe buffer.
  safePacket.size = packet->size;
  if (safePacket.size > 0) {
    memcpy(safePacket.data, packet->data, safePacket.size);
  }
  safePacket.rssi = packet->rssi;
  safePacket.broadcast = (packet->match == ESB_MULTICAST_ADDRESS_MATCH);  // The received packet was a broadcast, if received on local address 1

  // 3. Immediately release the radio's buffer. All subsequent logic will use the 'safePacket' object, not the volatile 'packet'.
  esbReleaseRxPacket();

  __enable_irq(); // Re-enable all interrupts

  // Only disable BLE once when the first ESB packet is received.
  if (!bleIsDisabled) {
    disableBle();
    bleIsDisabled = true;
  }

  // -- DEBUG --
  //uint8_t dbg[3];
  //dbg[0] = safePacket.size;        // how many bytes the nRF saw
  //dbg[1] = safePacket.data[0];     // first byte
  //dbg[2] = safePacket.data[1];     // second byte
  // Queue a debug packet on DEBUG_PORT (0x0E), channel 0
  //esbSendDebugPacket(DEBUG_PORT, 0, (char*)dbg, sizeof(dbg));
  // -- DEBUG --

  //Store RSSI here so that we can send it to STM later
  // Todo investigate if we can not just simply link this to the packet itself or find a way to separate this due to P2P
  rssi = safePacket.rssi;

  // Now, inspect the packet and decide what to do with it.
  // Is it a special radio command packet?
  if ((safePacket.size >= 4) && (safePacket.data[0]&0xf3) == 0xf3 && (safePacket.data[1]==0x03))
  {
    // This logic needs to be updated to pass the safePacket
    handleRadioCmd(&safePacket);
  }
  // Is it a special bootloader command packet?
  else if ((safePacket.size > 2) && (safePacket.data[0]&0xf3) == 0xf3 && (safePacket.data[1]==0xfe))
  {
    // This logic needs to be updated to pass the safePacket
    handleBootloaderCmd(&safePacket);
  }
  // Is it a peer-to-peer (P2P) packet?
  else if (safePacket.size >= 2 && (safePacket.data[0] & 0xf3) == 0xf3 && (safePacket.data[1] & 0xF0) == 0x80)
  {
    // Handle P2P logic (forward to STM with SYSLINK_RADIO_P2P type)

    slTxPacket.data[0] = safePacket.data[1] & 0x0F;  // The first byte sent is the P2P port
    slTxPacket.data[1] = safePacket.rssi; // Save RSSI between drones in packet
    memcpy(&slTxPacket.data[2], &safePacket.data[2], safePacket.size - 2);
    slTxPacket.length = safePacket.size;
    if (safePacket.broadcast) {
      slTxPacket.type = SYSLINK_RADIO_P2P_BROADCAST;
    } else {
      slTxPacket.type = SYSLINK_RADIO_P2P;
    }

    syslinkSend_buffered(&slTxPacket);
  }
  else if ((safePacket.data[0] & 0xf3) == 0xf3)
  {
    // This is a low-level radio packet (like an ACK or handshake) that is not
    // a command. We should do nothing and discard it, preventing it from
    // being forwarded to the STM.
  }
  else if (safePacket.data[0] == 0xff)
  {
    // This is a low-level radio packet (like an ACK or handshake) that is not
    // a command. We should do nothing and discard it, preventing it from
    // being forwarded to the STM.
  }
  // If it's none of the above, assume it's general data from the GCS.
  else
  {
    // Set the Syslink payload length to the radio packet size
    slTxPacket.length = safePacket.size;

    // Copy the radio packet's data into the Syslink packet
    memcpy(slTxPacket.data, &safePacket.data, slTxPacket.length);

    // Set the Syslink packet type.
    if (safePacket.broadcast) {
      slTxPacket.type = SYSLINK_RADIO_RAW_BROADCAST;
    } else {
      slTxPacket.type = SYSLINK_RADIO_MAVLINK;
      
      // -- DEBUG --
      //uint8_t dbg2[3];
      //dbg2[0] = slTxPacket.length;      // how many bytes the nRF saw
      //dbg2[1] = slTxPacket.data[0];     // first byte
      //dbg2[2] = slTxPacket.data[1];     // second byte
      // Queue a debug packet on DEBUG_PORT_2 (0x09), channel 0
      //esbSendDebugPacket(DEBUG_PORT_2, 0, (char*)dbg2, sizeof(dbg2));
      // -- DEBUG --
    }
    // Use the buffered send for all transmissions
    syslinkSend_buffered(&slTxPacket);
  }
}
```
This new block protects against data corruption from race conditions by disbling interreupts and storing data in the new structure we defined earlier.

We also need to add a new case for MAVLink data sent over radio from the GCS.
- Find the following case:
```
      case SYSLINK_RADIO_POWER:
        ...
        ...
        break;
```
- And add the following MAVLink case below it:
```
      case SYSLINK_RADIO_MAVLINK:
        // --- STM->GCS DEBUG ---
        //{
        //  uint8_t dbg_stm[3];
        //  dbg_stm[0] = slRxPacket.length;
        //  dbg_stm[1] = slRxPacket.data[0];
        //  dbg_stm[2] = slRxPacket.data[1];
          // Use DEBUG_PORT_3 (0x0A) to make this print distinct
        //  esbSendDebugPacket(DEBUG_PORT_3, 0, (char*)dbg_stm, sizeof(dbg_stm));
        //}
        // --- END OF DEBUG ---

        // The slRxPacket contains a MAVLink message from the STM.
        // Pass it to our new helper function which will wrap it in a CRTP header and queue it for radio transmission.
        esbSendSyslinkMavlinkPacket(&slRxPacket);
        
        break;
```

Additionally, we need to replace all of the calls of the legacy syslink send function to the new one we defined earlier:
- Example:

```
      case SYSLINK_RADIO_CONTWAVE:
        if(slRxPacket.length == 1) {
          esbSetContwave(slRxPacket.data[0]);

          slTxPacket.type = SYSLINK_RADIO_CONTWAVE;
          slTxPacket.data[0] = slRxPacket.data[0];
          slTxPacket.length = 1;
          syslinkSend(&slTxPacket); <-- REPLACE THIS
        }

      case SYSLINK_RADIO_CONTWAVE:
        if(slRxPacket.length == 1) {
          esbSetContwave(slRxPacket.data[0]);

          slTxPacket.type = SYSLINK_RADIO_CONTWAVE;
          slTxPacket.data[0] = slRxPacket.data[0];
          slTxPacket.length = 1;
          syslinkSend_buffered(&slTxPacket); <-- WITH THIS
        }
```

Ensure all of the legacy calls are replaced before proceeding.


***Modifly the Radio Implementation***

- Navigate to the esb file:
```
path\...\crazyflie2-nrf-firmware\src\esb.c
```
- Find the following function:
```
void esbSendP2PPacket(uint8_t port, char *data, uint8_t length)
...
```
- Under it, add a new send function:
```
void esbSendSyslinkMavlinkPacket(const struct syslinkPacket *slPacket)
{
  // Check if there's space in the transmit queue.
  if (!esbCanTxPacket())
  {
    // Not enough space, drop the packet. This could be signaled back to STM
    // in a more advanced implementation, but for now, we just drop it.
    return;
  }

  // Get a pointer to the next available packet buffer in the queue.
  EsbPacket* packet = esbGetTxPacket();
  if (!packet)
  {
    return;
  }

  // Define the MAVLink port and channel for CRTP
  const uint8_t MAVLINK_PORT = 0x0B;
  const uint8_t MAVLINK_CHANNEL = 0;

  // Truncate the payload if it's too large for a CRTP packet
  uint8_t payload_len = slPacket->length;
  if (payload_len > 30) {
    payload_len = 30;
  }

  // 1. Construct the CRTP header byte.
  uint8_t crtp_header = ((MAVLINK_PORT & 0x0F) << 4) | (0x03 << 2) | (MAVLINK_CHANNEL & 0x03);
  packet->data[0] = crtp_header;

  // 2. Copy the Syslink payload (the MAVLink data) after the header.
  memcpy(&packet->data[1], slPacket->data, payload_len);

  // 3. Set the total size of the radio packet (1-byte header + payload).
  packet->size = payload_len + 1;

  // 4. Mark the packet as ready to be sent by advancing the queue head.
  esbSendTxPacket();
}
```
This new send function critically wraps the packets with a CRTP header before sending them to a buffer to be sent over radio to the GCS.

- Next, navigate to the corresponding esb header file to add the new definition:
```
path\...\crazyflie2-nrf-firmware\src\esb.h
```
- Add the following defintion:
```
/* Immediately send a peer 2 peer packet in TX */
void esbSendP2PPacket(uint8_t port, char *data, uint8_t length);

/* Queues a Syslink payload as a CRTP MAVLink packet for sending to the GCS */
void esbSendSyslinkMavlinkPacket(const struct syslinkPacket *slPacket); <-- ADD THIS
```

## Compiling and Flashing
Once you have setup your build environment and have customized the firmware, we can move on to compiling and flashing the firmware.

Note that the Crazyflie must be flashed with the Bitcraze firmware for this to work. If you need to restore your Crazyflie to the factory firmware, please reference the [Restoring the Crazyflie Guide](restoring_the_crazyflie.md) before proceeding.

### Prepare for Flashing
- Start by inserting your CrazyRadio PA dongle into your PC.
- Put your Crazyflie into bootloader mode:
    - Disconnect all power sources (remove battery, unplug usb cable).
    - Press and hold the power button. While holding the power button:
        - Plug the usb cable into the drone.
        - The M2 LED should begin blinking slowly.
        - Release the power button.
          <video width="600" controls>
            <source src="images/quick_start_guide/bootloader-first.mp4" type="video/mp4">
          </video>

### Compile and Flash
Once you are ready to flash:
- Open a terminal.
- If you set up your build environment in a virtual environment, ensure your Python interpreter has access to the required prerequisites.
- Navigate to your custom NRF repository:
```
path\...\crazyflie2-nrf-firmware
```
- It is good practice to clean your build objects at this time (in case some submodules have been updated):
```
make clean
```
- Next, compile and flash the NRF over-the-air:
```
make cload
```

![Flashing the NRF](images/flashing_the_nrf/flashing.png)

- After the firmware is finished flashing, the Crazyflie should automatically reboot.

Your NRF is now flashed with your custom firmware. Note that flashing the NRF with this method does not affect the firmware that is present on the main STM32 microcontroller.

If you need to restore the NRF to the factory Bitcraze firmware for any reason, repeat the above instructions with the Bitcraze NRF repository.