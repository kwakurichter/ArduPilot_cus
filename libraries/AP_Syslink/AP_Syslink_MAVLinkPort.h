#pragma once

#include "AP_Syslink_config.h"

#if AP_SYSLINK_ENABLED

#include <AP_SerialManager/AP_SerialManager.h>

#if AP_SERIALMANAGER_REGISTER_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/utility/RingBuffer.h>

#include "AP_Syslink_Protocol.h"

class AP_Syslink;

/*
  Presents the nRF51 radio link to the GCS as an ordinary serial port.

  ArduPilot's MAVLink stack talks only to AP_HAL::UARTDriver, so the radio is
  exposed as a registered port rather than by teaching the GCS about syslink.
  Writes accumulate in a buffer, are cut on MAVLink frame boundaries by
  mavlink_packetise(), capped to one radio packet, and handed to AP_Syslink as
  RADIO_MAVLINK chunks. Received chunks are opaque bytes appended to the read
  buffer, where the GCS parser resyncs on STX after any loss.

  See AP_Syslink.md.
 */
class AP_Syslink_MAVLinkPort : public AP_SerialManager::RegisteredPort
{
public:
    /*
      Allocate buffers and register with the serial manager. Must run before
      GCS::setup_uarts(), which is when the GCS binds its channels.
     */
    bool init(AP_Syslink &syslink);

    // Move queued MAVLink bytes out as chunks. Runs on the syslink thread.
    void update();

    // Handler for RADIO_MAVLINK: opaque inbound stream bytes.
    void handle_chunk(uint8_t type, const uint8_t *data, uint8_t len);

    // Handler for RADIO_MAVLINK_SPACE: free unicast transmit slots.
    void handle_space(uint8_t type, const uint8_t *data, uint8_t len);

    // free radio transmit slots as last reported by the nRF51
    uint8_t get_free_slots() const { return _free_slots; }

    bool is_initialized() override { return true; }
    bool tx_pending() override { return false; }

private:
    uint32_t txspace() override;
    void _begin(uint32_t b, uint16_t rxS, uint16_t txS) override;
    size_t _write(const uint8_t *buffer, size_t size) override;
    ssize_t _read(uint8_t *buffer, uint16_t count) override;
    uint32_t _available() override;
    void _end() override {}
    void _flush() override {}
    bool _discard_input() override;

    uint32_t bw_in_bytes_per_second() const override;

    bool init_buffers(uint32_t rx_size, uint32_t tx_size);

    AP_Syslink *_syslink;
    ByteBuffer *_readbuf;
    ByteBuffer *_writebuf;
    HAL_Semaphore _sem;

    /*
      Free slots in the nRF51's unicast transmit queue: decremented locally on
      each send and refreshed from RADIO_MAVLINK_SPACE. Starts at full depth
      so the link is usable before the first report arrives.
     */
    uint8_t _free_slots = AP_Syslink_Protocol::MAVLINK_TX_SLOTS;
    bool _have_space_report;

    /*
      Bytes of the current frame still to be sent when it did not fit one
      chunk. Tracked here rather than by re-running mavlink_packetise() on the
      remainder, which would misread a payload byte of 0xFD or 0xFE as the
      start of a new frame and stall the tail.
     */
    uint32_t _frame_remaining;
};

#endif // AP_SERIALMANAGER_REGISTER_ENABLED

#endif // AP_SYSLINK_ENABLED
