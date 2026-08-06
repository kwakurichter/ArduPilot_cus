/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "AP_Syslink_MAVLinkPort.h"

#if AP_SYSLINK_ENABLED && AP_SERIALMANAGER_REGISTER_ENABLED

#include "AP_Syslink.h"

#include <AP_HAL/utility/packetise.h>
#include <AP_Math/AP_Math.h>

using namespace AP_Syslink_Protocol;

/*
  A MAVLink v2 frame reaches 267 bytes unsigned and 280 signed, so the write
  buffer must hold well over one frame for packetise() to ever see a complete
  one. The read side only needs to keep up with the GCS parser.
 */
#define AP_SYSLINK_PORT_TX_SIZE 2048
#define AP_SYSLINK_PORT_RX_SIZE 1024

// How long to wait for a free-slot report before releasing one slot anyway.
#define AP_SYSLINK_SPACE_TIMEOUT_MS 500

bool AP_Syslink_MAVLinkPort::init(AP_Syslink &syslink)
{
    _syslink = &syslink;

    if (!init_buffers(AP_SYSLINK_PORT_RX_SIZE, AP_SYSLINK_PORT_TX_SIZE)) {
        return false;
    }

    /*
      Advertise MAVLink2 so the GCS binds a channel to this port. The serial
      manager searches hardware ports before registered ones, so with SERIAL0
      and SERIAL1 on MAVLink this becomes MAVLINK_COMM_2
     */
    state.idx = HAL_SYSLINK_SERIAL_IDX;
    state.protocol.set(AP_SerialManager::SerialProtocol_MAVLink2);

    // informational: the GCS calls begin() with this, which this port ignores
    state.baud.set(1000);

    AP::serialmanager().register_port(this);
    return true;
}

bool AP_Syslink_MAVLinkPort::init_buffers(uint32_t rx_size, uint32_t tx_size)
{
    WITH_SEMAPHORE(_sem);

    if (_readbuf == nullptr) {
        _readbuf = NEW_NOTHROW ByteBuffer(rx_size);
    }
    if (_writebuf == nullptr) {
        _writebuf = NEW_NOTHROW ByteBuffer(tx_size);
    }
    return _readbuf != nullptr && _writebuf != nullptr;
}

void AP_Syslink_MAVLinkPort::_begin(uint32_t b, uint16_t rxS, uint16_t txS)
{
    // the GCS opens this port with begin(115200) and no sizes; the baud rate is
    // meaningless here and the buffers are already the size we want
    (void)b;
    init_buffers(MAX(uint32_t(rxS), uint32_t(AP_SYSLINK_PORT_RX_SIZE)),
                 MAX(uint32_t(txS), uint32_t(AP_SYSLINK_PORT_TX_SIZE)));
}

size_t AP_Syslink_MAVLinkPort::_write(const uint8_t *buffer, size_t size)
{
    WITH_SEMAPHORE(_sem);
    if (_writebuf == nullptr) {
        return 0;
    }
    return _writebuf->write(buffer, size);
}

ssize_t AP_Syslink_MAVLinkPort::_read(uint8_t *buffer, uint16_t count)
{
    WITH_SEMAPHORE(_sem);
    if (_readbuf == nullptr) {
        return -1;
    }
    return _readbuf->read(buffer, count);
}

uint32_t AP_Syslink_MAVLinkPort::_available()
{
    WITH_SEMAPHORE(_sem);
    if (_readbuf == nullptr) {
        return 0;
    }
    return _readbuf->available();
}

bool AP_Syslink_MAVLinkPort::_discard_input()
{
    WITH_SEMAPHORE(_sem);
    if (_readbuf == nullptr) {
        return false;
    }
    _readbuf->clear();
    return true;
}

/*
  Not the UART rate. Downlink chunks only leave in ack payloads when the ground
  station polls, so the achievable rate is set by the radio and is far below
  the 1 Mbaud serial line. GCS_Param and GCS_FTP pace parameter download and
  burst reads from this; log download does not use it at all.
 */
uint32_t AP_Syslink_MAVLinkPort::bw_in_bytes_per_second() const
{
    if (_syslink == nullptr) {
        return 1000;
    }
    return _syslink->link_bw();
}

uint32_t AP_Syslink_MAVLinkPort::txspace()
{
    WITH_SEMAPHORE(_sem);
    if (_writebuf == nullptr) {
        return 0;
    }

    // Local buffer space only. This must NOT be bounded by the radio queue.
    return _writebuf->space();
}

void AP_Syslink_MAVLinkPort::handle_chunk(uint8_t type, const uint8_t *data, uint8_t len)
{
    (void)type;
    if (len == 0) {
        return;
    }
    WITH_SEMAPHORE(_sem);
    if (_readbuf == nullptr) {
        return;
    }
    // opaque stream bytes; a dropped chunk costs one frame and the parser resyncs
    _readbuf->write(data, len);
}

/*
  Slots the radio can still take, counted as depth less what we have handed
  over and not yet seen transmitted.
 */
uint8_t AP_Syslink_MAVLinkPort::free_slots() const
{
    if (_outstanding >= MAVLINK_TX_SLOTS) {
        return 0;
    }
    return MAVLINK_TX_SLOTS - _outstanding;
}

void AP_Syslink_MAVLinkPort::handle_space(uint8_t type, const uint8_t *data, uint8_t len)
{
    (void)type;
    if (len < 1) {
        return;
    }
    const uint8_t free = MIN(data[0], MAVLINK_TX_SLOTS);

    /*
      Return credit only for slots the radio has demonstrably freed since the
      last report. The absolute value cannot be trusted as a credit because it
      predates anything still in flight.
     */
    if (free > _last_reported_free) {
        _outstanding -= MIN(_outstanding, uint8_t(free - _last_reported_free));
    }
    _last_reported_free = free;
    _last_report_ms = AP_HAL::millis();
    _have_space_report = true;
}

void AP_Syslink_MAVLinkPort::update()
{
    if (_syslink == nullptr || _writebuf == nullptr) {
        return;
    }

    /*
      Credit is only returned by a report, so a lost one would stall the link
      for good. Release a single slot if none has arrived for a while: if the
      queue really is full the chunk is discarded and we are no worse off, and
      if a report went missing the link recovers.
     */
    if (_outstanding > 0 && _have_space_report &&
        AP_HAL::millis() - _last_report_ms > AP_SYSLINK_SPACE_TIMEOUT_MS) {
        _outstanding--;
        _last_report_ms = AP_HAL::millis();
    }

    while (true) {
        // stop once the radio queue is full; further chunks would be discarded
        if (free_slots() == 0) {
            break;
        }

        uint32_t n;
        {
            WITH_SEMAPHORE(_sem);
            const uint32_t avail = _writebuf->available();
            if (avail == 0) {
                break;
            }

            if (_frame_remaining == 0) {
                /*
                  packetise() must see the whole buffer, not a view capped to
                  one chunk: it returns 0 unless the buffer holds a complete
                  frame, so a capped length would make anything longer than one
                  chunk look permanently incomplete.
                 */
                _frame_remaining = mavlink_packetise(*_writebuf, avail);
                if (_frame_remaining == 0) {
                    break;      // frame still arriving
                }
            }

            /*
              Send the rest of an oversized frame from the remembered length
              rather than asking packetise() again. Once the buffer starts
              mid-frame its first byte is payload, and roughly one time in 128
              that byte is 0xFD or 0xFE - which packetise() reads as a frame
              header, judges incomplete, and answers 0 to, stranding the tail
              until unrelated later traffic happens to satisfy the bogus
              length. FILE_TRANSFER_PROTOCOL frames are oversized, so this
              would show up as intermittent parameter and log download stalls.
             */
            n = MIN(_frame_remaining, uint32_t(MAVLINK_CHUNK_MAX));
            n = MIN(n, avail);
        }

        uint8_t buf[MAVLINK_CHUNK_MAX];
        uint32_t got;
        {
            WITH_SEMAPHORE(_sem);
            got = _writebuf->peekbytes(buf, n);
        }
        if (got == 0) {
            break;
        }

        if (!_syslink->send_packet(Type::RADIO_MAVLINK, buf, got)) {
            // syslink transmit buffer full; leave it queued and retry
            break;
        }

        {
            WITH_SEMAPHORE(_sem);
            _writebuf->advance(got);
        }
        _frame_remaining -= MIN(_frame_remaining, got);

        if (_outstanding < UINT8_MAX) {
            _outstanding++;
        }
    }
}

#endif // AP_SYSLINK_ENABLED && AP_SERIALMANAGER_REGISTER_ENABLED
