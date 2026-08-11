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
#pragma once

#include "AP_SwarmMesh_Backend.h"

#if AP_SWARMMESH_SYSLINK_ENABLED

#include <AP_HAL/utility/RingBuffer.h>

class AP_Syslink;

/*
  SwarmMesh over the Crazyflie's nRF51822 radio.

  Modelled on AP_SwarmMesh_SITL rather than AP_SwarmMesh_Serial: syslink peer
  broadcast is a datagram transport, like UDP multicast, not a byte stream. The
  backend interface is byte oriented because the base class does its own
  SYNC/CRC framing, so received datagrams are buffered and drained a byte at a
  time for it to re-frame. Losing a packet in the air leaves a truncated frame
  in the stream, which the parser discards on the next sync pair.

  Unlike the SITL backend this cannot hold a single datagram at a time.
  Broadcasts arrive on the syslink thread via a handler that is documented as
  not being allowed to block, whereas the parser is drained from the vehicle
  thread. A ring buffer decouples the two; a single packet slot would drop any
  broadcast that landed before the frontend got round to draining the last one.
 */
class AP_SwarmMesh_Syslink : public AP_SwarmMesh_Backend
{
public:
    AP_SwarmMesh_Syslink(AP_SwarmMesh &frontend);

    void log_stats() override;

protected:
    bool     transport_ready() const override;
    uint32_t transport_available() override;
    int16_t  transport_read() override;
    uint32_t transport_txspace() override;
    void     transport_write(const uint8_t *buf, uint16_t len) override;

private:
    // inbound broadcast sink. Runs on the syslink thread, so it only copies.
    void handle_broadcast(const uint8_t *data, uint8_t len);

    AP_Syslink *_syslink;
    ByteBuffer *_rx_buf;
    HAL_Semaphore _sem;

    bool _ready;

    // counters, reported by log_stats()
    uint32_t _rx_packets;
    uint32_t _rx_dropped;      // broadcast arrived with no room in _rx_buf
    uint32_t _tx_packets;
    uint32_t _tx_oversize;     // packet longer than one broadcast can carry
    uint32_t _tx_refused;      // radio would not take it
};

#endif  // AP_SWARMMESH_SYSLINK_ENABLED
