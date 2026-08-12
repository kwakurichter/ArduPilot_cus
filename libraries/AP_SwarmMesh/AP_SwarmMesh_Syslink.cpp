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

#include "AP_SwarmMesh_Syslink.h"

#if AP_SWARMMESH_SYSLINK_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <AP_Syslink/AP_Syslink.h>
#include <AP_Logger/AP_Logger.h>
#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL& hal;

/*
  Room for several maximum size broadcasts. The frontend drains at its update
  rate while the radio delivers on the syslink thread, so this only has to
  cover the gap between drains, not a whole stream.
 */
#define AP_SWARMMESH_SYSLINK_RX_BUF (4 * 256)

AP_SwarmMesh_Syslink::AP_SwarmMesh_Syslink(AP_SwarmMesh &frontend) :
    AP_SwarmMesh_Backend(frontend)
{
    _syslink = AP::syslink();
    if (_syslink == nullptr) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "SwarmMesh: syslink not available");
        return;
    }

    _rx_buf = NEW_NOTHROW ByteBuffer(AP_SWARMMESH_SYSLINK_RX_BUF);
    if (_rx_buf == nullptr) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "SwarmMesh: no memory for rx buffer");
        return;
    }

    /*
      No identity check here any more: AP_Syslink::get_address() returns
      MAV_SYSID, which is the same value SwarmMesh routes by, so the two
      cannot disagree.
     */
    _syslink->set_broadcast_handler(
        FUNCTOR_BIND_MEMBER(&AP_SwarmMesh_Syslink::handle_broadcast, void, const uint8_t *, uint8_t));

    _ready = true;
}

/*
  Inbound broadcast, on the syslink thread. The payload is whatever the peer
  passed to send_broadcast(); syslink framing and checksum are already stripped
  and verified. Copy it and return: this must not block.
 */
void AP_SwarmMesh_Syslink::handle_broadcast(const uint8_t *data, uint8_t len)
{
    if (len == 0 || _rx_buf == nullptr) {
        return;
    }

    WITH_SEMAPHORE(_sem);

    /*
      Drop the whole datagram rather than a prefix of it. A partial write would
      put a truncated frame into the stream, and the parser would spend the
      next sync pair recovering from something we chose to do to it.
     */
    if (_rx_buf->space() < len) {
        _rx_dropped++;
        return;
    }

    _rx_buf->write(data, len);
    _rx_packets++;
}

bool AP_SwarmMesh_Syslink::transport_ready() const
{
    // the radio still has to finish its channel/address handshake
    return _ready && _syslink != nullptr && _syslink->configured();
}

uint32_t AP_SwarmMesh_Syslink::transport_available()
{
    if (_rx_buf == nullptr) {
        return 0;
    }
    WITH_SEMAPHORE(_sem);
    return _rx_buf->available();
}

int16_t AP_SwarmMesh_Syslink::transport_read()
{
    if (_rx_buf == nullptr) {
        return -1;
    }
    WITH_SEMAPHORE(_sem);
    uint8_t b;
    if (!_rx_buf->read_byte(&b)) {
        return -1;
    }
    return b;
}

/*
  One broadcast is the transmission unit, so anything up to that limit can be
  taken. send_broadcast() refuses if the radio queue is actually full, which
  transport_write() records.
 */
uint32_t AP_SwarmMesh_Syslink::transport_txspace()
{
    if (!transport_ready()) {
        return 0;
    }
    return AP_Syslink::broadcast_max_len();
}

void AP_SwarmMesh_Syslink::transport_write(const uint8_t *buf, uint16_t len)
{
    if (!transport_ready() || len == 0) {
        return;
    }

    /*
      One SwarmMesh packet per broadcast, never split. A worst case packet is
      SWARMMESH_MSG_BUF_MAX, which is larger than a broadcast can carry, but
      the MAVLink messages actually streamed are far below it. Fragmenting
      would mean reassembly and a sequence space on a lossy link, for traffic
      that should not be generating packets this large in the first place.
     */
    if (len > AP_Syslink::broadcast_max_len()) {
        _tx_oversize++;
        return;
    }

    if (!_syslink->send_broadcast(buf, (uint8_t)len)) {
        _tx_refused++;
        return;
    }
    _tx_packets++;
}

void AP_SwarmMesh_Syslink::log_stats()
{
    AP_SwarmMesh_Backend::log_stats();

#if HAL_LOGGING_ENABLED
    // @LoggerMessage: SWSL
    // @Description: SwarmMesh syslink transport counters
    // @Field: TimeUS: Time since system startup
    // @Field: RxP: broadcasts received
    // @Field: RxD: broadcasts dropped, receive buffer full
    // @Field: TxP: broadcasts sent
    // @Field: TxO: packets refused, larger than one broadcast
    // @Field: TxR: packets refused by the radio
    AP::logger().WriteStreaming("SWSL", "TimeUS,RxP,RxD,TxP,TxO,TxR", "QIIIII",
                                AP_HAL::micros64(),
                                _rx_packets, _rx_dropped,
                                _tx_packets, _tx_oversize, _tx_refused);
#endif
}

#endif  // AP_SWARMMESH_SYSLINK_ENABLED
