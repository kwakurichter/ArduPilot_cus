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

#include "AP_Syslink.h"

#if AP_SYSLINK_ENABLED

#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_Logger/AP_Logger.h>
#include <AP_Math/AP_Math.h>
#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL& hal;

using namespace AP_Syslink_Protocol;

// The nRF51's UART is fixed at 1 Mbaud, 8N1.
#define SYSLINK_BAUD 1000000U

#define SYSLINK_TX_BUF_SIZE 1024
#define SYSLINK_UART_RX_SIZE 512
#define SYSLINK_UART_TX_SIZE 512

#define SYSLINK_THREAD_STACK 2048

/*
  How long the nRF51 may hold its RTS line deasserted before we transmit
  anyway. The line is only meant to throttle brief UART FIFO pressure, so a
  sustained deassertion means it is unwired or the nRF51 is wedged - and a
  permanently blocked transmit path would leave the vehicle silently dark.
 */
#define SYSLINK_FLOWCTRL_TIMEOUT_MS 100

AP_Syslink *AP_Syslink::_singleton;

const AP_Param::GroupInfo AP_Syslink::var_info[] = {

    // @Param: ENABLE
    // @DisplayName: Syslink enable
    // @Description: Enable the nRF51822 radio co-processor driver. The driver takes exclusive ownership of the serial port given by SYSL_PORT, which must not be assigned a protocol of its own.
    // @Values: 0:Disabled,1:Enabled
    // @RebootRequired: True
    // @User: Standard
    AP_GROUPINFO_FLAGS("ENABLE", 1, AP_Syslink, _enable, 1, AP_PARAM_FLAG_ENABLE),

    // @Param: PORT
    // @DisplayName: Syslink serial port
    // @Description: Serial port number the nRF51822 is wired to. Set that port's SERIALn_PROTOCOL to -1 so nothing else claims it.
    // @Range: 0 9
    // @RebootRequired: True
    // @User: Advanced
    AP_GROUPINFO("PORT", 2, AP_Syslink, _port_num, 2),

    // @Param: OPTIONS
    // @DisplayName: Syslink options
    // @Description: Bitmask of syslink driver options.
    // @Bitmask: 0:Use UART flow control line,1:Log SYSL statistics
    // @User: Advanced
    AP_GROUPINFO("OPTIONS", 3, AP_Syslink, _options, 3),

    AP_GROUPEND
};

AP_Syslink::AP_Syslink()
{
    AP_Param::setup_object_defaults(this, var_info);

    if (_singleton != nullptr) {
        AP_HAL::panic("AP_Syslink must be singleton");
    }
    _singleton = this;
}

void AP_Syslink::init()
{
    if (_initialised || !enabled()) {
        return;
    }

    _uart = hal.serial(_port_num);
    if (_uart == nullptr) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "Syslink: no serial port %d", int(_port_num));
        return;
    }

    _tx_buf = NEW_NOTHROW ByteBuffer(SYSLINK_TX_BUF_SIZE);
    if (_tx_buf == nullptr) {
        AP_BoardConfig::allocation_error("AP_Syslink tx buffer");
        return;
    }

    _uart->begin(SYSLINK_BAUD, SYSLINK_UART_RX_SIZE, SYSLINK_UART_TX_SIZE);

    // the driver answers its own debug probe requests
    if (!register_handler(Type::DEBUG_PROBE,
                          FUNCTOR_BIND_MEMBER(&AP_Syslink::handle_debug_probe, void, const uint8_t *, uint8_t))) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "Syslink: handler table full");
    }

    if (!hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&AP_Syslink::thread_main, void),
                                      "syslink", SYSLINK_THREAD_STACK,
                                      AP_HAL::Scheduler::PRIORITY_UART, 0)) {
        AP_BoardConfig::allocation_error("AP_Syslink thread");
        return;
    }

    _initialised = true;
}

/*
  Fletcher-8 checksum (RFC 1146) over TYPE, LEN and DATA.
 */
void AP_Syslink::fletcher8(const uint8_t *data, uint16_t len, uint8_t &c0, uint8_t &c1)
{
    c0 = 0;
    c1 = 0;
    for (uint16_t i = 0; i < len; i++) {
        c0 += data[i];
        c1 += c0;
    }
}

bool AP_Syslink::register_handler(Type type, PacketHandler handler)
{
    WITH_SEMAPHORE(_handler_sem);

    for (uint8_t i = 0; i < MAX_HANDLERS; i++) {
        if (_handlers[i].used && _handlers[i].type == uint8_t(type)) {
            // one owner per type
            return false;
        }
    }
    for (uint8_t i = 0; i < MAX_HANDLERS; i++) {
        if (!_handlers[i].used) {
            _handlers[i].handler = handler;
            _handlers[i].type = uint8_t(type);
            _handlers[i].used = true;
            return true;
        }
    }
    return false;
}

bool AP_Syslink::send_packet(Type type, const uint8_t *data, uint8_t len)
{
    if (_tx_buf == nullptr) {
        return false;
    }
    if (len > 0 && data == nullptr) {
        return false;
    }

    const uint16_t frame_len = FRAME_OVERHEAD + len;

    // Checksum covers TYPE, LEN and DATA.
    uint8_t c0 = uint8_t(type);
    uint8_t c1 = uint8_t(type);
    c0 += len;
    c1 += c0;
    for (uint8_t i = 0; i < len; i++) {
        c0 += data[i];
        c1 += c0;
    }

    const uint8_t header[4] = { SYNC0, SYNC1, uint8_t(type), len };
    const uint8_t cksum[2] = { c0, c1 };

    WITH_SEMAPHORE(_tx_sem);

    /*
      Reject the whole frame rather than writing part of it - a truncated
      frame would desynchronise the nRF51's parser until the next sync pair.
     */
    if (_tx_buf->space() < frame_len) {
        _stats.tx_dropped++;
        return false;
    }

    _tx_buf->write(header, sizeof(header));
    if (len > 0) {
        _tx_buf->write(data, len);
    }
    _tx_buf->write(cksum, sizeof(cksum));

    _stats.tx_packets++;
    return true;
}

void AP_Syslink::thread_main()
{
    while (true) {
        hal.scheduler->delay_microseconds(200);

        receive_bytes();
        send_pending();
        update_stats_1hz();
    }
}

void AP_Syslink::receive_bytes()
{
    uint32_t avail = _uart->available();
    if (avail == 0) {
        return;
    }

    // bound the work done in one pass so transmit is not starved
    avail = MIN(avail, 512U);

    uint8_t buf[64];
    while (avail > 0) {
        const uint32_t chunk = MIN(avail, uint32_t(sizeof(buf)));
        const ssize_t n = _uart->read(buf, chunk);
        if (n <= 0) {
            break;
        }
        for (ssize_t i = 0; i < n; i++) {
            parse_byte(buf[i]);
        }
        avail -= n;
    }
}

void AP_Syslink::parse_byte(uint8_t b)
{
    switch (_rx_state) {
    case RxState::SYNC0:
        if (b == SYNC0) {
            _rx_state = RxState::SYNC1;
        }
        break;

    case RxState::SYNC1:
        if (b == SYNC1) {
            _rx_state = RxState::TYPE;
        } else if (b == SYNC0) {
            // stay here; a repeated first sync byte may still start a frame
        } else {
            _rx_state = RxState::SYNC0;
        }
        break;

    case RxState::TYPE:
        _rx_type = b;
        // running Fletcher-8 starts at TYPE
        _rx_ck0 = b;
        _rx_ck1 = b;
        _rx_state = RxState::LEN;
        break;

    case RxState::LEN:
        _rx_len = b;
        _rx_ck0 += b;
        _rx_ck1 += _rx_ck0;
        _rx_idx = 0;
        _rx_state = (_rx_len == 0) ? RxState::CKSUM0 : RxState::DATA;
        break;

    case RxState::DATA:
        _rx_data[_rx_idx++] = b;
        _rx_ck0 += b;
        _rx_ck1 += _rx_ck0;
        if (_rx_idx >= _rx_len) {
            _rx_state = RxState::CKSUM0;
        }
        break;

    case RxState::CKSUM0:
        _rx_ck0_recv = b;
        _rx_state = RxState::CKSUM1;
        break;

    case RxState::CKSUM1:
        if (_rx_ck0_recv == _rx_ck0 && b == _rx_ck1) {
            _stats.rx_packets++;
            dispatch(_rx_type, _rx_data, _rx_len);
        } else {
            _stats.rx_cksum_errors++;
        }
        _rx_state = RxState::SYNC0;
        break;
    }
}

void AP_Syslink::dispatch(uint8_t type, const uint8_t *data, uint8_t len)
{
    int8_t idx = -1;
    {
        WITH_SEMAPHORE(_handler_sem);
        for (uint8_t i = 0; i < MAX_HANDLERS; i++) {
            if (_handlers[i].used && _handlers[i].type == type) {
                idx = i;
                break;
            }
        }
    }

    if (idx < 0) {
        // battery, RSSI and button traffic land here until their phases land
        _stats.rx_unhandled++;
        return;
    }

    // handlers are never unregistered, so this is safe outside the lock
    _handlers[idx].handler(data, len);
}

void AP_Syslink::send_pending()
{
    uint32_t avail;
    {
        WITH_SEMAPHORE(_tx_sem);
        avail = _tx_buf->available();
    }
    if (avail == 0) {
        return;
    }

    if (!flow_control_ok()) {
        return;
    }

    const uint32_t txspace = _uart->txspace();
    if (txspace == 0) {
        return;
    }

    uint8_t buf[128];
    uint32_t n = MIN(MIN(avail, txspace), uint32_t(sizeof(buf)));
    {
        WITH_SEMAPHORE(_tx_sem);
        n = _tx_buf->peekbytes(buf, n);
    }
    if (n == 0) {
        return;
    }

    const size_t written = _uart->write(buf, n);
    if (written > 0) {
        WITH_SEMAPHORE(_tx_sem);
        _tx_buf->advance(written);
    }
}

/*
  The nRF51 drives its UART RTS onto PA4. Asserted low means it can accept
  data. This guards the nRF51's UART receive FIFO only - it says nothing about
  the radio transmit queue, which is reported separately by
  RADIO_MAVLINK_SPACE.
 */
bool AP_Syslink::flow_control_ok()
{
    if (!option_set(Option::USE_FLOW_CONTROL)) {
        return true;
    }

    if (hal.gpio->read(HAL_SYSLINK_FLOWCTRL_PIN) == 0) {
        // clear to send; if we had given up on the line, start trusting it again
        if (_flowctrl_failed) {
            _flowctrl_failed = false;
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Syslink: flow control recovered");
        }
        _flowctrl_blocked_ms = 0;
        return true;
    }

    /*
      Once the line has been held off long enough to not be real FIFO
      backpressure, stop gating on it entirely rather than forcing one write
      per timeout - that would throttle the link to a trickle instead of
      falling back cleanly. Normal gating resumes if the line ever deasserts.
     */
    if (_flowctrl_failed) {
        return true;
    }

    const uint32_t now_ms = AP_HAL::millis();
    if (_flowctrl_blocked_ms == 0) {
        _flowctrl_blocked_ms = now_ms;
        return false;
    }
    if (now_ms - _flowctrl_blocked_ms < SYSLINK_FLOWCTRL_TIMEOUT_MS) {
        return false;
    }

    _stats.flowctrl_timeouts++;
    _flowctrl_failed = true;
    GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Syslink: flow control stuck, ignoring");
    return true;
}

void AP_Syslink::handle_debug_probe(const uint8_t *data, uint8_t len)
{
    if (len < DEBUG_PROBE_LEN) {
        return;
    }
    memcpy(&_probe, data, sizeof(_probe));
    _probe_time_ms = AP_HAL::millis();
}

void AP_Syslink::update_stats_1hz()
{
    const uint32_t now_ms = AP_HAL::millis();
    if (now_ms - _last_1hz_ms < 1000) {
        return;
    }
    _last_1hz_ms = now_ms;

    if (!option_set(Option::LOG_STATS)) {
        return;
    }

#if HAL_LOGGING_ENABLED
    // @LoggerMessage: SYSL
    // @Description: Syslink link statistics, local counters and nRF51 debug probe
    // @Field: TimeUS: Time since system startup
    // @Field: RxP: well-formed syslink packets received
    // @Field: CkE: receive checksum errors
    // @Field: Unh: packets received with no registered handler
    // @Field: TxP: packets queued for transmission
    // @Field: TxD: packets dropped, transmit buffer full
    // @Field: FcT: transmits forced after flow control timeout
    // @Field: Drp: nRF51 reports UART data dropped
    // @Field: UErr: nRF51 UART error flags
    // @Field: Ck1: nRF51 syslink receive checksum 1 error count
    // @Field: Ck2: nRF51 syslink receive checksum 2 error count
    AP::logger().WriteStreaming("SYSL",
                                "TimeUS,RxP,CkE,Unh,TxP,TxD,FcT,Drp,UErr,Ck1,Ck2",
                                "QIIIIIIBBBB",
                                AP_HAL::micros64(),
                                _stats.rx_packets,
                                _stats.rx_cksum_errors,
                                _stats.rx_unhandled,
                                _stats.tx_packets,
                                _stats.tx_dropped,
                                _stats.flowctrl_timeouts,
                                _probe.dropped,
                                _probe.uart_err,
                                _probe.cksum1,
                                _probe.cksum2);
#endif

    // keep the probe data fresh for the next log line
    request_debug_probe();
}

namespace AP {

AP_Syslink *syslink()
{
    return AP_Syslink::get_singleton();
}

}

#endif // AP_SYSLINK_ENABLED
