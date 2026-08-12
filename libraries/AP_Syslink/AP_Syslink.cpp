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

#include <AP_BattMonitor/AP_BattMonitor.h>
#include <AP_BattMonitor/AP_BattMonitor_Backend.h>
#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_Logger/AP_Logger.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include <AP_Math/AP_Math.h>
#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL& hal;

using namespace AP_Syslink_Protocol;

// The nRF51's UART is fixed at 1 Mbaud, 8N1.
#define SYSLINK_BAUD 1000000U

#define SYSLINK_TX_BUF_SIZE 2048
#define SYSLINK_UART_RX_SIZE 512
#define SYSLINK_UART_TX_SIZE 512

#define SYSLINK_THREAD_STACK 2048

/*
  How long the nRF51 may hold its RTS line deasserted before we transmit anyway. The line is only meant to throttle brief UART FIFO pressure, so a
  sustained deassertion means it is unwired or the nRF51 is wedged.
 */
#define SYSLINK_FLOWCTRL_TIMEOUT_MS 100

#define SYSLINK_BATTERY_INTERVAL_MS 100

// The Crazyflie has one pack, and the nRF51 owns the only divider on it.
#define AP_SYSLINK_BATT_INSTANCE 0

#define SYSLINK_CONFIG_RETRY_MS 100
#define SYSLINK_CONFIG_MAX_RETRIES 5

AP_Syslink *AP_Syslink::_singleton;

/*
  This vehicle's identity on the air. MAV_SYSID is the vehicle's one global
  id, so the radio address follows it rather than being set separately.
 */
uint8_t AP_Syslink::get_address() const
{
    return gcs().sysid_this_mav();
}

const AP_Param::GroupInfo AP_Syslink::var_info[] = {

    // @Param: _ENABLE
    // @DisplayName: Syslink enable
    // @Description: Enable the nRF51822 radio co-processor driver. The driver takes exclusive ownership of the serial port whose SERIALn_PROTOCOL is set to 52 (Syslink).
    // @Values: 0:Disabled,1:Enabled
    // @RebootRequired: True
    // @User: Standard
    AP_GROUPINFO_FLAGS("_ENABLE", 1, AP_Syslink, _enable, 1, AP_PARAM_FLAG_ENABLE),

    // index 2 was PORT, before the driver moved to find_serial()

    // @Param: _OPTIONS
    // @DisplayName: Syslink options
    // @Description: Bitmask of syslink driver options.
    // @Bitmask: 0:Use UART flow control line,1:Log SYSL statistics
    // @User: Advanced
    AP_GROUPINFO("_OPTIONS", 3, AP_Syslink, _options, 3),

    // @Param: _CHAN
    // @DisplayName: Radio channel
    // @Description: nRF51 radio channel. Channels are spaced 1MHz apart from 2400MHz, so channel 80 is 2480MHz. Must match the ground station.
    // @Range: 0 125
    // @RebootRequired: True
    // @User: Standard
    AP_GROUPINFO("_CHAN", 4, AP_Syslink, _channel, 80),

    // @Param: _RATE
    // @DisplayName: Radio datarate
    // @Description: nRF51 radio datarate. Must match the ground station.
    // @Values: 0:250Kbps,1:1Mbps,2:2Mbps
    // @RebootRequired: True
    // @User: Standard
    AP_GROUPINFO("_RATE", 5, AP_Syslink, _datarate, 2),

    // index 6 was ADDR; the radio address low byte is MAV_SYSID now

    // @Param: _TXPOW
    // @DisplayName: Radio transmit power
    // @Description: nRF51 radio transmit power in dBm. The nRF51822 supports -30, -20, -16, -12, -8, -4, 0 and +4 dBm; other values are rounded down by the radio.
    // @Range: -30 4
    // @RebootRequired: True
    // @User: Advanced
    AP_GROUPINFO("_TXPOW", 7, AP_Syslink, _txpower, 0),

    // @Param: _BW
    // @DisplayName: Assumed link bandwidth
    // @Description: Bytes per second the GCS layer should assume for the radio link. Paces parameter download and MAVLink FTP burst reads only; it does not affect log download, which is bounded by the transmit queue instead. Too high overruns the 5 deep radio queue and loses more to drops than it gains.
    // @Range: 500 20000
    // @User: Advanced
    AP_GROUPINFO("_BW", 8, AP_Syslink, _link_bw, 4000),

    // index 9 was BATT; the battery always feeds instance 0

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

    _tx_buf = NEW_NOTHROW ByteBuffer(SYSLINK_TX_BUF_SIZE);
    if (_tx_buf == nullptr) {
        AP_BoardConfig::allocation_error("AP_Syslink tx buffer");
        return;
    }

    /*
      Every configuration packet is echoed back by the nRF51. One handler serves them all; the boot sequence uses the echo as confirmation
     */
    const Type echoed[] = {
        Type::RADIO_READY,
        Type::RADIO_CHANNEL,
        Type::RADIO_DATARATE,
        Type::RADIO_ADDRESS,
        Type::RADIO_POWER,
    };
    for (const auto t : echoed) {
        if (!register_handler(t, FUNCTOR_BIND_MEMBER(&AP_Syslink::handle_config_echo, void, uint8_t, const uint8_t *, uint8_t))) {
            GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "Syslink: handler table full");
            break;
        }
    }

#if AP_SERIALMANAGER_REGISTER_ENABLED
    /*
      Register the virtual MAVLink port before GCS::setup_uarts() runs, which
      is when the GCS binds its channels. Unicast chunks and the free-slot
      report both belong to it.
     */
    if (!_mavlink_port.init(*this)) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "Syslink: MAVLink port init failed");
    } else {
        if (!register_handler(Type::RADIO_MAVLINK,
                              FUNCTOR_BIND(&_mavlink_port, &AP_Syslink_MAVLinkPort::handle_chunk, void, uint8_t, const uint8_t *, uint8_t)) ||
            !register_handler(Type::RADIO_MAVLINK_SPACE,
                              FUNCTOR_BIND(&_mavlink_port, &AP_Syslink_MAVLinkPort::handle_space, void, uint8_t, const uint8_t *, uint8_t))) {
            GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "Syslink: handler table full");
        }
    }
#endif

    /*
      Inbound peer broadcasts. Registered unconditionally so the receive path
      exists before any consumer attaches; without a handler the payload is
      counted and discarded.
     */
    if (!register_handler(Type::RADIO_MAVLINK_BROADCAST,
                          FUNCTOR_BIND_MEMBER(&AP_Syslink::handle_broadcast, void, uint8_t, const uint8_t *, uint8_t))) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "Syslink: handler table full");
    }

#if AP_BATTERY_SCRIPTING_ENABLED
    if (!register_handler(Type::PM_BATTERY_STATE,
                          FUNCTOR_BIND_MEMBER(&AP_Syslink::handle_battery_state, void, uint8_t, const uint8_t *, uint8_t))) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "Syslink: handler table full");
    }
#endif

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
      Reject the whole frame rather than writing part of it (a truncated frame would desynchronise the nRF51's parser until the next sync pair)
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

/*
  Open the port. Must run on the driver thread.
 */
bool AP_Syslink::init_port()
{
    /*
      The port must carry SerialProtocol_Syslink rather than being leftunassigned: AP_SerialManager::init() calls disable_rxtx() on a
      SerialProtocol_None port, and on STM32F4 nothing restores the pin muxing afterwards, so begin() would land on disconnected pins.
     */
    _uart = AP::serialmanager().find_serial(AP_SerialManager::SerialProtocol_Syslink, 0);
    if (_uart == nullptr) {
        return false;
    }
    _uart->begin(SYSLINK_BAUD, SYSLINK_UART_RX_SIZE, SYSLINK_UART_TX_SIZE);
    return true;
}

void AP_Syslink::thread_main()
{
    if (!init_port()) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "Syslink: no serial port with protocol %u",
                      unsigned(AP_SerialManager::SerialProtocol_Syslink));
        return;
    }
    _port_ready = true;

    while (true) {
        hal.scheduler->delay_microseconds(200);

        receive_bytes();
        update_config();
#if AP_SERIALMANAGER_REGISTER_ENABLED
        /*
          Hold telemetry until the radio is on the configured channel and
          address; chunks sent before that go out on the wrong settings and
          only burn transmit slots.
         */
        if (configured()) {
            _mavlink_port.update();
        }
#endif
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
        _stats.rx_bytes += n;
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
    _handlers[idx].handler(type, data, len);
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
        _stats.tx_bytes += written;
        WITH_SEMAPHORE(_tx_sem);
        _tx_buf->advance(written);
    }
}

/*
  The nRF51 drives its UART RTS onto PA4. Asserted low means it can accept data. This guards the nRF51's UART receive FIFO only, it says nothing about
  the radio transmit queue, which is reported separately by RADIO_MAVLINK_SPACE.
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
      Once the line has been held off long enough to not be real FIFO backpressure, stop gating on it entirely rather than forcing one write
      per timeout - that would throttle the link to a trickle instead of falling back cleanly. Normal gating resumes if the line ever deasserts.
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

bool AP_Syslink::send_broadcast(const uint8_t *data, uint8_t len)
{
    if (data == nullptr || len == 0 || len > broadcast_max_len()) {
        /*
          The nRF51 discards an over-length chunk rather than truncating it,
          so refusing here only makes a silent failure visible.
         */
        _stats.bcast_rejected++;
        return false;
    }

    /*
      Hold off until the radio is on the configured channel and address, for
      the same reason telemetry does: anything sent earlier goes out on the
      nRF51's compiled-in defaults, where no peer is listening.
     */
    if (!configured()) {
        return false;
    }

    // send_packet() counts its own drop if the transmit buffer is full
    if (!send_packet(Type::RADIO_MAVLINK_BROADCAST, data, len)) {
        return false;
    }

    _stats.bcast_tx++;
    return true;
}

void AP_Syslink::set_broadcast_handler(BroadcastHandler handler)
{
    WITH_SEMAPHORE(_handler_sem);
    _broadcast_handler = handler;
    _have_broadcast_handler = true;
}

void AP_Syslink::handle_broadcast(uint8_t type, const uint8_t *data, uint8_t len)
{
    (void)type;

    _stats.bcast_rx++;

    BroadcastHandler handler;
    bool have;
    {
        WITH_SEMAPHORE(_handler_sem);
        have = _have_broadcast_handler;
        handler = _broadcast_handler;
    }
    if (!have || len == 0) {
        return;
    }

    // payload only; the framing and checksum are already stripped and verified
    handler(data, len);
}

bool AP_Syslink::is_charging() const
{
    return (_batt_flags & BATTERY_FLAG_CHARGING) != 0;
}

bool AP_Syslink::is_usb_powered() const
{
    return (_batt_flags & BATTERY_FLAG_USB_POWERED) != 0;
}

#if AP_BATTERY_SCRIPTING_ENABLED
/*
  PM_BATTERY_STATE from the nRF51, fed to the battery monitor's scripting backend.
 */
void AP_Syslink::handle_battery_state(uint8_t type, const uint8_t *data, uint8_t len)
{
    (void)type;

    /*
      9 bytes without the optional die temperature, 13 with it. This build of
      the nRF51 firmware has PM_SYSLINK_INCLUDE_TEMP enabled so 13 is what
      arrives, but the field is a compile-time option there, so accept both
      rather than silently rejecting every packet after a firmware rebuild.
     */
    if (len != BATTERY_STATE_LEN && len != BATTERY_STATE_LEN_WITH_TEMP) {
        return;
    }

    _batt_flags = data[0];
    _batt_time_ms = AP_HAL::millis();

    if (_batt_time_ms - _last_battery_ms < SYSLINK_BATTERY_INTERVAL_MS) {
        return;
    }
    _last_battery_ms = _batt_time_ms;

    // copied out rather than read through a packed struct: data points into
    // the receive buffer and carries no alignment guarantee for a float
    float vbat;
    memcpy(&vbat, &data[1], sizeof(vbat));

    if (isnan(vbat) || vbat <= 0.0f || vbat > 20.0f) {
        // implausible; better to report nothing than to trip a failsafe
        return;
    }

    BattMonitorScript_State state {};
    state.voltage = vbat;
    state.healthy = true;
    state.cell_count = 1;                                   // Crazyflie 2.x is 1S
    state.cell_voltages[0] = uint16_t(vbat * 1000.0f);

    /*
      Everything else stays unknown, which the struct defaults to NaN.

      ISET is the *charge* current in milliamperes, not discharge, so feeding
      it to current_amps would read plausibly wrong in flight and poison the
      consumed-mAh integration that the backend derives from it.

      TEMP is the nRF51 die temperature - not the battery, not ambient. It
      sits above room temperature and climbs under radio load, and on boards
      without a charger it is never sampled and stays a convincing 0 C.
     */

    if (!AP::battery().handle_scripting(AP_SYSLINK_BATT_INSTANCE, state) && !_batt_warned) {
        _batt_warned = true;
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Syslink: set BATT_MONITOR=29 (Scripting)");
    }
}
#endif // AP_BATTERY_SCRIPTING_ENABLED

void AP_Syslink::handle_config_echo(uint8_t type, const uint8_t *data, uint8_t len)
{
    (void)data;
    (void)len;
    _config_echo_type = type;
    _config_echo_ms = AP_HAL::millis();
}

/*
  Drive the boot sequence.

  The nRF51 transmits nothing at all over the UART until it has received one syslink packet that passes both checksum bytes.
 */
void AP_Syslink::update_config()
{
    if (_config_state == ConfigState::DONE) {
        return;
    }

    const uint32_t now_ms = AP_HAL::millis();

    // current step acknowledged?
    if (_config_sent &&
        _config_echo_type == _config_expect &&
        _config_echo_ms >= _config_sent_ms) {
        if (_config_state == ConfigState::READY) {
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Syslink: nRF51 link up");
        }
        advance_config();
        return;
    }

    if (_config_sent && (now_ms - _config_sent_ms) < SYSLINK_CONFIG_RETRY_MS) {
        return;     // still waiting for the echo
    }

    if (_config_sent) {
        _config_retries++;
        /*
          RADIO_READY is retried indefinitely: nothing else can work until the
          nRF51's transmit gate lifts, and the nRF51 may simply not have booted
          yet. The remaining steps give up after a few attempts so a single
          lost echo does not wedge the sequence.
         */
        if (_config_state == ConfigState::READY) {
            if (_config_retries % 20 == 0) {
                /*
                  tx climbing with rx at zero means the nRF51 is not answering
                  or the read path is broken; both at zero means nothing is
                  leaving the STM32.
                 */
                GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Syslink: no echo (tx %lu rx %lu bytes)",
                              (unsigned long)_stats.tx_bytes,
                              (unsigned long)_stats.rx_bytes);
            }
        } else if (_config_retries > SYSLINK_CONFIG_MAX_RETRIES) {
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Syslink: step %u not confirmed",
                          unsigned(_config_state));
            advance_config();
            return;
        }
    }

    send_config_step();
}

void AP_Syslink::send_config_step()
{
    bool sent = false;

    switch (_config_state) {
    case ConfigState::READY:
        _config_expect = uint8_t(Type::RADIO_READY);
        sent = send_packet(Type::RADIO_READY);
        break;

    case ConfigState::AUTOUPDATE:
        // enables both battery state and RSSI; not echoed
        _config_expect = 0xFF;
        sent = send_packet(Type::PM_BATTERY_AUTOUPDATE);
        if (sent) {
            advance_config();
            return;
        }
        break;

    case ConfigState::CHANNEL: {
        const uint8_t chan = constrain_int16(_channel.get(), 0, 125);
        _config_expect = uint8_t(Type::RADIO_CHANNEL);
        sent = send_packet(Type::RADIO_CHANNEL, &chan, 1);
        break;
    }

    case ConfigState::DATARATE: {
        const uint8_t rate = constrain_int16(_datarate.get(), 0, 2);
        _config_expect = uint8_t(Type::RADIO_DATARATE);
        sent = send_packet(Type::RADIO_DATARATE, &rate, 1);
        break;
    }

    case ConfigState::ADDRESS: {
        /*
          Five bytes, little-endian, so the low byte goes first. The upper four
          are fixed at E7E7E7E7 by Crazyflie convention; MAV_SYSID sets the
          last byte, giving the E7E7E7E7xx of a radio:// URI.
         */
        const uint8_t addr[ADDRESS_LEN] = {
            get_address(),
            0xE7, 0xE7, 0xE7, 0xE7
        };
        _config_expect = uint8_t(Type::RADIO_ADDRESS);
        sent = send_packet(Type::RADIO_ADDRESS, addr, sizeof(addr));
        break;
    }

    case ConfigState::POWER: {
        const int8_t pwr = constrain_int16(_txpower.get(), -30, 4);
        _config_expect = uint8_t(Type::RADIO_POWER);
        sent = send_packet(Type::RADIO_POWER, (const uint8_t *)&pwr, 1);
        break;
    }

    case ConfigState::DONE:
        return;
    }

    if (sent) {
        _config_sent = true;
        _config_sent_ms = AP_HAL::millis();
    }
}

void AP_Syslink::advance_config()
{
    _config_sent = false;
    _config_retries = 0;
    _config_expect = 0xFF;

    switch (_config_state) {
    case ConfigState::READY:
        _config_state = ConfigState::AUTOUPDATE;
        break;
    case ConfigState::AUTOUPDATE:
        _config_state = ConfigState::CHANNEL;
        break;
    case ConfigState::CHANNEL:
        _config_state = ConfigState::DATARATE;
        break;
    case ConfigState::DATARATE:
        _config_state = ConfigState::ADDRESS;
        break;
    case ConfigState::ADDRESS:
        _config_state = ConfigState::POWER;
        break;
    case ConfigState::POWER:
        _config_state = ConfigState::DONE;
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Syslink: radio configured ch%u E7E7E7E7%02X",
                      unsigned(constrain_int16(_channel.get(), 0, 125)),
                      unsigned(get_address()));
        break;
    case ConfigState::DONE:
        break;
    }
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
    // @Description: Syslink link statistics
    // @Field: TimeUS: Time since system startup
    // @Field: RxB: raw bytes read from the UART
    // @Field: TxB: raw bytes written to the UART
    // @Field: RxP: well-formed syslink packets received
    // @Field: CkE: receive checksum errors
    // @Field: Unh: packets received with no registered handler
    // @Field: TxP: packets queued for transmission
    // @Field: TxD: packets dropped, transmit buffer full
    // @Field: FcT: transmits forced after flow control timeout
    // @Field: BTx: peer broadcasts queued
    // @Field: BRx: peer broadcasts received
    AP::logger().WriteStreaming("SYSL",
                                "TimeUS,RxB,TxB,RxP,CkE,Unh,TxP,TxD,FcT,BTx,BRx",
                                "QIIIIIIIIII",
                                AP_HAL::micros64(),
                                _stats.rx_bytes,
                                _stats.tx_bytes,
                                _stats.rx_packets,
                                _stats.rx_cksum_errors,
                                _stats.rx_unhandled,
                                _stats.tx_packets,
                                _stats.tx_dropped,
                                _stats.flowctrl_timeouts,
                                _stats.bcast_tx,
                                _stats.bcast_rx);
#endif
}

namespace AP {

AP_Syslink *syslink()
{
    return AP_Syslink::get_singleton();
}

}

#endif // AP_SYSLINK_ENABLED
