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

#include "AP_Ranging_DW1000.h"

#if AP_RANGING_DW1000_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <GCS_MAVLink/GCS.h>
#include <string.h>
#include <AP_Logger/AP_Logger.h>
#if AP_RANGING_RF_DIAGNOSTICS_ENABLED
#include "dw1000_radio_diag.h"
#endif

extern const AP_HAL::HAL &hal;

AP_Ranging_DW1000::AP_Ranging_DW1000(AP_Ranging &frontend) : AP_Ranging_Backend(frontend)
{
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "UWB: DW1000 initializing");
    if (!init_device()) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "UWB: DW1000 init FAILED");
    }
}

// acquire the SPI device, initialise libdw1000 and verify the chip id.
bool AP_Ranging_DW1000::init_device()
{
    _mode = get_mode();
#if AP_RANGING_RF_DIAGNOSTICS_ENABLED && HAL_LOGGING_ENABLED
    _rf_mask = _mode == AP_Ranging::Mode::Anchor ? get_rf_diag_mask() : 0;
    if (_rf_mask != 0) {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "UWB: radio diagnostics mask=%u", unsigned(_rf_mask));
    }
#endif
    _poll_ms = get_poll_ms();
    _reply_us = get_reply_us();
    const uint16_t timeout = get_xchg_ms();
    if (get_node_id() == 0 || _poll_ms < 10 || _poll_ms > 1000 ||
        _reply_us < 1500 || _reply_us > 20000 || timeout > 200 ||
        timeout * 1000U < 4U * _reply_us + 3000U) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "Ranging: invalid four-message timing");
        return false;
    }
    twr_init(&_engine, TWR_TAG, get_node_id(), get_network_id(), timeout);
    if (!hal.util->get_true_random_vals(reinterpret_cast<uint8_t *>(&_session), sizeof(_session), 1000)) {
        _session = (uint32_t(get_random16()) << 16) ^ get_random16() ^ AP_HAL::micros();
    }
    _random_state = (_session ^ (uint32_t(_engine.id) * 0x9e3779b9U)) | 1U;
    _poll_interval = random_below(50);
    if (_mode == AP_Ranging::Mode::Anchor) {
        for (uint8_t i = 0; i < AP_RANGING_MAX_ANCHORS; i++) {
            if (anchor_enabled(i)) {
                _targets[_target_count++] = i + 1;
            }
        }
    } else {
        for (uint8_t i = 0; i < AP_RANGING_MAX_NODES; i++) {
            const uint8_t id = get_peer_id(i);
            if (id != 0 && id != _engine.id && !admitted_peer(id)) {
                _targets[_target_count++] = id;
            }
        }
    }

    _dev = hal.spi->get_device("dw1000");
    if (!_dev) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "DW1000: SPI device not found");
        return false;
    }

    /*
      service_radio() drives the whole exchange off a polled read of the IRQ
      line, so an unmapped pin is fatal: hal.gpio->read() answers 0 for a pin
      that is not in the board's GPIO table, which reads as "no interrupt
      pending" forever. Without this check the driver reports itself ready,
      never ranges, and leaves a 3.3kHz callback polling a dead pin on a SPI
      bus it shares with the optical flow sensor.
     */
    if (!hal.gpio->valid_pin(HAL_DW1000_IRQ_PIN)) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "DW1000: IRQ pin GPIO(%u) not in hwdef",
                      unsigned(HAL_DW1000_IRQ_PIN));
        return false;
    }

    // hand our hardware ops to libdw1000. dwInit() only touches the device
    // struct (no bus traffic), so it needs no semaphore.
    _ops.spiRead     = &AP_Ranging_DW1000::spiRead;
    _ops.spiWrite    = &AP_Ranging_DW1000::spiWrite;
    _ops.spiSetSpeed = &AP_Ranging_DW1000::spiSetSpeed;
    _ops.delayms     = &AP_Ranging_DW1000::delayms;
    _ops.reset       = nullptr;   // SPI soft-reset; board startup handles the hardware reset

    dwInit(&_dw, &_ops);
    dwSetUserdata(&_dw, this);     // lets the C ops/handlers recover 'this'

    {
        WITH_SEMAPHORE(_dev->get_semaphore());

        _dev->set_speed(AP_HAL::Device::SPEED_LOW);

        const uint32_t id = dwGetDeviceId(&_dw);
        if (id != AP_RANGING_DW1000_DEVICE_ID) {
            GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "DW1000: not detected (id=0x%08x)", (unsigned)id);
            return false;
        }
        if (dwConfigure(&_dw) != 0) {
            GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "DW1000: configure failed");
            return false;
        }

        // Match the Loco deck firmware: startup blink, then hardware RX/TX/SFD/RXOK indicators.
        dwEnableAllLeds(&_dw);

        configure_radio();
        arm_receiver();          // start listening
    }

    // AP_HAL periods are microseconds. SPI/IRQ servicing stays on the bus thread.
    if (_dev->register_periodic_callback(300, FUNCTOR_BIND_MEMBER(&AP_Ranging_DW1000::timer, void)) == nullptr) {
        {
            WITH_SEMAPHORE(_dev->get_semaphore());
            dwIdle(&_dw);
        }
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "DW1000: callback registration failed");
        return false;
    }
    _initialised = true;
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "UWB: init OK, %s id=%u targets=%u",
                  _mode == AP_Ranging::Mode::Anchor ? "anchor" : "peer",
                  unsigned(_engine.id), unsigned(_target_count));
    return true;
}

bool AP_Ranging_DW1000::healthy()
{
    return _initialised && _have_range && AP_HAL::millis() - _last_update_ms < AP_RANGING_TIMEOUT_MS;
}

void AP_Ranging_DW1000::update()
{
    // Only the main thread mutates frontend snapshots and writes logs.
    for (uint8_t i = 0; i < QUEUE_SIZE; i++) {
        Sample sample;
        {
            WITH_SEMAPHORE(_queue_sem);
            if (_queue_count == 0) {
                break;
            }
            sample = _queue[_queue_head];
            _queue_head = (_queue_head + 1) % QUEUE_SIZE;
            _queue_count--;
        }
        if (_mode == AP_Ranging::Mode::Anchor) {
            set_anchor_distance(sample.remote_id, sample.distance, sample.time_ms);
        } else {
            set_node_distance(sample.remote_id, sample.distance, sample.time_ms);
        }
        _last_update_ms = sample.time_ms;
        _have_range = true;
#if HAL_LOGGING_ENABLED
        AP::logger().Write("RTWR", "TimeUS,AcqMS,Session,Seq,Remote,InitID,Init,Dist",
                          "QIIHBBBf", AP_HAL::micros64(), sample.time_ms, sample.session,
                          sample.sequence, sample.remote_id, sample.initiator_id,
                          uint8_t(sample.initiated), sample.distance);
#if AP_RANGING_RF_DIAGNOSTICS_ENABLED
        if (sample.rf_requested) {
            for (uint8_t j = 0; j < 3; j++) {
                const auto &rx = sample.rf_rx[j];
                // Pkt 1/3 are captured at the anchor, Pkt 2 at this tag.
                AP::logger().Write("RFRX", "TimeUS,Session,Seq,Anchor,Pkt,RxP,FpP,Corr,Raw,Flags,CapUS",
                                   "QIHBBhhhQBI", AP_HAL::micros64(), sample.session, sample.sequence,
                                   sample.remote_id, uint8_t(j + 1), rx.rx_cdbm, rx.fp_cdbm,
                                   rx.correction, rx.raw, rx.flags, j == 1 ? sample.rf_capture_us : 0U);
            }
            AP::logger().Write("RFRG", "TimeUS,AcqMS,Session,Seq,Anchor,Dist,Uncorr,Valid", "QIIHBffB",
                               AP_HAL::micros64(), sample.time_ms, sample.session, sample.sequence,
                               sample.remote_id, sample.distance, sample.uncorrected, uint8_t(sample.rf_valid));
        }
#endif
#endif
    }
    Diagnostics diag;
    {
        WITH_SEMAPHORE(_queue_sem);
        if (!_diag_ready) {
            return;
        }
        diag = _diag_snapshot;
        _diag_ready = false;
    }
    if (get_debug() <= 0) {
        return;
    }
#if HAL_LOGGING_ENABLED
    AP::logger().Write("RWDG", "TimeUS,Fail,TxSt,TxDif,Exp,RxErr,RxTO,SPI,QDrop,Gap,GapMax,Work",
                      "QIIIIIIIIIII", uint64_t(diag.time_ms) * 1000,
                      diag.failures, diag.tx_state, diag.tx_mismatch, diag.expired,
                      diag.rx_error, diag.rx_timeout, diag.spi_error, diag.queue_drops,
                      diag.gap_us, diag.gap_max_us, diag.work_max_us);
    if (diag.tx_time_us != 0 && diag.tx_time_us != _last_tx_diag_us) {
        _last_tx_diag_us = diag.tx_time_us;
        AP::logger().Write("RWTX", "TimeUS,ID,State,Seq,Expected,Actual,GapUS",
                          "QBBHQQI", diag.tx_time_us, diag.tx_remote, diag.tx_state_before,
                          diag.tx_sequence, diag.expected_tx, diag.actual_tx, diag.tx_gap_us);
    }
#endif
}

void AP_Ranging_DW1000::timer()
{
    const uint32_t start_us = AP_HAL::micros();
    _callback_gap_us = _last_callback_us == 0 ? 0 : start_us - _last_callback_us;
    _last_callback_us = start_us;
    _diag.gap_us = MAX(_diag.gap_us, _callback_gap_us);
    _diag.gap_max_us = MAX(_diag.gap_max_us, _callback_gap_us);
    // Expiry is checked before dispatch as well, so a late TX cannot revive a transaction.
    if (twr_expired(&_engine, AP_HAL::millis())) {
        _diag.expired++;
        abort_exchange();
    }
    service_radio();
    const uint32_t now = AP_HAL::millis();
    if (_engine.state == TWR_IDLE && now - _last_poll_ms >= _poll_interval) {
        _last_poll_ms = now;
        const uint16_t jitter = _mode == AP_Ranging::Mode::Anchor ? 6 : 50;
        _poll_interval = _poll_ms + random_below(jitter);
        const uint8_t target = next_poll_target();
        if (target != 0 && twr_start(&_engine, _mode == AP_Ranging::Mode::Anchor ? TWR_ANCHOR : TWR_TAG,
                                     target, _session, ++_next_sequence, now)) {
            send_packet();
        }
    }
    _diag.work_max_us = MAX(_diag.work_max_us, uint32_t(AP_HAL::micros() - start_us));
    if (now - _last_diag_ms >= 1000 && _queue_sem.take_nonblocking()) {
        _diag.time_ms = now;
        _diag.failures = _exchange_fail;
        _diag.queue_drops = _queue_drops;
        _diag_snapshot = _diag;
        _diag.gap_us = 0;
        _diag_ready = true;
        _last_diag_ms = now;
        _queue_sem.give();
    }
}

// configure RF settings (must match peers) and attach the event handlers
bool AP_Ranging_DW1000::configure_radio()
{
    dwAttachSentHandler(&_dw, &AP_Ranging_DW1000::handle_sent);
    dwAttachReceivedHandler(&_dw, &AP_Ranging_DW1000::handle_received);
    dwAttachReceiveTimeoutHandler(&_dw, &AP_Ranging_DW1000::handle_rx_timeout);
    dwAttachReceiveFailedHandler(&_dw, &AP_Ranging_DW1000::handle_rx_failed);

    dwNewConfiguration(&_dw);
    dwSetDefaults(&_dw);
    dwEnableMode(&_dw, MODE_SHORTDATA_FAST_ACCURACY);
    dwSetChannel(&_dw, get_channel());   // RNG_CHAN (all nodes must match)
    dwSetPreambleCode(&_dw, PREAMBLE_CODE_64MHZ_9);
    dwUseSmartPower(&_dw, true);

    // antenna delay from RNG_ANT_DLY (calibration)
    dwTime_t antenna_delay = {};
    antenna_delay.full = get_ant_delay();
    dwSetAntenaDelay(&_dw, antenna_delay);

    dwCommitConfiguration(&_dw);

    // Enable the chip's interrupt outputs so the IRQ pin reflects RX/TX events.
    // dwConfigure() cleared the mask, so set our sources and flush the mask.
    // We POLL the IRQ pin as a flag in service_radio() (not a hardware ISR), so
    // we only touch SYS_STATUS when the chip signals a real event
    dwInterruptOnSent(&_dw, true);
    dwInterruptOnReceived(&_dw, true);
    dwInterruptOnReceiveFailed(&_dw, true);
    dwInterruptOnReceiveTimeout(&_dw, true);
    dwWriteSystemEventMaskRegister(&_dw);
    return true;
}

// plain (non-permanent) receive: our "listening" / IDLE state. Mid-exchange RX
// is armed automatically by wait4resp, so this is only for idle listening and
// post-exchange recovery.
void AP_Ranging_DW1000::arm_receiver()
{
    dwIdle(&_dw);
    dwNewReceive(&_dw);
    dwSetDefaults(&_dw);
    dwStartReceive(&_dw);
}

// service the radio ONLY when the DW1000 IRQ line is asserted (polled as a flag,
// no hardware ISR). dwHandleInterrupt() reads/clears SYS_STATUS and dispatches to our handlers
void AP_Ranging_DW1000::service_radio()
{
    if (hal.gpio->read(HAL_DW1000_IRQ_PIN)) {
        dwHandleInterrupt(&_dw);
    }
}

uint16_t AP_Ranging_DW1000::random_below(uint16_t limit)
{
    // Private state avoids sharing AP_Math's deterministic generator across bus threads.
    _random_state ^= _random_state << 13;
    _random_state ^= _random_state >> 17;
    _random_state ^= _random_state << 5;
    return _random_state % limit;
}

uint8_t AP_Ranging_DW1000::next_poll_target()
{
    if (_target_count == 0) {
        return 0;
    }
    const uint32_t now = AP_HAL::millis();
    for (uint8_t i = 0; i < _target_count; i++) {
        const uint8_t slot = _next_target;
        _next_target = (_next_target + 1) % _target_count;
        if (now - _target_time_ms[slot] >= _target_wait_ms[slot]) {
            return _targets[slot];
        }
    }
    return 0;
}

bool AP_Ranging_DW1000::admitted_peer(uint8_t id) const
{
    for (uint8_t i = 0; i < _target_count; i++) {
        if (_targets[i] == id) {
            return true;
        }
    }
    return false;
}

void AP_Ranging_DW1000::send_packet()
{
    dwNewTransmit(&_dw);
    dwTime_t delay {};
    delay.full = uint64_t(double(_reply_us) / TIME_RES);
    const dwTime_t scheduled = dwSetDelay(&_dw, &delay);
    // libdw1000 writes two additional FCS bytes from this buffer.
    uint8_t frame[FRAME_SIZE + 2] {};
    unsigned len = twr_prepare_tx(&_engine, scheduled.full, frame);
#if AP_RANGING_RF_DIAGNOSTICS_ENABLED
    if (_engine.state == TWR_TX_POLL) {
        _rf_active = _mode == AP_Ranging::Mode::Anchor && _engine.remote_id <= 8 &&
                     (_rf_mask & (1U << (_engine.remote_id - 1))) != 0;
        memset(_rf_rx, 0, sizeof(_rf_rx));
        _rf_capture_us = 0;
        if (_rf_active) {
            len = twr_diag_request(frame, len);
        }
    }
#endif
    if (len == 0) {
        abort_exchange();
        return;
    }
    dwSetData(&_dw, frame, len);
    dwWaitForResponse(&_dw, _engine.state != TWR_TX_REPORT);
    dwStartTransmit(&_dw);
}

void AP_Ranging_DW1000::publish_range(uint64_t rx_timestamp)
{
    const uint64_t *times = _engine.initiator ? _engine.a : _engine.b;
    const uint32_t span_ms = ((times[2] - times[0]) & TWR_TIMESTAMP_MASK) / TWR_TICKS_PER_MS;
    const uint32_t report_ms = _engine.initiator ?
        ((rx_timestamp - times[2]) & TWR_TIMESTAMP_MASK) / TWR_TICKS_PER_MS : 0;
    for (uint8_t i = 0; i < _target_count; i++) {
        if (_targets[i] == _engine.remote_id) {
            _target_time_ms[i] = AP_HAL::millis();
            _target_failures[i] = 0;
            // Either exchange role refreshes a peer; avoid an immediate reverse poll.
            _target_wait_ms[i] = _mode == AP_Ranging::Mode::Peer ? _poll_ms : 0;
        }
    }
    Sample sample {};
    sample.distance = _engine.distance;
    sample.time_ms = AP_HAL::millis() - report_ms - span_ms / 2;
    sample.session = _engine.session;
    sample.sequence = _engine.sequence;
    sample.remote_id = _engine.remote_id;
    sample.initiator_id = _engine.initiator ? _engine.id : _engine.remote_id;
    sample.initiated = _engine.initiator;
#if AP_RANGING_RF_DIAGNOSTICS_ENABLED
    sample.rf_requested = _rf_active && _engine.initiator;
    if (sample.rf_requested) {
        memcpy(sample.rf_rx, _rf_rx, sizeof(_rf_rx));
        sample.rf_capture_us = _rf_capture_us;
        sample.uncorrected = NAN;
        sample.rf_valid = twr_diag_uncorrected(&_engine, _rf_rx, &sample.uncorrected);
    }
#endif
    if (!_queue_sem.take_nonblocking()) {
        _queue_drops++;
        return;
    }
    if (_queue_count < QUEUE_SIZE) {
        _queue[(_queue_head + _queue_count) % QUEUE_SIZE] = sample;
        _queue_count++;
    } else {
        _queue_drops++;
    }
    _queue_sem.give();
}

void AP_Ranging_DW1000::abort_exchange()
{
    _exchange_fail++;
    if (_engine.initiator && _engine.state != TWR_IDLE) {
        for (uint8_t i = 0; i < _target_count; i++) {
            if (_targets[i] == _engine.remote_id) {
                _target_failures[i] = MIN(_target_failures[i] + 1, 4);
                _target_time_ms[i] = AP_HAL::millis();
                _target_wait_ms[i] = 50U << _target_failures[i];
            }
        }
    }
    twr_abort(&_engine);
    // Stop TX before clearing its latch. dwNewReceive() only clears RX status,
    // so an expired exchange could otherwise dispatch TX-done against IDLE.
    dwIdle(&_dw);
    dwClearTransmitStatus(&_dw);
    arm_receiver();
}

void AP_Ranging_DW1000::handle_sent(dwDevice_t *dev)
{
    auto *b = static_cast<AP_Ranging_DW1000 *>(dwGetUserdata(dev));
    dwTime_t tx {};
    dwGetTransmitTimestamp(dev, &tx);
    const bool pending = twr_tx_pending(&b->_engine);
    if (!pending || (tx.full & TWR_TIMESTAMP_MASK) != b->_engine.expected_tx) {
        if (pending) {
            b->_diag.tx_mismatch++;
        } else {
            b->_diag.tx_state++;
        }
        b->_diag.tx_time_us = AP_HAL::micros64();
        b->_diag.tx_remote = b->_engine.remote_id;
        b->_diag.tx_sequence = b->_engine.sequence;
        b->_diag.tx_state_before = b->_engine.state;
        b->_diag.expected_tx = b->_engine.expected_tx;
        b->_diag.actual_tx = tx.full & TWR_TIMESTAMP_MASK;
        b->_diag.tx_gap_us = b->_callback_gap_us;
        // Preserve the pre-abort state so retry backoff still sees the target.
        b->abort_exchange();
        return;
    }
    twr_sent(&b->_engine, tx.full);
    if (b->_engine.state == TWR_IDLE) {
        b->arm_receiver();
    }
}

void AP_Ranging_DW1000::handle_received(dwDevice_t *dev)
{
    auto *b = static_cast<AP_Ranging_DW1000 *>(dwGetUserdata(dev));
    uint8_t frame[FRAME_SIZE];
    twr_packet packet;
    dwTime_t rx {};
    const unsigned len = dwGetDataLength(dev);
    bool valid = false;
#if AP_RANGING_RF_DIAGNOSTICS_ENABLED
    bool request = false, report = false;
    twr_diag_rx remote[2] {};
#endif
    if (len <= sizeof(frame)) {
        dwGetData(dev, frame, len);
#if AP_RANGING_RF_DIAGNOSTICS_ENABLED
        valid = twr_diag_decode(frame, len, &packet, &request, &report, remote);
        // This backend is a diagnostic requester, not an extended responder.
        valid = valid && !request;
#else
        valid = twr_decode(frame, len, &packet);
#endif
    }
#if AP_RANGING_RF_DIAGNOSTICS_ENABLED
    const bool matching = valid && b->_rf_active && b->_engine.initiator &&
        packet.network == b->_engine.network && packet.src_role == TWR_ANCHOR &&
        packet.src_id == b->_engine.remote_id && packet.dst_role == TWR_TAG &&
        packet.dst_id == b->_engine.id && packet.session == b->_engine.session &&
        packet.sequence == b->_engine.sequence;
    if (matching && packet.type == TWR_RESPONSE && b->_engine.state == TWR_WAIT_RESPONSE) {
        const uint32_t start = AP_HAL::micros();
        rx = twr_diag_read(dev, &b->_rf_rx[1]);
        b->_rf_capture_us = AP_HAL::micros() - start;
    } else {
        dwGetReceiveTimestamp(dev, &rx);
    }
    if (matching && report && b->_engine.state == TWR_WAIT_REPORT) {
        b->_rf_rx[0] = remote[0];
        b->_rf_rx[2] = remote[1];
    }
#else
    dwGetReceiveTimestamp(dev, &rx);
#endif
    enum twr_result result = TWR_IGNORED;
    if (valid) {
        const bool admit = b->_mode == AP_Ranging::Mode::Peer && b->admitted_peer(packet.src_id);
        result = twr_receive(&b->_engine, &packet, rx.full, AP_HAL::millis(), admit);
        if (b->_engine.new_range) {
            b->publish_range(rx.full);
        }
    }
    if (result == TWR_SEND) {
        b->send_packet();
    } else if (result == TWR_FAILED) {
        b->abort_exchange();
    } else if (!twr_tx_pending(&b->_engine)) {
        // Do not cancel a scheduled TX in response to an unrelated latched RX event.
        b->arm_receiver();
    }
}

void AP_Ranging_DW1000::handle_rx_timeout(dwDevice_t *dev)
{
    auto *b = static_cast<AP_Ranging_DW1000 *>(dwGetUserdata(dev));
    b->_diag.rx_timeout++;
    b->abort_exchange();
}

void AP_Ranging_DW1000::handle_rx_failed(dwDevice_t *dev)
{
    auto *b = static_cast<AP_Ranging_DW1000 *>(dwGetUserdata(dev));
    b->_diag.rx_error++;
    if (!twr_tx_pending(&b->_engine)) {
        b->arm_receiver();
    }
}

// ---- libdw1000 hardware ops ----

void AP_Ranging_DW1000::spiRead(dwDevice_t *dev, const void *header, size_t header_len,
                                void *data, size_t data_len)
{
    auto *backend = (AP_Ranging_DW1000 *)dwGetUserdata(dev);
    if (backend == nullptr || !backend->_dev) {
        return;
    }
    if (!backend->_dev->transfer((const uint8_t *)header, header_len, (uint8_t *)data, data_len)) {
        backend->_diag.spi_error++;
        memset(data, 0, data_len);
    }
}

void AP_Ranging_DW1000::spiWrite(dwDevice_t *dev, const void *header, size_t header_len, const void *data, size_t data_len)
{
    auto *backend = (AP_Ranging_DW1000 *)dwGetUserdata(dev);
    if (backend == nullptr || !backend->_dev) {
        return;
    }
    if (header_len + data_len > sizeof(backend->_tx_scratch)) {
        return;
    }
    memcpy(backend->_tx_scratch, header, header_len);
    if (data_len > 0) {
        memcpy(backend->_tx_scratch + header_len, data, data_len);
    }
    if (!backend->_dev->transfer(backend->_tx_scratch, header_len + data_len, nullptr, 0)) {
        backend->_diag.spi_error++;
    }
}

void AP_Ranging_DW1000::spiSetSpeed(dwDevice_t *dev, dwSpiSpeed_t speed)
{
    auto *backend = (AP_Ranging_DW1000 *)dwGetUserdata(dev);
    if (backend == nullptr || !backend->_dev) {
        return;
    }
    backend->_dev->set_speed(speed == dwSpiSpeedHigh ? AP_HAL::Device::SPEED_HIGH : AP_HAL::Device::SPEED_LOW);
}

void AP_Ranging_DW1000::delayms(dwDevice_t *dev, unsigned int delay)
{
    hal.scheduler->delay(delay);
}

#endif  // AP_RANGING_DW1000_ENABLED
