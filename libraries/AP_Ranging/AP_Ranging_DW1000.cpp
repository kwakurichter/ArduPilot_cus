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

extern const AP_HAL::HAL &hal;

AP_Ranging_DW1000::AP_Ranging_DW1000(AP_Ranging &frontend) : AP_Ranging_Backend(frontend)
{
    // attempt init now; healthy() reports the outcome
    init_device();
}

// acquire the SPI device, initialise libdw1000 and verify the chip id.
bool AP_Ranging_DW1000::init_device()
{
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

      The crazyflie2 hwdefs currently comment out DW1000_IRQ to give USART3
      back, so this is the expected path there.
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
    _ops.reset       = nullptr;   // SPI soft-reset (no reset GPIO wired)

    dwInit(&_dw, &_ops);
    dwSetUserdata(&_dw, this);     // lets the C ops/handlers recover 'this'

    // (Optional) hardware reset - requires a GPIO wired to the DW1000 RSTn pin.
    //hal.gpio->pinMode(HAL_DW1000_RESET_PIN, HAL_GPIO_OUTPUT);
    //hal.gpio->write(HAL_DW1000_RESET_PIN, 0);
    //hal.scheduler->delay(2);
    //hal.gpio->pinMode(HAL_DW1000_RESET_PIN, HAL_GPIO_INPUT);
    //hal.scheduler->delay(5);

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

        _node_id   = get_node_id();
        _num_nodes = get_num_nodes();
        _next_peer = _node_id;   // round-robin starts just past us
        configure_radio();
        arm_receiver();          // start listening
    }

    // service radio + drive the exchange from the SPI bus thread (300Hz)
    _dev->register_periodic_callback(300, FUNCTOR_BIND_MEMBER(&AP_Ranging_DW1000::timer, void));

    _initialised = true;
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "DW1000: ready (node %u of %u)", (unsigned)_node_id, (unsigned)_num_nodes);
    return true;
}

// healthy if we came up and have computed a range recently
bool AP_Ranging_DW1000::healthy()
{
    return _initialised && (AP_HAL::millis() - _last_update_ms < AP_RANGING_TIMEOUT_MS);
}

// main loop hook
void AP_Ranging_DW1000::update()
{
    // nothing to do here - ranges are published from compute_range()
}

// periodic bus-thread callback (~1kHz). Services radio events, times out stalled exchanges, and round-robin polls neighbours when idle.
void AP_Ranging_DW1000::timer()
{
    service_radio();

    const uint32_t now = AP_HAL::millis();

    // abort an exchange that stalled (lost packet / busy peer)
    if (_state != State::IDLE && (now - _exchange_start_ms) > get_xchg_ms()) {
        abort_exchange();
    }

    // when idle, start a new poll to the next neighbour on a JITTERED cadence.
    // The jitter is essential: two nodes on the same fixed period boot in phase
    // and livelock (each polls while the other is mid-poll (not IDLE)), so both
    // drop the incoming POLL and time out, forever. Random jitter drifts them
    // apart so one is usually IDLE when the other polls.
    if (_state == State::IDLE && (now - _last_poll_ms) >= _poll_interval) {
        _last_poll_ms = now;
        _poll_interval = get_poll_ms() + (get_random16() % POLL_JITTER_MS);
        const uint8_t target = next_poll_target();
        if (target != _node_id) {
            start_poll(target);
        }
    }

    // debug report, gated behind RNG_DEBUG (runtime toggle)
    if (get_debug() > 0 && (now - _last_report_ms >= LINK_REPORT_MS)) {
        _last_report_ms = now;
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "DW1000: rng=%lu rx=%lu tx=%lu irq=%lu",
                      (unsigned long)_range_count, (unsigned long)_rx_count,
                      (unsigned long)_tx_count, (unsigned long)_irq_count);
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "DW1000: xfail=%u rxfail=%u | %u=%.2fm",
                      (unsigned)_exchange_fail, (unsigned)_rx_failed,
                      (unsigned)_last_range_peer, (double)_last_range);
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
        _irq_count++;
        dwHandleInterrupt(&_dw);
    }
}

// RNG_REPLY_US -> DW1000 device ticks. TIME_RES is microseconds per tick.
uint64_t AP_Ranging_DW1000::reply_delay_ticks() const
{
    return (uint64_t)((double)get_reply_us() / TIME_RES);
}

// round-robin the next neighbour id in [0, _num_nodes), skipping our own id.
// returns _node_id if there is no other node to poll.
uint8_t AP_Ranging_DW1000::next_poll_target()
{
    if (_num_nodes <= 1) {
        return _node_id;
    }
    for (uint8_t i = 0; i < _num_nodes; i++) {
        _next_peer = (_next_peer + 1) % _num_nodes;
        if (_next_peer != _node_id) {
            return _next_peer;
        }
    }
    return _node_id;
}

// initiator: send POLL (delayed tx so poll_tx is known immediately), then wait4resp so the receiver arms for the RESPONSE.
void AP_Ranging_DW1000::start_poll(uint8_t dst)
{
    _peer = dst;
    _seq++;

    dwNewTransmit(&_dw);
    dwTime_t delay = {}; delay.full = reply_delay_ticks();
    const dwTime_t tx = dwSetDelay(&_dw, &delay);
    _poll_tx = tx.full & TS_MASK;

    const uint8_t f[FRAME_HDR_LEN] = { FRAME_POLL, _node_id, dst, _seq };
    dwSetData(&_dw, (uint8_t *)f, FRAME_HDR_LEN);
    dwWaitForResponse(&_dw, true);
    dwStartTransmit(&_dw);
    _tx_count++;

    _state = State::I_WAIT_RESP;
    _exchange_start_ms = AP_HAL::millis();
}

// responder: reply to a POLL with a delayed RESPONSE, wait4resp for the FINAL.
void AP_Ranging_DW1000::send_response(uint8_t dst)
{
    dwNewTransmit(&_dw);
    dwTime_t delay = {}; delay.full = reply_delay_ticks();
    const dwTime_t tx = dwSetDelay(&_dw, &delay);
    _resp_tx = tx.full & TS_MASK;

    const uint8_t f[FRAME_HDR_LEN] = { FRAME_RESPONSE, _node_id, dst, _seq };
    dwSetData(&_dw, (uint8_t *)f, FRAME_HDR_LEN);
    dwWaitForResponse(&_dw, true);
    dwStartTransmit(&_dw);
    _tx_count++;

    _state = State::R_WAIT_FINAL;
    _exchange_start_ms = AP_HAL::millis();
}

// initiator: FINAL carries our three timestamps so the responder can range.
// The delayed tx time (final_tx) is read from dwSetDelay before we build the
// payload, so it can be embedded.
void AP_Ranging_DW1000::send_final(uint8_t dst)
{
    dwNewTransmit(&_dw);
    dwTime_t delay = {}; delay.full = reply_delay_ticks();
    const dwTime_t tx = dwSetDelay(&_dw, &delay);
    _final_tx = tx.full & TS_MASK;

    uint8_t f[FINAL_LEN] = { FRAME_FINAL, _node_id, dst, _seq };
    ts_pack(&f[FRAME_HDR_LEN + 0 * TS_LEN], _poll_tx);
    ts_pack(&f[FRAME_HDR_LEN + 1 * TS_LEN], _resp_rx);
    ts_pack(&f[FRAME_HDR_LEN + 2 * TS_LEN], _final_tx);
    dwSetData(&_dw, f, FINAL_LEN);
    dwWaitForResponse(&_dw, false);   // initiator is done after the FINAL
    dwStartTransmit(&_dw);
    _tx_count++;

    _state = State::I_SENDING_FINAL;
}

// responder: apply Alternative DS-TWR (eq. 17) and publish the range.
void AP_Ranging_DW1000::compute_range()
{
    const uint64_t Ra = (_resp_rx  - _poll_tx) & TS_MASK;  // A round-trip
    const uint64_t Da = (_final_tx - _resp_rx) & TS_MASK;  // A reply delay
    const uint64_t Db = (_resp_tx  - _poll_rx) & TS_MASK;  // B reply delay
    const uint64_t Rb = (_final_rx - _resp_tx) & TS_MASK;  // B round-trip

    const uint64_t den = Ra + Da + Rb + Db;
    if (den == 0) {
        return;
    }
    const int64_t num = (int64_t)(Ra * Rb) - (int64_t)(Da * Db);
    const double tof_ticks = (double)num / (double)den;
    const float dist = (float)(tof_ticks * DISTANCE_OF_RADIO);

    if (dist <= RANGE_MIN_M || dist >= RANGE_MAX_M) {
        return;   // reject nonsense (bad exchange / clock glitch)
    } // TODO: pre-filter?

    set_node_distance(_peer, dist);
    _range_count++;
    _last_range = dist;
    _last_range_peer = _peer;
    _last_update_ms = AP_HAL::millis();
}

// give up on the in progress exchange and go back to listening
void AP_Ranging_DW1000::abort_exchange()
{
    _exchange_fail++;
    _state = State::IDLE;
    arm_receiver();
}

// ---- 40-bit timestamp (de)serialisation ----
void AP_Ranging_DW1000::ts_pack(uint8_t *dst, uint64_t v)
{
    for (uint8_t i = 0; i < TS_LEN; i++) {
        dst[i] = (uint8_t)((v >> (8 * i)) & 0xFF);
    }
}

uint64_t AP_Ranging_DW1000::ts_unpack(const uint8_t *src)
{
    uint64_t v = 0;
    for (uint8_t i = 0; i < TS_LEN; i++) {
        v |= (uint64_t)src[i] << (8 * i);
    }
    return v;
}

// ---- libdw1000 event handlers (bus thread, semaphore held) ----

void AP_Ranging_DW1000::handle_sent(dwDevice_t *dev)
{
    auto *b = (AP_Ranging_DW1000 *)dwGetUserdata(dev);
    if (b == nullptr) {
        return;
    }
    // POLL/RESPONSE sends auto-armed RX via wait4resp; only the FINAL (which
    // ends the initiator's job) needs us to resume plain listening.
    if (b->_state == State::I_SENDING_FINAL) {
        b->_state = State::IDLE;
        b->arm_receiver();
    }
}

void AP_Ranging_DW1000::handle_received(dwDevice_t *dev)
{
    auto *b = (AP_Ranging_DW1000 *)dwGetUserdata(dev);
    if (b == nullptr) {
        return;
    }

    uint8_t buf[RX_BUF_LEN];
    const unsigned int len = dwGetDataLength(dev);
    if (len < FRAME_HDR_LEN || len > sizeof(buf)) {
        b->arm_receiver();
        return;
    }
    dwGetData(dev, buf, len);
    b->_rx_count++;

    const uint8_t type = buf[0];
    const uint8_t src  = buf[1];
    const uint8_t dst  = buf[2];
    const uint8_t seq  = buf[3];

    // ignore frames not addressed to us, keep listening
    if (dst != b->_node_id) {
        b->arm_receiver();
        return;
    }

    dwTime_t rx = {};
    dwGetReceiveTimestamp(dev, &rx);

    switch (type) {
    case FRAME_POLL:
        if (b->_state == State::IDLE) {
            b->_peer = src;
            b->_seq  = seq;
            b->_poll_rx = rx.full & TS_MASK;
            b->send_response(src);   // wait4resp arms RX for the FINAL
        } else {
            b->arm_receiver();       // busy: drop, keep listening
        }
        break;

    case FRAME_RESPONSE:
        if (b->_state == State::I_WAIT_RESP && src == b->_peer && seq == b->_seq) {
            b->_resp_rx = rx.full & TS_MASK;
            b->send_final(src);      // -> I_SENDING_FINAL, completes in handle_sent
        } else {
            b->arm_receiver();
        }
        break;

    case FRAME_FINAL:
        if (b->_state == State::R_WAIT_FINAL && src == b->_peer && seq == b->_seq &&
            len >= FINAL_LEN) {
            b->_final_rx = rx.full & TS_MASK;
            b->_poll_tx  = ts_unpack(&buf[FRAME_HDR_LEN + 0 * TS_LEN]);
            b->_resp_rx  = ts_unpack(&buf[FRAME_HDR_LEN + 1 * TS_LEN]);
            b->_final_tx = ts_unpack(&buf[FRAME_HDR_LEN + 2 * TS_LEN]);
            b->compute_range();
            b->_state = State::IDLE;
        }
        b->arm_receiver();           // exchange done (or bad) -> listen again
        break;

    default:
        b->arm_receiver();
        break;
    }
}

void AP_Ranging_DW1000::handle_rx_timeout(dwDevice_t *dev)
{
    auto *b = (AP_Ranging_DW1000 *)dwGetUserdata(dev);
    if (b != nullptr) {
        b->abort_exchange();
    }
}

void AP_Ranging_DW1000::handle_rx_failed(dwDevice_t *dev)
{
    auto *b = (AP_Ranging_DW1000 *)dwGetUserdata(dev);
    if (b != nullptr) {
        b->_rx_failed++;
        // a corrupted frame doesn't end the exchange; keep waiting (the software
        // timeout in timer() gives up if the good frame never arrives)
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
    backend->_dev->transfer((const uint8_t *)header, header_len, (uint8_t *)data, data_len);
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
    backend->_dev->transfer(backend->_tx_scratch, header_len + data_len, nullptr, 0);
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

void AP_Ranging_DW1000::reset(dwDevice_t *dev)
{
    // optional hardware reset - not wired (_ops.reset is nullptr -> SPI soft-reset)
}

#endif  // AP_RANGING_DW1000_ENABLED
