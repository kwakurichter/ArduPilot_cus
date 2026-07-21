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

    // hand our hardware ops to libdw1000. dwInit() only touches the device struct (no bus traffic), so it needs no semaphore.
    _ops.spiRead     = &AP_Ranging_DW1000::spiRead;
    _ops.spiWrite    = &AP_Ranging_DW1000::spiWrite;
    _ops.spiSetSpeed = &AP_Ranging_DW1000::spiSetSpeed;
    _ops.delayms     = &AP_Ranging_DW1000::delayms;
    _ops.reset       = nullptr;   // use SPI soft reset until a reset GPIO is wired (TODO)

    dwInit(&_dw, &_ops);
    dwSetUserdata(&_dw, this);    // lets the C ops recover 'this'

    // (Optional) Hardware reset the DW1000 before any SPI access. Requires a GPIO to be wired to the DW1000 RSTn pin.
    //hal.gpio->pinMode(HAL_DW1000_RESET_PIN, HAL_GPIO_OUTPUT);
    //hal.gpio->write(HAL_DW1000_RESET_PIN, 0);       // assert reset
    //hal.scheduler->delay(2);                        // hold low (>=1ms)
    //hal.gpio->pinMode(HAL_DW1000_RESET_PIN, HAL_GPIO_INPUT);  // release (pull-up -> high)
    //hal.scheduler->delay(5);                        // let the chip boot to IDLE

    {
        // the ops assume the caller holds the bus semaphore
        WITH_SEMAPHORE(_dev->get_semaphore());

        // configure at low speed for reliable register access
        _dev->set_speed(AP_HAL::Device::SPEED_LOW);

        const uint32_t id = dwGetDeviceId(&_dw);
        if (id != AP_RANGING_DW1000_DEVICE_ID) {
            // wrong id - deck not present or SPI incorrectly wired
            GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "DW1000: not detected (id=0x%08x)", (unsigned)id);
            return false;
        }

        if (dwConfigure(&_dw) != 0) {
            GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "DW1000: configure failed");
            return false;
        }

        // cache our node address and bring up the radio (RF mode + handlers), then start listening
        _node_id = get_node_id();
        configure_radio();
        arm_receiver();
    }

    // service radio events + drive TX/ranging from the SPI bus thread (1kHz)
    // We poll here rather than using a hardware interrupt (see service_radio())
    _dev->register_periodic_callback(1000, FUNCTOR_BIND_MEMBER(&AP_Ranging_DW1000::timer, void));

    _initialised = true;
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "DW1000: detected and configured (node %u)", (unsigned)_node_id);
    return true;
}

// return true if we have a recent range and the device came up
bool AP_Ranging_DW1000::healthy()
{
    return _initialised && (AP_HAL::millis() - _last_update_ms < AP_RANGING_TIMEOUT_MS);
}

// called from the vehicle main loop. The main TWR happens in timer() on the bus thread; nothing to do here yet
void AP_Ranging_DW1000::update()
{
    // TODO: post-process / filter ranges accumulated by timer()
}

// periodic callback on the SPI bus thread (bus semaphore is held here), ~1kHz
void AP_Ranging_DW1000::timer()
{
    // 1) service any radio event (received/sent/timeout) by polling (no IRQ)
    service_radio();

    const uint32_t now = AP_HAL::millis();

    // 2) broadcast our heartbeat periodically so peer can hear us
    if (now - _last_tx_ms >= HEARTBEAT_PERIOD_MS) {
        _last_tx_ms = now;
        send_heartbeat();
    }

    // 3) DEBUG: report link status over MAVLink
    if (now - _last_report_ms >= LINK_REPORT_MS) {
        _last_report_ms = now;
        GCS_SEND_TEXT(MAV_SEVERITY_INFO,
                      "DW1000 link: rx=%lu last src=%u seq=%u pwr=%.1fdBm",
                      (unsigned long)_rx_count, (unsigned)_rx_last_src,
                      (unsigned)_rx_last_seq, (double)_rx_last_power);
    }
}

// configure the RF settings and attach event handlers
bool AP_Ranging_DW1000::configure_radio()
{
    // event handlers - invoked from dwHandleInterrupt() on the bus thread
    dwAttachSentHandler(&_dw, &AP_Ranging_DW1000::handle_sent);
    dwAttachReceivedHandler(&_dw, &AP_Ranging_DW1000::handle_received);
    dwAttachReceiveTimeoutHandler(&_dw, &AP_Ranging_DW1000::handle_rx_timeout);
    dwAttachReceiveFailedHandler(&_dw, &AP_Ranging_DW1000::handle_rx_failed);

    // RF settings (mirror the Bitcraze Loco deck defaults)
    // TODO: Performance tuning: mode, channel, preamble length/code, antenna delay, etc.
    dwNewConfiguration(&_dw);
    dwSetDefaults(&_dw);
    dwEnableMode(&_dw, MODE_SHORTDATA_FAST_ACCURACY);
    dwSetChannel(&_dw, CHANNEL_2);
    dwSetPreambleCode(&_dw, PREAMBLE_CODE_64MHZ_9);
    dwUseSmartPower(&_dw, true);

    dwTime_t antenna_delay = {};      // 0 for now; TODO: calibration
    dwSetAntenaDelay(&_dw, antenna_delay);

    dwCommitConfiguration(&_dw);
    return true;
}

// put the radio into continuous receive
void AP_Ranging_DW1000::arm_receiver()
{
    dwIdle(&_dw);
    dwNewReceive(&_dw);
    dwSetDefaults(&_dw);
    dwReceivePermanently(&_dw, true);   // auto rearm after each reception
    dwStartReceive(&_dw);
}

// broadcast a heartbeat: [type, src, seq]
void AP_Ranging_DW1000::send_heartbeat()
{
    uint8_t frame[HEARTBEAT_LEN] = { FRAME_TYPE_HEARTBEAT, _node_id, _tx_seq++ };
    dwIdle(&_dw);
    dwNewTransmit(&_dw);
    dwSetDefaults(&_dw);
    dwSetData(&_dw, frame, HEARTBEAT_LEN);
    dwStartTransmit(&_dw);
    // RX is rearmed in handle_sent() once the frame is on air
}

// poll for and dispatch any pending radio event. dwHandleInterrupt() reads the DW1000 status register over SPI and calls our attached handlers (no IRQ)
void AP_Ranging_DW1000::service_radio()
{
    dwHandleInterrupt(&_dw);
}

// ---- libdw1000 event handlers (bus thread, semaphore held) ----

void AP_Ranging_DW1000::handle_sent(dwDevice_t *dev)
{
    auto *b = (AP_Ranging_DW1000 *)dwGetUserdata(dev);
    if (b == nullptr) {
        return;
    }
    // transmit finished - go back to listening
    b->arm_receiver();
}

void AP_Ranging_DW1000::handle_received(dwDevice_t *dev)
{
    auto *b = (AP_Ranging_DW1000 *)dwGetUserdata(dev);
    if (b == nullptr) {
        return;
    }

    unsigned int len = dwGetDataLength(dev);
    if (len >= HEARTBEAT_LEN) {
        uint8_t frame[HEARTBEAT_LEN];
        dwGetData(dev, frame, HEARTBEAT_LEN);
        if (frame[0] == FRAME_TYPE_HEARTBEAT && frame[1] != b->_node_id) {
            b->_rx_count++;
            b->_rx_last_src   = frame[1];
            b->_rx_last_seq   = frame[2];
            b->_rx_last_power = dwGetReceivePower(dev);
        }
    } // TODO: add TWR frame handling here
    // permanent receive auto rearms; no explicit arm needed here
}

// TODO: add failure counters
void AP_Ranging_DW1000::handle_rx_timeout(dwDevice_t *dev)
{
    auto *b = (AP_Ranging_DW1000 *)dwGetUserdata(dev);
    if (b != nullptr) {
        b->arm_receiver();
    }
}

// TODO: add failure counters
void AP_Ranging_DW1000::handle_rx_failed(dwDevice_t *dev)
{
    auto *b = (AP_Ranging_DW1000 *)dwGetUserdata(dev);
    if (b != nullptr) {
        b->arm_receiver();
    }
}

// ---------------------------------------------------------------------------------------------------------------------------------
// libdw1000 hardware ops. Each recovers the backend via dwGetUserdata() and assumes the bus semaphore is already held by the caller
// ---------------------------------------------------------------------------------------------------------------------------------

void AP_Ranging_DW1000::spiRead(dwDevice_t *dev, const void *header, size_t header_len, void *data, size_t data_len)
{
    auto *backend = (AP_Ranging_DW1000 *)dwGetUserdata(dev);
    if (backend == nullptr || !backend->_dev) {
        return;
    }
    // send the header, then clock in data_len bytes (single CS transaction)
    backend->_dev->transfer((const uint8_t *)header, header_len, (uint8_t *)data, data_len);
}

void AP_Ranging_DW1000::spiWrite(dwDevice_t *dev, const void *header, size_t header_len, const void *data, size_t data_len)
{
    auto *backend = (AP_Ranging_DW1000 *)dwGetUserdata(dev);
    if (backend == nullptr || !backend->_dev) {
        return;
    }
    if (header_len + data_len > sizeof(backend->_tx_scratch)) {
        return;   // oversized write - should not happen for DW1000 frames
    }
    // AP_HAL SPI transfer takes a single send buffer, so concatenate header and payload to keep them within one CS assertion.
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
    // TODO: hardware reset (optional, not currently wired): _ops.reset is nullptr so libdw1000 uses dwSoftReset() over SPI. Implement here (toggle a reset GPIO) and
    // set _ops.reset = &AP_Ranging_DW1000::reset if a reset line is added.
}

#endif  // AP_RANGING_DW1000_ENABLED
