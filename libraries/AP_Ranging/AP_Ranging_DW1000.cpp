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

    {
        // the ops assume the caller holds the bus semaphore
        WITH_SEMAPHORE(_dev->get_semaphore());

        // configure at low speed for reliable register access
        _dev->set_speed(AP_HAL::Device::SPEED_LOW);

        if (dwGetDeviceId(&_dw) != AP_RANGING_DW1000_DEVICE_ID) {
            // wrong id - deck not present
            return false;
        }

        if (dwConfigure(&_dw) != 0) {
            return false;
        }

        // TODO: select an operating mode (e.g. MODE_LONGDATA_RANGE_LOWPOWER),
        //       set the antenna delay, install handleReceived/handleSent
        //       handlers, and start the receiver for the TWR exchange.
    }

    // drive the TWR state machine from the SPI bus thread (100Hz placeholder)
    _dev->register_periodic_callback(10000, FUNCTOR_BIND_MEMBER(&AP_Ranging_DW1000::timer, void));

    _initialised = true;
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

// periodic callback on the SPI bus thread (bus semaphore is held here)
void AP_Ranging_DW1000::timer()
{
    // TODO: implement the TWR state machine:
    //   - poll dwReadSystemEventStatusRegister() / service RX/TX events
    //   - run the poll -> response -> final message exchange with each peer
    //   - compute Time-of-Flight -> range, then publish it:
    //
    //       set_node_distance(node_index, range_m);
    //       _last_update_ms = AP_HAL::millis();
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
