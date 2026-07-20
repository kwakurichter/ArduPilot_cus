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

#include "AP_Ranging_Backend.h"

#if AP_RANGING_DW1000_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/SPIDevice.h>

// libdw1000 is a C driver (Bitcraze) tracked as a submodule at modules/libdw1000
extern "C" {
#include <libdw1000.h>
}

// DW1000 expected chip id returned by dwGetDeviceId()
#define AP_RANGING_DW1000_DEVICE_ID 0xDECA0130UL

class AP_Ranging_DW1000 : public AP_Ranging_Backend
{
public:
    AP_Ranging_DW1000(AP_Ranging &frontend);

    bool healthy() override;
    void update() override;

private:
    // acquire the SPI device, run dwInit/dwConfigure, verify the chip id.
    // returns true on success. Runs once from the main thread.
    bool init_device();

    // periodic callback (SPI bus thread) - drives the TWR state machine
    void timer();

    // ---- libdw1000 hardware ops (C callbacks) ----
    // These recover the owning backend instance via dwGetUserdata() so they can reach the AP_HAL SPI device. They assume the bus semaphore is
    // already held by the caller (init_device() holds it; the periodic callback runs on the bus thread which holds it).
    static void spiRead(dwDevice_t *dev, const void *header, size_t header_len, void *data, size_t data_len);
    static void spiWrite(dwDevice_t *dev, const void *header, size_t header_len, const void *data, size_t data_len);
    static void spiSetSpeed(dwDevice_t *dev, dwSpiSpeed_t speed);
    static void delayms(dwDevice_t *dev, unsigned int delay);
    static void reset(dwDevice_t *dev);

    AP_HAL::OwnPtr<AP_HAL::SPIDevice> _dev;

    dwDevice_t _dw;     // libdw1000 device context
    dwOps_t    _ops;    // hardware op function pointers handed to libdw1000

    bool     _initialised;
    uint32_t _last_update_ms;

    // scratch buffer for combined header+payload SPI writes (single CS transaction). DW1000 max frame is 1024 bytes (SPI header is up to 3)
    static constexpr uint16_t TX_SCRATCH_LEN = 1024 + 3;
    uint8_t _tx_scratch[TX_SCRATCH_LEN];
};

#endif  // AP_RANGING_DW1000_ENABLED
