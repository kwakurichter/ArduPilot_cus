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

// Loco deck RSTn / IRQ GPIO pin numbers. Must match the GPIO(n) numbers given to DW1000_RESET / DW1000_IRQ in the board hwdef
#ifndef HAL_DW1000_RESET_PIN
#define HAL_DW1000_RESET_PIN 61
#endif
#ifndef HAL_DW1000_IRQ_PIN
#define HAL_DW1000_IRQ_PIN 60
#endif

#include "twr_protocol.h"
#if AP_RANGING_RF_DIAGNOSTICS_ENABLED
#include "twr_radio_diag.h"
#endif

class AP_Ranging_DW1000 : public AP_Ranging_Backend
{
public:
    AP_Ranging_DW1000(AP_Ranging &frontend);
    bool healthy() override;
    void update() override;

private:
    bool init_device();
    bool configure_radio();
    void timer();
    void arm_receiver();
    void service_radio();
    void send_packet();
    void abort_exchange();
    uint8_t next_poll_target();
    uint16_t random_below(uint16_t limit);
    bool admitted_peer(uint8_t id) const;
    void publish_range(uint64_t rx_timestamp);
    static void handle_sent(dwDevice_t *dev);
    static void handle_received(dwDevice_t *dev);
    static void handle_rx_timeout(dwDevice_t *dev);
    static void handle_rx_failed(dwDevice_t *dev);

    twr_engine _engine {};
    AP_Ranging::Mode _mode = AP_Ranging::Mode::Peer;
    uint8_t _targets[AP_RANGING_MAX_ANCHORS] {};
    uint8_t _target_count = 0;
    uint8_t _next_target = 0;
    uint32_t _target_time_ms[AP_RANGING_MAX_ANCHORS] {};
    uint16_t _target_wait_ms[AP_RANGING_MAX_ANCHORS] {};
    uint8_t _target_failures[AP_RANGING_MAX_ANCHORS] {};
    uint16_t _next_sequence = 0;
    uint32_t _session = 0;
    uint32_t _random_state = 1;
    uint16_t _poll_ms = 0;
    uint16_t _reply_us = 0;
    uint32_t _last_poll_ms = 0;
    uint32_t _poll_interval = 0;

#if AP_RANGING_RF_DIAGNOSTICS_ENABLED
    uint8_t _rf_mask = 0;
    bool _rf_active = false;
    twr_diag_rx _rf_rx[3] {};
    uint32_t _rf_capture_us = 0;
    static constexpr uint8_t FRAME_SIZE = TWR_DIAG_MAX_FRAME_SIZE;
#else
    static constexpr uint8_t FRAME_SIZE = TWR_MAX_FRAME_SIZE;
#endif
    struct Sample {
        float distance;
        uint32_t time_ms;
        uint32_t session;
        uint16_t sequence;
        uint8_t remote_id;
        uint8_t initiator_id;
        bool initiated;
#if AP_RANGING_RF_DIAGNOSTICS_ENABLED
        bool rf_requested;
        bool rf_valid;
        twr_diag_rx rf_rx[3];
        uint32_t rf_capture_us;
        float uncorrected;
#endif
    };
    static constexpr uint8_t QUEUE_SIZE = 16;
    Sample _queue[QUEUE_SIZE] {};
    uint8_t _queue_head = 0;
    uint8_t _queue_count = 0;
    HAL_Semaphore _queue_sem;
    uint32_t _queue_drops = 0;
    uint32_t _exchange_fail = 0;

    // Written on the SPI thread; the main loop consumes a bounded snapshot.
    struct Diagnostics {
        uint32_t time_ms;
        uint32_t failures, tx_state, tx_mismatch, expired, rx_error, rx_timeout;
        uint32_t spi_error, queue_drops, gap_us, gap_max_us, work_max_us;
        uint64_t tx_time_us, expected_tx, actual_tx;
        uint32_t tx_gap_us;
        uint16_t tx_sequence;
        uint8_t tx_remote, tx_state_before;
    } _diag {}, _diag_snapshot {};
    uint32_t _last_callback_us = 0;
    uint32_t _callback_gap_us = 0;
    uint32_t _last_diag_ms = 0;
    uint64_t _last_tx_diag_us = 0;
    bool _diag_ready = false;

    // ---- libdw1000 hardware ops (C callbacks) ----
    // Recover the backend via dwGetUserdata(); assume the bus semaphore is held.
    static void spiRead(dwDevice_t *dev, const void *header, size_t header_len, void *data, size_t data_len);
    static void spiWrite(dwDevice_t *dev, const void *header, size_t header_len, const void *data, size_t data_len);
    static void spiSetSpeed(dwDevice_t *dev, dwSpiSpeed_t speed);
    static void delayms(dwDevice_t *dev, unsigned int delay);

    AP_HAL::OwnPtr<AP_HAL::SPIDevice> _dev;

    dwDevice_t _dw;     // libdw1000 device context
    dwOps_t    _ops;    // hardware op function pointers handed to libdw1000

    bool     _initialised = false;
    uint32_t _last_update_ms = 0;  // last successful range (drives healthy())

    bool _have_range = false;

    // scratch buffer for combined header+payload SPI writes (single CS transaction)
    static constexpr uint16_t TX_SCRATCH_LEN = 1024 + 3;
    uint8_t _tx_scratch[TX_SCRATCH_LEN];
};

#endif  // AP_RANGING_DW1000_ENABLED
