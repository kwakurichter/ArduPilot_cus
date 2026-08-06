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

// Alternative Double-Sided Two-Way Ranging (Decawave APS013, eq. 17):
//   Tf = (Ra*Rb - Da*Db) / (Ra + Da + Rb + Db)
// This form tolerates arbitrary/asymmetric reply delays, so we can use generous
// reply delays that comfortably exceed the ~1ms software polling latency.
//
// Every node both initiates (round-robin polls to all neighbours) and responds.
// In a 3-message exchange the RESPONDER receives last and holds all six
// timestamps, so it computes and stores the range - therefore a node's ranges
// come from responding to its neighbours' polls (no report needed).
//
// Only ONE exchange runs at a time (single half-duplex radio): the node is
// listening when IDLE, and is either initiating or responding otherwise.

class AP_Ranging_DW1000 : public AP_Ranging_Backend
{
public:
    AP_Ranging_DW1000(AP_Ranging &frontend);

    bool healthy() override;
    void update() override;

private:
    bool init_device();
    void timer();               // periodic bus-thread callback (~1kHz)

    // radio setup
    bool configure_radio();     // mode/channel/preamble + attach handlers, commit
    void arm_receiver();        // plain (non-permanent) receive = "listening"
    void service_radio();       // poll dwHandleInterrupt (no hardware interrupt)

    // ---- Alternative DS-TWR exchange ----
    void start_poll(uint8_t dst);   // initiator: POLL (delayed tx)
    void send_response(uint8_t dst);// responder: RESPONSE (delayed tx)
    void send_final(uint8_t dst);   // initiator: FINAL carrying our 3 timestamps
    void compute_range();           // responder: eq(17) -> distance -> publish
    void abort_exchange();          // count a failure, return to IDLE + listen
    uint8_t next_poll_target();     // round-robin neighbour id (skips self)

    // libdw1000 event handlers (recover 'this' via dwGetUserdata)
    static void handle_sent(dwDevice_t *dev);
    static void handle_received(dwDevice_t *dev);
    static void handle_rx_timeout(dwDevice_t *dev);
    static void handle_rx_failed(dwDevice_t *dev);

    // 40-bit timestamp (de)serialisation (little-endian, 5 bytes)
    static void     ts_pack(uint8_t *dst, uint64_t v);
    static uint64_t ts_unpack(const uint8_t *src);
    static constexpr uint64_t TS_MASK = 0xFFFFFFFFFFULL;   // 40-bit

    // frame layout: [type, src, dst, seq, <payload>]
    enum : uint8_t { FRAME_POLL = 0xC1, FRAME_RESPONSE = 0xC2, FRAME_FINAL = 0xC3 };
    static constexpr uint8_t FRAME_HDR_LEN = 4;
    static constexpr uint8_t TS_LEN        = 5;
    static constexpr uint8_t FINAL_LEN     = FRAME_HDR_LEN + 3 * TS_LEN;  // 19
    static constexpr uint8_t RX_BUF_LEN    = 32;

    // exchange state (single exchange at a time)
    enum class State : uint8_t {
        IDLE,             // listening
        I_WAIT_RESP,      // initiator: POLL sent, awaiting RESPONSE
        I_SENDING_FINAL,  // initiator: FINAL queued, awaiting sent event
        R_WAIT_FINAL,     // responder: RESPONSE sent, awaiting FINAL
    };
    State    _state = State::IDLE;
    uint8_t  _peer = 0;               // node id of the in progress exchange
    uint8_t  _seq = 0;                // sequence of the in progress exchange
    uint32_t _exchange_start_ms = 0;  // for the software exchange timeout

    // timestamps for the in progress exchange (device ticks, 40-bit)
    // initiator captures: poll_tx, resp_rx, final_tx
    // responder captures: poll_rx, resp_tx, final_rx (+ the 3 above from FINAL)
    uint64_t _poll_tx = 0, _resp_rx = 0, _final_tx = 0;
    uint64_t _poll_rx = 0, _resp_tx = 0, _final_rx = 0;

    // round-robin initiator scheduler
    uint8_t  _next_peer = 0;
    uint32_t _last_poll_ms = 0;
    uint32_t _poll_interval = 50;  // current (jittered) poll interval; from RNG_POLL_MS

    // cached config
    uint8_t  _node_id = 0;
    uint8_t  _num_nodes = 2;

    // Tuning now comes from params (RNG_POLL_MS / _REPLY_US / _XCHG_MS / _CHAN)
    // via the get_*() accessors. Reply delay is converted to device ticks here:
    uint64_t reply_delay_ticks() const;   // RNG_REPLY_US -> DW1000 ticks

    static constexpr uint32_t POLL_JITTER_MS = 50;   // random jitter to de-sync nodes
    static constexpr uint32_t LINK_REPORT_MS = 5000; // GCS debug cadence
    static constexpr float    RANGE_MIN_M = -1.0f;   // range sanity gate
    static constexpr float    RANGE_MAX_M = 1000.0f;

    // counters / diagnostics (bus thread only)
    uint32_t _rx_count = 0;        // frames received
    uint32_t _tx_count = 0;        // frames transmitted
    uint16_t _rx_failed = 0;       // RX CRC/PHY errors
    uint16_t _exchange_fail = 0;   // exchanges that timed out
    uint32_t _range_count = 0;     // ranges successfully computed
    float    _last_range = 0.0f;   // last computed range (m)
    uint8_t  _last_range_peer = 0; // peer of the last computed range
    uint32_t _irq_count = 0;       // times the DW1000 IRQ line was seen asserted
    uint32_t _last_report_ms = 0;

    // ---- libdw1000 hardware ops (C callbacks) ----
    // Recover the backend via dwGetUserdata(); assume the bus semaphore is held.
    static void spiRead(dwDevice_t *dev, const void *header, size_t header_len, void *data, size_t data_len);
    static void spiWrite(dwDevice_t *dev, const void *header, size_t header_len, const void *data, size_t data_len);
    static void spiSetSpeed(dwDevice_t *dev, dwSpiSpeed_t speed);
    static void delayms(dwDevice_t *dev, unsigned int delay);
    static void reset(dwDevice_t *dev);

    AP_HAL::OwnPtr<AP_HAL::SPIDevice> _dev;

    dwDevice_t _dw;     // libdw1000 device context
    dwOps_t    _ops;    // hardware op function pointers handed to libdw1000

    bool     _initialised = false;
    uint32_t _last_update_ms = 0;  // last successful range (drives healthy())

    // scratch buffer for combined header+payload SPI writes (single CS transaction)
    static constexpr uint16_t TX_SCRATCH_LEN = 1024 + 3;
    uint8_t _tx_scratch[TX_SCRATCH_LEN];
};

#endif  // AP_RANGING_DW1000_ENABLED
