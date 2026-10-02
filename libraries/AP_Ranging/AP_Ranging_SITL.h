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
/*
  SITL ranging backend (RNG_TYPE 100): SITL instances range each other.

  The exchange is the DW1000 driver's, frame for frame (POLL, RESPONSE, FINAL and, in the
  knot-aligned schedule, REPORT), on a UDP multicast group shared by every instance on the
  host ("the air"). It runs from a 1 kHz timer, so the schedule, both ends' records, the
  REPORT path and lost exchanges are exercised as on hardware; only the radio is replaced:

  - Instead of 40-bit DW1000 timestamps, every frame on the air carries the transmitter's
    true antenna position (SIM_CoopSensors: shared map, SIM_COOP_MAP_*, SIM_COOP_ANT_*), and
    the receiver takes the distance to its own antenna at reception. The FINAL carries the
    initiator's distance at the RESPONSE, as it carries resp_rx on hardware.
  - The responder's range is the Alternative DS-TWR weighting of the three distances,
    (d_poll + 2 d_response + d_final) / 4, plus the error of the thesis simulator
    (SIM_COOP_UWB_*: noise, NLOS excess, outliers, dropouts, receive powers).

  SIM_COOP_ENABLE must be 1 (the antenna truth comes from the cooperative sensor model).
  Every instance's FC clock is its own simulated time; the schedule assumes they agree to
  well within a slot (coop_sitl.py starts the instances on a common clock).
 */
#pragma once

#include "AP_Ranging_Backend.h"

#if AP_RANGING_SITL_ENABLED

#include <AP_HAL/utility/Socket_native.h>

class AP_Ranging_SITL : public AP_Ranging_Backend
{
public:
    AP_Ranging_SITL(AP_Ranging &frontend);

    bool healthy() override;
    void update() override {}

private:
    // 1 kHz timer: receive, schedule, time out
    void timer();

    enum : uint8_t { FRAME_POLL = 0xC1, FRAME_RESPONSE = 0xC2, FRAME_FINAL = 0xC3, FRAME_REPORT = 0xC4 };

    // One frame on the SITL air
    struct PACKED AirFrame {
        uint32_t magic;          // AIR_MAGIC
        float tx_antenna[3];     // transmitter's antenna at transmission (map ENU, m)
        uint8_t type, src, dst, seq;
        // FINAL: [0] initiator's distance at the RESPONSE reception
        // REPORT: [0] range, [1] rx power, [2] first-path power of the FINAL
        float value[3];
    };
    static constexpr uint32_t AIR_MAGIC = 0x31425755;  // "UWB1"

    void transmit(uint8_t type, uint8_t dst, const float value[3]);
    void receive(const AirFrame &f, float distance, uint64_t now_us);
    void start_poll(uint8_t dst, uint64_t now_us);
    bool antenna(uint64_t now_us, Vector3f &p) const;

    enum class State : uint8_t {
        IDLE,             // listening
        I_WAIT_RESP,      // initiator: POLL sent
        I_WAIT_REPORT,    // initiator (RNG_SCHED 1): FINAL sent
        R_WAIT_FINAL,     // responder: RESPONSE sent
    };
    State    _state = State::IDLE;
    uint8_t  _peer;
    uint8_t  _seq;
    uint64_t _start_us;       // exchange start (FC time)
    uint32_t _exchange_start_ms;
    float    _d_poll;         // responder: distance at the POLL

    // schedule
    uint32_t _sched_period = UINT32_MAX;
    uint32_t _last_poll_ms;
    uint32_t _poll_interval_ms = 50;
    uint8_t  _next_peer;

    // counters
    uint32_t _ranges;
    uint32_t _fails;
    uint32_t _missed;
    uint32_t _last_range_ms;
    uint32_t _last_report_ms;

    SocketAPM_native _sock{true};   // true = datagram (UDP)
    bool _sock_ok;
};

#endif  // AP_RANGING_SITL_ENABLED
