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
  SITL ranging backend; see AP_Ranging_SITL.h.
 */
#include "AP_Ranging_SITL.h"

#if AP_RANGING_SITL_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <GCS_MAVLink/GCS.h>
#include <SITL/SITL.h>

extern const AP_HAL::HAL &hal;

// The air: a multicast group shared by all SITL instances (next to AP_SwarmMesh's 57733)
#define RANGING_SITL_MCAST_ADDRESS "239.65.83.0"
#define RANGING_SITL_MCAST_PORT    57734U

namespace {
constexpr uint32_t POLL_JITTER_MS = 50;      // jittered polling (RNG_SCHED 0), as the DW1000 driver
constexpr uint32_t LINK_REPORT_MS = 5000;    // RNG_DEBUG cadence
}

AP_Ranging_SITL::AP_Ranging_SITL(AP_Ranging &frontend) : AP_Ranging_Backend(frontend)
{
    _sock_ok = _sock.connect(RANGING_SITL_MCAST_ADDRESS, RANGING_SITL_MCAST_PORT);
    if (!_sock_ok) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "RNG SITL: no multicast socket");
        return;
    }
    _sock.set_blocking(false);
    hal.scheduler->register_timer_process(FUNCTOR_BIND_MEMBER(&AP_Ranging_SITL::timer, void));
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "RNG SITL: ready (sysid %u)", unsigned(get_node_id()));
}

bool AP_Ranging_SITL::healthy()
{
    return _sock_ok && (AP_HAL::millis() - _last_range_ms < AP_RANGING_TIMEOUT_MS);
}

bool AP_Ranging_SITL::antenna(uint64_t now_us, Vector3f &p) const
{
    const SITL::SIM *sitl = AP::sitl();
    return sitl != nullptr && sitl->models.coop_sensors.enabled() &&
           sitl->models.coop_sensors.uwb_antenna(now_us, p);
}

void AP_Ranging_SITL::transmit(uint8_t type, uint8_t dst, const float value[3])
{
    AirFrame f {};
    Vector3f p;
    if (!antenna(AP_HAL::micros64(), p)) {
        return;
    }
    f.magic = AIR_MAGIC;
    f.tx_antenna[0] = p.x;
    f.tx_antenna[1] = p.y;
    f.tx_antenna[2] = p.z;
    f.type = type;
    f.src = get_node_id();
    f.dst = dst;
    f.seq = _seq;
    if (value != nullptr) {
        memcpy(f.value, value, sizeof(f.value));
    }
    _sock.send(&f, sizeof(f));
}

void AP_Ranging_SITL::start_poll(uint8_t dst, uint64_t now_us)
{
    _peer = dst;
    _seq++;
    _start_us = now_us;
    _exchange_start_ms = AP_HAL::millis();
    _state = State::I_WAIT_RESP;
    transmit(FRAME_POLL, dst, nullptr);
}

void AP_Ranging_SITL::timer()
{
    const uint64_t now_us = AP_HAL::micros64();
    const uint32_t now_ms = AP_HAL::millis();

    // the air: every frame for this node (own frames come back on the loopback)
    AirFrame f;
    for (uint8_t i = 0; i < 16; i++) {
        const ssize_t n = _sock.recv(&f, sizeof(f), 0);
        if (n <= 0) {
            break;
        }
        Vector3f own;
        if (n != sizeof(f) || f.magic != AIR_MAGIC || f.dst != get_node_id() ||
            f.src == get_node_id() || !antenna(now_us, own)) {
            continue;
        }
        const Vector3f tx(f.tx_antenna[0], f.tx_antenna[1], f.tx_antenna[2]);
        receive(f, (own - tx).length(), now_us);
    }

    // a stalled exchange (lost frame, dropout, busy peer)
    if (_state != State::IDLE && now_ms - _exchange_start_ms > get_xchg_ms()) {
        _state = State::IDLE;
        _fails++;
    }

    if (knot_schedule()) {
        // knot-aligned pairs, as AP_Ranging_DW1000::timer()
        Slot slot;
        if (scheduled_slot(now_us, slot) && slot.initiator && slot.period != _sched_period &&
            now_us >= slot.start_us) {
            _sched_period = slot.period;
            if (_state == State::IDLE && now_us < slot.start_us + get_slot_us()) {
                start_poll(slot.peer, now_us);
            } else {
                _missed++;
            }
        }
    } else if (_state == State::IDLE && now_ms - _last_poll_ms >= _poll_interval_ms) {
        // jittered polling of the roster in turn
        _last_poll_ms = now_ms;
        _poll_interval_ms = get_poll_ms() + (get_random16() % POLL_JITTER_MS);
        for (uint8_t i = 0; i < AP_RANGING_MAX_NODES; i++) {
            _next_peer = (_next_peer + 1) % AP_RANGING_MAX_NODES;
            const uint8_t id = get_peer_id(_next_peer);
            if (id != 0 && id != get_node_id()) {
                start_poll(id, now_us);
                break;
            }
        }
    }

    if (get_debug() > 0 && now_ms - _last_report_ms >= LINK_REPORT_MS) {
        _last_report_ms = now_ms;
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "RNG SITL: rng=%lu fail=%lu miss=%lu",
                      (unsigned long)_ranges, (unsigned long)_fails, (unsigned long)_missed);
    }
}

void AP_Ranging_SITL::receive(const AirFrame &f, float distance, uint64_t now_us)
{
    switch (f.type) {
    case FRAME_POLL: {
        if (_state != State::IDLE) {
            return;     // busy: the initiator times out, as on hardware
        }
        _peer = f.src;
        _seq = f.seq;
        _start_us = now_us;
        _d_poll = distance;
        _exchange_start_ms = AP_HAL::millis();
        _state = State::R_WAIT_FINAL;
        transmit(FRAME_RESPONSE, f.src, nullptr);
        return;
    }

    case FRAME_RESPONSE: {
        if (_state != State::I_WAIT_RESP || f.src != _peer || f.seq != _seq) {
            return;
        }
        const float value[3] {distance, 0.0f, 0.0f};
        transmit(FRAME_FINAL, f.src, value);
        if (knot_schedule()) {
            _state = State::I_WAIT_REPORT;
        } else {
            _state = State::IDLE;
        }
        return;
    }

    case FRAME_FINAL: {
        if (_state != State::R_WAIT_FINAL || f.src != _peer || f.seq != _seq) {
            return;
        }
        _state = State::IDLE;
        SITL::SIM *sitl = AP::sitl();
        if (sitl == nullptr) {
            return;
        }
        // the responder draws the exchange's error, as the thesis simulator; a dropout
        // loses the whole exchange (no record at either end)
        SITL::CoopSensors::UwbError err;
        sitl->models.coop_sensors.uwb_error(err);
        if (err.dropped) {
            _fails++;
            return;
        }
        AP_Ranging::Exchange e {};
        e.start_us = _start_us;
        e.duration_us = uint32_t(now_us - _start_us);
        e.range_m = 0.25f * (_d_poll + 2.0f * f.value[0] + distance) + err.error_m;
        e.rx_power_dbm = err.rx_power_dbm;
        e.fp_power_dbm = err.fp_power_dbm;
        e.initiator = f.src;
        e.responder = get_node_id();
        e.power_valid = true;
        record_exchange(e);
        _ranges++;
        _last_range_ms = AP_HAL::millis();
        if (knot_schedule()) {
            const float value[3] {e.range_m, e.rx_power_dbm, e.fp_power_dbm};
            transmit(FRAME_REPORT, f.src, value);
        }
        return;
    }

    case FRAME_REPORT: {
        if (_state != State::I_WAIT_REPORT || f.src != _peer || f.seq != _seq) {
            return;
        }
        _state = State::IDLE;
        AP_Ranging::Exchange e {};
        e.start_us = _start_us;
        e.duration_us = uint32_t(now_us - _start_us);
        e.range_m = f.value[0];
        e.rx_power_dbm = f.value[1];
        e.fp_power_dbm = f.value[2];
        e.initiator = get_node_id();
        e.responder = f.src;
        e.power_valid = true;
        record_exchange(e);
        _ranges++;
        _last_range_ms = AP_HAL::millis();
        return;
    }

    default:
        return;
    }
}

#endif  // AP_RANGING_SITL_ENABLED
