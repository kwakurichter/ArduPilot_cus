/*
 * ArduSwarm double-sided TWR protocol, version 1.
 * SPDX-License-Identifier: GPL-3.0-or-later
 * Portable C implementation shared with cus-node-firmware. Keep copies identical.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <math.h>

#define TWR_HEADER_SIZE 16U
#define TWR_MAX_FRAME_SIZE 32U
#define TWR_TIMESTAMP_MASK UINT64_C(0xffffffffff)
#define TWR_TICKS_PER_MS UINT64_C(63897600)
#define TWR_METERS_PER_TICK (299792458.0 / 63897600000.0)

enum twr_role { TWR_TAG = 1, TWR_ANCHOR = 2 };
enum twr_type { TWR_POLL = 1, TWR_RESPONSE, TWR_FINAL, TWR_REPORT };
enum twr_state {
    TWR_IDLE, TWR_TX_POLL, TWR_WAIT_RESPONSE, TWR_TX_RESPONSE,
    TWR_WAIT_FINAL, TWR_TX_FINAL, TWR_WAIT_REPORT, TWR_TX_REPORT
};
enum twr_result { TWR_IGNORED, TWR_SEND, TWR_COMPLETE, TWR_FAILED };

typedef struct {
    uint8_t type, src_role, src_id, dst_role, dst_id;
    uint16_t network, sequence;
    uint32_t session;
    uint64_t times[3];
} twr_packet;

static inline uint64_t twr_get_le(const uint8_t *p, unsigned n)
{
    uint64_t v = 0;
    for (unsigned i = 0; i < n; i++) {
        v |= (uint64_t)p[i] << (8U * i);
    }
    return v;
}

static inline void twr_put_le(uint8_t *p, uint64_t v, unsigned n)
{
    for (unsigned i = 0; i < n; i++) {
        p[i] = (uint8_t)(v >> (8U * i));
    }
}

static inline bool twr_identity_valid(uint8_t role, uint8_t id)
{
    return id != 0 && (role == TWR_TAG || (role == TWR_ANCHOR && id <= 8));
}

static inline unsigned twr_frame_size(uint8_t type)
{
    switch (type) {
    case TWR_POLL:
    case TWR_RESPONSE:
        return TWR_HEADER_SIZE;
    case TWR_FINAL:
        return 31;
    case TWR_REPORT:
        return 32;
    default:
        return 0;
    }
}

static inline unsigned twr_encode(const twr_packet *p, uint8_t *out, size_t capacity)
{
    unsigned len = twr_frame_size(p->type);
    if (len == 0 || capacity < len || !twr_identity_valid(p->src_role, p->src_id) ||
        !twr_identity_valid(p->dst_role, p->dst_id)) {
        return 0;
    }
    memset(out, 0, len);
    twr_put_le(out, 0xa752, 2);
    out[2] = 1;
    out[3] = p->type;
    twr_put_le(out + 4, p->network, 2);
    out[6] = p->src_role;
    out[7] = p->src_id;
    out[8] = p->dst_role;
    out[9] = p->dst_id;
    twr_put_le(out + 10, p->sequence, 2);
    twr_put_le(out + 12, p->session, 4);
    if (p->type == TWR_FINAL || p->type == TWR_REPORT) {
        unsigned offset = p->type == TWR_FINAL ? 16 : 17;
        for (unsigned i = 0; i < 3; i++) {
            twr_put_le(out + offset + i * 5, p->times[i], 5);
        }
    }
    return len;
}

static inline bool twr_decode(const uint8_t *in, size_t len, twr_packet *p)
{
    if (len < TWR_HEADER_SIZE || twr_get_le(in, 2) != 0xa752 || in[2] != 1 ||
        twr_frame_size(in[3]) != len || !twr_identity_valid(in[6], in[7]) ||
        !twr_identity_valid(in[8], in[9]) || (in[3] == TWR_REPORT && in[16] != 0)) {
        return false;
    }
    memset(p, 0, sizeof(*p));
    p->type = in[3];
    p->network = (uint16_t)twr_get_le(in + 4, 2);
    p->src_role = in[6];
    p->src_id = in[7];
    p->dst_role = in[8];
    p->dst_id = in[9];
    p->sequence = (uint16_t)twr_get_le(in + 10, 2);
    p->session = (uint32_t)twr_get_le(in + 12, 4);
    if (p->type == TWR_FINAL || p->type == TWR_REPORT) {
        unsigned offset = p->type == TWR_FINAL ? 16 : 17;
        for (unsigned i = 0; i < 3; i++) {
            p->times[i] = twr_get_le(in + offset + i * 5, 5);
        }
    }
    return true;
}

/* a: poll TX, response RX, final TX; b: poll RX, response TX, final RX. */
static inline bool twr_distance(const uint64_t a[3], const uint64_t b[3],
                                uint16_t timeout_ms, float *distance)
{
    const uint64_t ra = (a[1] - a[0]) & TWR_TIMESTAMP_MASK;
    const uint64_t da = (a[2] - a[1]) & TWR_TIMESTAMP_MASK;
    const uint64_t rb = (b[2] - b[1]) & TWR_TIMESTAMP_MASK;
    const uint64_t db = (b[1] - b[0]) & TWR_TIMESTAMP_MASK;
    const uint64_t limit = (uint64_t)timeout_ms * TWR_TICKS_PER_MS;
    if (timeout_ms == 0 || timeout_ms > 200 || ra == 0 || rb == 0 || da == 0 || db == 0 ||
        ra > limit || rb > limit || da > limit || db > limit || ra + da > limit || rb + db > limit) {
        return false;
    }
    // Cast before multiplying: even individually bounded intervals can overflow uint64_t.
    const double ticks = ((double)ra * (double)rb - (double)da * (double)db) /
                         (double)(ra + rb + da + db);
    const double meters = ticks * TWR_METERS_PER_TICK;
    if (!isfinite(meters) || meters <= 0.0 || meters >= 1000.0) {
        return false;
    }
    *distance = (float)meters;
    return true;
}

typedef struct {
    enum twr_state state;
    uint8_t role, id, remote_role, remote_id;
    uint16_t network, sequence, timeout_ms;
    uint32_t session, started_ms;
    uint64_t a[3], b[3], expected_tx;
    float distance;
    bool new_range, initiator;
    struct {
        uint32_t session, time_ms;
        uint16_t sequence;
        uint8_t id;
        bool valid;
    } recent[8];
    uint8_t recent_next;
} twr_engine;

static inline void twr_init(twr_engine *e, uint8_t role, uint8_t id,
                            uint16_t network, uint16_t timeout_ms)
{
    memset(e, 0, sizeof(*e));
    e->role = role;
    e->id = id;
    e->network = network;
    e->timeout_ms = timeout_ms;
}

static inline void twr_abort(twr_engine *e)
{
    e->state = TWR_IDLE;
    e->new_range = false;
}

static inline bool twr_expired(const twr_engine *e, uint32_t now_ms)
{
    return e->state != TWR_IDLE && (uint32_t)(now_ms - e->started_ms) >= e->timeout_ms;
}

static inline bool twr_tx_pending(const twr_engine *e)
{
    return e->state == TWR_TX_POLL || e->state == TWR_TX_RESPONSE ||
           e->state == TWR_TX_FINAL || e->state == TWR_TX_REPORT;
}

static inline bool twr_start(twr_engine *e, uint8_t remote_role, uint8_t remote_id,
                             uint32_t session, uint16_t sequence, uint32_t now_ms)
{
    if (e->state != TWR_IDLE || e->role != TWR_TAG ||
        !twr_identity_valid(remote_role, remote_id) ||
        (remote_role == e->role && remote_id == e->id)) {
        return false;
    }
    e->remote_role = remote_role;
    e->remote_id = remote_id;
    e->session = session;
    e->sequence = sequence;
    e->started_ms = now_ms;
    e->initiator = true;
    e->new_range = false;
    e->state = TWR_TX_POLL;
    return true;
}

/* Called after transport has programmed delayed TX; never use a software time estimate. */
static inline unsigned twr_prepare_tx(twr_engine *e, uint64_t scheduled_tx, uint8_t *out)
{
    twr_packet p;
    memset(&p, 0, sizeof(p));
    p.network = e->network;
    p.src_role = e->role;
    p.src_id = e->id;
    p.dst_role = e->remote_role;
    p.dst_id = e->remote_id;
    p.sequence = e->sequence;
    p.session = e->session;
    e->expected_tx = scheduled_tx & TWR_TIMESTAMP_MASK;
    switch (e->state) {
    case TWR_TX_POLL:
        p.type = TWR_POLL;
        break;
    case TWR_TX_RESPONSE:
        p.type = TWR_RESPONSE;
        break;
    case TWR_TX_FINAL:
        p.type = TWR_FINAL;
        e->a[2] = e->expected_tx;
        memcpy(p.times, e->a, sizeof(p.times));
        break;
    case TWR_TX_REPORT:
        p.type = TWR_REPORT;
        memcpy(p.times, e->b, sizeof(p.times));
        break;
    default:
        return 0;
    }
    return twr_encode(&p, out, TWR_MAX_FRAME_SIZE);
}

static inline bool twr_sent(twr_engine *e, uint64_t actual_tx)
{
    actual_tx &= TWR_TIMESTAMP_MASK;
    if (!twr_tx_pending(e) || actual_tx != e->expected_tx) {
        twr_abort(e);
        return false;
    }
    switch (e->state) {
    case TWR_TX_POLL:
        e->a[0] = actual_tx;
        e->state = TWR_WAIT_RESPONSE;
        break;
    case TWR_TX_RESPONSE:
        e->b[1] = actual_tx;
        e->state = TWR_WAIT_FINAL;
        break;
    case TWR_TX_FINAL:
        e->a[2] = actual_tx;
        e->state = TWR_WAIT_REPORT;
        break;
    case TWR_TX_REPORT:
        e->state = TWR_IDLE;
        break;
    default:
        return false;
    }
    return true;
}

/* Admission of a POLL against the configured peer roster is a transport policy. */
static inline enum twr_result twr_receive(twr_engine *e, const twr_packet *p,
                                          uint64_t rx, uint32_t now_ms, bool admit_poll)
{
    e->new_range = false;
    if (twr_expired(e, now_ms)) {
        twr_abort(e);
        return TWR_FAILED;
    }
    if (p->network != e->network || p->dst_role != e->role || p->dst_id != e->id ||
        (p->src_role == e->role && p->src_id == e->id)) {
        return TWR_IGNORED;
    }
    rx &= TWR_TIMESTAMP_MASK;
    if (p->type == TWR_POLL) {
        if (!admit_poll || e->state != TWR_IDLE || p->src_role != TWR_TAG) {
            return TWR_IGNORED;
        }
        for (unsigned i = 0; i < 8; i++) {
            if (e->recent[i].valid && e->recent[i].id == p->src_id &&
                e->recent[i].session == p->session && e->recent[i].sequence == p->sequence &&
                (uint32_t)(now_ms - e->recent[i].time_ms) <= 200) {
                return TWR_IGNORED;
            }
        }
        const unsigned slot = e->recent_next;
        e->recent_next = (e->recent_next + 1) % 8;
        e->recent[slot].id = p->src_id;
        e->recent[slot].session = p->session;
        e->recent[slot].sequence = p->sequence;
        e->recent[slot].time_ms = now_ms;
        e->recent[slot].valid = true;
        e->remote_role = p->src_role;
        e->remote_id = p->src_id;
        e->sequence = p->sequence;
        e->session = p->session;
        e->started_ms = now_ms;
        e->initiator = false;
        e->b[0] = rx;
        e->state = TWR_TX_RESPONSE;
        return TWR_SEND;
    }
    if (p->src_role != e->remote_role || p->src_id != e->remote_id ||
        p->sequence != e->sequence || p->session != e->session) {
        return TWR_IGNORED;
    }
    if (p->type == TWR_RESPONSE && e->state == TWR_WAIT_RESPONSE) {
        e->a[1] = rx;
        e->state = TWR_TX_FINAL;
        return TWR_SEND;
    }
    if (p->type == TWR_FINAL && e->state == TWR_WAIT_FINAL) {
        memcpy(e->a, p->times, sizeof(e->a));
        e->b[2] = rx;
        if (!twr_distance(e->a, e->b, e->timeout_ms, &e->distance)) {
            twr_abort(e);
            return TWR_FAILED;
        }
        e->new_range = true;
        e->state = TWR_TX_REPORT;
        return TWR_SEND;
    }
    if (p->type == TWR_REPORT && e->state == TWR_WAIT_REPORT) {
        memcpy(e->b, p->times, sizeof(e->b));
        if (!twr_distance(e->a, e->b, e->timeout_ms, &e->distance)) {
            twr_abort(e);
            return TWR_FAILED;
        }
        e->new_range = true;
        e->state = TWR_IDLE;
        return TWR_COMPLETE;
    }
    return TWR_IGNORED;
}
