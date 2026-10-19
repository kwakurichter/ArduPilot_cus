/* SPDX-License-Identifier: GPL-3.0-or-later */
#pragma once
#include "twr_protocol.h"

// Optional bench extension. Ordinary v1 frames remain byte-for-byte unchanged.
#define TWR_DIAG_MARKER 0xd1U
#define TWR_DIAG_RX_SIZE 12U
#define TWR_DIAG_MAX_FRAME_SIZE (33U + 2U * TWR_DIAG_RX_SIZE)
#define TWR_DIAG_VALID 7U

typedef struct {
    uint64_t raw;
    int16_t rx_cdbm, fp_cdbm, correction;
    uint8_t flags; // bit 0: RX power, bit 1: first-path power, bit 2: correction valid
} twr_diag_rx;

static inline void twr_diag_pack(uint8_t *out, const twr_diag_rx *d)
{
    twr_put_le(out, d->raw, 5);
    twr_put_le(out + 5, (uint16_t)d->rx_cdbm, 2);
    twr_put_le(out + 7, (uint16_t)d->fp_cdbm, 2);
    twr_put_le(out + 9, (uint16_t)d->correction, 2);
    out[11] = d->flags;
}

static inline int16_t twr_diag_signed(uint64_t value)
{
    return value <= INT16_MAX ? (int16_t)value : (int16_t)((int32_t)value - 65536);
}

static inline void twr_diag_unpack(const uint8_t *in, twr_diag_rx *d)
{
    d->raw = twr_get_le(in, 5);
    d->rx_cdbm = twr_diag_signed(twr_get_le(in + 5, 2));
    d->fp_cdbm = twr_diag_signed(twr_get_le(in + 7, 2));
    d->correction = twr_diag_signed(twr_get_le(in + 9, 2));
    d->flags = in[11];
}

static inline unsigned twr_diag_request(uint8_t *frame, unsigned len)
{
    if (len == TWR_HEADER_SIZE && frame[3] == TWR_POLL) {
        frame[len++] = TWR_DIAG_MARKER;
    }
    return len;
}

static inline unsigned twr_diag_report(uint8_t *frame, unsigned len, const twr_diag_rx rx[2])
{
    if (len == 32 && frame[3] == TWR_REPORT) {
        frame[len++] = TWR_DIAG_MARKER;
        twr_diag_pack(frame + len, &rx[0]);
        twr_diag_pack(frame + len + TWR_DIAG_RX_SIZE, &rx[1]);
        len += 2 * TWR_DIAG_RX_SIZE;
    }
    return len;
}

static inline bool twr_diag_decode(const uint8_t *frame, unsigned len, twr_packet *packet,
                                   bool *requested, bool *report, twr_diag_rx rx[2])
{
    *requested = false;
    *report = false;
    if (len == 17 && frame[3] == TWR_POLL && frame[16] == TWR_DIAG_MARKER) {
        if (!twr_decode(frame, 16, packet)) {
            return false;
        }
        *requested = true;
        return true;
    }
    if (len == TWR_DIAG_MAX_FRAME_SIZE && frame[3] == TWR_REPORT && frame[32] == TWR_DIAG_MARKER) {
        if (!twr_decode(frame, 32, packet)) {
            return false;
        }
        twr_diag_unpack(frame + 33, &rx[0]);
        twr_diag_unpack(frame + 33 + TWR_DIAG_RX_SIZE, &rx[1]);
        *report = true;
        return true;
    }
    return twr_decode(frame, len, packet);
}

static inline bool twr_diag_matches(const twr_diag_rx *d, uint64_t corrected)
{
    return d->flags == TWR_DIAG_VALID &&
           ((d->raw + (int64_t)d->correction) & TWR_TIMESTAMP_MASK) == (corrected & TWR_TIMESTAMP_MASK);
}

static inline bool twr_diag_uncorrected(const twr_engine *e, const twr_diag_rx rx[3], float *distance)
{
    // POLL at anchor, RESPONSE at tag, FINAL at anchor, all from this exchange.
    if (!twr_diag_matches(&rx[0], e->b[0]) || !twr_diag_matches(&rx[1], e->a[1]) ||
        !twr_diag_matches(&rx[2], e->b[2])) {
        return false;
    }
    const uint64_t a[3] = {e->a[0], rx[1].raw, e->a[2]};
    const uint64_t b[3] = {rx[0].raw, e->b[1], rx[2].raw};
    return twr_distance(a, b, e->timeout_ms, distance);
}
