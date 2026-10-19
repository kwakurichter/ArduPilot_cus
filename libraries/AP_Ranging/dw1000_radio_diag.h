/* SPDX-License-Identifier: GPL-3.0-or-later */
#pragma once
#include "libdw1000.h"
#include "twr_radio_diag.h"

static inline int16_t twr_diag_power(float dbm, uint8_t flag, uint8_t *flags)
{
    const float scaled = dbm * 100.0f;
    if (!isfinite(scaled) || scaled <= INT16_MIN || scaled > INT16_MAX) {
        return INT16_MIN;
    }
    *flags |= flag;
    return (int16_t)scaled;
}

// Capture while the received packet's registers are still latched. This is the
// same raw-read + correction path as dwGetReceiveTimestamp(), without rereading
// RX_TIME. Six additional SPI reads provide power diagnostics; no submodule edit.
static inline dwTime_t twr_diag_read(dwDevice_t *dev, twr_diag_rx *d)
{
    dwTime_t time = {.full = 0};
    memset(d, 0, sizeof(*d));
    dwGetRawReceiveTimestamp(dev, &time);
    d->raw = time.full & TWR_TIMESTAMP_MASK;
    dwCorrectTimestamp(dev, &time);
    const uint64_t delta = (time.full - d->raw) & TWR_TIMESTAMP_MASK;
    const int64_t signed_delta = delta < (UINT64_C(1) << 39) ? (int64_t)delta : (int64_t)delta - (INT64_C(1) << 40);
    if (signed_delta >= INT16_MIN && signed_delta <= INT16_MAX) {
        d->correction = (int16_t)signed_delta;
        d->flags |= 4;
    }
    d->rx_cdbm = twr_diag_power(dwGetReceivePower(dev), 1, &d->flags);
    d->fp_cdbm = twr_diag_power(dwGetFirstPathPower(dev), 2, &d->flags);
    return time;
}
