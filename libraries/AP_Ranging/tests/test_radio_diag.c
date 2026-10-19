/* SPDX-License-Identifier: GPL-3.0-or-later */
#include "dw1000_radio_diag.h"
#include <assert.h>
#include <stdio.h>

static uint8_t regs[64][1024];
static unsigned reads;
static void read_spi(dwDevice_t *dev, const void *header, size_t hlen, void *data, size_t len)
{
    (void)dev;
    const uint8_t *h = header;
    const unsigned offset = hlen == 1 ? 0 : (h[1] & 0x7fU) + (hlen == 3 ? (unsigned)h[2] * 128U : 0);
    assert(offset + len <= sizeof(regs[0]));
    memcpy(data, &regs[h[0] & 0x3fU][offset], len);
    reads++;
}

static void driver_capture(void)
{
    dwOps_t ops = {.spiRead = read_spi};
    dwDevice_t dev = {0};
    dwInit(&dev, &ops);
    dev.channel = CHANNEL_2;
    dev.pulseFrequency = TX_PULSE_FREQ_64MHZ;
    // Exercise both signs, timestamp wrap and invalid zero-preamble power.
    const uint64_t stamps[] = {3, TWR_TIMESTAMP_MASK - 3, UINT64_C(123456789)};
    const uint16_t powers[] = {1, 1000, 65000};
    for (unsigned i = 0; i < 3; i++) {
        for (unsigned j = 0; j < 3; j++) {
            memset(regs, 0, sizeof(regs));
            twr_put_le(regs[RX_TIME], stamps[i], 5);
            twr_put_le(regs[RX_FQUAL] + CIR_PWR_SUB, powers[j], 2);
            twr_put_le(regs[RX_TIME] + FP_AMPL1_SUB, 50, 2);
            twr_put_le(regs[RX_FQUAL] + FP_AMPL2_SUB, 70, 2);
            twr_put_le(regs[RX_FQUAL] + FP_AMPL3_SUB, 60, 2);
            // RXPACC is bits 20..31.
            twr_put_le(regs[RX_FINFO], 128U << 20, 4);
            dwTime_t normal = {.full = 0};
            reads = 0;
            dwGetReceiveTimestamp(&dev, &normal);
            const unsigned baseline = reads;
            twr_diag_rx diag;
            reads = 0;
            const dwTime_t captured = twr_diag_read(&dev, &diag);
            assert(captured.full == normal.full);
            assert(reads == baseline + 6);
            assert(diag.flags == TWR_DIAG_VALID);
            assert(diag.raw == stamps[i]);
            assert(twr_diag_matches(&diag, normal.full));
        }
    }
    memset(regs, 0, sizeof(regs));
    twr_diag_rx invalid;
    twr_diag_read(&dev, &invalid);
    assert((invalid.flags & 3) == 0);
    assert(invalid.rx_cdbm == INT16_MIN && invalid.fp_cdbm == INT16_MIN);
}

static void wire_validation(void)
{
    twr_packet packet = {.type = TWR_POLL, .src_role = TWR_TAG, .src_id = 1,
                         .dst_role = TWR_ANCHOR, .dst_id = 8, .network = 1,
                         .sequence = 65535, .session = 123};
    uint8_t frame[TWR_DIAG_MAX_FRAME_SIZE + 1] = {0};
    twr_packet decoded;
    twr_diag_rx remote[2];
    bool request, report;
    unsigned n = twr_encode(&packet, frame, sizeof(frame));
    assert(n == 16 && twr_diag_decode(frame, n, &decoded, &request, &report, remote));
    assert(!request && !report);
    n = twr_diag_request(frame, n);
    assert(n == 17 && twr_diag_decode(frame, n, &decoded, &request, &report, remote));
    assert(request && !report);
    assert(!twr_decode(frame, n, &decoded)); // old receivers fail closed
    frame[16] ^= 1;
    assert(!twr_diag_decode(frame, n, &decoded, &request, &report, remote));
    packet.type = TWR_REPORT;
    const twr_diag_rx original[2] = {
        {.raw = TWR_TIMESTAMP_MASK, .rx_cdbm = -9500, .fp_cdbm = -10123, .correction = -25, .flags = 7},
        {.raw = 3, .rx_cdbm = INT16_MIN, .fp_cdbm = -7000, .correction = 23, .flags = 6}
    };
    n = twr_encode(&packet, frame, sizeof(frame));
    assert(n == 32 && twr_diag_decode(frame, n, &decoded, &request, &report, remote));
    assert(!request && !report);
    n = twr_diag_report(frame, n, original);
    assert(n == TWR_DIAG_MAX_FRAME_SIZE);
    assert(twr_diag_decode(frame, n, &decoded, &request, &report, remote));
    assert(!request && report);
    for (unsigned i = 0; i < 2; i++) {
        assert(remote[i].raw == original[i].raw && remote[i].rx_cdbm == original[i].rx_cdbm);
        assert(remote[i].fp_cdbm == original[i].fp_cdbm && remote[i].correction == original[i].correction);
        assert(remote[i].flags == original[i].flags);
    }
    for (unsigned len = 0; len <= sizeof(frame); len++) {
        if (len != 32 && len != TWR_DIAG_MAX_FRAME_SIZE) {
            assert(!twr_diag_decode(frame, len, &decoded, &request, &report, remote));
        }
    }
    frame[32] ^= 1;
    assert(!twr_diag_decode(frame, n, &decoded, &request, &report, remote));
    frame[32] ^= 1;
    frame[16] = 1; // failed range status cannot be hidden in a diagnostic report
    assert(!twr_diag_decode(frame, n, &decoded, &request, &report, remote));
}

static twr_packet wire(twr_engine *e, uint64_t tx, bool enabled, const twr_diag_rx rx[2], twr_diag_rx decoded_rx[2])
{
    uint8_t data[TWR_DIAG_MAX_FRAME_SIZE];
    twr_packet packet;
    bool request, report;
    unsigned len = twr_prepare_tx(e, tx, data);
    if (enabled) {
        len = twr_diag_request(data, len);
        len = twr_diag_report(data, len, rx);
    }
    assert(twr_diag_decode(data, len, &packet, &request, &report, decoded_rx));
    assert(request == (enabled && packet.type == TWR_POLL));
    assert(report == (enabled && packet.type == TWR_REPORT));
    assert(twr_sent(e, tx));
    return packet;
}

static float exchange(bool enabled, uint64_t base)
{
    twr_engine tag, anchor;
    twr_init(&tag, TWR_TAG, 1, 1, 30);
    twr_init(&anchor, TWR_ANCHOR, 8, 1, 30);
    const uint64_t offset = 76543210, flight = 500;
    const uint64_t t1 = base, t2 = t1 + flight + offset, t3 = t2 + 200000000;
    const uint64_t t4 = t3 + flight - offset, t5 = t4 + 250000000, t6 = t5 + flight + offset;
    twr_diag_rx rx[3] = {
        {.raw = t2 & TWR_TIMESTAMP_MASK, .rx_cdbm = -8000, .fp_cdbm = -8200, .correction = -15, .flags = 7},
        {.raw = t4 & TWR_TIMESTAMP_MASK, .rx_cdbm = -8100, .fp_cdbm = -8300, .correction = 23, .flags = 7},
        {.raw = t6 & TWR_TIMESTAMP_MASK, .rx_cdbm = -8200, .fp_cdbm = -8400, .correction = -8, .flags = 7}
    };
    const twr_diag_rx pair[2] = {rx[0], rx[2]};
    twr_diag_rx decoded[2] = {0};
    assert(twr_start(&tag, TWR_ANCHOR, 8, 123, 65535, 0));
    twr_packet p = wire(&tag, t1, enabled, pair, decoded);
    assert(twr_receive(&anchor, &p, t2 - 15, 1, true) == TWR_SEND);
    p = wire(&anchor, t3, enabled, pair, decoded);
    twr_packet wrong = p;
    wrong.session++;
    assert(twr_receive(&tag, &wrong, t4 + 23, 3, false) == TWR_IGNORED);
    assert(twr_receive(&tag, &p, t4 + 23, 4, false) == TWR_SEND);
    p = wire(&tag, t5, enabled, pair, decoded);
    assert(twr_receive(&anchor, &p, t6 - 8, 8, true) == TWR_SEND);
    p = wire(&anchor, t6 + 200000000, enabled, pair, decoded);
    assert(twr_receive(&tag, &p, t6 + 200000000 + flight - offset, 12, false) == TWR_COMPLETE);
    assert(tag.distance == anchor.distance);
    if (enabled) {
        rx[0] = decoded[0]; rx[2] = decoded[1];
        float raw = NAN;
        assert(twr_diag_uncorrected(&tag, rx, &raw));
        assert(fabs(raw - flight * TWR_METERS_PER_TICK) < 1e-6);
        rx[0].raw++; // wrong-frame or corrupted diagnostics must not look valid
        assert(!twr_diag_uncorrected(&tag, rx, &raw));
        rx[0].raw--;
        rx[2].flags = 0;
        assert(!twr_diag_uncorrected(&tag, rx, &raw));
    }
    return tag.distance;
}

int main(void)
{
    driver_capture();
    wire_validation();
    assert(exchange(false, 10000000) == exchange(true, 10000000));
    assert(exchange(false, TWR_TIMESTAMP_MASK - 1000) == exchange(true, TWR_TIMESTAMP_MASK - 1000));
    puts("Radio diagnostics: exact corrected timestamps, six extra reads, wire bounds, exchange matching and raw reconstruction pass");
    return 0;
}
