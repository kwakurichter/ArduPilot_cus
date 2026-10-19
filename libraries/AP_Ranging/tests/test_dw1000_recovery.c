/* SPDX-License-Identifier: GPL-3.0-or-later */
// Exercise real libdw1000 IRQ dispatch against a register model, without hardware.
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "libdw1000.h"
#include "twr_protocol.h"

static uint8_t regs[64][1024];
static twr_engine engine;
static unsigned sent, rejected;

static unsigned address(const uint8_t *header, size_t len)
{
    return len == 1 ? 0 : (header[1] & 0x7fU) + (len == 3 ? (unsigned)header[2] * 128U : 0);
}

static void read_spi(dwDevice_t *dev, const void *header, size_t hlen, void *data, size_t len)
{
    (void)dev;
    const uint8_t *h = header;
    unsigned offset = address(h, hlen);
    assert(offset + len <= sizeof(regs[0]));
    memcpy(data, &regs[h[0] & 0x3fU][offset], len);
}

static void write_spi(dwDevice_t *dev, const void *header, size_t hlen, const void *data, size_t len)
{
    (void)dev;
    const uint8_t *h = header;
    const uint8_t *p = data;
    unsigned reg = h[0] & 0x3fU;
    unsigned offset = address(h, hlen);
    assert(offset + len <= sizeof(regs[0]));
    if (reg == SYS_STATUS) {
        for (size_t i = 0; i < len; i++) {
            regs[reg][offset + i] &= ~p[i]; // write-one-to-clear
        }
    } else {
        memcpy(&regs[reg][offset], data, len);
    }
}

static void tx_done(dwDevice_t *dev)
{
    dwTime_t tx = {.full = 0};
    dwGetTransmitTimestamp(dev, &tx);
    sent++;
    if (!twr_sent(&engine, tx.full)) {
        rejected++;
    }
}

static void latch_tx(uint64_t timestamp)
{
    regs[SYS_STATUS][TXFRS_BIT / 8] |= 1U << (TXFRS_BIT % 8);
    twr_put_le(regs[TX_TIME], timestamp, LEN_STAMP);
}

int main(void)
{
    dwOps_t ops = {.spiRead = read_spi, .spiWrite = write_spi};
    dwDevice_t dev = {0};
    dwInit(&dev, &ops);
    dwAttachSentHandler(&dev, tx_done);
    twr_init(&engine, TWR_TAG, 1, 1, 30);
    uint8_t frame[TWR_MAX_FRAME_SIZE];
    assert(twr_start(&engine, TWR_ANCHOR, 6, 123, 1, 0));
    assert(twr_prepare_tx(&engine, 5000, frame));
    latch_tx(5000);
    assert(twr_expired(&engine, 31));
    twr_abort(&engine);
    dwNewReceive(&dev);
    dwStartReceive(&dev);
    dwHandleInterrupt(&dev);
    // An RX restart alone leaves TXFRS latched and reports a false TX rejection.
    assert(sent == 1 && rejected == 1);

    assert(twr_start(&engine, TWR_ANCHOR, 6, 123, 2, 40));
    assert(twr_prepare_tx(&engine, 9000, frame));
    latch_tx(9000);
    assert(twr_expired(&engine, 71));
    twr_abort(&engine);
    dwIdle(&dev);
    dwClearTransmitStatus(&dev);
    dwNewReceive(&dev);
    dwStartReceive(&dev);
    dwHandleInterrupt(&dev);
    dwHandleInterrupt(&dev);
    assert(sent == 1 && rejected == 1);

    // The next exchange still receives a real completion, exactly once.
    assert(twr_start(&engine, TWR_ANCHOR, 6, 123, 3, 80));
    assert(twr_prepare_tx(&engine, 12000, frame));
    latch_tx(12000);
    dwHandleInterrupt(&dev);
    dwHandleInterrupt(&dev);
    assert(sent == 2 && rejected == 1 && engine.state == TWR_WAIT_RESPONSE);

    // Clearing TX status must not discard an unrelated received frame.
    regs[SYS_STATUS][RXFCG_BIT / 8] |= 1U << (RXFCG_BIT % 8);
    latch_tx(12000);
    dwClearTransmitStatus(&dev);
    dwReadSystemEventStatusRegister(&dev);
    assert(!dwIsTransmitDone(&dev));
    assert(dwIsReceiveDone(&dev));
    puts("DW1000: stale TX reproduced; cleanup, next exchange and RX preservation pass");
    return 0;
}
