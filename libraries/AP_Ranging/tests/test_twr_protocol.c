/* SPDX-License-Identifier: GPL-3.0-or-later */
#include "twr_protocol.h"
#include <assert.h>
#include <stdio.h>

static twr_packet wire(twr_engine *e, uint64_t tx)
{
    uint8_t data[32];
    twr_packet p;
    unsigned len = twr_prepare_tx(e, tx, data);
    assert(len != 0);
    assert(twr_decode(data, len, &p));
    assert(twr_sent(e, tx));
    return p;
}

static void exchange(bool anchor, bool lose_report, uint64_t base, uint32_t start)
{
    twr_engine a, b;
    twr_init(&a, TWR_TAG, 1, 17, 30);
    twr_init(&b, anchor ? TWR_ANCHOR : TWR_TAG, anchor ? 1 : 2, 17, 30);
    // Independent clock origins, unequal reply delays, and exactly 1000 ticks of flight.
    const uint64_t offset = 76543210;
    const uint64_t t1 = base;
    const uint64_t t2 = t1 + 1000 + offset;
    const uint64_t t3 = t2 + 200000000;
    const uint64_t t4 = t3 + 1000 - offset;
    const uint64_t t5 = t4 + 250000000;
    const uint64_t t6 = t5 + 1000 + offset;
    assert(twr_start(&a, b.role, b.id, 0x12345678, 65535, start));
    twr_packet p = wire(&a, t1);
    assert(twr_receive(&b, &p, t2, start + 1, true) == TWR_SEND);
    assert(!b.new_range);
    // Busy responders cannot be stolen by a competing tag.
    twr_packet other = p;
    other.src_id = 4;
    assert(twr_receive(&b, &other, t2, start + 2, true) == TWR_IGNORED);
    p = wire(&b, t3);
    other = p;
    other.session++;
    assert(twr_receive(&a, &other, t4, start + 4, false) == TWR_IGNORED);
    assert(twr_receive(&a, &p, t4, start + 4, false) == TWR_SEND);
    p = wire(&a, t5);
    assert(twr_receive(&b, &p, t6, start + 8, true) == TWR_SEND);
    assert(b.new_range);
    assert(fabs(b.distance - 1000 * TWR_METERS_PER_TICK) < 0.00001);
    p = wire(&b, t6 + 200000000);
    if (lose_report) {
        assert(!a.new_range);
        assert(twr_expired(&a, start + 30));
        twr_abort(&a);
        assert(twr_receive(&a, &p, t5 + 200002000, start + 31, false) == TWR_IGNORED);
    } else {
        assert(twr_receive(&a, &p, t5 + 200002000, start + 12, false) == TWR_COMPLETE);
        assert(a.new_range);
        assert(fabs(a.distance - b.distance) < 0.000001);
        assert(twr_receive(&a, &p, t5 + 200002000, start + 13, false) == TWR_IGNORED);
        assert(!a.new_range);
    }
}

static void codec(void)
{
    twr_packet p = {0}, decoded;
    p.type = TWR_POLL;
    p.src_role = TWR_TAG;
    p.src_id = 255;
    p.dst_role = TWR_ANCHOR;
    p.dst_id = 8;
    p.network = 0x1234;
    p.sequence = 0x5678;
    p.session = 0xabcdef12;
    const uint8_t expected[] = {0x52,0xa7,1,1,0x34,0x12,1,255,2,8,0x78,0x56,0x12,0xef,0xcd,0xab};
    uint8_t bytes[33] = {0};
    assert(twr_encode(&p, bytes, sizeof(bytes)) == sizeof(expected));
    assert(memcmp(bytes, expected, sizeof(expected)) == 0);
    for (unsigned n = 0; n <= 33; n++) {
        assert(twr_decode(bytes, n, &decoded) == (n == 16));
    }
    bytes[2] = 2;
    assert(!twr_decode(bytes, 16, &decoded));
    bytes[2] = 1;
    bytes[9] = 9;
    assert(!twr_decode(bytes, 16, &decoded));
    p.type = TWR_REPORT;
    p.times[0] = TWR_TIMESTAMP_MASK;
    p.times[1] = 0;
    p.times[2] = 0x0123456789;
    assert(twr_encode(&p, bytes, sizeof(bytes)) == 32);
    assert(twr_decode(bytes, 32, &decoded));
    assert(memcmp(p.times, decoded.times, sizeof(p.times)) == 0);
    bytes[16] = 1;
    assert(!twr_decode(bytes, 32, &decoded));
}

static void errors(void)
{
    twr_engine e;
    twr_init(&e, TWR_TAG, 1, 1, 30);
    assert(!twr_start(&e, TWR_TAG, 1, 1, 1, 0));
    assert(twr_start(&e, TWR_TAG, 2, 1, 1, 0));
    uint8_t bytes[32];
    twr_prepare_tx(&e, 1000, bytes);
    assert(!twr_sent(&e, 1001));
    assert(e.state == TWR_IDLE);
    uint64_t a[] = {0,1,2}, b[] = {0,1,2};
    float distance = 123;
    assert(!twr_distance(a,b,30,&distance));
    a[1] = TWR_TIMESTAMP_MASK;
    assert(!twr_distance(a,b,200,&distance));
    assert(distance == 123);
    twr_packet poll = {0};
    poll.type=TWR_POLL; poll.src_role=TWR_TAG; poll.src_id=2;
    poll.dst_role=TWR_TAG; poll.dst_id=1; poll.network=1;
    assert(twr_receive(&e,&poll,0,0,false)==TWR_IGNORED);
    assert(twr_receive(&e,&poll,0,0,true)==TWR_SEND);
    assert(twr_receive(&e,&poll,0,30,true)==TWR_FAILED);
    assert(e.state == TWR_IDLE);
}

static void loss_and_recovery(void)
{
    // Drop each frame in turn. No received frame can extend the original deadline.
    for (unsigned drop = 0; drop < 4; drop++) {
        twr_engine a, b;
        twr_init(&a, TWR_TAG, 255, 1, 30);
        twr_init(&b, TWR_ANCHOR, 8, 1, 30);
        assert(twr_start(&a, TWR_ANCHOR, 8, 77, 0, 0));
        twr_packet poll = wire(&a, 1000);
        if (drop > 0) {
            assert(twr_receive(&b, &poll, 2000, 1, true) == TWR_SEND);
            twr_packet response = wire(&b, 200002000);
            if (drop > 1) {
                assert(twr_receive(&a, &response, 200003000, 4, true) == TWR_SEND);
                twr_packet final = wire(&a, 400003000);
                if (drop > 2) {
                    assert(twr_receive(&b, &final, 400004000, 8, true) == TWR_SEND);
                    assert(b.new_range);
                    wire(&b, 600004000);
                }
            }
        }
        assert(!a.new_range);
        assert(twr_expired(&a, 30));
        if (drop == 1 || drop == 2) {
            assert(!b.new_range);
            assert(twr_expired(&b, 31));
        }
        twr_abort(&a);
        twr_abort(&b);
        if (drop > 0) {
            assert(twr_receive(&b, &poll, 700000000, 32, true) == TWR_IGNORED);
        }
        // A fresh session after initiator reboot can restart immediately.
        poll.session++;
        assert(twr_receive(&b, &poll, 700000000, 33, true) == TWR_SEND);
    }
}

int main(void)
{
    codec();
    errors();
    loss_and_recovery();
    exchange(false,false,100000,0);
    exchange(true,false,TWR_TIMESTAMP_MASK-100000,UINT32_MAX-5);
    exchange(false,true,0,0);
    puts("TWR codec, two-sided exchange, loss, identity, bounds and clock-wrap tests passed");
    return 0;
}
