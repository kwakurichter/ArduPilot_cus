#pragma once

#include <AP_Logger/LogStructure.h>

#define LOG_IDS_FROM_RANGING \
    LOG_RANGING_MSG

// @LoggerMessage: TWR
// @Description: UWB two-way-ranging: per-node id / health / range. Slots are
// stable - a given node id keeps the same slot for the life of the vehicle, so
// column N always refers to the same node across log messages.
// @Field: TimeUS: Time since system startup
// @Field: Cnt: Number of peer nodes being ranged
// @Field: Hlth: Per-node health bitmask (bit N set => node in slot N has a recent range)
// @Field: ID0: Node id in slot 0 (0 if unused)
// @Field: ID1: Node id in slot 1 (0 if unused)
// @Field: ID2: Node id in slot 2 (0 if unused)
// @Field: ID3: Node id in slot 3 (0 if unused)
// @Field: D0: Range to node in slot 0 (last measured; check bit 0 of Hlth)
// @Field: D1: Range to node in slot 1 (last measured; check bit 1 of Hlth)
// @Field: D2: Range to node in slot 2 (last measured; check bit 2 of Hlth)
// @Field: D3: Range to node in slot 3 (last measured; check bit 3 of Hlth)

struct PACKED log_Ranging {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t count;    // number of tracked nodes
    uint8_t health;   // per-node health bitmask (bit i = slot i healthy)
    uint8_t id0;      // node id per slot
    uint8_t id1;
    uint8_t id2;
    uint8_t id3;
    float dist0;      // last measured range per slot (m)
    float dist1;
    float dist2;
    float dist3;
};

#define LOG_STRUCTURE_FROM_RANGING \
    { LOG_RANGING_MSG, sizeof(log_Ranging), \
        "TWR", "QBBBBBBffff", "TimeUS,Cnt,Hlth,ID0,ID1,ID2,ID3,D0,D1,D2,D3", "s------mmmm", "F------0000", true },
