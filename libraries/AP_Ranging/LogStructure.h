#pragma once

#include <AP_Logger/LogStructure.h>

// NOTE: LOG_RANGING_MSG must be registered in the shared enum in
// libraries/AP_Logger/LogStructure.h, and LOG_IDS_FROM_RANGING /
// LOG_STRUCTURE_FROM_RANGING referenced from that file's aggregate macros,
// before this message will log. (The old library used LOG_IDS_FROM_BEACON.)
#define LOG_IDS_FROM_RANGING \
    LOG_RANGING_MSG

// @LoggerMessage: TWR
// @Description: UWB two-way-ranging information
// @Field: TimeUS: Time since system startup
// @Field: Health: True if the ranging sensor is healthy
// @Field: Cnt: Number of peer nodes being ranged
// @Field: D0: Range to first node
// @Field: D1: Range to second node
// @Field: D2: Range to third node
// @Field: D3: Range to fourth node

struct PACKED log_Ranging {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t health;
    uint8_t count;
    float dist0;
    float dist1;
    float dist2;
    float dist3;
};

#define LOG_STRUCTURE_FROM_RANGING \
    { LOG_RANGING_MSG, sizeof(log_Ranging), \
        "TWR", "QBBffff", "TimeUS,Health,Cnt,D0,D1,D2,D3", "s--mmmm", "F--0000", true },
