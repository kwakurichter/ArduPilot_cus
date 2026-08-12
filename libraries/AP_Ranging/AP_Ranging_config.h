#pragma once

#include <AP_HAL/AP_HAL_Boards.h>

#ifndef AP_RANGING_ENABLED
#define AP_RANGING_ENABLED 1
#endif

// maximum number of peer nodes we can hold a range to at once
#ifndef AP_RANGING_MAX_NODES
#define AP_RANGING_MAX_NODES 4
#endif

// a node is considered unhealthy if we have not received a range update
// from it within this many milliseconds
#ifndef AP_RANGING_TIMEOUT_MS
#define AP_RANGING_TIMEOUT_MS 300
#endif

#ifndef AP_RANGING_BACKEND_DEFAULT_ENABLED
#define AP_RANGING_BACKEND_DEFAULT_ENABLED AP_RANGING_ENABLED
#endif

#ifndef AP_RANGING_DW1000_ENABLED
#define AP_RANGING_DW1000_ENABLED AP_RANGING_BACKEND_DEFAULT_ENABLED
#endif

/*
  MAV_TUNNEL_PAYLOAD_TYPE for the forwarded peer table. 200-212 are taken by
  STORM32 and ModalAI; this is an unallocated value rather than a registered
  one, so it could collide if the enum is ever extended over it.
 */
#ifndef AP_RANGING_TUNNEL_PAYLOAD_TYPE
#define AP_RANGING_TUNNEL_PAYLOAD_TYPE 220
#endif
