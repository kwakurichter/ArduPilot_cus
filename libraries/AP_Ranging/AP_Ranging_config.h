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
