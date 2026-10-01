#pragma once

#include <AP_HAL/AP_HAL_Boards.h>
#include <GCS_MAVLink/GCS_config.h>

// AP_CoopBridge: the FC end of the cooperative-estimation link to a companion computer
// (Teensy 4.0 on the Crazyflie 2.1). Off by default; boards that carry the companion enable
// it in their hwdef, and SITL enables it so the bridge can be exercised without hardware.
#ifndef AP_COOPBRIDGE_ENABLED
#define AP_COOPBRIDGE_ENABLED (HAL_GCS_ENABLED && CONFIG_HAL_BOARD == HAL_BOARD_SITL)
#endif

// Length of one IMU first-stage sub-interval (us). The companion's second stage and the
// 100 ms knots assume 10 ms; changing it changes the protocol.
#ifndef AP_COOPBRIDGE_IMU_SUB_US
#define AP_COOPBRIDGE_IMU_SUB_US 10000
#endif
