#pragma once

#include <AP_HAL/AP_HAL_Boards.h>

// The syslink protocol only exists between the STM32 and the nRF51822 radio
// co-processor on Crazyflie 2.x, so this defaults on for those boards only.
#ifndef AP_SYSLINK_ENABLED
#ifdef HAL_CF21
#define AP_SYSLINK_ENABLED 1
#else
#define AP_SYSLINK_ENABLED 0
#endif
#endif

// GPIO number of the nRF51's UART RTS line, matching GPIO() in the hwdef.
// Reflects the nRF51's UART receive FIFO, not its radio transmit queue.
#ifndef HAL_SYSLINK_FLOWCTRL_PIN
#define HAL_SYSLINK_FLOWCTRL_PIN 62
#endif
