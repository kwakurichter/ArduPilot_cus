/* SPDX-License-Identifier: GPL-3.0-or-later */
#pragma once
#include "AP_Ranging_Backend.h"
#if AP_RANGING_ENABLED && CONFIG_HAL_BOARD == HAL_BOARD_SITL
class AP_Ranging_SITL : public AP_Ranging_Backend
{
public:
    explicit AP_Ranging_SITL(AP_Ranging &frontend) : AP_Ranging_Backend(frontend) {}
    void update() override;
    bool healthy() override { return _have_sample && AP_HAL::millis() - _last_ms < AP_RANGING_TIMEOUT_MS; }
private:
    uint8_t _next = 0;
    uint32_t _last_ms = 0;
    bool _have_sample = false;
};
#endif
