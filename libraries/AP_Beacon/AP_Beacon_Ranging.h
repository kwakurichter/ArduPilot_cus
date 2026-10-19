/* SPDX-License-Identifier: GPL-3.0-or-later */
#pragma once
#include "AP_Beacon_Backend.h"

#if AP_BEACON_RANGING_ENABLED
class AP_Beacon_Ranging : public AP_Beacon_Backend
{
public:
    explicit AP_Beacon_Ranging(AP_Beacon &frontend);
    bool healthy() override { return _healthy; }
    void update() override;
private:
    uint8_t _map[8] {};
    uint8_t _count = 0;
    uint8_t _fresh_mask = 0;
    uint32_t _samples[8] {};
    bool _healthy = false;
    Vector3f _origin;
};
#endif
