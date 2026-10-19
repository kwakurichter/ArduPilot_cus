/* SPDX-License-Identifier: GPL-3.0-or-later */
#include "AP_Ranging_SITL.h"
#if AP_RANGING_ENABLED && CONFIG_HAL_BOARD == HAL_BOARD_SITL
#include <AP_Beacon/AP_Beacon.h>
#include <SITL/SITL.h>

void AP_Ranging_SITL::update()
{
#if AP_BEACON_ENABLED
    const uint32_t now = AP_HAL::millis();
    const auto *sitl = AP::sitl();
    const auto *beacon = AP::beacon();
    Location origin;
    if (get_mode() != AP_Ranging::Mode::Anchor || now - _last_ms < 10 ||
        sitl == nullptr || beacon == nullptr || !beacon->get_origin(origin)) {
        return;
    }
    for (uint8_t i = 0; i < AP_RANGING_MAX_ANCHORS; i++) {
        const uint8_t index = _next;
        _next = (_next + 1) % AP_RANGING_MAX_ANCHORS;
        if (!anchor_enabled(index) || (_frontend.sim_mask() & (1U << index)) == 0) {
            continue;
        }
        const Location loc {int32_t(sitl->state.latitude * 1.0e7), int32_t(sitl->state.longitude * 1.0e7),
                            int32_t(sitl->state.altitude * 100), Location::AltFrame::ABSOLUTE};
        const Vector2f ne = origin.get_distance_NE(loc);
        const Vector3f position(ne.x, ne.y, (origin.alt - loc.alt) * 0.01f);
        const float distance = (position - _frontend.anchor(index).position).length();
        set_anchor_distance(index + 1, distance, now);
        _last_ms = now;
        _have_sample = true;
        break;
    }
#endif
}
#endif
