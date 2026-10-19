/* SPDX-License-Identifier: GPL-3.0-or-later */
#include "AP_Beacon_Ranging.h"
#if AP_BEACON_RANGING_ENABLED
#include "AP_Beacon_Ranging_Solver.h"
#include <AP_Ranging/AP_Ranging.h>

AP_Beacon_Ranging::AP_Beacon_Ranging(AP_Beacon &frontend) : AP_Beacon_Backend(frontend, false)
{
    static_assert(AP_BEACON_MAX_BEACONS >= AP_RANGING_MAX_ANCHORS, "Ranging needs eight beacon slots");
    _origin = Vector3f(get_beacon_origin_lat(), get_beacon_origin_lon(), get_beacon_origin_alt());
    const auto *ranging = AP::ranging();
    if (ranging == nullptr || ranging->mode() != AP_Ranging::Mode::Anchor) {
        return;
    }
    for (uint8_t i = 0; i < AP_RANGING_MAX_ANCHORS; i++) {
        if (ranging->anchor(i).enabled) {
            _map[_count] = i;
            set_beacon_position(_count++, ranging->anchor(i).position);
        }
    }
}

void AP_Beacon_Ranging::update()
{
    const auto *ranging = AP::ranging();
    Location origin;
    const Vector3f configured_origin(get_beacon_origin_lat(), get_beacon_origin_lon(), get_beacon_origin_alt());
    if (_origin.is_nan() || _origin.is_inf() || fabsf(_origin.x) > 90 || fabsf(_origin.y) > 180 ||
        configured_origin != _origin || !_frontend.get_origin(origin) || !has_ned_orientation() ||
        ranging == nullptr || !ranging->configuration_valid() || ranging->configuration_changed() ||
        ranging->mode() != AP_Ranging::Mode::Anchor || _count < 4) {
        _healthy = false;
        invalidate_ranging_data();
        return;
    }
    AP_Ranging_Multilateration::Observation observations[8] {};
    uint8_t n = 0;
    uint8_t mask = 0;
    uint32_t oldest_age = 0;
    uint32_t newest_age = UINT32_MAX;
    bool changed = false;
    const uint32_t now = AP_HAL::millis();
    for (uint8_t i = 0; i < _count; i++) {
        const uint8_t id = _map[i];
        const auto &a = ranging->anchor(id);
        changed |= _samples[i] != a.sample;
        _samples[i] = a.sample;
        if (!ranging->anchor_healthy(id)) {
            continue;
        }
        mask |= 1U << i;
        oldest_age = MAX(oldest_age, now - a.time_ms);
        newest_age = MIN(newest_age, now - a.time_ms);
        for (uint8_t k = 0; k < 3; k++) {
            observations[n].position[k] = a.position[k];
        }
        observations[n++].range = a.distance;
    }
    changed |= mask != _fresh_mask;
    _fresh_mask = mask;
    if (n < 4 || oldest_age - newest_age > 150) {
        _healthy = false;
        invalidate_ranging_data();
        return;
    }
    if (!changed) {
        return;
    }
    float position[3];
    float accuracy;
    if (!AP_Ranging_Multilateration::solve(observations, n, hypotf(ranging->anchor_noise(), (oldest_age - newest_age) * 0.003f), position, accuracy)) {
        _healthy = false;
        invalidate_ranging_data();
        return;
    }
    _healthy = true;
    // Clear anchors omitted from this solution; never refresh their timestamps.
    invalidate_ranging_data();
    for (uint8_t i = 0; i < _count; i++) {
        if ((mask & (1U << i)) != 0) {
            const auto &a = ranging->anchor(_map[i]);
            set_beacon_distance_at(i, a.distance, a.time_ms);
        }
    }
    const Vector3f p {float(position[0]), float(position[1]), float(position[2])};
    set_vehicle_position_at(p, float(accuracy), now - oldest_age);
}
#endif
