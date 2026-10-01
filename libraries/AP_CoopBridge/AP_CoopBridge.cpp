/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#include "AP_CoopBridge.h"

#if AP_COOPBRIDGE_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <AP_InertialSensor/AP_InertialSensor.h>
#include <AP_Logger/AP_Logger.h>
#include <AP_Scheduler/AP_Scheduler.h>
#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL& hal;

AP_CoopBridge *AP_CoopBridge::_singleton;

const AP_Param::GroupInfo AP_CoopBridge::var_info[] = {
    // @Param: _PORT
    // @DisplayName: Cooperative companion serial port
    // @Description: Serial port number (the n of SERIALn) of the companion computer that runs the cooperative estimator. The port must be a MAVLink2 port (SERIALn_PROTOCOL = 2). -1 disables the bridge.
    // @Range: -1 9
    // @User: Advanced
    // @RebootRequired: False
    AP_GROUPINFO_FLAGS("_PORT", 1, AP_CoopBridge, _port, -1, AP_PARAM_FLAG_ENABLE),

    AP_GROUPEND
};

AP_CoopBridge::AP_CoopBridge()
{
    AP_Param::setup_object_defaults(this, var_info);
    if (_singleton != nullptr) {
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
        AP_HAL::panic("Too many AP_CoopBridge instances");
#endif
        return;
    }
    _singleton = this;
}

void AP_CoopBridge::init()
{
    _imu.reset();
    _imu_locked = false;
    _last_tick_us = 0;
}

bool AP_CoopBridge::resolve_channel()
{
    const int8_t port = _port.get();
    if (port < 0) {
        return false;
    }
    if (port != _port_resolved) {
        // resolve once, and again whenever the parameter changes
        _port_resolved = port;
        _chan = gcs().get_channel_from_port_number(uint8_t(port));
    }
    return _chan != UINT8_MAX;
}

void AP_CoopBridge::update_fast()
{
    if (!resolve_channel()) {
        _last_tick_us = 0;   // restart cleanly when the port comes back
        return;
    }
    const AP_InertialSensor &ins = AP::ins();
    if (!_imu_locked) {
        // one instance for both gyro and accel, kept for the whole flight: a different IMU
        // has a different bias, which the companion would see as a jump
        _imu_instance = ins.get_first_usable_gyro();
        _last_clip_count = ins.get_accel_clip_count(_imu_instance);
        _imu_locked = true;
    }

    // The tick ends now, right after the INS update; it starts where the previous one ended.
    // The constant delay between the INS samples and this call does not matter: every message
    // is stamped with the same clock.
    const uint64_t now_us = AP_HAL::micros64();
    if (_last_tick_us == 0) {
        _last_tick_us = now_us;
        return;
    }
    const uint64_t start_us = _last_tick_us;
    _last_tick_us = now_us;

    Vector3f delta_angle, delta_velocity;
    float angle_dt, velocity_dt;
    uint8_t flags = COOP_SAMPLE_VALID;
    if (!ins.get_delta_angle(_imu_instance, delta_angle, angle_dt) ||
        !ins.get_delta_velocity(_imu_instance, delta_velocity, velocity_dt) ||
        !ins.get_gyro_health(_imu_instance) || !ins.get_accel_health(_imu_instance)) {
        // no data for this tick: integrate nothing and say so
        delta_angle.zero();
        delta_velocity.zero();
        flags |= COOP_SAMPLE_GAP;
    }
    // A tick much longer than the loop period means the scheduler or the INS stalled; the
    // INS deltas then cover less time than the tick.
    const float tick_s = (now_us - start_us) * 1.0e-6f;
    if (tick_s > 2.0f * AP::scheduler().get_loop_period_s()) {
        flags |= COOP_SAMPLE_GAP;
    }
    const uint32_t clip_count = ins.get_accel_clip_count(_imu_instance);
    if (clip_count != _last_clip_count) {
        flags |= COOP_SAMPLE_CLIPPED;
        _last_clip_count = clip_count;
    }
    if (flags & COOP_SAMPLE_GAP) {
        _stats.imu_gaps++;
    }
    if (flags & COOP_SAMPLE_CLIPPED) {
        _stats.imu_clipped++;
    }

    // body FRD (INS) to body FLU (estimator): a rotation by 180 degrees about x, so rotation
    // vectors and velocities transform alike
    delta_angle.y = -delta_angle.y;
    delta_angle.z = -delta_angle.z;
    delta_velocity.y = -delta_velocity.y;
    delta_velocity.z = -delta_velocity.z;

    // samples in this tick, from the INS sample rates (gyro + accel)
    const float samples = tick_s * (ins.get_gyro_rate_hz(_imu_instance) + ins.get_accel_rate_hz(_imu_instance));

    AP_CoopBridge_ImuStage::Sub subs[2];
    const uint16_t n = _imu.push(start_us, now_us, delta_angle, delta_velocity, samples, flags,
                                 subs, ARRAY_SIZE(subs));
    for (uint16_t i = 0; i < MIN(n, uint16_t(ARRAY_SIZE(subs))); i++) {
        send_imu(subs[i]);
    }
    if (n > ARRAY_SIZE(subs)) {
        // a stall longer than two sub-intervals: the companion restarts on the gap anyway
        _stats.imu_dropped += n - ARRAY_SIZE(subs);
    }
}

void AP_CoopBridge::send_imu(const AP_CoopBridge_ImuStage::Sub &sub)
{
    const mavlink_channel_t chan = mavlink_channel_t(_chan);
    // all or nothing: the companion detects a missing sub-interval from the time stamps
    if (!HAVE_PAYLOAD_SPACE(chan, COOP_IMU_DELTA)) {
        _stats.imu_dropped++;
        return;
    }
    mavlink_coop_imu_delta_t pkt {};
    pkt.time_start_usec = sub.t0_us;
    pkt.duration_us = sub.duration_us;
    for (uint8_t i = 0; i < 3; i++) {
        pkt.delta_angle[i] = sub.theta[i];
        pkt.delta_velocity[i] = sub.velocity[i];
        pkt.delta_position[i] = sub.position[i];
    }
    pkt.samples = sub.samples;
    pkt.imu_instance = _imu_instance;
    pkt.flags = sub.flags;
    mavlink_msg_coop_imu_delta_send_struct(chan, &pkt);
    _stats.imu_sent++;
}

void AP_CoopBridge::update()
{
    if (!resolve_channel()) {
        return;
    }
    const uint64_t now_us = AP_HAL::micros64();
    if (now_us - _last_status_us < 1000000U) {
        return;
    }
    _last_status_us = now_us;
    send_link_status(now_us);
#if HAL_LOGGING_ENABLED
    log_stats(now_us);
#endif
}

void AP_CoopBridge::send_link_status(uint64_t now_us)
{
    const mavlink_channel_t chan = mavlink_channel_t(_chan);
    if (!HAVE_PAYLOAD_SPACE(chan, COOP_LINK_STATUS)) {
        return;
    }
    mavlink_coop_link_status_t pkt {};
    pkt.time_usec = now_us;
    // The COOP_PEER counters and the critical queue come with AP_SwarmMesh's critical class.
    const mavlink_status_t *status = mavlink_get_channel_status(chan);
    if (status != nullptr) {
        pkt.uart_rx_errors = uint16_t(MIN(status->packet_rx_drop_count, uint32_t(UINT16_MAX)));
    }
    mavlink_msg_coop_link_status_send_struct(chan, &pkt);
}

#if HAL_LOGGING_ENABLED
void AP_CoopBridge::log_stats(uint64_t now_us)
{
    // @LoggerMessage: COOP
    // @Description: Cooperative companion bridge counters (cumulative)
    // @Field: TimeUS: Time since system startup
    // @Field: Sent: COOP_IMU_DELTA messages sent
    // @Field: Drop: completed IMU sub-intervals not sent (port full or stall)
    // @Field: Gap: ticks with missing or late INS data
    // @Field: Clip: ticks with accelerometer clipping
    AP::logger().WriteStreaming("COOP", "TimeUS,Sent,Drop,Gap,Clip", "s----", "F----", "QIIII",
                                now_us, _stats.imu_sent, _stats.imu_dropped, _stats.imu_gaps,
                                _stats.imu_clipped);
}
#endif

namespace AP {

AP_CoopBridge *coopbridge()
{
    return AP_CoopBridge::get_singleton();
}

};

#endif  // AP_COOPBRIDGE_ENABLED
