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
#include <AP_SwarmMesh/AP_SwarmMesh_config.h>
#if AP_SWARMMESH_ENABLED
#include <AP_SwarmMesh/AP_SwarmMesh.h>
#endif
#if AP_SIM_COOP_SENSORS_ENABLED
#include <SITL/SITL.h>
#endif

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

    // Flow: the PMW3901 reports motion counts; angle = C * counts with the 2x2 calibration C
    // (sensor axes, rad/count) and covariance C * (COOP_FLOW_SD^2 + 1/12) * C^T, the 1/12
    // being the count quantisation. Defaults: the thesis simulator's synthetic calibration.

    // @Param: _FLOW_C11
    // @DisplayName: Flow calibration, row 1 column 1
    // @Description: Element (1,1) of the matrix that converts PMW3901 motion counts to angle in the sensor axes.
    // @Units: rad
    // @User: Advanced
    AP_GROUPINFO("_FLOW_C11", 2, AP_CoopBridge, _flow_c[0], 0.001f),

    // @Param: _FLOW_C12
    // @DisplayName: Flow calibration, row 1 column 2
    // @Description: Element (1,2) of the matrix that converts PMW3901 motion counts to angle in the sensor axes.
    // @Units: rad
    // @User: Advanced
    AP_GROUPINFO("_FLOW_C12", 3, AP_CoopBridge, _flow_c[1], 0.00002f),

    // @Param: _FLOW_C21
    // @DisplayName: Flow calibration, row 2 column 1
    // @Description: Element (2,1) of the matrix that converts PMW3901 motion counts to angle in the sensor axes.
    // @Units: rad
    // @User: Advanced
    AP_GROUPINFO("_FLOW_C21", 4, AP_CoopBridge, _flow_c[2], 0.0f),

    // @Param: _FLOW_C22
    // @DisplayName: Flow calibration, row 2 column 2
    // @Description: Element (2,2) of the matrix that converts PMW3901 motion counts to angle in the sensor axes.
    // @Units: rad
    // @User: Advanced
    AP_GROUPINFO("_FLOW_C22", 5, AP_CoopBridge, _flow_c[3], 0.0011f),

    // @Param: _FLOW_SD
    // @DisplayName: Flow count noise
    // @Description: Standard deviation of the PMW3901 motion counts per axis, without the quantisation (added by the bridge).
    // @Range: 0 10
    // @User: Advanced
    AP_GROUPINFO("_FLOW_SD", 6, AP_CoopBridge, _flow_sd, 0.5f),

    // @Param: _TOF_SD
    // @DisplayName: ToF range noise
    // @Description: Standard deviation of one VL53L1X range.
    // @Units: m
    // @Range: 0.001 0.5
    // @User: Advanced
    AP_GROUPINFO("_TOF_SD", 7, AP_CoopBridge, _tof_sd, 0.015f),

    // @Param: _BARO_SD
    // @DisplayName: Baro pressure noise
    // @Description: Standard deviation of one baro reading (all its conversions together).
    // @Units: Pa
    // @Range: 0.01 20
    // @User: Advanced
    AP_GROUPINFO("_BARO_SD", 8, AP_CoopBridge, _baro_sd, 1.0607f),

    // @Param: _MAG_SD
    // @DisplayName: Magnetometer direction noise
    // @Description: Standard deviation of the field direction per tangent axis.
    // @Units: rad
    // @Range: 0.001 0.5
    // @User: Advanced
    AP_GROUPINFO("_MAG_SD", 9, AP_CoopBridge, _mag_sd, 0.008f),

    // @Param: _MAG_NORM
    // @DisplayName: Surveyed field strength
    // @Description: Strength of the undisturbed magnetic field at the site, in the compass driver's units (milligauss). The companion gates readings whose norm departs from it. 0 sends a norm ratio of 1 (gate disabled).
    // @Units: mGauss
    // @Range: 0 1000
    // @User: Advanced
    AP_GROUPINFO("_MAG_NORM", 10, AP_CoopBridge, _mag_norm, 0.0f),

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
    // readings first: each must reach the companion before the IMU data that moves its
    // clock past the reading's end
    send_aids();
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

bool AP_CoopBridge::push_flow(const AP_CoopReading::Flow &r)
{
    if (_port.get() < 0 || !_flow_q.push(r)) {
        _stats.aids_dropped++;
        return false;
    }
    return true;
}

bool AP_CoopBridge::push_tof(const AP_CoopReading::Tof &r)
{
    if (_port.get() < 0 || !_tof_q.push(r)) {
        _stats.aids_dropped++;
        return false;
    }
    return true;
}

bool AP_CoopBridge::push_baro(const AP_CoopReading::Baro &r)
{
    if (_port.get() < 0 || !_baro_q.push(r)) {
        _stats.aids_dropped++;
        return false;
    }
    return true;
}

bool AP_CoopBridge::push_camera(const AP_CoopReading::Camera &r)
{
    if (_port.get() < 0 || !_camera_q.push(r)) {
        _stats.aids_dropped++;
        return false;
    }
    return true;
}

bool AP_CoopBridge::push_mag(const AP_CoopReading::Mag &r)
{
    if (_port.get() < 0 || !_mag_q.push(r)) {
        _stats.aids_dropped++;
        return false;
    }
    return true;
}

void AP_CoopBridge::send_aids()
{
    AP_CoopReading::Flow flow;
    while (_flow_q.pop(flow)) {
        send_flow(flow);
    }
    AP_CoopReading::Tof tof;
    while (_tof_q.pop(tof)) {
        send_tof(tof);
    }
    AP_CoopReading::Baro baro;
    while (_baro_q.pop(baro)) {
        send_baro(baro);
    }
    AP_CoopReading::Mag mag;
    while (_mag_q.pop(mag)) {
        send_mag(mag);
    }
#if AP_SIM_COOP_SENSORS_ENABLED
    sitl_camera();
#endif
    AP_CoopReading::Camera camera;
    while (_camera_q.pop(camera)) {
        send_camera(camera);
    }
#if AP_RANGING_ENABLED
    // UWB exchanges come straight from AP_Ranging's ring (this is its only consumer)
    AP_Ranging *ranging = AP::ranging();
    if (ranging != nullptr) {
        AP_Ranging::Exchange e;
        while (ranging->pop_exchange(e)) {
            send_uwb(e);
        }
    }
#endif
}

#if AP_RANGING_ENABLED
void AP_CoopBridge::send_uwb(const AP_Ranging::Exchange &e)
{
    const mavlink_channel_t chan = mavlink_channel_t(_chan);
    if (!HAVE_PAYLOAD_SPACE(chan, COOP_UWB)) {
        _stats.aids_dropped++;
        return;
    }
    mavlink_coop_uwb_t pkt {};
    pkt.time_usec = e.start_us;
    pkt.end_after_capture_us = uint16_t(MIN(e.duration_us, uint32_t(UINT16_MAX)));
    pkt.range = e.range_m;
    pkt.variance = e.variance_m2;
    pkt.rx_power = e.rx_power_dbm;
    pkt.first_path_power = e.fp_power_dbm;
    pkt.initiator = e.initiator;
    pkt.responder = e.responder;
    pkt.flags = COOP_SAMPLE_VALID | (e.power_valid ? COOP_SAMPLE_POWER_VALID : 0);
    mavlink_msg_coop_uwb_send_struct(chan, &pkt);
    _stats.aids_sent++;
}
#endif

#if AP_SIM_COOP_SENSORS_ENABLED
void AP_CoopBridge::sitl_camera()
{
    // The HM01B0 marker frontend does not exist yet; in SITL the sensor model's sparse
    // position fixes (SIM_COOP_CAM_*) take its place on the same path. Called from the main
    // thread, the queue's only consumer.
    SITL::SIM *sitl = AP::sitl();
    if (sitl == nullptr) {
        return;
    }
    SITL::CoopSensors::Camera c;
    while (sitl->models.coop_sensors.pop_camera(c)) {
        AP_CoopReading::Camera r {};
        r.capture_us = c.capture_us;
        r.receipt_after_capture_us = uint32_t(c.receipt_us - c.capture_us);
        r.end_after_capture_us = 0;
        for (uint8_t i = 0; i < 4; i++) {
            r.q[i] = c.q[i];
        }
        r.position_m = c.position_m;
        // position only: the rotation carries (almost) no information
        const float rot_var = sq(SITL::CoopSensors::CAMERA_ROTATION_SD);
        const float pos_var = sq(c.sigma_m);
        const float diag[6] {rot_var, rot_var, rot_var, pos_var, pos_var, pos_var};
        uint8_t k = 0;
        for (uint8_t row = 0; row < 6; row++) {
            for (uint8_t col = row; col < 6; col++) {
                r.covariance[k++] = row == col ? diag[row] : 0.0f;
            }
        }
        r.valid = true;
        push_camera(r);
    }
}
#endif  // AP_SIM_COOP_SENSORS_ENABLED

void AP_CoopBridge::send_camera(const AP_CoopReading::Camera &r)
{
    const mavlink_channel_t chan = mavlink_channel_t(_chan);
    if (!HAVE_PAYLOAD_SPACE(chan, COOP_CAMERA)) {
        _stats.aids_dropped++;
        return;
    }
    mavlink_coop_camera_t pkt {};
    pkt.time_usec = r.capture_us;
    pkt.receipt_after_capture_us = r.receipt_after_capture_us;
    pkt.end_after_capture_us = r.end_after_capture_us;
    for (uint8_t i = 0; i < 4; i++) {
        pkt.q[i] = r.q[i];
    }
    pkt.position[0] = r.position_m.x;
    pkt.position[1] = r.position_m.y;
    pkt.position[2] = r.position_m.z;
    memcpy(pkt.covariance, r.covariance, sizeof(pkt.covariance));
    pkt.marker_id = r.marker_id;
    pkt.flags = r.valid ? COOP_SAMPLE_VALID : 0;
    mavlink_msg_coop_camera_send_struct(chan, &pkt);
    _stats.aids_sent++;
}

void AP_CoopBridge::send_flow(const AP_CoopReading::Flow &r)
{
    const mavlink_channel_t chan = mavlink_channel_t(_chan);
    if (!HAVE_PAYLOAD_SPACE(chan, COOP_FLOW)) {
        _stats.aids_dropped++;
        return;
    }
    mavlink_coop_flow_t pkt {};
    pkt.time_end_usec = r.t_end_us;
    pkt.mid_before_end_us = r.mid_before_end_us;
    pkt.duration = r.duration_s;
    pkt.quality = r.quality;
    pkt.flags = r.valid ? COOP_SAMPLE_VALID : 0;
    if (r.valid) {
        // angle = C * counts, covariance = s2 * C * C^T
        const float c11 = _flow_c[0], c12 = _flow_c[1], c21 = _flow_c[2], c22 = _flow_c[3];
        const float x = r.delta_counts[0], y = r.delta_counts[1];
        pkt.angle[0] = c11 * x + c12 * y;
        pkt.angle[1] = c21 * x + c22 * y;
        const float s2 = sq(_flow_sd.get()) + 1.0f / 12.0f;
        pkt.covariance[0] = s2 * (c11 * c11 + c12 * c12);
        pkt.covariance[1] = s2 * (c11 * c21 + c12 * c22);
        pkt.covariance[2] = s2 * (c21 * c21 + c22 * c22);
    } else {
        pkt.covariance[0] = pkt.covariance[2] = 1.0f;
    }
    mavlink_msg_coop_flow_send_struct(chan, &pkt);
    _stats.aids_sent++;
}

void AP_CoopBridge::send_tof(const AP_CoopReading::Tof &r)
{
    const mavlink_channel_t chan = mavlink_channel_t(_chan);
    if (!HAVE_PAYLOAD_SPACE(chan, COOP_TOF)) {
        _stats.aids_dropped++;
        return;
    }
    mavlink_coop_tof_t pkt {};
    pkt.time_end_usec = r.t_end_us;
    pkt.mid_before_end_us = r.mid_before_end_us;
    pkt.range = r.range_m;
    pkt.variance = sq(_tof_sd.get());
    pkt.range_status = r.range_status;
    pkt.flags = r.valid ? COOP_SAMPLE_VALID : 0;
    mavlink_msg_coop_tof_send_struct(chan, &pkt);
    _stats.aids_sent++;
}

void AP_CoopBridge::send_baro(const AP_CoopReading::Baro &r)
{
    const mavlink_channel_t chan = mavlink_channel_t(_chan);
    if (!HAVE_PAYLOAD_SPACE(chan, COOP_BARO)) {
        _stats.aids_dropped++;
        return;
    }
    mavlink_coop_baro_t pkt {};
    pkt.time_end_usec = r.t_end_us;
    pkt.mid_before_end_us = r.mid_before_end_us;
    pkt.pressure = r.pressure_pa;
    pkt.variance = sq(_baro_sd.get());
    pkt.temperature = r.temperature_c;
    pkt.flags = r.valid ? COOP_SAMPLE_VALID : 0;
    mavlink_msg_coop_baro_send_struct(chan, &pkt);
    _stats.aids_sent++;
}

void AP_CoopBridge::send_mag(const AP_CoopReading::Mag &r)
{
    const mavlink_channel_t chan = mavlink_channel_t(_chan);
    if (!HAVE_PAYLOAD_SPACE(chan, COOP_MAG)) {
        _stats.aids_dropped++;
        return;
    }
    mavlink_coop_mag_t pkt {};
    pkt.time_usec = r.capture_us;
    pkt.end_after_capture_us = r.end_after_capture_us;
    const float norm = r.field_frd.length();
    const bool valid = r.valid && is_positive(norm);
    if (valid) {
        // unit direction, body FRD to body FLU
        const Vector3f d = r.field_frd / norm;
        pkt.direction[0] = d.x;
        pkt.direction[1] = -d.y;
        pkt.direction[2] = -d.z;
        pkt.norm_ratio = is_positive(_mag_norm.get()) ? norm / _mag_norm.get() : 1.0f;
    } else {
        pkt.direction[0] = 1.0f;
        pkt.norm_ratio = 1.0f;
    }
    // isotropic tangent covariance: the same in any basis of the tangent plane
    const float s2 = sq(_mag_sd.get());
    pkt.covariance[0] = s2;
    pkt.covariance[2] = s2;
    pkt.flags = valid ? COOP_SAMPLE_VALID : 0;
    mavlink_msg_coop_mag_send_struct(chan, &pkt);
    _stats.aids_sent++;
}

void AP_CoopBridge::handle_coop_peer(mavlink_channel_t chan, const mavlink_message_t &msg)
{
    if (!resolve_channel() || chan != mavlink_channel_t(_chan)) {
        return;     // only the companion's own frames go to the mesh
    }
    _stats.peer_in++;
#if AP_SWARMMESH_ENABLED
    AP_SwarmMesh *mesh = AP::swarmmesh();
    if (mesh != nullptr) {
        // the frame exactly as the companion sent it (header, payload, CRC), so peers'
        // companions see the sender's own system ID and sequence
        uint8_t frame[MAVLINK_MAX_PACKET_LEN];
        const uint16_t len = mavlink_msg_to_send_buffer(frame, &msg);
        const uint8_t destination = mavlink_msg_coop_peer_get_destination(&msg);
        const uint16_t deadline_ms = mavlink_msg_coop_peer_get_deadline_ms(&msg);
        if (mesh->send_critical(frame, len, destination, deadline_ms)) {
            return;
        }
    }
#endif
    _stats.peer_rejected++;
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
#if AP_SWARMMESH_ENABLED
    // the mesh side of COOP_PEER: AP_SwarmMesh's critical class
    AP_SwarmMesh *mesh = AP::swarmmesh();
    SwarmCriticalStats crit {};
    if (mesh != nullptr && mesh->get_critical_stats(crit, true)) {
        pkt.peer_tx = crit.tx;
        pkt.peer_rx = crit.rx;
        pkt.peer_tx_dropped = uint16_t(MIN(crit.tx_dropped + _stats.peer_rejected, uint32_t(UINT16_MAX)));
        pkt.peer_fwd_dropped = uint16_t(MIN(crit.fwd_dropped, uint32_t(UINT16_MAX)));
        pkt.critical_queue = crit.queued;
        pkt.critical_queue_max = crit.queued_max;
    } else {
        pkt.peer_tx_dropped = uint16_t(MIN(_stats.peer_rejected, uint32_t(UINT16_MAX)));
    }
#endif
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
    // @Field: ASent: aid messages (flow, ToF, baro, mag, camera, UWB) sent
    // @Field: ADrop: aid readings lost (queue full or port full)
    AP::logger().WriteStreaming("COOP", "TimeUS,Sent,Drop,Gap,Clip,ASent,ADrop", "s------", "F------",
                                "QIIIIII", now_us, _stats.imu_sent, _stats.imu_dropped, _stats.imu_gaps,
                                _stats.imu_clipped, _stats.aids_sent, _stats.aids_dropped);
}
#endif

namespace AP {

AP_CoopBridge *coopbridge()
{
    return AP_CoopBridge::get_singleton();
}

};

#endif  // AP_COOPBRIDGE_ENABLED
