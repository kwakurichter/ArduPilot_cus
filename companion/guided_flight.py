#!/usr/bin/env python3
# AP_FLAKE8_CLEAN
"""Crazyradio bridge relay and opt-in GUIDED movement test.

Requires pymavlink. See guided_flight_README.txt. Without --execute this only
prints the plan. No flight commands are sent until explicitly executed.
"""

import argparse
import json
import math
from pathlib import Path
import select
import socket
import time

from pymavlink.dialects.v20 import ardupilotmega as mav

GUIDED = 4
LAND = 9
CMD_SET_GLOBAL_ORIGIN = 611  # MAV_CMD_DO_SET_GLOBAL_ORIGIN; older pymavlink omits the enum
MASK_POSITION_YAW = 0x09F8  # ignore velocity, acceleration and yaw rate
EKF_VERTICAL_REQUIRED = mav.EKF_ATTITUDE | mav.EKF_VELOCITY_VERT | mav.EKF_POS_VERT_ABS
EKF_REQUIRED = (mav.EKF_ATTITUDE | mav.EKF_VELOCITY_HORIZ | mav.EKF_VELOCITY_VERT
                | mav.EKF_POS_HORIZ_REL | mav.EKF_POS_VERT_ABS)


class FlightError(RuntimeError):
    pass


class Takeover(FlightError):
    pass


class CommandRejected(FlightError):
    def __init__(self, command, result):
        super().__init__(f'Command {command} rejected: MAV_RESULT={result}')
        self.result = result


def left_target(start, yaw, distance):
    """Fixed local-NED target left of the captured heading; never cumulative."""
    return (start[0] + distance * math.sin(yaw),
            start[1] - distance * math.cos(yaw), start[2])


def mission_targets(start, yaw, distance, pattern):
    """Targets relative to one captured heading and origin, with a return per leg."""
    legs = [('move left', left_target(start, yaw, distance)), ('return from left', start)]
    if pattern == 'three-axis':
        backward = (start[0] - distance * math.cos(yaw),
                    start[1] - distance * math.sin(yaw), start[2])
        legs.extend([('move right', left_target(start, yaw, -distance)), ('return from right', start),
                     ('move backward', backward), ('return from backward', start)])
    return legs


def endpoint(value):
    host, sep, port = value.rpartition(':')
    if not sep or not host or not 0 < int(port) < 65536:
        raise argparse.ArgumentTypeError('Expected IPv4 HOST:PORT')
    return socket.gethostbyname(host), int(port)


class Relay:
    """Keep one stable UDP peer for the supplied single-peer radio bridge.

    QGC has a separate socket, so its replies never replace the radio peer.
    Radio telemetry is parsed for the controller and forwarded unchanged.
    """

    def __init__(self, bind, qgc, source_system, target_system, log):
        self.radio = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.radio.bind(bind)
        self.qgc = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.qgc.bind(('127.0.0.1', 0))
        self.qgc_peer = qgc
        self.bridge_peer = None
        self.target_system = target_system
        self.source_system = source_system
        self.log = log
        self.cache = {}
        self.acks = []
        self.takeover = False
        self.watch_takeover = False
        self.last_heartbeat = 0
        self.heartbeat_enabled = True
        self.mav = mav.MAVLink(self, srcSystem=source_system, srcComponent=191)
        self.decoder = mav.MAVLink(None)
        self.decoder.robust_parsing = True
        self.qgc_decoder = mav.MAVLink(None)
        self.qgc_decoder.robust_parsing = True

    def write(self, data):
        if self.bridge_peer is None:
            raise FlightError('No bridge telemetry received')
        self.radio.sendto(data, self.bridge_peer)

    def qgc_command(self, message):
        if not self.watch_takeover:
            return
        if getattr(message, 'target_system', None) not in (0, self.target_system):
            return
        kind = message.get_type()
        if kind in ('SET_MODE', 'SET_POSITION_TARGET_LOCAL_NED', 'SET_POSITION_TARGET_GLOBAL_INT',
                    'SET_ATTITUDE_TARGET'):
            self.takeover = True
        if kind in ('COMMAND_LONG', 'COMMAND_INT') and message.command in (
                mav.MAV_CMD_DO_SET_MODE, mav.MAV_CMD_NAV_LAND, mav.MAV_CMD_NAV_RETURN_TO_LAUNCH,
                mav.MAV_CMD_COMPONENT_ARM_DISARM, mav.MAV_CMD_NAV_TAKEOFF):
            self.takeover = True

    def pump(self, timeout=0.05):
        for sock in select.select([self.radio, self.qgc], [], [], timeout)[0]:
            data, peer = sock.recvfrom(65535)
            if sock is self.qgc:
                if peer != self.qgc_peer:
                    continue
                for message in self.qgc_decoder.parse_buffer(data) or []:
                    self.qgc_command(message)
                if self.bridge_peer:
                    self.radio.sendto(data, self.bridge_peer)
                continue
            # Pin the bridge's ephemeral source port for the entire run.
            if self.bridge_peer is None:
                if peer[0] not in ('127.0.0.1', '::1'):
                    continue
                self.bridge_peer = peer
                self.log('bridge_connected', peer=peer)
            if peer != self.bridge_peer:
                continue
            self.qgc.sendto(data, self.qgc_peer)
            for message in self.decoder.parse_buffer(data) or []:
                if message.get_srcSystem() != self.target_system or message.get_srcComponent() != 1:
                    continue
                now = time.monotonic()
                kind = message.get_type()
                self.cache[kind] = (now, message)
                if kind == 'COMMAND_ACK':
                    if (getattr(message, 'target_system', 0) in (0, self.source_system)
                            and getattr(message, 'target_component', 0) in (0, 191)):
                        self.acks.append((now, message))
                        self.acks = self.acks[-100:]
                if kind in ('LOCAL_POSITION_NED', 'ATTITUDE', 'HEARTBEAT', 'EXTENDED_SYS_STATE',
                            'EKF_STATUS_REPORT', 'SYS_STATUS', 'STATUSTEXT', 'COMMAND_ACK', 'HOME_POSITION',
                            'GPS_GLOBAL_ORIGIN'):
                    self.log('telemetry', message=message.to_dict())
                if kind == 'STATUSTEXT':
                    print('Vehicle:', message.text, flush=True)
        now = time.monotonic()
        if self.bridge_peer and self.heartbeat_enabled and now - self.last_heartbeat >= 1:
            self.mav.heartbeat_send(mav.MAV_TYPE_GCS, mav.MAV_AUTOPILOT_INVALID, 0, 0, 0)
            self.last_heartbeat = now

    def fresh(self, kind, age=2):
        when, message = self.cache.get(kind, (0, None))
        return message if time.monotonic() - when <= age else None

    def close(self):
        self.radio.close()
        self.qgc.close()


class Flight:
    def __init__(self, relay, args, log):
        self.link = relay
        self.args = args
        self.log = log
        self.armed_by_script = False
        self.monitor_mode = False
        self.expected_modes = {GUIDED}

    def guard(self, position=False, horizontal=True):
        if self.link.takeover:
            raise Takeover('QGC sent a flight command; yielding control')
        hb = self.link.fresh('HEARTBEAT', 3)
        if hb is None:
            raise FlightError('Vehicle heartbeat stale')
        if self.monitor_mode and hb.custom_mode not in self.expected_modes:
            raise Takeover(f'Flight mode changed to {hb.custom_mode}; yielding control')
        if self.monitor_mode and not hb.base_mode & mav.MAV_MODE_FLAG_SAFETY_ARMED:
            raise Takeover('Vehicle disarmed; stopping the sequence')
        if position:
            p = self.link.fresh('LOCAL_POSITION_NED', 1.5)
            if p is None or not all(math.isfinite(v) for v in (p.x, p.y, p.z, p.vx, p.vy, p.vz)):
                raise FlightError('Local-position telemetry stale or non-finite')
            ekf = self.link.fresh('EKF_STATUS_REPORT', 3)
            required = EKF_REQUIRED if horizontal else EKF_VERTICAL_REQUIRED
            if ekf is None or ekf.flags & required != required or ekf.flags & mav.EKF_UNINITIALIZED:
                kind = 'horizontal position' if horizontal else 'vertical state'
                raise FlightError(f'EKF no longer reports a usable {kind}')

    def wait(self, predicate, timeout, description, guard=False):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self.link.pump()
            if self.link.takeover:
                raise Takeover('QGC sent a flight command; yielding control')
            if guard:
                self.guard()
            result = predicate()
            if result:
                return result
        raise FlightError(f'Timeout: {description}')

    def send_command(self, command, *params):
        values = list(params) + [0] * (7 - len(params))
        self.log('command', command=command, params=values)
        self.link.mav.command_long_send(self.args.target_system, 1, command, 0, *values)

    def command(self, command, *params):
        # Flight-changing commands are not blindly retried after an ambiguous ACK.
        started = time.monotonic()
        self.send_command(command, *params)
        self.wait_ack(command, started)

    def wait_ack(self, command, started):
        def ack():
            for when, message in self.link.acks:
                if when >= started and message.command == command:
                    if message.result == mav.MAV_RESULT_IN_PROGRESS:
                        continue
                    if message.result != mav.MAV_RESULT_ACCEPTED:
                        raise CommandRejected(command, message.result)
                    return True
            return False

        self.wait(ack, 5, f'ACK for command {command}', guard=self.monitor_mode)

    def command_origin(self):
        started = time.monotonic()
        self.log('command_int', command=CMD_SET_GLOBAL_ORIGIN, latitude=self.args.lat,
                 longitude=self.args.lon, altitude_m=self.args.amsl)
        self.link.mav.command_int_send(
            self.args.target_system, 1, mav.MAV_FRAME_GLOBAL, CMD_SET_GLOBAL_ORIGIN,
            0, 0, 0, 0, 0, 0, round(self.args.lat * 1e7), round(self.args.lon * 1e7), self.args.amsl)
        self.wait_ack(CMD_SET_GLOBAL_ORIGIN, started)

    def command_home(self, origin):
        ground = self.link.fresh('LOCAL_POSITION_NED', 1)
        if ground is None or not math.isfinite(ground.z):
            raise FlightError('Local ground height unavailable for home setup')
        altitude_m = origin.altitude / 1000 - ground.z
        # Use the indoor origin as the horizontal home reference. "Use current"
        # requires a valid horizontal EKF location, unavailable on this deck at rest.
        started = time.monotonic()
        self.link.cache.pop('HOME_POSITION', None)
        self.log('command_int', command=mav.MAV_CMD_DO_SET_HOME, latitude=origin.latitude / 1e7,
                 longitude=origin.longitude / 1e7, altitude_m=altitude_m)
        self.link.mav.command_int_send(
            self.args.target_system, 1, mav.MAV_FRAME_GLOBAL, mav.MAV_CMD_DO_SET_HOME,
            0, 0, 0, 0, 0, 0, origin.latitude, origin.longitude, altitude_m)
        self.wait_ack(mav.MAV_CMD_DO_SET_HOME, started)

    def prearm_checks(self):
        deadline = time.monotonic() + 30
        next_check = 0
        while time.monotonic() < deadline:
            self.link.pump()
            self.guard()
            if time.monotonic() >= next_check:
                # ACCEPTED means checks were run, not that they passed.
                self.command(mav.MAV_CMD_RUN_PREARM_CHECKS)
                next_check = time.monotonic() + 2
            state = self.link.fresh('SYS_STATUS', 2)
            if state and state.onboard_control_sensors_health & mav.MAV_SYS_STATUS_PREARM_CHECK:
                return
        raise FlightError('Prearm checks still failing after 30s; ARM was not sent')

    def arm(self):
        deadline = time.monotonic() + 8
        while time.monotonic() < deadline:
            self.guard()
            hb = self.link.fresh('HEARTBEAT', 3)
            if hb.custom_mode != GUIDED:
                raise Takeover('Vehicle left GUIDED before arming')
            if hb.base_mode & mav.MAV_MODE_FLAG_SAFETY_ARMED:
                raise Takeover('Vehicle was armed by another controller')
            self.armed_by_script = True
            try:
                self.command(mav.MAV_CMD_COMPONENT_ARM_DISARM, 1, 0)
            except CommandRejected as exc:
                if exc.result not in (mav.MAV_RESULT_TEMPORARILY_REJECTED, mav.MAV_RESULT_FAILED):
                    raise
                self.log('arm_retry', result=exc.result)
                end = time.monotonic() + 0.5
                while time.monotonic() < end:
                    self.link.pump()
                    self.guard()
                continue
            except Takeover:
                raise
            except FlightError:
                # Lost ACK: accept an observed state change, never blindly resend.
                hb = self.link.fresh('HEARTBEAT', 3)
                if hb and hb.custom_mode == GUIDED and hb.base_mode & mav.MAV_MODE_FLAG_SAFETY_ARMED:
                    return
                raise
            self.wait(lambda: (self.link.fresh('HEARTBEAT', 3) is not None
                               and self.link.fresh('HEARTBEAT', 3).base_mode & mav.MAV_MODE_FLAG_SAFETY_ARMED),
                      5, 'armed confirmation')
            return
        raise FlightError('Arming rejected throughout the retry window')

    def request(self, message_id):
        self.send_command(mav.MAV_CMD_REQUEST_MESSAGE, message_id)

    def setup_origin(self):
        self.request(mav.MAVLINK_MSG_ID_GPS_GLOBAL_ORIGIN)
        end = time.monotonic() + 2
        while time.monotonic() < end:
            self.link.pump()
        origin = self.link.fresh('GPS_GLOBAL_ORIGIN', 5)
        if origin is None:
            for _ in range(3):
                try:
                    self.command_origin()
                except Takeover:
                    raise
                except FlightError as exc:
                    # An ACK can be lost, or another GCS can set the origin.
                    # Confirm actual state before attempting another write.
                    self.log('origin_command_unconfirmed', reason=str(exc))
                try:
                    self.request(mav.MAVLINK_MSG_ID_GPS_GLOBAL_ORIGIN)
                    origin = self.wait(lambda: self.link.fresh('GPS_GLOBAL_ORIGIN', 10), 3, 'EKF origin')
                    break
                except Takeover:
                    raise
                except FlightError:
                    continue
        if origin is None:
            raise FlightError('EKF origin was not confirmed')
        north = (origin.latitude / 1e7 - self.args.lat) * 111320
        east = (origin.longitude / 1e7 - self.args.lon) * 111320 * math.cos(math.radians(self.args.lat))
        if self.args.explicit_origin and (math.hypot(north, east) > 2
                                          or abs(origin.altitude / 1000 - self.args.amsl) > 2):
            raise FlightError('Existing EKF origin differs from requested coordinates; it was not overwritten')
        print(f'Using EKF origin {origin.latitude / 1e7}, {origin.longitude / 1e7}, '
              f'{origin.altitude / 1000}m AMSL.', flush=True)
        self.log('origin_verified', latitude=origin.latitude, longitude=origin.longitude, altitude_mm=origin.altitude)
        return origin

    def setpoint(self, target, yaw):
        self.log('setpoint', ned=target, yaw=yaw)
        self.link.mav.set_position_target_local_ned_send(
            0, self.args.target_system, 1, mav.MAV_FRAME_LOCAL_NED, MASK_POSITION_YAW,
            *target, 0, 0, 0, 0, 0, 0, yaw, 0)

    def move(self, target, yaw, label):
        print(label, tuple(round(v, 3) for v in target), flush=True)
        self.log('phase', name=label, target=target)
        deadline = time.monotonic() + self.args.move_timeout
        settled_since = None
        next_send = 0
        next_report = 0
        while time.monotonic() < deadline:
            self.link.pump()
            self.guard(position=True)
            now = time.monotonic()
            if now >= next_send:
                self.setpoint(target, yaw)
                next_send = now + 0.2
            p = self.link.fresh('LOCAL_POSITION_NED', 1.5)
            error = math.dist((p.x, p.y, p.z), target)
            speed = math.sqrt(p.vx**2 + p.vy**2 + p.vz**2)
            if error <= self.args.position_tolerance and speed <= self.args.settle_speed:
                if settled_since is None:
                    settled_since = now
                if now - settled_since >= self.args.hold:
                    self.log('settled', name=label, error_m=error, speed_ms=speed)
                    return
            else:
                settled_since = None
            if now >= next_report:
                dwell = now - settled_since if settled_since is not None else 0.0
                self.log('arrival_progress', name=label, error_m=error, speed_ms=speed,
                         dwell_s=dwell, remaining_s=max(0, deadline - now))
                print(f'{label}: error {error:.2f}/{self.args.position_tolerance:.2f}m, '
                      f'speed {speed:.2f}/{self.args.settle_speed:.2f}m/s, '
                      f'settled {dwell:.1f}/{self.args.hold:.1f}s', flush=True)
                next_report = now + 1
        raise FlightError(f'{label} did not settle within {self.args.move_timeout}s '
                          f'(requires error <= {self.args.position_tolerance}m and '
                          f'speed <= {self.args.settle_speed}m/s for {self.args.hold}s)')

    def land(self):
        self.log('phase', name='land')
        print('Landing; disarm requires confirmed touchdown.', flush=True)
        self.expected_modes = {GUIDED, LAND}
        self.command(mav.MAV_CMD_NAV_LAND)
        self.wait(lambda: self.link.fresh('HEARTBEAT', 3).custom_mode == LAND,
                  5, 'LAND mode', guard=True)
        self.expected_modes = {LAND}
        deadline = time.monotonic() + 45
        grounded_since = None
        while time.monotonic() < deadline:
            self.link.pump()
            if self.link.takeover:
                raise Takeover('QGC takeover during landing')
            hb = self.link.fresh('HEARTBEAT', 3)
            if hb is None:
                raise FlightError('Heartbeat lost during landing; disarm not sent')
            if hb.custom_mode != LAND:
                raise Takeover('Pilot changed mode during landing')
            state = self.link.fresh('EXTENDED_SYS_STATE', 2)
            grounded = state is not None and state.landed_state == mav.MAV_LANDED_STATE_ON_GROUND
            if grounded and not hb.base_mode & mav.MAV_MODE_FLAG_SAFETY_ARMED:
                self.log('complete', disarm='autopilot')
                return
            if grounded:
                if grounded_since is None:
                    grounded_since = time.monotonic()
                if time.monotonic() - grounded_since >= 2:
                    self.monitor_mode = False
                    self.command(mav.MAV_CMD_COMPONENT_ARM_DISARM, 0, 0)
                    self.wait(lambda: (self.link.fresh('HEARTBEAT', 3) is not None
                                       and not self.link.fresh('HEARTBEAT', 3).base_mode
                                       & mav.MAV_MODE_FLAG_SAFETY_ARMED), 5, 'disarm confirmation')
                    self.log('complete', disarm='normal command after touchdown')
                    return
            else:
                grounded_since = None
        raise FlightError('Landing not confirmed; disarm not sent')

    def ground_ready(self):
        p = self.link.fresh('LOCAL_POSITION_NED', 1)
        ekf = self.link.fresh('EKF_STATUS_REPORT', 2)
        ground = self.link.fresh('EXTENDED_SYS_STATE', 2)
        att = self.link.fresh('ATTITUDE', 1)
        # HAL_CF21 permits GUIDED launch before flow has a valid ground range.
        # Defer horizontal validity to the first translation, not vertical takeoff.
        return (p and ekf and ground and att
                and all(math.isfinite(v) for v in (p.x, p.y, p.z, p.vx, p.vy, p.vz, att.yaw))
                and ekf.flags & EKF_VERTICAL_REQUIRED == EKF_VERTICAL_REQUIRED
                and not ekf.flags & mav.EKF_UNINITIALIZED
                and ground.landed_state == mav.MAV_LANDED_STATE_ON_GROUND)

    def run(self):
        self.wait(lambda: self.link.fresh('HEARTBEAT', 3), 20, 'autopilot heartbeat')
        hb = self.link.fresh('HEARTBEAT', 3)
        if hb.autopilot != mav.MAV_AUTOPILOT_ARDUPILOTMEGA:
            raise FlightError('Selected system is not an ArduPilot autopilot')
        if hb.base_mode & mav.MAV_MODE_FLAG_SAFETY_ARMED:
            raise FlightError('Start with the vehicle disarmed on the ground')
        self.link.watch_takeover = True
        for msg_id, hz in [(mav.MAVLINK_MSG_ID_HEARTBEAT, 2), (mav.MAVLINK_MSG_ID_LOCAL_POSITION_NED, 10),
                           (mav.MAVLINK_MSG_ID_ATTITUDE, 5), (mav.MAVLINK_MSG_ID_EXTENDED_SYS_STATE, 2),
                           (mav.MAVLINK_MSG_ID_EKF_STATUS_REPORT, 2), (mav.MAVLINK_MSG_ID_SYS_STATUS, 2)]:
            self.command(mav.MAV_CMD_SET_MESSAGE_INTERVAL, msg_id, 1e6 / hz)
        origin = self.setup_origin()

        try:
            self.log('phase', name='wait_ground_ready')
            self.wait(self.ground_ready, 30, 'vertical estimate, local telemetry, attitude and landed state')
        except Takeover:
            raise
        except FlightError as exc:
            ekf = self.link.fresh('EKF_STATUS_REPORT', 3)
            flags = ekf.flags if ekf else None
            raise FlightError(f'{exc}; EKF flags={flags}. See missing/stale telemetry in the JSONL log') from exc
        self.command_home(origin)
        self.request(mav.MAVLINK_MSG_ID_HOME_POSITION)
        home = self.wait(lambda: self.link.fresh('HOME_POSITION', 3), 5, 'home confirmation')
        ground_pos = self.link.fresh('LOCAL_POSITION_NED', 1)
        target_z = (origin.altitude - home.altitude) / 1000 - self.args.height
        if ground_pos is None or abs(target_z - (ground_pos.z - self.args.height)) > 0.25:
            raise FlightError('Home altitude and local ground position disagree by more than 0.25m')
        attitude = self.link.fresh('ATTITUDE', 1)
        if attitude is None or not math.isfinite(attitude.yaw):
            raise FlightError('Heading unavailable')
        yaw = attitude.yaw
        self.log('home_verified', home=home.to_dict(), takeoff_target_z=target_z, yaw=yaw)
        self.command(mav.MAV_CMD_DO_SET_MODE, mav.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, GUIDED)
        self.wait(lambda: (self.link.fresh('HEARTBEAT', 3) is not None
                           and self.link.fresh('HEARTBEAT', 3).custom_mode == GUIDED), 5, 'GUIDED mode')
        self.command(mav.MAV_CMD_DO_CHANGE_SPEED, 1, self.args.speed, -1)
        self.prearm_checks()
        self.link.watch_takeover = True
        hb = self.link.fresh('HEARTBEAT', 3)
        if hb is None or hb.custom_mode != GUIDED or hb.base_mode & mav.MAV_MODE_FLAG_SAFETY_ARMED:
            raise Takeover('Vehicle state changed before arming')
        self.arm()
        self.monitor_mode = True
        self.guard(position=True, horizontal=False)
        self.log('phase', name='takeoff', height=self.args.height)
        self.command(mav.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, self.args.height)
        deadline = time.monotonic() + 30
        reached = False
        while time.monotonic() < deadline:
            self.link.pump()
            self.guard(position=True, horizontal=False)
            p = self.link.fresh('LOCAL_POSITION_NED', 1.5)
            if abs(p.z - target_z) <= 0.15 and abs(p.vz) <= 0.15:
                reached = True
                break
        if not reached:
            raise FlightError('Takeoff altitude not reached within 30s')
        self.log('phase', name='validate_horizontal_position')
        self.guard(position=True)
        start = (p.x, p.y, target_z)
        self.move(start, yaw, 'settle after takeoff')
        for label, target in mission_targets(start, yaw, self.args.distance, self.args.pattern):
            self.move(target, yaw, label)
        self.land()
        self.monitor_mode = False
        self.armed_by_script = False
        print('Flight complete: landed and disarmed.', flush=True)

    def abort(self, reason):
        self.log('abort', reason=str(reason))
        print(f'Sequence stopped: {reason}', flush=True)
        hb = self.link.fresh('HEARTBEAT', 3)
        if (self.armed_by_script and not isinstance(reason, Takeover) and not self.link.takeover
                and hb is not None and hb.custom_mode == GUIDED):
            # Only a best-effort LAND; never force-disarm or override another mode.
            self.send_command(mav.MAV_CMD_NAV_LAND)
            print('Sent best-effort LAND; confirm landing in QGC. No disarm sent.', flush=True)
        else:
            print('No further flight commands sent. Use QGC/pilot control as needed.', flush=True)
        self.link.heartbeat_enabled = False
        self.link.watch_takeover = False


def arguments(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--lat', type=float, help='EKF origin latitude in degrees')
    parser.add_argument('--lon', type=float, help='EKF origin longitude in degrees')
    parser.add_argument('--amsl', type=float, help='EKF origin ground altitude above sea level, metres')
    parser.add_argument('--target-system', type=int, required=True, help='Vehicle MAVLink system ID')
    parser.add_argument('--source-system', type=int, default=255, help='Must match the permitted GCS system ID (default 255)')
    parser.add_argument('--listen', type=endpoint, default=('127.0.0.1', 14560))
    parser.add_argument('--qgc', type=endpoint, default=('127.0.0.1', 14550))
    parser.add_argument('--height', type=float, default=1.0, help='Takeoff metres above current home')
    parser.add_argument('--distance', type=float, default=1.0, help='Metres from the starting target for each excursion')
    parser.add_argument('--pattern', choices=['left-return', 'three-axis'], default='left-return',
                        help='Left/return only, or left/return, right/return, backward/return')
    parser.add_argument('--speed', type=float, default=0.2, help='Pre-takeoff speed request, m/s (see README)')
    parser.add_argument('--hold', type=float, default=1.0, help='Continuous settled dwell at each target, seconds')
    parser.add_argument('--position-tolerance', type=float, default=0.3, help='3D arrival radius, metres (default 0.3)')
    parser.add_argument('--settle-speed', type=float, default=0.4, help='Maximum 3D speed for arrival, m/s (default 0.4)')
    parser.add_argument('--move-timeout', type=float, default=30.0)
    parser.add_argument('--log', type=Path, default=Path('guided-flight.jsonl'))
    parser.add_argument('--execute', action='store_true', help='Actually send commands, including arm and takeoff')
    args = parser.parse_args(argv)
    args.explicit_origin = any(v is not None for v in (args.lat, args.lon, args.amsl))
    if args.explicit_origin and any(v is None for v in (args.lat, args.lon, args.amsl)):
        parser.error('Specify all of --lat, --lon and --amsl together')
    if not args.explicit_origin:
        args.lat, args.lon, args.amsl = 47.397742, 8.545594, 0.0
    if (not all(math.isfinite(v) for v in [args.lat, args.lon, args.amsl])
            or not -90 <= args.lat <= 90 or not -180 <= args.lon <= 180
            or not -1000 <= args.amsl <= 10000 or (args.lat == 0 and args.lon == 0)):
        parser.error('Provide a valid nonzero origin and ground altitude in metres AMSL')
    if not 1 <= args.target_system <= 255 or not 1 <= args.source_system <= 255:
        parser.error('System IDs must be 1..255')
    if args.target_system == args.source_system:
        parser.error('Vehicle and GCS system IDs must differ')
    for name in ['height', 'distance', 'speed', 'hold', 'position_tolerance', 'settle_speed', 'move_timeout']:
        if not math.isfinite(getattr(args, name)) or getattr(args, name) <= 0:
            parser.error(f'{name} must be finite and positive')
    if args.listen[0] != '127.0.0.1' or args.qgc[0] != '127.0.0.1' or args.listen == args.qgc:
        parser.error('Use separate loopback ports for the bridge listener and QGC')
    return args


def main(argv=None):
    args = arguments(argv)
    legs = '; '.join(label for label, _ in mission_targets((0, 0, 0), 0, args.distance, args.pattern))
    print(f'Plan: system {args.target_system}; origin {args.lat}, {args.lon}, {args.amsl}m AMSL; '
          f'takeoff {args.height}m; settle; {legs}; LAND; normal disarm. Excursions: {args.distance}m.')
    if not args.explicit_origin:
        print('Reuse an existing EKF origin; otherwise use the synthetic coordinates above (not a real location).')
    print(f'Bridge --udp {args.listen[0]}:{args.listen[1]}; QGC UDP {args.qgc[1]}; speed {args.speed}m/s.')
    if not args.execute:
        print('Preview only. Add --execute to run. No sockets opened or commands sent.')
        return 0
    # Exclusive creation prevents accidentally overwriting the preceding test.
    with args.log.open('x') as logfile:
        def log(event, **fields):
            logfile.write(json.dumps(dict(event=event, monotonic=time.monotonic(), **fields)) + '\n')
            logfile.flush()

        relay = Relay(args.listen, args.qgc, args.source_system, args.target_system, log)
        flight = Flight(relay, args, log)
        status = 0
        try:
            try:
                flight.run()
            except (Exception, KeyboardInterrupt) as exc:
                flight.abort(exc)
                status = 1
            relay.heartbeat_enabled = False
            print('QGC relay remains active. Ctrl-C now closes it; keep it running while QGC needs the radio.', flush=True)
            while True:
                relay.pump()
        except KeyboardInterrupt:
            pass
        finally:
            relay.close()
    return status


if __name__ == '__main__':
    raise SystemExit(main())
