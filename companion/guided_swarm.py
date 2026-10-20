#!/usr/bin/env python3
# AP_FLAKE8_CLEAN
"""GUIDED hover, movement and formation demos for 1–4 aircraft; preview by default.

Requires pymavlink. Normally run through swarm_hub.py; see swarm_hub_README.txt.
"""

import argparse
import itertools
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
HORIZONTAL_RECOVERY_TIMEOUT = 1.0  # Two requested 2 Hz EKF report periods.

MASK_POSITION_VELOCITY_YAW = (mav.POSITION_TARGET_TYPEMASK_AX_IGNORE
                              | mav.POSITION_TARGET_TYPEMASK_AY_IGNORE
                              | mav.POSITION_TARGET_TYPEMASK_AZ_IGNORE
                              | mav.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE)
SEND_PERIOD = 0.2
MAX_SEND_GAP = 0.5
POSITION_MAX_AGE = 0.6
MIN_SEPARATION = 1.0
MAX_SEPARATION = 2.0


class FlightError(RuntimeError):
    pass


class WaitTimeout(FlightError):
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


def pattern_directions(pattern):
    """Directions a fixed-target pattern visits, in order; empty for others."""
    if pattern == 'three-axis':
        return ('left', 'right', 'backward')
    if pattern.endswith('-return'):
        return (pattern.removesuffix('-return'),)
    return ()


def mission_targets(start, yaw, distance, pattern):
    """Targets relative to one captured heading and origin, with a return per leg."""
    if pattern == 'hover':
        return []
    targets = []
    for direction in pattern_directions(pattern):
        if direction in ('left', 'right'):
            target = left_target(start, yaw, distance if direction == 'left' else -distance)
        else:
            target = backward_target(start, yaw, -distance if direction == 'forward' else distance)
        targets.extend([(f'move {direction}', target), (f'return from {direction}', start)])
    return targets


def endpoint(value):
    host, sep, port = value.rpartition(':')
    if not sep or not host or not 0 < int(port) < 65536:
        raise argparse.ArgumentTypeError('Expected IPv4 HOST:PORT')
    return socket.gethostbyname(host), int(port)


class VehicleLink:
    """Per-aircraft state on a shared transport; never share ACKs or telemetry."""

    def __init__(self, relay, system):
        self.relay = relay
        self.target_system = system
        self.source_system = relay.source_system
        self.cache = {}
        self.acks = []
        self.takeover = False
        self.watch_takeover = False
        self.heartbeat_enabled = True
        self.mav = relay.mav

    def fresh(self, kind, age=2):
        when, message = self.cache.get(kind, (0, None))
        return message if time.monotonic() - when <= age else None

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

    def pump(self):
        self.relay.pump()


class Relay:
    """One shared UDP bridge, with independent state and ACKs for every aircraft."""

    def __init__(self, args, log):
        self.radio = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.radio.bind(args.listen)
        self.qgc = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.qgc.bind(('127.0.0.1', 0))
        self.qgc_peer = args.qgc
        self.bridge_peer = None
        self.source_system = args.source_system
        self.log = log
        self.last_heartbeat = 0
        self.heartbeat_enabled = True
        self.mav = mav.MAVLink(self, srcSystem=args.source_system, srcComponent=191)
        self.decoder = mav.MAVLink(None)
        self.decoder.robust_parsing = True
        self.qgc_decoder = mav.MAVLink(None)
        self.qgc_decoder.robust_parsing = True
        self.links = {sid: VehicleLink(self, sid) for sid in args.target_systems}
        self.guard_callback = None

    def write(self, data):
        if self.bridge_peer is None:
            raise FlightError('No bridge telemetry received')
        self.radio.sendto(data, self.bridge_peer)

    def pump(self, timeout=0.05):
        for sock in select.select([self.radio, self.qgc], [], [], timeout)[0]:
            data, peer = sock.recvfrom(65535)
            if sock is self.qgc:
                if peer != self.qgc_peer:
                    continue
                for message in self.qgc_decoder.parse_buffer(data) or []:
                    for link in self.links.values():
                        link.qgc_command(message)
                if self.bridge_peer:
                    self.radio.sendto(data, self.bridge_peer)
                continue
            if self.bridge_peer is None:
                if peer[0] != '127.0.0.1':
                    continue
                self.bridge_peer = peer
                self.log('bridge_connected', peer=peer)
            if peer != self.bridge_peer:
                continue
            self.qgc.sendto(data, self.qgc_peer)
            for message in self.decoder.parse_buffer(data) or []:
                link = self.links.get(message.get_srcSystem())
                if link is None or message.get_srcComponent() != mav.MAV_COMP_ID_AUTOPILOT1:
                    continue
                now = time.monotonic()
                kind = message.get_type()
                link.cache[kind] = (now, message)
                if (kind == 'COMMAND_ACK'
                        and getattr(message, 'target_system', 0) in (0, self.source_system)
                        and getattr(message, 'target_component', 0) in (0, 191)):
                    link.acks.append((now, message))
                    link.acks = link.acks[-100:]
                self.log('telemetry', system=link.target_system, message=message.to_dict())
                if kind == 'STATUSTEXT':
                    print(f'Vehicle {link.target_system}: {message.text}', flush=True)
        now = time.monotonic()
        if self.bridge_peer and self.heartbeat_enabled and now - self.last_heartbeat >= 1:
            self.mav.heartbeat_send(mav.MAV_TYPE_GCS, mav.MAV_AUTOPILOT_INVALID, 0, 0, 0)
            self.last_heartbeat = now
        if self.guard_callback:
            self.guard_callback()

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
        self.unconfirmed_takeoff_since = None

    def guard(self, position=False, horizontal=True, allow_horizontal_dropout=False):
        if self.link.takeover:
            raise Takeover('QGC sent a flight command; yielding control')
        if self.unconfirmed_takeoff_since is not None:
            if self.ack_received(mav.MAV_CMD_NAV_TAKEOFF, self.unconfirmed_takeoff_since):
                self.log('takeoff_ack_received_late')
                self.unconfirmed_takeoff_since = None
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
            if horizontal and allow_horizontal_dropout:
                required &= ~mav.EKF_POS_HORIZ_REL
            if ekf is None or ekf.flags & required != required or ekf.flags & mav.EKF_UNINITIALIZED:
                kind = 'horizontal position' if horizontal else 'vertical state'
                raise FlightError(f'EKF no longer reports a usable {kind}')
            if horizontal and not ekf.flags & mav.EKF_POS_HORIZ_REL:
                return False
        return True

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
        raise WaitTimeout(f'Timeout: {description}')

    def send_command(self, command, *params):
        values = list(params) + [0] * (7 - len(params))
        self.log('command', command=command, params=values)
        self.link.mav.command_long_send(self.args.target_system, 1, command, 0, *values)

    def command(self, command, *params):
        if command in (mav.MAV_CMD_SET_MESSAGE_INTERVAL, mav.MAV_CMD_DO_CHANGE_SPEED):
            self.configure_command(command, *params)
            return
        # Flight-changing commands are not blindly retried after an ambiguous ACK.
        ground = self.link.fresh('EXTENDED_SYS_STATE', 2) if command == mav.MAV_CMD_NAV_TAKEOFF else None
        started = time.monotonic()
        self.send_command(command, *params)
        try:
            self.wait_ack(command, started)
        except WaitTimeout:
            if command != mav.MAV_CMD_NAV_TAKEOFF:
                raise
            if not self.confirm_takeoff_state(started, ground):
                raise

    def confirm_takeoff_state(self, started, ground):
        """Confirm a missing TAKEOFF ACK from a fresh ground-to-air transition."""
        self.guard(position=True, horizontal=False)
        hb = self.link.fresh('HEARTBEAT', 3)
        state = self.link.fresh('EXTENDED_SYS_STATE', 2)
        # Armed/GUIDED alone is not evidence that TAKEOFF was accepted.
        if (ground is None or ground.landed_state != mav.MAV_LANDED_STATE_ON_GROUND
                or state is None or state.landed_state not in (mav.MAV_LANDED_STATE_TAKEOFF, mav.MAV_LANDED_STATE_IN_AIR)
                or hb.custom_mode != GUIDED or not hb.base_mode & mav.MAV_MODE_FLAG_SAFETY_ARMED
                or any(self.link.cache.get(kind, (0, None))[0] < started
                       for kind in ('EXTENDED_SYS_STATE', 'HEARTBEAT', 'LOCAL_POSITION_NED', 'EKF_STATUS_REPORT'))):
            return False
        self.unconfirmed_takeoff_since = started
        self.log('takeoff_ack_missing_state_confirmed', landed_state=state.landed_state)
        print(f'Vehicle {self.args.target_system}: takeoff ACK missing, but fresh aircraft state confirms takeoff; '
              'continuing monitored climb.', flush=True)
        return True

    def configure_command(self, command, *params):
        """Best-effort idempotent setup; readiness is checked from actual telemetry."""
        if command not in (mav.MAV_CMD_SET_MESSAGE_INTERVAL, mav.MAV_CMD_DO_CHANGE_SPEED):
            raise ValueError('Command is not a best-effort setup request')
        reason = 'ACK missing after two attempts'
        for attempt in range(1, 3):
            self.guard()
            started = time.monotonic()
            self.send_command(command, *params)
            try:
                self.wait_ack(command, started, timeout=2)
                return
            except WaitTimeout:
                self.log('setup_ack_timeout', command=command, params=params, attempt=attempt)
            except CommandRejected as exc:
                reason = str(exc)
                break
        self.guard()
        self.log('setup_unconfirmed', command=command, params=params, reason=reason)
        print(f'Setup command {command} unconfirmed ({reason}); continuing with telemetry/state checks.', flush=True)

    def ack_received(self, command, started):
        for when, message in self.link.acks:
            if when >= started and message.command == command:
                if message.result == mav.MAV_RESULT_IN_PROGRESS:
                    continue
                if message.result != mav.MAV_RESULT_ACCEPTED:
                    raise CommandRejected(command, message.result)
                return True
        return False

    def wait_ack(self, command, started, timeout=5):
        self.wait(lambda: self.ack_received(command, started), timeout,
                  f'ACK for command {command}', guard=self.monitor_mode)

    def command_origin(self):
        started = time.monotonic()
        self.log('command_int', command=CMD_SET_GLOBAL_ORIGIN, latitude=self.args.lat,
                 longitude=self.args.lon, altitude_m=self.args.amsl)
        self.link.mav.command_int_send(
            self.args.target_system, 1, mav.MAV_FRAME_GLOBAL, CMD_SET_GLOBAL_ORIGIN,
            0, 0, 0, 0, 0, 0, round(self.args.lat * 1e7), round(self.args.lon * 1e7), self.args.amsl)
        self.wait_ack(CMD_SET_GLOBAL_ORIGIN, started)

    def confirm_setup(self, command, send, effect, label, request=None):
        """Idempotent setup: retry once, and accept a lost ACK when fresh state shows the effect.

        Only for commands that are harmless to repeat and whose result is visible in
        telemetry (home, GUIDED on the ground). An explicit rejection still stops.
        """
        for attempt in (1, 2):
            started = time.monotonic()
            send()
            try:
                self.wait_ack(command, started, timeout=2)
                return
            except WaitTimeout:
                self.log('setup_ack_timeout', command=command, attempt=attempt)
            if request is not None:
                request()
            try:
                self.wait(lambda: self.ack_received(command, started) or effect(started), 2, label,
                          guard=self.monitor_mode)
            except WaitTimeout:
                continue
            self.log('ack_missing_state_confirmed', command=command, attempt=attempt)
            print(f'Vehicle {self.args.target_system}: ACK for command {command} lost; '
                  f'fresh telemetry confirms {label}.', flush=True)
            return
        raise WaitTimeout(f'Timeout: ACK for command {command}; {label} not observed after two attempts')

    def command_home(self, origin):
        ground = self.link.fresh('LOCAL_POSITION_NED', 1)
        if ground is None or not math.isfinite(ground.z):
            raise FlightError('Local ground height unavailable for home setup')
        altitude_m = origin.altitude / 1000 - ground.z
        # Use the indoor origin as the horizontal home reference. "Use current"
        # requires a valid horizontal EKF location, unavailable on this deck at rest.
        self.link.cache.pop('HOME_POSITION', None)

        def send():
            self.log('command_int', command=mav.MAV_CMD_DO_SET_HOME, latitude=origin.latitude / 1e7,
                     longitude=origin.longitude / 1e7, altitude_m=altitude_m)
            self.link.mav.command_int_send(
                self.args.target_system, 1, mav.MAV_FRAME_GLOBAL, mav.MAV_CMD_DO_SET_HOME,
                0, 0, 0, 0, 0, 0, origin.latitude, origin.longitude, altitude_m)

        def home_set(started):
            when, home = self.link.cache.get('HOME_POSITION', (0, None))
            return (home is not None and when >= started
                    and home.latitude == origin.latitude and home.longitude == origin.longitude
                    and abs(home.altitude / 1000 - altitude_m) <= 0.05)

        self.confirm_setup(mav.MAV_CMD_DO_SET_HOME, send, home_set, 'home position',
                           request=lambda: self.request(mav.MAVLINK_MSG_ID_HOME_POSITION))

    def set_guided(self):
        def guided(started):
            when, hb = self.link.cache.get('HEARTBEAT', (0, None))
            return hb is not None and when >= started and hb.custom_mode == GUIDED

        self.confirm_setup(mav.MAV_CMD_DO_SET_MODE,
                           lambda: self.send_command(mav.MAV_CMD_DO_SET_MODE,
                                                     mav.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, GUIDED),
                           guided, 'GUIDED mode')

    def prearm_checks(self):
        deadline = time.monotonic() + 30
        next_check = 0
        while time.monotonic() < deadline:
            self.link.pump()
            self.guard()
            if time.monotonic() >= next_check:
                # ACCEPTED means checks were run, not that they passed, so
                # readiness comes from SYS_STATUS. A lost ACK just means
                # asking again; an explicit rejection still stops.
                try:
                    self.command(mav.MAV_CMD_RUN_PREARM_CHECKS)
                except WaitTimeout:
                    self.log('prearm_ack_timeout')
                next_check = time.monotonic() + 2
            state = self.link.fresh('SYS_STATUS', 2)
            if state and state.onboard_control_sensors_health & mav.MAV_SYS_STATUS_PREARM_CHECK:
                return
        raise FlightError('Prearm checks still failing after 30s; ARM was not sent')

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


def backward_target(start, yaw, distance):
    return (start[0] - distance * math.cos(yaw), start[1] - distance * math.sin(yaw), start[2])


class GroupFlight:
    def __init__(self, relay, args, log):
        self.relay, self.args, self.log = relay, args, log
        self.flights = []
        self.starts, self.yaws, self.heights = {}, {}, {}
        self.landing = False
        for sid, link in relay.links.items():
            vehicle_args = argparse.Namespace(**vars(args), target_system=sid)

            def vehicle_log(event, system=sid, **fields):
                log(event, system=system, **fields)
            self.flights.append(Flight(link, vehicle_args, vehicle_log))

    def check_group(self):
        for f in self.flights:
            try:
                f.guard()
                if self.landing and f.link.fresh('HEARTBEAT', 3).custom_mode not in (GUIDED, LAND):
                    raise Takeover('Mode changed during landing')
            except FlightError as exc:
                if isinstance(exc, Takeover):
                    f.link.takeover = True
                raise FlightError(f'Vehicle {f.args.target_system}: {exc}') from exc

    def wait(self, predicate, timeout, description):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            self.relay.pump()
            if predicate():
                return
        raise WaitTimeout(f'Timeout: {description}')

    def command_all(self, command, *params, state_confirmation=None):
        ground = {f.args.target_system: f.link.fresh('EXTENDED_SYS_STATE', 2) for f in self.flights}
        started = time.monotonic()
        pending = set(self.args.target_systems)
        # Issue all commands before waiting for their responses.
        for f in self.flights:
            f.send_command(command, *params)

        def confirmed():
            for f in self.flights:
                sid = f.args.target_system
                if sid not in pending:
                    continue
                replies = [msg for when, msg in f.link.acks if when >= started and msg.command == command]
                if any(msg.result not in (mav.MAV_RESULT_ACCEPTED, mav.MAV_RESULT_IN_PROGRESS) for msg in replies):
                    raise FlightError(f'Vehicle {sid}: command {command} rejected')
                acked = any(msg.result == mav.MAV_RESULT_ACCEPTED for msg in replies)
                hb_time, hb = f.link.cache.get('HEARTBEAT', (0, None))
                observed = (state_confirmation is not None and hb_time >= started
                            and hb is not None and state_confirmation(hb))
                if acked or observed:
                    pending.remove(sid)
            return not pending

        try:
            self.wait(confirmed, 5, f'command {command}, missing confirmations {pending}')
        except WaitTimeout:
            if command != mav.MAV_CMD_NAV_TAKEOFF:
                raise
            for f in self.flights:
                sid = f.args.target_system
                if sid in pending and not f.confirm_takeoff_state(started, ground[sid]):
                    raise WaitTimeout(f'Vehicle {sid}: TAKEOFF ACK missing and takeoff state unconfirmed')

    def prepare(self, f):
        sid = f.args.target_system
        hb = f.link.fresh('HEARTBEAT', 3)
        if hb.autopilot != mav.MAV_AUTOPILOT_ARDUPILOTMEGA or hb.base_mode & mav.MAV_MODE_FLAG_SAFETY_ARMED:
            raise FlightError(f'Vehicle {sid}: start with an ArduPilot aircraft disarmed on the ground')
        f.link.watch_takeover = True
        for msg_id, hz in [(0, 2), (32, 10), (30, 5), (245, 2), (193, 2), (1, 2)]:
            f.command(mav.MAV_CMD_SET_MESSAGE_INTERVAL, msg_id, 1e6 / hz)
        origin = f.setup_origin()
        f.wait(f.ground_ready, 30, 'ground readiness')
        f.command_home(origin)
        f.request(mav.MAVLINK_MSG_ID_HOME_POSITION)
        home = f.wait(lambda: f.link.fresh('HOME_POSITION', 3), 5, 'home confirmation')
        ground = f.link.fresh('LOCAL_POSITION_NED', 1)
        target_z = (origin.altitude - home.altitude) / 1000 - self.args.height
        if ground is None or abs(target_z - (ground.z - self.args.height)) > 0.25:
            raise FlightError(f'Vehicle {sid}: home and local height disagree')
        self.heights[sid] = target_z
        f.set_guided()
        f.wait(lambda: f.link.fresh('HEARTBEAT', 3).custom_mode == GUIDED, 5, 'GUIDED confirmation')
        f.command(mav.MAV_CMD_DO_CHANGE_SPEED, 1, self.args.speed, -1)
        f.prearm_checks()
        f.log('prepared', takeoff_target_z=target_z, home=home.to_dict())

    def takeoff(self):
        for f in self.flights:
            hb = f.link.fresh('HEARTBEAT', 3)
            att = f.link.fresh('ATTITUDE', 1)
            if (not f.ground_ready() or hb is None or hb.custom_mode != GUIDED
                    or hb.base_mode & mav.MAV_MODE_FLAG_SAFETY_ARMED or att is None):
                raise FlightError(f'Vehicle {f.args.target_system}: readiness changed before arming')
            self.yaws[f.args.target_system] = att.yaw
        for f in self.flights:
            f.armed_by_script = True
        self.command_all(mav.MAV_CMD_COMPONENT_ARM_DISARM, 1, 0,
                         state_confirmation=lambda hb: bool(hb.base_mode & mav.MAV_MODE_FLAG_SAFETY_ARMED))
        self.wait(lambda: all(f.link.fresh('HEARTBEAT', 3).base_mode & mav.MAV_MODE_FLAG_SAFETY_ARMED
                              for f in self.flights), 5, 'all aircraft armed')
        for f in self.flights:
            f.monitor_mode = True
            f.guard(position=True, horizontal=False)
        self.log('phase', name='takeoff')
        self.command_all(mav.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, self.args.height)

        def climbed():
            ready = True
            for f in self.flights:
                f.guard(position=True, horizontal=False)
                p = f.link.fresh('LOCAL_POSITION_NED', 1.5)
                ready &= abs(p.z - self.heights[f.args.target_system]) <= 0.15 and abs(p.vz) <= 0.15
            return ready

        self.wait(climbed, self.args.takeoff_timeout, 'all aircraft at takeoff altitude')
        for f in self.flights:
            f.guard(position=True)
            sid = f.args.target_system
            p = f.link.fresh('LOCAL_POSITION_NED', 1.5)
            self.starts[sid] = (p.x, p.y, self.heights[sid])

    def horizontal_ready(self, lost_since, label):
        """Allow a bounded loss of only the relative-horizontal-position flag."""
        ready = True
        for f in self.flights:
            valid = f.guard(position=True, allow_horizontal_dropout=True)
            now = time.monotonic()
            sid = f.args.target_system
            since = lost_since.get(sid)
            if since is not None and now >= since + HORIZONTAL_RECOVERY_TIMEOUT:
                f.log('horizontal_recovery_timeout', name=label, elapsed_s=now - since)
                raise FlightError(f'Vehicle {sid}: horizontal position did not recover within '
                                  f'{HORIZONTAL_RECOVERY_TIMEOUT:g}s')
            if not valid:
                ready = False
                if since is None:
                    lost_since[sid] = now
                    f.log('horizontal_recovery_started', name=label, timeout_s=HORIZONTAL_RECOVERY_TIMEOUT)
                    print(f'Vehicle {sid}: horizontal position temporarily invalid; holding group targets.', flush=True)
            elif since is not None:
                f.log('horizontal_recovery_complete', name=label, elapsed_s=now - since)
                del lost_since[sid]
        return ready

    def move(self, targets, label, dwell, horizontal_recovery=False):
        self.log('phase', name=label, targets=targets, dwell=dwell)
        print(label, targets, flush=True)
        settled_since, next_send, next_report = None, 0, 0
        horizontal_lost_since = {}
        deadline = time.monotonic() + self.args.move_timeout + dwell
        while time.monotonic() < deadline:
            self.relay.pump()
            now = time.monotonic()
            horizontal_valid = True
            if horizontal_recovery:
                horizontal_valid = self.horizontal_ready(horizontal_lost_since, label)
            else:
                for f in self.flights:
                    f.guard(position=True)
            all_settled = horizontal_valid
            for f in self.flights:
                sid = f.args.target_system
                if now >= next_send:
                    f.setpoint(targets[sid], self.yaws[sid])
                p = f.link.fresh('LOCAL_POSITION_NED', 1.5)
                error = math.dist((p.x, p.y, p.z), targets[sid])
                speed = math.sqrt(p.vx**2 + p.vy**2 + p.vz**2)
                all_settled &= error <= self.args.position_tolerance and speed <= self.args.settle_speed
                if now >= next_report:
                    f.log('arrival_progress', name=label, error_m=error, speed_ms=speed)
            if now >= next_send:
                next_send = now + 0.2
            if now >= next_report:
                next_report = now + 1
            if all_settled:
                if settled_since is None:
                    settled_since = now
                if now - settled_since >= dwell:
                    self.log('settled', name=label)
                    return
            else:
                settled_since = None
        raise FlightError(f'Timeout: all aircraft settling at {label}')

    def land(self):
        self.log('phase', name='land')
        self.landing = True
        for f in self.flights:
            f.expected_modes = {GUIDED, LAND}
            f.monitor_mode = False
        self.command_all(mav.MAV_CMD_NAV_LAND, state_confirmation=lambda hb: hb.custom_mode == LAND)
        self.wait(lambda: all(f.link.fresh('HEARTBEAT', 3).custom_mode == LAND for f in self.flights),
                  5, 'all aircraft in LAND')
        # The landing loop explicitly permits normal auto-disarm.
        for f in self.flights:
            f.monitor_mode = False
        grounded_since, disarm_sent, complete = {}, {}, set()
        deadline = time.monotonic() + 45
        while time.monotonic() < deadline:
            self.relay.pump()
            now = time.monotonic()
            for f in self.flights:
                sid = f.args.target_system
                if sid in complete:
                    continue
                hb = f.link.fresh('HEARTBEAT', 3)
                if hb is None:
                    raise FlightError(f'Vehicle {sid}: landing heartbeat lost; disarm not sent')
                if hb.custom_mode != LAND:
                    f.link.takeover = True
                    raise FlightError(f'Vehicle {sid}: mode changed during landing')
                state = f.link.fresh('EXTENDED_SYS_STATE', 2)
                grounded = state is not None and state.landed_state == mav.MAV_LANDED_STATE_ON_GROUND
                if grounded and not hb.base_mode & mav.MAV_MODE_FLAG_SAFETY_ARMED:
                    complete.add(sid)
                    f.armed_by_script = False
                    f.log('complete', disarm='confirmed')
                elif grounded:
                    grounded_since.setdefault(sid, now)
                    if now - grounded_since[sid] >= 2 and sid not in disarm_sent:
                        f.send_command(mav.MAV_CMD_COMPONENT_ARM_DISARM, 0, 0)
                        disarm_sent[sid] = now
                else:
                    grounded_since.pop(sid, None)
                if sid in disarm_sent and sid not in complete and now - disarm_sent[sid] > 5:
                    raise FlightError(f'Vehicle {sid}: normal disarm unconfirmed; no retry or force-disarm')
            if len(complete) == len(self.flights):
                return
        raise FlightError('Landing not confirmed for all aircraft; no force-disarm')

    def abort(self, exc):
        self.relay.guard_callback = None
        for f in self.flights:
            hb = f.link.fresh('HEARTBEAT', 3)
            if hb and f.monitor_mode and hb.custom_mode not in f.expected_modes:
                f.link.takeover = True
            try:
                # A takeover on one aircraft must not suppress landing the other.
                f.abort(FlightError(str(exc)))
            except Exception as abort_error:
                f.log('abort_send_failed', reason=str(abort_error))
        self.relay.heartbeat_enabled = False


def layout(count, spacing):
    """Marked launch positions in metres: forward, right, down."""
    if count == 1:
        return [(0., 0., 0.)]
    if count == 2:
        return [(0., -spacing / 2, 0.), (0., spacing / 2, 0.)]
    if count == 3:
        return [(spacing / math.sqrt(3), 0., 0.),
                (-spacing / (2 * math.sqrt(3)), -spacing / 2, 0.),
                (-spacing / (2 * math.sqrt(3)), spacing / 2, 0.)]
    return [(spacing / 2, -spacing / 2, 0.), (spacing / 2, spacing / 2, 0.),
            (-spacing / 2, -spacing / 2, 0.), (-spacing / 2, spacing / 2, 0.)]


def rotate(vector, yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    x, y, z = vector
    return (c * x - s * y, s * x + c * y, z)


def sweep_sample(elapsed, duration, radius):
    """One closed circle with quintic phase: zero endpoint velocity/acceleration."""
    u = min(1., max(0., elapsed / duration))
    if u in (0., 1.):
        return (0., 0., 0.), (0., 0., 0.)
    phase = 10 * u**3 - 15 * u**4 + 6 * u**5
    theta = 2 * math.pi * phase
    omega = 2 * math.pi * 30 * u**2 * (1 - u)**2 / duration
    # The circle's centre is behind the initial hover point; maximum excursion is 2r.
    return ((radius * (math.cos(theta) - 1), radius * math.sin(theta), 0.),
            (-radius * omega * math.sin(theta), radius * omega * math.cos(theta), 0.))


def diamond_sample(elapsed, duration, radius):
    """Four straight legs with zero requested velocity/acceleration at corners."""
    if elapsed <= 0 or elapsed >= duration:
        return (0., 0., 0.), (0., 0., 0.)
    vertices = ((0., 0., 0.), (-radius, radius, 0.), (-2 * radius, 0., 0.),
                (-radius, -radius, 0.), (0., 0., 0.))
    leg_time = duration / 4
    leg = min(3, int(elapsed / leg_time))
    u = (elapsed - leg * leg_time) / leg_time
    phase = 10 * u**3 - 15 * u**4 + 6 * u**5
    rate = 30 * u**2 * (1 - u)**2 / leg_time
    start, end = vertices[leg:leg + 2]
    delta = tuple(b - a for a, b in zip(start, end))
    return (tuple(a + phase * d for a, d in zip(start, delta)), tuple(rate * d for d in delta))


def peak_speed(pattern, radius, duration):
    path_length = 4 * math.sqrt(2) * radius if pattern == 'diamond' else 2 * math.pi * radius
    return 1.875 * path_length / duration


class SwarmFlight(GroupFlight):
    def __init__(self, relay, args, log):
        super().__init__(relay, args, log)
        self.markers = dict(zip(args.target_systems, layout(len(args.target_systems), args.spacing)))
        self.ground = {}
        self.flight_started = None
        self.formation_monitoring = False
        self.next_spacing_log = 0

    def takeoff(self):
        # Capture ground coordinates before arming, not the later hover position.
        for f in self.flights:
            f.guard(position=True, horizontal=False)
            p = f.link.fresh('LOCAL_POSITION_NED', 1.5)
            if p is None:
                raise FlightError('Fresh ground position required before arming')
            self.ground[f.args.target_system] = (p.x, p.y, p.z)
        self.flight_started = time.monotonic()
        super().takeoff()
        self.formation_monitoring = True
        self.log('formation_reference', markers=self.markers, ground=self.ground,
                 starts=self.starts, yaws=self.yaws)

    def check_group(self):
        super().check_group()
        if self.flight_started is None or self.landing:
            return
        now = time.monotonic()
        if now - self.flight_started > self.args.max_flight_time:
            raise FlightError('Demo time limit reached; ending the choreography')
        # Arming can briefly interrupt telemetry. The inherited takeoff checks
        # require a usable vertical state; stricter formation checks start aloft.
        if not self.formation_monitoring:
            return
        positions = {}
        for f in self.flights:
            sid = f.args.target_system
            p = f.link.fresh('LOCAL_POSITION_NED', POSITION_MAX_AGE)
            if p is None or not all(math.isfinite(v) for v in (p.x, p.y, p.z)):
                raise FlightError(f'Vehicle {sid}: position stale or non-finite')
            # Local NED axes need not have matching yaw estimates. All airframes
            # must physically face the marked forward direction at launch.
            delta = rotate(tuple(v - g for v, g in zip((p.x, p.y, p.z), self.ground[sid])), -self.yaws[sid])
            positions[sid] = tuple(m + d for m, d in zip(self.markers[sid], delta))
        for first, second in itertools.combinations(positions, 2):
            separation = math.dist(positions[first], positions[second])
            if not MIN_SEPARATION <= separation <= MAX_SEPARATION:
                raise FlightError(f'Estimated separation {first}/{second}: {separation:.2f} m outside 1–2 m')
        if now >= self.next_spacing_log:
            self.log('estimated_formation', positions=positions)
            self.next_spacing_log = now + 1

    def sweep(self):
        self.log('phase', name='sweep', pattern=self.args.pattern, radius=self.args.radius, duration=self.args.duration)
        sample = diamond_sample if self.args.pattern == 'diamond' else sweep_sample
        started = time.monotonic()
        last_send = None
        next_send = started
        horizontal_lost_since = {}
        paused_since = None
        paused_elapsed = 0.0
        last_elapsed = 0.0
        paused_time = 0.0
        while True:
            self.relay.pump()
            now = time.monotonic()
            if last_send is not None and now - last_send > MAX_SEND_GAP:
                raise FlightError('Trajectory sender delayed; refusing to jump ahead on the path')
            horizontal_valid = self.horizontal_ready(horizontal_lost_since, 'sweep')
            if paused_since is not None and now >= paused_since + HORIZONTAL_RECOVERY_TIMEOUT:
                raise FlightError('Group horizontal position did not recover within '
                                  f'{HORIZONTAL_RECOVERY_TIMEOUT:g}s')
            if not horizontal_valid:
                if paused_since is None:
                    paused_since, paused_elapsed = now, last_elapsed
                    next_send = now
                    self.log('trajectory_paused', elapsed=paused_elapsed)
                elapsed = paused_elapsed
            else:
                if paused_since is not None:
                    # Resume from the last commanded phase, without catching up
                    # to the wall clock or jumping to a later path target.
                    paused_time = now - started - paused_elapsed
                    self.log('trajectory_resumed', elapsed=paused_elapsed, pause_s=now - paused_since)
                    paused_since = None
                    next_send = now
                elapsed = now - started - paused_time
            offset, velocity = sample(elapsed, self.args.duration, self.args.radius)
            if not horizontal_valid:
                velocity = (0., 0., 0.)
            targets = {}
            # Check every aircraft before transmitting to any member of the group.
            for f in self.flights:
                sid = f.args.target_system
                displacement = rotate(offset, self.yaws[sid])
                target = tuple(a + b for a, b in zip(self.starts[sid], displacement))
                targets[sid] = target
                p = f.link.fresh('LOCAL_POSITION_NED', POSITION_MAX_AGE)
                if p is None:
                    raise FlightError(f'Vehicle {sid}: sweep tracking unavailable: position stale')
                error = math.dist((p.x, p.y, p.z), target)
                if error > self.args.tracking_tolerance:
                    f.log('tracking_error', error_m=error, limit_m=self.args.tracking_tolerance,
                          target=target, position=(p.x, p.y, p.z), time_boot_ms=p.time_boot_ms,
                          position_age_s=now - f.link.cache['LOCAL_POSITION_NED'][0])
                    raise FlightError(f'Vehicle {sid}: sweep tracking error {error:.3f} m exceeded '
                                      f'{self.args.tracking_tolerance:.2f} m limit')
            if now + 1e-9 >= next_send or elapsed >= self.args.duration:
                for f in self.flights:
                    sid = f.args.target_system
                    local_velocity = rotate(velocity, self.yaws[sid])
                    f.log('trajectory_setpoint', elapsed=min(elapsed, self.args.duration),
                          ned=targets[sid], velocity=local_velocity, yaw=self.yaws[sid])
                    if time.monotonic() - now > MAX_SEND_GAP:
                        raise FlightError('Trajectory sender delayed within group; stopping the sweep')
                    f.link.mav.set_position_target_local_ned_send(
                        0, sid, 1, mav.MAV_FRAME_LOCAL_NED, MASK_POSITION_VELOCITY_YAW,
                        *targets[sid], *local_velocity, 0, 0, 0, self.yaws[sid], 0)
                last_send = now
                last_elapsed = min(elapsed, self.args.duration)
                next_send += SEND_PERIOD
                if next_send < now:
                    next_send = now + SEND_PERIOD
                if elapsed >= self.args.duration:
                    return

    def run(self):
        self.wait(lambda: all(f.link.fresh('HEARTBEAT', 3) for f in self.flights), 20, 'all autopilot heartbeats')
        self.relay.guard_callback = self.check_group
        for f in self.flights:
            self.prepare(f)
        self.takeoff()
        self.move(self.starts, 'hover', self.args.hover, horizontal_recovery=True)
        if self.args.pattern in ('circle', 'diamond'):
            self.sweep()
            self.move(self.starts, 'return', self.args.hold, horizontal_recovery=True)
        else:
            legs = {sid: mission_targets(start, self.yaws[sid], self.args.distance, self.args.pattern)
                    for sid, start in self.starts.items()}
            for i, (label, _) in enumerate(next(iter(legs.values()))):
                targets = {sid: entries[i][1] for sid, entries in legs.items()}
                self.move(targets, label, self.args.hold, horizontal_recovery=True)
        self.land()
        self.log('complete', systems=self.args.target_systems)
        print('All aircraft landed and disarmed.', flush=True)


def arguments(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--target-systems', nargs='+', type=int, default=[1])
    parser.add_argument('--pattern', choices=('hover', 'forward-return', 'backward-return', 'left-return',
                                              'right-return', 'three-axis', 'circle', 'diamond'), default='circle')
    parser.add_argument('--distance', type=float, default=1, help='Fixed-target excursion distance, metres')
    parser.add_argument('--speed', type=float, default=.2, help='Setup request; not a fixed-target movement speed cap')
    parser.add_argument('--radius', type=float, default=.25, help='Half the path width/depth in metres')
    parser.add_argument('--duration', type=float, default=12,
                        help='One full path duration in seconds, excluding recovery pauses')
    parser.add_argument('--spacing', type=float, help='Marked side length; default 1.5 m, or 1.25 m for four')
    parser.add_argument('--hover', type=float, default=2)
    parser.add_argument('--hold', type=float, default=2,
                        help='Settled dwell at each movement target and final return, seconds')
    parser.add_argument('--height', type=float, default=1, help='Nominal home-relative height, not floor clearance')
    parser.add_argument('--takeoff-timeout', type=float, default=25)
    parser.add_argument('--move-timeout', type=float, help='Arrival timeout: 8 s for paths, 30 s for fixed targets')
    parser.add_argument('--max-flight-time', type=float, default=45, help='Seconds before requesting LAND; excludes landing')
    parser.add_argument('--position-tolerance', type=float, help='Arrival radius: 0.15 m for paths, 0.30 m otherwise')
    parser.add_argument('--settle-speed', type=float, help='Arrival speed: 0.15 m/s for paths, 0.40 m/s otherwise')
    parser.add_argument('--tracking-tolerance', type=float, default=.5,
                        help='Maximum estimated 3-D path error, metres')
    parser.add_argument('--source-system', type=int, default=255)
    parser.add_argument('--listen', type=endpoint, default=('127.0.0.1', 14560))
    parser.add_argument('--qgc', type=endpoint, default=('127.0.0.1', 14550))
    parser.add_argument('--lat', type=float)
    parser.add_argument('--lon', type=float)
    parser.add_argument('--amsl', type=float)
    parser.add_argument('--log', type=Path, default=Path('guided-swarm.jsonl'))
    parser.add_argument('--execute', action='store_true')
    args = parser.parse_args(argv)
    timed_path = args.pattern in ('circle', 'diamond')
    for name, default in (('move_timeout', 8 if timed_path else 30),
                          ('position_tolerance', .15 if timed_path else .3),
                          ('settle_speed', .15 if timed_path else .4)):
        if getattr(args, name) is None:
            setattr(args, name, default)
    if (not 1 <= len(args.target_systems) <= 4 or len(set(args.target_systems)) != len(args.target_systems)
            or any(sid not in range(1, 256) for sid in args.target_systems)):
        parser.error('Choose one to four distinct vehicle IDs in 1..255')
    if args.source_system in args.target_systems:
        parser.error('GCS source ID must differ from every vehicle ID')
    args.spacing = args.spacing if args.spacing is not None else (1.25 if len(args.target_systems) == 4 else 1.5)
    for name in ('radius', 'duration', 'spacing', 'hover', 'takeoff_timeout', 'max_flight_time', 'tracking_tolerance'):
        if not math.isfinite(getattr(args, name)) or getattr(args, name) <= 0:
            parser.error(f'{name} must be finite and positive')
    if args.pattern in ('circle', 'diamond'):
        max_radius, max_duration = (1.0, 40) if args.pattern == 'diamond' else (.4, 20)
        if not .05 <= args.radius <= max_radius or not 10 <= args.duration <= max_duration:
            parser.error(f'For {args.pattern}, use radius 0.05–{max_radius:g} m and duration 10–{max_duration} s')
        if peak_speed(args.pattern, args.radius, args.duration) > .35:
            parser.error('Trajectory peak requested speed must not exceed 0.35 m/s; reduce radius or increase duration')
        minimum_time = args.duration + args.hover + args.hold
    else:
        minimum_time = args.hover + 2 * len(pattern_directions(args.pattern)) * args.hold
    if args.max_flight_time > 60 or args.max_flight_time <= minimum_time:
        parser.error('Flight time limit must cover the routine and be at most 60 seconds')
    points = layout(len(args.target_systems), args.spacing)
    if any(not 1.15 <= math.dist(a, b) <= 1.85 for a, b in itertools.combinations(points, 2)):
        parser.error('Marked pairwise distances must be within 1.15–1.85 m to leave a nominal margin')
    args.explicit_origin = any(v is not None for v in (args.lat, args.lon, args.amsl))
    if args.explicit_origin and any(v is None for v in (args.lat, args.lon, args.amsl)):
        parser.error('Specify all of --lat, --lon and --amsl together')
    if not args.explicit_origin:
        args.lat, args.lon, args.amsl = 47.397742, 8.545594, 0.0
    if (not all(math.isfinite(v) for v in [args.lat, args.lon, args.amsl])
            or not -90 <= args.lat <= 90 or not -180 <= args.lon <= 180
            or not -1000 <= args.amsl <= 10000 or (args.lat == 0 and args.lon == 0)):
        parser.error('Provide a valid nonzero origin and ground altitude in metres AMSL')
    if not 1 <= args.source_system <= 255:
        parser.error('System IDs must be 1..255')
    for name in ['height', 'distance', 'speed', 'hold', 'position_tolerance', 'settle_speed', 'move_timeout']:
        if not math.isfinite(getattr(args, name)) or getattr(args, name) <= 0:
            parser.error(f'{name} must be finite and positive')
    # A target closer than the arrival radius counts as reached before the move.
    if pattern_directions(args.pattern) and args.distance <= args.position_tolerance:
        parser.error(f'--distance {args.distance:g} m must exceed the {args.position_tolerance:g} m arrival '
                     'tolerance; lengthen it or lower --position-tolerance')
    if args.listen[0] != '127.0.0.1' or args.qgc[0] != '127.0.0.1' or args.listen == args.qgc:
        parser.error('Use separate loopback ports for the bridge listener and QGC')
    return args


def main(argv=None):
    args = arguments(argv)
    points = layout(len(args.target_systems), args.spacing)
    if args.pattern in ('circle', 'diamond'):
        sequence = f'one {args.pattern} (half-width {args.radius:g} m) in {args.duration:g} s; settle {args.hold:g} s'
        excursion = 2 * args.radius
    else:
        labels = []
        for direction in pattern_directions(args.pattern):
            labels += [f'move {direction} {args.distance:g} m', f'return from {direction}']
        sequence = '; '.join(labels) if labels else 'hover only'
        excursion = args.distance if labels else 0
    print(f'Aircraft {args.target_systems}: takeoff {args.height:g} m; hover {args.hover:g} s; {sequence}; LAND.')
    print('Place all aircraft on level marked spots, facing the SAME forward direction:')
    for sid, point in zip(args.target_systems, points):
        print(f'  System {sid}: forward={point[0]:+.3f} m, right={point[1]:+.3f} m')
    print(f'Maximum requested excursion from each hover spot: {excursion:.2f} m.')
    if args.pattern in ('circle', 'diamond'):
        print(f'Peak requested speed: {peak_speed(args.pattern, args.radius, args.duration):.3f} m/s.')
    elif args.pattern != 'hover':
        print('Fixed position targets; aircraft position control determines movement speed. '
              f'A leg counts as reached within {args.position_tolerance:g} m of its target.')
    if args.pattern == 'diamond':
        print('Diamond vertices relative to hover (forward, right): '
              f'(0, 0) -> ({-args.radius:g}, {args.radius:g}) -> ({-2 * args.radius:g}, 0) -> '
              f'({-args.radius:g}, {-args.radius:g}) -> (0, 0). Zero requested speed at corners.')
    print(f'Time limit from arming through final hover: {args.max_flight_time:g} s; '
          'landing is additional and recovery pauses count toward this limit.')
    if args.pattern in ('circle', 'diamond'):
        print(f'Path tracking-error limit: {args.tracking_tolerance:.2f} m.')
    print(f'Arrival: error ≤ {args.position_tolerance:g} m, speed ≤ {args.settle_speed:g} m/s; '
          'estimated pairwise spacing remains 1–2 m.')
    print('One bridge process, including both dongles if used. Spacing checks use drifting estimates, not measured distances.')
    print('Height is home-relative; takeoff overshoot and unmeasured horizontal drift remain possible.')
    if not args.execute:
        print('Preview only; no sockets opened or commands sent. Add --execute to fly.')
        return 0
    with args.log.open('x') as logfile:
        def log(event, **fields):
            logfile.write(json.dumps(dict(event=event, monotonic=time.monotonic(), **fields)) + '\n')
            logfile.flush()

        log('plan', systems=args.target_systems, markers=points, pattern=args.pattern, radius=args.radius,
            distance=args.distance, height=args.height, hover=args.hover, hold=args.hold,
            duration=args.duration, tracking_tolerance=args.tracking_tolerance, max_flight_time=args.max_flight_time)
        relay = Relay(args, log)
        flight = SwarmFlight(relay, args, log)
        result = 0
        try:
            try:
                flight.run()
            except (Exception, KeyboardInterrupt) as exc:
                flight.abort(exc)
                result = 1
            relay.guard_callback = None
            relay.heartbeat_enabled = False
            for link in relay.links.values():
                link.watch_takeover = False
            print('QGC relay remains active. Ctrl-C closes it.', flush=True)
            while True:
                relay.pump()
        except KeyboardInterrupt:
            pass
        finally:
            relay.close()
    return result


if __name__ == '__main__':
    raise SystemExit(main())
