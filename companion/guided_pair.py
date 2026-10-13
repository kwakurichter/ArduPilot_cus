#!/usr/bin/env python3
# AP_FLAKE8_CLEAN
"""Two-vehicle GUIDED test over one shared Crazyradio bridge; preview by default."""

import argparse
import json
import math
from pathlib import Path
import select
import time

import guided_flight as gf

mav = gf.mav


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

    fresh = gf.Relay.fresh
    qgc_command = gf.Relay.qgc_command

    def pump(self):
        self.relay.pump()


class PairRelay(gf.Relay):
    def __init__(self, args, log):
        super().__init__(args.listen, args.qgc, args.source_system, args.target_systems[0], log)
        self.links = {sid: VehicleLink(self, sid) for sid in args.target_systems}
        self.guard_callback = None

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


def backward_target(start, yaw, distance):
    return (start[0] - distance * math.cos(yaw), start[1] - distance * math.sin(yaw), start[2])


class PairFlight:
    def __init__(self, relay, args, log):
        self.relay, self.args, self.log = relay, args, log
        self.flights = []
        self.starts, self.yaws, self.heights = {}, {}, {}
        self.landing = False
        for sid, link in relay.links.items():
            vehicle_args = argparse.Namespace(**vars(args), target_system=sid)

            def vehicle_log(event, system=sid, **fields):
                log(event, system=system, **fields)
            self.flights.append(gf.Flight(link, vehicle_args, vehicle_log))

    def check_group(self):
        for f in self.flights:
            try:
                f.guard()
                if self.landing and f.link.fresh('HEARTBEAT', 3).custom_mode not in (gf.GUIDED, gf.LAND):
                    raise gf.Takeover('Mode changed during landing')
            except gf.FlightError as exc:
                if isinstance(exc, gf.Takeover):
                    f.link.takeover = True
                raise gf.FlightError(f'Vehicle {f.args.target_system}: {exc}') from exc

    def wait(self, predicate, timeout, description):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            self.relay.pump()
            if predicate():
                return
        raise gf.FlightError(f'Timeout: {description}')

    def command_all(self, command, *params, state_confirmation=None):
        started = time.monotonic()
        pending = set(self.args.target_systems)
        # Issue both commands before waiting for either response.
        for f in self.flights:
            f.send_command(command, *params)

        def confirmed():
            for f in self.flights:
                sid = f.args.target_system
                if sid not in pending:
                    continue
                replies = [msg for when, msg in f.link.acks if when >= started and msg.command == command]
                if any(msg.result not in (mav.MAV_RESULT_ACCEPTED, mav.MAV_RESULT_IN_PROGRESS) for msg in replies):
                    raise gf.FlightError(f'Vehicle {sid}: command {command} rejected')
                acked = any(msg.result == mav.MAV_RESULT_ACCEPTED for msg in replies)
                hb_time, hb = f.link.cache.get('HEARTBEAT', (0, None))
                observed = (state_confirmation is not None and hb_time >= started
                            and hb is not None and state_confirmation(hb))
                if acked or observed:
                    pending.remove(sid)
            return not pending

        self.wait(confirmed, 5, f'command {command}, missing confirmations {pending}')

    def prepare(self, f):
        sid = f.args.target_system
        hb = f.link.fresh('HEARTBEAT', 3)
        if hb.autopilot != mav.MAV_AUTOPILOT_ARDUPILOTMEGA or hb.base_mode & mav.MAV_MODE_FLAG_SAFETY_ARMED:
            raise gf.FlightError(f'Vehicle {sid}: start with an ArduPilot aircraft disarmed on the ground')
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
            raise gf.FlightError(f'Vehicle {sid}: home and local height disagree')
        self.heights[sid] = target_z
        f.command(mav.MAV_CMD_DO_SET_MODE, mav.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, gf.GUIDED)
        f.wait(lambda: f.link.fresh('HEARTBEAT', 3).custom_mode == gf.GUIDED, 5, 'GUIDED confirmation')
        f.command(mav.MAV_CMD_DO_CHANGE_SPEED, 1, self.args.speed, -1)
        f.prearm_checks()
        f.log('prepared', takeoff_target_z=target_z, home=home.to_dict())

    def takeoff(self):
        for f in self.flights:
            hb = f.link.fresh('HEARTBEAT', 3)
            att = f.link.fresh('ATTITUDE', 1)
            if (not f.ground_ready() or hb is None or hb.custom_mode != gf.GUIDED
                    or hb.base_mode & mav.MAV_MODE_FLAG_SAFETY_ARMED or att is None):
                raise gf.FlightError(f'Vehicle {f.args.target_system}: readiness changed before arming')
            self.yaws[f.args.target_system] = att.yaw
        for f in self.flights:
            f.armed_by_script = True
        self.command_all(mav.MAV_CMD_COMPONENT_ARM_DISARM, 1, 0,
                         state_confirmation=lambda hb: bool(hb.base_mode & mav.MAV_MODE_FLAG_SAFETY_ARMED))
        self.wait(lambda: all(f.link.fresh('HEARTBEAT', 3).base_mode & mav.MAV_MODE_FLAG_SAFETY_ARMED
                              for f in self.flights), 5, 'both aircraft armed')
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

        self.wait(climbed, self.args.takeoff_timeout, 'both aircraft at takeoff altitude')
        for f in self.flights:
            f.guard(position=True)
            sid = f.args.target_system
            p = f.link.fresh('LOCAL_POSITION_NED', 1.5)
            self.starts[sid] = (p.x, p.y, self.heights[sid])

    def move(self, targets, label, dwell):
        self.log('phase', name=label, targets=targets, dwell=dwell)
        print(label, targets, flush=True)
        settled_since, next_send, next_report = None, 0, 0
        deadline = time.monotonic() + self.args.move_timeout + dwell
        while time.monotonic() < deadline:
            self.relay.pump()
            now = time.monotonic()
            all_settled = True
            for f in self.flights:
                f.guard(position=True)
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
        raise gf.FlightError(f'Timeout: both aircraft settling at {label}')

    def land(self):
        self.log('phase', name='land')
        self.landing = True
        for f in self.flights:
            f.expected_modes = {gf.GUIDED, gf.LAND}
            f.monitor_mode = False
        self.command_all(mav.MAV_CMD_NAV_LAND, state_confirmation=lambda hb: hb.custom_mode == gf.LAND)
        self.wait(lambda: all(f.link.fresh('HEARTBEAT', 3).custom_mode == gf.LAND for f in self.flights),
                  5, 'both aircraft in LAND')
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
                    raise gf.FlightError(f'Vehicle {sid}: landing heartbeat lost; disarm not sent')
                if hb.custom_mode != gf.LAND:
                    f.link.takeover = True
                    raise gf.FlightError(f'Vehicle {sid}: mode changed during landing')
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
                    raise gf.FlightError(f'Vehicle {sid}: normal disarm unconfirmed; no retry or force-disarm')
            if len(complete) == len(self.flights):
                return
        raise gf.FlightError('Landing not confirmed for both aircraft; no force-disarm')

    def run(self):
        self.wait(lambda: all(f.link.fresh('HEARTBEAT', 3) for f in self.flights), 20, 'both autopilot heartbeats')
        self.relay.guard_callback = self.check_group
        for f in self.flights:
            self.prepare(f)
        self.takeoff()
        self.move(self.starts, 'hover', self.args.hover)
        backward = {sid: backward_target(start, self.yaws[sid], self.args.distance)
                    for sid, start in self.starts.items()}
        self.move(backward, 'backward', self.args.hold)
        self.move(self.starts, 'return', self.args.hold)
        self.land()
        self.log('complete', systems=self.args.target_systems)
        print('Both aircraft landed and disarmed.', flush=True)

    def abort(self, exc):
        self.relay.guard_callback = None
        for f in self.flights:
            hb = f.link.fresh('HEARTBEAT', 3)
            if hb and f.monitor_mode and hb.custom_mode not in f.expected_modes:
                f.link.takeover = True
            try:
                # A takeover on one aircraft must not suppress landing the other.
                f.abort(gf.FlightError(str(exc)))
            except Exception as abort_error:
                f.log('abort_send_failed', reason=str(abort_error))
        self.relay.heartbeat_enabled = False


def arguments(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--target-systems', nargs=2, type=int, default=[1, 2], metavar=('FIRST', 'SECOND'))
    parser.add_argument('--hover', type=float, default=3, help='Seconds both must hover before moving (default 3)')
    parser.add_argument('--takeoff-timeout', type=float, default=30)
    # Reuse the single-vehicle argument validation and defaults.
    parser.add_argument('--source-system', type=int, default=255)
    parser.add_argument('--listen', default='127.0.0.1:14560')
    parser.add_argument('--qgc', default='127.0.0.1:14550')
    parser.add_argument('--height', type=float, default=1,
                        help='Home-relative takeoff height; NOT guaranteed floor clearance')
    parser.add_argument('--distance', type=float, default=1)
    parser.add_argument('--hold', type=float, default=1)
    parser.add_argument('--speed', type=float, default=.2,
                        help='Pre-takeoff request; not an effective position-leg speed limit')
    parser.add_argument('--position-tolerance', type=float, default=.3)
    parser.add_argument('--settle-speed', type=float, default=.4)
    parser.add_argument('--move-timeout', type=float, default=30)
    parser.add_argument('--lat', type=float)
    parser.add_argument('--lon', type=float)
    parser.add_argument('--amsl', type=float)
    parser.add_argument('--log', type=Path, default=Path('guided-pair.jsonl'))
    parser.add_argument('--execute', action='store_true')
    args = parser.parse_args(argv)
    if len(set(args.target_systems)) != 2 or any(sid not in range(1, 256) for sid in args.target_systems):
        parser.error('Choose two distinct vehicle IDs in 1..255')
    if args.source_system in args.target_systems:
        parser.error('GCS source ID must differ from both vehicle IDs')
    for name in ('hover', 'takeoff_timeout'):
        if not math.isfinite(getattr(args, name)) or getattr(args, name) <= 0:
            parser.error(f'{name} must be finite and positive')
    common = ['--target-system', str(args.target_systems[0])]
    for name, value in vars(args).items():
        if name in ('target_systems', 'hover', 'takeoff_timeout', 'execute') or value is None:
            continue
        common.extend(['--' + name.replace('_', '-'), str(value)])
    validated = gf.arguments(common)
    for name in ('listen', 'qgc', 'lat', 'lon', 'amsl', 'explicit_origin'):
        setattr(args, name, getattr(validated, name))
    return args


def main(argv=None):
    args = arguments(argv)
    print(f'Aircraft {args.target_systems}: takeoff {args.height} m; hover {args.hover} s; '
          f'backward {args.distance} m; return to each hover spot; LAND.')
    print('Backward uses each preflight heading. One shared bridge; no collision avoidance or shared position frame.')
    print('Known altitude-reference issue remains: a 1 m command can produce a ~2 m hover and larger takeoff peak.')
    if not args.execute:
        print('Preview only; no sockets opened or commands sent. Add --execute to fly.')
        return 0
    with args.log.open('x') as logfile:
        def log(event, **fields):
            logfile.write(json.dumps(dict(event=event, monotonic=time.monotonic(), **fields)) + '\n')
            logfile.flush()

        relay = PairRelay(args, log)
        flight = PairFlight(relay, args, log)
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
