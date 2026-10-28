#!/usr/bin/env python3
# AP_FLAKE8_CLEAN
"""One long-running GCS hub for the Crazyflie swarm.

Owns the Crazyradio links, relays everything to QGC, and flies guided_swarm
missions on demand from a prompt -- no second process, no restart between
missions and no log filenames to invent.

    radio dongles <-> swarm_hub.py <-> QGC :14550
                           ^
                   swarm> fly diamond ...

Run it with the nrf-firmware-cus venv (it needs pymavlink and pyusb):

    ../../../nrf51/nrf-firmware-cus/.venv/bin/python companion/swarm_hub.py \\
        --uri radio://0/80/2M/E7E7E7E7E7 --uri radio://0/80/2M/E7E7E7E706

then at the prompt:

    swarm> fly --pattern hover --height 1 --hover 5
    swarm> fly --target-systems 1 2 --pattern diamond --radius 0.75 --duration 35 ...
    swarm> again
    swarm> status
    swarm> quit

Mission options are exactly guided_swarm.py's. --target-systems defaults to
every vehicle the hub can hear. Each mission is previewed and needs a "y"
before anything is sent. Ctrl-C during a mission aborts it the same way
guided_swarm.py does (best-effort LAND, never a forced disarm) and returns to
the prompt; QGC stays connected throughout.

Logs go to companion/logs/<date>/ by default:
    HHMMSS-session.tlog   every MAVLink frame in both directions (opens in
                          MAVExplorer, Mission Planner or QGC log replay)
    HHMMSS-session.jsonl  link statistics, radio failures and recoveries,
                          mission start/end
    HHMMSS-<pattern>-sysN.jsonl  one per mission, in guided_swarm's format

OptiTrack: with --mocap-body RIGID_BODY_ID:SYSID (repeatable), Motive poses are
converted to NED and sent as ATT_POS_MOCAP to the vehicle with that system id
only (mocap.py):

    ... companion/swarm_hub.py --uri ...E701 --uri ...E702 \\
        --mocap-server 192.168.50.208 --mocap-local 192.168.50.198 \\
        --mocap-body 66:1 --mocap-body 67:2
"""

import argparse
import collections
import contextlib
import io
import json
import os
from pathlib import Path
import queue
import shlex
import socket
import struct
import sys
import threading
import time
import traceback

HERE = Path(__file__).resolve().parent
# companion/ is Crazyflie/ArduPilot/optflow_v1/ardupilot/companion on the bench.
DEFAULT_RADIO_TOOLS = (HERE.parents[3] if len(HERE.parents) > 3 else HERE) / 'nrf51' / 'nrf-firmware-cus' / 'tools'

# Seconds without any frame from a vehicle that has been heard before it is
# reported as lost. Autopilot heartbeats arrive at 1 Hz at the very least.
LINK_LOST_S = 3.0
PROMPT = 'swarm> '


def load_bridge(explicit=None):
    """Import mavlink_bridge from the nrf firmware tools directory."""
    candidates = [explicit, os.environ.get('ARDUSWARM_RADIO_TOOLS'), DEFAULT_RADIO_TOOLS, HERE]
    for candidate in candidates:
        if candidate and (Path(candidate) / 'mavlink_bridge.py').is_file():
            sys.path.insert(0, str(Path(candidate).resolve()))
            import mavlink_bridge
            return mavlink_bridge
    raise SystemExit('mavlink_bridge.py not found. Pass --radio-tools DIR or set ARDUSWARM_RADIO_TOOLS '
                     f'(looked in {DEFAULT_RADIO_TOOLS}).')


def heartbeat_fields(frame):
    """(sysid, base_mode, custom_mode) of an autopilot HEARTBEAT, else None.

    Read straight from the header so the radio thread decodes nothing extra.
    """
    if len(frame) < 8:
        return None
    if frame[0] == 0xFE:
        sysid, compid, msgid, payload = frame[3], frame[4], frame[5], frame[6:6 + frame[1]]
    elif frame[0] == 0xFD and len(frame) >= 12:
        sysid, compid = frame[5], frame[6]
        msgid = frame[7] | frame[8] << 8 | frame[9] << 16
        payload = frame[10:10 + frame[1]]
    else:
        return None
    if msgid != 0 or compid != 1:
        return None
    # MAVLink 2 trims trailing zero bytes from the payload.
    payload = bytes(payload) + bytes(9 - min(9, len(payload)))
    if payload[5] == 8:  # MAV_AUTOPILOT_INVALID: a GCS or peripheral
        return None
    return sysid, payload[6], struct.unpack_from('<I', payload)[0]


class LogBook:
    """Hands out new, never-colliding log paths under <root>/<date>/."""

    def __init__(self, root):
        self.root = Path(root)

    def path(self, stem, suffix):
        directory = self.root / time.strftime('%Y-%m-%d')
        directory.mkdir(parents=True, exist_ok=True)
        ignore = self.root / '.gitignore'
        if not ignore.exists():
            # Logs live inside the ardupilot checkout by default; keep git quiet.
            ignore.write_text('*\n')
        base = f"{time.strftime('%H%M%S')}-{stem}"
        candidate = directory / f'{base}{suffix}'
        n = 2
        while candidate.exists():
            candidate = directory / f'{base}-{n}{suffix}'
            n += 1
        return candidate


class SessionLog:
    """Thread-safe session tlog plus event JSONL."""

    def __init__(self, logbook):
        self.tlog_path = logbook.path('session', '.tlog')
        self.events_path = self.tlog_path.with_suffix('.jsonl')
        self._tlog = self.tlog_path.open('xb')
        self._events = self.events_path.open('x', buffering=1)
        self._lock = threading.Lock()
        self._last_flush = 0

    def frame(self, frame):
        # tlog: big-endian microseconds since the epoch, then the raw frame.
        stamp = struct.pack('>Q', int(time.time() * 1e6))
        with self._lock:
            if self._tlog.closed:
                return
            self._tlog.write(stamp + frame)
            now = time.monotonic()
            if now - self._last_flush > 1:
                self._tlog.flush()
                self._last_flush = now

    def event(self, event, **fields):
        line = json.dumps(dict(event=event, time=time.time(), monotonic=time.monotonic(), **fields), default=str)
        with self._lock:
            if not self._events.closed:
                self._events.write(line + '\n')

    def close(self):
        with self._lock:
            self._tlog.close()
            self._events.close()


class SimVehicle:
    """A vehicle reached over UDP instead of radio, e.g. ArduCopter SITL started
    with --serial0 udpclient:127.0.0.1:PORT. Same surface the hub uses on a
    radio Vehicle."""

    def __init__(self, port, bridge):
        self.port = port
        self.uri = f'udp://127.0.0.1:{port}'
        self.sysid = None
        self.parser = bridge.make_parser()
        self.counts = collections.Counter()
        self.stats = collections.Counter()
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(('127.0.0.1', port))
        self.sock.setblocking(False)
        self.peer = None
        self._split = bridge.split_frames
        self._observe = bridge.Vehicle.observe
        self._buffer = bytearray()

    def observe(self, data):
        self._observe(self, data)

    def downlink_frames(self, data):
        self.observe(data)
        self._buffer.extend(data)
        return self._split(self._buffer)

    def queue_frame(self, frame):
        if self.peer is None:
            return
        try:
            self.sock.sendto(frame, self.peer)
            self.stats['up_bytes'] += len(frame)
        except OSError:
            self.stats['send_err'] += 1

    def pending_status(self):
        return {'chunks': 0, 'oldest_ms': 0}

    def label(self):
        return f'sim:{self.port}'


class SimGroup(threading.Thread):
    def __init__(self, vehicles, on_downlink):
        super().__init__(daemon=True)
        self.index = 'sim'
        self.vehicles = vehicles
        self.on_downlink = on_downlink
        self.usb_id = 'udp'
        self.failures = self.reopens = 0
        self.last_failure = None
        self.radio = None
        self._running = True

    def stop(self):
        self._running = False

    def run(self):
        import select
        socks = {v.sock: v for v in self.vehicles}
        while self._running:
            for sock in select.select(list(socks), [], [], 0.1)[0]:
                vehicle = socks[sock]
                try:
                    data, peer = sock.recvfrom(65535)
                except OSError:
                    continue
                vehicle.peer = peer
                vehicle.stats['down_bytes'] += len(data)
                vehicle.stats['polls'] += 1
                for frame in vehicle.downlink_frames(data):
                    vehicle.stats['down_frames'] += 1
                    try:
                        self.on_downlink(vehicle, frame)
                    except Exception as exc:  # noqa: BLE001
                        print(f'downlink handler error: {exc!r}', flush=True)


class QgcLink(threading.Thread):
    """UDP to QGC. Replies go to wherever QGC last sent from."""

    def __init__(self, hub, peer, split_frames):
        super().__init__(daemon=True)
        self.hub = hub
        self.peer = peer
        self.split_frames = split_frames
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(('0.0.0.0', 0))
        self.sock.setblocking(False)
        self.frames_in = self.frames_out = 0
        self.last_rx = None
        self._buffer = bytearray()
        self._running = True

    def stop(self):
        self._running = False

    def send(self, frame):
        try:
            self.sock.sendto(frame, self.peer)
            self.frames_out += 1
        except OSError:
            # Nobody listening, or the network is changing: QGC just misses it.
            pass

    def run(self):
        import select
        while self._running:
            try:
                ready = select.select([self.sock], [], [], 0.1)[0]
            except (OSError, ValueError):
                return
            if not ready:
                continue
            for _ in range(64):
                try:
                    data, src = self.sock.recvfrom(65535)
                except (BlockingIOError, OSError):
                    break
                self.peer = src
                self.last_rx = time.monotonic()
                self._buffer += data
            for frame in self.split_frames(self._buffer):
                self.frames_in += 1
                try:
                    self.hub.uplink(frame, from_qgc=True)
                except Exception as exc:  # noqa: BLE001
                    self.hub.notify(f'QGC uplink error: {exc!r}')


class VehicleState:
    def __init__(self):
        self.last_rx = None
        self.lost = False
        self.armed = False
        self.mode = None


class Hub:
    """Radio links, QGC relay and the attach point for one mission at a time."""

    def __init__(self, args, bridge, guided):
        self.args = args
        self.bridge = bridge
        self.guided = guided
        self.router = bridge.Router()
        self.logbook = LogBook(args.log_dir)
        self.session = SessionLog(self.logbook)
        self.vehicles = []
        self.groups = []
        self.state = {}
        self.mission = None
        self.mocap = None
        self._mission_lock = threading.Lock()
        # The router's decoder is stateful; QGC and mission threads both route.
        self._uplink_lock = threading.Lock()
        self.qgc = QgcLink(self, args.qgc, bridge.split_frames)
        self._running = True
        self._stats_previous = {}

    # ---------------------------------------------------------------- set-up

    def open_links(self):
        bridge = self.bridge
        by_index = collections.OrderedDict()
        for uri in self.args.uri or []:
            v = bridge.Vehicle(uri)
            by_index.setdefault(v.index, []).append(v)
        for index, members in by_index.items():
            try:
                group = bridge.RadioGroup(index, members, self.downlink, self.args.idle_poll_ms,
                                          on_event=self.notify)
            except Exception as exc:  # noqa: BLE001
                raise SystemExit(f'Could not open dongle index {index}: {exc}\n'
                                 'Check it is plugged in and running the large-packet firmware.')
            self.groups.append(group)
            shared = ' (round-robin)' if len(members) > 1 else ''
            print(f'Dongle {index} [{group.usb_id}]{shared}')
            for v in members:
                print(f'    {v.uri}')
        for w in bridge.configuration_warnings(by_index):
            print(f'WARNING: {w}')
        sims = [SimVehicle(port, bridge) for port in self.args.sim or []]
        if sims:
            self.groups.append(SimGroup(sims, self.downlink))
            for v in sims:
                print(f'Simulated vehicle on {v.uri}')
        for g in self.groups:
            self.vehicles.extend(g.vehicles)
        for v in self.vehicles:
            self.state[v] = VehicleState()
        if not self.vehicles:
            raise SystemExit('No vehicles: give at least one --uri (or --sim PORT).')

    def start(self):
        self.session.event('session_start', argv=sys.argv, uris=[v.uri for v in self.vehicles],
                           qgc=self.args.qgc)
        self.qgc.start()
        for g in self.groups:
            g.start()
        threading.Thread(target=self._monitor, daemon=True).start()
        print(f'QGC: sending to {self.args.qgc[0]}:{self.args.qgc[1]} (local port {self.qgc.sock.getsockname()[1]})')
        print(f'Logs: {self.session.tlog_path.parent}/')
        if getattr(self.args, 'mocap_body', None):
            self.start_mocap()

    def start_mocap(self, client_factory=None):
        import mocap
        args = self.args
        self.mocap = mocap.MocapForwarder(
            self, args.mocap_body, frame=args.mocap_frame, rate_hz=args.mocap_rate,
            max_queue=args.mocap_max_queue, server=args.mocap_server, local=args.mocap_local,
            multicast=not args.mocap_unicast, client_factory=client_factory)
        self.session.event('mocap_start', server=args.mocap_server, local=args.mocap_local,
                           bodies=args.mocap_body, frame=args.mocap_frame, rate_hz=args.mocap_rate)
        rate = f'{args.mocap_rate:g} Hz per body' if args.mocap_rate else 'every frame'
        print(f'OptiTrack: Motive {args.mocap_server} via {args.mocap_local}, {args.mocap_frame}, {rate}')
        for body, sysid in args.mocap_body.items():
            print(f'    rigid body {body} -> system {sysid}')
        self.mocap.start()

    def stop(self):
        self._running = False
        if self.mocap is not None:
            self.mocap.stop()
            self.session.event('mocap_stats', **self.mocap.summary())
        for g in self.groups:
            g.stop()
        self.qgc.stop()
        for thread in [*self.groups, self.qgc]:
            if thread.is_alive():
                thread.join(timeout=1.5)
        self.qgc.sock.close()
        for v in self.vehicles:
            if isinstance(v, SimVehicle):
                v.sock.close()
        self.session.event('session_end')
        self.session.close()

    # --------------------------------------------------------------- traffic

    def downlink(self, vehicle, frame):
        """Radio thread: one complete frame from one vehicle."""
        self.router.learn(vehicle.sysid, vehicle)
        state = self.state[vehicle]
        state.last_rx = time.monotonic()
        hb = heartbeat_fields(frame)
        if hb is not None and hb[0] == vehicle.sysid:
            state.armed = bool(hb[1] & 0x80)
            state.mode = hb[2]
        self.session.frame(frame)
        self.qgc.send(frame)
        relay = self.mission
        if relay is not None:
            relay.inbox.put((False, frame))

    def uplink(self, frame, from_qgc):
        """Any thread: one frame towards the vehicles."""
        self.session.frame(frame)
        with self._uplink_lock:
            targets = self.router.vehicles_for(frame, self.vehicles)
        for v in targets:
            v.queue_frame(frame)
        if from_qgc:
            relay = self.mission
            if relay is not None:
                relay.inbox.put((True, frame))

    def vehicle_for_sysid(self, sysid):
        """The one link whose autopilot heartbeat carries sysid; None if unheard or ambiguous."""
        matches = [v for v in self.vehicles if v.sysid == sysid]
        return matches[0] if len(matches) == 1 else None

    def send_to_vehicle(self, vehicle, frame):
        """Any thread: a frame for exactly one vehicle, bypassing the router."""
        self.session.frame(frame)
        vehicle.queue_frame(frame)

    def attach(self, relay):
        with self._mission_lock:
            if self.mission is not None:
                raise RuntimeError('A mission is already running')
            self.mission = relay

    def detach(self, relay):
        with self._mission_lock:
            if self.mission is relay:
                self.mission = None

    # ---------------------------------------------------------------- health

    def notify(self, text):
        self.session.event('notice', text=text)
        print(f'\r[hub] {text}', flush=True)

    def heard(self, max_age=LINK_LOST_S):
        """System IDs heard from recently, sorted."""
        now = time.monotonic()
        return sorted({v.sysid for v in self.vehicles
                       if v.sysid is not None and self.state[v].last_rx is not None
                       and now - self.state[v].last_rx <= max_age})

    def name(self, vehicle):
        return f'Vehicle {vehicle.sysid}' if vehicle.sysid is not None else vehicle.label()

    def _monitor(self):
        next_stats = time.monotonic() + self.args.stats_sec
        while self._running:
            time.sleep(0.25)
            now = time.monotonic()
            for v in self.vehicles:
                s = self.state[v]
                if s.last_rx is None:
                    continue
                silent = now - s.last_rx
                if silent > LINK_LOST_S and not s.lost:
                    s.lost = True
                    self.notify(f'{self.name(v)} ({v.label()}): no telemetry for {silent:.0f} s -- link lost')
                elif silent <= LINK_LOST_S and s.lost:
                    s.lost = False
                    self.notify(f'{self.name(v)} ({v.label()}): link restored')
            if self.mocap is not None:
                self.mocap.check(now)
            if now >= next_stats:
                next_stats = now + self.args.stats_sec
                if self.mocap is not None:
                    self.session.event('mocap_stats', **self.mocap.summary())
                for g in self.groups:
                    for v in g.vehicles:
                        self.session.event(
                            'link_stats', uri=v.uri, sysid=v.sysid, dongle=g.index, usb_id=g.usb_id,
                            counters=dict(v.stats), queue=v.pending_status(),
                            worker_alive=g.is_alive(), failures=g.failures, reopens=g.reopens,
                            last_failure=g.last_failure,
                            last_usb_error=getattr(g.radio, 'last_error', None))

    def status_lines(self):
        now = time.monotonic()
        lines = []
        for g in self.groups:
            alive = 'ok' if g.is_alive() else 'STOPPED'
            extra = f', {g.reopens} reopen(s), last failure: {g.last_failure}' if g.failures else ''
            lines.append(f'dongle {g.index} [{g.usb_id}] worker {alive}{extra}')
            for v in g.vehicles:
                s = self.state[v]
                totals = dict(v.stats)
                previous, then = self._stats_previous.get(v, ({}, None))
                self._stats_previous[v] = (totals, now)
                rate = ''
                if then is not None and now > then:
                    dt = now - then
                    d = {k: totals.get(k, 0) - previous.get(k, 0) for k in ('polls', 'down_bytes', 'no_ack')}
                    rate = (f' polls {d["polls"] / dt:.0f}/s down {d["down_bytes"] / dt:.0f} B/s'
                            f' no-ack {d["no_ack"]}')
                if s.last_rx is None:
                    link = 'never heard'
                else:
                    link = f'last rx {now - s.last_rx:.1f} s ago' + (' LOST' if s.lost else '')
                if getattr(v, 'silent', False):
                    link += ', radio not answering (polled 4x/s)'
                armed = 'ARMED' if s.armed else 'disarmed'
                mode = f' mode {s.mode}' if s.mode is not None else ''
                pending = v.pending_status()
                lines.append(f'    sys{v.sysid if v.sysid is not None else "?"} {v.label()}: {link}, {armed}{mode},'
                             f' queue {pending["chunks"]}{rate}')
        qgc_age = 'never' if self.qgc.last_rx is None else f'{now - self.qgc.last_rx:.1f} s ago'
        lines.append(f'QGC {self.qgc.peer[0]}:{self.qgc.peer[1]}: last heard {qgc_age}, '
                     f'{self.qgc.frames_out} frames out, {self.qgc.frames_in} in')
        if self.mocap is not None:
            lines.extend(self.mocap.status_lines(now))
        lines.append(f'Logs: {self.session.tlog_path}')
        return lines


class MissionRelay:
    """guided_swarm's Relay interface, fed by the hub instead of UDP sockets.

    Only the main (console) thread touches the vehicle caches: radio and QGC
    threads hand frames over through `inbox`.
    """

    def __init__(self, hub, args, log):
        mav = hub.guided.mav
        self.hub = hub
        self.log = log
        self.source_system = args.source_system
        self.inbox = queue.Queue()
        self.last_heartbeat = 0
        self.heartbeat_enabled = True
        self.guard_callback = None
        self.mav = mav.MAVLink(self, srcSystem=args.source_system, srcComponent=191)
        self.decoder = mav.MAVLink(None)
        self.decoder.robust_parsing = True
        self.qgc_decoder = mav.MAVLink(None)
        self.qgc_decoder.robust_parsing = True
        self.links = {sid: hub.guided.VehicleLink(self, sid) for sid in args.target_systems}

    def write(self, data):
        self.hub.uplink(bytes(data), from_qgc=False)

    def pump(self, timeout=0.05):
        mav = self.hub.guided.mav
        items = []
        try:
            items.append(self.inbox.get(timeout=timeout))
            while True:
                items.append(self.inbox.get_nowait())
        except queue.Empty:
            pass
        for from_qgc, frame in items:
            if from_qgc:
                for message in self.qgc_decoder.parse_buffer(frame) or []:
                    if message.get_type() == 'BAD_DATA':
                        continue
                    for link in self.links.values():
                        link.qgc_command(message)
                continue
            for message in self.decoder.parse_buffer(frame) or []:
                if message.get_type() == 'BAD_DATA':
                    continue
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
        if self.heartbeat_enabled and now - self.last_heartbeat >= 1:
            self.mav.heartbeat_send(mav.MAV_TYPE_GCS, mav.MAV_AUTOPILOT_INVALID, 0, 0, 0)
            self.last_heartbeat = now
        if self.guard_callback:
            self.guard_callback()


class Console:
    HELP = """Commands:
  fly [options]      preview a guided_swarm mission, confirm, fly it
  preview [options]  show the plan only
  again              preview and fly the previous mission again
  status             links, dongles, QGC, OptiTrack and log location
  help fly           all mission options (guided_swarm.py's)
  quit               leave (Ctrl-D too)
Mission options are guided_swarm.py's; --execute, --log, --listen and --qgc are
not needed. --target-systems defaults to every vehicle currently heard.
Ctrl-C during a mission aborts it (best-effort LAND) and returns here."""

    def __init__(self, hub):
        self.hub = hub
        self.last_argv = None
        self.last_interrupt = 0

    def mission_argv(self, words):
        """Drop hub-managed options and fill in --target-systems."""
        argv, explicit_log, skip = [], None, False
        for i, word in enumerate(words):
            if skip:
                skip = False
                continue
            if word == '--execute':
                continue
            if word in ('--log', '--listen', '--qgc'):
                if word == '--log' and i + 1 < len(words):
                    explicit_log = Path(words[i + 1])
                skip = True
                continue
            if word.startswith(('--log=', '--listen=', '--qgc=')):
                if word.startswith('--log='):
                    explicit_log = Path(word.split('=', 1)[1])
                continue
            argv.append(word)
        if not any(w == '--target-systems' or w.startswith('--target-systems=') for w in argv):
            heard = self.hub.heard()
            if heard:
                argv = ['--target-systems', *map(str, heard)] + argv
        return argv, explicit_log

    def preview(self, argv):
        """Print the plan; return parsed args or None if the options are invalid."""
        guided = self.hub.guided
        errors = io.StringIO()
        try:
            with contextlib.redirect_stderr(errors):
                args = guided.arguments(argv)
        except SystemExit:
            # Only argparse's final "error:" line; "help fly" shows the usage.
            lines = errors.getvalue().strip().splitlines()
            print(f'Invalid mission options: {lines[-1].split("error: ", 1)[-1] if lines else "see help fly"}')
            return None
        assert not args.execute
        out = io.StringIO()
        with contextlib.redirect_stdout(out):
            guided.main(argv)
        for line in out.getvalue().splitlines():
            if line.startswith(('Preview only', 'One bridge process')):
                continue
            print(line)
        heard = self.hub.heard()
        missing = [sid for sid in args.target_systems if sid not in heard]
        if missing:
            print(f'WARNING: not hearing system(s) {missing} right now (hearing {heard or "none"}).')
        return args

    def confirm(self, question):
        try:
            return input(f'{question} [y/N] ').strip().lower() in ('y', 'yes')
        except (EOFError, KeyboardInterrupt):
            print()
            return False

    def fly(self, words):
        argv, explicit_log = self.mission_argv(words)
        args = self.preview(argv)
        if args is None:
            return
        self.last_argv = words
        if not self.confirm('Fly this mission?'):
            print('Not flown.')
            return
        run_mission(self.hub, args, argv, explicit_log)

    def status(self):
        for line in self.hub.status_lines():
            print(line)

    def quit_ok(self):
        armed = [v.sysid for v in self.hub.vehicles if self.hub.state[v].armed and not self.hub.state[v].lost]
        if armed:
            return self.confirm(f'System(s) {armed} report ARMED; quitting cuts QGC off from them. Quit anyway?')
        return True

    def run(self):
        try:
            import readline  # noqa: F401  (line editing and up-arrow history)
        except ImportError:
            pass
        print('\nType "help" for commands. QGC is live now; missions are optional.\n')
        while True:
            try:
                line = input(PROMPT)
            except EOFError:
                print()
                if self.quit_ok():
                    return
                continue
            except KeyboardInterrupt:
                now = time.monotonic()
                if now - self.last_interrupt < 2 and self.quit_ok():
                    print()
                    return
                self.last_interrupt = now
                print('\n(Ctrl-C again or "quit" to exit)')
                continue
            try:
                words = shlex.split(line)
            except ValueError as exc:
                print(f'Could not parse that: {exc}')
                continue
            if not words:
                continue
            command, rest = words[0].lower(), words[1:]
            try:
                if command in ('quit', 'exit', 'q'):
                    if self.quit_ok():
                        return
                elif command == 'help':
                    if rest and rest[0] == 'fly':
                        with contextlib.suppress(SystemExit):
                            self.hub.guided.arguments(['--help'])
                    else:
                        print(self.HELP)
                elif command == 'status':
                    self.status()
                elif command == 'preview':
                    self.preview(self.mission_argv(rest)[0])
                elif command == 'fly':
                    self.fly(rest)
                elif command == 'again':
                    if self.last_argv is None:
                        print('No previous mission.')
                    else:
                        print('Previous mission: fly ' + ' '.join(shlex.quote(w) for w in self.last_argv))
                        self.fly(self.last_argv)
                else:
                    print(f'Unknown command "{command}". Type "help".')
            except KeyboardInterrupt:
                print('\nInterrupted.')


def mission_log_path(hub, args, explicit_log):
    if explicit_log is not None:
        path = explicit_log
        n = 2
        while path.exists():
            path = explicit_log.with_name(f'{explicit_log.stem}-{n}{explicit_log.suffix}')
            n += 1
        return path
    systems = '-'.join(map(str, args.target_systems))
    return hub.logbook.path(f'{args.pattern}-sys{systems}', '.jsonl')


def run_mission(hub, args, argv, explicit_log=None):
    """Fly one guided_swarm mission through the hub. Returns True on success.

    Runs in the main thread so Ctrl-C lands here, as it did in guided_swarm.py.
    """
    guided = hub.guided
    path = mission_log_path(hub, args, explicit_log)
    path.parent.mkdir(parents=True, exist_ok=True)
    ok = False
    with path.open('x') as logfile:
        def log(event, **fields):
            logfile.write(json.dumps(dict(event=event, monotonic=time.monotonic(), **fields), default=str) + '\n')
            logfile.flush()

        points = guided.layout(len(args.target_systems), args.spacing)
        log('plan', systems=args.target_systems, markers=points, pattern=args.pattern, radius=args.radius,
            distance=args.distance, height=args.height, hover=args.hover, hold=args.hold,
            duration=args.duration, tracking_tolerance=args.tracking_tolerance,
            max_flight_time=args.max_flight_time, argv=argv, hub_session=str(hub.session.tlog_path))
        hub.session.event('mission_start', log=str(path), argv=argv)
        print(f'Mission log: {path}', flush=True)
        relay = MissionRelay(hub, args, log)
        flight = guided.SwarmFlight(relay, args, log)
        hub.attach(relay)
        reason = None
        try:
            try:
                flight.run()
                ok = True
            except (Exception, KeyboardInterrupt) as exc:
                if isinstance(exc, KeyboardInterrupt):
                    reason = guided.FlightError('stopped by operator (Ctrl-C)')
                elif isinstance(exc, guided.FlightError):
                    reason = exc
                else:
                    log('unexpected_exception', traceback=traceback.format_exc())
                    reason = guided.FlightError(f'{type(exc).__name__}: {exc}')
                try:
                    flight.abort(reason)
                except (Exception, KeyboardInterrupt) as abort_error:
                    log('abort_failed', reason=repr(abort_error))
                    print(f'Abort did not complete: {abort_error!r}. Use QGC.', flush=True)
        finally:
            relay.guard_callback = None
            relay.heartbeat_enabled = False
            for link in relay.links.values():
                link.watch_takeover = False
            hub.detach(relay)
            hub.session.event('mission_end', log=str(path), ok=ok, reason=str(reason) if reason else None)
    print('Mission complete.' if ok else 'Mission ended early; see above. QGC remains connected.', flush=True)
    return ok


def arguments(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--uri', action='append', metavar='URI',
                    help='radio URI, repeatable; URIs sharing a dongle index are round-robined on that dongle')
    ap.add_argument('--sim', action='append', type=int, metavar='PORT',
                    help='also link a simulated vehicle that sends to UDP 127.0.0.1:PORT '
                         '(ArduCopter SITL --serial0 udpclient:127.0.0.1:PORT); repeatable')
    ap.add_argument('--qgc', default='127.0.0.1:14550', metavar='HOST:PORT',
                    help='where QGC listens (default 127.0.0.1:14550)')
    ap.add_argument('--log-dir', type=Path, default=HERE / 'logs',
                    help=f'log root; a dated folder is created per day (default {HERE / "logs"})')
    ap.add_argument('--radio-tools', metavar='DIR',
                    help=f'folder containing mavlink_bridge.py (default {DEFAULT_RADIO_TOOLS})')
    ap.add_argument('--idle-poll-ms', type=float, default=2.0,
                    help='pause after a polling round with no traffic (default 2.0)')
    ap.add_argument('--stats-sec', type=float, default=2.0,
                    help='seconds between link_stats records in the session log (default 2.0)')
    mocap = ap.add_argument_group('OptiTrack (NatNet) forwarding')
    mocap.add_argument('--mocap-body', action='append', metavar='RIGID_BODY_ID:SYSID',
                       help='forward this Motive rigid body to this MAVLink system as ATT_POS_MOCAP; repeatable')
    mocap.add_argument('--mocap-server', metavar='IP', help='Motive (NatNet server) address')
    mocap.add_argument('--mocap-local', metavar='IP', help='this computer\'s address on the Motive network')
    mocap.add_argument('--mocap-unicast', action='store_true', help='unicast NatNet instead of multicast')
    mocap.add_argument('--mocap-frame', choices=('y-up', 'z-up'), default='y-up',
                       help='Motive up axis: y-up (default; N=-Z, E=X, D=-Y) or z-up (N=X, E=-Y, D=-Z)')
    mocap.add_argument('--mocap-rate', type=float, default=30.0, metavar='HZ',
                       help='maximum poses per second per vehicle; 0 sends every Motive frame (default 30)')
    mocap.add_argument('--mocap-max-queue', type=int, default=2, metavar='CHUNKS',
                       help='skip a pose while the vehicle\'s uplink queue holds more chunks (default 2)')
    args = ap.parse_args(argv)
    if args.mocap_body:
        import mocap as mocap_module
        try:
            args.mocap_body = mocap_module.body_map(mocap_module.parse_body(b) for b in args.mocap_body)
        except ValueError as exc:
            ap.error(f'--mocap-body: {exc}')
        if not (args.mocap_server and args.mocap_local):
            ap.error('--mocap-body needs --mocap-server and --mocap-local')
        if args.mocap_rate < 0:
            ap.error('--mocap-rate must be >= 0')
    host, _, port = args.qgc.rpartition(':')
    try:
        args.qgc = (socket.gethostbyname(host), int(port))
    except (OSError, ValueError):
        ap.error('--qgc must be HOST:PORT')
    if not args.uri and not args.sim:
        args.uri = ['radio://0/80/2M/E7E7E7E7E7']
    return args


def main(argv=None):
    args = arguments(argv)
    bridge = load_bridge(args.radio_tools)
    sys.path.insert(0, str(HERE))
    import guided_swarm
    hub = Hub(args, bridge, guided_swarm)
    hub.open_links()
    try:
        hub.start()
        Console(hub).run()
    finally:
        print('Closing links...', flush=True)
        hub.stop()
        print(f'Session log: {hub.session.tlog_path}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
