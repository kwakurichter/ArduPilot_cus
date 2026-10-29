#!/usr/bin/env python3
# AP_FLAKE8_CLEAN
"""OptiTrack (Motive / NatNet) poses forwarded to each vehicle by swarm_hub.

Every Motive rigid body is mapped to one MAVLink system id. Each frame from
Motive is converted to NED / FRD and sent as ATT_POS_MOCAP to that vehicle
only, as if from its own onboard component 197 (visual-inertial odometry).
ArduPilot hands it to AP_VisualOdom (VISO_TYPE = 1) and EKF3's ExternalNav
source.

ATT_POS_MOCAP has no target_system, so the hub's router would copy it to every
vehicle on the link. The forwarder therefore queues each pose directly on the
vehicle whose heartbeat carries the mapped system id.

The radio uplink sends one frame per poll, so a pose never waits behind a
backlog: it is skipped while the vehicle's uplink queue holds more than
`max_queue` chunks, and poses are thinned to `rate_hz` per body.

Timing (docs/plans/p5_global_fix_plan.md 3.5 in the thesis repository). Each pose is
stamped at its capture: the NatNet arrival minus Motive's own latency (camera
mid-exposure to transmit, from the frame suffix; its tick rate is estimated against
the hub clock) or a fixed `latency_ms`. In `time_mode='hub'` time_usec is that capture
in the hub's monotonic clock (ArduPilot's stock jitter correction absorbs the offset);
in `time_mode='fc'` it is mapped into each vehicle's micros64() with TIMESYNC
(clocksync.ClockSync), which AP_CoopBridge uses as the capture time (COOP_FIX_TIME 1).
A vehicle gets no poses until its clock fit is ready.

Every Motive frame (all rigid bodies, full rate) can also go to a CSV ground-truth log,
independent of what is sent.
"""

import csv
import math
import threading
import time

import clocksync

from pymavlink.dialects.v20 import ardupilotmega as mav

# NED <- Motive world, as rows (north, east, down) of Motive (x, y, z).
# The same rotation maps Motive's rigid-body axes to FRD, so a rigid body
# must be created in Motive with the vehicle's nose along the axis that maps
# to north and its top along "up".
FRAMES = {
    # Motive default (Y up): north = -Z, east = +X, down = -Y.
    'y-up': ((0, 0, -1), (1, 0, 0), (0, -1, 0)),
    # Motive "Z up" (X forward, Y left): north = +X, east = -Y, down = -Z.
    'z-up': ((1, 0, 0), (0, -1, 0), (0, 0, -1)),
}

MAV_COMP_ID_VISUAL_INERTIAL_ODOMETRY = 197

# A body that has not produced a valid pose for this long is reported lost.
TRACKING_LOST_S = 0.5

# Motive latency assumed until the frame suffix's tick rate is known (or without it).
DEFAULT_LATENCY_MS = 5.0
# Hub-clock span over which the suffix tick rate is estimated before it is used.
TICK_RATE_SPAN_S = 2.0
# TIMESYNC requests per second per vehicle.
SYNC_HZ = 2.0

# Tolerance on the rate-limit grid for frame arrival jitter.
RATE_SLACK_S = 0.001


def _determinant(m):
    return (m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1])
            - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
            + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]))


def _apply(m, v):
    return tuple(m[i][0] * v[0] + m[i][1] * v[1] + m[i][2] * v[2] for i in range(3))


class PoseConverter:
    """Motive position and quaternion (x, y, z, w) -> NED position and (w, x, y, z).

    One rotation C is used for both, so position and attitude cannot disagree:
    p_ned = C p,  R_ned<-frd = C R C^T, whose quaternion is (w, C v).
    """

    def __init__(self, frame='y-up'):
        if frame not in FRAMES:
            raise ValueError(f'unknown mocap frame {frame!r}; choose from {sorted(FRAMES)}')
        self.frame = frame
        self.matrix = FRAMES[frame]
        if abs(_determinant(self.matrix) - 1) > 1e-9:
            raise ValueError(f'mocap frame {frame!r} is not a proper rotation')

    def position(self, pos):
        return _apply(self.matrix, pos)

    def quaternion(self, rot):
        x, y, z, w = rot
        norm = math.sqrt(x * x + y * y + z * z + w * w)
        if not norm > 0:
            raise ValueError('zero quaternion')
        vx, vy, vz = _apply(self.matrix, (x / norm, y / norm, z / norm))
        w /= norm
        if w < 0:  # one hemisphere, for readable logs
            w, vx, vy, vz = -w, -vx, -vy, -vz
        return [w, vx, vy, vz]


def parse_body(text):
    """'RIGID_BODY_ID:SYSID' -> (rigid_body_id, sysid)."""
    try:
        body, sysid = (int(part) for part in text.split(':'))
    except ValueError:
        raise ValueError(f'expected RIGID_BODY_ID:SYSID, got {text!r}') from None
    if not 1 <= sysid <= 255 or body < 0:
        raise ValueError(f'bad mapping {text!r}: sysid must be 1-255, rigid body id >= 0')
    return body, sysid


def body_map(pairs):
    """{rigid_body_id: sysid} from (body, sysid) pairs; each side used once."""
    bodies, systems = {}, {}
    for body, sysid in pairs:
        if body in bodies:
            raise ValueError(f'rigid body {body} is mapped twice')
        if sysid in systems:
            raise ValueError(f'system {sysid} is fed by rigid bodies {systems[sysid]} and {body}')
        bodies[body], systems[sysid] = sysid, body
    return bodies


class LatencyEstimator:
    """Motive's latency (camera mid-exposure to transmit) of each frame, in seconds.

    NatNet 3+ frames carry both stamps in the server's high-resolution ticks; the `natnet`
    package does not decode the tick rate from the server description, so it is estimated
    from the transmit stamps against the hub's clock over at least TICK_RATE_SPAN_S. Until
    then, or without the stamps, the fixed latency is used; a given `fixed_ms` always wins.
    """

    def __init__(self, fixed_ms=None):
        self.fixed_ms = fixed_ms
        self.tick_hz = None
        self._first = None    # (hub s, transmit ticks)
        self.last_s = None

    def latency_s(self, data_frame, now):
        if self.fixed_ms is not None:
            self.last_s = self.fixed_ms / 1e3
            return self.last_s
        suffix = getattr(data_frame, 'suffix', None)
        mid = getattr(suffix, 'stamp_camera_mid_exposure', None)
        transmit = getattr(suffix, 'stamp_transmit', None)
        latency = DEFAULT_LATENCY_MS / 1e3
        if isinstance(mid, int) and isinstance(transmit, int) and transmit > 0:
            if self._first is None or transmit < self._first[1]:
                self._first = (now, transmit)
            elif now - self._first[0] >= TICK_RATE_SPAN_S:
                self.tick_hz = (transmit - self._first[1]) / (now - self._first[0])
            if self.tick_hz:
                measured = (transmit - mid) / self.tick_hz
                if 0.0 <= measured < 0.2:
                    latency = measured
        self.last_s = latency
        return latency


class TruthLog:
    """CSV of every rigid body in every Motive frame (raw Motive frame and NED)."""

    COLUMNS = ('hub_s', 'capture_hub_s', 'frame', 'motive_s', 'latency_ms', 'rigid_body', 'sysid',
               'tracked', 'marker_error_m', 'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw',
               'north', 'east', 'down', 'qw_ned', 'qx_ned', 'qy_ned', 'qz_ned', 'capture_fc_us')

    def __init__(self, path):
        self.path = path
        self._file = open(path, 'x', newline='')
        self._writer = csv.writer(self._file)
        self._writer.writerow(self.COLUMNS)
        self._last_flush = 0.0
        self.rows = 0

    def write(self, row, now):
        self._writer.writerow(row)
        self.rows += 1
        if now - self._last_flush > 1.0:
            self._file.flush()
            self._last_flush = now

    def close(self):
        if not self._file.closed:
            self._file.close()


class BodyStats:
    def __init__(self, body, sysid):
        self.body = body
        self.sysid = sysid
        self.frames = 0          # frames that contained this body
        self.sent = 0
        self.untracked = 0       # Motive reported tracking_valid = False
        self.thinned = 0         # skipped by the rate limit
        self.backlog = 0         # skipped because the uplink queue was busy
        self.no_vehicle = 0      # system id not heard (or heard on two links)
        self.unsynced = 0        # time_mode 'fc' and the vehicle's clock fit is not ready yet
        self.last_valid = None   # monotonic time of the last valid pose
        self.last_sent = None
        self.next_due = None     # rate limit: send when a frame reaches this time
        self.last_ned = None
        self.last_error = None   # Motive mean marker error, metres
        self.lost = False
        self.previous = (0, None)

    def counters(self):
        return dict(frames=self.frames, sent=self.sent, untracked=self.untracked, thinned=self.thinned,
                    backlog=self.backlog, no_vehicle=self.no_vehicle, unsynced=self.unsynced)


class MocapForwarder:
    """Feeds ATT_POS_MOCAP to each mapped vehicle from NatNet frames.

    `on_frame` runs in the NatNet receive thread; everything it touches on the
    hub (vehicle uplink queues, session log) is thread-safe.
    """

    def __init__(self, hub, bodies, frame='y-up', rate_hz=30.0, max_queue=2,
                 server='127.0.0.1', local='127.0.0.1', multicast=True,
                 component=MAV_COMP_ID_VISUAL_INERTIAL_ODOMETRY, client_factory=None,
                 time_mode='hub', latency_ms=None, truth_path=None):
        if time_mode not in ('hub', 'fc'):
            raise ValueError(f'unknown mocap time mode {time_mode!r}')
        self.time_mode = time_mode
        self.latency = LatencyEstimator(latency_ms)
        self.truth = TruthLog(truth_path) if truth_path is not None else None
        self.clocks = {sysid: clocksync.ClockSync(sysid) for sysid in bodies.values()}
        self._last_sync = None
        self.hub = hub
        self.bodies = dict(bodies)
        self.converter = PoseConverter(frame)
        self.min_interval = 1.0 / rate_hz if rate_hz and rate_hz > 0 else 0.0
        self.max_queue = max_queue
        self.server, self.local, self.multicast = server, local, multicast
        self.component = component
        self.client_factory = client_factory
        self.client = None
        self.frames = 0
        self.errors = 0
        self.last_frame = None
        self.stats = {body: BodyStats(body, sysid) for body, sysid in self.bodies.items()}
        # One encoder per vehicle: each is a separate MAVLink source with its own sequence.
        self._encoders = {sysid: mav.MAVLink(None, srcSystem=sysid, srcComponent=component)
                          for sysid in self.bodies.values()}
        self._lock = threading.Lock()

    # ------------------------------------------------------------- NatNet

    def start(self):
        factory = self.client_factory
        if factory is None:
            try:
                from natnet import NatNetClient
            except ImportError:
                raise SystemExit('OptiTrack forwarding needs the NatNet client: pip install natnet') from None
            factory = NatNetClient
        self.client = factory(server_ip_address=self.server, local_ip_address=self.local,
                              use_multicast=self.multicast)
        self.client.on_data_frame_received_event.handlers.append(self.on_frame)
        try:
            self.client.connect(timeout=3.0)
        except TimeoutError:
            self.client = None
            raise SystemExit(f'No answer from Motive at {self.server} (local interface {self.local}). '
                             'Check both addresses and that streaming is on in Motive.') from None
        except OSError as exc:
            self.client = None
            raise SystemExit(f'Could not open NatNet sockets on {self.local}: {exc}') from None
        self.client.run_async()

    def stop(self):
        client, self.client = self.client, None
        if client is not None:
            try:
                client.shutdown()
            except Exception:  # noqa: BLE001
                pass
        if self.truth is not None:
            with self._lock:
                self.truth.close()

    # ------------------------------------------------------------- TIMESYNC

    def tick(self, now=None):
        """Hub monitor: send due TIMESYNC requests, straight to each mapped vehicle."""
        now = time.monotonic() if now is None else now
        if self._last_sync is not None and now - self._last_sync < 1.0 / SYNC_HZ:
            return
        self._last_sync = now
        for sysid, clock in self.clocks.items():
            vehicle = self.hub.vehicle_for_sysid(sysid)
            if vehicle is None:
                continue
            message = mav.MAVLink_timesync_message(0, clock.request())
            self.hub.send_to_vehicle(vehicle, message.pack(self._encoders[sysid]))

    def on_downlink(self, vehicle, frame):
        """Radio thread: take a TIMESYNC answer from a mapped vehicle."""
        fields = clocksync.timesync_fields(frame)
        if fields is None:
            return False
        sysid, _, tc1, ts1 = fields
        clock = self.clocks.get(sysid)
        if clock is None or tc1 == 0 or sysid != vehicle.sysid:
            return False
        return clock.reply(tc1, ts1)

    # ------------------------------------------------------------- frames

    def on_frame(self, data_frame):
        """NatNet thread: forward every mapped rigid body in one frame."""
        try:
            self._on_frame(data_frame, time.monotonic())
        except Exception as exc:  # noqa: BLE001  (an exception would end NatNet's thread)
            self.errors += 1
            if self.errors <= 3:
                self.hub.notify(f'mocap frame error: {type(exc).__name__}: {exc}')

    def _on_frame(self, data_frame, now):
        self.frames += 1
        self.last_frame = now
        capture = now - self.latency.latency_s(data_frame, now)
        capture_us = int(round(capture * 1e6))
        for rb in data_frame.rigid_bodies:
            stats = self.stats.get(rb.id_num)
            with self._lock:
                if self.truth is not None:
                    self._log_truth(data_frame, rb, stats, now, capture, capture_us)
                if stats is not None:
                    self._forward(rb, stats, now, capture_us)

    def _log_truth(self, data_frame, rb, stats, now, capture, capture_us):
        prefix = getattr(data_frame, 'prefix', None)
        suffix = getattr(data_frame, 'suffix', None)
        x, y, z = rb.pos
        qx, qy, qz, qw = rb.rot
        try:
            ned = self.converter.position(rb.pos)
            q = self.converter.quaternion(rb.rot)
        except ValueError:
            ned, q = (math.nan,) * 3, [math.nan] * 4
        fc_us = None
        if stats is not None:
            fc_us = self.clocks[stats.sysid].to_fc_us(capture_us)
        self.truth.write((f'{now:.6f}', f'{capture:.6f}', getattr(prefix, 'frame_number', ''),
                          getattr(suffix, 'timestamp', ''), f'{(now - capture) * 1e3:.3f}', rb.id_num,
                          stats.sysid if stats is not None else '', int(rb.tracking_valid is not False),
                          getattr(rb, 'marker_error', ''), x, y, z, qx, qy, qz, qw, *ned, *q,
                          '' if fc_us is None else fc_us), now)

    def _forward(self, rb, stats, now, capture_us):
        stats.frames += 1
        # tracking_valid is None on NatNet streams older than 2.6.
        if rb.tracking_valid is False:
            stats.untracked += 1
            return
        stats.last_valid = now
        stats.last_error = getattr(rb, 'marker_error', None)
        # Grid-based thinning: 120 Hz from Motive at 30 Hz is every 4th frame, without
        # float jitter turning that into every 5th.
        if stats.next_due is not None and now < stats.next_due - RATE_SLACK_S:
            stats.thinned += 1
            return
        vehicle = self.hub.vehicle_for_sysid(stats.sysid)
        if vehicle is None:
            stats.no_vehicle += 1
            return
        if vehicle.pending_status()['chunks'] > self.max_queue:
            stats.backlog += 1
            return
        if self.time_mode == 'fc':
            stamp_us = self.clocks[stats.sysid].to_fc_us(capture_us)
            if stamp_us is None:
                stats.unsynced += 1
                return
        else:
            stamp_us = capture_us
        north, east, down = self.converter.position(rb.pos)
        q = self.converter.quaternion(rb.rot)
        # covariance[0] = NaN: "unknown", so ArduPilot uses VISO_POS_M_NSE / VISO_YAW_M_NSE.
        covariance = [math.nan] + [0.0] * 20
        message = mav.MAVLink_att_pos_mocap_message(stamp_us, q, north, east, down, covariance)
        frame = message.pack(self._encoders[stats.sysid])
        self.hub.send_to_vehicle(vehicle, frame)
        stats.sent += 1
        stats.last_sent = now
        if self.min_interval:
            due = stats.next_due
            stats.next_due = (due + self.min_interval if due is not None and now - due < self.min_interval
                              else now + self.min_interval)
        stats.last_ned = (north, east, down)

    # ------------------------------------------------------------- health

    def check(self, now=None):
        """Report bodies that lost or regained tracking. Called by the hub monitor."""
        now = time.monotonic() if now is None else now
        with self._lock:
            for s in self.stats.values():
                valid = s.last_valid is not None and now - s.last_valid <= TRACKING_LOST_S
                if s.last_valid is None:
                    continue
                if not valid and not s.lost:
                    s.lost = True
                    self.hub.notify(f'mocap: rigid body {s.body} (sys{s.sysid}) not tracked for '
                                    f'{now - s.last_valid:.1f} s -- no poses sent')
                elif valid and s.lost:
                    s.lost = False
                    self.hub.notify(f'mocap: rigid body {s.body} (sys{s.sysid}) tracked again')

    def summary(self):
        with self._lock:
            return dict(frames=self.frames, errors=self.errors, time_mode=self.time_mode,
                        latency_ms=None if self.latency.last_s is None else self.latency.last_s * 1e3,
                        tick_hz=self.latency.tick_hz,
                        truth_rows=self.truth.rows if self.truth is not None else None,
                        bodies={s.body: dict(sysid=s.sysid, **s.counters()) for s in self.stats.values()},
                        timesync={sysid: c.status() for sysid, c in self.clocks.items()})

    def status_lines(self, now=None):
        now = time.monotonic() if now is None else now
        age = 'never' if self.last_frame is None else f'{now - self.last_frame:.1f} s ago'
        lines = [f'mocap {self.server} ({self.converter.frame}, <= {1 / self.min_interval:.0f} Hz per body)'
                 if self.min_interval else f'mocap {self.server} ({self.converter.frame}, every frame)']
        lines[0] += f': last frame {age}, {self.frames} frames'
        latency = self.latency
        if latency.last_s is not None:
            source = ('fixed' if latency.fixed_ms is not None
                      else 'measured' if latency.tick_hz else 'assumed')
            lines[0] += f', latency {latency.last_s * 1e3:.1f} ms ({source}), time {self.time_mode}'
        with self._lock:
            for s in self.stats.values():
                sent, then = s.previous
                rate = ''
                if then is not None and now > then:
                    rate = f' {(s.sent - sent) / (now - then):.0f} Hz'
                s.previous = (s.sent, now)
                if s.last_valid is None:
                    state = 'never tracked'
                else:
                    state = 'NOT TRACKED' if s.lost else 'tracked'
                pose = ''
                if s.last_ned is not None:
                    pose = ' NED ({:+.2f}, {:+.2f}, {:+.2f}) m'.format(*s.last_ned)
                drops = ', '.join(f'{k} {v}' for k, v in s.counters().items()
                                  if k in ('untracked', 'backlog', 'no_vehicle', 'unsynced') and v)
                lines.append(f'    body {s.body} -> sys{s.sysid}: {state}, sent {s.sent}{rate}{pose}'
                             + (f' (skipped: {drops})' if drops else ''))
                sync = self.clocks[s.sysid].status()
                if sync.get('envelope'):
                    lines.append(f'        timesync: rtt min {sync["rtt_min_ms"]:.1f} ms, fit residual '
                                 f'{sync["residual_ms"]:.2f} ms, drift {sync["drift_ppm"]:+.0f} ppm, '
                                 f'{sync["envelope"]} of {sync["replies"]} replies'
                                 + ('' if sync['ready'] else ' (not ready)'))
                else:
                    lines.append(f'        timesync: {sync["replies"]} replies to {sync["requests"]} requests')
        return lines
