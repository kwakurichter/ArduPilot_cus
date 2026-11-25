#!/usr/bin/env python3
# AP_FLAKE8_CLEAN
"""Host-to-vehicle clock mapping from MAVLink TIMESYNC.

The orchestrator sends TIMESYNC (tc1 = 0, ts1 = host monotonic ns) to one vehicle;
stock ArduPilot (GCS_MAVLINK::handle_timesync) answers with tc1 = the receive time of
the request in its own AP_HAL::micros64() clock, in ns, and ts1 mirrored. That is the
clock behind Lua's millis(). Each answer gives one sample

    offset = tc1 - (t_send + t_recv) / 2,    rtt = t_recv - t_send

Only the lower envelope of the recent samples is used (the shortest round trips, whose
asymmetry is smallest), and a line offset(t) = a + b (t - t0) is fitted through it. The
slope b absorbs the rate difference between the host clock and the simulated clock,
which is not 1 when Gazebo runs below real time. The method follows the swarm hub's
clock sync on the optitrack branch (companion/clocksync.py).
"""

import collections
import threading
import time

WINDOW_S = 30.0
ENVELOPE_US = 1000.0
ENVELOPE_FRACTION = 0.25
READY_SAMPLES = 10
MIN_DRIFT_SPAN_S = 5.0
MAX_PENDING = 32


class ClockSync:
    """Maps host monotonic microseconds to one vehicle's micros64() (thread-safe)."""

    def __init__(self, sysid):
        self.sysid = sysid
        self.requests = 0
        self.replies = 0
        self._pending = collections.OrderedDict()  # ts1 -> send time (host ns)
        self._samples = collections.deque()        # (t_mid_us, offset_us, rtt_us)
        self._fit = None                           # (t0_us, a_us, b, residual_us, n, rtt_min_us, window)
        self._lock = threading.Lock()

    # ------------------------------------------------------------------ exchange

    def request(self, now_ns=None):
        """ts1 for a new request (host monotonic ns); remembered until answered."""
        ts1 = time.monotonic_ns() if now_ns is None else int(now_ns)
        with self._lock:
            self._pending[ts1] = ts1
            while len(self._pending) > MAX_PENDING:
                self._pending.popitem(last=False)
            self.requests += 1
        return ts1

    def reply(self, tc1_ns, ts1, recv_ns=None):
        """Take an answer; False if it does not match a request of ours."""
        recv_ns = time.monotonic_ns() if recv_ns is None else int(recv_ns)
        with self._lock:
            sent = self._pending.pop(ts1, None)
            if sent is None or tc1_ns <= 0 or recv_ns < sent:
                return False
            self.replies += 1
            t_mid_us = (sent + recv_ns) / 2e3
            self._samples.append((t_mid_us, tc1_ns / 1e3 - t_mid_us, (recv_ns - sent) / 1e3))
            while self._samples and self._samples[0][0] < t_mid_us - WINDOW_S * 1e6:
                self._samples.popleft()
            self._refit()
        return True

    def _refit(self):
        by_rtt = sorted(self._samples, key=lambda s: s[2])
        rtt_min = by_rtt[0][2]
        keep = max(3, int(len(by_rtt) * ENVELOPE_FRACTION + 0.5))
        env = sorted((s for i, s in enumerate(by_rtt) if i < keep or s[2] <= rtt_min + ENVELOPE_US),
                     key=lambda s: s[0])
        t0 = env[-1][0]
        span = env[-1][0] - env[0][0]
        if len(env) >= 3 and span >= MIN_DRIFT_SPAN_S * 1e6:
            n = len(env)
            mt = sum(s[0] - t0 for s in env) / n
            mo = sum(s[1] for s in env) / n
            stt = sum((s[0] - t0 - mt) ** 2 for s in env)
            b = sum((s[0] - t0 - mt) * (s[1] - mo) for s in env) / stt
            a = mo - b * mt
        else:
            b = 0.0
            a = sum(s[1] for s in env) / len(env)
        residual = (sum((s[1] - a - b * (s[0] - t0)) ** 2 for s in env) / len(env)) ** 0.5
        self._fit = (t0, a, b, residual, len(env), rtt_min, len(self._samples))

    # ------------------------------------------------------------------ use

    @property
    def ready(self):
        with self._lock:
            return self._fit is not None and self._fit[6] >= READY_SAMPLES

    def to_fc_us(self, host_us):
        """Vehicle micros64() at host monotonic ``host_us``, or None until ready."""
        with self._lock:
            if self._fit is None or self._fit[6] < READY_SAMPLES:
                return None
            t0, a, b = self._fit[:3]
        return int(round(host_us + a + b * (host_us - t0)))

    def to_host_us(self, fc_us):
        """Host monotonic microseconds at vehicle micros64() ``fc_us``, or None until ready."""
        with self._lock:
            if self._fit is None or self._fit[6] < READY_SAMPLES:
                return None
            t0, a, b = self._fit[:3]
        # fc = h + a + b (h - t0)  =>  h = (fc - a + b t0) / (1 + b)
        return int(round((fc_us - a + b * t0) / (1.0 + b)))

    def status(self):
        with self._lock:
            if self._fit is None:
                return dict(sysid=self.sysid, requests=self.requests, replies=self.replies, ready=False)
            t0, a, b, residual, n, rtt_min, window = self._fit
            return dict(sysid=self.sysid, requests=self.requests, replies=self.replies,
                        ready=window >= READY_SAMPLES, envelope=n, rtt_min_ms=rtt_min / 1e3,
                        residual_ms=residual / 1e3, drift_ppm=b * 1e6, offset_s=a / 1e6)
