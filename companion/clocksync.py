#!/usr/bin/env python3
# AP_FLAKE8_CLEAN
"""Hub-to-FC clock mapping from MAVLink TIMESYNC (docs/plans/p5_global_fix_plan.md 3.5).

The hub sends TIMESYNC (tc1 = 0, ts1 = hub monotonic ns) straight to one vehicle; stock
ArduPilot answers with tc1 = the UART receive time of the request in its micros64() clock
(ns) and ts1 mirrored. Each answer gives one sample

    offset = tc1 - (t_send + t_recv) / 2,    rtt = t_recv - t_send

The polled Crazyradio adds queue and poll waits, unevenly in the two directions, so only the
lower envelope of the last WINDOW_S is used: the ENVELOPE_FRACTION of samples with the
shortest round trips, and every sample within ENVELOPE_US of the shortest. A line
offset(t) = a + b (t - t0) is fitted through them (crystal drift is tens of ppm). The
error is about half the round-trip asymmetry of those samples. The fit is used once the
window holds READY_SAMPLES answers.
"""

import collections
import struct
import threading
import time

TIMESYNC_ID = 111
WINDOW_S = 30.0
ENVELOPE_US = 1000.0
ENVELOPE_FRACTION = 0.25
READY_SAMPLES = 10
MIN_DRIFT_SPAN_S = 5.0
MAX_PENDING = 32


def timesync_fields(frame):
    """(sysid, compid, tc1, ts1) of a TIMESYNC frame, else None. Header parsing only."""
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
    if msgid != TIMESYNC_ID:
        return None
    # MAVLink 2 trims trailing zero bytes; tc1 and ts1 are the first 16 bytes.
    payload = bytes(payload) + bytes(max(0, 16 - len(payload)))
    tc1, ts1 = struct.unpack_from('<qq', payload)
    return sysid, compid, tc1, ts1


class ClockSync:
    """Maps hub monotonic microseconds to one FC's micros64() (thread-safe)."""

    def __init__(self, sysid):
        self.sysid = sysid
        self.requests = 0
        self.replies = 0
        self._pending = collections.OrderedDict()  # ts1 -> send time (hub ns)
        self._samples = collections.deque()        # (t_mid_us, offset_us, rtt_us)
        self._fit = None                           # (t0_us, a_us, b, residual_us, n, rtt_min_us)
        self._lock = threading.Lock()

    # ------------------------------------------------------------------ exchange

    def request(self, now_ns=None):
        """ts1 for a new request (the hub's monotonic ns); remembered until answered."""
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

    def to_fc_us(self, hub_us):
        """FC micros64() at hub monotonic ``hub_us``, or None until READY_SAMPLES."""
        with self._lock:
            if self._fit is None or self._fit[6] < READY_SAMPLES:
                return None
            t0, a, b = self._fit[:3]
        return int(round(hub_us + a + b * (hub_us - t0)))

    def status(self):
        with self._lock:
            if self._fit is None:
                return dict(sysid=self.sysid, requests=self.requests, replies=self.replies, ready=False)
            t0, a, b, residual, n, rtt_min, window = self._fit
            return dict(sysid=self.sysid, requests=self.requests, replies=self.replies,
                        ready=window >= READY_SAMPLES, envelope=n, rtt_min_ms=rtt_min / 1e3,
                        residual_ms=residual / 1e3, drift_ppm=b * 1e6, offset_s=a / 1e6)
