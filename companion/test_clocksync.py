#!/usr/bin/env python3
# AP_FLAKE8_CLEAN
"""Offline tests of the TIMESYNC clock mapping and the mocap timing (phase E)."""

import csv
import random
import sys
import tempfile
from types import SimpleNamespace
import unittest

from pymavlink.dialects.v20 import ardupilotmega as mav

import clocksync
import mocap
import test_mocap as tm


def fc_clock(offset_us, drift):
    return lambda hub_us: hub_us + offset_us + drift * hub_us


def exchange(sync, fc, t_ns, out_ms, back_ms):
    """One request at hub time t_ns, answered by an FC with clock ``fc``."""
    ts1 = sync.request(t_ns)
    arrive_ns = t_ns + out_ms * 1e6
    tc1 = int(round(fc(arrive_ns / 1e3) * 1e3))
    return sync.reply(tc1, ts1, int(arrive_ns + back_ms * 1e6))


class FieldTests(unittest.TestCase):
    def test_parse_v2_v1_and_others(self):
        for tc1, ts1 in ((0, 123456789), (987654321000, 5)):
            frame = mav.MAVLink_timesync_message(tc1, ts1).pack(mav.MAVLink(None, srcSystem=3, srcComponent=1))
            self.assertEqual(clocksync.timesync_fields(frame), (3, 1, tc1, ts1))
        from pymavlink.dialects.v10 import ardupilotmega as mav1
        frame = mav1.MAVLink_timesync_message(7, 8).pack(mav1.MAVLink(None, srcSystem=2, srcComponent=1))
        self.assertEqual(clocksync.timesync_fields(frame), (2, 1, 7, 8))
        other = mav.MAVLink_heartbeat_message(2, 3, 0, 0, 0, 3).pack(mav.MAVLink(None))
        self.assertIsNone(clocksync.timesync_fields(other))
        self.assertIsNone(clocksync.timesync_fields(b'\xfd\x01'))


class ClockSyncTests(unittest.TestCase):
    def test_lower_envelope_fit_over_a_polled_radio(self):
        rng = random.Random(3)
        fc = fc_clock(5e6, 40e-6)
        sync = clocksync.ClockSync(1)
        self.assertIsNone(sync.to_fc_us(0))
        t = 1e9
        for i in range(120):  # 60 s at 2 Hz
            # 1 ms minimum each way, then poll and queue waits, longer on the way back
            self.assertTrue(exchange(sync, fc, t, 1.0 + rng.expovariate(1 / 3.0), 1.0 + rng.expovariate(1 / 6.0)))
            if i == 5:
                self.assertFalse(sync.ready)
            t += 0.5e9
        status = sync.status()
        self.assertTrue(status['ready'])
        self.assertAlmostEqual(status['drift_ppm'], 40, delta=5)
        for hub_us in (t / 1e3, t / 1e3 - 10e6):
            self.assertLess(abs(sync.to_fc_us(hub_us) - fc(hub_us)), 1000)

    def test_unmatched_and_stale_replies(self):
        sync = clocksync.ClockSync(1)
        self.assertFalse(sync.reply(10, 42, 100))
        ts1 = sync.request(1000)
        self.assertFalse(sync.reply(0, ts1, 2000))       # a request, not an answer
        self.assertTrue(exchange(sync, fc_clock(0, 0), 3000, 1, 1))
        for i in range(clocksync.MAX_PENDING + 1):
            sync.request(10_000 + i)
        self.assertFalse(sync.reply(5, 10_000, 20_000))  # forgotten


class LatencyTests(unittest.TestCase):
    def frame_at(self, t, latency_s, hz=1e7):
        transmit = int(t * hz) + 12345
        return SimpleNamespace(suffix=SimpleNamespace(stamp_transmit=transmit,
                                                      stamp_camera_mid_exposure=transmit - int(latency_s * hz)))

    def test_measured_after_the_tick_rate_span(self):
        est = mocap.LatencyEstimator()
        self.assertAlmostEqual(est.latency_s(self.frame_at(10.0, 0.004), 10.0), mocap.DEFAULT_LATENCY_MS / 1e3)
        self.assertAlmostEqual(est.latency_s(self.frame_at(13.0, 0.004), 13.0), 0.004, places=5)
        self.assertAlmostEqual(est.tick_hz, 1e7, delta=10)
        self.assertAlmostEqual(est.latency_s(SimpleNamespace(), 14.0), mocap.DEFAULT_LATENCY_MS / 1e3)
        fixed = mocap.LatencyEstimator(fixed_ms=7.5)
        self.assertAlmostEqual(fixed.latency_s(self.frame_at(13.0, 0.004), 13.0), 0.0075)


class HubTimeTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.one, self.two = tm.FakeVehicle(1), tm.FakeVehicle(2)
        self.hub = tm.make_hub(self.tmp.name, [self.one, self.two],
                               tm.MOCAP_ARGV + ['--mocap-time', 'fc', '--mocap-latency-ms', '4'])
        with tm.contextlib.redirect_stdout(tm.io.StringIO()):
            self.hub.start_mocap(client_factory=tm.FakeClient)
        self.forwarder = self.hub.mocap

    def tearDown(self):
        self.hub.stop()
        self.tmp.cleanup()

    def test_requests_go_straight_to_mapped_vehicles_and_replies_are_taken(self):
        self.two.sysid = None  # not heard yet: no request for it
        self.forwarder.tick(100.0)
        self.forwarder.tick(100.1)  # not due yet
        self.assertEqual(len(self.two.sent), 0)
        self.assertEqual(len(self.one.sent), 1)
        request = tm.decode(self.one.sent[0])
        self.assertEqual((request.get_type(), request.tc1), ('TIMESYNC', 0))
        reply = mav.MAVLink_timesync_message(5_000_000_000, request.ts1).pack(
            mav.MAVLink(None, srcSystem=1, srcComponent=1))
        self.hub.downlink(self.one, reply)
        self.assertEqual(self.forwarder.clocks[1].replies, 1)
        # the vehicle's own request (tc1 = 0) and another vehicle's answer are ignored
        own = mav.MAVLink_timesync_message(0, 99).pack(mav.MAVLink(None, srcSystem=1, srcComponent=1))
        self.assertFalse(self.forwarder.on_downlink(self.one, own))

    def test_fc_mode_waits_for_the_fit_then_stamps_the_capture_in_fc_time(self):
        self.forwarder._on_frame(tm.frame(tm.body(66)), 200.0)
        self.assertEqual(self.one.sent, [])
        self.assertEqual(self.forwarder.summary()['bodies'][66]['unsynced'], 1)
        fc = fc_clock(3e6, 0.0)
        sync = self.forwarder.clocks[1]
        for i in range(12):
            exchange(sync, fc, (150.0 + i) * 1e9, 1.0, 1.0)
        self.forwarder._on_frame(tm.frame(tm.body(66)), 200.5)
        message = tm.decode(self.one.sent[-1])
        self.assertAlmostEqual(message.time_usec, fc((200.5 - 0.004) * 1e6), delta=2)

    def test_truth_log_has_every_body_of_every_frame(self):
        self.forwarder._on_frame(tm.frame(tm.body(66, pos=(1, 2, 3)), tm.body(67, valid=False), tm.body(99)),
                                 300.0)
        self.forwarder._on_frame(tm.frame(tm.body(99)), 300.01)
        path = self.forwarder.truth.path
        self.hub.stop()
        self.hub.stop = lambda: None
        with open(path, newline='') as f:
            rows = list(csv.DictReader(f))
        self.assertEqual([r['rigid_body'] for r in rows], ['66', '67', '99', '99'])
        self.assertEqual([r['sysid'] for r in rows], ['1', '2', '', ''])
        self.assertEqual([r['tracked'] for r in rows], ['1', '0', '1', '1'])
        self.assertEqual((rows[0]['north'], rows[0]['east'], rows[0]['down']), ('-3', '1', '-2'))
        self.assertAlmostEqual(float(rows[0]['latency_ms']), 4.0)
        self.assertIn('mocap-truth', path.name)


if __name__ == '__main__':
    sys.exit(unittest.main())
