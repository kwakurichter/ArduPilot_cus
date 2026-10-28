#!/usr/bin/env python3
# AP_FLAKE8_CLEAN
"""Offline OptiTrack forwarding tests. No Motive, radio or aircraft connection is opened."""

import argparse
import contextlib
import io
import math
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import unittest

from pymavlink.dialects.v20 import ardupilotmega as mav

import mocap
import swarm_hub as hub_module

bridge = hub_module.load_bridge()
import guided_swarm  # noqa: E402


def quat_xyzw(axis, angle):
    s = math.sin(angle / 2)
    return (axis[0] * s, axis[1] * s, axis[2] * s, math.cos(angle / 2))


def rotate(q_wxyz, v):
    """Rotate v by the unit quaternion (w, x, y, z)."""
    w, x, y, z = q_wxyz
    m = ((1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)),
         (2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)),
         (2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)))
    return tuple(sum(m[i][j] * v[j] for j in range(3)) for i in range(3))


def body(id_num, pos=(0.0, 0.0, 0.0), rot=(0.0, 0.0, 0.0, 1.0), valid=True):
    return SimpleNamespace(id_num=id_num, pos=pos, rot=rot, tracking_valid=valid, marker_error=0.0004)


def frame(*bodies):
    return SimpleNamespace(rigid_bodies=bodies)


def decode(raw):
    messages = mav.MAVLink(None).parse_buffer(raw)
    assert len(messages) == 1
    return messages[0]


class FakeVehicle:
    def __init__(self, sysid, pending=0):
        self.sysid = sysid
        self.uri = f'fake://{sysid}'
        self.stats = {}
        self.sent = []
        self.pending = pending

    def queue_frame(self, raw):
        self.sent.append(raw)

    def pending_status(self):
        return {'chunks': self.pending, 'oldest_ms': 0}

    def label(self):
        return f'fake{self.sysid}'


class FakeClient:
    instances = []

    def __init__(self, **kwargs):
        self.kwargs = kwargs
        self.on_data_frame_received_event = SimpleNamespace(handlers=[])
        self.running = self.closed = False
        FakeClient.instances.append(self)

    def connect(self, timeout):
        pass

    def run_async(self):
        self.running = True

    def shutdown(self):
        self.closed = True

    def emit(self, data_frame):
        for handler in self.on_data_frame_received_event.handlers:
            handler(data_frame)


def make_hub(tmp, vehicles, argv=()):
    args = hub_module.arguments(['--sim', '1', *argv])
    args.log_dir = Path(tmp)
    hub = hub_module.Hub(args, bridge, guided_swarm)
    for v in vehicles:
        hub.vehicles.append(v)
        hub.state[v] = hub_module.VehicleState()
    return hub


MOCAP_ARGV = ['--mocap-server', '10.0.0.2', '--mocap-local', '10.0.0.1',
              '--mocap-body', '66:1', '--mocap-body', '67:2', '--mocap-rate', '0']


class ConverterTests(unittest.TestCase):
    def test_y_up_axes(self):
        c = mocap.PoseConverter('y-up')
        # Motive -Z is north, +X east, +Y up.
        self.assertEqual(c.position((0, 0, -1)), (1, 0, 0))
        self.assertEqual(c.position((1, 0, 0)), (0, 1, 0))
        self.assertEqual(c.position((0, 1, 0)), (0, 0, -1))

    def test_z_up_axes(self):
        c = mocap.PoseConverter('z-up')
        self.assertEqual(c.position((1, 0, 0)), (1, 0, 0))
        self.assertEqual(c.position((0, 1, 0)), (0, -1, 0))
        self.assertEqual(c.position((0, 0, 1)), (0, 0, -1))

    def test_attitude_uses_the_same_rotation_as_position(self):
        # A body yawed 90 degrees about "up", in both conventions: its nose (FRD x)
        # must point where the position map sends the Motive nose direction.
        for frame_name, up, nose in (('y-up', (0, 1, 0), (0, 0, -1)), ('z-up', (0, 0, 1), (1, 0, 0))):
            c = mocap.PoseConverter(frame_name)
            q_motive = quat_xyzw(up, math.pi / 2)
            nose_motive = rotate((q_motive[3], *q_motive[:3]), nose)
            q = c.quaternion(q_motive)
            for got, want in zip(rotate(q, (1, 0, 0)), c.position(nose_motive)):
                self.assertAlmostEqual(got, want)
            # Counter-clockwise seen from above is a negative NED yaw.
            yaw = math.atan2(2 * (q[0] * q[3] + q[1] * q[2]), 1 - 2 * (q[2] ** 2 + q[3] ** 2))
            self.assertAlmostEqual(yaw, -math.pi / 2)

    def test_quaternion_is_normalised_and_rejects_bad_frames(self):
        q = mocap.PoseConverter().quaternion((0, 0, 0, -2))
        self.assertEqual(q, [1, 0, 0, 0])
        with self.assertRaises(ValueError):
            mocap.PoseConverter('x-up')
        with self.assertRaises(ValueError):
            mocap.PoseConverter().quaternion((0, 0, 0, 0))


class MappingTests(unittest.TestCase):
    def test_parse_and_map(self):
        self.assertEqual(mocap.body_map([mocap.parse_body('66:1'), mocap.parse_body('7:12')]), {66: 1, 7: 12})
        for bad in ('66', '66:0', 'a:1', '1:256'):
            with self.assertRaises(ValueError):
                mocap.parse_body(bad)
        with self.assertRaises(ValueError):
            mocap.body_map([(66, 1), (67, 1)])
        with self.assertRaises(ValueError):
            mocap.body_map([(66, 1), (66, 2)])

    def test_hub_arguments(self):
        args = hub_module.arguments(['--sim', '1', *MOCAP_ARGV])
        self.assertEqual(args.mocap_body, {66: 1, 67: 2})
        for argv in (['--mocap-body', '66:1'], [*MOCAP_ARGV, '--mocap-body', '68:2']):
            with self.assertRaises(SystemExit), contextlib.redirect_stderr(io.StringIO()):
                hub_module.arguments(['--sim', '1', *argv])
        self.assertIsNone(hub_module.arguments(['--sim', '1']).mocap_body)


class ForwardingTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.one, self.two = FakeVehicle(1), FakeVehicle(2)
        self.hub = make_hub(self.tmp.name, [self.one, self.two], MOCAP_ARGV)
        FakeClient.instances.clear()
        with contextlib.redirect_stdout(io.StringIO()):
            self.hub.start_mocap(client_factory=FakeClient)
        self.client = FakeClient.instances[-1]

    def tearDown(self):
        self.hub.stop()
        self.tmp.cleanup()

    def test_each_body_reaches_only_its_vehicle(self):
        self.assertTrue(self.client.running)
        self.assertEqual(self.client.kwargs['server_ip_address'], '10.0.0.2')
        self.client.emit(frame(body(66, pos=(1.0, 0.5, -2.0)), body(67, pos=(-1.0, 0.25, 3.0)), body(99)))
        self.assertEqual((len(self.one.sent), len(self.two.sent)), (1, 1))
        m1, m2 = decode(self.one.sent[0]), decode(self.two.sent[0])
        self.assertEqual(m1.get_type(), 'ATT_POS_MOCAP')
        self.assertEqual((m1.get_srcSystem(), m1.get_srcComponent()), (1, 197))
        self.assertEqual((m2.get_srcSystem(), m2.get_srcComponent()), (2, 197))
        self.assertEqual((m1.x, m1.y, m1.z), (2.0, 1.0, -0.5))
        self.assertEqual((m2.x, m2.y, m2.z), (-3.0, -1.0, -0.25))
        self.assertTrue(math.isnan(m1.covariance[0]))
        # Poses are in the session tlog, not sent to QGC.
        self.assertEqual(self.hub.qgc.frames_out, 0)

    def test_untracked_unheard_and_busy_vehicles_are_skipped(self):
        self.client.emit(frame(body(66, valid=False), body(67)))
        self.assertEqual((len(self.one.sent), len(self.two.sent)), (0, 1))
        self.two.pending = 3
        self.client.emit(frame(body(67)))
        self.assertEqual(len(self.two.sent), 1)
        self.two.sysid = None  # link not identified (yet)
        self.two.pending = 0
        self.client.emit(frame(body(67)))
        self.assertEqual(len(self.two.sent), 1)
        counts = self.hub.mocap.summary()['bodies']
        self.assertEqual(counts[66]['untracked'], 1)
        self.assertEqual((counts[67]['backlog'], counts[67]['no_vehicle'], counts[67]['sent']), (1, 1, 1))

    def test_two_links_with_one_sysid_get_nothing(self):
        self.hub.vehicles.append(FakeVehicle(1))
        self.client.emit(frame(body(66)))
        self.assertEqual(self.one.sent, [])

    def test_rate_limit_per_body(self):
        forwarder = self.hub.mocap
        forwarder.min_interval = 0.1
        for t in (10.0, 10.05, 10.1, 10.15, 10.21):
            forwarder._on_frame(frame(body(66), body(67)), t)
        self.assertEqual((len(self.one.sent), len(self.two.sent)), (3, 3))
        self.assertEqual(forwarder.summary()['bodies'][66]['thinned'], 2)

    def test_tracking_loss_is_reported_once(self):
        notes = []
        self.hub.notify = notes.append
        forwarder = self.hub.mocap
        forwarder._on_frame(frame(body(66)), 10.0)
        forwarder.check(10.2)
        forwarder.check(11.0)
        forwarder.check(11.5)
        forwarder._on_frame(frame(body(66)), 12.0)
        forwarder.check(12.0)
        self.assertEqual(len(notes), 2)
        self.assertIn('not tracked', notes[0])
        self.assertIn('tracked again', notes[1])

    def test_a_bad_frame_does_not_escape_into_natnet(self):
        notes = []
        self.hub.notify = notes.append
        self.client.emit(frame(body(66, rot=(0, 0, 0, 0))))
        self.client.emit(SimpleNamespace())
        self.assertEqual(self.hub.mocap.errors, 2)
        self.assertEqual(len(notes), 2)

    def test_status_and_shutdown(self):
        self.client.emit(frame(body(66, pos=(0, 1, 0))))
        lines = '\n'.join(self.hub.status_lines())
        self.assertIn('body 66 -> sys1: tracked, sent 1 NED (+0.00, +0.00, -1.00) m', lines)
        self.assertIn('body 67 -> sys2: never tracked', lines)
        self.hub.stop()
        self.assertTrue(self.client.closed)
        self.assertIn('mocap_stats', self.hub.session.events_path.read_text())
        self.hub.stop = lambda: None  # already stopped; tearDown must not close twice


class NoMocapTests(unittest.TestCase):
    def test_hub_without_mocap_is_unchanged(self):
        with tempfile.TemporaryDirectory() as tmp:
            v = FakeVehicle(1)
            hub = make_hub(tmp, [v])
            hub.args = argparse.Namespace(**{**vars(hub.args), 'mocap_body': None})
            self.assertIsNone(hub.mocap)
            self.assertFalse(any(line.startswith('mocap') for line in hub.status_lines()))
            hub.stop()


if __name__ == '__main__':
    sys.exit(unittest.main())
