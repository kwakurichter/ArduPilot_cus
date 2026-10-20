#!/usr/bin/env python3
# AP_FLAKE8_CLEAN
"""Offline swarm_hub tests. No radio, QGC or aircraft connection is opened."""

import argparse
import contextlib
import io
from pathlib import Path
import socket
import struct
import sys
import tempfile
import time
import unittest
from unittest.mock import patch

from pymavlink.dialects.v20 import ardupilotmega as mav

import swarm_hub as hub_module

bridge = hub_module.load_bridge()
import guided_swarm  # noqa: E402


def pack(message, sid=1, comp=1):
    return message.pack(mav.MAVLink(None, srcSystem=sid, srcComponent=comp))


def heartbeat(sid, armed=False, mode=4, autopilot=3, comp=1):
    base = 0x80 if armed else 0
    return pack(mav.MAVLink_heartbeat_message(2, autopilot, base | 1, mode, 3, 3), sid, comp)


def ack(sid, command, result=0, target_component=191):
    return pack(mav.MAVLink_command_ack_message(command, result, 0, 0, 255, target_component), sid)


class FakeVehicle:
    def __init__(self, sysid):
        self.sysid = sysid
        self.uri = f'fake://{sysid}'
        self.stats = {}
        self.sent = []

    def queue_frame(self, frame):
        self.sent.append(frame)

    def pending_status(self):
        return {'chunks': 0, 'oldest_ms': 0}

    def label(self):
        return f'fake{self.sysid}'


def make_hub(tmp, vehicles=()):
    args = argparse.Namespace(log_dir=Path(tmp), qgc=('127.0.0.1', 9), uri=None, sim=None,
                              idle_poll_ms=2, stats_sec=60)
    hub = hub_module.Hub(args, bridge, guided_swarm)
    for v in vehicles:
        hub.vehicles.append(v)
        hub.state[v] = hub_module.VehicleState()
    return hub


class HeartbeatFieldTests(unittest.TestCase):
    def test_v2_armed_mode_and_v1(self):
        self.assertEqual(hub_module.heartbeat_fields(heartbeat(3, armed=True, mode=4)), (3, 0x81, 4))
        from pymavlink.dialects.v10 import ardupilotmega as mav1
        frame = mav1.MAVLink_heartbeat_message(2, 3, 1, 9, 3, 3).pack(mav1.MAVLink(None, srcSystem=2, srcComponent=1))
        self.assertEqual(frame[0], 0xFE)
        self.assertEqual(hub_module.heartbeat_fields(frame), (2, 1, 9))

    def test_trimmed_payload_and_non_autopilot(self):
        # Mode 0, disarmed: MAVLink 2 trims nothing here, but a zero tail must not break parsing.
        frame = heartbeat(1, armed=False, mode=0)
        self.assertEqual(hub_module.heartbeat_fields(frame)[2], 0)
        self.assertIsNone(hub_module.heartbeat_fields(heartbeat(255, autopilot=8, comp=190)))
        self.assertIsNone(hub_module.heartbeat_fields(heartbeat(1, comp=154)))
        self.assertIsNone(hub_module.heartbeat_fields(ack(1, 400)))
        self.assertIsNone(hub_module.heartbeat_fields(b'\xfd\x09'))


class LogTests(unittest.TestCase):
    def test_names_never_collide(self):
        with tempfile.TemporaryDirectory() as tmp:
            book = hub_module.LogBook(tmp)
            with patch.object(hub_module.time, 'strftime', side_effect=lambda f: '2026-10-20' if '%Y' in f else '151300'):
                first = book.path('hover-sys1', '.jsonl')
                first.write_text('x')
                second = book.path('hover-sys1', '.jsonl')
                second.write_text('x')
                third = book.path('hover-sys1', '.jsonl')
            self.assertEqual([p.name for p in (first, second, third)],
                             ['151300-hover-sys1.jsonl', '151300-hover-sys1-2.jsonl', '151300-hover-sys1-3.jsonl'])
            self.assertEqual(first.parent.name, '2026-10-20')

    def test_session_tlog_records_timestamped_frames(self):
        with tempfile.TemporaryDirectory() as tmp:
            hub = make_hub(tmp)
            frame = heartbeat(1)
            hub.session.frame(frame)
            hub.stop()
            self.assertIn(b'session_end', hub.session.events_path.read_bytes())
            data = hub.session.tlog_path.read_bytes()
            stamp = struct.unpack('>Q', data[:8])[0]
            self.assertLess(abs(stamp / 1e6 - time.time()), 5)
            self.assertEqual(data[8:], frame)
            # pymavlink reads it back as a normal telemetry log.
            from pymavlink import mavutil
            log = mavutil.mavlink_connection(str(hub.session.tlog_path))
            self.assertEqual(log.recv_match(type='HEARTBEAT').get_srcSystem(), 1)
            log.close()


class RoutingTests(unittest.TestCase):
    def test_downlink_reaches_qgc_session_and_mission(self):
        with tempfile.TemporaryDirectory() as tmp:
            v = FakeVehicle(2)
            hub = make_hub(tmp, [v])
            listener = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            listener.bind(('127.0.0.1', 0))
            listener.settimeout(1)
            hub.qgc.peer = listener.getsockname()
            relay = hub_module.MissionRelay(hub, argparse.Namespace(source_system=255, target_systems=[2]),
                                            lambda *a, **k: None)
            hub.attach(relay)
            hub.downlink(v, heartbeat(2, armed=True))
            self.assertEqual(listener.recv(1000), heartbeat(2, armed=True))
            self.assertTrue(hub.state[v].armed)
            self.assertEqual(hub.heard(), [2])
            relay.pump(timeout=0)
            self.assertIn('HEARTBEAT', relay.links[2].cache)
            hub.detach(relay)
            hub.downlink(v, heartbeat(2))
            self.assertTrue(relay.inbox.empty())
            listener.close()
            hub.stop()

    def test_mission_commands_route_to_their_vehicle_and_acks_are_kept(self):
        with tempfile.TemporaryDirectory() as tmp:
            one, two = FakeVehicle(1), FakeVehicle(2)
            hub = make_hub(tmp, [one, two])
            for v in (one, two):
                hub.router.learn(v.sysid, v)
            log = []
            relay = hub_module.MissionRelay(hub, argparse.Namespace(source_system=255, target_systems=[1, 2]),
                                            lambda event, **k: log.append(event))
            hub.attach(relay)
            relay.mav.command_long_send(2, 1, mav.MAV_CMD_NAV_LAND, 0, 0, 0, 0, 0, 0, 0, 0)
            self.assertEqual((len(one.sent), len(two.sent)), (0, 1))
            hub.downlink(two, ack(2, mav.MAV_CMD_NAV_LAND))
            hub.downlink(two, ack(2, mav.MAV_CMD_NAV_TAKEOFF, target_component=190))  # QGC's, not ours
            relay.pump(timeout=0)
            self.assertEqual([m.command for _, m in relay.links[2].acks], [mav.MAV_CMD_NAV_LAND])
            self.assertEqual(relay.links[1].acks, [])
            hub.stop()

    def test_qgc_command_is_forwarded_and_flags_takeover(self):
        with tempfile.TemporaryDirectory() as tmp:
            v = FakeVehicle(1)
            hub = make_hub(tmp, [v])
            hub.router.learn(1, v)
            relay = hub_module.MissionRelay(hub, argparse.Namespace(source_system=255, target_systems=[1]),
                                            lambda *a, **k: None)
            relay.links[1].watch_takeover = True
            hub.attach(relay)
            frame = pack(mav.MAVLink_command_long_message(1, 1, mav.MAV_CMD_NAV_RETURN_TO_LAUNCH,
                                                          0, 0, 0, 0, 0, 0, 0, 0), 255, 190)
            hub.uplink(frame, from_qgc=True)
            self.assertEqual(v.sent, [frame])
            relay.pump(timeout=0)
            self.assertTrue(relay.links[1].takeover)
            hub.stop()

    def test_mission_heartbeat_only_while_enabled(self):
        with tempfile.TemporaryDirectory() as tmp:
            v = FakeVehicle(1)
            hub = make_hub(tmp, [v])
            relay = hub_module.MissionRelay(hub, argparse.Namespace(source_system=255, target_systems=[1]),
                                            lambda *a, **k: None)
            relay.pump(timeout=0)
            self.assertEqual(len(v.sent), 1)
            relay.heartbeat_enabled = False
            relay.last_heartbeat = 0
            relay.pump(timeout=0)
            self.assertEqual(len(v.sent), 1)
            hub.stop()


class ConsoleTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.vehicles = [FakeVehicle(1), FakeVehicle(2)]
        self.hub = make_hub(self.tmp.name, self.vehicles)
        self.console = hub_module.Console(self.hub)

    def tearDown(self):
        self.hub.stop()
        self.tmp.cleanup()

    def hear(self, *sids):
        for v in self.vehicles:
            if v.sysid in sids:
                self.hub.state[v].last_rx = time.monotonic()

    def test_hub_managed_options_are_dropped(self):
        argv, explicit = self.console.mission_argv(
            ['--target-systems', '2', '--pattern', 'hover', '--execute', '--listen', '127.0.0.1:1',
             '--qgc=127.0.0.1:2', '--log', 'mine.jsonl'])
        self.assertEqual(argv, ['--target-systems', '2', '--pattern', 'hover'])
        self.assertEqual(explicit, Path('mine.jsonl'))

    def test_target_systems_default_to_heard_vehicles(self):
        self.hear(1, 2)
        self.assertEqual(self.console.mission_argv(['--pattern', 'hover'])[0],
                         ['--target-systems', '1', '2', '--pattern', 'hover'])
        self.hub.state[self.vehicles[0]].last_rx = time.monotonic() - 10
        self.assertEqual(self.console.mission_argv([])[0], ['--target-systems', '2'])

    def test_preview_never_flies_and_flags_missing_vehicles(self):
        self.hear(1)
        out = io.StringIO()
        with contextlib.redirect_stdout(out), patch.object(hub_module, 'run_mission') as run:
            args = self.console.preview(['--target-systems', '1', '2', '--pattern', 'hover'])
        self.assertFalse(args.execute)
        run.assert_not_called()
        text = out.getvalue()
        self.assertIn('Aircraft [1, 2]', text)
        self.assertIn('not hearing system(s) [2]', text)
        self.assertNotIn('Add --execute', text)

    def test_invalid_options_do_not_fly(self):
        out = io.StringIO()
        with contextlib.redirect_stdout(out), patch.object(hub_module, 'run_mission') as run:
            self.console.fly(['--pattern', 'diamond', '--radius', '5'])
        run.assert_not_called()
        self.assertIn('Invalid mission options: For diamond, use radius', out.getvalue())
        self.assertNotIn('usage:', out.getvalue())

    def test_fly_needs_confirmation_and_again_repeats(self):
        self.hear(1)
        with contextlib.redirect_stdout(io.StringIO()), patch.object(hub_module, 'run_mission') as run, \
                patch('builtins.input', side_effect=['n', 'y']):
            self.console.fly(['--pattern', 'hover'])
            run.assert_not_called()
            self.console.fly(self.console.last_argv)
        run.assert_called_once()
        args, argv = run.call_args.args[1:3]
        self.assertEqual(args.target_systems, [1])
        self.assertEqual(argv, ['--target-systems', '1', '--pattern', 'hover'])

    def test_status_shows_slow_polled_address(self):
        self.vehicles[1].silent = True
        group = argparse.Namespace(index=1, usb_id='bus1.addr2', failures=0, reopens=0, last_failure=None,
                                   vehicles=self.vehicles, is_alive=lambda: True, stop=lambda: None,
                                   join=lambda timeout: None)
        self.hub.groups.append(group)
        lines = '\n'.join(self.hub.status_lines())
        self.assertIn('sys2 fake2: never heard, radio not answering (polled 4x/s)', lines)
        self.assertNotIn('sys1 fake1: never heard, radio', lines)

    def test_quit_asks_when_a_vehicle_is_armed(self):
        self.hear(1)
        self.hub.state[self.vehicles[0]].armed = True
        with patch('builtins.input', return_value='n'), contextlib.redirect_stdout(io.StringIO()):
            self.assertFalse(self.console.quit_ok())
        self.hub.state[self.vehicles[0]].armed = False
        self.assertTrue(self.console.quit_ok())


class MissionRunTests(unittest.TestCase):
    def run_with(self, error):
        with tempfile.TemporaryDirectory() as tmp:
            hub = make_hub(tmp, [FakeVehicle(1)])
            args = guided_swarm.arguments(['--target-systems', '1', '--pattern', 'hover'])
            aborted = []

            class Flight:
                def __init__(self, relay, args, log):
                    self.relay = relay

                def run(self):
                    assert hub.mission is self.relay
                    if error:
                        raise error

                def abort(self, reason):
                    aborted.append(reason)

            with patch.object(guided_swarm, 'SwarmFlight', Flight), contextlib.redirect_stdout(io.StringIO()):
                ok = hub_module.run_mission(hub, args, ['--target-systems', '1'])
                ok_again = hub_module.run_mission(hub, args, ['--target-systems', '1'])
            logs = sorted(p.name for p in Path(tmp).rglob('*hover*.jsonl'))
            self.assertIsNone(hub.mission)
            hub.stop()
            events = hub.session.events_path.read_text()
            return ok, ok_again, aborted, logs, events

    def test_back_to_back_missions_get_their_own_logs(self):
        ok, ok_again, aborted, logs, events = self.run_with(None)
        self.assertTrue(ok and ok_again)
        self.assertEqual(aborted, [])
        self.assertEqual(len(logs), 2)
        self.assertEqual(events.count('"mission_end"'), 2)

    def test_ctrl_c_aborts_with_a_readable_reason(self):
        ok, _, aborted, _, events = self.run_with(KeyboardInterrupt())
        self.assertFalse(ok)
        self.assertEqual(str(aborted[0]), 'stopped by operator (Ctrl-C)')
        self.assertIn('stopped by operator', events)

    def test_unexpected_exception_names_its_type(self):
        _, _, aborted, _, _ = self.run_with(KeyError('cache'))
        self.assertIsInstance(aborted[0], guided_swarm.FlightError)
        self.assertIn('KeyError', str(aborted[0]))


if __name__ == '__main__':
    sys.exit(unittest.main())
