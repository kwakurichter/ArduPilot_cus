#!/usr/bin/env python3
# AP_FLAKE8_CLEAN
"""Two-vehicle protocol and deterministic mission tests; no aircraft connection."""

import argparse
import contextlib
import io
import math
import socket
import unittest
from unittest.mock import Mock, patch

import guided_pair as gp


class SimRelay:
    """Minimal autopilot responses and kinematics, not a flight-dynamics model."""

    def __init__(self, args):
        self.now = 100.0
        self.source_system = args.source_system
        self.heartbeat_enabled = True
        self.guard_callback = None
        self.mav = Mock()
        self.mav.command_long_send.side_effect = self.command
        self.mav.command_int_send.side_effect = self.command_int
        self.mav.set_position_target_local_ned_send.side_effect = self.setpoint
        self.links = {sid: gp.VehicleLink(self, sid) for sid in args.target_systems}
        self.state = {sid: dict(mode=0, armed=False, pos=[3.0 * i, 4.0 * i, 0.0], target=None,
                                yaw=i * math.pi / 2, home=0) for i, sid in enumerate(args.target_systems)}
        self.events = []
        self.drop_acks = set()
        self.reject_arm = None
        self.stale = None
        self.takeover = None
        self.speed = {sid: 1.0 for sid in self.links}
        self.publish()

    def ack(self, sid, cmd, result=gp.mav.MAV_RESULT_ACCEPTED):
        if (sid, cmd) not in self.drop_acks:
            self.links[sid].acks.append((self.now, argparse.Namespace(command=cmd, result=result)))

    def command(self, sid, comp, cmd, confirmation, *params):
        self.events.append((self.now, sid, cmd, params))
        assert comp == 1
        s = self.state[sid]
        if cmd == gp.mav.MAV_CMD_DO_SET_MODE:
            s['mode'] = int(params[1])
        elif cmd == gp.mav.MAV_CMD_COMPONENT_ARM_DISARM:
            if params[0] and sid == self.reject_arm:
                self.ack(sid, cmd, gp.mav.MAV_RESULT_DENIED)
                return
            assert params[1] == 0  # force-arm/disarm is forbidden
            if not params[0]:
                assert s['pos'][2] == 0  # never disarm airborne
            s['armed'] = bool(params[0])
        elif cmd == gp.mav.MAV_CMD_NAV_TAKEOFF:
            assert all(v['armed'] for v in self.state.values())
            s['target'] = [*s['pos'][:2], -params[6]]
        elif cmd == gp.mav.MAV_CMD_NAV_LAND:
            s['mode'] = gp.gf.LAND
            s['target'] = [*s['pos'][:2], 0.0]
        self.ack(sid, cmd)

    def command_int(self, sid, comp, frame, cmd, *params):
        if cmd == gp.mav.MAV_CMD_DO_SET_HOME:
            self.state[sid]['home'] = round(params[-1] * 1000)
        self.ack(sid, cmd)

    def setpoint(self, boot, sid, comp, frame, mask, x, y, z, *rest):
        assert comp == 1 and frame == gp.mav.MAV_FRAME_LOCAL_NED and mask == gp.gf.MASK_POSITION_YAW
        self.state[sid]['target'] = [x, y, z]
        self.events.append((self.now, sid, 'setpoint', (x, y, z)))

    def publish(self):
        for sid, link in self.links.items():
            if sid == self.stale:
                continue
            s = self.state[sid]
            velocity = s.get('velocity', [0, 0, 0])
            messages = {
                'HEARTBEAT': gp.mav.MAVLink_heartbeat_message(2, 3, 128 if s['armed'] else 0, s['mode'], 4, 3),
                'LOCAL_POSITION_NED': gp.mav.MAVLink_local_position_ned_message(
                    int(self.now * 1000), *s['pos'], *velocity),
                'ATTITUDE': argparse.Namespace(yaw=s['yaw']),
                'EKF_STATUS_REPORT': argparse.Namespace(flags=gp.gf.EKF_REQUIRED),
                'EXTENDED_SYS_STATE': argparse.Namespace(landed_state=(gp.mav.MAV_LANDED_STATE_ON_GROUND
                                                                       if s['pos'][2] == 0 else 2)),
                'SYS_STATUS': argparse.Namespace(onboard_control_sensors_health=gp.mav.MAV_SYS_STATUS_PREARM_CHECK),
                'GPS_GLOBAL_ORIGIN': argparse.Namespace(latitude=473977420, longitude=85455940, altitude=0),
                'HOME_POSITION': gp.mav.MAVLink_home_position_message(
                    473977420, 85455940, s['home'], 0, 0, 0, [1, 0, 0, 0], 0, 0, 0),
            }
            for kind, message in messages.items():
                link.cache[kind] = (self.now, message)

    def pump(self):
        self.now += .1
        for sid, s in self.state.items():
            s['velocity'] = [0., 0., 0.]
            if s['armed'] and s['target'] is not None:
                delta = [b - a for a, b in zip(s['pos'], s['target'])]
                distance = math.sqrt(sum(v * v for v in delta))
                fraction = min(1, self.speed[sid] * .1 / distance) if distance else 0
                s['velocity'] = [v * fraction / .1 for v in delta]
                s['pos'] = [a + v * fraction for a, v in zip(s['pos'], delta)]
        self.publish()
        if self.takeover:
            self.links[self.takeover].takeover = True
        if self.guard_callback:
            self.guard_callback()


class PairTests(unittest.TestCase):
    def mission(self):
        args = gp.arguments(['--hover', '.5', '--hold', '.3'])
        relay = SimRelay(args)
        log = Mock()
        return gp.PairFlight(relay, args, log), relay, log

    def test_preview_has_no_sockets(self):
        with patch.object(gp, 'PairRelay') as relay, contextlib.redirect_stdout(io.StringIO()):
            self.assertEqual(gp.main([]), 0)
            relay.assert_not_called()

    def test_invalid_ids_and_dwell_rejected(self):
        for options in [['--target-systems', '2', '2'], ['--target-systems', '1', '255'],
                        ['--target-systems', '0', '2'], ['--hover', 'nan'], ['--takeoff-timeout', '-1']]:
            with self.subTest(options=options), contextlib.redirect_stderr(io.StringIO()):
                with self.assertRaises(SystemExit):
                    gp.arguments(options)

    def test_backwards_uses_each_heading(self):
        self.assertEqual(gp.backward_target((3, 4, -1), 0, 1), (2, 4, -1))
        target = gp.backward_target((3, 4, -1), math.pi / 2, 1)
        self.assertAlmostEqual(target[0], 3)
        self.assertAlmostEqual(target[1], 3)
        self.assertEqual(target[2], -1)

    def test_full_mission_and_lost_land_ack(self):
        flight, relay, log = self.mission()
        relay.drop_acks.add((2, gp.mav.MAV_CMD_NAV_LAND))
        relay.speed[2] = .5  # The quicker aircraft must wait at each barrier.
        with patch.object(gp.time, 'monotonic', side_effect=lambda: relay.now), contextlib.redirect_stdout(io.StringIO()):
            flight.run()
        self.assertTrue(all(not s['armed'] for s in relay.state.values()))
        phases = [c.kwargs['name'] for c in log.call_args_list if c.args == ('phase',)]
        self.assertEqual(phases, ['takeoff', 'hover', 'backward', 'return', 'land'])
        for sid, start in flight.starts.items():
            targets = [e[3] for e in relay.events if e[1:3] == (sid, 'setpoint')]
            self.assertEqual(targets[0], start)
            self.assertIn(gp.backward_target(start, flight.yaws[sid], 1), targets)
            self.assertEqual(targets[-1], start)
        takeoffs = [e for e in relay.events if e[2] == gp.mav.MAV_CMD_NAV_TAKEOFF]
        self.assertEqual(len(takeoffs), 2)
        self.assertEqual(takeoffs[0][0], takeoffs[1][0])
        settled = [c for c in log.call_args_list if c.args == ('settled',)]
        self.assertEqual(len(settled), 3)

    def test_arm_failure_never_starts_takeoff_and_lands_peer(self):
        flight, relay, _ = self.mission()
        relay.reject_arm = 2
        with patch.object(gp.time, 'monotonic', side_effect=lambda: relay.now), contextlib.redirect_stdout(io.StringIO()):
            with self.assertRaises(gp.gf.FlightError) as error:
                flight.run()
            flight.abort(error.exception)
        self.assertFalse(any(e[2] == gp.mav.MAV_CMD_NAV_TAKEOFF for e in relay.events))
        self.assertTrue(any(e[1:3] == (1, gp.mav.MAV_CMD_NAV_LAND) for e in relay.events))

    def test_stale_vehicle_aborts_peer_without_sending_to_lost_vehicle(self):
        flight, relay, _ = self.mission()
        with patch.object(gp.time, 'monotonic', side_effect=lambda: relay.now), contextlib.redirect_stdout(io.StringIO()):
            for f in flight.flights:
                f.armed_by_script = True
                f.monitor_mode = True
                relay.state[f.args.target_system].update(mode=gp.gf.GUIDED, armed=True)
            relay.publish()
            relay.stale = 2
            relay.now += 4
            relay.publish()
            with self.assertRaises(gp.gf.FlightError) as error:
                flight.check_group()
            flight.abort(error.exception)
        lands = [e[1] for e in relay.events if e[2] == gp.mav.MAV_CMD_NAV_LAND]
        self.assertEqual(lands, [1])

    def test_takeover_preserved_and_other_vehicle_lands(self):
        flight, relay, _ = self.mission()
        with patch.object(gp.time, 'monotonic', side_effect=lambda: relay.now), contextlib.redirect_stdout(io.StringIO()):
            for f in flight.flights:
                f.armed_by_script = f.monitor_mode = True
                relay.state[f.args.target_system].update(mode=gp.gf.GUIDED, armed=True)
            relay.publish()
            relay.links[2].takeover = True
            with self.assertRaises(gp.gf.FlightError) as error:
                flight.check_group()
            flight.abort(error.exception)
        self.assertEqual([e[1] for e in relay.events if e[2] == gp.mav.MAV_CMD_NAV_LAND], [1])

    def test_stationary_peer_times_out_and_both_receive_land(self):
        flight, relay, _ = self.mission()
        relay.speed[2] = 0
        flight.args.move_timeout = .5
        for f in flight.flights:
            f.armed_by_script = f.monitor_mode = True
            relay.state[f.args.target_system].update(mode=gp.gf.GUIDED, armed=True)
        flight.yaws = {1: 0, 2: 0}
        with patch.object(gp.time, 'monotonic', side_effect=lambda: relay.now), contextlib.redirect_stdout(io.StringIO()):
            relay.publish()
            relay.guard_callback = flight.check_group
            with self.assertRaises(gp.gf.FlightError) as error:
                flight.move({1: (-1, 0, 0), 2: (2, 4, 0)}, 'backward', .3)
            flight.abort(error.exception)
        self.assertEqual([e[1] for e in relay.events if e[2] == gp.mav.MAV_CMD_NAV_LAND], [1, 2])

    def test_disarm_requires_two_seconds_continuously_on_ground(self):
        flight, relay, _ = self.mission()
        for f in flight.flights:
            f.armed_by_script = f.monitor_mode = True
            relay.state[f.args.target_system].update(mode=gp.gf.GUIDED, armed=True)
        with patch.object(gp.time, 'monotonic', side_effect=lambda: relay.now):
            relay.publish()
            start = relay.now
            flight.land()
        disarms = [e for e in relay.events if e[2] == gp.mav.MAV_CMD_COMPONENT_ARM_DISARM]
        self.assertEqual(len(disarms), 2)
        self.assertTrue(all(e[0] - start >= 2 and e[3][:2] == (0, 0) for e in disarms))

    def test_unconfirmed_touchdown_never_disarms(self):
        flight, relay, _ = self.mission()
        for sid, s in relay.state.items():
            s.update(mode=gp.gf.GUIDED, armed=True)
            s['pos'][2] = -1
            relay.speed[sid] = 0
        with patch.object(gp.time, 'monotonic', side_effect=lambda: relay.now):
            relay.publish()
            with self.assertRaises(gp.gf.FlightError):
                flight.land()
        self.assertFalse(any(e[2] == gp.mav.MAV_CMD_COMPONENT_ARM_DISARM for e in relay.events))

    def test_shared_udp_demultiplexes_telemetry_acks_and_takeover(self):
        args = gp.arguments(['--listen', '127.0.0.1:14561'])
        args.listen = ('127.0.0.1', 0)
        with contextlib.ExitStack() as stack:
            bridge = stack.enter_context(socket.socket(socket.AF_INET, socket.SOCK_DGRAM))
            qgc = stack.enter_context(socket.socket(socket.AF_INET, socket.SOCK_DGRAM))
            bridge.bind(('127.0.0.1', 0))
            bridge.settimeout(1)
            qgc.bind(('127.0.0.1', 0))
            qgc.settimeout(1)
            args.qgc = qgc.getsockname()
            relay = gp.PairRelay(args, Mock())
            relay.heartbeat_enabled = False
            try:
                for sid in [1, 2, 42]:
                    enc = gp.mav.MAVLink(None, srcSystem=sid, srcComponent=1)
                    hb = gp.mav.MAVLink_heartbeat_message(2, 3, 0, sid, 4, 3).pack(enc)
                    ack = gp.mav.MAVLink_command_ack_message(22, 0, 0, 0, 255, 191).pack(enc)
                    # Exercise fragmented frames and several frames in one datagram.
                    bridge.sendto(hb[:7], relay.radio.getsockname())
                    relay.pump()
                    qgc.recvfrom(4096)
                    bridge.sendto(hb[7:] + ack, relay.radio.getsockname())
                    relay.pump()
                    qgc.recvfrom(4096)
                for sid in [1, 2]:
                    self.assertEqual(relay.links[sid].fresh('HEARTBEAT').get_srcSystem(), sid)
                    self.assertEqual(len(relay.links[sid].acks), 1)
                    relay.links[sid].watch_takeover = True
                msg = gp.mav.MAVLink_set_mode_message(2, 1, gp.gf.LAND)
                raw = msg.pack(gp.mav.MAVLink(None, srcSystem=255, srcComponent=190))
                qgc.sendto(raw, relay.qgc.getsockname())
                relay.pump()
                self.assertEqual(bridge.recvfrom(4096)[0], raw)
                self.assertFalse(relay.links[1].takeover)
                self.assertTrue(relay.links[2].takeover)
            finally:
                relay.close()


if __name__ == '__main__':
    unittest.main()
