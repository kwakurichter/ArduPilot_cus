#!/usr/bin/env python3
# AP_FLAKE8_CLEAN
"""Protocol and abort-path tests; no hardware connection."""
import io
import math
import socket
import unittest
from unittest.mock import Mock, patch

import guided_swarm as gf


class ProtocolTests(unittest.TestCase):
    def test_hover_preview_has_no_excursions_and_never_opens_relay(self):
        self.assertEqual(gf.mission_targets((3, 4, -1), .7, 1, 'hover'), [])
        with patch.object(gf, 'Relay') as relay, patch('sys.stdout', new_callable=io.StringIO) as output:
            self.assertEqual(gf.main(['--target-systems', '2', '--pattern', 'hover', '--hover', '5']), 0)
            relay.assert_not_called()
        self.assertIn('takeoff 1 m; hover 5 s; hover only; LAND', output.getvalue())
        self.assertIn('Maximum requested excursion from each hover spot: 0.00 m.', output.getvalue())

    def test_left_is_relative_to_captured_heading(self):
        self.assertEqual(gf.left_target((3, 4, -1), 0, 1), (3, 3, -1))
        p = gf.left_target((3, 4, -1), math.pi / 2, 1)
        self.assertAlmostEqual(p[0], 4)
        self.assertAlmostEqual(p[1], 4)
        self.assertEqual(p[2], -1)

    def test_three_axis_targets_return_to_fixed_start_at_arbitrary_heading(self):
        start = (3, 4, -1)
        for yaw in [0, math.pi / 2, math.pi, -.7]:
            legs = gf.mission_targets(start, yaw, 1.7, 'three-axis')
            self.assertEqual([label for label, _ in legs],
                             ['move left', 'return from left', 'move right', 'return from right',
                              'move backward', 'return from backward'])
            for i, (label, target) in enumerate(legs):
                self.assertEqual(target[2], start[2])
                if i % 2:
                    self.assertEqual(target, start)
                else:
                    dn, de = target[0] - start[0], target[1] - start[1]
                    forward = dn * math.cos(yaw) + de * math.sin(yaw)
                    right = -dn * math.sin(yaw) + de * math.cos(yaw)
                    expected = {0: (0, -1.7), 2: (0, 1.7), 4: (-1.7, 0)}[i]
                    self.assertAlmostEqual(forward, expected[0])
                    self.assertAlmostEqual(right, expected[1])

    def test_three_axis_preview_never_opens_relay(self):
        with patch.object(gf, 'Relay') as relay:
            self.assertEqual(gf.main(['--target-systems', '1', '--pattern', 'three-axis']), 0)
            relay.assert_not_called()

    def test_unknown_pattern_rejected(self):
        with self.assertRaises(SystemExit):
            gf.arguments(['--target-systems', '1', '--pattern', 'unknown'])

    def test_preview_never_opens_relay(self):
        with patch.object(gf, 'Relay') as relay:
            self.assertEqual(gf.main(['--target-systems', '1']), 0)
            relay.assert_not_called()

    def test_nan_distance_rejected(self):
        with self.assertRaises(SystemExit):
            gf.arguments(['--target-systems', '1', '--distance', 'nan'])

    def test_origin_arguments_must_be_complete(self):
        with self.assertRaises(SystemExit):
            gf.arguments(['--target-systems', '1', '--lat', '1'])

    def flight(self, mode=gf.GUIDED, stale=False):
        link = Mock()
        link.takeover = False
        hb = gf.mav.MAVLink_heartbeat_message(2, 3, 128, mode, 4, 3)
        link.fresh.return_value = None if stale else hb
        args = gf.arguments(['--target-systems', '1'])
        args.target_system = 1
        args.position_tolerance, args.settle_speed, args.hold = .3, .4, 1.
        flight = gf.Flight(link, args, Mock())
        flight.armed_by_script = True
        return flight, link

    def test_abort_guided_sends_land_never_disarm(self):
        flight, link = self.flight()
        flight.abort(gf.FlightError('position stale'))
        command = link.mav.command_long_send.call_args.args
        self.assertEqual(command[2], gf.mav.MAV_CMD_NAV_LAND)
        self.assertFalse(link.heartbeat_enabled)

    def test_abort_respects_takeover_and_other_modes(self):
        for mode, reason in [(0, gf.FlightError('mode')), (4, gf.Takeover('pilot'))]:
            flight, link = self.flight(mode)
            flight.abort(reason)
            link.mav.command_long_send.assert_not_called()

    def test_stale_heartbeat_abort_sends_no_commands(self):
        flight, link = self.flight(stale=True)
        flight.abort(gf.FlightError('link lost'))
        link.mav.command_long_send.assert_not_called()

    def test_relay_both_directions_and_sysid_filter(self):
        bridge = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        qgc = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        qgc.bind(('127.0.0.1', 0))
        qgc.settimeout(1)
        bridge.bind(('127.0.0.1', 0))
        bridge.settimeout(1)
        args = gf.arguments(['--target-systems', '1'])
        args.listen, args.qgc = ('127.0.0.1', 0), qgc.getsockname()
        relay = gf.Relay(args, Mock())
        link = relay.links[1]
        relay.heartbeat_enabled = False
        encoder = gf.mav.MAVLink(None, srcSystem=1, srcComponent=1)
        packet = gf.mav.MAVLink_heartbeat_message(2, 3, 0, 0, 3, 3).pack(encoder)
        try:
            # Radio can split a MAVLink frame across successive UDP datagrams.
            bridge.sendto(packet[:7], relay.radio.getsockname())
            relay.pump()
            first, reply_address = qgc.recvfrom(4096)
            bridge.sendto(packet[7:], relay.radio.getsockname())
            relay.pump()
            second, _ = qgc.recvfrom(4096)
            self.assertEqual(first + second, packet)
            self.assertIsNotNone(link.fresh('HEARTBEAT'))
            link.watch_takeover = True
            command = gf.mav.MAVLink_set_mode_message(1, 1, 0)
            raw = command.pack(gf.mav.MAVLink(None, srcSystem=255, srcComponent=190))
            qgc.sendto(raw, reply_address)
            relay.pump()
            self.assertEqual(bridge.recvfrom(4096)[0], raw)
            self.assertTrue(link.takeover)
            self.assertEqual(relay.bridge_peer, bridge.getsockname())
            # Another vehicle must not replace the controller's state.
            other = gf.mav.MAVLink_heartbeat_message(2, 3, 128, 9, 4, 3).pack(
                gf.mav.MAVLink(None, srcSystem=2, srcComponent=1))
            bridge.sendto(other, relay.radio.getsockname())
            relay.pump()
            self.assertEqual(link.fresh('HEARTBEAT').custom_mode, 0)
            # QGC's ACK for the same command must not confirm our request.
            for component, count in ((190, 0), (191, 1)):
                ack = gf.mav.MAVLink_command_ack_message(511, 0, 0, 0, 255, component).pack(encoder)
                bridge.sendto(ack, relay.radio.getsockname())
                relay.pump()
                self.assertEqual(len(link.acks), count)
        finally:
            relay.close()
            bridge.close()
            qgc.close()

    def test_position_mask_holds_yaw_and_uses_fixed_local_coordinates(self):
        flight, link = self.flight()
        flight.setpoint((3, 4, -1), 0.5)
        args = link.mav.set_position_target_local_ned_send.call_args.args
        self.assertEqual(args[3], gf.mav.MAV_FRAME_LOCAL_NED)
        self.assertEqual(args[4], 0x09F8)
        self.assertEqual(args[5:8], (3, 4, -1))
        self.assertEqual(args[-2:], (0.5, 0))

    def test_prearm_ack_is_not_treated_as_readiness(self):
        flight, link = self.flight()
        hb = link.fresh.return_value
        state = Mock(onboard_control_sensors_health=0)
        link.fresh.side_effect = lambda kind, age: state if kind == 'SYS_STATUS' else hb
        flight.command = Mock()  # Every RUN_PREARM_CHECKS is acknowledged ACCEPTED.
        now = [0.0]
        link.pump.side_effect = lambda: now.__setitem__(0, now[0] + 0.5)
        with patch.object(gf.time, 'monotonic', side_effect=lambda: now[0]):
            with self.assertRaises(gf.FlightError):
                flight.prearm_checks()
        self.assertTrue(flight.command.called)
        self.assertTrue(all(call.args == (gf.mav.MAV_CMD_RUN_PREARM_CHECKS,)
                            for call in flight.command.call_args_list))

    def position_flight(self, flags=39):
        flight, link = self.flight()
        messages = {
            'HEARTBEAT': link.fresh.return_value,
            'LOCAL_POSITION_NED': Mock(x=0., y=0., z=0., vx=0., vy=0., vz=0.),
            'EKF_STATUS_REPORT': Mock(flags=flags),
            'ATTITUDE': Mock(yaw=0.),
            'EXTENDED_SYS_STATE': Mock(landed_state=gf.mav.MAV_LANDED_STATE_ON_GROUND),
        }
        link.fresh.side_effect = lambda kind, age=2: messages.get(kind)
        return flight, link, messages

    def test_ground_flow_position_missing_allows_vertical_only(self):
        flight, link, messages = self.position_flight()
        self.assertTrue(flight.ground_ready())
        flight.guard(position=True, horizontal=False)
        with self.assertRaises(gf.FlightError):
            flight.guard(position=True)
        messages['EKF_STATUS_REPORT'].flags = gf.EKF_REQUIRED
        flight.guard(position=True)

    def test_ground_readiness_still_requires_vertical_state_and_fresh_telemetry(self):
        for missing in ('LOCAL_POSITION_NED', 'EKF_STATUS_REPORT', 'ATTITUDE', 'EXTENDED_SYS_STATE'):
            flight, link, messages = self.position_flight()
            messages.pop(missing)
            self.assertFalse(flight.ground_ready())
        for flags in (gf.EKF_VERTICAL_REQUIRED & ~gf.mav.EKF_VELOCITY_VERT,
                      gf.EKF_VERTICAL_REQUIRED | gf.mav.EKF_UNINITIALIZED):
            flight, link, messages = self.position_flight(flags)
            self.assertFalse(flight.ground_ready())
            with self.assertRaises(gf.FlightError):
                flight.guard(position=True, horizontal=False)
        flight, link, messages = self.position_flight()
        messages['ATTITUDE'].yaw = float('nan')
        self.assertFalse(flight.ground_ready())

    def test_origin_command_units_and_ack(self):
        flight, link = self.flight()
        flight.args.amsl = 123.5
        flight.wait_ack = Mock()
        flight.command_origin()
        args = link.mav.command_int_send.call_args.args
        self.assertEqual(args[:6], (1, 1, gf.mav.MAV_FRAME_GLOBAL, 611, 0, 0))
        self.assertEqual(args[6:10], (0, 0, 0, 0))
        self.assertEqual(args[10:], (473977420, 85455940, 123.5))
        self.assertEqual(flight.wait_ack.call_args.args[0], 611)

    def test_home_uses_explicit_origin_and_local_ground_height(self):
        flight, link, messages = self.position_flight()
        messages['LOCAL_POSITION_NED'].z = .2
        flight.wait_ack = Mock()
        flight.command_home(Mock(latitude=473977420, longitude=85455940, altitude=123500))
        args = link.mav.command_int_send.call_args.args
        self.assertEqual(args[:10], (1, 1, gf.mav.MAV_FRAME_GLOBAL, gf.mav.MAV_CMD_DO_SET_HOME,
                                     0, 0, 0, 0, 0, 0))
        self.assertEqual(args[10:12], (473977420, 85455940))
        self.assertAlmostEqual(args[12], 123.3)
        self.assertEqual(flight.wait_ack.call_args.args[0], gf.mav.MAV_CMD_DO_SET_HOME)

    def test_origin_reuses_existing_without_writing(self):
        flight, link = self.flight()
        origin = Mock(latitude=473977420, longitude=85455940, altitude=0)
        link.fresh.return_value = origin
        flight.command_origin = Mock()
        now = [0.]
        link.pump.side_effect = lambda: now.__setitem__(0, now[0] + .1)
        with patch.object(gf.time, 'monotonic', side_effect=lambda: now[0]):
            self.assertIs(flight.setup_origin(), origin)
        flight.command_origin.assert_not_called()

    def takeoff_ack_flight(self, state=gf.mav.MAV_LANDED_STATE_TAKEOFF, ack_result=None,
                           stale_kind=None, initial_state=gf.mav.MAV_LANDED_STATE_ON_GROUND, fault=None):
        flight, link, messages = self.position_flight()
        now = [10.]
        flight.monitor_mode = True
        messages['EXTENDED_SYS_STATE'] = Mock(landed_state=initial_state)
        link.cache = {kind: (9.9, msg) for kind, msg in messages.items()}
        link.acks = []
        link.fresh.side_effect = lambda kind, age=2: gf.VehicleLink.fresh(link, kind, age)
        flight.send_command = Mock()

        def pump():
            now[0] = round(now[0] + .1, 2)
            messages['EXTENDED_SYS_STATE'] = Mock(landed_state=state)
            if fault:
                fault(link, messages)
            for kind, msg in messages.items():
                if kind != stale_kind:
                    link.cache[kind] = (now[0], msg)
            if ack_result is not None and not link.acks:
                link.acks.append((now[0], Mock(command=gf.mav.MAV_CMD_NAV_TAKEOFF, result=ack_result)))

        link.pump.side_effect = pump
        return flight, link, now

    def test_takeoff_missing_ack_accepts_fresh_ground_to_takeoff_or_air_transition_once(self):
        for state in (gf.mav.MAV_LANDED_STATE_TAKEOFF, gf.mav.MAV_LANDED_STATE_IN_AIR):
            flight, link, now = self.takeoff_ack_flight(state=state)
            with patch.object(gf.time, 'monotonic', side_effect=lambda: now[0]):
                flight.command(gf.mav.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 1.)
            self.assertEqual(now[0], 15.)
            self.assertEqual(flight.unconfirmed_takeoff_since, 10.)
            flight.send_command.assert_called_once_with(gf.mav.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 1.)
            self.assertTrue(any(c.args[0] == 'takeoff_ack_missing_state_confirmed' for c in flight.log.call_args_list))

    def test_takeoff_ack_accepted_does_not_require_state_fallback(self):
        flight, link, now = self.takeoff_ack_flight(
            state=gf.mav.MAV_LANDED_STATE_ON_GROUND, ack_result=gf.mav.MAV_RESULT_ACCEPTED)
        with patch.object(gf.time, 'monotonic', side_effect=lambda: now[0]):
            flight.command(gf.mav.MAV_CMD_NAV_TAKEOFF)
        self.assertEqual(now[0], 10.1)
        self.assertIsNone(flight.unconfirmed_takeoff_since)

    def test_takeoff_rejection_is_not_overridden_by_airborne_state(self):
        for result in (gf.mav.MAV_RESULT_DENIED, gf.mav.MAV_RESULT_FAILED, gf.mav.MAV_RESULT_TEMPORARILY_REJECTED):
            flight, link, now = self.takeoff_ack_flight(ack_result=result)
            with patch.object(gf.time, 'monotonic', side_effect=lambda: now[0]):
                with self.assertRaises(gf.CommandRejected):
                    flight.command(gf.mav.MAV_CMD_NAV_TAKEOFF)
            flight.send_command.assert_called_once()
            self.assertIsNone(flight.unconfirmed_takeoff_since)

    def test_takeoff_missing_ack_requires_fresh_transition_and_valid_vertical_state(self):
        cases = [{'state': state} for state in (
            gf.mav.MAV_LANDED_STATE_UNDEFINED, gf.mav.MAV_LANDED_STATE_ON_GROUND, gf.mav.MAV_LANDED_STATE_LANDING)]
        cases += [{'stale_kind': kind} for kind in (
            'EXTENDED_SYS_STATE', 'HEARTBEAT', 'LOCAL_POSITION_NED', 'EKF_STATUS_REPORT')]
        cases += [{'initial_state': gf.mav.MAV_LANDED_STATE_TAKEOFF},
                  {'fault': lambda link, messages: setattr(messages['EKF_STATUS_REPORT'], 'flags', 0)},
                  {'fault': lambda link, messages: setattr(messages['LOCAL_POSITION_NED'], 'z', float('nan'))},
                  {'fault': lambda link, messages: setattr(messages['HEARTBEAT'], 'base_mode', 0)},
                  {'fault': lambda link, messages: setattr(messages['HEARTBEAT'], 'custom_mode', gf.LAND)},
                  {'fault': lambda link, messages: setattr(link, 'takeover', True)}]
        for kwargs in cases:
            with self.subTest(kwargs=kwargs):
                flight, link, now = self.takeoff_ack_flight(**kwargs)
                with patch.object(gf.time, 'monotonic', side_effect=lambda: now[0]):
                    with self.assertRaises(gf.FlightError):
                        flight.command(gf.mav.MAV_CMD_NAV_TAKEOFF)
                flight.send_command.assert_called_once()
                self.assertIsNone(flight.unconfirmed_takeoff_since)

    def test_takeoff_state_fallback_still_handles_late_acceptance_or_rejection(self):
        for result in (gf.mav.MAV_RESULT_ACCEPTED, gf.mav.MAV_RESULT_DENIED):
            flight, link, now = self.takeoff_ack_flight()
            with patch.object(gf.time, 'monotonic', side_effect=lambda: now[0]):
                flight.command(gf.mav.MAV_CMD_NAV_TAKEOFF)
                link.acks.append((now[0], Mock(command=gf.mav.MAV_CMD_NAV_TAKEOFF, result=result)))
                if result == gf.mav.MAV_RESULT_ACCEPTED:
                    flight.guard(position=True, horizontal=False)
                    self.assertIsNone(flight.unconfirmed_takeoff_since)
                else:
                    with self.assertRaises(gf.CommandRejected):
                        flight.guard(position=True, horizontal=False)

    def test_other_command_ack_timeouts_remain_failures(self):
        flight, link, now = self.takeoff_ack_flight()
        with patch.object(gf.time, 'monotonic', side_effect=lambda: now[0]):
            with self.assertRaises(gf.WaitTimeout):
                flight.command(gf.mav.MAV_CMD_REQUEST_MESSAGE, 49)
        self.assertIsNone(flight.unconfirmed_takeoff_since)

    def test_setup_missing_ack_retries_once_then_continues_without_flight_commands(self):
        for command, params in ((511, (32, 100000)), (178, (1, .2, -1))):
            flight, link, now = self.takeoff_ack_flight(state=gf.mav.MAV_LANDED_STATE_ON_GROUND)
            with patch.object(gf.time, 'monotonic', side_effect=lambda: now[0]):
                flight.command(command, *params)
            self.assertEqual(flight.send_command.call_count, 2)
            self.assertTrue(all(c.args == (command, *params) for c in flight.send_command.call_args_list))
            self.assertEqual(now[0], 14.)
            self.assertTrue(any(c.args[0] == 'setup_unconfirmed' for c in flight.log.call_args_list))

    def test_setup_retry_can_receive_ack_and_rejection_is_logged_without_retry(self):
        for result in (gf.mav.MAV_RESULT_ACCEPTED, gf.mav.MAV_RESULT_UNSUPPORTED):
            flight, link, now = self.takeoff_ack_flight()
            pump = link.pump.side_effect

            def receive():
                pump()
                if (flight.send_command.call_count == 2 or result == gf.mav.MAV_RESULT_UNSUPPORTED) and not link.acks:
                    link.acks.append((now[0], Mock(command=511, result=result)))

            link.pump.side_effect = receive
            with patch.object(gf.time, 'monotonic', side_effect=lambda: now[0]):
                flight.command(511, 32, 100000)
            self.assertEqual(flight.send_command.call_count, 2 if result == gf.mav.MAV_RESULT_ACCEPTED else 1)
            warned = any(c.args[0] == 'setup_unconfirmed' for c in flight.log.call_args_list)
            self.assertEqual(warned, result == gf.mav.MAV_RESULT_UNSUPPORTED)

    def test_setup_does_not_swallow_link_loss_or_pilot_takeover(self):
        for kwargs in ({'stale_kind': 'HEARTBEAT'},
                       {'fault': lambda link, messages: setattr(link, 'takeover', True)}):
            flight, link, now = self.takeoff_ack_flight(**kwargs)
            with patch.object(gf.time, 'monotonic', side_effect=lambda: now[0]):
                with self.assertRaises(gf.FlightError):
                    flight.command(511, 32, 100000)
            self.assertFalse(any(c.args[0] == 'setup_unconfirmed' for c in flight.log.call_args_list))

    def test_setup_helper_cannot_be_used_for_arm_or_takeoff(self):
        flight, link = self.flight()
        for command in (gf.mav.MAV_CMD_COMPONENT_ARM_DISARM, gf.mav.MAV_CMD_NAV_TAKEOFF, gf.mav.MAV_CMD_NAV_LAND):
            with self.assertRaises(ValueError):
                flight.configure_command(command)
        link.mav.command_long_send.assert_not_called()

    def attach_group_move(self, flight, link):
        group = object.__new__(gf.GroupFlight)
        group.args, group.log, group.relay = flight.args, flight.log, link
        group.flights, group.yaws = [flight], {1: 0}

        def move(target, yaw, label, horizontal_recovery=False):
            group.yaws[1] = yaw
            group.move({1: target}, label, flight.args.hold, horizontal_recovery=horizontal_recovery)

        flight.move = move

    def moving_flight(self, positions, use_defaults=False):
        flight, link, messages = self.position_flight(gf.EKF_REQUIRED)
        self.attach_group_move(flight, link)
        flight.args.move_timeout = 8
        if not use_defaults:
            # Retain coverage of user-selected strict limits and long dwell.
            flight.args.position_tolerance = .2
            flight.args.settle_speed = .2
            flight.args.hold = 3.
        now = [0.]
        calls = []
        flight.setpoint = Mock(side_effect=lambda *args: calls.append('target'))
        flight.command = Mock(side_effect=lambda *args: calls.append(('speed', args)))

        def pump():
            now[0] += .1
            x, z, vx, vz = positions(now[0])
            p = messages['LOCAL_POSITION_NED']
            p.x, p.y, p.z, p.vx, p.vy, p.vz = x, 0., z, vx, 0., vz

        link.pump.side_effect = pump
        return flight, now, calls

    def test_arrival_accepts_observed_small_motion_without_changing_speed(self):
        flight, now, calls = self.moving_flight(lambda t: (.12, 0., .14, 0.))
        with patch.object(gf.time, 'monotonic', side_effect=lambda: now[0]):
            flight.move((0, 0, 0), 0, 'test leg')
        self.assertGreaterEqual(now[0], 3.)
        self.assertLess(now[0], 3.5)
        self.assertTrue(calls)
        self.assertTrue(all(call == 'target' for call in calls))
        flight.command.assert_not_called()

    def hover_recovery_flight(self, invalid_until, during_dropout=None):
        flight, link, messages = self.position_flight(gf.EKF_REQUIRED)
        self.attach_group_move(flight, link)
        flight.args.hold = 1.
        flight.args.move_timeout = 4.
        flight.monitor_mode = True
        now = [0.]
        sent = []
        flight.setpoint = Mock(side_effect=lambda target, yaw: sent.append((now[0], target, yaw)))

        def pump():
            now[0] = round(now[0] + .05, 2)
            invalid = .4 <= now[0] < invalid_until
            messages['EKF_STATUS_REPORT'].flags = gf.EKF_REQUIRED & ~gf.mav.EKF_POS_HORIZ_REL if invalid else gf.EKF_REQUIRED
            if invalid and during_dropout:
                during_dropout(link, messages)

        link.pump.side_effect = pump
        return flight, now, sent

    def test_hover_recovers_from_brief_position_loss_and_restarts_dwell(self):
        flight, now, sent = self.hover_recovery_flight(.65)
        with patch.object(gf.time, 'monotonic', side_effect=lambda: now[0]):
            flight.move((0, 0, 0), .7, 'hover', horizontal_recovery=True)
        self.assertGreaterEqual(now[0], 1.65)
        self.assertLess(now[0], 1.8)
        self.assertTrue(any(.4 <= t < .65 for t, _, _ in sent))
        self.assertTrue(all(target == (0, 0, 0) and yaw == .7 for _, target, yaw in sent))
        events = [c.args[0] for c in flight.log.call_args_list]
        self.assertEqual(events.count('horizontal_recovery_started'), 1)
        self.assertEqual(events.count('horizontal_recovery_complete'), 1)
        self.assertIn('settled', events)

    def test_hover_persistent_loss_or_late_recovery_aborts_within_bound(self):
        # Repeated invalid reports cannot extend the timeout; a good report at
        # the deadline must not erase an already expired recovery window.
        for until in (10., 1.4):
            flight, now, sent = self.hover_recovery_flight(until)
            with patch.object(gf.time, 'monotonic', side_effect=lambda: now[0]):
                with self.assertRaisesRegex(gf.FlightError, 'did not recover'):
                    flight.move((0, 0, 0), 0, 'hover', horizontal_recovery=True)
            self.assertLessEqual(now[0], 1.45)
            self.assertFalse(any(c.args[0] == 'settled' for c in flight.log.call_args_list))

    def test_position_legs_keep_immediate_horizontal_abort(self):
        flight, now, sent = self.hover_recovery_flight(.65)
        with patch.object(gf.time, 'monotonic', side_effect=lambda: now[0]):
            with self.assertRaises(gf.FlightError):
                flight.move((0, 0, 0), 0, 'translation')
        self.assertEqual(now[0], .4)

    def test_hover_recovery_never_masks_other_required_state_failures(self):
        faults = []
        for bit in (gf.mav.EKF_ATTITUDE, gf.mav.EKF_VELOCITY_HORIZ, gf.mav.EKF_VELOCITY_VERT, gf.mav.EKF_POS_VERT_ABS):
            faults.append(lambda link, messages, bit=bit: setattr(
                messages['EKF_STATUS_REPORT'], 'flags', messages['EKF_STATUS_REPORT'].flags & ~bit))
        faults.extend([
            lambda link, messages: setattr(messages['EKF_STATUS_REPORT'], 'flags', gf.EKF_REQUIRED | gf.mav.EKF_UNINITIALIZED),
            lambda link, messages: messages.pop('EKF_STATUS_REPORT'),
            lambda link, messages: messages.pop('LOCAL_POSITION_NED'),
            lambda link, messages: setattr(messages['LOCAL_POSITION_NED'], 'z', float('nan')),
            lambda link, messages: messages.pop('HEARTBEAT'),
            lambda link, messages: setattr(messages['HEARTBEAT'], 'custom_mode', gf.LAND),
            lambda link, messages: setattr(messages['HEARTBEAT'], 'base_mode', 0),
            lambda link, messages: setattr(link, 'takeover', True),
        ])
        for fault in faults:
            flight, now, sent = self.hover_recovery_flight(10., fault)
            with patch.object(gf.time, 'monotonic', side_effect=lambda: now[0]):
                with self.assertRaises(gf.FlightError):
                    flight.move((0, 0, 0), 0, 'hover', horizontal_recovery=True)
            self.assertEqual(now[0], .4)

    def test_arrival_speed_or_distance_excursion_resets_dwell(self):
        for excursion in ((.25, 0., .1, 0.), (.1, 0., .25, 0.)):
            flight, now, calls = self.moving_flight(lambda t: excursion if 2. <= t <= 2.5 else (.1, 0., .1, 0.))
            with patch.object(gf.time, 'monotonic', side_effect=lambda: now[0]):
                flight.move((0, 0, 0), 0, 'test leg')
            self.assertGreaterEqual(now[0], 5.5)
            self.assertLess(now[0], 6.)

    def test_arrival_never_completes_a_fast_pass_or_wrong_altitude(self):
        for state in ((0., 0., .25, 0.), (0., .25, 0., 0.), (0., 0., 0., .25)):
            flight, now, calls = self.moving_flight(lambda t: state)
            with patch.object(gf.time, 'monotonic', side_effect=lambda: now[0]):
                with self.assertRaises(gf.FlightError):
                    flight.move((0, 0, 0), 0, 'test leg')
            self.assertTrue(any(c.args[0] == 'arrival_progress' for c in flight.log.call_args_list))
            self.assertFalse(any(c.args[0] == 'settled' for c in flight.log.call_args_list))

    def test_configured_coarse_limits_accept_moderate_motion_after_one_second(self):
        flight, now, calls = self.moving_flight(lambda t: (.25, 0., .35, 0.), use_defaults=True)
        self.assertEqual((flight.args.position_tolerance, flight.args.settle_speed, flight.args.hold), (.3, .4, 1.))
        with patch.object(gf.time, 'monotonic', side_effect=lambda: now[0]):
            flight.move((0, 0, 0), 0, 'untuned leg')
        self.assertGreaterEqual(now[0], 1.)
        self.assertLess(now[0], 1.5)
        flight.command.assert_not_called()

    def test_configured_coarse_limits_reject_brief_pass_fast_motion_and_excess_height_error(self):
        profiles = (
            lambda t: (.1, 0., .3, 0.) if 1. < t < 1.5 else (.5, 0., .3, 0.),
            lambda t: (0., 0., .5, 0.),
            lambda t: (0., .4, 0., 0.),
        )
        for profile in profiles:
            flight, now, calls = self.moving_flight(profile, use_defaults=True)
            with patch.object(gf.time, 'monotonic', side_effect=lambda: now[0]):
                with self.assertRaises(gf.FlightError):
                    flight.move((0, 0, 0), 0, 'untuned leg')
            self.assertFalse(any(c.args[0] == 'settled' for c in flight.log.call_args_list))

    def test_settle_speed_must_be_finite_and_positive(self):
        for speed in ('0', '-1', 'nan', 'inf'):
            with self.assertRaises(SystemExit):
                gf.arguments(['--target-systems', '1', '--settle-speed', speed])

    def test_wrong_vehicle_qgc_command_is_not_takeover(self):
        link = object.__new__(gf.VehicleLink)
        link.watch_takeover = True
        link.takeover = False
        link.target_system = 1
        link.qgc_command(gf.mav.MAVLink_set_mode_message(2, 1, 0))
        self.assertFalse(link.takeover)


if __name__ == '__main__':
    unittest.main()
