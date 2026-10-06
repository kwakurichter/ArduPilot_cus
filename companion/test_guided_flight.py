#!/usr/bin/env python3
# AP_FLAKE8_CLEAN
"""Protocol and abort-path tests; no hardware connection."""
import math
import socket
import unittest
from unittest.mock import Mock, patch

import guided_flight as gf


class ProtocolTests(unittest.TestCase):
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
            self.assertEqual(gf.main(['--target-system', '1', '--pattern', 'three-axis']), 0)
            relay.assert_not_called()

    def test_unknown_pattern_rejected(self):
        with self.assertRaises(SystemExit):
            gf.arguments(['--target-system', '1', '--pattern', 'unknown'])

    def test_preview_never_opens_relay(self):
        with patch.object(gf, 'Relay') as relay:
            self.assertEqual(gf.main(['--target-system', '1']), 0)
            relay.assert_not_called()

    def test_nan_distance_rejected(self):
        with self.assertRaises(SystemExit):
            gf.arguments(['--target-system', '1', '--distance', 'nan'])

    def test_origin_arguments_must_be_complete(self):
        with self.assertRaises(SystemExit):
            gf.arguments(['--target-system', '1', '--lat', '1'])

    def flight(self, mode=gf.GUIDED, stale=False):
        link = Mock()
        link.takeover = False
        hb = gf.mav.MAVLink_heartbeat_message(2, 3, 128, mode, 4, 3)
        link.fresh.return_value = None if stale else hb
        args = gf.arguments(['--target-system', '1'])
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
        relay = gf.Relay(('127.0.0.1', 0), qgc.getsockname(), 255, 1, Mock())
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
            self.assertIsNotNone(relay.fresh('HEARTBEAT'))
            relay.watch_takeover = True
            command = gf.mav.MAVLink_set_mode_message(1, 1, 0)
            raw = command.pack(gf.mav.MAVLink(None, srcSystem=255, srcComponent=190))
            qgc.sendto(raw, reply_address)
            relay.pump()
            self.assertEqual(bridge.recvfrom(4096)[0], raw)
            self.assertTrue(relay.takeover)
            self.assertEqual(relay.bridge_peer, bridge.getsockname())
            # Another vehicle must not replace the controller's state.
            other = gf.mav.MAVLink_heartbeat_message(2, 3, 128, 9, 4, 3).pack(
                gf.mav.MAVLink(None, srcSystem=2, srcComponent=1))
            bridge.sendto(other, relay.radio.getsockname())
            relay.pump()
            self.assertEqual(relay.fresh('HEARTBEAT').custom_mode, 0)
        finally:
            relay.close()
            bridge.close()
            qgc.close()

    def test_landing_disarms_only_after_ground_confirmation(self):
        for grounded in (False, True):
            flight, link = self.flight(mode=gf.LAND)
            hb = link.fresh.return_value
            state = Mock(landed_state=(gf.mav.MAV_LANDED_STATE_ON_GROUND if grounded
                                       else gf.mav.MAV_LANDED_STATE_IN_AIR))
            link.fresh.side_effect = lambda kind, age: hb if kind == 'HEARTBEAT' else state
            now = [0.0]
            link.pump.side_effect = lambda: now.__setitem__(0, now[0] + 0.1)

            def command(cmd, *params):
                if cmd == gf.mav.MAV_CMD_COMPONENT_ARM_DISARM:
                    self.assertTrue(grounded)
                    self.assertEqual(params, (0, 0))  # Never force-disarm.
                    hb.base_mode = 0

            flight.command = Mock(side_effect=command)
            with patch.object(gf.time, 'monotonic', side_effect=lambda: now[0]):
                if grounded:
                    flight.land()
                    self.assertGreaterEqual(now[0], 2.0)
                    self.assertEqual(flight.command.call_count, 2)
                else:
                    with self.assertRaises(gf.FlightError):
                        flight.land()
                    flight.command.assert_called_once_with(gf.mav.MAV_CMD_NAV_LAND)

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

    def test_sequence_takes_off_without_horizontal_flag_but_never_translates_without_it(self):
        for acquire_position, pattern in [(False, 'left-return'), (True, 'left-return'),
                                          (False, 'three-axis'), (True, 'three-axis')]:
            flight, link, messages = self.position_flight()
            flight.args.pattern = pattern
            hb = messages['HEARTBEAT']
            hb.base_mode = 0
            hb.custom_mode = 0
            messages['HOME_POSITION'] = Mock(altitude=0)
            flight.setup_origin = Mock(return_value=Mock(altitude=0))
            flight.command_home = Mock()
            flight.prearm_checks = Mock()
            flight.move = Mock()
            flight.land = Mock()
            now = [0.]
            link.pump.side_effect = lambda: now.__setitem__(0, now[0] + .1)

            def command(cmd, *params):
                if cmd == gf.mav.MAV_CMD_DO_SET_MODE:
                    hb.custom_mode = gf.GUIDED
                elif cmd == gf.mav.MAV_CMD_COMPONENT_ARM_DISARM:
                    hb.base_mode = gf.mav.MAV_MODE_FLAG_SAFETY_ARMED
                elif cmd == gf.mav.MAV_CMD_NAV_TAKEOFF:
                    self.assertEqual(messages['EKF_STATUS_REPORT'].flags, 39)
                    self.assertEqual(params, (0, 0, 0, 0, 0, 0, 1.))
                    messages['LOCAL_POSITION_NED'].z = -1.
                    if acquire_position:
                        messages['EKF_STATUS_REPORT'].flags = gf.EKF_REQUIRED

            flight.command = Mock(side_effect=command)
            with patch.object(gf.time, 'monotonic', side_effect=lambda: now[0]):
                if acquire_position:
                    flight.run()
                    self.assertEqual(flight.move.call_count, 7 if pattern == 'three-axis' else 3)
                    expected = gf.mission_targets((0., 0., -1.), 0., 1., pattern)
                    self.assertEqual([c.args for c in flight.move.call_args_list[1:]],
                                     [(target, 0., label) for label, target in expected])
                    flight.land.assert_called_once()
                else:
                    with self.assertRaises(gf.FlightError):
                        flight.run()
                    flight.move.assert_not_called()
            takeoffs = [call for call in flight.command.call_args_list if call.args[0] == gf.mav.MAV_CMD_NAV_TAKEOFF]
            self.assertEqual(len(takeoffs), 1)

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

    def test_arm_retries_explicit_transient_rejection_without_force(self):
        for result in (gf.mav.MAV_RESULT_TEMPORARILY_REJECTED, gf.mav.MAV_RESULT_FAILED):
            flight, link = self.flight()
            hb = link.fresh.return_value
            hb.base_mode = 0
            now = [0.]
            link.pump.side_effect = lambda: now.__setitem__(0, now[0] + .1)

            def command(*args):
                if flight.command.call_count == 1:
                    raise gf.CommandRejected(args[0], result)
                hb.base_mode = gf.mav.MAV_MODE_FLAG_SAFETY_ARMED

            flight.command = Mock(side_effect=command)
            with patch.object(gf.time, 'monotonic', side_effect=lambda: now[0]):
                flight.arm()
            self.assertEqual(flight.command.call_count, 2)
            for call in flight.command.call_args_list:
                self.assertEqual(call.args, (gf.mav.MAV_CMD_COMPONENT_ARM_DISARM, 1, 0))

    def test_arm_does_not_retry_denial_or_ambiguous_timeout(self):
        for error in (gf.CommandRejected(400, gf.mav.MAV_RESULT_DENIED), gf.FlightError('ACK timeout')):
            flight, link = self.flight()
            link.fresh.return_value.base_mode = 0
            flight.command = Mock(side_effect=error)
            with self.assertRaises(gf.FlightError):
                flight.arm()
            flight.command.assert_called_once_with(gf.mav.MAV_CMD_COMPONENT_ARM_DISARM, 1, 0)

    def test_arm_lost_ack_accepts_observed_armed_state(self):
        flight, link = self.flight()
        hb = link.fresh.return_value
        hb.base_mode = 0

        def command(*args):
            hb.base_mode = gf.mav.MAV_MODE_FLAG_SAFETY_ARMED
            raise gf.FlightError('ACK timeout')

        flight.command = Mock(side_effect=command)
        flight.arm()
        flight.command.assert_called_once()

    def test_arm_retry_stops_on_mode_takeover(self):
        flight, link = self.flight()
        hb = link.fresh.return_value
        hb.base_mode = 0
        now = [0.]

        def pump():
            now[0] += .1
            hb.custom_mode = 0

        link.pump.side_effect = pump
        flight.command = Mock(side_effect=gf.CommandRejected(400, gf.mav.MAV_RESULT_TEMPORARILY_REJECTED))
        with patch.object(gf.time, 'monotonic', side_effect=lambda: now[0]):
            with self.assertRaises(gf.Takeover):
                flight.arm()
        flight.command.assert_called_once()

    def moving_flight(self, positions, use_defaults=False):
        flight, link, messages = self.position_flight(gf.EKF_REQUIRED)
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

    def test_untuned_defaults_accept_moderate_motion_after_one_second(self):
        flight, now, calls = self.moving_flight(lambda t: (.25, 0., .35, 0.), use_defaults=True)
        self.assertEqual((flight.args.position_tolerance, flight.args.settle_speed, flight.args.hold), (.3, .4, 1.))
        with patch.object(gf.time, 'monotonic', side_effect=lambda: now[0]):
            flight.move((0, 0, 0), 0, 'untuned leg')
        self.assertGreaterEqual(now[0], 1.)
        self.assertLess(now[0], 1.5)
        flight.command.assert_not_called()

    def test_untuned_defaults_reject_brief_pass_fast_motion_and_excess_height_error(self):
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
                gf.arguments(['--target-system', '1', '--settle-speed', speed])

    def test_wrong_vehicle_qgc_command_is_not_takeover(self):
        relay = object.__new__(gf.Relay)
        relay.watch_takeover = True
        relay.takeover = False
        relay.target_system = 1
        relay.qgc_command(gf.mav.MAVLink_set_mode_message(2, 1, 0))
        self.assertFalse(relay.takeover)


if __name__ == '__main__':
    unittest.main()
