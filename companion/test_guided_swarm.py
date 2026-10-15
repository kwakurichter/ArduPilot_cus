#!/usr/bin/env python3
# AP_FLAKE8_CLEAN
"""GUIDED demo geometry, protocol, group sequencing and abort regression tests."""

import contextlib
import io
import itertools
import math
import unittest
from unittest.mock import Mock, patch

import guided_swarm as gs
from test_guided_swarm_group import SimRelay


class SwarmSimRelay(SimRelay):
    def setpoint(self, boot, sid, comp, frame, mask, x, y, z, *rest):
        if mask != gs.MASK_POSITION_VELOCITY_YAW:
            return super().setpoint(boot, sid, comp, frame, mask, x, y, z, *rest)
        assert comp == 1 and frame == gs.mav.MAV_FRAME_LOCAL_NED
        self.state[sid]['target'] = [x, y, z]
        self.events.append((self.now, sid, 'trajectory', (x, y, z, *rest)))


class SwarmTests(unittest.TestCase):
    def mission(self, count=2):
        args = gs.arguments(['--target-systems', *map(str, range(1, count + 1)), '--hover', '.5', '--hold', '.5'])
        relay = SwarmSimRelay(args)
        log = Mock()
        return gs.SwarmFlight(relay, args, log), relay, log

    def run_mission(self, flight, relay):
        with patch.object(gs.time, 'monotonic', side_effect=lambda: relay.now), contextlib.redirect_stdout(io.StringIO()):
            flight.run()

    def test_preview_does_not_open_transport(self):
        for count in range(1, 5):
            with patch.object(gs, 'Relay') as transport, contextlib.redirect_stdout(io.StringIO()):
                self.assertEqual(gs.main(['--target-systems', *map(str, range(1, count + 1))]), 0)
                transport.assert_not_called()

    def test_reject_invalid_plan(self):
        for options in (['--target-systems', '1', '1'], ['--target-systems', '1', '2', '3', '4', '5'],
                        ['--target-systems', '0'], ['--target-systems', '255'], ['--duration', 'nan'],
                        ['--radius', 'inf'], ['--radius', '.4', '--duration', '10'],
                        ['--target-systems', '1', '2', '3', '4', '--spacing', '1.5'],
                        ['--max-flight-time', '100'], ['--duration', '5'], ['--height', 'nan']):
            with self.subTest(options=options), contextlib.redirect_stderr(io.StringIO()):
                with self.assertRaises(SystemExit):
                    gs.arguments(options)

    def test_circle_is_closed_smooth_and_speed_bounded(self):
        zero = (0., 0., 0.)
        for t in (-1, 0, 12, 13):
            self.assertEqual(gs.sweep_sample(t, 12, .25), (zero, zero))
        peak = 0
        for i in range(1, 1200):
            t = i / 100
            p, v = gs.sweep_sample(t, 12, .25)
            self.assertAlmostEqual(math.hypot(p[0] + .25, p[1]), .25)
            eps = 1e-5
            before, _ = gs.sweep_sample(t - eps, 12, .25)
            after, _ = gs.sweep_sample(t + eps, 12, .25)
            for axis in range(3):
                self.assertAlmostEqual((after[axis] - before[axis]) / (2 * eps), v[axis], places=7)
            peak = max(peak, math.dist(v, zero))
        self.assertLessEqual(peak, .246)

    def test_all_nominal_distances_preserved(self):
        for count in range(1, 5):
            points = gs.layout(count, 1.25 if count == 4 else 1.5)
            for tick in range(121):
                delta, _ = gs.sweep_sample(tick / 10, 12, .25)
                moved = [tuple(a + b for a, b in zip(p, delta)) for p in points]
                for i, j in itertools.combinations(range(count), 2):
                    distance = math.dist(moved[i], moved[j])
                    self.assertAlmostEqual(distance, math.dist(points[i], points[j]))
                    self.assertTrue(1.15 <= distance <= 1.85)

    def test_complete_missions_for_one_through_four(self):
        for count in range(1, 5):
            with self.subTest(count=count):
                flight, relay, log = self.mission(count)
                self.run_mission(flight, relay)
                self.assertTrue(all(not s['armed'] for s in relay.state.values()))
                phases = [c.kwargs['name'] for c in log.call_args_list if c.args == ('phase',)]
                self.assertEqual(phases, ['takeoff', 'hover', 'sweep', 'return', 'land'])
                frames = {}
                for c in log.call_args_list:
                    if c.args != ('trajectory_setpoint',):
                        continue
                    frame = c.kwargs
                    sid = frame['system']
                    delta = tuple(a - b for a, b in zip(frame['ned'], flight.starts[sid]))
                    offset = gs.rotate(delta, -flight.yaws[sid])
                    velocity = gs.rotate(frame['velocity'], -flight.yaws[sid])
                    frames.setdefault(frame['elapsed'], []).append((offset, velocity))
                self.assertGreaterEqual(len(frames), 45)
                for members in frames.values():
                    self.assertEqual(len(members), count)
                    for p, v in members[1:]:
                        self.assertLess(math.dist(p, members[0][0]), 1e-12)
                        self.assertLess(math.dist(v, members[0][1]), 1e-12)
                self.assertIn(12, frames)
                self.assertLess(math.dist(frames[12][0][1], (0, 0, 0)), 1e-12)

    def test_circles_complete_with_missing_takeoff_acks(self):
        for count in range(1, 5):
            with self.subTest(count=count):
                flight, relay, log = self.mission(count)
                missing = {sid for sid in relay.links if sid % 2 or count == 1}
                relay.drop_acks.update((sid, gs.mav.MAV_CMD_NAV_TAKEOFF) for sid in missing)
                self.run_mission(flight, relay)
                confirmed = {c.kwargs['system'] for c in log.call_args_list
                             if c.args == ('takeoff_ack_missing_state_confirmed',)}
                self.assertEqual(confirmed, missing)
                takeoffs = [e for e in relay.events if e[2] == gs.mav.MAV_CMD_NAV_TAKEOFF]
                self.assertEqual(len(takeoffs), count)
                self.assertEqual(len({e[0] for e in takeoffs}), 1)
                self.assertTrue(any(e[2] == 'trajectory' for e in relay.events))
                self.assertTrue(all(not s['armed'] for s in relay.state.values()))

    def install_flag_faults(self, relay):
        faults = []
        publish = relay.publish

        def publish_with_faults():
            publish()
            for sid, start, end, flags in faults:
                if start <= relay.now < end:
                    relay.links[sid].cache['EKF_STATUS_REPORT'][1].flags = flags

        relay.publish = publish_with_faults
        return faults

    def test_brief_horizontal_loss_in_hover_restarts_group_dwell(self):
        flight, relay, log = self.mission(2)
        faults = self.install_flag_faults(relay)
        observed = []

        def events(event, **fields):
            observed.append((relay.now, event, fields))
            if event == 'phase' and fields['name'] == 'hover':
                faults.append((2, relay.now + .2, relay.now + .6, gs.EKF_REQUIRED & ~gs.mav.EKF_POS_HORIZ_REL))

        log.side_effect = events
        self.run_mission(flight, relay)
        recovered = next(t for t, e, _ in observed if e == 'horizontal_recovery_complete')
        settled = next(t for t, e, f in observed if e == 'settled' and f['name'] == 'hover')
        self.assertGreaterEqual(settled - recovered, flight.args.hover)
        self.assertTrue(all(not s['armed'] for s in relay.state.values()))

    def test_sweep_pauses_every_vehicle_then_resumes_without_phase_jump(self):
        flight, relay, log = self.mission(4)
        faults = self.install_flag_faults(relay)
        observed = []

        def events(event, **fields):
            observed.append((relay.now, event, fields))
            if event == 'trajectory_setpoint' and fields['elapsed'] >= 4 and not faults:
                faults.append((2, relay.now + .05, relay.now + .55, gs.EKF_REQUIRED & ~gs.mav.EKF_POS_HORIZ_REL))

        log.side_effect = events
        self.run_mission(flight, relay)
        pause_t, _, pause = next(e for e in observed if e[1] == 'trajectory_paused')
        resume_t, _, resume = next(e for e in observed if e[1] == 'trajectory_resumed')
        self.assertGreater(resume_t - pause_t, .3)
        self.assertAlmostEqual(pause['elapsed'], resume['elapsed'])
        for sid in relay.links:
            frames = [(t, f) for t, e, f in observed if e == 'trajectory_setpoint' and f['system'] == sid]
            before = [f for t, f in frames if t < pause_t][-1]
            paused = [f for t, f in frames if pause_t <= t < resume_t]
            after = [f for t, f in frames if t >= resume_t][0]
            self.assertGreaterEqual(len(paused), 2)
            for f in paused:
                self.assertEqual(f['ned'], before['ned'])
                self.assertEqual(f['velocity'], (0., 0., 0.))
                self.assertEqual(f['elapsed'], before['elapsed'])
            self.assertEqual(after['ned'], before['ned'])
            self.assertAlmostEqual(after['elapsed'], before['elapsed'])
        self.assertTrue(all(not s['armed'] for s in relay.state.values()))

    def test_persistent_horizontal_loss_times_out_in_hover_and_sweep(self):
        for phase in ('hover', 'sweep', 'return'):
            flight, relay, log = self.mission(2)
            faults = self.install_flag_faults(relay)

            def events(event, **fields):
                if event == 'phase' and fields['name'] == phase:
                    faults.append((2, relay.now, relay.now + 2, gs.EKF_REQUIRED & ~gs.mav.EKF_POS_HORIZ_REL))

            log.side_effect = events
            with self.subTest(phase=phase), self.assertRaisesRegex(gs.FlightError, 'did not recover within 1s'):
                self.run_mission(flight, relay)

    def test_other_ekf_losses_remain_immediate(self):
        for bit in (gs.mav.EKF_VELOCITY_HORIZ, gs.mav.EKF_VELOCITY_VERT, gs.mav.EKF_POS_VERT_ABS):
            flight, relay, log = self.mission(1)
            faults = self.install_flag_faults(relay)

            def events(event, **fields):
                if event == 'phase' and fields['name'] == 'sweep':
                    faults.append((1, relay.now, relay.now + .4, gs.EKF_REQUIRED & ~bit))

            log.side_effect = events
            with self.subTest(bit=bit), self.assertRaisesRegex(gs.FlightError, 'EKF no longer reports'):
                self.run_mission(flight, relay)
            self.assertFalse(any(c.args == ('trajectory_paused',) for c in log.call_args_list))

    def test_recovery_deadline_is_checked_even_when_next_report_is_good(self):
        flight, relay, _ = self.mission(1)
        lost = {}
        with patch.object(gs.time, 'monotonic', side_effect=lambda: relay.now):
            relay.links[1].cache['EKF_STATUS_REPORT'][1].flags &= ~gs.mav.EKF_POS_HORIZ_REL
            self.assertFalse(flight.horizontal_ready(lost, 'sweep'))
            relay.now += gs.HORIZONTAL_RECOVERY_TIMEOUT
            relay.publish()
            with self.assertRaisesRegex(gs.FlightError, 'did not recover within 1s'):
                flight.horizontal_ready(lost, 'sweep')

    def test_paused_sweep_retains_position_tracking_and_spacing_checks(self):
        for failure in ('stale', 'tracking', 'separation'):
            flight, relay, log = self.mission(2)
            faults = self.install_flag_faults(relay)

            def events(event, **fields):
                if event == 'phase' and fields['name'] == 'sweep':
                    faults.append((2, relay.now, relay.now + .5, gs.EKF_REQUIRED & ~gs.mav.EKF_POS_HORIZ_REL))
                if event == 'trajectory_paused':
                    if failure == 'stale':
                        relay.stale = 2
                        _, p = relay.links[2].cache['LOCAL_POSITION_NED']
                        relay.links[2].cache['LOCAL_POSITION_NED'] = (relay.now - 2, p)
                    elif failure == 'tracking':
                        relay.links[2].cache['LOCAL_POSITION_NED'][1].x += 1
                    else:
                        relay.state[2]['pos'][0] += 5

            log.side_effect = events
            # A stale position may be caught by the tracking check first;
            # either way, pausing must never suppress the stop.
            pattern = 'tracking' if failure in ('stale', 'tracking') else 'separation'
            with self.subTest(failure=failure), self.assertRaisesRegex(gs.FlightError, pattern):
                self.run_mission(flight, relay)

    def test_overlapping_peer_dropouts_cannot_extend_group_pause(self):
        flight, relay, log = self.mission(2)
        faults = self.install_flag_faults(relay)

        def events(event, **fields):
            if event == 'phase' and fields['name'] == 'sweep':
                flags = gs.EKF_REQUIRED & ~gs.mav.EKF_POS_HORIZ_REL
                faults.extend([(1, relay.now, relay.now + .8, flags),
                               (2, relay.now + .6, relay.now + 1.4, flags)])

        log.side_effect = events
        with self.assertRaisesRegex(gs.FlightError, 'Group horizontal position did not recover'):
            self.run_mission(flight, relay)

    def test_diamond_vertices_velocity_and_straight_edges(self):
        vertices = ((0., 0., 0.), (-.25, .25, 0.), (-.5, 0., 0.), (-.25, -.25, 0.), (0., 0., 0.))
        for i, vertex in enumerate(vertices):
            self.assertEqual(gs.diamond_sample(i * 4, 16, .25), (vertex, (0., 0., 0.)))
        for i in range(1, 1600):
            t = i / 100
            p, v = gs.diamond_sample(t, 16, .25)
            self.assertAlmostEqual(abs(p[0] + .25) + abs(p[1]), .25)
            eps = 1e-5
            before, _ = gs.diamond_sample(t - eps, 16, .25)
            after, _ = gs.diamond_sample(t + eps, 16, .25)
            for axis in range(3):
                self.assertAlmostEqual((after[axis] - before[axis]) / (2 * eps), v[axis], places=7)
            self.assertLessEqual(math.dist(v, (0, 0, 0)), gs.peak_speed('diamond', .25, 16) + 1e-12)
        for t in (-1, 17):
            self.assertEqual(gs.diamond_sample(t, 16, .25), ((0., 0., 0.), (0., 0., 0.)))

    def test_diamond_missions_for_one_through_four(self):
        for count in range(1, 5):
            with self.subTest(count=count):
                flight, relay, log = self.mission(count)
                flight.args.pattern = 'diamond'
                flight.args.duration = 16
                self.run_mission(flight, relay)
                frames = [c.kwargs for c in log.call_args_list if c.args == ('trajectory_setpoint',)]
                self.assertEqual({f['system'] for f in frames}, set(relay.links))
                for f in frames:
                    offset, velocity = gs.diamond_sample(f['elapsed'], 16, .25)
                    yaw = flight.yaws[f['system']]
                    expected = tuple(a + b for a, b in zip(flight.starts[f['system']], gs.rotate(offset, yaw)))
                    self.assertLess(math.dist(f['ned'], expected), 1e-12)
                    self.assertLess(math.dist(f['velocity'], gs.rotate(velocity, yaw)), 1e-12)
                self.assertEqual(frames[-1]['elapsed'], 16)
                self.assertTrue(all(not v['armed'] for v in relay.state.values()))

    def test_tracking_limit_accepts_moderate_error_but_override_can_reject(self):
        for limit in (.5, .3):
            flight, relay, log = self.mission(1)
            self.assertEqual(flight.args.tracking_tolerance, .5)
            flight.args.tracking_tolerance = limit

            def events(event, **fields):
                if event == 'phase' and fields['name'] == 'sweep':
                    relay.state[1]['pos'][0] += .4

            log.side_effect = events
            with self.subTest(limit=limit):
                if limit == .5:
                    self.run_mission(flight, relay)
                    self.assertTrue(any(c.args == ('complete',) for c in log.call_args_list))
                else:
                    with self.assertRaisesRegex(gs.FlightError, 'tracking error .*0.30 m limit'):
                        self.run_mission(flight, relay)
                    errors = [c.kwargs for c in log.call_args_list if c.args == ('tracking_error',)]
                    self.assertEqual(len(errors), 1)
                    self.assertGreater(errors[0]['error_m'], .3)

    def test_diamond_argument_speed_limit_and_preview(self):
        args = gs.arguments(['--pattern', 'diamond', '--duration', '16', '--target-systems', '2'])
        self.assertEqual(args.pattern, 'diamond')
        self.assertEqual(args.tracking_tolerance, .5)
        with patch.object(gs, 'Relay') as transport, contextlib.redirect_stdout(io.StringIO()) as output:
            self.assertEqual(gs.main(['--pattern', 'diamond', '--target-systems', '2']), 0)
            transport.assert_not_called()
        self.assertIn('Diamond vertices', output.getvalue())
        for options in (['--pattern', 'unknown'], ['--pattern', 'diamond', '--radius', '.4', '--duration', '10']):
            with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
                gs.arguments(options)

    def test_one_metre_diamond_limits(self):
        args = gs.arguments(['--pattern', 'diamond', '--radius', '1', '--duration', '40', '--max-flight-time', '60'])
        self.assertAlmostEqual(gs.peak_speed(args.pattern, args.radius, args.duration), .2651650429449553)
        self.assertEqual(gs.diamond_sample(20, 40, 1)[0], (-2., 0., 0.))
        for options in (['--pattern', 'diamond', '--radius', '1', '--duration', '20'],
                        ['--pattern', 'diamond', '--radius', '1.01', '--duration', '40', '--max-flight-time', '60'],
                        ['--pattern', 'diamond', '--duration', '41', '--max-flight-time', '60'],
                        ['--pattern', 'circle', '--radius', '1', '--duration', '40', '--max-flight-time', '60']):
            with self.subTest(options=options), contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
                gs.arguments(options)

    def test_one_metre_diamond_missions(self):
        for count in (1, 2, 4):
            with self.subTest(count=count):
                args = gs.arguments(['--target-systems', *map(str, range(1, count + 1)), '--pattern', 'diamond',
                                     '--radius', '1', '--duration', '40', '--hover', '3', '--hold', '2',
                                     '--max-flight-time', '60'])
                relay = SwarmSimRelay(args)
                log = Mock()
                flight = gs.SwarmFlight(relay, args, log)
                self.run_mission(flight, relay)
                frames = [c.kwargs for c in log.call_args_list if c.args == ('trajectory_setpoint',)]
                self.assertEqual(frames[-1]['elapsed'], 40)
                for sid in relay.links:
                    vertex = [f for f in frames if f['system'] == sid and 19.8 <= f['elapsed'] <= 20.2]
                    self.assertTrue(vertex)
                    self.assertGreater(max(math.dist(f['ned'], flight.starts[sid]) for f in vertex), 1.99)
                self.assertTrue(all(not v['armed'] for v in relay.state.values()))

    def test_hover_and_directional_missions_for_one_through_four(self):
        patterns = ('hover', 'forward-return', 'backward-return', 'left-return', 'right-return', 'three-axis')
        for count in range(1, 5):
            for pattern in patterns:
                with self.subTest(count=count, pattern=pattern):
                    flight, relay, log = self.mission(count)
                    flight.args.pattern = pattern
                    self.run_mission(flight, relay)
                    self.assertFalse(any(e[2] == 'trajectory' for e in relay.events))
                    for sid in relay.links:
                        expected = gs.mission_targets(flight.starts[sid], flight.yaws[sid], 1, pattern)
                        # Each group phase has exactly the intended fixed target for every member.
                        phases = [c.kwargs for c in log.call_args_list if c.args == ('phase',) and 'targets' in c.kwargs]
                        self.assertEqual([(p['name'], p['targets'][sid]) for p in phases],
                                         [('hover', flight.starts[sid])] + expected)
                    self.assertTrue(all(not v['armed'] for v in relay.state.values()))

    def test_takeoff_accepts_vertical_state_but_requires_horizontal_before_movement(self):
        for acquire in (True, False):
            flight, relay, log = self.mission(2)
            publish = relay.publish

            def vertical_ground():
                publish()
                for sid, link in relay.links.items():
                    if not acquire or relay.state[sid]['pos'][2] > -.8:
                        link.cache['EKF_STATUS_REPORT'][1].flags = gs.EKF_VERTICAL_REQUIRED

            relay.publish = vertical_ground
            relay.publish()
            with self.subTest(acquire=acquire):
                if acquire:
                    self.run_mission(flight, relay)
                else:
                    with self.assertRaisesRegex(gs.FlightError, 'horizontal position'):
                        self.run_mission(flight, relay)
                    self.assertFalse(any(e[2] in ('setpoint', 'trajectory') for e in relay.events))
                self.assertEqual(sum(e[2] == gs.mav.MAV_CMD_NAV_TAKEOFF for e in relay.events), 2)

    def test_missing_arm_ack_uses_each_observed_armed_state(self):
        flight, relay, _ = self.mission(2)
        relay.drop_acks.update((sid, gs.mav.MAV_CMD_COMPONENT_ARM_DISARM) for sid in (1, 2))
        self.run_mission(flight, relay)
        arms = [e for e in relay.events if e[2] == gs.mav.MAV_CMD_COMPONENT_ARM_DISARM and e[3][0] == 1]
        self.assertEqual(len(arms), 2)
        self.assertTrue(all(e[3][1] == 0 for e in arms))

    def test_explicit_arm_failure_never_retries_or_takes_off(self):
        for result in (gs.mav.MAV_RESULT_DENIED, gs.mav.MAV_RESULT_FAILED, gs.mav.MAV_RESULT_TEMPORARILY_REJECTED):
            flight, relay, _ = self.mission(2)
            relay.reject_arm = 2
            ack = relay.ack

            def reject(sid, command, value=gs.mav.MAV_RESULT_ACCEPTED):
                ack(sid, command, result if sid == 2 and command == gs.mav.MAV_CMD_COMPONENT_ARM_DISARM else value)

            relay.ack = reject
            with self.subTest(result=result), self.assertRaises(gs.FlightError):
                self.run_mission(flight, relay)
            self.assertEqual(sum(e[2] == gs.mav.MAV_CMD_COMPONENT_ARM_DISARM for e in relay.events), 2)
            self.assertFalse(any(e[2] == gs.mav.MAV_CMD_NAV_TAKEOFF for e in relay.events))

    def test_pattern_defaults_preserve_relaxed_fixed_target_arrival(self):
        for pattern in ('hover', 'left-return', 'three-axis'):
            args = gs.arguments(['--pattern', pattern])
            self.assertEqual((args.position_tolerance, args.settle_speed, args.move_timeout), (.3, .4, 30))
        for pattern in ('circle', 'diamond'):
            args = gs.arguments(['--pattern', pattern])
            self.assertEqual((args.position_tolerance, args.settle_speed, args.move_timeout), (.15, .15, 8))
        args = gs.arguments(['--pattern', 'hover', '--position-tolerance', '.1', '--settle-speed', '.2',
                             '--move-timeout', '12'])
        self.assertEqual((args.position_tolerance, args.settle_speed, args.move_timeout), (.1, .2, 12))

    def test_takeover_lands_other_members_only(self):
        flight, relay, log = self.mission(4)

        def events(event, **fields):
            if event == 'phase' and fields['name'] == 'sweep':
                relay.takeover = 2
        log.side_effect = events
        with self.assertRaises(gs.FlightError):
            self.run_mission(flight, relay)
        with patch.object(gs.time, 'monotonic', side_effect=lambda: relay.now):
            flight.abort(gs.FlightError('takeover'))
        landed = {e[1] for e in relay.events if e[2] == gs.mav.MAV_CMD_NAV_LAND}
        self.assertEqual(landed, {1, 3, 4})

    def test_stale_position_aborts_before_next_setpoint(self):
        flight, relay, log = self.mission()

        def events(event, **fields):
            if event == 'phase' and fields['name'] == 'sweep':
                relay.links[2].cache['LOCAL_POSITION_NED'] = (relay.now - 1, relay.links[2].cache['LOCAL_POSITION_NED'][1])
                relay.stale = 2
        log.side_effect = events
        with self.assertRaisesRegex(gs.FlightError, 'position stale'):
            self.run_mission(flight, relay)
        self.assertFalse(any(e[2] == 'trajectory' for e in relay.events))

    def test_position_pause_while_arm_ack_pending_is_allowed(self):
        class ArmingPauseRelay(SwarmSimRelay):
            def __init__(self, args):
                self.pending_arm = None
                self.max_position_age = 0
                super().__init__(args)

            def command(self, sid, comp, cmd, confirmation, *params):
                if cmd == gs.mav.MAV_CMD_COMPONENT_ARM_DISARM and params[0] == 1:
                    self.pending_arm = (self.now + 1, (sid, comp, cmd, confirmation, *params))
                    return
                super().command(sid, comp, cmd, confirmation, *params)

            def publish(self):
                old = self.links[1].cache.get('LOCAL_POSITION_NED')
                super().publish()
                if self.pending_arm and old:
                    self.links[1].cache['LOCAL_POSITION_NED'] = old
                    self.max_position_age = max(self.max_position_age, self.now - old[0])

            def pump(self):
                if self.pending_arm and self.now >= self.pending_arm[0]:
                    command = self.pending_arm[1]
                    self.pending_arm = None
                    super().command(*command)
                super().pump()

        args = gs.arguments(['--target-systems', '1', '--hover', '.5', '--hold', '.5'])
        relay = ArmingPauseRelay(args)
        flight = gs.SwarmFlight(relay, args, Mock())
        self.run_mission(flight, relay)
        self.assertGreater(relay.max_position_age, gs.POSITION_MAX_AGE)
        self.assertTrue(flight.formation_monitoring)
        self.assertFalse(relay.state[1]['armed'])

    def test_position_loss_during_climb_still_aborts(self):
        flight, relay, log = self.mission(1)

        def events(event, **fields):
            if event == 'phase' and fields['name'] == 'takeoff':
                relay.stale = 1
        log.side_effect = events
        with self.assertRaisesRegex(gs.FlightError, 'Local-position telemetry stale'):
            self.run_mission(flight, relay)
        self.assertFalse(flight.formation_monitoring)

    def test_sender_stall_does_not_jump_ahead(self):
        flight, relay, log = self.mission()
        count = 0

        def events(event, **fields):
            nonlocal count
            if event == 'trajectory_setpoint':
                count += 1
                if count == 2:
                    relay.now += .7
        log.side_effect = events
        with self.assertRaisesRegex(gs.FlightError, 'sender delayed'):
            self.run_mission(flight, relay)
        self.assertEqual(len([e for e in relay.events if e[2] == 'trajectory']), 1)

    def test_time_limit_requests_land(self):
        flight, relay, log = self.mission()

        def events(event, **fields):
            if event == 'phase' and fields['name'] == 'sweep':
                flight.flight_started = relay.now - flight.args.max_flight_time
        log.side_effect = events
        with self.assertRaisesRegex(gs.FlightError, 'time limit'):
            self.run_mission(flight, relay)
        with patch.object(gs.time, 'monotonic', side_effect=lambda: relay.now):
            flight.abort(gs.FlightError('time limit'))
        landed = {e[1] for e in relay.events if e[2] == gs.mav.MAV_CMD_NAV_LAND}
        self.assertEqual(landed, {1, 2})

    def test_rejected_arm_never_starts_takeoff(self):
        flight, relay, log = self.mission(4)
        relay.reject_arm = 3
        with self.assertRaisesRegex(gs.FlightError, 'rejected'):
            self.run_mission(flight, relay)
        self.assertFalse(any(e[2] == gs.mav.MAV_CMD_NAV_TAKEOFF for e in relay.events))

    def test_spacing_and_tracking_departures_abort(self):
        for kind in ('separation', 'tracking'):
            flight, relay, log = self.mission()

            def events(event, **fields):
                if event == 'phase' and fields['name'] == 'sweep':
                    if kind == 'separation':
                        relay.state[2]['pos'][0] += 2
                    else:
                        # Common translation preserves spacing but misses the path.
                        for sid, state in relay.state.items():
                            d = gs.rotate((1., 0., 0.), flight.yaws[sid])
                            state['pos'] = [p + v for p, v in zip(state['pos'], d)]
            log.side_effect = events
            with self.subTest(kind=kind), self.assertRaisesRegex(gs.FlightError, kind):
                self.run_mission(flight, relay)


if __name__ == '__main__':
    unittest.main()
