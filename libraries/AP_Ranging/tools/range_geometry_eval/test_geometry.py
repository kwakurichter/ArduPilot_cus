#!/usr/bin/env python3
# AP_FLAKE8_CLEAN
import itertools
import unittest

import numpy as np

from geometry import leave_one_out, pairwise_violation, plane, sphere_intersections


class GeometryTests(unittest.TestCase):
    def setUp(self):
        self.anchors = np.array([[-1., -1., -.8], [-1., 1., -.8], [1., -1., -.8], [1., 1., -.8]])
        self.point = np.array([.2, -.3, .1])
        self.ranges = np.linalg.norm(self.anchors - self.point, axis=1)

    def test_exact_and_mirror(self):
        for indices in itertools.combinations(range(4), 3):
            a = self.anchors[list(indices)]
            r = self.ranges[list(indices)]
            solutions = sphere_intersections(a, r)
            self.assertEqual(len(solutions), 2)
            self.assertLess(min(np.linalg.norm(s - self.point) for s in solutions), 1.e-9)
            for solution in solutions:
                np.testing.assert_allclose(np.linalg.norm(a - solution, axis=1), r, atol=1.e-9)

    def test_tangent(self):
        a = self.anchors[:3]
        p = np.array([0., 0., -.8])
        solutions = sphere_intersections(a, np.linalg.norm(a - p, axis=1))
        for solution in solutions:
            np.testing.assert_allclose(solution, p, atol=1.e-7)

    def test_infeasible_collinear_duplicate(self):
        self.assertEqual(sphere_intersections(self.anchors[:3], [.1, .1, .1]), [])
        self.assertEqual(sphere_intersections([[0, 0, 0], [1, 0, 0], [2, 0, 0]], [2, 2, 2]), [])
        self.assertEqual(sphere_intersections([[0, 0, 0], [0, 0, 0], [0, 1, 0]], [2, 2, 2]), [])

    def test_invalid(self):
        for r in [[0, 1, 1], [-1, 1, 1], [np.nan, 1, 1], [np.inf, 1, 1]]:
            with self.assertRaises(ValueError):
                sphere_intersections(self.anchors[:3], r)
        with self.assertRaises(ValueError):
            sphere_intersections(self.anchors, self.ranges)

    def test_rigid_transform_and_permutation(self):
        rot = np.array([[0., -1., 0.], [1., 0., 0.], [0., 0., 1.]])
        origin = np.array([1000., -2000., 55.])
        a = self.anchors[:3] @ rot.T + origin
        target = self.point @ rot.T + origin
        for order in itertools.permutations(range(3)):
            s = sphere_intersections(a[list(order)], self.ranges[list(order)])
            self.assertLess(min(np.linalg.norm(p - target) for p in s), 1.e-8)

    def test_four_ranges_do_not_isolate_positive_fault(self):
        r = np.linalg.norm(self.anchors, axis=1)
        r[3] += .3
        hypotheses = leave_one_out(self.anchors, r)
        self.assertTrue(all(hypotheses))
        for candidates in hypotheses:
            self.assertTrue(all(p['kept_rms_m'] < 1.e-9 for p in candidates))
        positive = [i for i, candidates in enumerate(hypotheses) if any(p['held_out_m'] > .1 for p in candidates)]
        self.assertGreaterEqual(len(positive), 2)
        self.assertEqual(pairwise_violation(self.anchors, r), 0)

    def test_known_side_still_leaves_multiple_positive_faults(self):
        ranges = np.linalg.norm(self.anchors, axis=1)
        ranges[3] += .3
        center, normal, _ = plane(self.anchors)
        hypotheses = leave_one_out(self.anchors, ranges)
        possible = [i for i, candidates in enumerate(hypotheses)
                    if any(p['held_out_m'] > .1 and np.dot(np.array(p['position']) - center, normal) > 0
                           for p in candidates)]
        self.assertGreaterEqual(len(possible), 2)

    def test_triangle_bounds(self):
        self.assertEqual(pairwise_violation(self.anchors, self.ranges), 0)
        r = self.ranges.copy()
        r[0] += 10
        self.assertGreater(pairwise_violation(self.anchors, r), 5)

    def test_motion_does_not_create_geometric_inconsistency(self):
        for point in [[0, 0, 0], [0, -.9, 0], [0, .9, 0], [-.9, 0, 0], [.9, 0, 0]]:
            r = np.linalg.norm(self.anchors - point, axis=1)
            self.assertEqual(pairwise_violation(self.anchors, r), 0)
            for candidates in leave_one_out(self.anchors, r):
                self.assertLess(min(abs(p['held_out_m']) for p in candidates), 1.e-9)

    def test_plane_orientation(self):
        center, normal, singular = plane(self.anchors)
        self.assertGreater(normal[2], 0)
        self.assertLess(singular[-1], 1.e-9)
        self.assertGreater(np.dot(self.point - center, normal), 0)

    def test_fifth_anchor_distinguishes_counterexample(self):
        # A synthetic example, not a universal guarantee or a real-data result.
        a = np.vstack([self.anchors, [0.3, -.4, -2.2]])
        r = np.linalg.norm(a, axis=1)
        r[3] += .3
        counts = []
        for omitted in range(5):
            keep = [i for i in range(5) if i != omitted]
            errors = []
            for triplet in itertools.combinations(keep, 3):
                for p in sphere_intersections(a[list(triplet)], r[list(triplet)]):
                    errors.append(np.max(np.abs(np.linalg.norm(a[keep] - p, axis=1) - r[keep])))
            counts.append(min(errors))
        self.assertLess(counts[3], 1.e-9)
        self.assertTrue(all(error > .01 for i, error in enumerate(counts) if i != 3))


if __name__ == '__main__':
    unittest.main()
