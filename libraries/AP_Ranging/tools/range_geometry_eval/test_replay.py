#!/usr/bin/env python3
# AP_FLAKE8_CLEAN
import unittest

from replay import causal_rows
from log_reader import unwrap_ms


def event(arrival, acquired, anchor=1, session=1, distance=2.):
    return {'arrival': arrival, 'time': acquired, 'id': anchor, 'session': session, 'range': distance}


class ReplayTests(unittest.TestCase):
    def rows(self, events):
        return list(causal_rows(events, [1, 3, 6, 8], step_ms=1))

    def initial(self):
        return [event(0, 0, i) for i in [1, 3, 6, 8]]

    def test_no_future_delivery_and_expires_at_300(self):
        events = self.initial() + [event(301, 301, 1)]
        rows = self.rows(events)
        self.assertTrue(rows[299]['ready'])
        self.assertFalse(rows[300]['ready'])
        self.assertFalse(rows[301]['ready'])

    def test_uint32_unwrap(self):
        wrap = 1 << 32
        self.assertEqual(unwrap_ms(wrap - 20, wrap + 10), wrap - 20)
        self.assertEqual(unwrap_ms(20, wrap + 30), wrap + 20)
        self.assertEqual(unwrap_ms(20, 30), 20)
        self.assertEqual(unwrap_ms(40, 30), 40)

    def test_spread_boundary(self):
        rows = self.rows(self.initial() + [event(150, 150), event(151, 151)])
        self.assertTrue(rows[150]['ready'])
        self.assertFalse(rows[151]['ready'])

    def test_invalid_new_session_clears_previous_sample(self):
        rows = self.rows(self.initial() + [event(1, 1, session=2, distance=float('nan'))])
        self.assertFalse(rows[-1]['ready'])

    def test_reordered_duplicate_invalid_and_future_samples(self):
        events = self.initial() + [event(10, 10, distance=3), event(11, 9, distance=4),
                                   event(12, 10, distance=5), event(13, 14, distance=6),
                                   event(14, 14, distance=-1), event(15, 15, distance=float('inf'))]
        self.assertEqual(self.rows(events)[-1]['ranges'], [3, 2, 2, 2])

    def test_new_session_allows_restarted_acquisition_counter(self):
        rows = self.rows(self.initial() + [event(10, 10, distance=3), event(11, 1, session=2, distance=4)])
        self.assertEqual(rows[-1]['ranges'][0], 4)


if __name__ == '__main__':
    unittest.main()
