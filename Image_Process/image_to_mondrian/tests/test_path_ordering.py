import math
import random
import time
import unittest

import context  # noqa: F401

from path_ordering import order_strokes, total_travel_distance


def _linear_scan_order(strokes_data, home_position=(0.0, 0.0)):
    """Reference implementation: the original O(n^2) greedy nearest-neighbor
    linear scan that order_strokes' spatial-hash version must match exactly.
    Kept here as the executable spec for the tie-breaking (ascending stroke
    index, start endpoint before end endpoint)."""
    remaining = list(range(len(strokes_data)))
    ordered = []
    current = home_position
    while remaining:
        best_idx = best_reverse = None
        best_dist = math.inf
        for idx in remaining:
            points, closed = strokes_data[idx]
            d_start = math.dist(current, points[0])
            if d_start < best_dist:
                best_dist, best_idx, best_reverse = d_start, idx, False
            if not closed:
                d_end = math.dist(current, points[-1])
                if d_end < best_dist:
                    best_dist, best_idx, best_reverse = d_end, idx, True
        points, _ = strokes_data[best_idx]
        if best_reverse:
            points = list(reversed(points))
        ordered.append(points)
        current = points[-1]
        remaining.remove(best_idx)
    return ordered


def _random_stroke(rng, grid=None, center=(0.0, 0.0), spread=210.0):
    def point():
        if grid is not None:
            # Snap to a coarse integer grid so many endpoints land at exactly
            # equal distances - this is what exercises the tie-break path.
            return (float(rng.randint(0, grid)), float(rng.randint(0, grid)))
        return (center[0] + rng.uniform(-spread, spread),
                center[1] + rng.uniform(-spread, spread))

    points = [point() for _ in range(rng.randint(2, 5))]
    closed = rng.random() < 0.4
    if closed:
        points = points + [points[0]]
    return (points, closed)


class TestOrderStrokesEquivalence(unittest.TestCase):
    def test_matches_linear_scan_on_random_inputs(self):
        # The spatial-hash ordering must be byte-for-byte identical to the
        # original linear scan on every input, so switching to it never
        # perturbs a generated (or committed reference) painting_paths.json.
        # The parameter sweep deliberately includes tiny clusters placed far
        # from a far-away home, which drives the outside-the-bounds fallback.
        homes = [(0.0, 0.0), (5.0, 5.0), (-50.0, 400.0), (105.0, 148.0), (1e6, 1e6)]
        centers = [(0.0, 0.0), (200.0, 200.0), (1000.0, -500.0)]
        spreads = [210.0, 1.0, 1e-4]
        for trial in range(800):
            rng = random.Random(trial)
            n = rng.randint(0, 40)
            grid = rng.choice([None, 3, 5, 8])  # coarse grids force exact ties
            center = rng.choice(centers)
            spread = rng.choice(spreads)
            strokes = [_random_stroke(rng, grid, center, spread) for _ in range(n)]
            home = rng.choice(homes)
            with self.subTest(trial=trial, n=n, grid=grid, center=center,
                              spread=spread, home=home):
                self.assertEqual(
                    order_strokes(strokes, home),
                    _linear_scan_order(list(strokes), home),
                )

    def test_distant_tight_cluster_terminates_and_matches(self):
        # Regression: a tiny cluster (sub-millimeter span) far from home_position
        # makes cell_size tiny, so the old code walked millions of empty rings
        # from the home cell out to the cluster and effectively hung. It must
        # now finish promptly and still match the linear scan exactly.
        strokes = []
        for i in range(60):
            x = 200.0 + i * 1e-5
            y = 200.0 + i * 1e-5
            strokes.append(([(x, y), (x + 1e-5, y + 1e-5)], False))
        start = time.perf_counter()
        ordered = order_strokes(strokes, home_position=(0.0, 0.0))
        elapsed = time.perf_counter() - start
        self.assertEqual(ordered, _linear_scan_order(list(strokes), (0.0, 0.0)))
        # Generous bound: the fix makes this sub-millisecond; the pre-fix code
        # could not finish in minutes. Guards against a silent reintroduction.
        self.assertLess(elapsed, 1.0)

    def test_empty_input_returns_empty(self):
        self.assertEqual(order_strokes([]), [])

    def test_single_stroke_is_returned_as_is(self):
        stroke = ([(10.0, 10.0), (20.0, 20.0)], False)
        self.assertEqual(order_strokes([stroke], home_position=(0.0, 0.0)),
                         [[(10.0, 10.0), (20.0, 20.0)]])

    def test_open_stroke_walked_in_reverse_when_far_end_is_closer(self):
        # Pen at origin; the stroke's *end* is nearer than its start, so it
        # should be drawn reversed (end-first).
        stroke = ([(100.0, 100.0), (5.0, 5.0)], False)
        ordered = order_strokes([stroke], home_position=(0.0, 0.0))
        self.assertEqual(ordered, [[(5.0, 5.0), (100.0, 100.0)]])

    def test_closed_stroke_keeps_its_start_point(self):
        # Closed loops are never reversed even if the far end is closer.
        loop = ([(100.0, 100.0), (150.0, 100.0), (150.0, 150.0), (100.0, 100.0)], True)
        ordered = order_strokes([loop], home_position=(0.0, 0.0))
        self.assertEqual(ordered, [[(100.0, 100.0), (150.0, 100.0), (150.0, 150.0), (100.0, 100.0)]])

    def test_ordering_reduces_travel_versus_input_order(self):
        rng = random.Random(7)
        strokes = [([(rng.uniform(0, 210), rng.uniform(0, 297)),
                     (rng.uniform(0, 210), rng.uniform(0, 297))], False)
                   for _ in range(200)]
        home = (0.0, 0.0)
        baseline = total_travel_distance([p for p, _ in strokes], home)
        optimized = total_travel_distance(order_strokes(strokes, home), home)
        self.assertLess(optimized, baseline)


if __name__ == "__main__":
    unittest.main()
