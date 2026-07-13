import unittest

from context import CONFIGS_DIR  # noqa: F401  (sets up sys.path)

from path_optimizer import (
    chain_polylines,
    optimize_polylines,
    order_polylines,
    polyline_length,
    serpentine_points,
    travel_distance,
)


def poly(points, color="black", label="p"):
    return {"points": [tuple(p) for p in points], "color": color, "label": label}


class TestSerpentinePoints(unittest.TestCase):
    def test_single_row_is_one_segment(self):
        self.assertEqual(
            serpentine_points([5.0], 0.0, 10.0),
            [(0.0, 5.0), (10.0, 5.0)],
        )

    def test_rows_alternate_direction_and_connect(self):
        points = serpentine_points([5.0, 10.0, 15.0], 0.0, 20.0)
        self.assertEqual(
            points,
            [
                (0.0, 5.0), (20.0, 5.0),   # row 1 rightward
                (20.0, 10.0), (0.0, 10.0), # row 2 leftward
                (0.0, 15.0), (20.0, 15.0), # row 3 rightward
            ],
        )
        # Continuous: each point connects to the next, no jumps larger
        # than the rectangle diagonal.
        for a, b in zip(points, points[1:]):
            self.assertLessEqual(abs(a[0] - b[0]) + abs(a[1] - b[1]), 25.0)


class TestChainPolylines(unittest.TestCase):
    def test_touching_lines_merge_into_one(self):
        chains = chain_polylines([
            poly([(0, 0), (10, 0)], label="a"),
            poly([(10, 0), (10, 10)], label="b"),
        ])
        self.assertEqual(len(chains), 1)
        self.assertEqual(chains[0]["points"], [(0, 0), (10, 0), (10, 10)])
        self.assertEqual(chains[0]["label"], "a+b")

    def test_reversed_candidate_still_chains(self):
        chains = chain_polylines([
            poly([(0, 0), (10, 0)]),
            poly([(10, 10), (10, 0)]),  # its END touches our end
        ])
        self.assertEqual(len(chains), 1)
        self.assertEqual(chains[0]["points"], [(0, 0), (10, 0), (10, 10)])

    def test_border_rectangle_becomes_closed_loop(self):
        chains = chain_polylines([
            poly([(0, 0), (10, 0)], label="top"),
            poly([(10, 0), (10, 10)], label="right"),
            poly([(10, 10), (0, 10)], label="bottom"),
            poly([(0, 10), (0, 0)], label="left"),
        ])
        self.assertEqual(len(chains), 1)
        points = chains[0]["points"]
        self.assertEqual(points[0], points[-1])  # closed loop
        self.assertEqual(len(points), 5)

    def test_gap_larger_than_tolerance_does_not_chain(self):
        chains = chain_polylines([
            poly([(0, 0), (10, 0)]),
            poly([(12, 0), (20, 0)]),  # 2 mm gap
        ])
        self.assertEqual(len(chains), 2)

    def test_t_junction_does_not_chain(self):
        # One line ends on the MIDDLE of another: needs a lift, no chain.
        chains = chain_polylines([
            poly([(0, 0), (20, 0)]),
            poly([(10, 0), (10, 10)]),
        ])
        self.assertEqual(len(chains), 2)

    def test_different_colors_do_not_chain(self):
        chains = chain_polylines([
            poly([(0, 0), (10, 0)], color="red"),
            poly([(10, 0), (10, 10)], color="black"),
        ])
        self.assertEqual(len(chains), 2)


class TestOrderPolylines(unittest.TestCase):
    def test_picks_nearest_and_reverses(self):
        far = poly([(100, 100), (200, 100)], label="far")
        # Near, but its END is at the start position: should be drawn
        # reversed, from (1, 0) backwards to (50, 0).
        near_reversed = poly([(50, 0), (1, 0)], label="near")
        ordered, end = order_polylines([far, near_reversed], (0.0, 0.0))
        self.assertEqual([p["label"] for p in ordered], ["near", "far"])
        self.assertEqual(ordered[0]["points"][0], (1, 0))  # reversed
        self.assertEqual(end, (200, 100))

    def test_ordering_never_increases_travel(self):
        polys = [
            poly([(100, 0), (150, 0)], label="c"),
            poly([(0, 0), (50, 0)], label="a"),
            poly([(200, 0), (250, 0)], label="d"),
            poly([(50, 0), (100, 0)], label="b"),
        ]
        start = (0.0, 0.0)
        ordered, _ = order_polylines(list(polys), start)
        self.assertLessEqual(
            travel_distance(ordered, start),
            travel_distance(polys, start),
        )

    def test_optimize_chains_then_orders(self):
        polys = [
            poly([(50, 50), (60, 50)], label="lonely"),
            poly([(0, 0), (10, 0)], label="a"),
            poly([(10, 0), (10, 10)], label="b"),
        ]
        ordered, end = optimize_polylines(polys, (0.0, 0.0))
        self.assertEqual(len(ordered), 2)  # a+b chained
        self.assertEqual(ordered[0]["label"], "a+b")
        self.assertEqual(end, (60, 50))


class TestHelpers(unittest.TestCase):
    def test_polyline_length(self):
        self.assertAlmostEqual(
            polyline_length([(0, 0), (3, 4), (3, 14)]), 15.0
        )

    def test_travel_distance(self):
        polys = [
            poly([(10, 0), (20, 0)]),
            poly([(30, 0), (40, 0)]),
        ]
        self.assertAlmostEqual(travel_distance(polys, (0.0, 0.0)), 20.0)


if __name__ == "__main__":
    unittest.main()
