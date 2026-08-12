import unittest

import context  # noqa: F401

from path_ordering import join_overlapping_strokes


class TestNoBlankConnectors(unittest.TestCase):
    def test_nearby_nonoverlapping_endpoints_are_never_joined(self):
        strokes = [
            ([(0.0, 0.0), (10.0, 0.0)], False),
            ([(10.001, 0.0), (20.0, 0.0)], False),
        ]

        joined, joins = join_overlapping_strokes(strokes, max_turn_degrees=180)

        self.assertEqual(joins, 0)
        self.assertEqual(joined, strokes)


if __name__ == "__main__":
    unittest.main()
