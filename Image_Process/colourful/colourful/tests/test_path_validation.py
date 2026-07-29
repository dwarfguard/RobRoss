import unittest

import context  # noqa: F401  (adds this module's dir to sys.path)

from path_validation import validate_command, validate_painting_paths

A4_CANVAS = {"width_mm": 210.0, "height_mm": 297.0, "origin": "top-left"}


class TestValidateCommand(unittest.TestCase):
    def test_move_to_inside_canvas_is_ok(self):
        command = {"command": "move_to", "label": "ok", "x_mm": 100.0, "y_mm": 100.0}
        errors, warnings = validate_command(command, 0, A4_CANVAS)
        self.assertEqual(errors, [])
        self.assertEqual(warnings, [])

    def test_move_to_outside_canvas_is_an_error(self):
        command = {"command": "move_to", "label": "bad", "x_mm": 250.0, "y_mm": 100.0}
        errors, _ = validate_command(command, 0, A4_CANVAS)
        self.assertEqual(len(errors), 1)
        self.assertIn("outside canvas bounds", errors[0])

    def test_move_to_without_coordinates_is_an_error(self):
        command = {"command": "move_to", "label": "bad"}
        errors, _ = validate_command(command, 0, A4_CANVAS)
        self.assertEqual(len(errors), 1)

    def test_paint_stroke_outside_canvas_is_an_error(self):
        command = {
            "command": "paint_stroke",
            "label": "bad",
            "color": "black",
            "from_mm": [10.0, 10.0],
            "to_mm": [300.0, 10.0],
        }
        errors, _ = validate_command(command, 0, A4_CANVAS)
        self.assertEqual(len(errors), 1)
        self.assertIn("outside canvas bounds", errors[0])

    def test_zero_length_stroke_is_an_error(self):
        command = {
            "command": "paint_stroke",
            "label": "dot",
            "color": "black",
            "from_mm": [10.0, 10.0],
            "to_mm": [10.0, 10.0],
        }
        errors, _ = validate_command(command, 0, A4_CANVAS)
        self.assertEqual(len(errors), 1)
        self.assertIn("zero distance", errors[0])

    def test_unknown_command_is_a_warning_not_an_error(self):
        command = {"command": "wash_brush", "label": "future"}
        errors, warnings = validate_command(command, 0, A4_CANVAS)
        self.assertEqual(errors, [])
        self.assertEqual(len(warnings), 1)

    def test_missing_color_is_a_warning(self):
        command = {"command": "select_tool", "label": "no_color"}
        _, warnings = validate_command(command, 0, A4_CANVAS)
        self.assertEqual(len(warnings), 1)


class TestValidatePaintingPaths(unittest.TestCase):
    def test_missing_commands_list_is_an_error(self):
        result = validate_painting_paths({"canvas": A4_CANVAS})
        self.assertFalse(result["passed"])

    def test_well_formed_paths_pass(self):
        paths = {
            "canvas": A4_CANVAS,
            "commands": [
                {"command": "select_tool", "color": "red"},
                {"command": "dip_paint", "color": "red"},
                {"command": "move_to", "x_mm": 20.0, "y_mm": 20.0},
                {"command": "lower_tool"},
                {
                    "command": "paint_stroke",
                    "color": "red",
                    "from_mm": [20.0, 20.0],
                    "to_mm": [40.0, 20.0],
                },
                {"command": "lift_tool"},
            ],
        }
        result = validate_painting_paths(paths)
        self.assertTrue(result["passed"], result["errors"])
        self.assertEqual(result["warnings"], [])


if __name__ == "__main__":
    unittest.main()
