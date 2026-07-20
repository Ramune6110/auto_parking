"""Tests for documentation geometry. Verifies: COORD-EQ-05, PP-EQ-06〜PP-EQ-10."""

from pathlib import Path
import math
import sys
import unittest

import matplotlib.image as mpimg
import numpy as np


THIS_DIR = Path(__file__).resolve().parent
PYTHON_DIR = THIS_DIR.parent
PROJECT_ROOT = PYTHON_DIR.parent
sys.path.insert(0, str(THIS_DIR))
sys.path.insert(0, str(PYTHON_DIR))

from geometry import (  # noqa: E402
    bicycle_turning_radius,
    create_pure_pursuit_geometry,
    exact_bicycle_step,
    tracking_errors,
)
from vehicle_parameters import VEHICLE  # noqa: E402


FIGURE_NAMES = (
    "coordinate_system",
    "tracking_error_projection",
    "path_geometry",
    "path_arc_update",
    "velocity_profile",
    "velocity_interval",
    "pure_pursuit_geometry",
    "pure_pursuit_forward_reverse",
    "stanley_geometry",
    "stanley_reference_points",
    "bicycle_geometry",
    "bicycle_state_update",
    "linear_error_model",
    "linearization_comparison",
    "steering_actuator_flow",
    "steering_response",
    "lqr_feedback_loop",
    "lqr_dimensions",
)


class GeometryTest(unittest.TestCase):
    def test_vehicle_parameters_are_physically_valid(self):
        self.assertGreater(VEHICLE.wheelbase, 0.0)
        self.assertGreater(VEHICLE.width, 0.0)
        self.assertGreater(VEHICLE.length, VEHICLE.wheelbase)
        self.assertLess(VEHICLE.track_width, VEHICLE.width)
        self.assertLess(VEHICLE.cog_from_rear_axle, VEHICLE.wheelbase)

    def test_pure_pursuit_circle_conditions_for_left_and_right_targets(self):
        for alpha_deg in (-40.0, -15.0, 15.0, 40.0):
            geometry = create_pure_pursuit_geometry(
                math.radians(alpha_deg), chord_length=4.2)
            self.assertAlmostEqual(
                np.linalg.norm(geometry.vehicle - geometry.center),
                abs(geometry.radius), places=10)
            self.assertAlmostEqual(
                np.linalg.norm(geometry.target - geometry.center),
                abs(geometry.radius), places=10)
            self.assertAlmostEqual(
                geometry.motion_curvature,
                2.0 * math.sin(geometry.alpha) / geometry.chord_length,
                places=10)

    def test_tracking_error_projection_recovers_components(self):
        yaw = math.radians(31.0)
        tangent = np.array([math.cos(yaw), math.sin(yaw)])
        normal = np.array([-math.sin(yaw), math.cos(yaw)])
        reference = np.array([1.2, -0.4])
        vehicle = reference + 2.3 * tangent - 0.7 * normal
        longitudinal, lateral = tracking_errors(vehicle, reference, yaw)
        self.assertAlmostEqual(longitudinal, 2.3, places=12)
        self.assertAlmostEqual(lateral, -0.7, places=12)

    def test_bicycle_radius_and_exact_step(self):
        steering = math.radians(20.0)
        radius = bicycle_turning_radius(VEHICLE.wheelbase, steering)
        self.assertAlmostEqual(
            1.0 / radius,
            math.tan(steering) / VEHICLE.wheelbase,
            places=12)
        state = np.array([0.0, 0.0, 0.0])
        next_state, yaw_rate = exact_bicycle_step(
            state, 1.0, steering, VEHICLE.wheelbase, 0.5)
        self.assertAlmostEqual(next_state[2], yaw_rate * 0.5, places=12)
        self.assertGreater(next_state[0], 0.0)
        self.assertGreater(next_state[1], 0.0)


class GeneratedFilesTest(unittest.TestCase):
    def test_all_expected_png_and_svg_files_exist(self):
        figure_dir = PROJECT_ROOT / "docs" / "figures"
        for name in FIGURE_NAMES:
            with self.subTest(name=name):
                png = figure_dir / f"{name}.png"
                svg = figure_dir / f"{name}.svg"
                self.assertTrue(png.is_file(), png)
                self.assertTrue(svg.is_file(), svg)
                image = mpimg.imread(png)
                self.assertGreaterEqual(image.shape[0], 700)
                self.assertGreaterEqual(image.shape[1], 900)
                self.assertIn("<svg", svg.read_text(encoding="utf-8")[:1000])


if __name__ == "__main__":
    unittest.main()
