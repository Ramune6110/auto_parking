"""Numerically verifiable geometry. Traceability: COORD-EQ-05, PP-EQ-06〜PP-EQ-10."""

from dataclasses import dataclass
import math
import numpy as np


@dataclass(frozen=True)
class PurePursuitGeometry:
    vehicle: np.ndarray
    target: np.ndarray
    center: np.ndarray
    alpha: float
    chord_length: float
    radius: float
    motion_curvature: float


def create_pure_pursuit_geometry(alpha: float, chord_length: float):
    if abs(math.sin(alpha)) < 1.0e-9 or chord_length <= 0.0:
        raise ValueError("alpha must be nonzero and chord_length positive")
    vehicle = np.array([0.0, 0.0])
    target = chord_length * np.array([math.cos(alpha), math.sin(alpha)])
    radius = chord_length / (2.0 * math.sin(alpha))
    center = np.array([0.0, radius])
    geometry = PurePursuitGeometry(
        vehicle, target, center, alpha, chord_length,
        radius, 1.0 / radius)
    validate_pure_pursuit_geometry(geometry)
    return geometry


def validate_pure_pursuit_geometry(g: PurePursuitGeometry) -> None:
    tolerance = 1.0e-10
    cp = np.linalg.norm(g.vehicle - g.center)
    ct = np.linalg.norm(g.target - g.center)
    chord = np.linalg.norm(g.target - g.vehicle)
    tangent = np.array([1.0, 0.0])
    radius_vector = g.vehicle - g.center
    assert abs(cp - abs(g.radius)) < tolerance
    assert abs(ct - abs(g.radius)) < tolerance
    assert abs(chord - g.chord_length) < tolerance
    assert abs(np.dot(tangent, radius_vector)) < tolerance
    assert abs(g.motion_curvature - 2.0 * math.sin(g.alpha) /
               g.chord_length) < tolerance


def tracking_errors(vehicle, reference, reference_yaw):
    difference = np.asarray(vehicle) - np.asarray(reference)
    tangent = np.array([math.cos(reference_yaw), math.sin(reference_yaw)])
    normal = np.array([-math.sin(reference_yaw), math.cos(reference_yaw)])
    return float(difference @ tangent), float(difference @ normal)


def bicycle_turning_radius(wheelbase: float, steering: float) -> float:
    return wheelbase / math.tan(steering)


def exact_bicycle_step(state, velocity, steering, wheelbase, dt):
    x, y, yaw = state
    yaw_rate = velocity * math.tan(steering) / wheelbase
    next_yaw = yaw + yaw_rate * dt
    if abs(yaw_rate) < 1.0e-10:
        next_x = x + velocity * math.cos(yaw) * dt
        next_y = y + velocity * math.sin(yaw) * dt
    else:
        next_x = x + velocity / yaw_rate * (math.sin(next_yaw) - math.sin(yaw))
        next_y = y - velocity / yaw_rate * (math.cos(next_yaw) - math.cos(yaw))
    return np.array([next_x, next_y, next_yaw]), yaw_rate
