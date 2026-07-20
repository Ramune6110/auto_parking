"""Load canonical vehicle dimensions. Traceability: IMPL-VEHICLE-01, BICYCLE-EQ-03."""

from dataclasses import dataclass
from pathlib import Path
import re


PROJECT_ROOT = Path(__file__).resolve().parents[1]
PARAMETER_HEADER = PROJECT_ROOT / "include" / "core" / "vehicle_parameters.hpp"


def _read_constant(name: str) -> float:
    text = PARAMETER_HEADER.read_text(encoding="utf-8")
    match = re.search(rf"\b{name}\s*=\s*([-+0-9.eE]+)\s*;", text)
    if not match:
        raise ValueError(f"Cannot find {name} in {PARAMETER_HEADER}")
    return float(match.group(1))


@dataclass(frozen=True)
class VehicleParameters:
    wheelbase: float
    width: float
    front_overhang: float
    rear_overhang: float
    cog_from_rear_axle: float
    track_width: float
    max_steering_angle_deg: float

    @property
    def length(self) -> float:
        return self.rear_overhang + self.wheelbase + self.front_overhang


VEHICLE = VehicleParameters(
    wheelbase=_read_constant("kWheelbase"),
    width=_read_constant("kVehicleWidth"),
    front_overhang=_read_constant("kFrontOverhang"),
    rear_overhang=_read_constant("kRearOverhang"),
    cog_from_rear_axle=_read_constant("kCogFromRearAxle"),
    track_width=_read_constant("kTrackWidth"),
    max_steering_angle_deg=_read_constant("kMaxSteeringAngleDeg"),
)
