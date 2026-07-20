"""Load and draw parking-space context. Traceability: IMPL-PARKING-ENV-01."""

from pathlib import Path

import numpy as np
from matplotlib.patches import Rectangle


def load_parking_environment(output_directory: Path):
    path = output_directory / "parking_environment.csv"
    if not path.is_file():
        return None
    return np.atleast_1d(
        np.genfromtxt(path, delimiter=",", names=True, dtype=None, encoding="utf-8")
    )


def draw_parking_environment(axis, environment):
    """Draw parking bay, parked obstacles, and the three important poses."""
    if environment is None:
        return
    for row in environment:
        record = str(row["record"])
        if record in {"parking_space", "obstacle"}:
            is_space = record == "parking_space"
            rectangle = Rectangle(
                (float(row["min_x"]), float(row["min_y"])),
                float(row["max_x"] - row["min_x"]),
                float(row["max_y"] - row["min_y"]),
                facecolor="none" if is_space else "#777777",
                edgecolor="#333333",
                linewidth=2.0,
                hatch="//" if is_space else None,
                alpha=0.9 if is_space else 0.75,
                zorder=0,
            )
            axis.add_patch(rectangle)
        elif record.endswith("_point"):
            styles = {
                "initial_point": ("#009E73", "Initial point"),
                "reversal_point": ("#E69F00", "Reversal point"),
                "target_point": ("#D55E00", "Target point"),
            }
            color, label = styles[record]
            axis.scatter(row["x"], row["y"], s=70, color=color,
                         edgecolor="black", label=label, zorder=7)

