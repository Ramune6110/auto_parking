"""Generate the geometric and control-system figures used by docs.

Traceability: IMPL-FIGURE-01 and the equation IDs cited by each target document.

Run from the project root:
    python python/doc_figures/generate_doc_figures.py

Every figure is saved as both a high-resolution PNG for Markdown preview and
an SVG for future PDF generation. Geometry is computed from the same symbols
used in the accompanying derivations instead of being positioned by eye.
"""

from pathlib import Path
import sys

import matplotlib.pyplot as plt
import numpy as np
from matplotlib import patches, transforms

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from vehicle_parameters import VEHICLE
from geometry import (
    bicycle_turning_radius,
    create_pure_pursuit_geometry,
    exact_bicycle_step,
    tracking_errors,
)


PROJECT_ROOT = Path(__file__).resolve().parents[2]
OUTPUT_DIR = PROJECT_ROOT / "docs" / "figures"
BLUE = "#1769aa"
RED = "#cf3f3f"
GREEN = "#238636"
GRAY = "#62676d"
LIGHT_BLUE = "#edf5fc"
LIGHT_GRAY = "#f3f4f6"


def configure_style() -> None:
    """Apply a consistent, printable style to every documentation figure."""
    plt.rcParams.update({
        "font.family": "sans-serif",
        "font.sans-serif": ["Yu Gothic", "Meiryo", "DejaVu Sans"],
        "font.size": 12,
        "axes.unicode_minus": False,
        "mathtext.fontset": "dejavusans",
        "figure.facecolor": "white",
        "savefig.facecolor": "white",
    })


def save_figure(fig: plt.Figure, name: str) -> None:
    """Save one source figure in raster and vector formats."""
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    fig.savefig(OUTPUT_DIR / f"{name}.png", dpi=220, bbox_inches="tight")
    fig.savefig(OUTPUT_DIR / f"{name}.svg", bbox_inches="tight")
    plt.close(fig)


def setup_geometry_axes(figsize=(9.2, 5.8)):
    fig, ax = plt.subplots(figsize=figsize, constrained_layout=True)
    ax.set_aspect("equal", adjustable="box")
    ax.axis("off")
    return fig, ax


def arrow(ax, start, end, color=BLUE, width=2.0, mutation=14, **kwargs):
    patch = patches.FancyArrowPatch(
        start, end, arrowstyle="-|>", mutation_scale=mutation,
        linewidth=width, color=color, shrinkA=0, shrinkB=0, **kwargs)
    ax.add_patch(patch)
    return patch


def draw_vehicle(ax, center, yaw, length=1.15, width=0.58,
                 steering=0.0, label=None, alpha=1.0):
    """Draw a compact top-view vehicle with steerable front wheels."""
    x, y = center
    transform = transforms.Affine2D().rotate(yaw).translate(x, y) + ax.transData
    body = patches.FancyBboxPatch(
        (-0.38 * length, -width / 2), length, width,
        boxstyle="round,pad=0.02,rounding_size=0.08",
        facecolor="#d9dde1", edgecolor="#444", linewidth=1.4,
        alpha=alpha, transform=transform, zorder=5)
    ax.add_patch(body)
    rear_x, front_x = -0.20 * length, 0.46 * length
    wheel_length, wheel_width = 0.30 * length, 0.10
    for axle_x, wheel_angle in ((rear_x, 0.0), (front_x, steering)):
        for wheel_y in (-0.40 * width, 0.40 * width):
            wheel_transform = (
                transforms.Affine2D()
                .rotate_around(axle_x, wheel_y, wheel_angle)
                .rotate(yaw).translate(x, y) + ax.transData)
            wheel = patches.Rectangle(
                (axle_x - wheel_length / 2, wheel_y - wheel_width / 2),
                wheel_length, wheel_width, facecolor="#333", edgecolor="black",
                transform=wheel_transform, zorder=7)
            ax.add_patch(wheel)
    arrow(ax, (x, y),
          (x + 0.95 * length * np.cos(yaw), y + 0.95 * length * np.sin(yaw)),
          color=BLUE, width=1.8, mutation=12, zorder=8)
    ax.plot(x, y, "o", color=RED, ms=5, zorder=9)
    if label:
        ax.text(x, y + 0.52 * width, label, ha="center", va="bottom", zorder=10)


def draw_vehicle_from_rear(ax, rear, yaw, steering=0.0, alpha=1.0):
    """Draw the canonical vehicle at its state position (rear axle center)."""
    rear = np.asarray(rear)
    transform = transforms.Affine2D().rotate(yaw).translate(*rear) + ax.transData
    half_width = VEHICLE.width / 2.0
    vertices = np.array([
        [-VEHICLE.rear_overhang, -0.86 * half_width],
        [-0.15, -half_width],
        [VEHICLE.wheelbase + 0.65 * VEHICLE.front_overhang, -half_width],
        [VEHICLE.wheelbase + VEHICLE.front_overhang, -0.72 * half_width],
        [VEHICLE.wheelbase + VEHICLE.front_overhang, 0.72 * half_width],
        [VEHICLE.wheelbase + 0.65 * VEHICLE.front_overhang, half_width],
        [-0.15, half_width],
        [-VEHICLE.rear_overhang, 0.86 * half_width],
    ])
    ax.add_patch(patches.Polygon(vertices, closed=True, facecolor="#d9dde1",
                                 edgecolor="#444", lw=1.5, alpha=alpha,
                                 transform=transform, zorder=5))
    wheel_length, wheel_width = 0.62, 0.18
    for axle_x, wheel_angle in ((0.0, 0.0), (VEHICLE.wheelbase, steering)):
        for wheel_y in (-VEHICLE.track_width / 2, VEHICLE.track_width / 2):
            wheel_transform = (transforms.Affine2D()
                .rotate_around(axle_x, wheel_y, wheel_angle)
                .rotate(yaw).translate(*rear) + ax.transData)
            ax.add_patch(patches.Rectangle(
                (axle_x-wheel_length/2, wheel_y-wheel_width/2),
                wheel_length, wheel_width, facecolor="#333", edgecolor="black",
                transform=wheel_transform, zorder=7, alpha=alpha))
    forward = np.array([np.cos(yaw), np.sin(yaw)])
    cog = rear + VEHICLE.cog_from_rear_axle * forward
    front = rear + VEHICLE.wheelbase * forward
    ax.scatter(*rear, color="#7a3db8", s=32, zorder=9)
    ax.scatter(*cog, color=RED, s=32, zorder=9)
    ax.scatter(*front, color=GREEN, s=32, zorder=9)


def draw_angle(ax, center, radius, angle1, angle2, text, color=BLUE):
    """Draw an angle arc where angles are specified in radians."""
    theta1, theta2 = np.degrees([angle1, angle2])
    if theta2 < theta1:
        theta1, theta2 = theta2, theta1
    ax.add_patch(patches.Arc(
        center, 2 * radius, 2 * radius, theta1=theta1, theta2=theta2,
        color=color, linewidth=2.0, zorder=9))
    middle = 0.5 * (angle1 + angle2)
    ax.text(center[0] + 1.25 * radius * np.cos(middle),
            center[1] + 1.25 * radius * np.sin(middle), text,
            color=color, ha="center", va="center", zorder=10)


def pure_pursuit_geometry() -> None:
    fig, ax = setup_geometry_axes()
    alpha = np.deg2rad(32.0)
    chord_length = 5.2
    geometry = create_pure_pursuit_geometry(alpha, chord_length)
    radius = geometry.radius
    vehicle = geometry.vehicle
    target = geometry.target
    center = geometry.center

    theta_vehicle = -np.pi / 2.0
    theta_target = np.arctan2(target[1] - center[1], target[0] - center[0])
    theta = np.linspace(theta_vehicle, theta_target, 160)
    arc = center[:, None] + radius * np.vstack((np.cos(theta), np.sin(theta)))

    # A smooth target path is shown only to explain how the look-ahead point is selected.
    x_path = np.linspace(-1.0, 7.0, 240)
    y_path = 0.10 * (x_path + 1.0) ** 1.65
    ax.plot(x_path, y_path, "--", color="#92979d", lw=1.8,
            label="目標経路")
    ax.plot(arc[0], arc[1], color=BLUE, lw=3.2, label="追従円弧")
    ax.plot([vehicle[0], target[0]], [vehicle[1], target[1]],
            color=RED, lw=2.4)
    ax.plot([center[0], vehicle[0]], [center[1], vehicle[1]],
            "--", color=GRAY, lw=1.5)
    ax.plot([center[0], target[0]], [center[1], target[1]],
            "--", color=GRAY, lw=1.5)
    ax.scatter(*center, s=45, color=GRAY, zorder=9)
    ax.scatter(*target, s=60, color=GREEN, zorder=10)
    draw_vehicle(ax, vehicle, yaw=0.0, steering=0.35,
                 length=1.35, width=0.62)
    arrow(ax, vehicle, (1.65, 0.0), color=BLUE, width=2.1)
    draw_angle(ax, vehicle, 1.10, 0.0, alpha, r"$\alpha$")

    ax.text(center[0] - 0.18, center[1] + 0.35, "旋回中心 $C$", ha="right")
    ax.text(target[0] + 0.18, target[1] + 0.15, "注視点 $T$", color=GREEN)
    ax.text(0.55 * target[0], 0.55 * target[1] + 0.20, "$L$", color=RED)
    ax.text(-0.28, 0.5 * radius, "$R$", ha="right", color=GRAY)
    ax.text(0.88, -0.55, r"進行方向 $\psi_m$")
    ax.text(3.55, radius + 0.10, "$CP=CT=R$", color=GRAY)
    ax.text(2.0, -1.15,
            r"$L=2R\sin\alpha\quad\Rightarrow\quad"
            r"\kappa_m=\dfrac{2\sin\alpha}{L}$",
            bbox=dict(boxstyle="round,pad=0.35", fc=LIGHT_BLUE, ec="#9ab9d4"))
    ax.legend(loc="upper right", frameon=False)
    ax.set_xlim(-1.3, 7.4)
    ax.set_ylim(-1.6, radius + 0.8)
    save_figure(fig, "pure_pursuit_geometry")


def stanley_geometry() -> None:
    fig, ax = setup_geometry_axes()
    reference_yaw = np.deg2rad(12.0)
    yaw_error = np.deg2rad(18.0)
    vehicle_yaw = reference_yaw - yaw_error
    tangent = np.array([np.cos(reference_yaw), np.sin(reference_yaw)])
    normal = np.array([-np.sin(reference_yaw), np.cos(reference_yaw)])
    nearest = np.array([0.0, 0.0])
    lateral_error = 1.55
    vehicle = nearest + lateral_error * normal

    u = np.linspace(-4.0, 5.0, 260)
    path = nearest[:, None] + tangent[:, None] * u + normal[:, None] * (0.018 * u**2)
    ax.plot(path[0], path[1], color=GRAY, lw=3.0, label="目標経路")
    ax.scatter(*nearest, s=55, color=GREEN, zorder=10)
    ax.plot([nearest[0], vehicle[0]], [nearest[1], vehicle[1]],
            color=RED, lw=2.6)
    arrow(ax, nearest, nearest + 2.15 * tangent, color=GREEN, width=2.0)
    arrow(ax, vehicle, vehicle + 2.15 * np.array(
        [np.cos(vehicle_yaw), np.sin(vehicle_yaw)]), color=BLUE, width=2.0)
    draw_vehicle(ax, vehicle, vehicle_yaw, steering=-0.25,
                 length=1.25, width=0.60)
    draw_angle(ax, vehicle, 1.05, vehicle_yaw, reference_yaw,
               r"$e_\psi$", color=BLUE)

    midpoint = 0.5 * (nearest + vehicle)
    ax.text(*(midpoint + 0.18 * tangent), r"$e_y>0$", color=RED)
    ax.text(*(nearest - 0.45 * normal), "最近傍点 $R$", ha="center")
    ax.text(*(nearest + 2.2 * tangent - 0.25 * normal),
            r"参照方位 $\psi_r$", color=GREEN, ha="center")
    ax.text(*(vehicle + 1.7 * np.array([np.cos(vehicle_yaw),
                                        np.sin(vehicle_yaw)]) - 0.42 * normal),
            r"車両方位 $\psi$", color=BLUE, ha="center", va="top")
    ax.text(-3.55, -1.35,
            r"$\delta_y=-\arctan\!\left(\dfrac{k_y e_y}"
            r"{|v_r|+v_{\mathrm{soft}}}\right)$",
            bbox=dict(boxstyle="round,pad=0.35", fc=LIGHT_BLUE, ec="#9ab9d4"))
    ax.annotate("経路側へ向ける補正", xy=vehicle + 0.65 * tangent,
                xytext=(2.3, 2.65), color=RED,
                arrowprops=dict(arrowstyle="->", color=RED, lw=1.6))
    ax.legend(loc="upper left", frameon=False)
    ax.set_xlim(-4.3, 5.1)
    ax.set_ylim(-1.8, 3.3)
    save_figure(fig, "stanley_geometry")


def bicycle_geometry() -> None:
    fig, ax = setup_geometry_axes()
    wheelbase = VEHICLE.wheelbase
    steering = np.deg2rad(25.0)
    radius = bicycle_turning_radius(wheelbase, steering)
    rear = np.array([0.0, 0.0])
    front = np.array([wheelbase, 0.0])
    icr = np.array([0.0, radius])

    draw_vehicle_from_rear(ax, rear, yaw=0.0, steering=steering)
    ax.plot([rear[0], icr[0]], [rear[1], icr[1]], "--", color=GRAY, lw=1.6)
    ax.plot([front[0], icr[0]], [front[1], icr[1]], "--", color=GRAY, lw=1.6)
    ax.plot([rear[0], front[0]], [rear[1], front[1]], color=BLUE, lw=2.1)
    ax.scatter(*icr, s=55, color=RED, zorder=10)
    ax.scatter(*rear, s=40, color=GREEN, zorder=10)
    ax.scatter(*front, s=40, color=GREEN, zorder=10)
    draw_angle(ax, front, 0.85, 0.0, steering, r"$\delta$")
    ax.text(-0.18, radius + 0.35, "瞬間旋回中心 ICR", ha="right")
    ax.text(-0.22, radius / 2, "$R$", color=GRAY, ha="right")
    ax.text(wheelbase / 2, -0.42, "$L$", color=BLUE, ha="center")
    ax.text(-0.05, -1.10, "後輪車軸中心", ha="center", va="top")
    ax.text(wheelbase + 0.10, -1.10, "前輪車軸中心", ha="center", va="top")
    ax.text(0.45, radius + 0.12,
            r"$\tan\delta=\dfrac{L}{R}\quad\Rightarrow\quad"
            r"\kappa=\dfrac{1}{R}=\dfrac{\tan\delta}{L}$",
            bbox=dict(boxstyle="round,pad=0.35", fc=LIGHT_BLUE, ec="#9ab9d4"))
    ax.set_xlim(-1.8, 5.0)
    ax.set_ylim(-1.65, radius + 1.0)
    save_figure(fig, "bicycle_geometry")


def linear_error_model() -> None:
    fig, ax = setup_geometry_axes()
    reference_yaw = np.deg2rad(8.0)
    vehicle_yaw = np.deg2rad(-10.0)
    tangent = np.array([np.cos(reference_yaw), np.sin(reference_yaw)])
    normal = np.array([-np.sin(reference_yaw), np.cos(reference_yaw)])
    reference = np.array([0.0, 0.0])
    vehicle = reference + 1.45 * normal
    u = np.linspace(-4.0, 4.5, 200)
    path = reference[:, None] + tangent[:, None] * u
    ax.plot(path[0], path[1], color=GRAY, lw=3.0, label="参照経路")
    ax.plot([reference[0], vehicle[0]], [reference[1], vehicle[1]],
            color=RED, lw=2.6)
    arrow(ax, reference, reference + 2.0 * tangent, color=GREEN, width=2.0)
    arrow(ax, vehicle, vehicle + 2.0 * np.array(
        [np.cos(vehicle_yaw), np.sin(vehicle_yaw)]), color=BLUE, width=2.0)
    draw_vehicle(ax, vehicle, vehicle_yaw, length=1.25, width=0.58)
    draw_angle(ax, vehicle, 1.0, vehicle_yaw, reference_yaw,
               r"$e_\psi=\psi_{\mathrm{ref}}-\psi$")
    ax.text(*(0.52 * (reference + vehicle) + 0.18 * tangent),
            r"$e_y>0$", color=RED)
    ax.text(*(reference + 1.8 * tangent - 0.28 * normal),
            r"$\psi_{\mathrm{ref}}$", color=GREEN)
    ax.text(*(vehicle + 1.65 * np.array([np.cos(vehicle_yaw),
                                         np.sin(vehicle_yaw)]) + 0.20 * normal),
            r"$\psi$", color=BLUE)
    ax.text(-2.7, -1.35,
            r"小さい誤差では $\sin(e_\psi)\approx e_\psi$ と一次近似",
            bbox=dict(boxstyle="round,pad=0.35", fc=LIGHT_BLUE, ec="#9ab9d4"))
    ax.legend(loc="upper left", frameon=False)
    ax.set_xlim(-4.2, 4.7)
    ax.set_ylim(-1.7, 2.7)
    save_figure(fig, "linear_error_model")


def add_block(ax, xy, size, title, subtitle="", color=LIGHT_BLUE):
    x, y = xy
    width, height = size
    box = patches.FancyBboxPatch(
        (x, y), width, height, boxstyle="round,pad=0.03,rounding_size=0.04",
        facecolor=color, edgecolor="#7398b7", linewidth=1.5)
    ax.add_patch(box)
    ax.text(x + width / 2, y + 0.60 * height, title,
            ha="center", va="center", fontsize=12)
    if subtitle:
        ax.text(x + width / 2, y + 0.29 * height, subtitle,
                ha="center", va="center", fontsize=10, color=GRAY)
    return box


def steering_actuator_flow() -> None:
    fig, ax = plt.subplots(figsize=(11.0, 3.5), constrained_layout=True)
    ax.set_xlim(0, 12)
    ax.set_ylim(0, 4)
    ax.axis("off")
    blocks = [
        (0.25, "要求舵角", r"$\delta_{\mathrm{cmd}}$"),
        (2.65, "最大舵角制約", "式 (2)"),
        (5.05, "一次遅れ", "式 (3)～式 (5)"),
        (7.45, "操舵速度制約", "式 (6)～式 (8)"),
        (9.85, "実舵角", r"$\delta$"),
    ]
    for x, title, subtitle in blocks:
        add_block(ax, (x, 1.65), (1.9, 1.15), title, subtitle)
    for left, right in zip(blocks[:-1], blocks[1:]):
        arrow(ax, (left[0] + 1.9, 2.22), (right[0], 2.22), width=1.9)
    ax.text(6.0, 0.75,
            "制御器の要求値と、車両が実現できる舵角を分離する",
            ha="center", fontsize=13,
            bbox=dict(boxstyle="round,pad=0.35", fc=LIGHT_GRAY, ec="#b4b7ba"))
    save_figure(fig, "steering_actuator_flow")


def lqr_feedback_loop() -> None:
    fig, ax = plt.subplots(figsize=(10.5, 5.2), constrained_layout=True)
    ax.set_xlim(0, 12)
    ax.set_ylim(0, 6)
    ax.axis("off")
    add_block(ax, (0.35, 3.65), (1.75, 1.15), "目標軌道", r"$\mathbf{x}_{\mathrm{ref}}$")
    summing = patches.Circle((2.75, 4.23), 0.33, facecolor="white",
                             edgecolor="#7398b7", linewidth=1.5)
    ax.add_patch(summing)
    ax.text(2.75, 4.23, "$-$", ha="center", va="center", fontsize=17)
    add_block(ax, (3.55, 3.65), (2.05, 1.15), "LQRゲイン", r"$u=-K\mathbf{x}$")
    add_block(ax, (6.45, 3.65), (2.05, 1.15), "車両モデル", r"$x_{k+1}=Ax_k+Bu_k$")
    add_block(ax, (9.35, 3.65), (1.75, 1.15), "車両状態", r"$\mathbf{x}$")
    arrow(ax, (2.10, 4.23), (2.42, 4.23), width=1.9)
    arrow(ax, (3.08, 4.23), (3.55, 4.23), width=1.9)
    arrow(ax, (5.60, 4.23), (6.45, 4.23), width=1.9)
    arrow(ax, (8.50, 4.23), (9.35, 4.23), width=1.9)
    # Route the feedback below the blocks so it never crosses block labels.
    ax.plot([10.22, 10.22, 2.75], [3.65, 3.10, 3.10],
            color=BLUE, linewidth=1.9)
    arrow(ax, (2.75, 3.10), (2.75, 3.90), width=1.9)
    ax.text(6.5, 2.75, "状態を戻して次周期の誤差を計算", ha="center", color=BLUE)
    add_block(ax, (3.35, 0.65), (5.3, 1.05),
              "評価関数でゲインを決める",
              r"$Q$: 誤差の重み　　$R$: 操舵量の重み", color=LIGHT_GRAY)
    arrow(ax, (6.0, 1.70), (4.58, 3.65), color=GRAY, width=1.6)
    save_figure(fig, "lqr_feedback_loop")


def coordinate_system() -> None:
    fig, ax = setup_geometry_axes((9.5, 6.2))
    origin = np.array([0.0, 0.0])
    rear = np.array([3.0, 2.0])
    yaw = np.deg2rad(28.0)
    forward = np.array([np.cos(yaw), np.sin(yaw)])
    left = np.array([-np.sin(yaw), np.cos(yaw)])
    arrow(ax, origin, (6.7, 0), color=GRAY)
    arrow(ax, origin, (0, 5.4), color=GRAY)
    ax.text(6.85, 0, "$X$ [m]", va="center")
    ax.text(0, 5.55, "$Y$ [m]", ha="center")
    ax.plot([rear[0], rear[0]], [0, rear[1]], "--", color="#999")
    ax.plot([0, rear[0]], [rear[1], rear[1]], "--", color="#999")
    draw_vehicle_from_rear(ax, rear, yaw, steering=np.deg2rad(18))
    arrow(ax, rear, rear + 2.1 * forward, color=BLUE, width=2.4)
    arrow(ax, rear, rear + 1.7 * left, color=GREEN, width=2.4)
    draw_angle(ax, rear, 1.0, 0.0, yaw, r"$\psi$")
    cog = rear + VEHICLE.cog_from_rear_axle * forward
    ax.scatter(*rear, color="#7a3db8", s=55, zorder=12)
    ax.scatter(*cog, color=RED, s=55, zorder=12)
    ax.text(*(rear - 0.45 * left), "状態位置 $(x,y)$\n後輪車軸中心", ha="center")
    ax.text(*(cog + 0.42 * left), "COG", color=RED, ha="center")
    ax.text(*(rear + 2.25 * forward), "$x_v$（車両前方）", color=BLUE)
    ax.text(*(rear + 1.8 * left), "$y_v$（車両左方）", color=GREEN)
    ax.text(1.45, 0.16, "$x$")
    ax.text(3.15, 0.95, "$y$")
    ax.set_xlim(-0.6, 7.4); ax.set_ylim(-0.6, 5.9)
    save_figure(fig, "coordinate_system")


def tracking_error_projection() -> None:
    fig, ax = setup_geometry_axes((9.5, 5.8))
    reference = np.array([0.0, 0.0]); psi_r = np.deg2rad(24)
    tangent = np.array([np.cos(psi_r), np.sin(psi_r)])
    normal = np.array([-np.sin(psi_r), np.cos(psi_r)])
    e_lon, e_lat = 2.2, 1.35
    vehicle = reference + e_lon * tangent + e_lat * normal
    calculated_lon, calculated_lat = tracking_errors(vehicle, reference, psi_r)
    assert abs(calculated_lon - e_lon) < 1e-12
    assert abs(calculated_lat - e_lat) < 1e-12
    yaw = psi_r - np.deg2rad(16)
    path_u = np.linspace(-2.5, 5.2, 50)
    path = reference[:, None] + tangent[:, None] * path_u
    ax.plot(path[0], path[1], color=GRAY, lw=3, label="参照経路")
    projected = reference + e_lon * tangent
    ax.plot([reference[0], vehicle[0]], [reference[1], vehicle[1]],
            color="#9b59b6", lw=2.2, label=r"位置差 $\Delta\mathbf{p}$")
    ax.plot([reference[0], projected[0]], [reference[1], projected[1]],
            color=BLUE, lw=3)
    ax.plot([projected[0], vehicle[0]], [projected[1], vehicle[1]],
            color=RED, lw=3)
    ax.plot([projected[0], projected[0] + 0.25 * tangent[0], vehicle[0]],
            [projected[1], projected[1] + 0.25 * tangent[1], vehicle[1]],
            color="#333", lw=1.2)
    arrow(ax, reference, reference + 1.6 * tangent, color=BLUE)
    arrow(ax, reference, reference + 1.35 * normal, color=GREEN)
    draw_vehicle(ax, vehicle, yaw, length=1.25, width=0.62)
    draw_angle(ax, vehicle, 0.85, yaw, psi_r, r"$e_\psi$")
    ax.scatter(*reference, color=GREEN, s=55, zorder=10)
    ax.text(*(reference - 0.35 * normal), "参照点 $(X_r,Y_r)$", ha="center")
    ax.text(*(vehicle + 0.5 * normal), "車両 $(X,Y)$", ha="center")
    ax.text(*(0.5 * (reference + projected) - 0.28 * normal),
            r"$e_{\mathrm{lon}}=\Delta\mathbf{p}\cdot\mathbf{t}_r$", color=BLUE)
    ax.text(*(0.5 * (projected + vehicle) + 0.18 * tangent),
            r"$e_{\mathrm{lat}}=\Delta\mathbf{p}\cdot\mathbf{n}_r$", color=RED)
    ax.text(*(reference + 1.65 * tangent), r"$\mathbf{t}_r$", color=BLUE)
    ax.text(*(reference + 1.40 * normal), r"$\mathbf{n}_r$", color=GREEN)
    ax.legend(loc="upper left", frameon=False)
    ax.set_xlim(-2.3, 5.3); ax.set_ylim(-1.4, 4.2)
    save_figure(fig, "tracking_error_projection")


def path_geometry() -> None:
    fig, axes = plt.subplots(1, 2, figsize=(12, 5.2), constrained_layout=True)
    for ax in axes: ax.set_aspect("equal"); ax.axis("off")
    # Straight segment.
    ax = axes[0]; p0=np.array([0.,0.]); psi=np.deg2rad(28); ds=2.4
    direction=np.array([np.cos(psi),np.sin(psi)]); p1=p0+ds*direction
    arrow(ax,p0,p1,color=BLUE,width=2.5); draw_angle(ax,p0,0.65,0,psi,r"$\psi_k$")
    ax.plot([p1[0],p1[0]],[0,p1[1]],"--",color="#999")
    ax.plot([0,p1[0]],[p1[1],p1[1]],"--",color="#999")
    ax.scatter(*p0,color=GREEN);ax.scatter(*p1,color=RED)
    ax.text(*p0,"$P_k$",ha="right",va="top");ax.text(*p1,"$P_{k+1}$",ha="left")
    ax.text(*(0.5*(p0+p1)+0.18*np.array([-direction[1],direction[0]])),r"$q\Delta s$")
    ax.text(p1[0]/2,-0.25,r"$q\Delta s\cos\psi_k$",ha="center",color=BLUE)
    ax.text(p1[0]+0.15,p1[1]/2,r"$q\Delta s\sin\psi_k$",va="center",color=RED)
    ax.set_title(r"(a) 直線区間 $\kappa=0$");ax.set_xlim(-.5,3);ax.set_ylim(-.6,2.2)
    # Arc segment.
    ax=axes[1]; R=2.1; center=np.array([0.,R]); theta=np.linspace(-np.pi/2,-np.pi/2+0.95,120)
    arc=center[:,None]+R*np.vstack([np.cos(theta),np.sin(theta)])
    ax.plot(arc[0],arc[1],color=BLUE,lw=3); ax.plot([center[0],arc[0,0]],[center[1],arc[1,0]],"--",color=GRAY)
    ax.plot([center[0],arc[0,-1]],[center[1],arc[1,-1]],"--",color=GRAY)
    arrow(ax,arc[:,0],arc[:,0]+np.array([1.2,0]),color=GREEN)
    end_tangent=np.array([np.cos(theta[-1]+np.pi/2),np.sin(theta[-1]+np.pi/2)])
    arrow(ax,arc[:,-1],arc[:,-1]+1.2*end_tangent,color=RED)
    draw_angle(ax,center,.75,theta[0],theta[-1],r"$q\kappa\Delta s$")
    ax.scatter(*center,color="#444"); ax.text(center[0]-.12,center[1]+.2,"$C$",ha="right")
    ax.text(-.18,R/2,"$R=1/|\kappa|$",ha="right")
    ax.text(*arc[:,0],"$P_k,\,\psi_k$",ha="right",va="top")
    ax.text(*arc[:,-1],"$P_{k+1},\,\psi_{k+1}$",ha="left",va="bottom")
    ax.set_title(r"(b) 一定曲率円弧 $\kappa\ne 0$");ax.set_xlim(-1,3.3);ax.set_ylim(-.6,3.2)
    save_figure(fig,"path_geometry")


def path_arc_update() -> None:
    fig, ax = plt.subplots(figsize=(10,5),constrained_layout=True);ax.axis("off")
    rows=[(r"$d\psi=q\kappa ds$","式 (3): 距離から方位変化"),(r"$qds=d\psi/\kappa$","式 (6): 積分変数を変換"),(r"$dx=\cos\psi\,d\psi/\kappa$", "式 (7): X方向"),(r"$dy=\sin\psi\,d\psi/\kappa$","Y方向"),(r"$\Delta x=(\sin\psi_{k+1}-\sin\psi_k)/\kappa$","式 (8)"),(r"$\Delta y=-(\cos\psi_{k+1}-\cos\psi_k)/\kappa$","式 (9)")]
    for i,(formula,label) in enumerate(rows):
        y=5.35-i*.82;add_block(ax,(1.1,y),(5.5,.58),formula,label,color=LIGHT_BLUE)
        if i<len(rows)-1:arrow(ax,(3.85,y),(3.85,y-.22),color=GRAY,width=1.4)
    ax.text(8.0,3.1,r"$q$: 進行方向"+"\n"+r"$\kappa$: 符号付き曲率"+"\n"+r"$ds$: 微小経路長"+"\n"+r"$\psi$: 接線方位",bbox=dict(boxstyle="round",fc=LIGHT_GRAY,ec="#aaa"))
    ax.set_xlim(0,11);ax.set_ylim(.4,6.2);save_figure(fig,"path_arc_update")


def velocity_profile() -> None:
    fig, axes=plt.subplots(2,1,figsize=(10,7),sharex=True,constrained_layout=True)
    s_end=12.;s=np.linspace(0,s_end,400);a_max=.6;d_max=.5;v_max=1.8
    vacc=np.sqrt(2*a_max*s);vdec=np.sqrt(2*d_max*(s_end-s));v=np.minimum.reduce([np.full_like(s,v_max),vacc,vdec])
    ax=axes[0];ax.plot(s,vacc,"--",label=r"$v_{acc}=\sqrt{2a_{max}s}$");ax.plot(s,vdec,"--",label=r"$v_{dec}=\sqrt{2d_{max}(s_{end}-s)}$");ax.axhline(v_max,color=GRAY,ls=":",label=r"$v_{max}$");ax.plot(s,v,color=BLUE,lw=3,label=r"$|v(s)|=\min(\cdots)$");ax.fill_between(s,0,v,color=BLUE,alpha=.10);ax.set_ylabel("速さ [m/s]");ax.legend(ncol=2);ax.grid(alpha=.25)
    axes[1].plot(s,v,color=BLUE,lw=2.5,label="$q=+1$ 前進");axes[1].plot(s,-v,color=RED,lw=2.5,label="$q=-1$ 後退");axes[1].axhline(0,color="#333",lw=1);axes[1].set_xlabel("累積距離 $s$ [m]");axes[1].set_ylabel("符号付き速度 $v=q|v|$ [m/s]");axes[1].legend();axes[1].grid(alpha=.25)
    save_figure(fig,"velocity_profile")


def velocity_interval() -> None:
    fig,axes=plt.subplots(1,2,figsize=(11,4.5),constrained_layout=True)
    for ax in axes:ax.grid(alpha=.25)
    t=np.array([0,1.4]);v=np.array([.6,1.5]);axes[0].plot(t,v,"o-",color=BLUE,lw=3);axes[0].fill_between(t,0,v,color=BLUE,alpha=.15);axes[0].text(.7,.82,r"面積 $=\Delta s$",ha="center");axes[0].text(.7,1.28,r"$\bar v_k=(|v_k|+|v_{k+1}|)/2$",ha="center");axes[0].set(xlabel=r"時間 $t$ [s]",ylabel=r"速さ $|v|$ [m/s]",title="(a) 台形面積から区間時間")
    s=np.array([0,2.]);v2=np.array([.6**2,1.5**2]);axes[1].plot(s,v2,"o-",color=RED,lw=3);axes[1].text(1,1.5,r"傾き $=2a_k$",ha="center");axes[1].set(xlabel=r"距離 $s$ [m]",ylabel=r"速度二乗 $v^2$",title="(b) $v_{k+1}^2-v_k^2=2a_k\Delta s$")
    save_figure(fig,"velocity_interval")


def pure_pursuit_forward_reverse() -> None:
    fig,axes=plt.subplots(1,2,figsize=(12,5.3),constrained_layout=True)
    for ax,q,title in zip(axes,[1,-1],["(a) 前進 $q=+1$","(b) 後退 $q=-1$"]):
        ax.set_aspect("equal");ax.axis("off");alpha=np.deg2rad(28);L=4.;g=create_pure_pursuit_geometry(alpha,L)
        direction_yaw=0 if q==1 else np.pi;target=g.target if q==1 else np.array([-g.target[0],g.target[1]])
        motion_alpha=alpha;motion_kappa=2*np.sin(motion_alpha)/L;vehicle_kappa=q*motion_kappa;delta=np.arctan(VEHICLE.wheelbase*vehicle_kappa)
        draw_vehicle(ax,(0,0),0,1.35,.65,steering=delta);arrow(ax,(0,0),(1.4*q,0),color=BLUE);ax.plot([0,target[0]],[0,target[1]],color=RED,lw=2.5);ax.scatter(*target,color=GREEN,s=55);draw_angle(ax,(0,0),.9,min(direction_yaw,np.arctan2(target[1],target[0])),max(direction_yaw,np.arctan2(target[1],target[0])),r"$\alpha$");ax.text(target[0],target[1]+.25,"$T$");ax.text(.5*target[0],.5*target[1]+.2,"$L$");box_x=-.8 if q==1 else -4.25;ax.text(box_x,-1.25,rf"$\kappa_m={motion_kappa:.3f}$  $\kappa=q\kappa_m={vehicle_kappa:.3f}$"+"\n"+rf"$\delta=\arctan(L_{{wb}}\kappa)={np.degrees(delta):.1f}^\circ$",bbox=dict(boxstyle="round",fc=LIGHT_BLUE,ec="#9ab9d4"));ax.set_title(title);ax.set_xlim((-1.2,4.6) if q==1 else (-4.6,1.2));ax.set_ylim(-1.6,2.8)
    save_figure(fig,"pure_pursuit_forward_reverse")


def stanley_reference_points() -> None:
    fig,axes=plt.subplots(1,2,figsize=(12,5),constrained_layout=True)
    for ax,title,base_x in zip(axes,["(a) 本実装: 後輪車軸中心","(b) 一般例: 前輪車軸中心"],[0.,VEHICLE.wheelbase]):
        ax.set_aspect("equal");ax.axis("off");x=np.linspace(-3,5,100);ax.plot(x,.08*x**2,color=GRAY,lw=3);rear=np.array([0.,1.45]);draw_vehicle(ax,rear+np.array([VEHICLE.wheelbase/2,0]),0,length=VEHICLE.length,width=VEHICLE.width,steering=-.2);base=rear+np.array([base_x,0]);idx=np.argmin((x-base[0])**2+(.08*x**2-base[1])**2);nearest=np.array([x[idx],.08*x[idx]**2]);ax.plot([base[0],nearest[0]],[base[1],nearest[1]],color=RED,lw=3);ax.scatter(*base,color=BLUE,s=55);ax.scatter(*nearest,color=GREEN,s=55);ax.text(*base," 基準点",va="bottom");ax.text(*nearest," 最近傍点",va="top");ax.text(*(0.5*(base+nearest)),"$e_y$",color=RED);ax.set_title(title);ax.set_xlim(-3.2,5.3);ax.set_ylim(-.7,3)
    fig.text(.5,.02,"基準点が異なると、同じ車両姿勢でも最近傍点と横偏差が変わる",ha="center",bbox=dict(boxstyle="round",fc=LIGHT_GRAY,ec="#aaa"))
    save_figure(fig,"stanley_reference_points")


def bicycle_state_update() -> None:
    fig,ax=setup_geometry_axes((10,6));state=np.array([0.5,.5,np.deg2rad(20)]);v=1.2;delta=np.deg2rad(28);dt=.8;next_state,omega=exact_bicycle_step(state,v,delta,VEHICLE.wheelbase,dt)
    radius=v/omega;left=np.array([-np.sin(state[2]),np.cos(state[2])]);center=state[:2]+radius*left
    theta=np.linspace(np.arctan2(state[1]-center[1],state[0]-center[0]),np.arctan2(next_state[1]-center[1],next_state[0]-center[0]),100);ax.plot(center[0]+abs(radius)*np.cos(theta),center[1]+abs(radius)*np.sin(theta),color=BLUE,lw=3)
    draw_vehicle(ax,state[:2],state[2],1.3,.65,delta);draw_vehicle(ax,next_state[:2],next_state[2],1.3,.65,delta,alpha=.65);ax.scatter(*center,color=RED);ax.plot([center[0],state[0]],[center[1],state[1]],"--",color=GRAY);ax.text(*center," ICR");ax.text(*(state[:2]-.5*left),r"$(x_k,y_k,\psi_k)$");ax.text(*(next_state[:2]+.5*left),r"$(x_{k+1},y_{k+1},\psi_{k+1})$");ax.text(2.4,4.45,rf"$v={v}$ m/s, $\delta={np.degrees(delta):.0f}^\circ$, $\Delta t={dt}$ s"+"\n"+rf"$\omega=v\tan\delta/L={omega:.3f}$ rad/s"+"\n"+r"$\Delta\psi=\omega\Delta t$",bbox=dict(boxstyle="round",fc=LIGHT_BLUE,ec="#9ab9d4"));ax.set_xlim(-2.2,6);ax.set_ylim(-.8,6.1);save_figure(fig,"bicycle_state_update")


def linearization_comparison() -> None:
    fig,axes=plt.subplots(1,2,figsize=(11,4.5),constrained_layout=True)
    e=np.linspace(-1.2,1.2,300);axes[0].plot(e,np.sin(e),label=r"非線形 $\sin e_\psi$",lw=3);axes[0].plot(e,e,"--",label=r"線形近似 $e_\psi$",lw=2);axes[0].axvspan(-.3,.3,color=GREEN,alpha=.1,label="小誤差領域");axes[0].set(xlabel=r"$e_\psi$ [rad]",ylabel="値",title=r"(a) $\sin e_\psi\approx e_\psi$");axes[0].legend();axes[0].grid(alpha=.25)
    d0=np.deg2rad(15);dd=np.linspace(-.45,.45,300);exact=np.tan(d0+dd);linear=np.tan(d0)+dd/np.cos(d0)**2;axes[1].plot(dd,exact,lw=3,label=r"$\tan(\delta_{ff}+\Delta\delta)$");axes[1].plot(dd,linear,"--",lw=2,label="一次近似");axes[1].axvspan(-.12,.12,color=GREEN,alpha=.1);axes[1].set(xlabel=r"補正舵角 $\Delta\delta$ [rad]",ylabel="tan値",title=r"(b) $\delta_{ff}$周りの線形化");axes[1].legend();axes[1].grid(alpha=.25);save_figure(fig,"linearization_comparison")


def steering_response() -> None:
    fig,axes=plt.subplots(2,1,figsize=(10,7),sharex=True,constrained_layout=True);dt=.01;t=np.arange(0,4+dt,dt);requested=np.where(t<.5,0,np.where(t<2.2,np.deg2rad(50),np.deg2rad(-15)));max_angle=np.deg2rad(VEHICLE.max_steering_angle_deg);angle=np.clip(requested,-max_angle,max_angle);tau=.3;lag=np.zeros_like(t);applied=np.zeros_like(t);rate_limit=np.deg2rad(60)
    for i in range(1,len(t)):
        lag[i]=lag[i-1]+(1-np.exp(-dt/tau))*(angle[i]-lag[i-1]);change=np.clip(lag[i]-applied[i-1],-rate_limit*dt,rate_limit*dt);applied[i]=applied[i-1]+change
    axes[0].plot(t,np.rad2deg(requested),label=r"要求 $\delta_{cmd}$");axes[0].plot(t,np.rad2deg(angle),label=r"角度制約後 $\delta_{angle}$");axes[0].plot(t,np.rad2deg(lag),label=r"一次遅れ後 $\delta_{lag}$");axes[0].plot(t,np.rad2deg(applied),lw=3,label=r"実舵角 $\delta$");axes[0].axhline(VEHICLE.max_steering_angle_deg,color=GRAY,ls=":");axes[0].axhline(-VEHICLE.max_steering_angle_deg,color=GRAY,ls=":");axes[0].set_ylabel("舵角 [deg]");axes[0].legend(ncol=2);axes[0].grid(alpha=.25)
    rate=np.gradient(applied,dt);axes[1].plot(t,np.rad2deg(rate),color=RED);axes[1].axhline(60,color=GRAY,ls=":",label=r"$\dot\delta_{max}$");axes[1].axhline(-60,color=GRAY,ls=":");axes[1].set(xlabel="時間 [s]",ylabel="操舵速度 [deg/s]");axes[1].legend();axes[1].grid(alpha=.25);save_figure(fig,"steering_response")


def lqr_dimensions() -> None:
    fig,ax=plt.subplots(figsize=(11,5),constrained_layout=True);ax.axis("off");ax.set_xlim(0,12);ax.set_ylim(0,6)
    items=[(1,3.7,2.2,1.2,"状態 $x_k$",r"$2\times1$"+"\n"+r"$[e_y,e_\psi]^T$"),(4,3.7,2.2,1.2,"状態行列 $A$",r"$2\times2$"),(7,3.7,2.2,1.2,"入力行列 $B$",r"$2\times1$"),(10,3.7,1.3,1.2,"入力 $u_k$",r"$1\times1$")]
    for x,y,w,h,title,sub in items:add_block(ax,(x,y),(w,h),title,sub)
    ax.text(6,2.8,r"$x_{k+1}=A x_k+B u_k$",ha="center",fontsize=20)
    add_block(ax,(1.4,.7),(2.3,1.1),"状態重み $Q$",r"$2\times2$");add_block(ax,(4.9,.7),(2.3,1.1),"入力重み $R$",r"$1\times1$");add_block(ax,(8.4,.7),(2.3,1.1),"ゲイン $K$",r"$1\times2$")
    ax.text(6,2.15,r"$J=\sum(x_k^TQx_k+u_k^TRu_k),\qquad u_k=-Kx_k$",ha="center",fontsize=16)
    save_figure(fig,"lqr_dimensions")


def main() -> None:
    configure_style()
    pure_pursuit_geometry()
    stanley_geometry()
    bicycle_geometry()
    linear_error_model()
    steering_actuator_flow()
    lqr_feedback_loop()
    coordinate_system()
    tracking_error_projection()
    path_geometry()
    path_arc_update()
    velocity_profile()
    velocity_interval()
    pure_pursuit_forward_reverse()
    stanley_reference_points()
    bicycle_state_update()
    linearization_comparison()
    steering_response()
    lqr_dimensions()
    print(f"Generated documentation figures in: {OUTPUT_DIR}")


if __name__ == "__main__":
    main()
