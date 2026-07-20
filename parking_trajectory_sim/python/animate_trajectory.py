"""Animate logged vehicle motion. Traceability: IMPL-ANIM-01, BICYCLE-EQ-03."""

from argparse import ArgumentParser
from pathlib import Path

import matplotlib

# GIF生成は画面表示を必要としないため、非GUIバックエンドを使用する。
matplotlib.use("Agg")

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter
from matplotlib.patches import FancyArrowPatch, Polygon
import numpy as np

from vehicle_parameters import VEHICLE
from parking_environment import draw_parking_environment, load_parking_environment


PROJECT_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_OUTPUT_DIRECTORY = PROJECT_ROOT / "output" / "feedforward" / "forward"

REFERENCE_COLUMNS = {"x", "y"}
VEHICLE_COLUMNS = {
    "time",
    "x",
    "y",
    "yaw",
    "reference_x",
    "reference_y",
    "reference_yaw",
    "steering",
    "position_error",
    "lateral_error",
    "longitudinal_error",
    "yaw_error",
    "actual_velocity",
    "applied_acceleration",
    "parking_mode",
}


def parse_arguments():
    """アニメーション生成条件をコマンドラインから読み取る。"""
    parser = ArgumentParser(
        description="Animate vehicle tracking of the reference trajectory."
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=DEFAULT_OUTPUT_DIRECTORY,
        help="Directory containing CSV files (default: output/feedforward/forward).",
    )
    parser.add_argument(
        "--filename",
        default="trajectory_animation.gif",
        help="Output GIF filename.",
    )
    parser.add_argument(
        "--fps",
        type=int,
        default=20,
        help="Animation frame rate (default: 20).",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=12.0,
        help="Approximate animation duration in seconds (default: 12).",
    )
    parser.add_argument(
        "--wheelbase",
        type=float,
        default=VEHICLE.wheelbase,
        help=f"Vehicle wheelbase in metres (default: {VEHICLE.wheelbase}).",
    )
    parser.add_argument(
        "--vehicle-width",
        type=float,
        default=VEHICLE.width,
        help=f"Vehicle width in metres (default: {VEHICLE.width}).",
    )
    return parser.parse_args()


def validate_arguments(args):
    """描画・保存に使用する数値条件を検証する。"""
    if args.fps <= 0:
        raise ValueError("--fps must be positive")
    if args.duration <= 0.0:
        raise ValueError("--duration must be positive")
    if args.wheelbase <= 0.0:
        raise ValueError("--wheelbase must be positive")
    if args.vehicle_width <= 0.0:
        raise ValueError("--vehicle-width must be positive")
    if Path(args.filename).suffix.lower() != ".gif":
        raise ValueError("--filename must have the .gif extension")


def load_csv(path, required_columns):
    """CSVを読み込み、アニメーションに必要な列を検証する。"""
    if not path.is_file():
        raise FileNotFoundError(
            f"Required CSV file was not found: {path}\n"
            "Run parking_trajectory_sim before creating the animation."
        )

    data = np.atleast_1d(np.genfromtxt(
        path, delimiter=",", names=True, dtype=None, encoding="utf-8"
    ))
    actual_columns = set(data.dtype.names or ())
    missing_columns = required_columns - actual_columns
    if missing_columns:
        missing = ", ".join(sorted(missing_columns))
        raise ValueError(f"{path} is missing required columns: {missing}")
    if data.size == 0:
        raise ValueError(f"{path} does not contain any data rows")
    return data


def load_results(output_directory):
    """参照軌道と車両ログを読み込む。"""
    reference = load_csv(
        output_directory / "reference_trajectory.csv",
        REFERENCE_COLUMNS,
    )
    vehicle = load_csv(
        output_directory / "vehicle_log.csv",
        VEHICLE_COLUMNS,
    )
    return reference, vehicle, load_parking_environment(output_directory)


def make_frame_indices(sample_count, fps, duration):
    """指定した表示時間に収まるよう、車両ログのフレームを間引く。"""
    maximum_frames = max(2, int(round(fps * duration)))
    frame_count = min(sample_count, maximum_frames)
    return np.unique(
        np.linspace(0, sample_count - 1, frame_count, dtype=int)
    )


def transform_vertices(local_vertices, x, y, yaw):
    """車体座標系の頂点を世界座標系へ変換する。"""
    cosine = np.cos(yaw)
    sine = np.sin(yaw)
    rotation = np.array([[cosine, -sine], [sine, cosine]])
    return local_vertices @ rotation.T + np.array([x, y])


def vehicle_vertices(x, y, yaw, wheelbase, vehicle_width):
    """後輪軸中心を基準に、上面視の車体輪郭を世界座標で求める。"""
    # 既定値では理論図と同じ共通車両諸元を使用する。
    rear_overhang = (VEHICLE.rear_overhang
                     if wheelbase == VEHICLE.wheelbase else 0.25 * wheelbase)
    front_overhang = (VEHICLE.front_overhang
                      if wheelbase == VEHICLE.wheelbase else 0.25 * wheelbase)
    half_width = vehicle_width / 2.0
    shoulder = 0.88 * half_width

    # 前後端を少し絞り、単純な長方形より車体らしい輪郭にする。
    local_vertices = np.array(
        [
            [-rear_overhang, -shoulder],
            [-0.05 * wheelbase, -half_width],
            [0.85 * wheelbase, -half_width],
            [wheelbase + front_overhang, -shoulder],
            [wheelbase + front_overhang, shoulder],
            [0.85 * wheelbase, half_width],
            [-0.05 * wheelbase, half_width],
            [-rear_overhang, shoulder],
        ]
    )
    return transform_vertices(local_vertices, x, y, yaw)


def wheel_vertices(
    vehicle_x,
    vehicle_y,
    vehicle_yaw,
    longitudinal_position,
    lateral_position,
    steering_angle,
    wheel_length,
    wheel_width,
):
    """車輪中心と操舵角から、タイヤ輪郭の世界座標を求める。"""
    center_x = vehicle_x + (
        longitudinal_position * np.cos(vehicle_yaw)
        - lateral_position * np.sin(vehicle_yaw)
    )
    center_y = vehicle_y + (
        longitudinal_position * np.sin(vehicle_yaw)
        + lateral_position * np.cos(vehicle_yaw)
    )
    half_length = wheel_length / 2.0
    half_width = wheel_width / 2.0
    local_vertices = np.array(
        [
            [-half_length, -half_width],
            [half_length, -half_width],
            [half_length, half_width],
            [-half_length, half_width],
        ]
    )
    return transform_vertices(
        local_vertices,
        center_x,
        center_y,
        vehicle_yaw + steering_angle,
    )


def direction_segment(x, y, yaw, length):
    """姿勢を示す線分の始点・終点を求める。"""
    return (
        [x, x + length * np.cos(yaw)],
        [y, y + length * np.sin(yaw)],
    )


def vehicle_cog(x, y, yaw, wheelbase):
    """後輪軸中心からホイールベース中央に置いた仮想COGを求める。"""
    cog_distance = (VEHICLE.cog_from_rear_axle
                    if wheelbase == VEHICLE.wheelbase else 0.5 * wheelbase)
    return (
        x + cog_distance * np.cos(yaw),
        y + cog_distance * np.sin(yaw),
    )


def create_animation(reference, vehicle, environment, frame_indices, args):
    """軌道追従の時系列変化を示すMatplotlibアニメーションを作成する。"""
    figure, axis = plt.subplots(figsize=(9, 8))

    draw_parking_environment(axis, environment)
    forward = reference["velocity"] > 0.0 if "velocity" in reference.dtype.names else np.ones(len(reference), dtype=bool)
    reverse = reference["velocity"] < 0.0 if "velocity" in reference.dtype.names else np.zeros(len(reference), dtype=bool)
    axis.plot(reference["x"][forward], reference["y"][forward],
              color="#009E73", linestyle="--", linewidth=2,
              label="Forward reference")
    axis.plot(reference["x"][reverse], reference["y"][reverse],
              color="#D55E00", linestyle="--", linewidth=2,
              label="Reverse reference")
    vehicle_trail, = axis.plot([], [], color="tab:blue", linewidth=2, label="Vehicle trail")
    reference_point, = axis.plot([], [], "ro", markersize=6, label="Current reference")
    reference_heading, = axis.plot([], [], color="tab:red", linewidth=2)
    vehicle_heading, = axis.plot([], [], color="tab:blue", linewidth=2)

    initial_vertices = vehicle_vertices(
        vehicle["x"][0],
        vehicle["y"][0],
        vehicle["yaw"][0],
        args.wheelbase,
        args.vehicle_width,
    )
    vehicle_body = Polygon(
        initial_vertices,
        closed=True,
        facecolor="#c8c8c8",
        edgecolor="#555555",
        linewidth=2,
        alpha=0.75,
        label="Vehicle body",
    )
    axis.add_patch(vehicle_body)

    # 後輪軸中心が車両状態 (x, y)、前輪軸はwheelbase前方にある。
    half_track = 0.39 * args.vehicle_width
    wheel_length = 0.24 * args.wheelbase
    wheel_width = 0.16 * args.vehicle_width
    initial_wheel_specs = (
        (args.wheelbase, half_track, vehicle["steering"][0]),
        (args.wheelbase, -half_track, vehicle["steering"][0]),
        (0.0, half_track, 0.0),
        (0.0, -half_track, 0.0),
    )
    wheel_patches = []
    for longitudinal, lateral, steering in initial_wheel_specs:
        wheel = Polygon(
            wheel_vertices(
                vehicle["x"][0],
                vehicle["y"][0],
                vehicle["yaw"][0],
                longitudinal,
                lateral,
                steering,
                wheel_length,
                wheel_width,
            ),
            closed=True,
            facecolor="#242424",
            edgecolor="black",
            linewidth=1.2,
            zorder=5,
        )
        axis.add_patch(wheel)
        wheel_patches.append(wheel)

    rear_axle, = axis.plot([], [], color="#4a4a4a", linewidth=2, zorder=3)
    front_axle, = axis.plot([], [], color="#4a4a4a", linewidth=2, zorder=3)

    initial_cog_x, initial_cog_y = vehicle_cog(
        vehicle["x"][0],
        vehicle["y"][0],
        vehicle["yaw"][0],
        args.wheelbase,
    )
    cog_marker, = axis.plot(
        [initial_cog_x],
        [initial_cog_y],
        "o",
        color="crimson",
        markersize=6,
        zorder=7,
    )
    cog_label = axis.text(
        initial_cog_x,
        initial_cog_y,
        "COG",
        color="crimson",
        fontsize=9,
        fontweight="bold",
        zorder=7,
    )

    coordinate_axis_length = 0.48 * args.wheelbase
    vehicle_x_axis = FancyArrowPatch(
        (initial_cog_x, initial_cog_y),
        (initial_cog_x + coordinate_axis_length, initial_cog_y),
        arrowstyle="-|>",
        mutation_scale=13,
        color="#b22222",
        linewidth=1.8,
        zorder=7,
    )
    vehicle_y_axis = FancyArrowPatch(
        (initial_cog_x, initial_cog_y),
        (initial_cog_x, initial_cog_y + coordinate_axis_length),
        arrowstyle="-|>",
        mutation_scale=13,
        color="#2e7d32",
        linewidth=1.8,
        zorder=7,
    )
    axis.add_patch(vehicle_x_axis)
    axis.add_patch(vehicle_y_axis)
    vehicle_x_label = axis.text(
        initial_cog_x + coordinate_axis_length,
        initial_cog_y,
        "x_vehicle",
        color="#b22222",
        fontsize=8,
        zorder=7,
    )
    vehicle_y_label = axis.text(
        initial_cog_x,
        initial_cog_y + coordinate_axis_length,
        "y_vehicle",
        color="#2e7d32",
        fontsize=8,
        zorder=7,
    )

    status_text = axis.text(
        0.02,
        0.98,
        "",
        transform=axis.transAxes,
        va="top",
        bbox={"facecolor": "white", "alpha": 0.85, "edgecolor": "0.7"},
    )

    all_x = np.concatenate(
        (reference["x"], vehicle["x"], vehicle["reference_x"])
    )
    all_y = np.concatenate(
        (reference["y"], vehicle["y"], vehicle["reference_y"])
    )
    if environment is not None:
        rectangle_rows = [row for row in environment
                          if str(row["record"]) in {"parking_space", "obstacle"}]
        if rectangle_rows:
            all_x = np.concatenate((all_x,
                np.array([value for row in rectangle_rows
                          for value in (row["min_x"], row["max_x"])])))
            all_y = np.concatenate((all_y,
                np.array([value for row in rectangle_rows
                          for value in (row["min_y"], row["max_y"])])))
    vehicle_length = 1.5 * args.wheelbase
    margin = max(1.0, 0.6 * vehicle_length)
    axis.set_xlim(np.min(all_x) - margin, np.max(all_x) + margin)
    axis.set_ylim(np.min(all_y) - margin, np.max(all_y) + margin)
    axis.set_aspect("equal", adjustable="box")
    axis.set_xlabel("World X [m]")
    axis.set_ylabel("World Y [m]")
    axis.set_title("Vehicle trajectory tracking")
    axis.grid(True)
    axis.legend(loc="lower right")

    heading_length = 0.8 * args.wheelbase

    def update(frame_number):
        sample_index = frame_indices[frame_number]

        vehicle_trail.set_data(
            vehicle["x"][: sample_index + 1],
            vehicle["y"][: sample_index + 1],
        )
        vehicle_body.set_xy(
            vehicle_vertices(
                vehicle["x"][sample_index],
                vehicle["y"][sample_index],
                vehicle["yaw"][sample_index],
                args.wheelbase,
                args.vehicle_width,
            )
        )

        steering = vehicle["steering"][sample_index]
        wheel_specs = (
            (args.wheelbase, half_track, steering),
            (args.wheelbase, -half_track, steering),
            (0.0, half_track, 0.0),
            (0.0, -half_track, 0.0),
        )
        for wheel, (longitudinal, lateral, wheel_steering) in zip(
            wheel_patches,
            wheel_specs,
        ):
            wheel.set_xy(
                wheel_vertices(
                    vehicle["x"][sample_index],
                    vehicle["y"][sample_index],
                    vehicle["yaw"][sample_index],
                    longitudinal,
                    lateral,
                    wheel_steering,
                    wheel_length,
                    wheel_width,
                )
            )

        # 軸線を車体中心から左右へ伸ばすため、反対側の端点も計算する。
        rear_left_x = vehicle["x"][sample_index] - half_track * np.sin(
            vehicle["yaw"][sample_index]
        )
        rear_left_y = vehicle["y"][sample_index] + half_track * np.cos(
            vehicle["yaw"][sample_index]
        )
        rear_right_x = vehicle["x"][sample_index] + half_track * np.sin(
            vehicle["yaw"][sample_index]
        )
        rear_right_y = vehicle["y"][sample_index] - half_track * np.cos(
            vehicle["yaw"][sample_index]
        )
        rear_axle.set_data(
            [rear_left_x, rear_right_x],
            [rear_left_y, rear_right_y],
        )

        front_center_x = vehicle["x"][sample_index] + args.wheelbase * np.cos(
            vehicle["yaw"][sample_index]
        )
        front_center_y = vehicle["y"][sample_index] + args.wheelbase * np.sin(
            vehicle["yaw"][sample_index]
        )
        front_axle.set_data(
            [
                front_center_x - half_track * np.sin(vehicle["yaw"][sample_index]),
                front_center_x + half_track * np.sin(vehicle["yaw"][sample_index]),
            ],
            [
                front_center_y + half_track * np.cos(vehicle["yaw"][sample_index]),
                front_center_y - half_track * np.cos(vehicle["yaw"][sample_index]),
            ],
        )

        cog_x, cog_y = vehicle_cog(
            vehicle["x"][sample_index],
            vehicle["y"][sample_index],
            vehicle["yaw"][sample_index],
            args.wheelbase,
        )
        cog_marker.set_data([cog_x], [cog_y])

        forward_x = cog_x + coordinate_axis_length * np.cos(
            vehicle["yaw"][sample_index]
        )
        forward_y = cog_y + coordinate_axis_length * np.sin(
            vehicle["yaw"][sample_index]
        )
        lateral_x = cog_x - coordinate_axis_length * np.sin(
            vehicle["yaw"][sample_index]
        )
        lateral_y = cog_y + coordinate_axis_length * np.cos(
            vehicle["yaw"][sample_index]
        )
        vehicle_x_axis.set_positions((cog_x, cog_y), (forward_x, forward_y))
        vehicle_y_axis.set_positions((cog_x, cog_y), (lateral_x, lateral_y))

        cog_label.set_position((cog_x, cog_y))
        cog_label.set_ha("right")
        cog_label.set_va("top")
        vehicle_x_label.set_position((forward_x, forward_y))
        vehicle_y_label.set_position((lateral_x, lateral_y))

        reference_x = vehicle["reference_x"][sample_index]
        reference_y = vehicle["reference_y"][sample_index]
        reference_yaw = vehicle["reference_yaw"][sample_index]
        reference_point.set_data([reference_x], [reference_y])
        reference_heading.set_data(
            *direction_segment(
                reference_x,
                reference_y,
                reference_yaw,
                heading_length,
            )
        )
        vehicle_heading.set_data(
            *direction_segment(
                vehicle["x"][sample_index],
                vehicle["y"][sample_index],
                vehicle["yaw"][sample_index],
                heading_length,
            )
        )

        status_text.set_text(
            f"Time: {vehicle['time'][sample_index]:.2f} s\n"
            f"Mode: {vehicle['parking_mode'][sample_index]}\n"
            f"Velocity: {vehicle['actual_velocity'][sample_index]:+.3f} m/s\n"
            f"Acceleration: {vehicle['applied_acceleration'][sample_index]:+.3f} m/s²\n"
            f"Position error: {vehicle['position_error'][sample_index]:.4f} m\n"
            f"Lateral error: {vehicle['lateral_error'][sample_index]:+.4f} m\n"
            f"Longitudinal error: {vehicle['longitudinal_error'][sample_index]:+.4f} m\n"
            f"Yaw error: {np.rad2deg(vehicle['yaw_error'][sample_index]):.3f} deg\n"
            f"Steering: {np.rad2deg(steering):.2f} deg"
        )
        return (
            vehicle_trail,
            vehicle_body,
            *wheel_patches,
            rear_axle,
            front_axle,
            cog_marker,
            cog_label,
            vehicle_x_axis,
            vehicle_y_axis,
            vehicle_x_label,
            vehicle_y_label,
            reference_point,
            reference_heading,
            vehicle_heading,
            status_text,
        )

    animation = FuncAnimation(
        figure,
        update,
        frames=len(frame_indices),
        interval=1000.0 / args.fps,
        blit=True,
    )
    figure.tight_layout()
    return figure, animation


def main():
    args = parse_arguments()
    validate_arguments(args)
    reference, vehicle, environment = load_results(args.output_dir)
    frame_indices = make_frame_indices(
        len(vehicle),
        args.fps,
        args.duration,
    )
    figure, animation = create_animation(
        reference,
        vehicle,
        environment,
        frame_indices,
        args,
    )

    args.output_dir.mkdir(parents=True, exist_ok=True)
    animation_path = args.output_dir / args.filename
    animation.save(animation_path, writer=PillowWriter(fps=args.fps))
    plt.close(figure)

    actual_duration = len(frame_indices) / args.fps
    print(f"Saved: {animation_path.resolve()}")
    print(
        f"Frames: {len(frame_indices)}, FPS: {args.fps}, "
        f"duration: {actual_duration:.1f} s"
    )


if __name__ == "__main__":
    main()
