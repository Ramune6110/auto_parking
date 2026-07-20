"""Plot logged theory/state/control quantities. Traceability: IMPL-PLOT-01."""

from argparse import ArgumentParser
from pathlib import Path

import matplotlib
import numpy as np

from parking_environment import draw_parking_environment, load_parking_environment


PROJECT_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_OUTPUT_DIRECTORY = PROJECT_ROOT / "output" / "feedforward" / "forward"

REFERENCE_COLUMNS = {
    "time",
    "s",
    "x",
    "y",
    "curvature",
    "velocity",
    "acceleration",
    "steering",
}

VEHICLE_COLUMNS = {
    "time",
    "x",
    "y",
    "position_error",
    "yaw_error",
    "steering",
    "requested_steering",
    "applied_steering",
    "steering_rate",
    "parking_mode",
    "target_velocity",
    "actual_velocity",
    "requested_acceleration",
    "applied_acceleration",
    "jerk",
    "remaining_distance",
    "reference_velocity",
    "curvature",
    "controller_computation_time_ms",
}


def parse_arguments():
    """コマンドライン引数を読み取る。"""
    parser = ArgumentParser(
        description="Plot the parking trajectory simulation results."
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=DEFAULT_OUTPUT_DIRECTORY,
        help="Directory containing CSV files (default: output/feedforward/forward).",
    )
    parser.add_argument(
        "--show",
        action="store_true",
        help="Open an interactive plot window after saving the PNG.",
    )
    return parser.parse_args()


def load_csv(path, required_columns):
    """CSVを読み込み、グラフ作成に必要な列が存在することを確認する。"""
    if not path.is_file():
        raise FileNotFoundError(
            f"Required CSV file was not found: {path}\n"
            "Run parking_trajectory_sim before plotting the results."
        )

    data = np.genfromtxt(
        path, delimiter=",", names=True, dtype=None, encoding="utf-8"
    )
    data = np.atleast_1d(data)

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
    reference_path = output_directory / "reference_trajectory.csv"
    vehicle_path = output_directory / "vehicle_log.csv"
    reference = load_csv(reference_path, REFERENCE_COLUMNS)
    vehicle = load_csv(vehicle_path, VEHICLE_COLUMNS)
    return reference, vehicle, load_parking_environment(output_directory)


def create_trajectory_figure(reference, vehicle, environment=None):
    """目標軌道と車両軌跡だけを描いた図を作成する。"""
    import matplotlib.pyplot as plt

    figure, trajectory_axis = plt.subplots(figsize=(9, 8))
    draw_parking_environment(trajectory_axis, environment)
    forward = reference["velocity"] > 0.0
    reverse = reference["velocity"] < 0.0
    trajectory_axis.plot(reference["x"][forward], reference["y"][forward],
                         color="#009E73", linestyle="--", linewidth=2,
                         label="Forward reference")
    trajectory_axis.plot(reference["x"][reverse], reference["y"][reverse],
                         color="#D55E00", linestyle="--", linewidth=2,
                         label="Reverse reference")
    trajectory_axis.plot(vehicle["x"], vehicle["y"], "b", label="Vehicle")
    if environment is None:
        trajectory_axis.scatter(
            reference["x"][0], reference["y"][0], c="green", label="Start"
        )
        trajectory_axis.scatter(
            reference["x"][-1], reference["y"][-1], c="red", label="Goal"
        )
    trajectory_axis.set_aspect("equal", adjustable="box")
    trajectory_axis.set_xlabel("World X [m]")
    trajectory_axis.set_ylabel("World Y [m]")
    trajectory_axis.set_title("Reference and vehicle trajectory")
    trajectory_axis.grid(True)
    trajectory_axis.legend()
    figure.tight_layout()
    return figure


def create_time_series_figure(reference, vehicle):
    """7種類の制御・追従時系列を、それぞれ独立した軸に描画する。"""
    import matplotlib.pyplot as plt

    figure, axes = plt.subplots(10, 1, figsize=(12, 32))

    velocity_axis = axes[0]
    velocity_axis.plot(vehicle["time"], vehicle["reference_velocity"],
                       color="tab:blue", label="Trajectory reference")
    velocity_axis.plot(vehicle["time"], vehicle["target_velocity"],
                       color="tab:orange", linestyle="--", label="Control target")
    velocity_axis.plot(vehicle["time"], vehicle["actual_velocity"],
                       color="tab:green", label="Actual")
    velocity_axis.set_ylabel("Velocity [m/s]")
    velocity_axis.set_title("Velocity")
    velocity_axis.grid(True)
    velocity_axis.legend()

    acceleration_axis = axes[1]
    acceleration_axis.plot(vehicle["time"], vehicle["requested_acceleration"],
                           color="tab:orange", linestyle="--", label="Requested")
    acceleration_axis.plot(vehicle["time"], vehicle["applied_acceleration"],
                           color="tab:blue", label="Applied")
    acceleration_axis.set_ylabel("Acceleration [m/s²]")
    acceleration_axis.set_title("Acceleration")
    acceleration_axis.grid(True)
    acceleration_axis.legend()

    jerk_axis = axes[2]
    jerk_axis.plot(vehicle["time"], vehicle["jerk"], color="tab:purple")
    jerk_axis.set_ylabel("Jerk [m/s³]")
    jerk_axis.set_title("Longitudinal jerk")
    jerk_axis.grid(True)

    remaining_axis = axes[3]
    remaining_axis.plot(vehicle["time"], vehicle["remaining_distance"],
                        color="tab:brown")
    remaining_axis.set_ylabel("Remaining [m]")
    remaining_axis.set_title("Remaining distance in active segment")
    remaining_axis.grid(True)

    curvature_axis = axes[4]
    curvature_axis.plot(
        vehicle["time"],
        vehicle["curvature"],
        color="tab:blue",
    )
    curvature_axis.set_ylabel("Curvature [1/m]")
    curvature_axis.set_title("Curvature")
    curvature_axis.grid(True)

    steering_axis = axes[5]
    steering_axis.plot(
        vehicle["time"],
        np.rad2deg(vehicle["requested_steering"]),
        color="tab:orange",
        linestyle="--",
        label="Requested",
    )
    steering_axis.plot(
        vehicle["time"],
        np.rad2deg(vehicle["applied_steering"]),
        color="tab:blue",
        label="Applied",
    )
    steering_axis.set_ylabel("Steering [deg]")
    steering_axis.set_title("Steering angle")
    steering_axis.grid(True)
    steering_axis.legend()

    steering_rate_axis = axes[6]
    steering_rate_axis.plot(
        vehicle["time"],
        np.rad2deg(vehicle["steering_rate"]),
        color="tab:green",
    )
    steering_rate_axis.set_ylabel("Steering rate [deg/s]")
    steering_rate_axis.set_title("Applied steering rate")
    steering_rate_axis.grid(True)

    position_error_axis = axes[7]
    position_error_axis.plot(
        vehicle["time"],
        vehicle["position_error"],
        color="tab:blue",
    )
    position_error_axis.set_ylabel("Position error [m]")
    position_error_axis.set_title("Position error")
    position_error_axis.grid(True)

    yaw_error_axis = axes[8]
    yaw_error_axis.plot(
        vehicle["time"],
        np.rad2deg(vehicle["yaw_error"]),
        color="tab:orange",
    )
    yaw_error_axis.set_ylabel("Yaw error [deg]")
    yaw_error_axis.set_title("Yaw error")
    yaw_error_axis.grid(True)

    computation_axis = axes[9]
    computation_axis.plot(
        vehicle["time"], vehicle["controller_computation_time_ms"],
        color="tab:gray")
    computation_axis.set_ylabel("Compute [ms]")
    computation_axis.set_title("Controller computation time")
    computation_axis.grid(True)

    # 時系列グラフは時間または経路長を横軸として明示する。
    velocity_axis.set_xlabel("Time [s]")
    acceleration_axis.set_xlabel("Time [s]")
    jerk_axis.set_xlabel("Time [s]")
    remaining_axis.set_xlabel("Time [s]")
    curvature_axis.set_xlabel("Time [s]")
    steering_axis.set_xlabel("Time [s]")
    steering_rate_axis.set_xlabel("Time [s]")
    position_error_axis.set_xlabel("Time [s]")
    yaw_error_axis.set_xlabel("Time [s]")
    computation_axis.set_xlabel("Time [s]")

    # 停止状態を薄い背景色で示し、前進・後退の境界を全時系列で共有する。
    modes = np.asarray(vehicle["parking_mode"])
    times = np.asarray(vehicle["time"], dtype=float)
    for mode, color in (("gear_change_stop", "gold"), ("final_stop", "lightgray")):
        active = modes == mode
        if np.any(active):
            start = times[np.flatnonzero(active)[0]]
            end = times[np.flatnonzero(active)[-1]]
            for axis in axes:
                axis.axvspan(start, end, color=color, alpha=0.25)

    figure.tight_layout(h_pad=2.0)
    return figure


def save_figures(trajectory_figure, time_series_figure, output_directory):
    """軌跡図と時系列図をそれぞれPNGへ保存する。"""
    output_directory.mkdir(parents=True, exist_ok=True)
    trajectory_path = output_directory / "trajectory_result.png"
    time_series_path = output_directory / "simulation_result.png"
    trajectory_figure.savefig(trajectory_path, dpi=150)
    time_series_figure.savefig(time_series_path, dpi=150)
    return trajectory_path, time_series_path


def main():
    args = parse_arguments()

    # 通常実行ではGUIに依存せずPNGを生成する。
    if not args.show:
        matplotlib.use("Agg")

    import matplotlib.pyplot as plt

    reference, vehicle, environment = load_results(args.output_dir)
    trajectory_figure = create_trajectory_figure(reference, vehicle, environment)
    time_series_figure = create_time_series_figure(reference, vehicle)
    trajectory_path, time_series_path = save_figures(
        trajectory_figure,
        time_series_figure,
        args.output_dir,
    )
    print(f"Saved: {trajectory_path.resolve()}")
    print(f"Saved: {time_series_path.resolve()}")

    if args.show:
        plt.show()
    else:
        plt.close(trajectory_figure)
        plt.close(time_series_figure)


if __name__ == "__main__":
    main()
