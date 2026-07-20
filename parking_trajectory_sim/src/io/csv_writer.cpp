// Traceability: IMPL-CSV-01（理論量・制御量・状態量のログ保存）
#include "io/csv_writer.hpp"

#include <cerrno>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <stdexcept>

#ifdef _WIN32
#include <direct.h>
#else
#include <sys/stat.h>
#endif

namespace {

// C++17のfilesystemへ依存せず、WindowsとPOSIXの双方で1階層を作成する。
void createSingleDirectory(const std::string& directory) {
#ifdef _WIN32
    const int result = _mkdir(directory.c_str());
#else
    const int result = mkdir(directory.c_str(), 0755);
#endif
    if (result != 0 && errno != EEXIST) {
        throw std::runtime_error(
            "cannot create output directory " + directory);
    }
}

void createOutputDirectory(const std::string& directory) {
    // パスを区切りごとに分解し、親ディレクトリから順番に作成する。
    std::string partial;
    for (std::size_t i = 0; i < directory.size(); ++i) {
        const char character = directory[i];
        partial += character;
        const bool is_separator = character == '/' || character == '\\';
        const bool is_drive_prefix = i == 2 && directory[1] == ':';
        if (is_separator && partial.size() > 1 && !is_drive_prefix) {
            partial.pop_back();
            createSingleDirectory(partial);
            partial += character;
        }
    }
    if (!partial.empty() && partial.back() != '/' && partial.back() != '\\') {
        createSingleDirectory(partial);
    }
}

std::string makeOutputPath(
    const std::string& directory,
    const std::string& filename) {
#ifdef _WIN32
    constexpr char separator = '\\';
#else
    constexpr char separator = '/';
#endif
    return directory.empty()
        ? filename
        : directory + separator + filename;
}

void writeReferenceTrajectory(
    const std::string& filename,
    const Trajectory& trajectory,
    double wheelbase) {
    std::ofstream file(filename);
    if (!file) {
        throw std::runtime_error("cannot open " + filename);
    }

    file << "time,s,x,y,yaw,curvature,velocity,acceleration,steering\n";
    // 再解析時に丸め誤差を増やさないよう、十分な有効桁を保存する。
    file << std::setprecision(12);
    for (const TrajectoryPoint& point : trajectory) {
        const double steering =
            std::atan(wheelbase * point.curvature);
        file << point.time << ',' << point.s << ',' << point.x << ','
             << point.y << ',' << point.yaw << ',' << point.curvature << ','
             << point.velocity << ',' << point.acceleration << ','
             << steering << '\n';
    }
}

void writeVehicleLog(
    const std::string& filename,
    const std::vector<SimulationSample>& samples) {
    std::ofstream file(filename);
    if (!file) {
        throw std::runtime_error("cannot open " + filename);
    }

    file << "time,x,y,yaw,velocity,acceleration,curvature,steering,"
            "requested_steering,applied_steering,steering_rate,"
            "angle_limit_active,rate_limit_active,"
            "reference_x,reference_y,reference_yaw,position_error,"
            "lateral_error,longitudinal_error,yaw_error,parking_mode,"
            "segment_index,mode_elapsed_time,state_transition,controller_reset,"
            "reference_velocity,target_velocity,actual_velocity,velocity_error,"
            "requested_acceleration,applied_acceleration,jerk,remaining_distance,"
            "stopping_velocity_limit,acceleration_limit_active,jerk_limit_active,"
            "stop_position_control_active,stop_condition_satisfied,"
            "curvature_speed_limit,lateral_error_speed_limit,yaw_error_speed_limit,"
            "coordination_limit_active,controller_computation_time_ms\n";
    // 上の既存列との互換性を維持しつつ、縦制御列は末尾へ追加する。
    file << std::setprecision(12);
    for (const SimulationSample& sample : samples) {
        // steering列は既存の描画スクリプトとの互換性のため適用舵角を出力する。
        file << sample.time << ','
             << sample.vehicle_state.x << ','
             << sample.vehicle_state.y << ','
             << sample.vehicle_state.yaw << ','
             << sample.command.target_velocity << ','
             << sample.reference.acceleration << ','
             << sample.reference.curvature << ','
             << sample.applied_steering << ','
             << sample.command.target_steering << ','
             << sample.applied_steering << ','
             << sample.steering_rate << ','
             << sample.angle_limit_active << ','
             << sample.rate_limit_active << ','
             << sample.reference.x << ','
             << sample.reference.y << ','
             << sample.reference.yaw << ','
             << sample.position_error << ','
             << sample.lateral_error << ','
             << sample.longitudinal_error << ','
             << sample.yaw_error << ','
             << parkingModeName(sample.parking_mode) << ','
             << sample.segment_index << ','
             << sample.mode_elapsed_time << ','
             << sample.state_transition << ','
             << sample.controller_reset << ','
             << sample.longitudinal.reference_velocity << ','
             << sample.longitudinal.target_velocity << ','
             << sample.vehicle_state.velocity << ','
             << sample.longitudinal.velocity_error << ','
             << sample.longitudinal.requested_acceleration << ','
             << sample.longitudinal.applied_acceleration << ','
             << sample.longitudinal.jerk << ','
             << sample.longitudinal.remaining_distance << ','
             << sample.longitudinal.stopping_velocity_limit << ','
             << sample.longitudinal.acceleration_limit_active << ','
             << sample.longitudinal.jerk_limit_active << ','
             << sample.longitudinal.stop_position_control_active << ','
             << sample.longitudinal.stop_condition_satisfied << ','
             << sample.coordination.curvature_speed_limit << ','
             << sample.coordination.lateral_error_speed_limit << ','
             << sample.coordination.yaw_error_speed_limit << ','
             << sample.coordination.coordination_limit_active << ','
             << sample.controller_computation_time_ms << '\n';
    }
}

void writeParkingEnvironment(
    const std::string& filename,
    const ParkingEnvironment& environment) {
    if (!environment.enabled) {
        return;
    }
    std::ofstream file(filename);
    if (!file) {
        throw std::runtime_error("cannot open " + filename);
    }
    file << "record,index,min_x,max_x,min_y,max_y,x,y,yaw\n"
         << std::setprecision(12);
    const AxisAlignedRectangle& space = environment.parking_space;
    file << "parking_space,0," << space.minimum_x << ',' << space.maximum_x
         << ',' << space.minimum_y << ',' << space.maximum_y << ",nan,nan,nan\n";
    for (std::size_t index = 0; index < environment.obstacles.size(); ++index) {
        const AxisAlignedRectangle& obstacle = environment.obstacles[index];
        file << "obstacle," << index << ',' << obstacle.minimum_x << ','
             << obstacle.maximum_x << ',' << obstacle.minimum_y << ','
             << obstacle.maximum_y << ",nan,nan,nan\n";
    }
    const auto writePose = [&file](const char* name, const VehicleState& pose) {
        file << name << ",0,nan,nan,nan,nan," << pose.x << ',' << pose.y
             << ',' << pose.yaw << '\n';
    };
    writePose("initial_point", environment.initial_pose);
    writePose("reversal_point", environment.reversal_pose);
    writePose("target_point", environment.target_pose);
}

}  // namespace

void writeSimulationCsvFiles(
    const SimulationResult& result,
    const std::string& output_directory,
    double wheelbase) {
    if (wheelbase <= 0.0) {
        throw std::invalid_argument("wheelbase must be positive");
    }
    createOutputDirectory(output_directory);
    writeReferenceTrajectory(
        makeOutputPath(
            output_directory, "reference_trajectory.csv"),
        result.reference_trajectory,
        wheelbase);
    writeVehicleLog(
        makeOutputPath(output_directory, "vehicle_log.csv"),
        result.samples);
    writeParkingEnvironment(
        makeOutputPath(output_directory, "parking_environment.csv"),
        result.parking_environment);
}
