// Traceability: IMPL-SCENARIO-SAMPLE-01（連続する前進・後退軌道）
#include "scenario/sample_parking_scenario.hpp"

#include "trajectory/path_generator.hpp"
#include "trajectory/velocity_profile.hpp"

#include <cmath>
#include <utility>

namespace {

Trajectory transformTrajectory(
    const Trajectory& local,
    const VehicleState& origin) {
    Trajectory transformed = local;
    const double cosine = std::cos(origin.yaw);
    const double sine = std::sin(origin.yaw);
    for (TrajectoryPoint& point : transformed) {
        const double local_x = point.x;
        const double local_y = point.y;
        point.x = origin.x + cosine * local_x - sine * local_y;
        point.y = origin.y + sine * local_x + cosine * local_y;
        point.yaw += origin.yaw;
    }
    return transformed;
}

}  // namespace

ParkingScenario createForwardStopReverseScenario(
    const SampleTrajectoryConfig& config,
    double gear_change_wait_time,
    double final_stop_time) {
    constexpr double kPi = 3.14159265358979323846;
    // 道路左側から前進し、60度の切り返し姿勢を作る。
    PathGenerator forward_path(config.sample_distance, 1.0);
    forward_path.addStraight(3.0);
    forward_path.addArc(4.0, kPi / 3.0, 1.0);
    Trajectory forward = transformTrajectory(
        createVelocityProfile(
            forward_path.points(), config.velocity_profile, 1.0),
        VehicleState{-6.0, 1.0, 0.0});

    // 後退右旋回で方位を90度へ揃え、駐車枠内へ直線後退する。
    PathGenerator reverse_path(config.sample_distance, -1.0);
    reverse_path.addArc(4.0, kPi / 6.0, -1.0);
    reverse_path.addStraight(4.0);
    const VehicleState reverse_origin{
        forward.back().x, forward.back().y, forward.back().yaw};
    Trajectory reverse = transformTrajectory(
        createVelocityProfile(
            reverse_path.points(), config.velocity_profile, -1.0),
        reverse_origin);
    const VehicleState target{
        reverse.back().x, reverse.back().y, reverse.back().yaw};
    return ParkingScenario{
        {{MotionDirection::Forward, std::move(forward)},
         {MotionDirection::Reverse, std::move(reverse)}},
        gear_change_wait_time,
        final_stop_time,
        ParkingEnvironment{
            true,
            {-1.15, 1.15, -4.0, 1.35},
            {{-3.35, -1.35, -3.8, 0.6},
             {1.35, 3.35, -3.8, 0.6}},
            {-6.0, 1.0, 0.0},
            reverse_origin,
            target}};
}

ParkingScenario createSimpleForwardStopReverseScenario(
    const SampleTrajectoryConfig& config,
    double gear_change_wait_time,
    double final_stop_time) {
    Trajectory forward = createForwardSampleTrajectory(config);
    const Trajectory reverse_local = createReverseSampleTrajectory(config);
    const VehicleState reverse_origin{
        forward.back().x, forward.back().y, forward.back().yaw};
    Trajectory reverse = transformTrajectory(reverse_local, reverse_origin);
    return ParkingScenario{
        {{MotionDirection::Forward, std::move(forward)},
         {MotionDirection::Reverse, std::move(reverse)}},
        gear_change_wait_time,
        final_stop_time,
        {}};
}
