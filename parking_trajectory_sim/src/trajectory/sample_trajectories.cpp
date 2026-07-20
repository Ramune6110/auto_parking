// Traceability: IMPL-SCENARIO-01（前進・後退の再現可能な基準軌道）
#include "trajectory/sample_trajectories.hpp"

#include "trajectory/path_generator.hpp"

namespace {
constexpr double kPi = 3.14159265358979323846;
}

Trajectory createForwardSampleTrajectory(const SampleTrajectoryConfig& config) {
    // 基準ケースは「直線→左旋回→直線」の単純な形状とする。
    PathGenerator path(config.sample_distance, 1.0);
    path.addStraight(5.0);
    path.addArc(4.0, kPi / 2.0);
    path.addStraight(3.0);
    return createVelocityProfile(path.points(), config.velocity_profile, 1.0);
}

Trajectory createReverseSampleTrajectory(const SampleTrajectoryConfig& config) {
    // 後退ケースでは、経路生成方向と速度の両方に負の符号を与える。
    PathGenerator path(config.sample_distance, -1.0);
    path.addStraight(3.0);
    path.addArc(4.0, kPi / 2.0);
    return createVelocityProfile(path.points(), config.velocity_profile, -1.0);
}
