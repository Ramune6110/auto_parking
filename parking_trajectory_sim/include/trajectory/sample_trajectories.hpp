// Traceability: IMPL-SCENARIO-01
#pragma once

#include "core/types.hpp"
#include "trajectory/velocity_profile.hpp"

struct SampleTrajectoryConfig {
    double sample_distance{0.02};
    VelocityProfileConfig velocity_profile;
};

/** @brief 学習用の前進軌道（直線、左円弧、直線）を生成する。 */
Trajectory createForwardSampleTrajectory(const SampleTrajectoryConfig& config);

/** @brief 後退時の符号と運動を確認する学習用軌道を生成する。 */
Trajectory createReverseSampleTrajectory(const SampleTrajectoryConfig& config);
