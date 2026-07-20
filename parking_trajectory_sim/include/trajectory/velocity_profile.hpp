// Traceability: TRAJ-EQ-09, TRAJ-EQ-11〜TRAJ-EQ-17
#pragma once

#include "core/types.hpp"

struct VelocityProfileConfig {
    double maximum_velocity{1.0};
    double acceleration_limit{0.5};
    double deceleration_limit{0.5};
};

/**
 * @brief 幾何学的な経路へ台形または三角形の速度プロファイルを付与する。
 * @param path 時間情報を持たない幾何学的な経路。
 * @param config 最高速度と加減速度の制限値。
 * @param velocity_sign 前進は +1、後退は -1。
 * @return 時刻、速度、加速度を含む目標軌道。
 */
Trajectory createVelocityProfile(
    const GeometricPath& path,
    const VelocityProfileConfig& config,
    double velocity_sign);
