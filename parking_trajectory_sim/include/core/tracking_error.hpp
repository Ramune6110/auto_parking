// Traceability: COORD-EQ-01, COORD-EQ-04, COORD-EQ-05, COORD-EQ-07
#pragma once

#include "core/angle_utils.hpp"
#include "core/types.hpp"

#include <cmath>

/** @brief 参照経路の左側を正として横偏差を計算する。 */
inline double calculateLateralError(
    const VehicleState& state,
    const TrajectoryPoint& reference) {
    const double position_x = state.x - reference.x;
    const double position_y = state.y - reference.y;
    return -std::sin(reference.yaw) * position_x +
        std::cos(reference.yaw) * position_y;
}

/** @brief 参照経路の前方を正として縦偏差を計算する。 */
inline double calculateLongitudinalError(
    const VehicleState& state,
    const TrajectoryPoint& reference) {
    const double position_x = state.x - reference.x;
    const double position_y = state.y - reference.y;
    return std::cos(reference.yaw) * position_x +
        std::sin(reference.yaw) * position_y;
}

/** @brief 参照方位－車両方位を [-pi, pi] へ正規化する。 */
inline double calculateYawError(
    const VehicleState& state,
    const TrajectoryPoint& reference) {
    return normalizeAngle(reference.yaw - state.yaw);
}
