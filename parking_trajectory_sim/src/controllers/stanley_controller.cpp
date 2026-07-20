// Traceability: STANLEY-EQ-02〜STANLEY-EQ-10（方位・横偏差補正）
#include "controllers/stanley_controller.hpp"

#include "core/tracking_error.hpp"

#include <cmath>
#include <stdexcept>

StanleyController::StanleyController(
    double wheelbase,
    double lateral_error_gain,
    double softening_velocity)
    : wheelbase_(wheelbase),
      lateral_error_gain_(lateral_error_gain),
      softening_velocity_(softening_velocity) {
    if (wheelbase_ <= 0.0 || lateral_error_gain_ < 0.0 ||
        softening_velocity_ <= 0.0) {
        throw std::invalid_argument(
            "Stanley parameters must satisfy L>0, k>=0, v_soft>0");
    }
}

ControlCommand StanleyController::computeCommand(
    const ControllerInput& input) const {
    // Stanley法は、現在時刻の参照点ではなく車両に最も近い点を基準にする。
    const TrajectoryPoint& nearest =
        input.trajectory[input.nearest_index];
    const double motion_direction =
        input.time_reference.velocity < 0.0 ? -1.0 : 1.0;
    // STANLEY-EQ-09：最近傍点の曲率からフィードフォワード舵角を求める。
    const double feedforward_steering =
        std::atan(wheelbase_ * nearest.curvature);
    // STANLEY-EQ-03, STANLEY-EQ-04：方位誤差と符号付き横偏差。
    const double yaw_error =
        calculateYawError(input.current_state, nearest);
    const double lateral_error =
        calculateLateralError(input.current_state, nearest);
    // STANLEY-EQ-08：低速時の発散を緩和した横偏差補正。
    const double lateral_steering = -std::atan2(
        lateral_error_gain_ * lateral_error,
        std::abs(input.time_reference.velocity) + softening_velocity_);

    // STANLEY-EQ-10：三つの舵角成分を合成する。
    return {
        input.time_reference.velocity,
        feedforward_steering + motion_direction * yaw_error +
            lateral_steering};
}
