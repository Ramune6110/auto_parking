// Traceability: YAW-EQ-01〜YAW-EQ-08, COORD-EQ-07（方位誤差補正）
#include "controllers/yaw_feedback_controller.hpp"

#include "core/tracking_error.hpp"

#include <cmath>
#include <stdexcept>

YawFeedbackController::YawFeedbackController(
    double wheelbase,
    double yaw_error_gain)
    : wheelbase_(wheelbase), yaw_error_gain_(yaw_error_gain) {
    if (wheelbase_ <= 0.0) {
        throw std::invalid_argument("wheelbase must be positive");
    }
    if (yaw_error_gain_ < 0.0) {
        throw std::invalid_argument("yaw_error_gain must not be negative");
    }
}

ControlCommand YawFeedbackController::computeCommand(
    const ControllerInput& input) const {
    const VehicleState& current_state = input.current_state;
    const TrajectoryPoint& reference = input.time_reference;
    // YAW-EQ-03：参照曲率に対応するフィードフォワード舵角。
    const double feedforward_steering =
        std::atan(wheelbase_ * reference.curvature);
    // YAW-EQ-01, YAW-EQ-02 / COORD-EQ-07：正規化した方位誤差。
    const double yaw_error =
        calculateYawError(current_state, reference);

    // v=0の点では車両が動かないため、符号は前進側としても状態に影響しない。
    const double motion_direction = reference.velocity < 0.0 ? -1.0 : 1.0;
    // YAW-EQ-07：前後進の運動方向を反映した方位補正舵角。
    const double feedback_steering =
        motion_direction * yaw_error_gain_ * yaw_error;

    // YAW-EQ-08：フィードフォワード項と方位補正項を合成する。
    return {
        reference.velocity,
        feedforward_steering + feedback_steering};
}
