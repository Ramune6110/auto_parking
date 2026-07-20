// Traceability: LAT-EQ-03, LAT-EQ-04, LAT-EQ-10, COORD-EQ-05
#include "controllers/lateral_feedback_controller.hpp"

#include "core/tracking_error.hpp"

#include <cmath>
#include <stdexcept>

LateralFeedbackController::LateralFeedbackController(
    double wheelbase,
    double yaw_error_gain,
    double lateral_error_gain)
    : wheelbase_(wheelbase),
      yaw_error_gain_(yaw_error_gain),
      lateral_error_gain_(lateral_error_gain) {
    if (wheelbase_ <= 0.0) {
        throw std::invalid_argument("wheelbase must be positive");
    }
    if (yaw_error_gain_ < 0.0 || lateral_error_gain_ < 0.0) {
        throw std::invalid_argument("feedback gains must not be negative");
    }
}

ControlCommand LateralFeedbackController::computeCommand(
    const ControllerInput& input) const {
    const VehicleState& current_state = input.current_state;
    const TrajectoryPoint& reference = input.time_reference;
    // LAT-EQ-01, LAT-EQ-02：曲率舵角と方位誤差。
    const double feedforward_steering =
        std::atan(wheelbase_ * reference.curvature);
    const double yaw_error =
        calculateYawError(current_state, reference);
    // LAT-EQ-03 / COORD-EQ-05：参照座標系へ射影した符号付き横偏差。
    const double lateral_error =
        calculateLateralError(current_state, reference);
    const double motion_direction = reference.velocity < 0.0 ? -1.0 : 1.0;

    // LAT-EQ-07, LAT-EQ-08：前後進を考慮した方位補正。
    const double yaw_feedback =
        motion_direction * yaw_error_gain_ * yaw_error;
    // 左側を正とする横偏差には、前進・後退とも負の舵角で応答する。
    // LAT-EQ-04：横偏差に比例する補正舵角。
    const double lateral_feedback =
        -lateral_error_gain_ * lateral_error;

    // LAT-EQ-10：フィードフォワード・方位・横偏差の三項を合成。
    return {
        reference.velocity,
        feedforward_steering + yaw_feedback + lateral_feedback};
}
