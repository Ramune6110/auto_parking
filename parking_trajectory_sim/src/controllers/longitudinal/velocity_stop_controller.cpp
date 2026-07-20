// Traceability: LON-EQ-01〜LON-EQ-05, LONLIM-EQ-01〜LONLIM-EQ-04,
// STOP-EQ-01〜STOP-EQ-06
#include "controllers/longitudinal/velocity_stop_controller.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

VelocityStopController::VelocityStopController(
    const VelocityStopControllerConfig& config) : config_(config) {
    if (config_.velocity_gain <= 0.0 || config_.position_gain <= 0.0 ||
        config_.maximum_acceleration <= 0.0 ||
        config_.maximum_deceleration <= 0.0 || config_.maximum_jerk <= 0.0 ||
        config_.position_tolerance < 0.0 ||
        config_.velocity_tolerance < 0.0) {
        throw std::invalid_argument("invalid longitudinal controller config");
    }
}

LongitudinalCommand VelocityStopController::computeCommand(
    const LongitudinalControllerInput& input) const {
    if (input.time_step <= 0.0) {
        throw std::invalid_argument("longitudinal time step must be positive");
    }

    LongitudinalCommand output;
    output.reference_velocity = input.reference_velocity;
    output.remaining_distance = std::max(0.0, input.remaining_distance);
    const double direction = input.reference_velocity < 0.0 ? -1.0 : 1.0;
    double target_speed = std::abs(input.reference_velocity);
    output.stopping_velocity_limit = target_speed;

    if (config_.enable_stop_position_control) {
        // STOP-EQ-02：最大減速度で残距離内に停止できる速度上限。
        output.stopping_velocity_limit = std::sqrt(
            2.0 * config_.maximum_deceleration * output.remaining_distance);
        // STOP-EQ-03, STOP-EQ-04：終端付近では停止距離上限と位置比例上限の
        // 小さい方を選び、目標速度を徐々に0へ近づける。
        const double position_speed_limit =
            config_.position_gain * output.remaining_distance;
        const double limited_speed = std::min(
            {target_speed, output.stopping_velocity_limit, position_speed_limit});
        output.stop_position_control_active =
            limited_speed + 1.0e-12 < target_speed;
        target_speed = limited_speed;
    }
    output.target_velocity = direction * target_speed;

    // LON-EQ-01, LON-EQ-02：目標速度と実車速の差をP制御で加速度へ変換。
    output.velocity_error = output.target_velocity - input.current_velocity;
    output.requested_acceleration =
        config_.velocity_gain * output.velocity_error;
    double acceleration = output.requested_acceleration;

    if (config_.enable_acceleration_limit) {
        // LONLIM-EQ-01：駆動側と制動側で異なる加速度上下限を適用する。
        const double limited = std::clamp(
            acceleration,
            -config_.maximum_deceleration,
            config_.maximum_acceleration);
        output.acceleration_limit_active =
            std::abs(limited - acceleration) > 1.0e-12;
        acceleration = limited;
    }

    if (config_.enable_jerk_limit) {
        // LONLIM-EQ-02〜LONLIM-EQ-04：1周期に許す加速度変化量を
        // 最大ジャーク×制御周期に制限する。
        const double maximum_change = config_.maximum_jerk * input.time_step;
        const double limited = previous_acceleration_ + std::clamp(
            acceleration - previous_acceleration_, -maximum_change, maximum_change);
        output.jerk_limit_active =
            std::abs(limited - acceleration) > 1.0e-12;
        acceleration = limited;
    }

    output.applied_acceleration = acceleration;
    output.jerk = (acceleration - previous_acceleration_) / input.time_step;
    previous_acceleration_ = acceleration;
    // STOP-EQ-05, STOP-EQ-06：位置と速度の両方が許容内に入って初めて停止完了。
    output.stop_condition_satisfied =
        output.remaining_distance <= config_.position_tolerance &&
        std::abs(input.current_velocity) <= config_.velocity_tolerance;
    return output;
}

void VelocityStopController::reset() const noexcept {
    previous_acceleration_ = 0.0;
}

