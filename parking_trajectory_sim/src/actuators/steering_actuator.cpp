// Traceability: ACT-EQ-02〜ACT-EQ-08（操舵角・操舵速度・一次遅れ）
#include "actuators/steering_actuator.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

SteeringActuator::SteeringActuator(const SteeringActuatorConfig& config)
    : config_(config) {
    if (!std::isfinite(config_.steering_bias)) {
        throw std::invalid_argument("steering bias must be finite");
    }
    if (config_.enable_angle_limit && config_.max_steering_angle <= 0.0) {
        throw std::invalid_argument("max steering angle must be positive");
    }
    if (config_.enable_rate_limit && config_.max_steering_rate <= 0.0) {
        throw std::invalid_argument("max steering rate must be positive");
    }
    if (config_.enable_first_order_lag && config_.time_constant <= 0.0) {
        throw std::invalid_argument("steering time constant must be positive");
    }
}

SteeringActuatorOutput SteeringActuator::update(
    double requested_steering,
    double dt) {
    if (dt <= 0.0) {
        throw std::invalid_argument("steering actuator dt must be positive");
    }

    double target = requested_steering;
    bool angle_limit_active = false;
    // ACT-EQ-02：要求舵角を機構の最大舵角内へ制限する。
    if (config_.enable_angle_limit) {
        const double limited = std::clamp(
            target,
            -config_.max_steering_angle,
            config_.max_steering_angle);
        angle_limit_active = std::abs(limited - target) > 1.0e-12;
        target = limited;
    }

    // ACT-EQ-05：一次遅れを厳密離散化して遅れ後の目標舵角を求める。
    if (config_.enable_first_order_lag) {
        const double response = 1.0 - std::exp(-dt / config_.time_constant);
        target = applied_steering_ + response * (target - applied_steering_);
    }

    // ACT-EQ-07：前周期の実舵角に対する要求変化量。
    double change = target - applied_steering_;
    bool rate_limit_active = false;
    // 1ステップで動かせる舵角を操舵速度×時間刻みに制限する。
    if (config_.enable_rate_limit) {
        // ACT-EQ-06, ACT-EQ-08：1周期の最大変化量を求め、変化量を制限する。
        const double max_change = config_.max_steering_rate * dt;
        const double limited_change = std::clamp(change, -max_change, max_change);
        rate_limit_active = std::abs(limited_change - change) > 1.0e-12;
        change = limited_change;
    }

    const double previous_steering = applied_steering_;
    applied_steering_ += change;
    if (config_.enable_angle_limit) {
        applied_steering_ = std::clamp(
            applied_steering_,
            -config_.max_steering_angle,
            config_.max_steering_angle);
    }

    // バイアスは、制御器が指令しても取り除けない機械的なずれを表す。
    double actual_steering = applied_steering_ + config_.steering_bias;
    if (config_.enable_angle_limit) {
        const double limited_actual = std::clamp(
            actual_steering,
            -config_.max_steering_angle,
            config_.max_steering_angle);
        angle_limit_active = angle_limit_active ||
            std::abs(limited_actual - actual_steering) > 1.0e-12;
        actual_steering = limited_actual;
    }

    return SteeringActuatorOutput{
        requested_steering,
        actual_steering,
        (applied_steering_ - previous_steering) / dt,
        angle_limit_active,
        rate_limit_active};
}

double SteeringActuator::steering() const noexcept {
    const double actual_steering = applied_steering_ + config_.steering_bias;
    return config_.enable_angle_limit
        ? std::clamp(
            actual_steering,
            -config_.max_steering_angle,
            config_.max_steering_angle)
        : actual_steering;
}
