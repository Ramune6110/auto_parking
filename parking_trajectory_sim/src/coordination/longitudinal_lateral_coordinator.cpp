// Traceability: COOP-EQ-01〜COOP-EQ-05（横追従状態から速度上限を調整）
#include "coordination/longitudinal_lateral_coordinator.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

LongitudinalLateralCoordinator::LongitudinalLateralCoordinator(
    const LongitudinalLateralCoordinatorConfig& config) : config_(config) {
    if (config_.maximum_lateral_acceleration <= 0.0 ||
        config_.lateral_error_gain < 0.0 || config_.yaw_error_gain < 0.0 ||
        config_.steering_limit_speed_ratio <= 0.0 ||
        config_.steering_limit_speed_ratio > 1.0) {
        throw std::invalid_argument("invalid coordination config");
    }
}

CoordinationOutput LongitudinalLateralCoordinator::coordinate(
    const CoordinationInput& input) const {
    const double reference_speed = std::abs(input.reference_velocity);
    const double infinity = std::numeric_limits<double>::infinity();
    // COOP-EQ-01：曲率が大きい区間では横加速度上限から速度を抑える。
    const double curvature_limit = std::abs(input.curvature) > 1.0e-12
        ? std::sqrt(config_.maximum_lateral_acceleration /
                    std::abs(input.curvature))
        : infinity;
    // COOP-EQ-02, COOP-EQ-03：横偏差・方位誤差が大きいほど速度上限を下げる。
    const double lateral_limit = reference_speed /
        (1.0 + config_.lateral_error_gain * std::abs(input.lateral_error));
    const double yaw_limit = reference_speed /
        (1.0 + config_.yaw_error_gain * std::abs(input.yaw_error));
    const double steering_limit = input.steering_limit_active
        ? reference_speed * config_.steering_limit_speed_ratio
        : infinity;
    // COOP-EQ-04, COOP-EQ-05：すべての速度上限の最小値に進行方向符号を戻す。
    const double target_speed = std::min(
        {reference_speed, curvature_limit, lateral_limit, yaw_limit, steering_limit});
    return {
        input.reference_velocity < 0.0 ? -target_speed : target_speed,
        curvature_limit,
        lateral_limit,
        yaw_limit,
        target_speed + 1.0e-12 < reference_speed};
}
