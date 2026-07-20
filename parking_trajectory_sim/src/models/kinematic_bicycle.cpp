// Traceability: BICYCLE-EQ-03, BICYCLE-EQ-06, BICYCLE-EQ-09〜BICYCLE-EQ-13
#include "models/kinematic_bicycle.hpp"

#include <cmath>
#include <stdexcept>

KinematicBicycle::KinematicBicycle(
    double wheelbase, const VehicleState& initial_state)
    : wheelbase_(wheelbase), state_(initial_state) {
    if (wheelbase_ <= 0.0) {
        throw std::invalid_argument("wheelbase must be positive");
    }
}

void KinematicBicycle::step(double velocity, double steering_angle, double dt) {
    if (dt <= 0.0) {
        throw std::invalid_argument("dt must be positive");
    }

    // BICYCLE-EQ-06：速度と舵角からヨーレートを求める。
    const double yaw_rate = velocity * std::tan(steering_angle) / wheelbase_;
    // 式（BICYCLE-EQ-09〜13）の円弧積分を使い、除算が不安定な極小旋回だけ
    // その極限である直進式へ切り替える。
    if (std::abs(yaw_rate) < 1.0e-12) {
        // BICYCLE-EQ-13：ヨーレートがほぼ0のときの直進位置更新。
        state_.x += velocity * std::cos(state_.yaw) * dt;
        state_.y += velocity * std::sin(state_.yaw) * dt;
        state_.yaw += yaw_rate * dt;
    } else {
        // BICYCLE-EQ-09：1周期後の方位を更新する。
        const double next_yaw = state_.yaw + yaw_rate * dt;
        // BICYCLE-EQ-11, BICYCLE-EQ-12：円弧運動を解析積分して位置を更新する。
        state_.x += velocity * (std::sin(next_yaw) - std::sin(state_.yaw)) / yaw_rate;
        state_.y -= velocity * (std::cos(next_yaw) - std::cos(state_.yaw)) / yaw_rate;
        state_.yaw = next_yaw;
    }
    state_.velocity = velocity;
}

void KinematicBicycle::stepWithAcceleration(
    double acceleration, double steering_angle, double dt) {
    if (dt <= 0.0) {
        throw std::invalid_argument("dt must be positive");
    }
    // LON-EQ-04：一定加速度を仮定して次周期の実車速を求める。
    const double next_velocity = state_.velocity + acceleration * dt;
    // LON-EQ-05：周期中の平均車速で移動量を評価すると、加速中の距離を
    // 現在速度だけで計算するより正確になる。
    const double average_velocity = 0.5 * (state_.velocity + next_velocity);
    step(average_velocity, steering_angle, dt);
    state_.velocity = next_velocity;
}

void KinematicBicycle::stop() noexcept {
    state_.velocity = 0.0;
}

const VehicleState& KinematicBicycle::state() const {
    return state_;
}
