// Traceability: TRAJ-EQ-09, TRAJ-EQ-11〜TRAJ-EQ-17（速度・加速度・時刻）
#include "trajectory/velocity_profile.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

Trajectory createVelocityProfile(
    const GeometricPath& path,
    const VelocityProfileConfig& config,
    double velocity_sign) {
    if (path.size() < 2 || config.maximum_velocity <= 0.0 ||
        config.acceleration_limit <= 0.0 ||
        config.deceleration_limit <= 0.0) {
        throw std::invalid_argument("velocity profile parameters must be positive");
    }
    if (velocity_sign != 1.0 && velocity_sign != -1.0) {
        throw std::invalid_argument("velocity_sign must be +1 or -1");
    }

    const double total_length = path.back().s;
    Trajectory trajectory(path.size());
    // 各点の速度は、最高速度、始点から加速可能な速度、終点までに
    // 減速可能な速度のうち最も小さい値とする。これで始点・終点は0 m/sになる。
    for (std::size_t i = 0; i < path.size(); ++i) {
        // TRAJ-EQ-09：始点から加速度上限内で到達できる速さ。
        const double acceleration_speed = std::sqrt(std::max(
            0.0, 2.0 * config.acceleration_limit * path[i].s));
        // TRAJ-EQ-11：終点で停止できる減速側の速さ上限。
        const double deceleration_speed = std::sqrt(std::max(
            0.0, 2.0 * config.deceleration_limit *
                (total_length - path[i].s)));
        // TRAJ-EQ-12, TRAJ-EQ-13：三つの上限の最小値へ前後進符号を付ける。
        const double speed = std::min({
            config.maximum_velocity, acceleration_speed, deceleration_speed});
        trajectory[i] = {0.0, path[i].s, path[i].x, path[i].y,
            path[i].yaw, path[i].curvature, velocity_sign * speed, 0.0};
    }

    // 区間平均速度から通過時刻を、v^2-v0^2=2aΔs から加速度を求める。
    for (std::size_t i = 1; i < trajectory.size(); ++i) {
        const double distance = trajectory[i].s - trajectory[i - 1].s;
        const double speed_sum = std::abs(trajectory[i - 1].velocity) +
            std::abs(trajectory[i].velocity);
        if (distance <= 0.0 || speed_sum <= 1.0e-12) {
            throw std::runtime_error("trajectory contains an invalid interval");
        }
        // TRAJ-EQ-14, TRAJ-EQ-15：区間平均速さから通過時間を求める。
        trajectory[i].time = trajectory[i - 1].time +
            2.0 * distance / speed_sum;
        // TRAJ-EQ-16, TRAJ-EQ-17：速度二乗差から符号付き区間加速度を求める。
        trajectory[i].acceleration = velocity_sign *
            (trajectory[i].velocity * trajectory[i].velocity -
             trajectory[i - 1].velocity * trajectory[i - 1].velocity) /
            (2.0 * distance);
    }
    // 始点には直後の区間で使用する加速度を持たせる。
    trajectory.front().acceleration = trajectory[1].acceleration;
    return trajectory;
}
