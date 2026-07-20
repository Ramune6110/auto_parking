// Traceability: IMPL-SAMPLER-01（時刻に対応する軌道点の選択）
#include "trajectory/trajectory_sampler.hpp"

#include <algorithm>
#include <stdexcept>

TrajectorySampler::TrajectorySampler(const Trajectory& trajectory)
    : trajectory_(trajectory) {
    if (trajectory_.empty()) {
        throw std::invalid_argument("trajectory must not be empty");
    }
    for (std::size_t i = 1; i < trajectory_.size(); ++i) {
        if (trajectory_[i].time <= trajectory_[i - 1].time) {
            throw std::invalid_argument(
                "trajectory time must be strictly increasing");
        }
    }
}

TrajectoryPoint TrajectorySampler::sampleByTime(double time) const {
    // 軌道の時間範囲外では、外挿せず端点を返す。
    if (time <= trajectory_.front().time) {
        return trajectory_.front();
    }
    if (time >= trajectory_.back().time) {
        return trajectory_.back();
    }

    // 指定時刻を挟む2点を二分探索し、その間を線形補間する。
    const auto upper = std::upper_bound(
        trajectory_.begin(), trajectory_.end(), time,
        [](double value, const TrajectoryPoint& point) {
            return value < point.time;
        });
    const auto lower = upper - 1;
    const double ratio =
        (time - lower->time) / (upper->time - lower->time);
    const auto interpolate = [ratio](double first, double second) {
        return first + ratio * (second - first);
    };

    TrajectoryPoint point;
    point.time = time;
    point.s = interpolate(lower->s, upper->s);
    point.x = interpolate(lower->x, upper->x);
    point.y = interpolate(lower->y, upper->y);
    point.yaw = interpolate(lower->yaw, upper->yaw);

    // 曲率と加速度は区間内で一定とみなし、直後の区間値を採用する。
    point.curvature = upper->curvature;
    point.velocity = interpolate(lower->velocity, upper->velocity);
    point.acceleration = upper->acceleration;
    return point;
}
