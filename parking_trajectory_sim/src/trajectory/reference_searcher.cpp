// Traceability: PP-EQ-01, PP-EQ-02, STANLEY-EQ-01, IMPL-SEARCH-01
#include "trajectory/reference_searcher.hpp"

#include <limits>
#include <stdexcept>

ReferenceSearcher::ReferenceSearcher(const Trajectory& trajectory)
    : trajectory_(trajectory) {
    if (trajectory_.empty()) {
        throw std::invalid_argument("trajectory must not be empty");
    }
}

std::size_t ReferenceSearcher::findNearestIndex(
    const VehicleState& state,
    std::size_t previous_index) const {
    if (previous_index >= trajectory_.size()) {
        throw std::out_of_range("previous_index is outside trajectory");
    }

    // 探索開始位置を前回値に限定し、参照点が過去側へ戻るのを防ぐ。
    std::size_t nearest_index = previous_index;
    double nearest_squared_distance =
        std::numeric_limits<double>::infinity();
    for (std::size_t i = previous_index; i < trajectory_.size(); ++i) {
        const double dx = trajectory_[i].x - state.x;
        const double dy = trajectory_[i].y - state.y;
        // 大小比較だけなので平方根を計算する必要はない。
        const double squared_distance = dx * dx + dy * dy;
        if (squared_distance < nearest_squared_distance) {
            nearest_squared_distance = squared_distance;
            nearest_index = i;
        }
    }
    return nearest_index;
}

std::size_t ReferenceSearcher::findLookaheadIndex(
    std::size_t nearest_index,
    double lookahead_distance) const {
    if (nearest_index >= trajectory_.size()) {
        throw std::out_of_range("nearest_index is outside trajectory");
    }
    if (lookahead_distance <= 0.0) {
        throw std::invalid_argument("lookahead_distance must be positive");
    }

    // 最近傍点から経路長sを先読みし、最初に到達した点を返す。
    const double target_s =
        trajectory_[nearest_index].s + lookahead_distance;
    for (std::size_t i = nearest_index; i < trajectory_.size(); ++i) {
        if (trajectory_[i].s >= target_s) {
            return i;
        }
    }
    // 軌道終端を越える先読みは、最後の点へ固定する。
    return trajectory_.size() - 1;
}
