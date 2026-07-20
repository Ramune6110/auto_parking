// Traceability: PP-EQ-03〜PP-EQ-12, BICYCLE-EQ-03（注視点幾何と舵角）
#include "controllers/pure_pursuit_controller.hpp"

#include "core/angle_utils.hpp"
#include "trajectory/reference_searcher.hpp"

#include <cmath>
#include <stdexcept>

PurePursuitController::PurePursuitController(
    double wheelbase,
    double lookahead_distance)
    : wheelbase_(wheelbase), lookahead_distance_(lookahead_distance) {
    if (wheelbase_ <= 0.0 || lookahead_distance_ <= 0.0) {
        throw std::invalid_argument(
            "wheelbase and lookahead_distance must be positive");
    }
}

ControlCommand PurePursuitController::computeCommand(
    const ControllerInput& input) const {
    const ReferenceSearcher searcher(input.trajectory);
    const std::size_t target_index = searcher.findLookaheadIndex(
        input.nearest_index, lookahead_distance_);
    const TrajectoryPoint& target = input.trajectory[target_index];

    // PP-EQ-03, PP-EQ-04：車両から注視点までの世界座標差と距離。
    const double dx = target.x - input.current_state.x;
    const double dy = target.y - input.current_state.y;
    const double target_distance = std::hypot(dx, dy);
    if (target_distance <= 1.0e-12) {
        return {input.time_reference.velocity, 0.0};
    }

    // PP-EQ-05, PP-EQ-06：前後進を含む進行方向方位と注視角alpha。
    const double motion_direction =
        input.time_reference.velocity < 0.0 ? -1.0 : 1.0;
    const double motion_yaw = input.current_state.yaw +
        (motion_direction < 0.0 ? 3.14159265358979323846 : 0.0);
    const double alpha = normalizeAngle(std::atan2(dy, dx) - motion_yaw);

    // PP-EQ-10, PP-EQ-11：注視点の幾何から曲率を求め、車体基準へ変換。
    // 後退時も車両前方基準の曲率へ戻すため、進行方向の符号を掛ける。
    const double curvature = motion_direction *
        2.0 * std::sin(alpha) / target_distance;
    // PP-EQ-12 / BICYCLE-EQ-03：曲率をキネマティックモデルの舵角へ変換。
    const double steering = std::atan(wheelbase_ * curvature);
    return {input.time_reference.velocity, steering};
}
