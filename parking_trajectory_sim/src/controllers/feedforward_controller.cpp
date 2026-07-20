// Traceability: FF-EQ-03, FF-EQ-04, BICYCLE-EQ-03（曲率から舵角への変換）
#include "controllers/feedforward_controller.hpp"

#include <cmath>
#include <stdexcept>

FeedforwardController::FeedforwardController(double wheelbase)
    : wheelbase_(wheelbase) {
    if (wheelbase_ <= 0.0) {
        throw std::invalid_argument("wheelbase must be positive");
    }
}

ControlCommand FeedforwardController::computeCommand(
    const ControllerInput& input) const {
    // フィードフォワード制御では現在状態を使わず、目標曲率だけから
    // キネマティック自転車モデルの実舵角を求める。
    const TrajectoryPoint& reference = input.time_reference;
    static_cast<void>(input.current_state);
    // FF-EQ-03, FF-EQ-04：曲率舵角と参照速度をそのまま指令する。
    return {
        reference.velocity,
        std::atan(wheelbase_ * reference.curvature)};
}
