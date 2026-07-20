// Traceability: FF-EQ-03, FF-EQ-04
#pragma once

#include "controllers/trajectory_controller.hpp"

/**
 * @brief 参照曲率を舵角へ変換するフィードフォワード制御器。
 *
 * 現在の追従誤差は補正しない。今後追加するフィードバック制御器との
 * 比較基準として使用する。
 */
class FeedforwardController final : public TrajectoryController {
public:
    explicit FeedforwardController(double wheelbase);

    ControlCommand computeCommand(
        const ControllerInput& input) const override;

private:
    double wheelbase_;
};
