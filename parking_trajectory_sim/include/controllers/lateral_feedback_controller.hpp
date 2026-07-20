// Traceability: LAT-EQ-03, LAT-EQ-04, LAT-EQ-10
#pragma once

#include "controllers/trajectory_controller.hpp"

/**
 * @brief 曲率、方位誤差、横偏差を使って舵角を計算する制御器。
 *
 * 段階3の方位誤差補正を含み、さらに参照経路の左右位置ずれを
 * 減少させる横偏差比例項を加える。
 */
class LateralFeedbackController final : public TrajectoryController {
public:
    LateralFeedbackController(
        double wheelbase,
        double yaw_error_gain,
        double lateral_error_gain);

    ControlCommand computeCommand(
        const ControllerInput& input) const override;

private:
    double wheelbase_;
    double yaw_error_gain_;
    double lateral_error_gain_;
};
