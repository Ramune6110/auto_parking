// Traceability: YAW-EQ-01〜YAW-EQ-08
#pragma once

#include "controllers/trajectory_controller.hpp"

/**
 * @brief 曲率フィードフォワードへ方位誤差フィードバックを加える制御器。
 *
 * 後退時は速度符号に応じて補正舵角の符号を反転し、前進・後退の
 * どちらでも方位誤差が減少する方向へ操舵する。
 */
class YawFeedbackController final : public TrajectoryController {
public:
    YawFeedbackController(double wheelbase, double yaw_error_gain);

    ControlCommand computeCommand(
        const ControllerInput& input) const override;

private:
    double wheelbase_;
    double yaw_error_gain_;
};
