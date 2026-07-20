// Traceability: STANLEY-EQ-01〜STANLEY-EQ-10
#pragma once

#include "controllers/trajectory_controller.hpp"

/** @brief 最近傍点の方位誤差と横偏差を用いるStanley制御器。 */
class StanleyController final : public TrajectoryController {
public:
    /**
     * @param wheelbase ホイールベース [m]。
     * @param lateral_error_gain 横偏差補正の強さ。非負値を指定する。
     * @param softening_velocity 低速時の過大な補正を防ぐ速度項 [m/s]。
     */
    StanleyController(
        double wheelbase,
        double lateral_error_gain,
        double softening_velocity);

    /** @brief 最近傍点の曲率、方位誤差、横偏差から要求舵角を返す。 */
    ControlCommand computeCommand(
        const ControllerInput& input) const override;

private:
    double wheelbase_;
    double lateral_error_gain_;
    double softening_velocity_;
};
