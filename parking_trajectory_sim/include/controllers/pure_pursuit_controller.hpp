// Traceability: PP-EQ-01〜PP-EQ-12
#pragma once

#include "controllers/trajectory_controller.hpp"

/** @brief 車両から一定経路距離先の注視点を追いかける制御器。 */
class PurePursuitController final : public TrajectoryController {
public:
    /**
     * @param wheelbase ホイールベース [m]。正の値を指定する。
     * @param lookahead_distance 最近傍点から注視点までの経路距離 [m]。
     */
    PurePursuitController(double wheelbase, double lookahead_distance);

    /** @brief 注視点を車両座標系で捉え、そこへ向かう要求舵角を返す。 */
    ControlCommand computeCommand(
        const ControllerInput& input) const override;

private:
    double wheelbase_;
    double lookahead_distance_;
};
