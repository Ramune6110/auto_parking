// Traceability: COOP-EQ-01〜COOP-EQ-05
#pragma once

#include "core/longitudinal_types.hpp"

struct LongitudinalLateralCoordinatorConfig {
    double maximum_lateral_acceleration{0.8};
    double lateral_error_gain{1.0};
    double yaw_error_gain{1.0};
    double steering_limit_speed_ratio{0.5};
};

class LongitudinalLateralCoordinator {
public:
    /** @brief 曲率・誤差から速度上限を作るための正の設定値を保持する。 */
    explicit LongitudinalLateralCoordinator(
        const LongitudinalLateralCoordinatorConfig& config);
    /**
     * @brief 横追従状態ごとの速度上限を計算し、最も厳しい上限を返す。
     * @note 舵角は変更せず、縦制御器へ渡す速度目標だけを調整する。
     */
    CoordinationOutput coordinate(const CoordinationInput& input) const;
private:
    LongitudinalLateralCoordinatorConfig config_;
};
