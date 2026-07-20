// Traceability: LON-EQ-01〜LON-EQ-05, LONLIM-EQ-01〜LONLIM-EQ-04,
// STOP-EQ-01〜STOP-EQ-06
#pragma once

#include "core/longitudinal_types.hpp"

struct VelocityStopControllerConfig {
    double velocity_gain{2.0};
    double position_gain{0.8};
    double maximum_acceleration{0.8};
    double maximum_deceleration{1.0};
    double maximum_jerk{1.5};
    double position_tolerance{0.05};
    double velocity_tolerance{0.03};
    bool enable_acceleration_limit{};
    bool enable_jerk_limit{};
    bool enable_stop_position_control{};
};

class VelocityStopController {
public:
    /** @brief ゲイン、物理制約、停止許容値を検証して保持する。 */
    explicit VelocityStopController(const VelocityStopControllerConfig& config);

    /**
     * @brief 目標速度、実車速、残距離から1周期の適用加速度を求める。
     *
     * 停止位置速度上限→速度P制御→加減速度制約→ジャーク制約の順に処理する。
     */
    LongitudinalCommand computeCommand(
        const LongitudinalControllerInput& input) const;

    /** @brief 区間切り替え時に前周期加速度を0へ戻す。 */
    void reset() const noexcept;

private:
    VelocityStopControllerConfig config_;
    mutable double previous_acceleration_{};
};
