// Traceability: LINERR-EQ-01, LINERR-EQ-02, LQR-EQ-06〜LQR-EQ-12
#pragma once

#include "controllers/trajectory_controller.hpp"
#include "core/vehicle_parameters.hpp"

#include <Eigen/Core>

/** @brief LQRの車両条件、評価重み、積分器設定。 */
struct LqrControllerConfig {
    double wheelbase{vehicle_parameters::kWheelbase}; // ホイールベース [m]
    double time_step{0.01};                // 制御周期 [s]
    double lateral_error_weight{4.0};      // 横偏差の評価重み
    double yaw_error_weight{2.0};          // 方位誤差の評価重み
    double steering_weight{1.0};           // 補正舵角の評価重み
    bool enable_integral_action{};         // trueなら横偏差積分を状態へ追加
    double integral_error_weight{0.005};   // 横偏差積分値の評価重み
    double integral_limit{0.3};            // 積分値の絶対上限 [m s]
    double anti_windup_steering_limit{};   // 0以下なら飽和判定を無効化 [rad]
    double minimum_linearization_speed{0.5}; // ゲイン計算用の最低速度 [m/s]
};

/** @brief 線形誤差モデルに基づく通常・積分器付きLQR制御器。 */
class LqrController final : public TrajectoryController {
public:
    /** @param config 正の車両諸元・周期と、非負の評価重みを持つ設定。 */
    explicit LqrController(const LqrControllerConfig& config);

    /**
     * @brief 曲率フィードフォワードとLQR補正を合成して舵角を返す。
     * @param input 現在状態、時刻参照点、軌道探索情報。
     * @return 軌道の速度と、LQRで補正した要求舵角。
     */
    ControlCommand computeCommand(
        const ControllerInput& input) const override;

    void reset() const override;

    /** @return 現在保持している横偏差積分値 [m s]。 */
    double integralError() const noexcept;

private:
    double selectLinearizationVelocity(
        const ControllerInput& input) const;

    LqrControllerConfig config_;
    // 共通インターフェースがconstのため、制御器内部状態だけをmutableで保持する。
    mutable double integral_error_{};
    mutable double previous_time_{};
    mutable bool has_previous_time_{};
    mutable Eigen::RowVector3d cached_gain_{Eigen::RowVector3d::Zero()};
    mutable double cached_velocity_{};
    mutable double cached_feedforward_steering_{};
    mutable bool has_cached_gain_{};
};
