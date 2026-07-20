// Traceability: BICYCLE-EQ-03, BICYCLE-EQ-06, BICYCLE-EQ-09〜BICYCLE-EQ-13
#pragma once

#include "core/types.hpp"

/** @brief 横滑りを無視した非線形キネマティック自転車モデル。 */
class KinematicBicycle {
public:
    /**
     * @param wheelbase ホイールベース [m]。正の値を指定する。
     * @param initial_state 世界座標系での初期車両状態。
     */
    KinematicBicycle(double wheelbase, const VehicleState& initial_state);

    /**
     * @brief 入力を一定と仮定して車両状態を1周期進める。
     * @param velocity 符号付き前後方向速度 [m/s]。正は前進、負は後退。
     * @param steering_angle 前輪舵角 [rad]。正は左、負は右。
     * @param dt 更新時間 [s]。正の値を指定する。
     */
    void step(double velocity, double steering_angle, double dt);

    /**
     * @brief 一定加速度を1周期積分し、平均車速で車両姿勢を更新する。
     * @param acceleration 車体前後方向の符号付き加速度 [m/s^2]。
     */
    void stepWithAcceleration(
        double acceleration, double steering_angle, double dt);

    /** @brief 停止許容値内へ入った車速を厳密な0へ確定する。 */
    void stop() noexcept;

    const VehicleState& state() const;

private:
    double wheelbase_;
    VehicleState state_;
};
