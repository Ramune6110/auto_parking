// Traceability: LINERR-EQ-13〜LINERR-EQ-19
#pragma once

#include <Eigen/Core>

/** @brief 横偏差・方位誤差の2状態離散線形モデル。 */
struct LinearErrorModelMatrices {
    Eigen::Matrix2d state_matrix{Eigen::Matrix2d::Zero()};
    Eigen::Vector2d input_matrix{Eigen::Vector2d::Zero()};
};

/**
 * @brief 参照軌道周りで線形化した誤差モデルを生成する。
 * @param velocity 符号付き線形化速度 [m/s]。
 * @param feedforward_steering 参照曲率に対応する舵角 [rad]。
 * @param wheelbase ホイールベース [m]。
 * @param time_step 離散時間周期 [s]。
 */
LinearErrorModelMatrices createLinearErrorModel(
    double velocity,
    double feedforward_steering,
    double wheelbase,
    double time_step);
