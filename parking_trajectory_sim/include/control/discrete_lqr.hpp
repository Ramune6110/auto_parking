// Traceability: LQR-EQ-04, LQR-EQ-05
#pragma once

#include <Eigen/Core>

struct DiscreteLqrConfig {
    int maximum_iterations{100000};       // Riccati反復の上限回数
    double convergence_tolerance{1.0e-6}; // 行列Pの収束判定値
};

struct DiscreteLqrResult {
    Eigen::MatrixXd gain;              // 最適フィードバックゲインK
    Eigen::MatrixXd riccati_solution; // 収束したRiccati方程式の解P
    int iterations{};                 // 収束までに要した反復回数
};

/**
 * @brief 離散時間代数Riccati方程式を反復してLQRゲインを求める。
 *
 * A、B、Q、Rのサイズと有限性を検証し、収束しなければ例外を送出する。
 * @param state_matrix 状態遷移行列A。
 * @param input_matrix 入力行列B（入力は舵角1つ）。
 * @param state_weight 状態偏差に対する重みQ。
 * @param input_weight 操舵量に対する重みR。
 * @param config 反復計算の終了条件。
 * @return ゲインK、Riccati解P、反復回数。
 */
DiscreteLqrResult solveDiscreteLqr(
    const Eigen::Matrix2d& state_matrix,
    const Eigen::Vector2d& input_matrix,
    const Eigen::Matrix2d& state_weight,
    double input_weight,
    const DiscreteLqrConfig& config = {});

DiscreteLqrResult solveDiscreteLqr(
    const Eigen::Matrix3d& state_matrix,
    const Eigen::Vector3d& input_matrix,
    const Eigen::Matrix3d& state_weight,
    double input_weight,
    const DiscreteLqrConfig& config = {});
