// Traceability: LQR-EQ-04, LQR-EQ-05（Riccati反復とフィードバックゲイン）
#include "control/discrete_lqr.hpp"

#include <cmath>
#include <stdexcept>

namespace {

// 状態数が2または3と決まっているため、固定サイズ行列で次元を明確にする。
template <int StateCount>
using FixedStateMatrix = Eigen::Matrix<double, StateCount, StateCount>;

template <int StateCount>
using FixedInputVector = Eigen::Matrix<double, StateCount, 1>;

template <int StateCount>
using FixedGain = Eigen::Matrix<double, 1, StateCount>;

template <int StateCount>
FixedGain<StateCount> calculateGain(
    const FixedStateMatrix<StateCount>& state_matrix,
    const FixedInputVector<StateCount>& input_matrix,
    double input_weight,
    const FixedStateMatrix<StateCount>& riccati_solution) {
    // 入力が舵角1つなので、R + B'PB は逆行列ではなくスカラー除算になる。
    // LQR-EQ-05：K=(R+B'PB)^-1 B'PA。入力1次元なので分母はスカラー。
    const double denominator = input_weight +
        (input_matrix.transpose() * riccati_solution * input_matrix)(0, 0);
    if (!std::isfinite(denominator) || denominator <= 1.0e-14) {
        throw std::runtime_error("LQR input Hessian is not positive");
    }
    return input_matrix.transpose() * riccati_solution * state_matrix /
        denominator;
}

template <int StateCount>
DiscreteLqrResult solveFixedSizeDiscreteLqr(
    const FixedStateMatrix<StateCount>& state_matrix,
    const FixedInputVector<StateCount>& input_matrix,
    const FixedStateMatrix<StateCount>& state_weight,
    double input_weight,
    const DiscreteLqrConfig& config) {
    if (!state_matrix.allFinite() || !input_matrix.allFinite() ||
        !state_weight.allFinite() || !std::isfinite(input_weight)) {
        throw std::invalid_argument("discrete LQR matrices must be finite");
    }
    if (input_weight <= 0.0 || config.maximum_iterations <= 0 ||
        config.convergence_tolerance <= 0.0) {
        throw std::invalid_argument("invalid discrete LQR configuration");
    }

    // LQR-EQ-04：P=Qを初期値として離散Riccati方程式を反復する。
    FixedStateMatrix<StateCount> riccati_solution = state_weight;
    for (int iteration = 1;
         iteration <= config.maximum_iterations;
         ++iteration) {
        const FixedGain<StateCount> gain = calculateGain(
            state_matrix,
            input_matrix,
            input_weight,
            riccati_solution);
        FixedStateMatrix<StateCount> next_solution =
            state_matrix.transpose() * riccati_solution * state_matrix -
            state_matrix.transpose() * riccati_solution * input_matrix * gain +
            state_weight;
        // 丸め誤差で失われる対称性を戻し、反復計算を安定させる。
        next_solution = 0.5 * (next_solution + next_solution.transpose()).eval();

        const double maximum_change =
            (next_solution - riccati_solution).cwiseAbs().maxCoeff();
        riccati_solution = next_solution;
        if (maximum_change < config.convergence_tolerance) {
            return {
                calculateGain(
                    state_matrix,
                    input_matrix,
                    input_weight,
                    riccati_solution),
                riccati_solution,
                iteration};
        }
    }
    throw std::runtime_error("discrete Riccati iteration did not converge");
}

}  // namespace

DiscreteLqrResult solveDiscreteLqr(
    const Eigen::Matrix2d& state_matrix,
    const Eigen::Vector2d& input_matrix,
    const Eigen::Matrix2d& state_weight,
    double input_weight,
    const DiscreteLqrConfig& config) {
    // 通常LQR: 状態は横偏差と方位誤差の2つ。
    return solveFixedSizeDiscreteLqr<2>(
        state_matrix, input_matrix, state_weight, input_weight, config);
}

DiscreteLqrResult solveDiscreteLqr(
    const Eigen::Matrix3d& state_matrix,
    const Eigen::Vector3d& input_matrix,
    const Eigen::Matrix3d& state_weight,
    double input_weight,
    const DiscreteLqrConfig& config) {
    // 積分付きLQR: 横偏差積分を加えた3状態。
    return solveFixedSizeDiscreteLqr<3>(
        state_matrix, input_matrix, state_weight, input_weight, config);
}
