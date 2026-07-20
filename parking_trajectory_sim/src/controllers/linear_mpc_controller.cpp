// Traceability: MPC-EQ-01〜MPC-EQ-15（有限ホライズン横方向MPC）
#include "controllers/linear_mpc_controller.hpp"

#include "core/tracking_error.hpp"
#include "models/linear_error_model.hpp"

#include <Eigen/Cholesky>

#include <algorithm>
#include <cmath>
#include <stdexcept>

LinearMpcController::LinearMpcController(const LinearMpcControllerConfig& config)
    : config_(config), warm_start_(Eigen::VectorXd::Zero(config.prediction_horizon)) {
    if (config_.wheelbase <= 0.0 || config_.time_step <= 0.0 ||
        config_.prediction_horizon <= 0 || config_.lateral_error_weight < 0.0 ||
        config_.yaw_error_weight < 0.0 || config_.terminal_weight_scale <= 0.0 ||
        config_.steering_correction_weight <= 0.0 ||
        config_.maximum_steering_angle <= 0.0 ||
        config_.maximum_steering_rate <= 0.0 ||
        config_.maximum_solver_iterations <= 0 || config_.solver_tolerance <= 0.0 ||
        config_.minimum_linearization_speed <= 0.0) {
        throw std::invalid_argument("invalid linear MPC configuration");
    }
}

ControlCommand LinearMpcController::computeCommand(
    const ControllerInput& input) const {
    const TrajectoryPoint& reference = input.time_reference;
    // MPC-EQ-01：状態x=[横偏差, 方位誤差]を現在姿勢から作る。
    const Eigen::Vector2d state(
        calculateLateralError(input.current_state, reference),
        calculateYawError(input.current_state, reference));
    // MPC-EQ-02, MPC-EQ-03：参照曲率に対応する舵角の周りで、
    // 1 ステップ先の誤差を予測する離散時間モデルを作る。
    const double feedforward_steering =
        std::atan(config_.wheelbase * reference.curvature);
    const LinearErrorModelMatrices model = createLinearErrorModel(
        selectLinearizationVelocity(input), feedforward_steering,
        config_.wheelbase, config_.time_step);

    // MPC-EQ-04：この先 N ステップ分の舵角補正量を最適化変数とする。
    const int horizon = config_.prediction_horizon;
    // MPC-EQ-05, MPC-EQ-06：x1～xNをX=Sx*x0+Su*Uへまとめる予測行列。
    Eigen::MatrixXd state_prediction = Eigen::MatrixXd::Zero(2 * horizon, 2);
    Eigen::MatrixXd input_prediction = Eigen::MatrixXd::Zero(2 * horizon, horizon);
    Eigen::Matrix2d state_power = Eigen::Matrix2d::Identity();
    for (int row = 0; row < horizon; ++row) {
        state_power = state_power * model.state_matrix;
        state_prediction.block<2, 2>(2 * row, 0) = state_power;
        for (int column = 0; column <= row; ++column) {
            // 対角ブロックはB、それより下は直前行のブロックへAを掛ける。
            // Aのべき乗を毎回最初から計算しないため、初心者向けの式構造を
            // 保ったまま計算量を減らせる。
            input_prediction.block<2, 1>(2 * row, column) = row == column
                ? model.input_matrix
                : model.state_matrix *
                    input_prediction.block<2, 1>(2 * (row - 1), column);
        }
    }

    // MPC-EQ-07：各予測状態と補正舵角へ重みを並べる。
    Eigen::MatrixXd state_weight = Eigen::MatrixXd::Zero(2 * horizon, 2 * horizon);
    for (int index = 0; index < horizon; ++index) {
        const double scale = index + 1 == horizon
            ? config_.terminal_weight_scale : 1.0;
        state_weight(2 * index, 2 * index) =
            scale * config_.lateral_error_weight;
        state_weight(2 * index + 1, 2 * index + 1) =
            scale * config_.yaw_error_weight;
    }
    const Eigen::MatrixXd input_weight =
        config_.steering_correction_weight *
        Eigen::MatrixXd::Identity(horizon, horizon);

    // MPC-EQ-08, MPC-EQ-09, MPC-EQ-10：予測式を評価関数へ代入し、QPのHとgを得る。
    const Eigen::MatrixXd hessian =
        input_prediction.transpose() * state_weight * input_prediction +
        input_weight;
    const Eigen::VectorXd gradient =
        input_prediction.transpose() * state_weight * state_prediction * state;
    Eigen::VectorXd sequence = solveConstrainedSequence(
        hessian, gradient, feedforward_steering);

    // MPC-EQ-15：Receding Horizonでは最適列の先頭だけを今周期へ適用する。
    const double steering = feedforward_steering + sequence(0);
    previous_steering_command_ = steering;
    warm_start_.head(horizon - 1) = sequence.tail(horizon - 1);
    warm_start_(horizon - 1) = sequence(horizon - 1);
    last_cost_ = (sequence.transpose() * hessian * sequence)(0, 0) +
        2.0 * gradient.dot(sequence);
    return {reference.velocity, steering};
}

Eigen::VectorXd LinearMpcController::solveConstrainedSequence(
    const Eigen::MatrixXd& hessian,
    const Eigen::VectorXd& gradient,
    double feedforward_steering) const {
    const bool constrained = config_.enable_steering_angle_constraint ||
        config_.enable_steering_rate_constraint;
    if (!constrained) {
        // MPC-EQ-11：制約なし凸二次問題は連立一次方程式HU=-gで解ける。
        last_iterations_ = 1;
        return -hessian.ldlt().solve(gradient);
    }

    Eigen::VectorXd sequence = warm_start_;
    projectSteeringConstraints(sequence, feedforward_steering);
    // MPC-EQ-12：最大固有値の逆数を使うと、凸二次関数の安定な勾配刻みになる。
    // 最大絶対行和は最大固有値の安全な上界になる。毎周期の固有値分解を
    // 避けることで、教育用ソルバーでもシミュレーション時間を抑える。
    const double eigenvalue_upper_bound =
        hessian.cwiseAbs().rowwise().sum().maxCoeff();
    const double step_size = 1.0 / std::max(eigenvalue_upper_bound, 1.0e-12);
    last_iterations_ = config_.maximum_solver_iterations;
    for (int iteration = 1; iteration <= config_.maximum_solver_iterations; ++iteration) {
        const Eigen::VectorXd previous = sequence;
        sequence -= step_size * (hessian * sequence + gradient);
        projectSteeringConstraints(sequence, feedforward_steering);
        if ((sequence - previous).lpNorm<Eigen::Infinity>() <
            config_.solver_tolerance) {
            last_iterations_ = iteration;
            break;
        }
    }
    return sequence;
}

void LinearMpcController::projectSteeringConstraints(
    Eigen::VectorXd& sequence,
    double feedforward_steering) const {
    double previous = previous_steering_command_;
    const double maximum_change =
        config_.maximum_steering_rate * config_.time_step;
    for (Eigen::Index index = 0; index < sequence.size(); ++index) {
        double steering = feedforward_steering + sequence(index);
        // MPC-EQ-13：各予測周期の実舵角を機械限界へ射影する。
        if (config_.enable_steering_angle_constraint) {
            steering = std::clamp(
                steering, -config_.maximum_steering_angle,
                config_.maximum_steering_angle);
        }
        // MPC-EQ-14：直前舵角からの変化を最大操舵速度×周期へ射影する。
        if (config_.enable_steering_rate_constraint) {
            steering = std::clamp(
                steering, previous - maximum_change, previous + maximum_change);
        }
        sequence(index) = steering - feedforward_steering;
        previous = steering;
    }
}

double LinearMpcController::selectLinearizationVelocity(
    const ControllerInput& input) const {
    double velocity = std::abs(input.current_state.velocity) > 1.0e-9
        ? input.current_state.velocity : input.time_reference.velocity;
    const double direction = velocity < 0.0 ? -1.0 : 1.0;
    return direction * std::max(
        std::abs(velocity), config_.minimum_linearization_speed);
}

void LinearMpcController::reset() const {
    warm_start_.setZero();
    previous_steering_command_ = 0.0;
    last_cost_ = 0.0;
    last_iterations_ = 0;
}

double LinearMpcController::lastCost() const noexcept { return last_cost_; }
int LinearMpcController::lastIterations() const noexcept { return last_iterations_; }
double LinearMpcController::previousSteeringCommand() const noexcept {
    return previous_steering_command_;
}
