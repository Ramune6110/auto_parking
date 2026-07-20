// Traceability: LINERR-EQ-01, LINERR-EQ-02, LQR-EQ-06〜LQR-EQ-12
#include "controllers/lqr_controller.hpp"

#include "control/discrete_lqr.hpp"
#include "core/tracking_error.hpp"
#include "models/linear_error_model.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

LqrController::LqrController(const LqrControllerConfig& config)
    : config_(config) {
    if (config_.wheelbase <= 0.0 || config_.time_step <= 0.0 ||
        config_.lateral_error_weight < 0.0 ||
        config_.yaw_error_weight < 0.0 ||
        config_.steering_weight <= 0.0 ||
        config_.integral_error_weight < 0.0 ||
        config_.integral_limit <= 0.0 ||
        config_.minimum_linearization_speed <= 0.0) {
        throw std::invalid_argument("invalid LQR controller configuration");
    }
}

ControlCommand LqrController::computeCommand(
    const ControllerInput& input) const {
    // 時刻が巻き戻った場合は新しいシミュレーションとみなし、内部状態を初期化する。
    if (has_previous_time_ && input.time + 1.0e-12 < previous_time_) {
        integral_error_ = 0.0;
        has_previous_time_ = false;
    }

    // 目標曲率からフィードフォワード舵角を求め、その周りで誤差モデルを線形化する。
    const TrajectoryPoint& reference = input.time_reference;
    // LINERR-EQ-09, LINERR-EQ-10：参照曲率の舵角を線形化動作点とする。
    const double feedforward_steering =
        std::atan(config_.wheelbase * reference.curvature);
    const double linearization_velocity =
        selectLinearizationVelocity(input);
    const LinearErrorModelMatrices model = createLinearErrorModel(
        linearization_velocity,
        feedforward_steering,
        config_.wheelbase,
        config_.time_step);
    const double lateral_error =
        calculateLateralError(input.current_state, reference);
    const double yaw_error =
        calculateYawError(input.current_state, reference);
    // 毎周期のRiccati反復を避け、動作点が十分変化したときだけゲインを更新する。
    const bool update_gain = !has_cached_gain_ ||
        std::abs(linearization_velocity - cached_velocity_) >= 0.05 ||
        std::abs(feedforward_steering - cached_feedforward_steering_) >= 0.01;

    double correction = 0.0;
    if (!config_.enable_integral_action) {
        // 通常LQRは [横偏差, 方位誤差] の2状態をフィードバックする。
        if (update_gain) {
            Eigen::Matrix2d state_weight = Eigen::Matrix2d::Zero();
            state_weight.diagonal() <<
                config_.lateral_error_weight,
                config_.yaw_error_weight;
            const DiscreteLqrResult result = solveDiscreteLqr(
                model.state_matrix,
                model.input_matrix,
                state_weight,
                config_.steering_weight);
            cached_gain_.head<2>() = result.gain;
        }
        // LQR-EQ-01, LQR-EQ-06：誤差状態x=[e_y,e_psi]から補正舵角-Kxを求める。
        const Eigen::Vector2d error_state(lateral_error, yaw_error);
        correction = -cached_gain_.head<2>().dot(error_state);
    } else {
        // 積分付きLQRでは横偏差積分を第3状態としてモデルへ追加する。
        // LQR-EQ-09〜LQR-EQ-11：横偏差積分を加えた拡大状態・入力行列。
        Eigen::Matrix3d augmented_state = Eigen::Matrix3d::Zero();
        augmented_state.topLeftCorner<2, 2>() = model.state_matrix;
        augmented_state(2, 0) = config_.time_step;
        augmented_state(2, 2) = 1.0;
        Eigen::Vector3d augmented_input = Eigen::Vector3d::Zero();
        augmented_input.head<2>() = model.input_matrix;
        if (update_gain) {
            Eigen::Matrix3d state_weight = Eigen::Matrix3d::Zero();
            state_weight.diagonal() <<
                config_.lateral_error_weight,
                config_.yaw_error_weight,
                config_.integral_error_weight;
            const DiscreteLqrResult result = solveDiscreteLqr(
                augmented_state,
                augmented_input,
                state_weight,
                config_.steering_weight);
            cached_gain_ = result.gain;
        }

        // 積分候補を一度計算し、飽和を悪化させない場合だけ採用する。
        // これにより、舵角飽和中に誤差が蓄積するワインドアップを抑える。
        // LQR-EQ-08, LQR-EQ-12：横偏差を積分し、上下限内へ制限する。
        const double candidate_integral = std::clamp(
            integral_error_ + lateral_error * config_.time_step,
            -config_.integral_limit,
            config_.integral_limit);
        const Eigen::Vector3d old_state(
            lateral_error, yaw_error, integral_error_);
        const Eigen::Vector3d candidate_state(
            lateral_error, yaw_error, candidate_integral);
        const double old_correction = -cached_gain_.dot(old_state);
        const double candidate_correction =
            -cached_gain_.dot(candidate_state);
        const double old_command = feedforward_steering + old_correction;
        const double candidate_command =
            feedforward_steering + candidate_correction;
        const bool worsens_saturation =
            config_.anti_windup_steering_limit > 0.0 &&
            std::abs(candidate_command) >
                config_.anti_windup_steering_limit &&
            std::abs(candidate_command) > std::abs(old_command);
        if (!worsens_saturation) {
            integral_error_ = candidate_integral;
        }
        correction = -cached_gain_.dot(Eigen::Vector3d(
            lateral_error, yaw_error, integral_error_));
    }

    if (update_gain) {
        cached_velocity_ = linearization_velocity;
        cached_feedforward_steering_ = feedforward_steering;
        has_cached_gain_ = true;
    }

    previous_time_ = input.time;
    has_previous_time_ = true;
    // LQR-EQ-07：フィードフォワード舵角とLQR補正舵角を合成する。
    return {
        reference.velocity,
        feedforward_steering + correction};
}

double LqrController::integralError() const noexcept {
    return integral_error_;
}

void LqrController::reset() const {
    integral_error_ = 0.0;
    previous_time_ = 0.0;
    has_previous_time_ = false;
    cached_gain_.setZero();
    cached_velocity_ = 0.0;
    cached_feedforward_steering_ = 0.0;
    has_cached_gain_ = false;
}

double LqrController::selectLinearizationVelocity(
    const ControllerInput& input) const {
    // 停止点では入力行列BがゼロになりLQRゲインを求めにくいため、近傍の
    // 非ゼロ速度を探索する。この速度は線形化専用で、速度指令は変更しない。
    double velocity = input.time_reference.velocity;
    if (std::abs(velocity) < 1.0e-9) {
        for (std::size_t index = input.nearest_index;
             index < input.trajectory.size(); ++index) {
            if (std::abs(input.trajectory[index].velocity) >= 1.0e-9) {
                velocity = input.trajectory[index].velocity;
                break;
            }
        }
    }
    if (std::abs(velocity) < 1.0e-9) {
        for (std::size_t index = input.nearest_index + 1; index > 0; --index) {
            const double candidate = input.trajectory[index - 1].velocity;
            if (std::abs(candidate) >= 1.0e-9) {
                velocity = candidate;
                break;
            }
        }
    }
    // 前進・後退の符号を維持したまま、数値計算用の最小速度を保証する。
    const double direction = velocity < 0.0 ? -1.0 : 1.0;
    return direction * std::max(
        std::abs(velocity), config_.minimum_linearization_speed);
}
