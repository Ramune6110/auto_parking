// Traceability: MPC-EQ-01〜MPC-EQ-15
#pragma once

#include "controllers/trajectory_controller.hpp"
#include "core/vehicle_parameters.hpp"

#include <Eigen/Core>

struct LinearMpcControllerConfig {
    double wheelbase{vehicle_parameters::kWheelbase};
    double time_step{0.01};
    int prediction_horizon{30};          // 0.01 s周期なら0.3 s先まで予測
    double lateral_error_weight{20.0};
    double yaw_error_weight{4.0};
    double terminal_weight_scale{8.0};
    double steering_correction_weight{0.5};
    double maximum_steering_angle{0.610865238198};  // 35 deg [rad]
    double maximum_steering_rate{1.0471975512};     // 60 deg/s [rad/s]
    int maximum_solver_iterations{15};
    double solver_tolerance{1.0e-6};
    double minimum_linearization_speed{0.2};
    bool enable_steering_angle_constraint{true};
    bool enable_steering_rate_constraint{true};
};

/**
 * @brief 横偏差・方位誤差を有限区間予測し、補正舵角列を最適化する線形MPC。
 *
 * 学習用として、予測行列の構築と射影勾配法をクラス内に明示している。
 * 実車向けでは専用QPソルバーへの置き換えを検討する。
 */
class LinearMpcController final : public TrajectoryController {
public:
    explicit LinearMpcController(const LinearMpcControllerConfig& config);

    ControlCommand computeCommand(const ControllerInput& input) const override;
    void reset() const override;

    double lastCost() const noexcept;
    int lastIterations() const noexcept;
    double previousSteeringCommand() const noexcept;

private:
    double selectLinearizationVelocity(const ControllerInput& input) const;
    Eigen::VectorXd solveConstrainedSequence(
        const Eigen::MatrixXd& hessian,
        const Eigen::VectorXd& gradient,
        double feedforward_steering) const;
    void projectSteeringConstraints(
        Eigen::VectorXd& correction_sequence,
        double feedforward_steering) const;

    LinearMpcControllerConfig config_;
    mutable Eigen::VectorXd warm_start_;
    mutable double previous_steering_command_{};
    mutable double last_cost_{};
    mutable int last_iterations_{};
};
