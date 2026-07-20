// Verifies: LINERR-EQ-13〜LINERR-EQ-19, LQR-EQ-04〜LQR-EQ-12
#include "controllers/lqr_controller.hpp"
#include "models/linear_error_model.hpp"
#include "simulation/parking_simulator.hpp"

#include <cmath>
#include <iostream>
#include <stdexcept>

namespace {

ControlCommand compute(
    const LqrController& controller,
    const VehicleState& state,
    double velocity) {
    TrajectoryPoint reference;
    reference.velocity = velocity;
    const Trajectory trajectory{reference};
    return controller.computeCommand(
        ControllerInput{0.0, state, trajectory.front(), trajectory, 0});
}

// Verifies: LINERR-EQ-18, LINERR-EQ-19
void testLinearModelSigns() {
    const auto forward = createLinearErrorModel(1.0, 0.0, 2.0, 0.1);
    if (forward.state_matrix(0, 1) >= 0.0 ||
        forward.input_matrix(1) >= 0.0) {
        throw std::runtime_error("forward linear model signs are incorrect");
    }
    const auto reverse = createLinearErrorModel(-1.0, 0.0, 2.0, 0.1);
    if (reverse.state_matrix(0, 1) <= 0.0 ||
        reverse.input_matrix(1) <= 0.0) {
        throw std::runtime_error("reverse linear model signs are incorrect");
    }
}

// Verifies: LINERR-EQ-09, LQR-EQ-07
void testZeroErrorUsesFeedforward() {
    const LqrController controller(LqrControllerConfig{});
    const auto command = compute(controller, VehicleState{}, 1.0);
    if (std::abs(command.target_steering) > 1.0e-10) {
        throw std::runtime_error("zero LQR error must use feedforward");
    }
}

// Verifies: LQR-EQ-01, LQR-EQ-05, LQR-EQ-06
void testLeftOffsetSteersRightInBothDirections() {
    const VehicleState state{0.0, 0.5, 0.0};
    const LqrController forward_controller(LqrControllerConfig{});
    const LqrController reverse_controller(LqrControllerConfig{});
    if (compute(forward_controller, state, 1.0).target_steering >= 0.0 ||
        compute(reverse_controller, state, -1.0).target_steering >= 0.0) {
        throw std::runtime_error(
            "LQR must steer right from a left offset in both directions");
    }
}

void testClosedLoopReduction(bool integral) {
    LqrControllerConfig controller_config;
    controller_config.enable_integral_action = integral;
    controller_config.anti_windup_steering_limit = 0.61;
    const LqrController controller(controller_config);
    const Trajectory trajectory{
        {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0},
        {8.0, 8.0, 8.0, 0.0, 0.0, 0.0, 1.0, 0.0}};
    SteeringActuatorConfig actuator;
    actuator.max_steering_angle = 0.61;
    actuator.max_steering_rate = 1.05;
    actuator.time_constant = 0.3;
    actuator.enable_angle_limit = true;
    actuator.enable_rate_limit = true;
    actuator.enable_first_order_lag = true;
    const ParkingSimulationConfig simulation_config{
        2.7, 0.01, VehicleState{0.0, 0.5, 0.0}, actuator};
    const auto result = runParkingSimulation(
        trajectory, controller, simulation_config);
    if (std::abs(result.samples.back().lateral_error) >= 0.1) {
        throw std::runtime_error("LQR closed loop did not reduce offset");
    }
}

// Verifies: LQR-EQ-08〜LQR-EQ-12
void testIntegralStateIsBounded() {
    LqrControllerConfig config;
    config.enable_integral_action = true;
    config.integral_limit = 0.05;
    const LqrController controller(config);
    const VehicleState state{0.0, 1.0, 0.0};
    TrajectoryPoint reference;
    reference.velocity = 1.0;
    const Trajectory trajectory{reference};
    for (int index = 0; index < 100; ++index) {
        controller.computeCommand(ControllerInput{
            0.01 * index, state, trajectory.front(), trajectory, 0});
    }
    if (std::abs(controller.integralError()) > 0.05 + 1.0e-12) {
        throw std::runtime_error("LQR integral limit was exceeded");
    }
}

void testIntegralReducesConstantSteeringBias() {
    LqrControllerConfig normal_config;
    LqrControllerConfig integral_config;
    integral_config.enable_integral_action = true;
    const LqrController normal_controller(normal_config);
    const LqrController integral_controller(integral_config);
    const Trajectory trajectory{
        {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0},
        {20.0, 20.0, 20.0, 0.0, 0.0, 0.0, 1.0, 0.0}};
    SteeringActuatorConfig actuator;
    actuator.steering_bias = 0.02;
    const ParkingSimulationConfig simulation_config{
        2.7, 0.01, VehicleState{}, actuator};
    const auto normal_result = runParkingSimulation(
        trajectory, normal_controller, simulation_config);
    const auto integral_result = runParkingSimulation(
        trajectory, integral_controller, simulation_config);
    const double normal_error =
        std::abs(normal_result.samples.back().lateral_error);
    const double integral_error =
        std::abs(integral_result.samples.back().lateral_error);
    if (integral_error >= 0.7 * normal_error) {
        throw std::runtime_error(
            "integral LQR did not reduce constant-bias steady error: normal=" +
            std::to_string(normal_error) + ", integral=" +
            std::to_string(integral_error));
    }
}

}  // namespace

int main() {
    try {
        testLinearModelSigns();
        testZeroErrorUsesFeedforward();
        testLeftOffsetSteersRightInBothDirections();
        testClosedLoopReduction(false);
        testClosedLoopReduction(true);
        testIntegralStateIsBounded();
        testIntegralReducesConstantSteeringBias();
        return 0;
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
