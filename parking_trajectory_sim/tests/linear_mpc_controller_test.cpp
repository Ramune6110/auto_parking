// Verifies: MPC-EQ-01〜MPC-EQ-15
#include "controllers/linear_mpc_controller.hpp"

#include <cmath>
#include <iostream>
#include <stdexcept>

namespace {

Trajectory straightTrajectory(double velocity) {
    return {
        {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, velocity, 0.0},
        {1.0, 1.0, velocity > 0.0 ? 1.0 : -1.0,
         0.0, 0.0, 0.0, velocity, 0.0}};
}

ControlCommand commandFor(
    const LinearMpcController& controller,
    double velocity,
    double lateral_offset) {
    const Trajectory trajectory = straightTrajectory(velocity);
    const VehicleState state{0.0, lateral_offset, 0.0, velocity};
    return controller.computeCommand(
        {0.0, state, trajectory.front(), trajectory, 0});
}

// Verifies: MPC-EQ-01〜MPC-EQ-11（無制約有限ホライズン解）
void testUnconstrainedMpcCorrectsLateralError() {
    LinearMpcControllerConfig config;
    config.enable_steering_angle_constraint = false;
    config.enable_steering_rate_constraint = false;
    LinearMpcController controller(config);
    const ControlCommand command = commandFor(controller, 1.0, 0.5);
    if (!(command.target_steering < 0.0) || controller.lastIterations() != 1 ||
        !std::isfinite(controller.lastCost())) {
        throw std::runtime_error("unconstrained MPC solution is invalid");
    }
}

// Verifies: MPC-EQ-13, MPC-EQ-14（舵角・操舵速度制約）
void testSteeringConstraints() {
    LinearMpcControllerConfig config;
    config.maximum_steering_angle = 0.1;
    config.maximum_steering_rate = 0.2;
    config.time_step = 0.1;
    LinearMpcController controller(config);
    const ControlCommand first = commandFor(controller, 1.0, 2.0);
    if (std::abs(first.target_steering) > 0.0200000001) {
        throw std::runtime_error("MPC steering rate constraint was violated");
    }
    for (int index = 0; index < 10; ++index) {
        const double previous = controller.previousSteeringCommand();
        const ControlCommand next = commandFor(controller, 1.0, 2.0);
        if (std::abs(next.target_steering) > 0.1000000001 ||
            std::abs(next.target_steering - previous) > 0.0200000001) {
            throw std::runtime_error("MPC steering constraints were violated");
        }
    }
}

// Verifies: MPC-EQ-02, MPC-EQ-15（前進・後退と区間リセット）
void testForwardAndReverseAreFinite() {
    LinearMpcController controller({});
    const ControlCommand forward = commandFor(controller, 1.0, 0.3);
    controller.reset();
    const ControlCommand reverse = commandFor(controller, -1.0, 0.3);
    if (!std::isfinite(forward.target_steering) ||
        !std::isfinite(reverse.target_steering) ||
        controller.lastIterations() <= 0) {
        throw std::runtime_error("MPC forward/reverse result is invalid");
    }
}

}  // namespace

int main() {
    try {
        testUnconstrainedMpcCorrectsLateralError();
        testSteeringConstraints();
        testForwardAndReverseAreFinite();
        std::cout << "All linear MPC tests passed.\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "Test failed: " << error.what() << '\n';
        return 1;
    }
}
