// Verifies: LAT-EQ-03, LAT-EQ-04, LAT-EQ-10
#include "controllers/lateral_feedback_controller.hpp"
#include "simulation/parking_simulator.hpp"

#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>

namespace {

constexpr double kTolerance = 1.0e-10;

ControlCommand compute(
    const LateralFeedbackController& controller,
    const VehicleState& state,
    const TrajectoryPoint& reference) {
    const Trajectory trajectory{reference};
    return controller.computeCommand(
        ControllerInput{0.0, state, trajectory.front(), trajectory, 0});
}

void requireNear(double actual, double expected, const std::string& message) {
    if (std::abs(actual - expected) > kTolerance) {
        throw std::runtime_error(message);
    }
}

void requireNegative(double value, const std::string& message) {
    if (value >= 0.0) {
        throw std::runtime_error(message);
    }
}

void testZeroErrorsUseFeedforwardSteering() {
    const LateralFeedbackController controller(2.7, 1.0, 0.2);
    const VehicleState state{1.0, 2.0, 0.3};
    const TrajectoryPoint reference{
        0.0, 0.0, 1.0, 2.0, 0.3, 0.1, 1.0, 0.0};
    const ControlCommand command = compute(controller, state, reference);
    requireNear(command.target_steering, std::atan(2.7 * 0.1),
        "zero errors must use feedforward steering");
}

void testPositiveLateralErrorSteersRightInBothDirections() {
    const LateralFeedbackController controller(2.7, 1.0, 0.2);
    const VehicleState state{0.0, 0.5, 0.0};
    TrajectoryPoint reference;
    reference.velocity = 1.0;

    const ControlCommand forward = compute(controller, state, reference);
    requireNegative(forward.target_steering,
        "forward left offset must produce right steering");

    reference.velocity = -1.0;
    const ControlCommand reverse = compute(controller, state, reference);
    requireNegative(reverse.target_steering,
        "reverse left offset must produce right steering");
}

void testYawCorrectionStillUsesMotionDirection() {
    const LateralFeedbackController controller(2.7, 1.0, 0.2);
    const VehicleState state{0.0, 0.0, -0.1};
    TrajectoryPoint reference;
    reference.velocity = 1.0;
    const ControlCommand forward = compute(controller, state, reference);

    reference.velocity = -1.0;
    const ControlCommand reverse = compute(controller, state, reference);

    requireNear(forward.target_steering, 0.1,
        "forward yaw feedback sign");
    requireNear(reverse.target_steering, -0.1,
        "reverse yaw feedback sign");
}

void testClosedLoopReducesLateralOffset() {
    const LateralFeedbackController controller(2.7, 1.0, 0.2);
    const Trajectory trajectory{
        {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0},
        {10.0, 10.0, 10.0, 0.0, 0.0, 0.0, 1.0, 0.0}};
    const ParkingSimulationConfig config{
        2.7, 0.01, VehicleState{0.0, 0.5, 0.0}, {}};

    const SimulationResult result =
        runParkingSimulation(trajectory, controller, config);
    const double initial_error =
        std::abs(result.samples.front().lateral_error);
    const double final_error =
        std::abs(result.samples.back().lateral_error);
    if (final_error >= 0.2 * initial_error) {
        throw std::runtime_error(
            "closed-loop lateral feedback did not reduce the offset");
    }
}

}  // namespace

int main() {
    try {
        testZeroErrorsUseFeedforwardSteering();
        testPositiveLateralErrorSteersRightInBothDirections();
        testYawCorrectionStillUsesMotionDirection();
        testClosedLoopReducesLateralOffset();
        std::cout << "All lateral feedback controller tests passed.\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "Test failed: " << error.what() << '\n';
        return 1;
    }
}
