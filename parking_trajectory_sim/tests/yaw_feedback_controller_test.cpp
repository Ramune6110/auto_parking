// Verifies: YAW-EQ-01〜YAW-EQ-08
#include "controllers/yaw_feedback_controller.hpp"

#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>

namespace {

constexpr double kTolerance = 1.0e-10;

ControlCommand compute(
    const YawFeedbackController& controller,
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

void requirePositive(double value, const std::string& message) {
    if (value <= 0.0) {
        throw std::runtime_error(message);
    }
}

void requireNegative(double value, const std::string& message) {
    if (value >= 0.0) {
        throw std::runtime_error(message);
    }
}

void testZeroErrorUsesFeedforwardSteering() {
    const YawFeedbackController controller(2.7, 1.0);
    const VehicleState state{0.0, 0.0, 0.2};
    const TrajectoryPoint reference{
        0.0, 0.0, 0.0, 0.0, 0.2, 0.1, 1.0, 0.0};
    const ControlCommand command = compute(controller, state, reference);
    requireNear(command.target_steering, std::atan(2.7 * 0.1),
        "zero yaw error must use feedforward steering");
}

void testForwardAndReverseCorrectionDirections() {
    const YawFeedbackController controller(2.7, 1.0);
    const VehicleState state{0.0, 0.0, -0.1};
    TrajectoryPoint reference;
    reference.yaw = 0.0;
    reference.velocity = 1.0;

    const ControlCommand forward = compute(controller, state, reference);
    requirePositive(forward.target_steering,
        "forward positive yaw error must produce positive correction");

    reference.velocity = -1.0;
    const ControlCommand reverse = compute(controller, state, reference);
    requireNegative(reverse.target_steering,
        "reverse positive yaw error must produce negative correction");
}

void testYawErrorWrapsAcrossPiBoundary() {
    const YawFeedbackController controller(2.7, 1.0);
    const VehicleState state{0.0, 0.0, 179.0 * 3.14159265358979323846 / 180.0};
    TrajectoryPoint reference;
    reference.yaw = -179.0 * 3.14159265358979323846 / 180.0;
    reference.velocity = 1.0;

    const ControlCommand command = compute(controller, state, reference);
    requireNear(
        command.target_steering,
        2.0 * 3.14159265358979323846 / 180.0,
        "yaw error must take the shortest direction across pi");
}

}  // namespace

int main() {
    try {
        testZeroErrorUsesFeedforwardSteering();
        testForwardAndReverseCorrectionDirections();
        testYawErrorWrapsAcrossPiBoundary();
        std::cout << "All yaw feedback controller tests passed.\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "Test failed: " << error.what() << '\n';
        return 1;
    }
}
