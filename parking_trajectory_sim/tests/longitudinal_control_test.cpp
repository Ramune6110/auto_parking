// Verifies: LON-EQ-01〜LON-EQ-05, LONLIM-EQ-01〜LONLIM-EQ-04,
// STOP-EQ-01〜STOP-EQ-06, COOP-EQ-01〜COOP-EQ-05
#include "controllers/longitudinal/velocity_stop_controller.hpp"
#include "coordination/longitudinal_lateral_coordinator.hpp"
#include "models/kinematic_bicycle.hpp"

#include <cmath>
#include <iostream>
#include <stdexcept>

namespace {

void requireNear(double actual, double expected, double tolerance = 1.0e-10) {
    if (std::abs(actual - expected) > tolerance) {
        throw std::runtime_error("unexpected longitudinal control value");
    }
}

// Verifies: LON-EQ-01, LON-EQ-02
void testVelocityFeedback() {
    VelocityStopController controller({});
    const auto output = controller.computeCommand({1.0, 0.0, 10.0, 0.1});
    requireNear(output.velocity_error, 1.0);
    requireNear(output.requested_acceleration, 2.0);
}

// Verifies: LONLIM-EQ-01〜LONLIM-EQ-04
void testAccelerationAndJerkLimits() {
    VelocityStopControllerConfig config;
    config.enable_acceleration_limit = true;
    config.enable_jerk_limit = true;
    config.maximum_acceleration = 0.8;
    config.maximum_jerk = 1.5;
    VelocityStopController controller(config);
    const auto output = controller.computeCommand({1.0, 0.0, 10.0, 0.1});
    requireNear(output.applied_acceleration, 0.15);
    requireNear(output.jerk, 1.5);
    if (!output.acceleration_limit_active || !output.jerk_limit_active) {
        throw std::runtime_error("longitudinal constraints were not reported");
    }
}

// Verifies: STOP-EQ-01〜STOP-EQ-06
void testStopPositionControl() {
    VelocityStopControllerConfig config;
    config.enable_stop_position_control = true;
    VelocityStopController controller(config);
    const auto approach = controller.computeCommand({1.0, 0.2, 0.1, 0.1});
    if (!(approach.target_velocity <= config.position_gain * 0.1 + 1.0e-12) ||
        !approach.stop_position_control_active) {
        throw std::runtime_error("stop position speed limit was not applied");
    }
    const auto stopped = controller.computeCommand({0.0, 0.01, 0.01, 0.1});
    if (!stopped.stop_condition_satisfied) {
        throw std::runtime_error("position and velocity stop condition failed");
    }
}

// Verifies: COOP-EQ-01〜COOP-EQ-05
void testLongitudinalLateralCoordination() {
    LongitudinalLateralCoordinator coordinator({});
    const auto nominal = coordinator.coordinate({1.0, 0.0, 0.0, 0.0, false});
    const auto erroneous = coordinator.coordinate({1.0, 0.25, 0.5, 0.4, true});
    requireNear(nominal.target_velocity, 1.0);
    if (!(erroneous.target_velocity < nominal.target_velocity) ||
        !erroneous.coordination_limit_active) {
        throw std::runtime_error("coordination did not reduce target speed");
    }
}

// Verifies: LON-EQ-04, LON-EQ-05
void testAccelerationUpdatesVehicleSpeedAndPosition() {
    KinematicBicycle vehicle(2.7, {});
    vehicle.stepWithAcceleration(1.0, 0.0, 0.1);
    requireNear(vehicle.state().velocity, 0.1);
    requireNear(vehicle.state().x, 0.005);
}

}  // namespace

int main() {
    try {
        testVelocityFeedback();
        testAccelerationAndJerkLimits();
        testStopPositionControl();
        testLongitudinalLateralCoordination();
        testAccelerationUpdatesVehicleSpeedAndPosition();
        std::cout << "All longitudinal control tests passed.\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "Test failed: " << error.what() << '\n';
        return 1;
    }
}
