// Verifies: BICYCLE-EQ-06, BICYCLE-EQ-09〜BICYCLE-EQ-13
#include "models/kinematic_bicycle.hpp"

#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>

namespace {

constexpr double kTolerance = 1.0e-10;

void requireNear(
    double actual,
    double expected,
    const std::string& message) {
    if (std::abs(actual - expected) > kTolerance) {
        throw std::runtime_error(
            message + ": expected " + std::to_string(expected) +
            ", actual " + std::to_string(actual));
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

// Verifies: BICYCLE-EQ-13（前進直進）
void testForwardStraight() {
    KinematicBicycle vehicle(2.7, {});
    vehicle.step(1.0, 0.0, 2.0);
    requireNear(vehicle.state().x, 2.0, "forward straight x");
    requireNear(vehicle.state().y, 0.0, "forward straight y");
    requireNear(vehicle.state().yaw, 0.0, "forward straight yaw");
}

// Verifies: BICYCLE-EQ-13（後退直進）
void testReverseStraight() {
    KinematicBicycle vehicle(2.7, {});
    vehicle.step(-1.0, 0.0, 2.0);
    requireNear(vehicle.state().x, -2.0, "reverse straight x");
    requireNear(vehicle.state().y, 0.0, "reverse straight y");
    requireNear(vehicle.state().yaw, 0.0, "reverse straight yaw");
}

// Verifies: BICYCLE-EQ-06, BICYCLE-EQ-09, BICYCLE-EQ-11, BICYCLE-EQ-12
void testSteeringDirection() {
    constexpr double steering = 0.3;

    KinematicBicycle forward_left(2.7, {});
    forward_left.step(1.0, steering, 1.0);
    requirePositive(
        forward_left.state().yaw,
        "forward with positive steering must increase yaw");

    KinematicBicycle forward_right(2.7, {});
    forward_right.step(1.0, -steering, 1.0);
    requireNegative(
        forward_right.state().yaw,
        "forward with negative steering must decrease yaw");

    KinematicBicycle reverse_left(2.7, {});
    reverse_left.step(-1.0, steering, 1.0);
    requireNegative(
        reverse_left.state().yaw,
        "reverse with positive steering must decrease yaw");

    KinematicBicycle reverse_right(2.7, {});
    reverse_right.step(-1.0, -steering, 1.0);
    requirePositive(
        reverse_right.state().yaw,
        "reverse with negative steering must increase yaw");
}

}  // namespace

int main() {
    try {
        testForwardStraight();
        testReverseStraight();
        testSteeringDirection();
        std::cout << "All kinematic bicycle tests passed.\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "Test failed: " << error.what() << '\n';
        return 1;
    }
}
