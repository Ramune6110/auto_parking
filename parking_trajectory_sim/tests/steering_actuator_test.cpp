// Verifies: ACT-EQ-02〜ACT-EQ-08
#include "actuators/steering_actuator.hpp"

#include <cmath>
#include <iostream>
#include <stdexcept>

namespace {

void expectNear(double actual, double expected, double tolerance) {
    if (std::abs(actual - expected) > tolerance) {
        throw std::runtime_error("unexpected steering actuator value");
    }
}

void testIdealResponse() {
    SteeringActuator actuator(SteeringActuatorConfig{});
    const auto output = actuator.update(0.7, 0.1);
    expectNear(output.applied_steering, 0.7, 1.0e-12);
    expectNear(output.steering_rate, 7.0, 1.0e-12);
}

// Verifies: ACT-EQ-02
void testAngleLimit() {
    SteeringActuatorConfig config;
    config.max_steering_angle = 0.5;
    config.enable_angle_limit = true;
    SteeringActuator actuator(config);
    const auto output = actuator.update(0.8, 0.1);
    expectNear(output.applied_steering, 0.5, 1.0e-12);
    if (!output.angle_limit_active) {
        throw std::runtime_error("angle limit must be reported active");
    }
}

// Verifies: ACT-EQ-06, ACT-EQ-07, ACT-EQ-08
void testRateLimitInBothDirections() {
    SteeringActuatorConfig config;
    config.max_steering_rate = 1.0;
    config.enable_rate_limit = true;
    SteeringActuator actuator(config);
    expectNear(actuator.update(0.5, 0.1).applied_steering, 0.1, 1.0e-12);
    const auto output = actuator.update(-0.5, 0.1);
    expectNear(output.applied_steering, 0.0, 1.0e-12);
    if (!output.rate_limit_active) {
        throw std::runtime_error("rate limit must be reported active");
    }
}

// Verifies: ACT-EQ-05
void testFirstOrderLag() {
    SteeringActuatorConfig config;
    config.time_constant = 0.5;
    config.enable_first_order_lag = true;
    SteeringActuator actuator(config);
    const auto output = actuator.update(1.0, 0.5);
    expectNear(output.applied_steering, 1.0 - std::exp(-1.0), 1.0e-12);
}

void testSteeringBiasRespectsAngleLimit() {
    SteeringActuatorConfig config;
    config.max_steering_angle = 0.5;
    config.steering_bias = 0.1;
    config.enable_angle_limit = true;
    SteeringActuator actuator(config);
    expectNear(actuator.update(0.45, 0.1).applied_steering, 0.5, 1.0e-12);
}

void testInvalidConfiguration() {
    SteeringActuatorConfig config;
    config.enable_angle_limit = true;
    try {
        SteeringActuator actuator(config);
        static_cast<void>(actuator);
    } catch (const std::invalid_argument&) {
        return;
    }
    throw std::runtime_error("invalid configuration was accepted");
}

}  // namespace

int main() {
    try {
        testIdealResponse();
        testAngleLimit();
        testRateLimitInBothDirections();
        testFirstOrderLag();
        testSteeringBiasRespectsAngleLimit();
        testInvalidConfiguration();
        return 0;
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
