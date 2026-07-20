// Verifies: PP-EQ-01〜PP-EQ-12, STANLEY-EQ-01〜STANLEY-EQ-10
#include "controllers/pure_pursuit_controller.hpp"
#include "controllers/stanley_controller.hpp"
#include "simulation/parking_simulator.hpp"

#include <cmath>
#include <iostream>
#include <stdexcept>

namespace {

void requireNegative(double value, const char* message) {
    if (value >= 0.0) {
        throw std::runtime_error(message);
    }
}

Trajectory makeTrajectory(double velocity) {
    return {
        {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, velocity, 0.0},
        {1.0, 1.0, velocity, 0.0, 0.0, 0.0, velocity, 0.0},
        {2.0, 2.0, 2.0 * velocity, 0.0, 0.0, 0.0, velocity, 0.0}};
}

template <typename Controller>
ControlCommand compute(
    const Controller& controller,
    const VehicleState& state,
    const Trajectory& trajectory) {
    return controller.computeCommand(ControllerInput{
        0.0, state, trajectory.front(), trajectory, 0});
}

void testPurePursuitCorrectsLeftOffsetInBothDirections() {
    const PurePursuitController controller(2.7, 1.0);
    const VehicleState state{0.0, 0.5, 0.0};
    const Trajectory forward = makeTrajectory(1.0);
    const Trajectory reverse = makeTrajectory(-1.0);
    requireNegative(compute(controller, state, forward).target_steering,
        "forward Pure Pursuit must steer right from a left offset");
    requireNegative(compute(controller, state, reverse).target_steering,
        "reverse Pure Pursuit must steer right from a left offset");
}

void testStanleyCorrectsLeftOffsetInBothDirections() {
    const StanleyController controller(2.7, 1.0, 0.5);
    const VehicleState state{0.0, 0.5, 0.0};
    const Trajectory forward = makeTrajectory(1.0);
    const Trajectory reverse = makeTrajectory(-1.0);
    requireNegative(compute(controller, state, forward).target_steering,
        "forward Stanley must steer right from a left offset");
    requireNegative(compute(controller, state, reverse).target_steering,
        "reverse Stanley must steer right from a left offset");
}

template <typename Controller>
void requireClosedLoopReduction(
    const Controller& controller,
    double velocity,
    const char* message) {
    const Trajectory trajectory{
        {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, velocity, 0.0},
        {10.0, 10.0, 10.0 * velocity, 0.0, 0.0, 0.0, velocity, 0.0}};
    const ParkingSimulationConfig config{
        2.7, 0.01, VehicleState{0.0, 0.5, 0.0}, {}};
    const SimulationResult result =
        runParkingSimulation(trajectory, controller, config);
    if (std::abs(result.samples.back().lateral_error) >= 0.1) {
        throw std::runtime_error(message);
    }
}

void testClosedLoopReductionInBothDirections() {
    const PurePursuitController pure_pursuit(2.7, 1.0);
    const StanleyController stanley(2.7, 0.5, 1.0);
    requireClosedLoopReduction(
        pure_pursuit, 1.0, "forward Pure Pursuit did not reduce offset");
    requireClosedLoopReduction(
        pure_pursuit, -1.0, "reverse Pure Pursuit did not reduce offset");
    requireClosedLoopReduction(
        stanley, 1.0, "forward Stanley did not reduce offset");
    requireClosedLoopReduction(
        stanley, -1.0, "reverse Stanley did not reduce offset");
}

}  // namespace

int main() {
    try {
        testPurePursuitCorrectsLeftOffsetInBothDirections();
        testStanleyCorrectsLeftOffsetInBothDirections();
        testClosedLoopReductionInBothDirections();
        std::cout << "All path tracking controller tests passed.\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "Test failed: " << error.what() << '\n';
        return 1;
    }
}
