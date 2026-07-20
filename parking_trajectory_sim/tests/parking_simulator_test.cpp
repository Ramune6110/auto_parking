// Verifies: IMPL-SIM-01, COORD-EQ-01, COORD-EQ-04, COORD-EQ-05, COORD-EQ-07
#include "controllers/trajectory_controller.hpp"
#include "controllers/feedforward_controller.hpp"
#include "simulation/parking_simulator.hpp"

#include <cmath>
#include <iostream>
#include <stdexcept>

namespace {

constexpr double kTolerance = 1.0e-10;

class StopController final : public TrajectoryController {
public:
    ControlCommand computeCommand(
        const ControllerInput&) const override {
        return {0.0, 0.0};
    }
};

class ScenarioController final : public TrajectoryController {
public:
    ControlCommand computeCommand(const ControllerInput& input) const override {
        return {input.time_reference.velocity, 0.0};
    }
    void reset() const override { ++reset_count_; }
    int resetCount() const noexcept { return reset_count_; }
private:
    mutable int reset_count_{};
};

void requireNear(double actual, double expected, const char* message) {
    if (std::abs(actual - expected) > kTolerance) {
        throw std::runtime_error(message);
    }
}

void testInitialStateAndErrorConvention() {
    const Trajectory trajectory{
        {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
        {1.0, 1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
    const VehicleState initial_state{0.0, 1.0, -0.1};
    const ParkingSimulationConfig config{2.7, 0.1, initial_state, {}};
    const StopController controller;

    const SimulationResult result =
        runParkingSimulation(trajectory, controller, config);
    const SimulationSample& first = result.samples.front();

    requireNear(first.vehicle_state.x, 0.0, "initial x was not applied");
    requireNear(first.vehicle_state.y, 1.0, "initial y was not applied");
    requireNear(first.lateral_error, 1.0, "left lateral error must be positive");
    requireNear(first.longitudinal_error, 0.0, "longitudinal error convention");
    requireNear(first.yaw_error, 0.1, "yaw error must be reference minus actual");
}

void testFeedforwardDoesNotCorrectInitialLateralOffset() {
    const Trajectory trajectory{
        {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0},
        {1.0, 1.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0}};
    const ParkingSimulationConfig config{
        2.7, 0.01, VehicleState{0.0, 0.5, 0.0}, {}};
    const FeedforwardController controller(config.wheelbase);

    const SimulationResult result =
        runParkingSimulation(trajectory, controller, config);

    requireNear(
        result.samples.front().lateral_error,
        0.5,
        "initial lateral offset");
    requireNear(
        result.samples.back().lateral_error,
        0.5,
        "feedforward must not correct lateral offset");
    requireNear(
        result.samples.back().yaw_error,
        0.0,
        "straight feedforward yaw error");
}

// Verifies: IMPL-SCENARIO-FSM-01（区間限定、停止指令、制御器リセット）
void testForwardStopReverseScenarioIntegration() {
    const Trajectory forward{
        {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0},
        {0.2, 0.2, 0.2, 0.0, 0.0, 0.0, 1.0, 0.0}};
    const Trajectory reverse{
        {0.0, 0.0, 0.2, 0.0, 0.0, 0.0, -1.0, 0.0},
        {0.2, 0.2, 0.0, 0.0, 0.0, 0.0, -1.0, 0.0}};
    const ParkingScenario scenario{
        {{MotionDirection::Forward, forward},
         {MotionDirection::Reverse, reverse}},
        0.2,
        0.1,
        {}};
    ParkingSimulationConfig config{2.7, 0.1, std::nullopt, {}};
    config.longitudinal_mode =
        ParkingSimulationConfig::LongitudinalMode::StopPosition;
    const ScenarioController controller;
    const SimulationResult result = runParkingScenario(
        scenario, controller, config);

    bool saw_forward = false;
    bool saw_stop = false;
    bool saw_reverse = false;
    bool saw_final_stop = false;
    for (const SimulationSample& sample : result.samples) {
        saw_forward = saw_forward || sample.parking_mode == ParkingMode::Forward;
        saw_reverse = saw_reverse || sample.parking_mode == ParkingMode::Reverse;
        if (sample.parking_mode == ParkingMode::GearChangeStop) {
            saw_stop = true;
            requireNear(sample.command.target_velocity, 0.0, "gear stop velocity");
        }
        if (sample.state_transition &&
            (sample.parking_mode == ParkingMode::Forward ||
             sample.parking_mode == ParkingMode::Reverse) &&
            std::abs(sample.vehicle_state.velocity) >
                config.longitudinal_controller.velocity_tolerance + 1.0e-12) {
            throw std::runtime_error("segment changed before actual vehicle stopped");
        }
        if (sample.parking_mode == ParkingMode::FinalStop) {
            saw_final_stop = true;
            requireNear(sample.command.target_velocity, 0.0, "final stop velocity");
        }
    }
    if (!saw_forward || !saw_stop || !saw_reverse || !saw_final_stop ||
        controller.resetCount() != 2) {
        throw std::runtime_error("scenario modes or controller resets are invalid");
    }
}

}  // namespace

int main() {
    try {
        testInitialStateAndErrorConvention();
        testFeedforwardDoesNotCorrectInitialLateralOffset();
        testForwardStopReverseScenarioIntegration();
        std::cout << "All parking simulator tests passed.\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "Test failed: " << error.what() << '\n';
        return 1;
    }
}
