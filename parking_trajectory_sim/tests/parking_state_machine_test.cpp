// Verifies: IMPL-SCENARIO-FSM-01（前進→停止→後退→最終停止）
#include "scenario/parking_scenario.hpp"
#include "scenario/sample_parking_scenario.hpp"
#include "core/vehicle_parameters.hpp"

#include <cmath>
#include <iostream>
#include <stdexcept>

namespace {

Trajectory onePointTrajectory(double velocity) {
    return {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, velocity, 0.0}};
}

void requireMode(const ParkingStateMachine& machine, ParkingMode expected) {
    if (machine.mode() != expected) {
        throw std::runtime_error("unexpected parking mode");
    }
}

void testForwardStopReverseFinalStop() {
    const ParkingScenario scenario{
        {{MotionDirection::Forward, onePointTrajectory(1.0)},
         {MotionDirection::Reverse, onePointTrajectory(-1.0)}},
        0.2,
        0.1,
        {}};
    ParkingStateMachine machine(scenario);
    requireMode(machine, ParkingMode::Forward);

    machine.update(0.1, true);
    requireMode(machine, ParkingMode::GearChangeStop);
    machine.update(0.1, false);
    requireMode(machine, ParkingMode::GearChangeStop);
    machine.update(0.1, false);
    requireMode(machine, ParkingMode::Reverse);
    if (machine.segmentIndex() != 1) {
        throw std::runtime_error("reverse segment was not selected");
    }

    machine.update(0.1, true);
    requireMode(machine, ParkingMode::FinalStop);
    machine.update(0.1, false);
    requireMode(machine, ParkingMode::Completed);
}

void testDoesNotChangeBeforeSegmentCompletion() {
    const ParkingScenario scenario{
        {{MotionDirection::Forward, onePointTrajectory(1.0)}}, 0.2, 0.1, {}};
    ParkingStateMachine machine(scenario);
    machine.update(1.0, false);
    requireMode(machine, ParkingMode::Forward);
}

void requireNear(double actual, double expected, double tolerance) {
    if (std::abs(actual - expected) > tolerance) {
        throw std::runtime_error("unexpected parking scenario geometry");
    }
}

// Verifies: IMPL-SCENARIO-SAMPLE-01（連続性、速度符号、駐車枠内の目標姿勢）
void testPracticalParkingScenarioGeometry() {
    const ParkingScenario scenario = createForwardStopReverseScenario({});
    if (scenario.segments.size() != 2 || !scenario.environment.enabled) {
        throw std::runtime_error("practical parking scenario is incomplete");
    }
    const Trajectory& forward = scenario.segments[0].trajectory;
    const Trajectory& reverse = scenario.segments[1].trajectory;
    requireNear(forward.back().x, reverse.front().x, 1.0e-10);
    requireNear(forward.back().y, reverse.front().y, 1.0e-10);
    requireNear(forward.back().yaw, reverse.front().yaw, 1.0e-10);
    if (forward[1].velocity <= 0.0 || reverse[1].velocity >= 0.0) {
        throw std::runtime_error("parking segment velocity sign is invalid");
    }

    const VehicleState& target = scenario.environment.target_pose;
    const AxisAlignedRectangle& space = scenario.environment.parking_space;
    const double half_width = 0.5 * vehicle_parameters::kVehicleWidth;
    // 目標方位は90度なので、車幅方向が世界X、車長方向が世界Yに一致する。
    if (target.x - half_width < space.minimum_x ||
        target.x + half_width > space.maximum_x ||
        target.y - vehicle_parameters::kRearOverhang < space.minimum_y ||
        target.y + vehicle_parameters::kWheelbase +
                vehicle_parameters::kFrontOverhang > space.maximum_y) {
        throw std::runtime_error("target vehicle body is outside parking space");
    }
}

}  // namespace

int main() {
    try {
        testForwardStopReverseFinalStop();
        testDoesNotChangeBeforeSegmentCompletion();
        testPracticalParkingScenarioGeometry();
        std::cout << "All parking state machine tests passed.\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "Test failed: " << error.what() << '\n';
        return 1;
    }
}
