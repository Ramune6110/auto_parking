// Traceability: IMPL-SCENARIO-SAMPLE-01
#pragma once

#include "scenario/parking_scenario.hpp"
#include "trajectory/sample_trajectories.hpp"

ParkingScenario createForwardStopReverseScenario(
    const SampleTrajectoryConfig& config,
    double gear_change_wait_time = 1.0,
    double final_stop_time = 0.5);

ParkingScenario createSimpleForwardStopReverseScenario(
    const SampleTrajectoryConfig& config,
    double gear_change_wait_time = 1.0,
    double final_stop_time = 0.5);
