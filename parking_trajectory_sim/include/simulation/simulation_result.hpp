// Traceability: IMPL-DATA-01, IMPL-CSV-01
#pragma once

#include "core/control_types.hpp"
#include "core/longitudinal_types.hpp"
#include "core/types.hpp"
#include "scenario/parking_scenario.hpp"

#include <vector>

/** @brief 1制御周期の状態、参照値、指令値、追従誤差。 */
struct SimulationSample {
    double time{};
    VehicleState vehicle_state;
    TrajectoryPoint reference;
    ControlCommand command;
    double applied_steering{};     // 車両モデルへ適用した舵角 [rad]
    double steering_rate{};        // 実舵角の変化速度 [rad/s]
    bool angle_limit_active{};
    bool rate_limit_active{};
    double position_error{};
    double lateral_error{};       // 参照経路の左側を正 [m]
    double longitudinal_error{};  // 参照経路の前方を正 [m]
    double yaw_error{};           // 参照方位 - 車両方位 [rad]
    ParkingMode parking_mode{ParkingMode::Forward};
    std::size_t segment_index{};
    double mode_elapsed_time{};
    bool state_transition{};
    bool controller_reset{};
    LongitudinalCommand longitudinal;
    CoordinationOutput coordination;
    double controller_computation_time_ms{};
};

/** @brief 1回の軌道追従シミュレーションで得られた全データ。 */
struct SimulationResult {
    Trajectory reference_trajectory;
    std::vector<SimulationSample> samples;
    VehicleState final_state;
    ParkingEnvironment parking_environment;
};
