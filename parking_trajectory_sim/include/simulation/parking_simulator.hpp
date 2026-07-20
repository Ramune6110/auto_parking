// Traceability: IMPL-SIM-01
#pragma once

#include "controllers/trajectory_controller.hpp"
#include "actuators/steering_actuator.hpp"
#include "core/vehicle_parameters.hpp"
#include "simulation/simulation_result.hpp"
#include "scenario/parking_scenario.hpp"
#include "controllers/longitudinal/velocity_stop_controller.hpp"
#include "coordination/longitudinal_lateral_coordinator.hpp"

#include <optional>

struct ParkingSimulationConfig {
    ParkingSimulationConfig() = default;
    ParkingSimulationConfig(
        double wheelbase_value,
        double time_step_value,
        std::optional<VehicleState> initial_state_value,
        const SteeringActuatorConfig& steering_actuator_value)
        : wheelbase(wheelbase_value),
          time_step(time_step_value),
          initial_state(initial_state_value),
          steering_actuator(steering_actuator_value) {}

    double wheelbase{vehicle_parameters::kWheelbase}; // ホイールベース [m]
    double time_step{0.01};  // 制御・シミュレーション周期 [s]
    // 未指定なら目標軌道の始点姿勢から開始する。
    std::optional<VehicleState> initial_state;
    SteeringActuatorConfig steering_actuator;
    enum class LongitudinalMode {
        Ideal,
        VelocityFeedback,
        AccelerationLimit,
        JerkLimit,
        StopPosition,
        Coordinated
    } longitudinal_mode{LongitudinalMode::Ideal};
    VelocityStopControllerConfig longitudinal_controller;
    LongitudinalLateralCoordinatorConfig coordinator;
};

/**
 * @brief 目標軌道の参照、制御指令計算、車両更新を周期的に実行する。
 *
 * ファイル入出力は行わず、計算結果を SimulationResult として返す。
 */
SimulationResult runParkingSimulation(
    const Trajectory& trajectory,
    const TrajectoryController& controller,
    const ParkingSimulationConfig& config);

/** @brief 複数走行区間と停止状態を含む駐車シナリオを連続実行する。 */
SimulationResult runParkingScenario(
    const ParkingScenario& scenario,
    const TrajectoryController& controller,
    const ParkingSimulationConfig& config);
