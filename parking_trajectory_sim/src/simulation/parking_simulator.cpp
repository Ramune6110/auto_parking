// Traceability: IMPL-SIM-01, COORD-EQ-01, COORD-EQ-04, COORD-EQ-05, COORD-EQ-07
#include "simulation/parking_simulator.hpp"

#include "core/tracking_error.hpp"
#include "models/kinematic_bicycle.hpp"
#include "trajectory/reference_searcher.hpp"
#include "trajectory/trajectory_sampler.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <stdexcept>

namespace {

SimulationSample makeSimulationSample(
    double time,
    const VehicleState& state,
    const TrajectoryPoint& reference,
    const ControlCommand& command,
    const SteeringActuatorOutput& actuator_output,
    ParkingMode parking_mode = ParkingMode::Forward,
    std::size_t segment_index = 0,
    double mode_elapsed_time = 0.0,
    bool state_transition = false,
    bool controller_reset = false,
    const LongitudinalCommand& longitudinal = {},
    const CoordinationOutput& coordination = {},
    double controller_computation_time_ms = 0.0) {
    SimulationSample sample;
    sample.time = time;
    sample.vehicle_state = state;
    sample.reference = reference;
    sample.command = command;
    sample.applied_steering = actuator_output.applied_steering;
    sample.steering_rate = actuator_output.steering_rate;
    sample.angle_limit_active = actuator_output.angle_limit_active;
    sample.rate_limit_active = actuator_output.rate_limit_active;
    sample.position_error =
        std::hypot(state.x - reference.x, state.y - reference.y);
    sample.lateral_error = calculateLateralError(state, reference);
    sample.longitudinal_error = calculateLongitudinalError(state, reference);
    sample.yaw_error = calculateYawError(state, reference);
    sample.parking_mode = parking_mode;
    sample.segment_index = segment_index;
    sample.mode_elapsed_time = mode_elapsed_time;
    sample.state_transition = state_transition;
    sample.controller_reset = controller_reset;
    sample.longitudinal = longitudinal;
    sample.coordination = coordination;
    sample.controller_computation_time_ms = controller_computation_time_ms;
    return sample;
}

Trajectory combineScenarioTrajectory(const ParkingScenario& scenario) {
    Trajectory combined;
    double time_offset = 0.0;
    double distance_offset = 0.0;
    for (std::size_t index = 0; index < scenario.segments.size(); ++index) {
        for (TrajectoryPoint point : scenario.segments[index].trajectory) {
            point.time += time_offset;
            point.s += distance_offset;
            combined.push_back(point);
        }
        time_offset = combined.back().time;
        distance_offset = combined.back().s;
        if (index + 1 < scenario.segments.size()) {
            time_offset += scenario.gear_change_wait_time;
        }
    }
    return combined;
}

VelocityStopControllerConfig makeLongitudinalConfig(
    const ParkingSimulationConfig& config) {
    // 比較モードに応じて機能を一つずつ累積的に有効化する。
    // 例：JerkLimitは速度FB＋加速度制約＋ジャーク制約を含む。
    VelocityStopControllerConfig longitudinal = config.longitudinal_controller;
    using Mode = ParkingSimulationConfig::LongitudinalMode;
    longitudinal.enable_acceleration_limit =
        config.longitudinal_mode == Mode::AccelerationLimit ||
        config.longitudinal_mode == Mode::JerkLimit ||
        config.longitudinal_mode == Mode::StopPosition ||
        config.longitudinal_mode == Mode::Coordinated;
    longitudinal.enable_jerk_limit =
        config.longitudinal_mode == Mode::JerkLimit ||
        config.longitudinal_mode == Mode::StopPosition ||
        config.longitudinal_mode == Mode::Coordinated;
    longitudinal.enable_stop_position_control =
        config.longitudinal_mode == Mode::StopPosition ||
        config.longitudinal_mode == Mode::Coordinated;
    return longitudinal;
}

}  // namespace

SimulationResult runParkingSimulation(
    const Trajectory& trajectory,
    const TrajectoryController& controller,
    const ParkingSimulationConfig& config) {
    if (trajectory.empty()) {
        throw std::invalid_argument("trajectory must not be empty");
    }
    if (config.wheelbase <= 0.0 || config.time_step <= 0.0) {
        throw std::invalid_argument(
            "wheelbase and time_step must be positive");
    }

    const TrajectorySampler sampler(trajectory);
    const ReferenceSearcher reference_searcher(trajectory);
    const VehicleState trajectory_initial_state{
        trajectory.front().x,
        trajectory.front().y,
        trajectory.front().yaw};
    const VehicleState initial_state =
        config.initial_state.value_or(trajectory_initial_state);
    KinematicBicycle vehicle(config.wheelbase, initial_state);
    SteeringActuator steering_actuator(config.steering_actuator);

    SimulationResult result;
    result.reference_trajectory = trajectory;

    const double end_time = trajectory.back().time;
    double time = 0.0;
    std::size_t nearest_index = 0;
    while (true) {
        // 1. 現在時刻の参照点と、現在位置の最近傍点を取得する。
        const TrajectoryPoint reference = sampler.sampleByTime(time);
        nearest_index = reference_searcher.findNearestIndex(
            vehicle.state(), nearest_index);

        // 2. 全制御器へ同じ入力情報を渡して制御指令を計算する。
        const auto controller_start = std::chrono::steady_clock::now();
        const ControlCommand command = controller.computeCommand(
            ControllerInput{
                time,
                vehicle.state(),
                reference,
                trajectory,
                nearest_index});
        const double controller_time_ms = std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - controller_start).count();

        const SteeringActuatorOutput actuator_output =
            steering_actuator.update(
                command.target_steering, config.time_step);

        // 3. 同時刻の状態、参照、指令、追従誤差を記録する。
        result.samples.push_back(makeSimulationSample(
            time, vehicle.state(), reference, command, actuator_output,
            ParkingMode::Forward, 0, 0.0, false, false, {}, {},
            controller_time_ms));

        if (time >= end_time - 1.0e-12) {
            break;
        }

        const double integration_dt =
            std::min(config.time_step, end_time - time);

        // 4. 制御指令で車両状態を次周期まで進める。
        vehicle.step(
            command.target_velocity,
            actuator_output.applied_steering,
            integration_dt);

        // 5. 次の制御周期へ進む。
        time += integration_dt;
    }

    result.final_state = vehicle.state();
    return result;
}

SimulationResult runParkingScenario(
    const ParkingScenario& scenario,
    const TrajectoryController& controller,
    const ParkingSimulationConfig& config) {
    ParkingStateMachine state_machine(scenario);
    if (config.wheelbase <= 0.0 || config.time_step <= 0.0) {
        throw std::invalid_argument("wheelbase and time_step must be positive");
    }

    const Trajectory& first_trajectory = scenario.segments.front().trajectory;
    const VehicleState default_initial{
        first_trajectory.front().x,
        first_trajectory.front().y,
        first_trajectory.front().yaw};
    KinematicBicycle vehicle(
        config.wheelbase, config.initial_state.value_or(default_initial));
    SteeringActuator steering_actuator(config.steering_actuator);
    controller.reset();
    const bool use_longitudinal_control =
        config.longitudinal_mode != ParkingSimulationConfig::LongitudinalMode::Ideal;
    VelocityStopController longitudinal_controller(makeLongitudinalConfig(config));
    LongitudinalLateralCoordinator coordinator(config.coordinator);

    SimulationResult result;
    result.reference_trajectory = combineScenarioTrajectory(scenario);
    result.parking_environment = scenario.environment;
    double global_time = 0.0;
    double segment_time = 0.0;
    std::size_t nearest_index = 0;
    bool controller_was_reset = true;
    bool previous_steering_limit_active = false;
    // 設定ミスで停止条件へ到達できない場合に無限ループへ入らないための上限。
    constexpr double kMaximumScenarioTime = 120.0;

    while (!state_machine.isCompleted()) {
        // 1. 状態機械が選んだ現在区間だけを参照対象にする。
        //    前進軌道と後退軌道が空間的に近くても別区間へ飛ばない。
        const std::size_t segment_index = state_machine.segmentIndex();
        const Trajectory& trajectory = scenario.segments[segment_index].trajectory;
        const bool driving = state_machine.isDriving();
        TrajectoryPoint reference;
        ControlCommand command;
        SteeringActuatorOutput actuator_output;
        LongitudinalCommand longitudinal_output;
        CoordinationOutput coordination_output;
        double controller_time_ms = 0.0;

        if (driving) {
            // 2. 横方向制御：最近傍点を探し、要求舵角と実舵角を求める。
            const ReferenceSearcher searcher(trajectory);
            nearest_index = searcher.findNearestIndex(vehicle.state(), nearest_index);
            if (use_longitudinal_control) {
                // 3. 縦横協調：横追従状態を「速度上限」へ変換する。
                //    横制御器の舵角そのものは変更しない。
                // 実車速を持つ場合は時間ではなく車両位置に近い軌道点を使う。
                // 始点は速度0なので、発進時だけ1点先を参照して正の速度目標を得る。
                const std::size_t reference_index = std::min(
                    nearest_index + (nearest_index + 1 < trajectory.size() ? 1U : 0U),
                    trajectory.size() - 1);
                reference = trajectory[reference_index];
            } else {
                const TrajectorySampler sampler(trajectory);
                reference = sampler.sampleByTime(segment_time);
            }
            const auto controller_start = std::chrono::steady_clock::now();
            command = controller.computeCommand(ControllerInput{
                segment_time, vehicle.state(), reference, trajectory, nearest_index});
            controller_time_ms = std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - controller_start).count();
            actuator_output = steering_actuator.update(
                command.target_steering, config.time_step);
            if (use_longitudinal_control) {
                const double lateral_error =
                    calculateLateralError(vehicle.state(), reference);
                const double yaw_error = calculateYawError(vehicle.state(), reference);
                const bool coordinated = config.longitudinal_mode ==
                    ParkingSimulationConfig::LongitudinalMode::Coordinated;
                coordination_output = coordinated
                    ? coordinator.coordinate({
                        reference.velocity,
                        reference.curvature,
                        lateral_error,
                        yaw_error,
                        previous_steering_limit_active})
                    : CoordinationOutput{reference.velocity};
                const double remaining_distance = std::max(
                    0.0, trajectory.back().s - trajectory[nearest_index].s);
                longitudinal_output = longitudinal_controller.computeCommand({
                    coordination_output.target_velocity,
                    vehicle.state().velocity,
                    remaining_distance,
                    config.time_step});
                command.target_velocity = longitudinal_output.target_velocity;
            }
        } else {
            // 4. ギア切り替え・最終停止中：速度目標を0、舵角を保持する。
            reference = trajectory.back();
            reference.velocity = 0.0;
            reference.acceleration = 0.0;
            command = {0.0, steering_actuator.steering()};
            actuator_output = {
                command.target_steering,
                steering_actuator.steering(),
                0.0,
                false,
                false};
            if (use_longitudinal_control) {
                longitudinal_output = longitudinal_controller.computeCommand({
                    0.0, vehicle.state().velocity, 0.0, config.time_step});
                longitudinal_output.stop_condition_satisfied =
                    std::abs(vehicle.state().velocity) <=
                    config.longitudinal_controller.velocity_tolerance;
            }
        }

        result.samples.push_back(makeSimulationSample(
            global_time,
            vehicle.state(),
            reference,
            command,
            actuator_output,
            state_machine.mode(),
            segment_index,
            state_machine.modeElapsedTime(),
            false,
            controller_was_reset,
            longitudinal_output,
            coordination_output,
            controller_time_ms));
        controller_was_reset = false;

        const bool segment_completed = driving && (use_longitudinal_control
            ? longitudinal_output.stop_condition_satisfied
            : segment_time >= trajectory.back().time - 1.0e-12);
        if (driving && !segment_completed) {
            // 5. 終端へ到達していなければ、制約後の操作量で次状態へ進める。
            const double dt = use_longitudinal_control
                ? config.time_step
                : std::min(config.time_step, trajectory.back().time - segment_time);
            if (use_longitudinal_control) {
                // LON-EQ-04, LON-EQ-05：制約後加速度で実車速と姿勢を同時更新。
                vehicle.stepWithAcceleration(
                    longitudinal_output.applied_acceleration,
                    actuator_output.applied_steering,
                    dt);
            } else {
                vehicle.step(command.target_velocity, actuator_output.applied_steering, dt);
            }
            segment_time = std::min(segment_time + dt, trajectory.back().time);
            previous_steering_limit_active =
                actuator_output.angle_limit_active || actuator_output.rate_limit_active;
            global_time += dt;
            if (global_time > kMaximumScenarioTime) {
                throw std::runtime_error("parking scenario did not converge");
            }
            continue;
        }

        const bool transitioned = state_machine.update(
            config.time_step, segment_completed);
        result.samples.back().state_transition = transitioned;
        if (transitioned && use_longitudinal_control && segment_completed) {
            // STOP-EQ-05, STOP-EQ-06：位置・速度許容値を満たしたので、数値上の
            // 微小速度を0へ確定し、ギア切り替え中の車両を静止させる。
            vehicle.stop();
        }
        global_time += config.time_step;
        if (transitioned && state_machine.isDriving()) {
            // 6. 新しい走行区間では最近傍探索、積分器、縦制御の履歴を初期化する。
            segment_time = 0.0;
            nearest_index = 0;
            controller.reset();
            longitudinal_controller.reset();
            controller_was_reset = true;
        }
    }

    result.final_state = vehicle.state();
    return result;
}
