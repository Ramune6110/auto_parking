// Traceability: IMPL-APP-01（シナリオ選択、実行、結果保存の組み立て）
#include "controllers/feedforward_controller.hpp"
#include "controllers/lateral_feedback_controller.hpp"
#include "controllers/lqr_controller.hpp"
#include "controllers/linear_mpc_controller.hpp"
#include "controllers/pure_pursuit_controller.hpp"
#include "controllers/stanley_controller.hpp"
#include "controllers/yaw_feedback_controller.hpp"
#include "core/vehicle_parameters.hpp"
#include "io/csv_writer.hpp"
#include "simulation/parking_simulator.hpp"
#include "scenario/sample_parking_scenario.hpp"
#include "trajectory/sample_trajectories.hpp"

#include <exception>
#include <cmath>
#include <iostream>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>

namespace {

// 学習ケースで共通使用する車両諸元、制御ゲイン、初期偏差、操舵制約。
constexpr double kWheelbase = vehicle_parameters::kWheelbase;
constexpr double kYawErrorGain = 1.0;
constexpr double kLateralErrorGain = 0.2;
constexpr double kPurePursuitLookahead = 1.5;
constexpr double kStanleyLateralGain = 0.5;
constexpr double kStanleySofteningVelocity = 1.0;
constexpr double kInitialLongitudinalOffset = 0.3;
constexpr double kInitialLateralOffset = 0.5;
constexpr double kInitialYawOffset = 5.0 * 3.14159265358979323846 / 180.0;
constexpr double kPi = 3.14159265358979323846;
constexpr double kMaxSteeringAngle =
    vehicle_parameters::kMaxSteeringAngleDeg * kPi / 180.0;
constexpr double kMaxSteeringRate = 60.0 * kPi / 180.0;
constexpr double kSteeringTimeConstant = 0.3;

/** @brief コマンドラインから選択できる軌道追従制御器。 */
enum class ControllerKind {
    Feedforward,
    YawFeedback,
    LateralFeedback,
    PurePursuit,
    Stanley,
    Lqr,
    LqrIntegral,
    LinearMpc
};

/** @brief 軌道の進行方向と、初期姿勢に与える偏差の組み合わせ。 */
enum class Scenario {
    Forward,
    Reverse,
    ForwardLateralOffset,
    ForwardPoseOffset,
    ReverseLateralOffset,
    ReversePoseOffset,
    ForwardStopReverse,
    ForwardStopReverseSimple
};

enum class SteeringModelKind {
    Ideal,
    AngleLimit,
    AngleRateLimit,
    Actuator
};

/** @brief コマンドライン引数を型付きの設定値へ変換した結果。 */
struct CommandLineOptions {
    ControllerKind controller{ControllerKind::Feedforward};
    Scenario scenario{Scenario::Forward};
    SteeringModelKind steering_model{SteeringModelKind::Ideal};
    bool steering_model_was_set{};
    double steering_bias{};
    ParkingSimulationConfig::LongitudinalMode longitudinal_mode{
        ParkingSimulationConfig::LongitudinalMode::Ideal};
};

bool tryParseScenario(const std::string& option, Scenario& scenario) {
    if (option == "--reverse") {
        scenario = Scenario::Reverse;
    } else if (option == "--forward-lateral-offset") {
        scenario = Scenario::ForwardLateralOffset;
    } else if (option == "--forward-pose-offset") {
        scenario = Scenario::ForwardPoseOffset;
    } else if (option == "--reverse-lateral-offset") {
        scenario = Scenario::ReverseLateralOffset;
    } else if (option == "--reverse-pose-offset") {
        scenario = Scenario::ReversePoseOffset;
    } else if (option == "--forward-stop-reverse") {
        scenario = Scenario::ForwardStopReverse;
    } else if (option == "--forward-stop-reverse-simple") {
        scenario = Scenario::ForwardStopReverseSimple;
    } else {
        return false;
    }
    return true;
}

CommandLineOptions parseCommandLine(int argc, char* argv[]) {
    CommandLineOptions options;
    bool scenario_was_set = false;

    // 各引数を左から読み、値を伴うオプションでは次の引数も消費する。
    for (int i = 1; i < argc; ++i) {
        const std::string option = argv[i];
        if (option == "--controller") {
            if (i + 1 >= argc) {
                throw std::invalid_argument("--controller requires a value");
            }
            const std::string controller_name = argv[++i];
            if (controller_name == "feedforward") {
                options.controller = ControllerKind::Feedforward;
            } else if (controller_name == "yaw-feedback") {
                options.controller = ControllerKind::YawFeedback;
            } else if (controller_name == "lateral-feedback") {
                options.controller = ControllerKind::LateralFeedback;
            } else if (controller_name == "pure-pursuit") {
                options.controller = ControllerKind::PurePursuit;
            } else if (controller_name == "stanley") {
                options.controller = ControllerKind::Stanley;
            } else if (controller_name == "lqr") {
                options.controller = ControllerKind::Lqr;
            } else if (controller_name == "lqr-integral") {
                options.controller = ControllerKind::LqrIntegral;
            } else if (controller_name == "linear-mpc") {
                options.controller = ControllerKind::LinearMpc;
            } else {
                throw std::invalid_argument(
                    "controller must be feedforward, yaw-feedback, or "
                    "lateral-feedback, pure-pursuit, stanley, lqr, or "
                    "lqr-integral, or linear-mpc");
            }
            continue;
        }
        if (option == "--steering-bias-deg") {
            if (i + 1 >= argc) {
                throw std::invalid_argument(
                    "--steering-bias-deg requires a value");
            }
            options.steering_bias = std::stod(argv[++i]) * kPi / 180.0;
            continue;
        }
        if (option == "--steering-model") {
            if (i + 1 >= argc) {
                throw std::invalid_argument("--steering-model requires a value");
            }
            const std::string model_name = argv[++i];
            if (model_name == "ideal") {
                options.steering_model = SteeringModelKind::Ideal;
            } else if (model_name == "angle-limit") {
                options.steering_model = SteeringModelKind::AngleLimit;
            } else if (model_name == "angle-rate-limit") {
                options.steering_model = SteeringModelKind::AngleRateLimit;
            } else if (model_name == "actuator") {
                options.steering_model = SteeringModelKind::Actuator;
            } else {
                throw std::invalid_argument(
                    "steering model must be ideal, angle-limit, "
                    "angle-rate-limit, or actuator");
            }
            options.steering_model_was_set = true;
            continue;
        }
        if (option == "--longitudinal-model") {
            if (i + 1 >= argc) {
                throw std::invalid_argument("--longitudinal-model requires a value");
            }
            const std::string name = argv[++i];
            using Mode = ParkingSimulationConfig::LongitudinalMode;
            if (name == "ideal") options.longitudinal_mode = Mode::Ideal;
            else if (name == "velocity-feedback") options.longitudinal_mode = Mode::VelocityFeedback;
            else if (name == "acceleration-limit") options.longitudinal_mode = Mode::AccelerationLimit;
            else if (name == "jerk-limit") options.longitudinal_mode = Mode::JerkLimit;
            else if (name == "stop-position") options.longitudinal_mode = Mode::StopPosition;
            else if (name == "coordinated") options.longitudinal_mode = Mode::Coordinated;
            else throw std::invalid_argument(
                "longitudinal model must be ideal, velocity-feedback, "
                "acceleration-limit, jerk-limit, stop-position, or coordinated");
            continue;
        }

        Scenario parsed_scenario;
        if (!tryParseScenario(option, parsed_scenario)) {
            throw std::invalid_argument("unknown option: " + option);
        }
        if (scenario_was_set) {
            throw std::invalid_argument("specify at most one scenario option");
        }
        options.scenario = parsed_scenario;
        scenario_was_set = true;
    }
    return options;
}

std::string steeringModelName(SteeringModelKind model) {
    switch (model) {
    case SteeringModelKind::Ideal:
        return "ideal";
    case SteeringModelKind::AngleLimit:
        return "angle_limit";
    case SteeringModelKind::AngleRateLimit:
        return "angle_rate_limit";
    case SteeringModelKind::Actuator:
        return "actuator";
    }
    throw std::logic_error("unhandled steering model");
}

std::string longitudinalModeName(ParkingSimulationConfig::LongitudinalMode mode) {
    using Mode = ParkingSimulationConfig::LongitudinalMode;
    switch (mode) {
    case Mode::Ideal: return "ideal_velocity";
    case Mode::VelocityFeedback: return "velocity_feedback";
    case Mode::AccelerationLimit: return "acceleration_limit";
    case Mode::JerkLimit: return "jerk_limit";
    case Mode::StopPosition: return "stop_position";
    case Mode::Coordinated: return "coordinated";
    }
    throw std::logic_error("unhandled longitudinal mode");
}

SteeringActuatorConfig createSteeringActuatorConfig(
    SteeringModelKind model,
    double steering_bias) {
    SteeringActuatorConfig config;
    config.max_steering_angle = kMaxSteeringAngle;
    config.max_steering_rate = kMaxSteeringRate;
    config.time_constant = kSteeringTimeConstant;
    config.steering_bias = steering_bias;
    config.enable_angle_limit = model != SteeringModelKind::Ideal;
    config.enable_rate_limit =
        model == SteeringModelKind::AngleRateLimit ||
        model == SteeringModelKind::Actuator;
    config.enable_first_order_lag = model == SteeringModelKind::Actuator;
    return config;
}

bool usesReverseTrajectory(Scenario scenario) {
    return scenario == Scenario::Reverse ||
        scenario == Scenario::ReverseLateralOffset ||
        scenario == Scenario::ReversePoseOffset;
}

bool usesParkingStateMachine(Scenario scenario) {
    return scenario == Scenario::ForwardStopReverse ||
        scenario == Scenario::ForwardStopReverseSimple;
}

Trajectory createTrajectory(
    Scenario scenario,
    const SampleTrajectoryConfig& config) {
    return usesReverseTrajectory(scenario)
        ? createReverseSampleTrajectory(config)
        : createForwardSampleTrajectory(config);
}

std::string controllerName(ControllerKind controller) {
    switch (controller) {
    case ControllerKind::Feedforward:
        return "feedforward";
    case ControllerKind::YawFeedback:
        return "yaw_feedback";
    case ControllerKind::LateralFeedback:
        return "lateral_feedback";
    case ControllerKind::PurePursuit:
        return "pure_pursuit";
    case ControllerKind::Stanley:
        return "stanley";
    case ControllerKind::Lqr:
        return "lqr";
    case ControllerKind::LqrIntegral:
        return "lqr_integral";
    case ControllerKind::LinearMpc:
        return "linear_mpc";
    }
    throw std::logic_error("unhandled controller");
}

std::string scenarioName(Scenario scenario) {
    switch (scenario) {
    case Scenario::Forward:
        return "forward";
    case Scenario::Reverse:
        return "reverse";
    case Scenario::ForwardLateralOffset:
        return "forward_lateral_offset";
    case Scenario::ForwardPoseOffset:
        return "forward_pose_offset";
    case Scenario::ReverseLateralOffset:
        return "reverse_lateral_offset";
    case Scenario::ReversePoseOffset:
        return "reverse_pose_offset";
    case Scenario::ForwardStopReverse:
        return "forward_stop_reverse";
    case Scenario::ForwardStopReverseSimple:
        return "forward_stop_reverse_simple";
    }
    throw std::logic_error("unhandled scenario");
}

std::optional<VehicleState> initialState(Scenario scenario) {
    // nullopt の基準ケースでは、シミュレータが軌道始点を初期姿勢に使う。
    switch (scenario) {
    case Scenario::ForwardLateralOffset:
    case Scenario::ReverseLateralOffset:
        return VehicleState{0.0, kInitialLateralOffset, 0.0};
    case Scenario::ForwardPoseOffset:
    case Scenario::ReversePoseOffset:
        return VehicleState{
            kInitialLongitudinalOffset,
            kInitialLateralOffset,
            kInitialYawOffset};
    case Scenario::Forward:
    case Scenario::Reverse:
    case Scenario::ForwardStopReverse:
    case Scenario::ForwardStopReverseSimple:
        return std::nullopt;
    }
    throw std::logic_error("unhandled scenario");
}

std::unique_ptr<TrajectoryController> createController(
    ControllerKind controller,
    SteeringModelKind steering_model) {
    // すべての制御器を共通インターフェースで返し、mainを個別実装から分離する。
    if (controller == ControllerKind::YawFeedback) {
        return std::make_unique<YawFeedbackController>(
            kWheelbase, kYawErrorGain);
    }
    if (controller == ControllerKind::LateralFeedback) {
        return std::make_unique<LateralFeedbackController>(
            kWheelbase, kYawErrorGain, kLateralErrorGain);
    }
    if (controller == ControllerKind::PurePursuit) {
        return std::make_unique<PurePursuitController>(
            kWheelbase, kPurePursuitLookahead);
    }
    if (controller == ControllerKind::Stanley) {
        return std::make_unique<StanleyController>(
            kWheelbase,
            kStanleyLateralGain,
            kStanleySofteningVelocity);
    }
    if (controller == ControllerKind::Lqr ||
        controller == ControllerKind::LqrIntegral) {
        LqrControllerConfig config;
        config.wheelbase = kWheelbase;
        config.time_step = 0.01;
        config.enable_integral_action =
            controller == ControllerKind::LqrIntegral;
        // 実舵角に制約がある場合だけ、積分器のアンチワインドアップを有効にする。
        config.anti_windup_steering_limit =
            steering_model == SteeringModelKind::Ideal
                ? 0.0
                : kMaxSteeringAngle;
        return std::make_unique<LqrController>(config);
    }
    if (controller == ControllerKind::LinearMpc) {
        LinearMpcControllerConfig config;
        config.wheelbase = kWheelbase;
        config.time_step = 0.01;
        config.maximum_steering_angle = kMaxSteeringAngle;
        config.maximum_steering_rate = kMaxSteeringRate;
        return std::make_unique<LinearMpcController>(config);
    }
    return std::make_unique<FeedforwardController>(kWheelbase);
}

void printSummary(
    const SimulationResult& result,
    const std::string& output_directory) {
    const SimulationSample& first_sample = result.samples.front();
    const SimulationSample& final_sample = result.samples.back();
    std::cout << "Simulation completed.\n"
              << "Initial lateral error    : "
              << first_sample.lateral_error << " m\n"
              << "Initial longitudinal err.: "
              << first_sample.longitudinal_error << " m\n"
              << "Initial yaw error        : "
              << first_sample.yaw_error << " rad\n"
              << "Final lateral error      : "
              << final_sample.lateral_error << " m\n"
              << "Final longitudinal error : "
              << final_sample.longitudinal_error << " m\n"
              << "Final yaw error          : "
              << final_sample.yaw_error << " rad\n"
              << "CSV output               : " << output_directory << '\n';
}

}  // namespace

int main(int argc, char* argv[]) {
    try {
        // 1. 実行条件を読み取り、軌道・制御器・シミュレーション設定を組み立てる。
        const CommandLineOptions options = parseCommandLine(argc, argv);
        const SampleTrajectoryConfig trajectory_config;
        const Trajectory trajectory = usesParkingStateMachine(options.scenario)
            ? Trajectory{}
            : createTrajectory(options.scenario, trajectory_config);
        const std::unique_ptr<TrajectoryController> controller =
            createController(options.controller, options.steering_model);
        ParkingSimulationConfig simulation_config{
            kWheelbase,
            0.01,
            initialState(options.scenario),
            createSteeringActuatorConfig(
                options.steering_model, options.steering_bias)};
        simulation_config.longitudinal_mode = options.longitudinal_mode;
        // 制御器とシナリオごとに出力先を分け、比較結果の上書きを防ぐ。
        std::string output_directory =
            "output/" + controllerName(options.controller) + "/" +
            scenarioName(options.scenario);
        if (options.steering_model_was_set) {
            output_directory += "/" +
                steeringModelName(options.steering_model);
            if (std::abs(options.steering_bias) > 1.0e-12) {
                output_directory += "_bias";
            }
        }
        if (options.longitudinal_mode !=
            ParkingSimulationConfig::LongitudinalMode::Ideal) {
            output_directory += "/" + longitudinalModeName(options.longitudinal_mode);
        }

        // 2. 軌道追従シミュレーションを実行する。
        const ParkingScenario parking_scenario =
            options.scenario == Scenario::ForwardStopReverseSimple
                ? createSimpleForwardStopReverseScenario(trajectory_config)
                : createForwardStopReverseScenario(trajectory_config);
        const SimulationResult result = usesParkingStateMachine(options.scenario)
            ? runParkingScenario(
                parking_scenario,
                *controller,
                simulation_config)
            : runParkingSimulation(trajectory, *controller, simulation_config);

        // 3. 数値結果を保存し、主要な初期・最終誤差を画面へ表示する。
        writeSimulationCsvFiles(result, output_directory, kWheelbase);

        printSummary(result, output_directory);
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "Error: " << error.what() << '\n';
        return 1;
    }
}
