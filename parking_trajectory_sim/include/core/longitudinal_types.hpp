// Traceability: IMPL-LONG-DATA-01（縦制御の入出力とログ契約）
#pragma once

struct LongitudinalControllerInput {
    double reference_velocity{};   // 軌道が要求する符号付き速度 [m/s]
    double current_velocity{};     // 車両の符号付き実車速 [m/s]
    double remaining_distance{};   // 現在区間終端までの経路長 [m]
    double time_step{};            // 制御周期 [s]
};

struct LongitudinalCommand {
    double reference_velocity{};
    double target_velocity{};          // 停止位置・協調上限を反映した速度 [m/s]
    double velocity_error{};
    double requested_acceleration{};   // P制御直後の要求加速度 [m/s^2]
    double applied_acceleration{};     // 制約後に車両へ適用する加速度 [m/s^2]
    double jerk{};                     // 適用加速度の変化率 [m/s^3]
    double remaining_distance{};
    double stopping_velocity_limit{};
    bool acceleration_limit_active{};
    bool jerk_limit_active{};
    bool stop_position_control_active{};
    bool stop_condition_satisfied{};
};

struct CoordinationInput {
    double reference_velocity{};
    double curvature{};
    double lateral_error{};
    double yaw_error{};
    bool steering_limit_active{};
};

struct CoordinationOutput {
    double target_velocity{};
    double curvature_speed_limit{};
    double lateral_error_speed_limit{};
    double yaw_error_speed_limit{};
    bool coordination_limit_active{};
};
