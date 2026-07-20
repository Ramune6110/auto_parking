// Traceability: IMPL-DATA-01（制御器入出力の共通データ契約）
#pragma once

#include "core/types.hpp"

#include <cstddef>

/**
 * @brief 制御器が1周期ごとに要求する速度と前輪舵角。
 *
 * 舵角は後段のSteeringActuatorで制約・遅れを適用してから車両へ渡す。
 */
struct ControlCommand {
    double target_velocity{};  // 目標前後方向速度 [m/s]
    double target_steering{};  // 目標前輪舵角 [rad]
};

/**
 * @brief 全軌道追従制御器へ共通して渡す1周期分の入力。
 *
 * 時刻参照型、最近傍点型、注視点型の制御器を同じインターフェースで
 * 交換できるよう、現在状態と軌道参照情報をまとめている。
 */
struct ControllerInput {
    double time{};                         // シミュレーション時刻 [s]
    const VehicleState& current_state;     // 制御計算前の車両状態
    const TrajectoryPoint& time_reference; // 現在時刻で補間した参照点
    const Trajectory& trajectory;          // 注視点探索にも使う軌道全体
    std::size_t nearest_index{};           // 現在位置に最も近い軌道点番号
};
