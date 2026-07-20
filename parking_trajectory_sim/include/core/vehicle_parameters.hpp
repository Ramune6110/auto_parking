// Traceability: IMPL-VEHICLE-01, BICYCLE-EQ-03
#pragma once

/** @brief シミュレーション、可視化、理論図で共有する基準車両諸元。 */
namespace vehicle_parameters {

inline constexpr double kWheelbase = 2.7;             // ホイールベース [m]
inline constexpr double kVehicleWidth = 1.8;          // 車幅 [m]
inline constexpr double kFrontOverhang = 0.9;         // 前輪軸から車体前端 [m]
inline constexpr double kRearOverhang = 0.8;          // 後輪軸から車体後端 [m]
inline constexpr double kCogFromRearAxle = 1.2;       // 後輪軸からCOG [m]
inline constexpr double kTrackWidth = 1.5;            // 左右輪中心間 [m]
inline constexpr double kMaxSteeringAngleDeg = 35.0;  // 最大絶対舵角 [deg]

}  // namespace vehicle_parameters
