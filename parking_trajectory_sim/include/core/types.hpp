// Traceability: IMPL-DATA-01（状態・経路点・軌道点の共通データ契約）
#pragma once

#include <vector>

/** @brief 時間情報を持たない、目標経路上の1点。 */
struct GeometricPathPoint {
    double s{};          // 経路始点からの累積距離 [m]
    double x{};          // 世界座標 x [m]
    double y{};          // 世界座標 y [m]
    double yaw{};        // 経路接線の方位角 [rad]
    double curvature{};  // 符号付き経路曲率 [1/m]
};

/** @brief 経路点へ時刻・速度・加速度を付加した目標軌道上の1点。 */
struct TrajectoryPoint {
    double time{};          // 軌道開始からの時刻 [s]
    double s{};             // 経路始点からの累積距離 [m]
    double x{};             // 目標世界座標 x [m]
    double y{};             // 目標世界座標 y [m]
    double yaw{};           // 目標方位角 [rad]
    double curvature{};     // 目標曲率 [1/m]
    double velocity{};      // 符号付き前後方向速度 [m/s]
    double acceleration{};  // 符号付き前後方向加速度 [m/s^2]
};

/**
 * @brief シミュレーションで使用する車両姿勢。
 *
 * 現段階では速度を目標軌道から直接与えるため、状態は位置と方位だけを持つ。
 */
struct VehicleState {
    constexpr VehicleState(
        double x_value = 0.0,
        double y_value = 0.0,
        double yaw_value = 0.0,
        double velocity_value = 0.0) noexcept
        : x(x_value), y(y_value), yaw(yaw_value), velocity(velocity_value) {}

    double x{};    // 後輪車軸中心の世界座標 x [m]
    double y{};    // 後輪車軸中心の世界座標 y [m]
    double yaw{};  // 車両方位角 [rad]
    double velocity{};  // 符号付き実車速 [m/s]。前進正、後退負
};

/** @brief 始点から終点へ順番に並んだ幾何学的な経路点列。 */
using GeometricPath = std::vector<GeometricPathPoint>;
/** @brief 時刻の昇順に並んだ目標軌道点列。 */
using Trajectory = std::vector<TrajectoryPoint>;
