// Traceability: IMPL-SCENARIO-FSM-01（前進・停止・後退の上位状態管理）
#pragma once

#include "core/types.hpp"

#include <cstddef>
#include <string>
#include <vector>

enum class MotionDirection { Forward, Reverse };

enum class ParkingMode {
    Forward,
    GearChangeStop,
    Reverse,
    FinalStop,
    Completed
};

struct DrivingSegment {
    MotionDirection direction{MotionDirection::Forward};
    Trajectory trajectory;
};

struct AxisAlignedRectangle {
    double minimum_x{};
    double maximum_x{};
    double minimum_y{};
    double maximum_y{};
};

struct ParkingEnvironment {
    bool enabled{};
    AxisAlignedRectangle parking_space;
    std::vector<AxisAlignedRectangle> obstacles;
    VehicleState initial_pose;
    VehicleState reversal_pose;
    VehicleState target_pose;
};

struct ParkingScenario {
    std::vector<DrivingSegment> segments;
    double gear_change_wait_time{1.0};
    double final_stop_time{0.5};
    ParkingEnvironment environment;
};

const char* parkingModeName(ParkingMode mode) noexcept;

class ParkingStateMachine {
public:
    explicit ParkingStateMachine(const ParkingScenario& scenario);

    ParkingMode mode() const noexcept;
    std::size_t segmentIndex() const noexcept;
    double modeElapsedTime() const noexcept;
    bool isDriving() const noexcept;
    bool isCompleted() const noexcept;

    /** @brief 1周期進め、必要なら走行・停止状態を遷移する。 */
    bool update(double dt, bool active_segment_completed);

private:
    ParkingMode drivingMode(std::size_t segment_index) const;

    const ParkingScenario& scenario_;
    ParkingMode mode_;
    std::size_t segment_index_{};
    double mode_elapsed_time_{};
};
