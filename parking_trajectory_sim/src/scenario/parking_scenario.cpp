// Traceability: IMPL-SCENARIO-FSM-01（駐車モード遷移）
#include "scenario/parking_scenario.hpp"

#include <stdexcept>

const char* parkingModeName(ParkingMode mode) noexcept {
    switch (mode) {
    case ParkingMode::Forward: return "forward";
    case ParkingMode::GearChangeStop: return "gear_change_stop";
    case ParkingMode::Reverse: return "reverse";
    case ParkingMode::FinalStop: return "final_stop";
    case ParkingMode::Completed: return "completed";
    }
    return "unknown";
}

ParkingStateMachine::ParkingStateMachine(const ParkingScenario& scenario)
    : scenario_(scenario), mode_(ParkingMode::Completed) {
    if (scenario_.segments.empty()) {
        throw std::invalid_argument("parking scenario must contain a segment");
    }
    if (scenario_.gear_change_wait_time < 0.0 ||
        scenario_.final_stop_time < 0.0) {
        throw std::invalid_argument("parking stop times must not be negative");
    }
    for (const DrivingSegment& segment : scenario_.segments) {
        if (segment.trajectory.empty()) {
            throw std::invalid_argument("driving segment must not be empty");
        }
    }
    mode_ = drivingMode(0);
}

ParkingMode ParkingStateMachine::mode() const noexcept { return mode_; }
std::size_t ParkingStateMachine::segmentIndex() const noexcept { return segment_index_; }
double ParkingStateMachine::modeElapsedTime() const noexcept { return mode_elapsed_time_; }
bool ParkingStateMachine::isDriving() const noexcept {
    return mode_ == ParkingMode::Forward || mode_ == ParkingMode::Reverse;
}
bool ParkingStateMachine::isCompleted() const noexcept {
    return mode_ == ParkingMode::Completed;
}

ParkingMode ParkingStateMachine::drivingMode(std::size_t index) const {
    return scenario_.segments[index].direction == MotionDirection::Forward
        ? ParkingMode::Forward : ParkingMode::Reverse;
}

bool ParkingStateMachine::update(double dt, bool active_segment_completed) {
    if (dt <= 0.0) {
        throw std::invalid_argument("state machine dt must be positive");
    }
    if (isCompleted()) {
        return false;
    }
    mode_elapsed_time_ += dt;
    if (isDriving() && active_segment_completed) {
        mode_elapsed_time_ = 0.0;
        mode_ = segment_index_ + 1 < scenario_.segments.size()
            ? ParkingMode::GearChangeStop : ParkingMode::FinalStop;
        return true;
    }
    if (mode_ == ParkingMode::GearChangeStop &&
        mode_elapsed_time_ + 1.0e-12 >= scenario_.gear_change_wait_time) {
        ++segment_index_;
        mode_ = drivingMode(segment_index_);
        mode_elapsed_time_ = 0.0;
        return true;
    }
    if (mode_ == ParkingMode::FinalStop &&
        mode_elapsed_time_ + 1.0e-12 >= scenario_.final_stop_time) {
        mode_ = ParkingMode::Completed;
        mode_elapsed_time_ = 0.0;
        return true;
    }
    return false;
}
