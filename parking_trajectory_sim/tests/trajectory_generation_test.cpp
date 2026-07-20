// Verifies: PATH-EQ-04, PATH-EQ-05, PATH-EQ-10〜PATH-EQ-12,
// TRAJ-EQ-09, TRAJ-EQ-11〜TRAJ-EQ-17
#include "trajectory/path_generator.hpp"
#include "trajectory/velocity_profile.hpp"

#include <cmath>
#include <iostream>
#include <stdexcept>

namespace {

void requireNear(double actual, double expected, double tolerance) {
    if (std::abs(actual - expected) > tolerance) {
        throw std::runtime_error("unexpected trajectory generation value");
    }
}

// Verifies: PATH-EQ-05（前進・後退の直線位置更新）
void testStraightPathInBothDirections() {
    PathGenerator forward(0.5, 1.0);
    forward.addStraight(1.0);
    requireNear(forward.points().back().x, 1.0, 1.0e-12);

    PathGenerator reverse(0.5, -1.0);
    reverse.addStraight(1.0);
    requireNear(reverse.points().back().x, -1.0, 1.0e-12);
}

// Verifies: PATH-EQ-04, PATH-EQ-10, PATH-EQ-11, PATH-EQ-12
void testArcEndpoint() {
    PathGenerator path(0.05, 1.0);
    path.addArc(2.0, 0.5 * 3.14159265358979323846, 1.0);
    requireNear(path.points().back().x, 2.0, 1.0e-10);
    requireNear(path.points().back().y, 2.0, 1.0e-10);
    requireNear(path.points().back().yaw, 0.5 * 3.14159265358979323846, 1.0e-10);
    requireNear(path.points().back().curvature, 0.5, 1.0e-12);
}

// Verifies: TRAJ-EQ-09, TRAJ-EQ-11〜TRAJ-EQ-17
void testVelocityProfileSignsAndEndpoints() {
    PathGenerator path(0.25, 1.0);
    path.addStraight(2.0);
    const VelocityProfileConfig config{1.0, 0.5, 0.5};
    const Trajectory forward = createVelocityProfile(path.points(), config, 1.0);
    const Trajectory reverse = createVelocityProfile(path.points(), config, -1.0);
    requireNear(forward.front().velocity, 0.0, 1.0e-12);
    requireNear(forward.back().velocity, 0.0, 1.0e-12);
    if (forward[1].velocity <= 0.0 || reverse[1].velocity >= 0.0 ||
        forward.back().time <= 0.0) {
        throw std::runtime_error("velocity sign or time is invalid");
    }
}

}  // namespace

int main() {
    try {
        testStraightPathInBothDirections();
        testArcEndpoint();
        testVelocityProfileSignsAndEndpoints();
        std::cout << "All trajectory generation tests passed.\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "Test failed: " << error.what() << '\n';
        return 1;
    }
}
