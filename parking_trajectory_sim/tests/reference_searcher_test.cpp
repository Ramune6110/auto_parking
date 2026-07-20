// Verifies: PP-EQ-01, PP-EQ-02, STANLEY-EQ-01
#include "trajectory/reference_searcher.hpp"

#include <iostream>
#include <stdexcept>

namespace {

Trajectory makeStraightTrajectory() {
    return {
        {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0},
        {1.0, 1.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0},
        {2.0, 2.0, 2.0, 0.0, 0.0, 0.0, 1.0, 0.0},
        {3.0, 3.0, 3.0, 0.0, 0.0, 0.0, 1.0, 0.0}};
}

void testNearestIndex() {
    const Trajectory trajectory = makeStraightTrajectory();
    const ReferenceSearcher searcher(trajectory);
    const std::size_t index = searcher.findNearestIndex(
        VehicleState{1.2, 0.3, 0.0}, 0);
    if (index != 1) {
        throw std::runtime_error("nearest index must be 1");
    }
}

void testSearchDoesNotMoveBehindPreviousIndex() {
    const Trajectory trajectory = makeStraightTrajectory();
    const ReferenceSearcher searcher(trajectory);
    const std::size_t index = searcher.findNearestIndex(
        VehicleState{0.0, 0.0, 0.0}, 2);
    if (index != 2) {
        throw std::runtime_error("nearest search moved behind previous index");
    }
}

void testLookaheadAndEndClamp() {
    const Trajectory trajectory = makeStraightTrajectory();
    const ReferenceSearcher searcher(trajectory);
    if (searcher.findLookaheadIndex(1, 1.5) != 3) {
        throw std::runtime_error("lookahead index must be 3");
    }
    if (searcher.findLookaheadIndex(2, 5.0) != 3) {
        throw std::runtime_error("lookahead must clamp to trajectory end");
    }
}

}  // namespace

int main() {
    try {
        testNearestIndex();
        testSearchDoesNotMoveBehindPreviousIndex();
        testLookaheadAndEndClamp();
        std::cout << "All reference searcher tests passed.\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "Test failed: " << error.what() << '\n';
        return 1;
    }
}
