// Traceability: PP-EQ-01, PP-EQ-02, STANLEY-EQ-01
#pragma once

#include "core/types.hpp"

#include <cstddef>

/** @brief 車両位置を基準に最近傍点と注視点を探索する。 */
class ReferenceSearcher {
public:
    explicit ReferenceSearcher(const Trajectory& trajectory);

    /**
     * @brief 前回点以降から車両位置に最も近い軌道点を探索する。
     * @param state 現在の車両状態。
     * @param previous_index 前周期の最近傍点インデックス。
     */
    std::size_t findNearestIndex(
        const VehicleState& state,
        std::size_t previous_index) const;

    /**
     * @brief 最近傍点から経路距離でlookahead_distance以上先の点を返す。
     */
    std::size_t findLookaheadIndex(
        std::size_t nearest_index,
        double lookahead_distance) const;

private:
    const Trajectory& trajectory_;
};
