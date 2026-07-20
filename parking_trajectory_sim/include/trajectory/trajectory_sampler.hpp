// Traceability: IMPL-SAMPLER-01
#pragma once

#include "core/types.hpp"

/** @brief 時系列の目標軌道から指定時刻の参照点を線形補間する。 */
class TrajectorySampler {
public:
    /** @param trajectory 空でなく、時刻の昇順に並んだ目標軌道。 */
    explicit TrajectorySampler(const Trajectory& trajectory);
    /**
     * @brief 指定時刻を挟む2点を検索し、軌道情報を線形補間する。
     * @param time 軌道開始からの時刻 [s]。範囲外は始点・終点へ固定する。
     */
    TrajectoryPoint sampleByTime(double time) const;

private:
    const Trajectory& trajectory_;
};
