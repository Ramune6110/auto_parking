// Traceability: IMPL-CONTROLLER-IF-01（全制御器に共通する交換可能な境界）
#pragma once

#include "core/control_types.hpp"
#include "core/types.hpp"

/** @brief 車両状態と参照軌道点から制御指令を計算する共通インターフェース。 */
class TrajectoryController {
public:
    virtual ~TrajectoryController() = default;

    /**
     * @brief 1制御周期に適用する速度指令と舵角指令を計算する。
     * @param input 現在状態、時間参照点、最近傍点をまとめた制御入力。
     * @return 操舵アクチュエータと車両モデルへ渡す制御指令。
     */
    virtual ControlCommand computeCommand(
        const ControllerInput& input) const = 0;

    /** @brief 新しい走行区間の開始前に区間依存の内部状態を初期化する。 */
    virtual void reset() const {}
};
