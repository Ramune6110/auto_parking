# 前進・停止・後退の状態管理

## 1. この章の目的

軌道追従制御器の上位で、前進、ギア切り替え停止、後退、最終停止を安全な順序で実行する方法を説明します。この段階では車速動特性をまだ持たないため、停止は速度指令を0にする簡易モデルです。

## 2. なぜ状態管理が必要か

前進軌道と後退軌道を一つの点列として最近傍探索すると、空間的に近い別区間へ参照点が飛ぶ可能性があります。そこで、上位状態管理が現在有効な`DrivingSegment`を一つだけ選び、その区間だけを制御器へ渡します。

```text
ParkingStateMachine
        │ 現在のsegment_indexを選択
        ▼
DrivingSegment::trajectory
        │
        ▼
TrajectoryController::computeCommand()
        │
        ▼
SteeringActuator → KinematicBicycle
```

## 3. 状態遷移

```text
Forward
  │ 前進区間の終端
  ▼
GearChangeStop ── 停止待ち時間 ──► Reverse
                                      │ 後退区間の終端
                                      ▼
                                  FinalStop
                                      │ 最終停止時間
                                      ▼
                                  Completed
```

| 状態 | 速度指令 | 参照軌道 | 操舵 | 終了条件 |
|---|---:|---|---|---|
| `Forward` | 前進軌道の速度 | 前進区間のみ | 選択した制御器 | 区間終端時刻 |
| `GearChangeStop` | 0 | 前進区間終端 | 実舵角を保持 | 停止待ち時間 |
| `Reverse` | 後退軌道の速度 | 後退区間のみ | 同じ制御器を再利用 | 区間終端時刻 |
| `FinalStop` | 0 | 後退区間終端 | 実舵角を保持 | 最終停止時間 |
| `Completed` | ― | ― | ― | 完了 |

### 3.1 基準となる切り返し駐車軌道

`--forward-stop-reverse`は、道路上の初期点から切り返し点まで前進旋回し、後退旋回で車体方位を駐車枠へ合わせてから直線後退します。

```text
初期点 (-6, 1, 0 deg)
  └─ 前進直線 3 m
      └─ 左旋回 R=4 m、60 deg
          └─ 切り返し点
              └─ 後退右旋回 R=4 m、30 deg
                  └─ 方位90 degで直線後退 4 m
                      └─ 目標点（駐車枠内）
```

前進軌道は緑、後退軌道は橙、駐車枠は斜線、左右の駐車車両相当の障害物は灰色で描画します。障害物回避をオンライン計算する段階ではなく、衝突しない入力軌道を追従する段階です。

## 4. 区間切り替え時の扱い

車両姿勢と実舵角は保持します。最近傍インデックスと区間内時刻は0へ戻し、積分器付きLQRの積分値とゲインキャッシュは`TrajectoryController::reset()`で初期化します。

## 5. ログの読み方

| 列 | 意味 |
|---|---|
| `parking_mode` | 現在の状態名 |
| `segment_index` | 現在選択している走行区間 |
| `mode_elapsed_time` | 現在状態へ入ってからの時間 |
| `state_transition` | その周期で状態遷移したか |
| `controller_reset` | 新区間開始時に制御器をリセットしたか |

時系列図では`gear_change_stop`を薄黄色、`final_stop`を薄灰色で表示します。

## 6. ソースコードとの対応

| 処理ID | 宣言・実装 | 検証 |
|---|---|---|
| `IMPL-SCENARIO-FSM-01` | `include/scenario/parking_scenario.hpp`、`src/scenario/parking_scenario.cpp` | `parking_state_machine_test.cpp` |
| `IMPL-SCENARIO-SAMPLE-01` | `include/scenario/sample_parking_scenario.hpp`、`src/scenario/sample_parking_scenario.cpp` | 複合シナリオ実行 |
| `IMPL-PARKING-ENV-01` | `parking_environment.csv`、`python/parking_environment.py` | 駐車枠・障害物・三つの要点の共通描画 |
| `IMPL-SIM-01` | `runParkingScenario()` | `testForwardStopReverseScenarioIntegration()` |

全体の関係は[トレーサビリティ](../traceability.md)も参照してください。

## 7. 現段階の限界

- 実車速を状態として持たず、速度指令0を停止とみなします。
- 加速度、ブレーキ、ジャーク、シフト応答はモデル化していません。
- 停止中は据え切りせず、実舵角を保持します。

この状態管理を現実的な停止条件へ接続する理論は、[車速状態と速度フィードバック制御](../controllers/longitudinal_velocity_control.md)、[加減速度・ジャーク制約](../controllers/longitudinal_constraints.md)、[停止位置制御](../controllers/stop_position_control.md)を参照してください。横追従状態から速度も調整する処理は[縦方向・横方向制御の協調](../controllers/longitudinal_lateral_coordination.md)で説明します。
