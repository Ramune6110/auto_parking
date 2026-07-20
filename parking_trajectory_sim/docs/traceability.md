# 理論・数式・ソースコード トレーサビリティ

## 1. この文書の目的

この文書は、理論資料を読んだ後に「どのC++関数を読めばよいか」、ソースコードを読んだときに「どの数式を実装しているか」を双方向に調べるための索引です。

```text
図・数式
  ↕ 理論ID
C++クラス／関数
  ↕ C++変数
単体テスト
```

行番号はコード変更でずれるため使用しません。理論ID、関数名、変数名、テスト関数名を安定した接続点として使用します。

## 2. ID規則

| ID形式 | 用途 | 例 |
|---|---|---|
| `<分野>-EQ-<番号>` | 文書内の数式 | `PP-EQ-10` |
| `IMPL-<分野>-<番号>` | 数式を直接持たない実装処理 | `IMPL-SIM-01` |

数式は表示上の式 (1)、式 (2) を維持し、HTMLアンカーとして一意な理論IDを持ちます。例えば[`PP-EQ-10`](controllers/pure_pursuit.md#pp-eq-10)はPure Pursuit資料の式 (10)です。

## 3. 座標系と追従誤差

| 理論ID | 内容 | C++関数 | 主なC++変数 | 検証テスト |
|---|---|---|---|---|
| [`COORD-EQ-01`](fundamentals/coordinate_system.md#coord-eq-01) | 世界座標の位置差 | `calculateLongitudinalError()`、`calculateLateralError()` | `dx`, `dy` | `testInitialStateAndErrorConvention()` |
| [`COORD-EQ-04`](fundamentals/coordinate_system.md#coord-eq-04) | 縦偏差 | `calculateLongitudinalError()` | `dx`, `dy`, `reference.yaw` | 同上 |
| [`COORD-EQ-05`](fundamentals/coordinate_system.md#coord-eq-05) | 横偏差 | `calculateLateralError()` | `dx`, `dy`, `reference.yaw` | `testPositiveLateralErrorSteersRightInBothDirections()` |
| [`COORD-EQ-07`](fundamentals/coordinate_system.md#coord-eq-07) | 方位誤差の正規化 | `calculateYawError()`、`normalizeAngle()` | `reference.yaw`, `state.yaw` | `testYawErrorWrapsAcrossPiBoundary()` |

実装：`include/core/tracking_error.hpp`、`include/core/angle_utils.hpp`  
利用：`src/simulation/parking_simulator.cpp`と各フィードバック制御器

## 4. 経路・軌道生成

| 理論ID | 内容 | C++関数 | 主なC++変数 | 検証先 |
|---|---|---|---|---|
| [`PATH-EQ-04`](fundamentals/path_generation.md#path-eq-04) | 方位更新 | `PathGenerator::appendSegment()` | `next.yaw`, `motion_direction_`, `curvature`, `step` | `testArcEndpoint()` |
| [`PATH-EQ-05`](fundamentals/path_generation.md#path-eq-05) | 直線位置更新 | 同上 | `next.x`, `next.y`, `previous.yaw` | `testStraightPathInBothDirections()` |
| [`PATH-EQ-10`](fundamentals/path_generation.md#path-eq-10) | 円弧X更新 | 同上 | `next.x`, `next.yaw`, `curvature` | `testArcEndpoint()` |
| [`PATH-EQ-11`](fundamentals/path_generation.md#path-eq-11) | 円弧Y更新 | 同上 | `next.y`, `next.yaw`, `curvature` | 同上 |
| [`PATH-EQ-12`](fundamentals/path_generation.md#path-eq-12) | 半径と曲率 | `PathGenerator::addArc()` | `radius`, `turn_direction` | 同上 |
| [`TRAJ-EQ-09`](fundamentals/trajectory_generation.md#traj-eq-09) | 加速可能速度 | `createVelocityProfile()` | `acceleration_speed` | `testVelocityProfileSignsAndEndpoints()` |
| [`TRAJ-EQ-11`](fundamentals/trajectory_generation.md#traj-eq-11) | 減速可能速度 | 同上 | `deceleration_speed` | 同上 |
| [`TRAJ-EQ-12`](fundamentals/trajectory_generation.md#traj-eq-12) | 三つの速度上限 | 同上 | `speed` | 同上 |
| [`TRAJ-EQ-13`](fundamentals/trajectory_generation.md#traj-eq-13) | 前進・後退速度 | 同上 | `velocity_sign`, `velocity` | 前進・後退シナリオ |
| [`TRAJ-EQ-15`](fundamentals/trajectory_generation.md#traj-eq-15) | 区間時間 | 同上 | `distance`, `speed_sum`, `time` | 同上 |
| [`TRAJ-EQ-17`](fundamentals/trajectory_generation.md#traj-eq-17) | 区間加速度 | 同上 | `acceleration` | 同上 |

上表の経路・軌道生成は`tests/trajectory_generation_test.cpp`で直接検証します。補助実装の`TrajectorySampler::sampleByTime()`は時間補間、`ReferenceSearcher`は位置基準探索、`createForwardSampleTrajectory()`と`createReverseSampleTrajectory()`は学習用入力を構成します。

## 5. キネマティック自転車モデル

| 理論ID | 内容 | C++関数 | 主なC++変数 | 検証テスト |
|---|---|---|---|---|
| [`BICYCLE-EQ-03`](fundamentals/kinematic_bicycle_model.md#bicycle-eq-03) | 舵角と曲率 | 各制御器の`computeCommand()` | `wheelbase`, `curvature`, `steering` | 各制御器テスト |
| [`BICYCLE-EQ-06`](fundamentals/kinematic_bicycle_model.md#bicycle-eq-06) | ヨーレート | `KinematicBicycle::step()` | `yaw_rate`, `velocity`, `steering_angle` | `testSteeringDirection()` |
| [`BICYCLE-EQ-09`](fundamentals/kinematic_bicycle_model.md#bicycle-eq-09) | 方位更新 | 同上 | `next_yaw`, `dt` | `testSteeringDirection()` |
| [`BICYCLE-EQ-11`](fundamentals/kinematic_bicycle_model.md#bicycle-eq-11) | 旋回時X更新 | 同上 | `state_.x`, `velocity`, `yaw_rate` | 同上 |
| [`BICYCLE-EQ-12`](fundamentals/kinematic_bicycle_model.md#bicycle-eq-12) | 旋回時Y更新 | 同上 | `state_.y`, `velocity`, `yaw_rate` | 同上 |
| [`BICYCLE-EQ-13`](fundamentals/kinematic_bicycle_model.md#bicycle-eq-13) | 直進近似 | 同上 | `state_.x`, `state_.y`, `state_.yaw` | `testForwardStraight()`、`testReverseStraight()` |

車両諸元の正本は`include/core/vehicle_parameters.hpp`です。Python可視化と理論図も同じ値を読みます。

## 6. 操舵アクチュエータ

| 理論ID | 内容 | C++関数 | 主なC++変数 | 検証テスト |
|---|---|---|---|---|
| [`ACT-EQ-02`](fundamentals/steering_actuator.md#act-eq-02) | 最大舵角 | `SteeringActuator::update()` | `target`, `max_steering_angle` | `testAngleLimit()` |
| [`ACT-EQ-05`](fundamentals/steering_actuator.md#act-eq-05) | 一次遅れの離散応答 | 同上 | `response`, `time_constant`, `applied_steering_` | `testFirstOrderLag()` |
| [`ACT-EQ-06`](fundamentals/steering_actuator.md#act-eq-06) | 1周期最大変化 | 同上 | `max_change`, `max_steering_rate`, `dt` | `testRateLimitInBothDirections()` |
| [`ACT-EQ-08`](fundamentals/steering_actuator.md#act-eq-08) | 操舵速度制約 | 同上 | `change`, `limited_change` | 同上 |

## 7. 制御器

### 7.1 フィードフォワード・方位・横偏差

| 理論ID | 内容 | C++関数 | 主なC++変数 | 検証テスト |
|---|---|---|---|---|
| [`FF-EQ-03`](controllers/feedforward.md#ff-eq-03) | 曲率から舵角 | `FeedforwardController::computeCommand()` | `reference.curvature`, `wheelbase_` | `testZeroErrorUsesFeedforwardSteering()` |
| [`FF-EQ-04`](controllers/feedforward.md#ff-eq-04) | 参照速度の伝達 | 同上 | `reference.velocity` | シミュレーター試験 |
| [`YAW-EQ-01`](controllers/yaw_feedback.md#yaw-eq-01) | 方位誤差 | `YawFeedbackController::computeCommand()` | `yaw_error` | `testYawErrorWrapsAcrossPiBoundary()` |
| [`YAW-EQ-07`](controllers/yaw_feedback.md#yaw-eq-07) | 前後進方位補正 | 同上 | `motion_direction`, `yaw_error_gain_` | `testForwardAndReverseCorrectionDirections()` |
| [`LAT-EQ-03`](controllers/lateral_feedback.md#lat-eq-03) | 横偏差 | `LateralFeedbackController::computeCommand()` | `lateral_error` | `testPositiveLateralErrorSteersRightInBothDirections()` |
| [`LAT-EQ-04`](controllers/lateral_feedback.md#lat-eq-04) | 横偏差比例補正 | 同上 | `lateral_error_gain_` | 同上 |
| [`LAT-EQ-10`](controllers/lateral_feedback.md#lat-eq-10) | 最終舵角 | 同上 | `feedforward_steering`, `yaw_steering`, `lateral_steering` | `testClosedLoopReducesLateralOffset()` |

### 7.2 Pure Pursuit

| 理論ID | 内容 | C++関数 | 主なC++変数 | 検証テスト |
|---|---|---|---|---|
| [`PP-EQ-01`](controllers/pure_pursuit.md#pp-eq-01) | 最近傍距離 | `ReferenceSearcher::findNearestIndex()` | `squared_distance`, `nearest_index` | `testNearestIndex()` |
| [`PP-EQ-02`](controllers/pure_pursuit.md#pp-eq-02) | 注視点 | `ReferenceSearcher::findLookaheadIndex()` | `target_s`, `lookahead_distance` | `testLookaheadAndEndClamp()` |
| [`PP-EQ-06`](controllers/pure_pursuit.md#pp-eq-06) | 注視角 | `PurePursuitController::computeCommand()` | `motion_yaw`, `alpha` | `testPurePursuitCorrectsLeftOffsetInBothDirections()` |
| [`PP-EQ-10`](controllers/pure_pursuit.md#pp-eq-10) | 進行方向基準曲率 | 同上 | `alpha`, `target_distance` | 図幾何テスト |
| [`PP-EQ-11`](controllers/pure_pursuit.md#pp-eq-11) | 前後進符号変換 | 同上 | `motion_direction`, `curvature` | `testPurePursuitCorrectsLeftOffsetInBothDirections()` |
| [`PP-EQ-12`](controllers/pure_pursuit.md#pp-eq-12) | 曲率から舵角 | 同上 | `wheelbase_`, `steering` | 同上 |

### 7.3 Stanley

| 理論ID | 内容 | C++関数 | 主なC++変数 | 検証テスト |
|---|---|---|---|---|
| [`STANLEY-EQ-01`](controllers/stanley.md#stanley-eq-01) | 最近傍点 | `ReferenceSearcher::findNearestIndex()` | `nearest_index` | `testNearestIndex()` |
| [`STANLEY-EQ-03`](controllers/stanley.md#stanley-eq-03) | 方位誤差 | `StanleyController::computeCommand()` | `yaw_error` | `testStanleyCorrectsLeftOffsetInBothDirections()` |
| [`STANLEY-EQ-08`](controllers/stanley.md#stanley-eq-08) | 速度依存横偏差補正 | 同上 | `lateral_error`, `lateral_error_gain_`, `softening_velocity_` | 同上 |
| [`STANLEY-EQ-09`](controllers/stanley.md#stanley-eq-09) | 曲率フィードフォワード | 同上 | `nearest.curvature`, `feedforward_steering` | 同上 |
| [`STANLEY-EQ-10`](controllers/stanley.md#stanley-eq-10) | 最終舵角 | 同上 | `motion_direction`, `lateral_steering` | `testClosedLoopReductionInBothDirections()` |

### 7.4 線形誤差モデル・LQR

| 理論ID | 内容 | C++関数 | 主なC++変数 | 検証テスト |
|---|---|---|---|---|
| [`LINERR-EQ-18`](fundamentals/linear_error_model.md#linerr-eq-18) | 離散状態行列 | `createLinearErrorModel()` | `state_matrix` | `testLinearModelSigns()` |
| [`LINERR-EQ-19`](fundamentals/linear_error_model.md#linerr-eq-19) | 離散入力行列 | 同上 | `input_matrix` | 同上 |
| [`LQR-EQ-04`](controllers/lqr.md#lqr-eq-04) | Riccati反復 | `solveFixedSizeDiscreteLqr()` | `riccati_solution`, `next_solution` | `lqr_controller_test`全般 |
| [`LQR-EQ-05`](controllers/lqr.md#lqr-eq-05) | LQRゲイン | `calculateGain()` | `denominator`, `gain` | 同上 |
| [`LQR-EQ-06`](controllers/lqr.md#lqr-eq-06) | 状態フィードバック | `LqrController::computeCommand()` | `cached_gain_`, `error_state`, `correction` | `testLeftOffsetSteersRightInBothDirections()` |
| [`LQR-EQ-08`](controllers/lqr.md#lqr-eq-08) | 横偏差積分 | 同上 | `candidate_integral`, `integral_error_` | `testIntegralStateIsBounded()` |
| [`LQR-EQ-12`](controllers/lqr.md#lqr-eq-12) | 積分制限 | 同上 | `integral_limit`, `worsens_saturation` | `testIntegralReducesConstantSteeringBias()` |

### 7.5 縦制御・停止位置・縦横協調

| 理論ID | 内容 | C++関数 | 主な変数 | 検証テスト |
|---|---|---|---|---|
| [`LON-EQ-02`](controllers/longitudinal_velocity_control.md#lon-eq-02) | 速度P制御 | `VelocityStopController::computeCommand()` | `velocity_error`, `requested_acceleration` | `testVelocityFeedback()` |
| [`LON-EQ-04`](controllers/longitudinal_velocity_control.md#lon-eq-04) | 車速更新 | `KinematicBicycle::stepWithAcceleration()` | `next_velocity` | `testAccelerationUpdatesVehicleSpeedAndPosition()` |
| [`LONLIM-EQ-01`](controllers/longitudinal_constraints.md#lonlim-eq-01) | 加減速度制約 | `VelocityStopController::computeCommand()` | `limited`, `acceleration` | `testAccelerationAndJerkLimits()` |
| [`LONLIM-EQ-04`](controllers/longitudinal_constraints.md#lonlim-eq-04) | ジャーク制約後加速度 | 同上 | `maximum_change`, `previous_acceleration_` | 同上 |
| [`STOP-EQ-02`](controllers/stop_position_control.md#stop-eq-02) | 停止可能速度 | 同上 | `stopping_velocity_limit` | `testStopPositionControl()` |
| [`STOP-EQ-04`](controllers/stop_position_control.md#stop-eq-04) | 停止位置用目標速度 | 同上 | `target_speed`, `position_speed_limit` | 同上 |
| [`STOP-EQ-05`](controllers/stop_position_control.md#stop-eq-05) | 位置停止条件 | 同上 | `remaining_distance` | 駐車統合テスト |
| [`STOP-EQ-06`](controllers/stop_position_control.md#stop-eq-06) | 速度停止条件 | 同上 | `current_velocity` | 駐車統合テスト |
| [`COOP-EQ-05`](controllers/longitudinal_lateral_coordination.md#coop-eq-05) | 協調速度上限 | `LongitudinalLateralCoordinator::coordinate()` | `target_speed` | `testLongitudinalLateralCoordination()` |

実行の統合点は`runParkingScenario()`です。横制御器の舵角、操舵制約、協調速度、縦制御加速度、車両更新の順に処理します。

### 7.6 横方向線形MPC

| 理論ID | 内容 | C++関数 | 検証 |
|---|---|---|---|
| [`MPC-EQ-01`](controllers/linear_mpc.md#mpc-eq-01) | MPC状態 | `LinearMpcController::computeCommand()` | `testUnconstrainedMpcCorrectsLateralError()` |
| [`MPC-EQ-06`](controllers/linear_mpc.md#mpc-eq-06) | 入力予測行列 | 同上 | 同上 |
| [`MPC-EQ-09`](controllers/linear_mpc.md#mpc-eq-09) | QPの二次項 | 同上 | 同上 |
| [`MPC-EQ-11`](controllers/linear_mpc.md#mpc-eq-11) | 無制約解 | `solveConstrainedSequence()` | 同上 |
| [`MPC-EQ-13`](controllers/linear_mpc.md#mpc-eq-13) | 舵角制約 | `projectSteeringConstraints()` | `testSteeringConstraints()` |
| [`MPC-EQ-14`](controllers/linear_mpc.md#mpc-eq-14) | 操舵速度制約 | 同上 | 同上 |
| [`MPC-EQ-15`](controllers/linear_mpc.md#mpc-eq-15) | 先頭入力の適用 | `computeCommand()` | `testForwardAndReverseAreFinite()` |

詳細な導出は[横方向線形MPC](controllers/linear_mpc.md)を参照してください。

## 8. 数式を持たない実装フロー

| 実装ID | 実装 | 役割 | 関連資料 |
|---|---|---|---|
| `IMPL-APP-01` | `main()` | 引数解析、部品生成、実行、保存 | [処理フロー](architecture/processing_flow.md) |
| `IMPL-DATA-01` | `types.hpp`、`control_types.hpp`、`simulation_result.hpp` | モジュール間の共通データ契約 | [データフロー](architecture/data_flow.md) |
| `IMPL-CONTROLLER-IF-01` | `TrajectoryController` | 制御器を交換する共通インターフェース | [制御器追加ガイド](guides/controller_development.md) |
| `IMPL-VEHICLE-01` | `vehicle_parameters.hpp`、`vehicle_parameters.py` | C++・可視化・理論図で共有する車両諸元 | [自転車モデル](fundamentals/kinematic_bicycle_model.md) |
| `IMPL-SIM-01` | `runParkingSimulation()` | 参照取得→制御→アクチュエータ→記録→状態更新 | [処理フロー](architecture/processing_flow.md) |
| `IMPL-SAMPLER-01` | `TrajectorySampler::sampleByTime()` | 時刻参照点の補間 | [データフロー](architecture/data_flow.md) |
| `IMPL-SEARCH-01` | `ReferenceSearcher` | 最近傍点と注視点の探索 | [処理フロー](architecture/processing_flow.md) |
| `IMPL-SCENARIO-01` | `sample_trajectories.cpp` | 前進・後退の学習入力 | [実行方法](guides/running_simulation.md) |
| `IMPL-SCENARIO-FSM-01` | `ParkingStateMachine` | 前進・停止・後退・完了の状態遷移 | [状態管理](fundamentals/parking_state_management.md) |
| `IMPL-SCENARIO-SAMPLE-01` | `createForwardStopReverseScenario()` | 連続する前進・後退区間の配置 | [状態管理](fundamentals/parking_state_management.md) |
| `IMPL-PARKING-ENV-01` | `ParkingEnvironment`、`parking_environment.py` | 駐車枠・障害物・初期／切り返し／目標点の保存と描画 | [状態管理](fundamentals/parking_state_management.md) |
| `IMPL-CSV-01` | `writeSimulationCsvFiles()` | 参照軌道・車両ログ保存 | [結果の読み方](guides/reading_results.md) |
| `IMPL-PLOT-01` | `plot_result.py` | CSVの静止画可視化 | [結果の読み方](guides/reading_results.md) |
| `IMPL-ANIM-01` | `animate_trajectory.py` | 車両追従アニメーション | [実行方法](guides/running_simulation.md) |
| `IMPL-FIGURE-01` | `generate_doc_figures.py` | 理論図の再生成 | [記載規約](writing_conventions.md) |
| `IMPL-TRACE-01` | `check_traceability.py` | 式アンカーとコード参照の自動検査 | [記載規約](writing_conventions.md) |

## 9. ファイル別の入口

### 9.1 `include`

| ファイル群 | 役割 | 読む理論 |
|---|---|---|
| `include/core/types.hpp`, `control_types.hpp` | モジュール間データ | [データフロー](architecture/data_flow.md) |
| `include/core/angle_utils.hpp`, `tracking_error.hpp` | 角度・追従誤差 | `COORD-EQ-01`～`COORD-EQ-07` |
| `include/core/vehicle_parameters.hpp` | 共通車両諸元 | [自転車モデル](fundamentals/kinematic_bicycle_model.md) |
| `include/controllers/*.hpp` | 制御器共通APIと設定 | 各`docs/controllers`資料 |
| `include/models/*.hpp` | 実車相当・予測モデルAPI | `BICYCLE-*`, `LINERR-*` |
| `include/trajectory/*.hpp` | 経路・軌道API | `PATH-*`, `TRAJ-*` |
| `include/actuators/*.hpp` | 操舵応答API | `ACT-*` |
| `include/simulation/*.hpp` | 閉ループ設定・結果型 | `IMPL-SIM-01` |
| `include/io/*.hpp` | CSV出力API | `IMPL-CSV-01` |

### 9.2 `src`、`tests`、`python`

各`src`ファイルの先頭または計算ブロック直前に理論ID／実装IDを記載します。`tests`は検証対象ID、Python可視化は対応する実装IDを記載します。IDを検索すれば、文書と全実装箇所を横断できます。

```powershell
rg "PP-EQ-10|PP-EQ-11|PP-EQ-12" docs include src tests python
```

## 10. 自動検査

`python/check_traceability.py`は次を検査します。

1. 170式すべてに一意なアンカーがあること。
2. ソースコードが参照する理論IDが実在すること。
3. `include`、`src`、`tests`、`python`の全対象ファイルに理論IDまたは実装IDがあること。
4. 車両モデルと各制御器の主要式が実装から参照されていること。

```powershell
python python/check_traceability.py
```
