# ドキュメントガイド

## 1. この文書の目的

プロジェクト内の全Markdownを役割別に分類し、「最初に何を読むか」「各学習段階で何を読むか」「実装箇所をどう探すか」を示します。段階番号は学習順序であり、文書タイトルには含めません。

## 2. 初めて読むときの推奨順序

### 2.1 最初に全体像をつかむ

ここでは個別制御理論へ入らず、プロジェクトの目的・構造・処理・データを把握します。

1. [プロジェクト全体像](architecture/project_overview.md)
2. [ソースコードの処理フロー](architecture/processing_flow.md)
3. [データフロー](architecture/data_flow.md)
4. [フォルダ・ファイル構成](architecture/directory_structure.md)
5. [学習ロードマップ](learning_roadmap.md)

### 2.2 理論と実装の対応を確認する

1. [理論・数式・ソースコード トレーサビリティ](traceability.md)
2. [ドキュメント記載規約](writing_conventions.md)

## 3. 基礎理論

### 3.1 座標・経路・軌道・車両モデル

1. [座標系と追従誤差](fundamentals/coordinate_system.md)
2. [目標経路生成](fundamentals/path_generation.md)
3. [目標軌道生成](fundamentals/trajectory_generation.md)
4. [キネマティック自転車モデル](fundamentals/kinematic_bicycle_model.md)
5. [操舵制約と操舵一次遅れ](fundamentals/steering_actuator.md)
6. [線形軌道誤差モデル](fundamentals/linear_error_model.md)
7. [前進・停止・後退の状態管理](fundamentals/parking_state_management.md)

## 4. 制御器理論

### 4.1 横方向制御：段階1～9

1. [曲率フィードフォワード制御](controllers/feedforward.md)
2. [方位誤差フィードバック制御](controllers/yaw_feedback.md)
3. [横偏差フィードバック制御](controllers/lateral_feedback.md)
4. [Pure Pursuit制御](controllers/pure_pursuit.md)
5. [Stanley制御](controllers/stanley.md)
6. [LQR・積分器付きLQR制御](controllers/lqr.md)

### 4.2 縦方向制御と縦横協調：段階11～14

次の4文書は全体像ではなく、段階10まで理解した後に読む制御理論資料です。

1. [車速状態と速度フィードバック制御](controllers/longitudinal_velocity_control.md)
2. [加減速度・ジャーク制約](controllers/longitudinal_constraints.md)
3. [停止位置制御](controllers/stop_position_control.md)
4. [縦方向・横方向制御の協調](controllers/longitudinal_lateral_coordination.md)

### 4.3 モデル予測制御：段階15以降

1. [横方向線形MPC](controllers/linear_mpc.md)

## 5. 実行・評価・拡張ガイド

1. [ビルドと実行](guides/running_simulation.md)
2. [結果の読み方](guides/reading_results.md)
3. [制御器の追加方法](guides/controller_development.md)

## 6. 学習実験

1. [フィードフォワード制御の初期誤差比較](experiments/feedforward_initial_offset_comparison.md)
2. [方位誤差フィードバック比較実験](experiments/yaw_feedback_comparison.md)
3. [横偏差フィードバック比較実験](experiments/lateral_feedback_comparison.md)
4. [Pure Pursuit・Stanley比較実験](experiments/pure_pursuit_stanley_comparison.md)
5. [操舵制約・一次遅れ比較実験](experiments/steering_actuator_comparison.md)
6. [LQR・積分器付きLQR比較実験](experiments/lqr_comparison.md)

段階11～14は`--longitudinal-model`で比較でき、実行方法と出力先は[ビルドと実行](guides/running_simulation.md#4-段階1114の縦制御比較)を参照してください。今後、比較結果を固定条件で整理する場合は`docs/experiments`へ専用文書を追加します。

## 7. 補助・旧統合資料

- [直線・円弧軌道とキネマティック自転車モデル 理論解説](Parking_Trajectory_Simulation_Theory_Guide.md)：初期段階の統合資料です。最新の正本は各`fundamentals`・`controllers`文書です。

## 8. 数式と理論IDの読み方

表示上は式 (1)、式 (2)の連番を使い、その直前に`LON-EQ-01`のような一意な理論IDを併記します。C++計算行の`Traceability`、テスト関数の`Verifies`と同じIDを検索すると、理論から実装・検証へ移動できます。
