# フォルダ・ファイル構成

## 1. この文書の目的

現在のプロジェクトツリーと各ファイルの責務を対応付けます。ヘッダーは外部から利用する型・クラス・関数の宣言、同名のCPPは具体的な計算処理を担当します。

## 2. 全体構成

```text
parking_trajectory_sim/
├── CMakeLists.txt
├── README.md
├── include/
│   ├── core/
│   ├── control/
│   ├── controllers/
│   ├── actuators/
│   ├── trajectory/
│   ├── models/
│   ├── simulation/
│   ├── scenario/
│   ├── coordination/
│   └── io/
├── src/
│   ├── main.cpp
│   ├── control/
│   ├── controllers/
│   ├── actuators/
│   ├── trajectory/
│   ├── models/
│   ├── simulation/
│   ├── scenario/
│   ├── coordination/
│   └── io/
├── tests/
├── python/
├── docs/
└── output/
    ├── feedforward/
        ├── forward/
        ├── reverse/
        ├── forward_lateral_offset/
        ├── forward_pose_offset/
        ├── reverse_lateral_offset/
    │   └── reverse_pose_offset/
    ├── pure_pursuit/
    │   └── （同じ6シナリオ）
    └── stanley/
        └── （同じ6シナリオ）
```

## 3. ルートファイル

| ファイル | 役割 |
|---|---|
| `CMakeLists.txt` | C++17、実行ファイル、テスト、警告オプションを定義する |
| `README.md` | プロジェクトの短い紹介、構成、基本コマンドを示す |
| `docs/traceability.md` | 理論式、図、C++関数、変数、テストの対応を一元管理する |

## 4. `include/core`：共通データ型

| ファイル | 定義内容 | 主な利用先 |
|---|---|---|
| `types.hpp` | `GeometricPathPoint`、`TrajectoryPoint`、`VehicleState`、配列型 | 軌道、制御器、車両モデル、シミュレーション |
| `control_types.hpp` | `ControlCommand`、共通入力 `ControllerInput` | 制御器、シミュレーター、ログ |
| `longitudinal_types.hpp` | 縦制御・縦横協調の入力、出力、制約状態 | 縦制御器、協調器、シミュレーター、ログ |
| `vehicle_parameters.hpp` | ホイールベース、車幅、前後オーバーハング、COG、最大舵角 | C++シミュレーション、Python可視化、理論図 |

このフォルダは、複数モジュールが共有する軽量なデータ構造だけを持ちます。具体的な計算処理は置きません。

## 5. `control`：共通制御数値計算

| ファイル | 役割 |
|---|---|
| `include/control/discrete_lqr.hpp` | 離散LQRソルバーの設定、結果、2状態・3状態インターフェースを宣言 |
| `src/control/discrete_lqr.cpp` | Eigen固定サイズ行列でRiccati反復とゲイン計算を実行 |

制御器固有の誤差管理から数値解法を分離し、数式と行列演算の対応を読みやすくしています。

## 6. `controllers`：制御器

| ファイル | 役割 |
|---|---|
| `include/controllers/trajectory_controller.hpp` | 全制御器が従う `computeCommand()` インターフェースを定義 |
| `include/controllers/feedforward_controller.hpp` | フィードフォワード制御器を宣言 |
| `src/controllers/feedforward_controller.cpp` | 参照曲率から目標舵角を計算し、参照速度を指令する |

新しい制御方式は原則としてこのフォルダへ追加します。シミュレーターは具体的な制御器名ではなく `TrajectoryController` を利用するため、制御器を交換できます。

### 6.1 現在実装されている横方向制御器

| 制御器 | 宣言 | 実装 |
|---|---|---|
| 曲率フィードフォワード | `feedforward_controller.hpp` | `feedforward_controller.cpp` |
| 方位誤差フィードバック | `yaw_feedback_controller.hpp` | `yaw_feedback_controller.cpp` |
| 横偏差フィードバック | `lateral_feedback_controller.hpp` | `lateral_feedback_controller.cpp` |
| Pure Pursuit | `pure_pursuit_controller.hpp` | `pure_pursuit_controller.cpp` |
| Stanley | `stanley_controller.hpp` | `stanley_controller.cpp` |
| LQR・積分器付きLQR | `lqr_controller.hpp` | `lqr_controller.cpp` |
| 横方向線形MPC | `linear_mpc_controller.hpp` | `linear_mpc_controller.cpp` |

方位誤差フィードバックは進行方向を考慮して方位誤差を補正します。横偏差フィードバックはその方位補正を含み、さらに左右位置ずれを補正します。

### 6.2 `controllers/longitudinal`：縦方向制御器

| ファイル | 役割 |
|---|---|
| `velocity_stop_controller.hpp/.cpp` | 速度P制御、停止可能速度、加減速度・ジャーク制約、停止判定 |

横制御器が要求舵角を返すのに対し、縦制御器は実車速と残距離から適用加速度を返します。

### 6.3 `coordination`：縦横協調

| ファイル | 役割 |
|---|---|
| `longitudinal_lateral_coordinator.hpp/.cpp` | 曲率・横偏差・方位誤差・操舵制約を速度上限へ変換 |

協調層は舵角を変更せず、縦制御へ渡す目標速度だけを調整します。

## 7. `actuators`：操舵アクチュエータ

| ファイル | 役割 |
|---|---|
| `include/actuators/steering_actuator.hpp` | 最大舵角、最大操舵速度、一次遅れの設定・入出力を宣言 |
| `src/actuators/steering_actuator.cpp` | 要求舵角から車両へ適用する実舵角を計算 |

制御則と車両モデルの間に配置し、全制御器へ同じ物理制約を適用します。

## 8. `trajectory`：経路と軌道

| ファイル | 役割 |
|---|---|
| `path_generator.hpp/.cpp` | 直線と一定曲率円弧から幾何学的な経路点を生成 |
| `velocity_profile.hpp/.cpp` | 経路へ速度、加速度、時刻を付与して軌道へ変換 |
| `sample_trajectories.hpp/.cpp` | 前進・後退の学習用目標軌道を組み立てる |
| `trajectory_sampler.hpp/.cpp` | 軌道を時刻で検索し、隣接点間を補間して参照点を返す |
| `reference_searcher.hpp/.cpp` | 車両位置の最近傍点と指定距離先の注視点を探索する |

形状を作る処理、速度を計画する処理、完成済み軌道を参照する処理を分離しています。

## 9. `models`：車両モデル

| ファイル | 役割 |
|---|---|
| `include/models/kinematic_bicycle.hpp` | 車両モデルの生成、状態更新、状態取得を宣言 |
| `src/models/kinematic_bicycle.cpp` | 速度と舵角から車両位置・方位を解析的に更新 |
| `linear_error_model.hpp/.cpp` | Eigenの`Matrix2d`と`Vector2d`でLQR用線形離散モデルを生成 |

この車両モデルはシミュレーション対象です。将来MPCを追加する場合、制御器内部の予測モデルとは役割を分けます。

## 10. `simulation`：制御周期と結果

| ファイル | 役割 |
|---|---|
| `simulation_result.hpp` | 1周期のログ `SimulationSample` と全結果 `SimulationResult` を定義 |
| `parking_simulator.hpp` | シミュレーション条件と `runParkingSimulation()` を宣言 |
| `parking_simulator.cpp` | 参照点取得、制御計算、誤差記録、車両更新を繰り返す |

制御学習の中核は `parking_simulator.cpp` です。ただし、制御則そのものは `controllers` に委譲します。

## 11. `io`：ファイル出力

| ファイル | 役割 |
|---|---|
| `include/io/csv_writer.hpp` | CSV保存関数を宣言 |
| `src/io/csv_writer.cpp` | 出力ディレクトリを作成し、参照軌道と車両ログを保存 |

CSV保存を制御周期から分離することで、シミュレーションロジックがファイル形式へ依存しないようにしています。

## 12. `src/main.cpp`：組み立て

`scenario`は、走行区間の軌道を生成する`sample_parking_scenario`と、前進・停止・後退の遷移を管理する`ParkingStateMachine`を持ちます。制御則は置かず、現在有効な軌道区間だけをシミュレーターへ選択します。

`controllers/longitudinal`は速度・停止位置から加速度を計算し、`coordination`は横追従状態から縦制御へ渡す速度上限を調整します。Pure PursuitやLQRなど既存の横制御器は変更せず再利用します。

`main.cpp` は次の選択と組み立てだけを担当します。

- コマンドラインから前進・後退を選ぶ
- 入力軌道を生成する
- 使用する制御器を生成する
- シミュレーション条件を設定する
- シミュレーションを呼び出す
- 完了後にCSV保存を呼び出す

経路計算、制御則、車両運動、CSVの具体的な処理は `main.cpp` に書きません。

## 13. `tests`：自動テスト

`trajectory_generation_test.cpp`は、前進・後退の直線経路、一定曲率円弧の終端、速度プロファイルの符号・停止点・時刻を検証します。各テスト関数の`Verifies`コメントから対応する理論式へ戻れます。

| ファイル | 検証内容 |
|---|---|
| `kinematic_bicycle_test.cpp` | 前進・後退、直進・旋回、極小旋回時の状態更新 |
| `parking_simulator_test.cpp` | 初期状態設定と追従誤差の符号 |
| `reference_searcher_test.cpp` | 最近傍点、単調な探索開始位置、注視点の選択 |
| `path_tracking_controllers_test.cpp` | Pure Pursuit・Stanleyの前進／後退補正と閉ループ収束 |
| `lqr_controller_test.cpp` | 線形モデル符号、前後進LQR、積分上限、一定外乱補償 |

## 14. `python`：結果の可視化

| ファイル | 役割 |
|---|---|
| `plot_result.py` | 軌跡を個別図、速度・加速度・曲率・舵角・誤差を縦型グラフへ出力 |
| `animate_trajectory.py` | 目標軌道に対する車両追従をGIF化し、前輪舵角と車両座標系を描画 |
| `vehicle_parameters.py` | C++の共通車両諸元を読み、Python側へ提供 |
| `doc_figures/generate_doc_figures.py` | 数式と共通車両諸元から理論図をPNG・SVGで生成 |
| `doc_figures/geometry.py` | Pure Pursuit、誤差射影、自転車モデルの検証可能な幾何計算 |
| `doc_figures/test_doc_figures.py` | 幾何条件と全理論図の生成を自動検証 |

Pythonは計算済みCSVだけを読み、C++の制御計算には関与しません。

## 15. `docs` と `output`

| フォルダ | 内容 |
|---|---|
| `docs/architecture` | プロジェクト構造、処理、データの全体像 |
| `docs/fundamentals` | 座標、経路、軌道、車両モデルの理論 |
| `docs/controllers` | 制御器ごとの理論 |
| `docs/guides` | 実行、結果確認、制御器追加の手順 |
| `docs/figures` | Pythonから再生成される理論図のPNG・SVG |
| `output/<controller>/<scenario>` | 制御器・シナリオ別のCSV、PNG、GIF |

## 16. 目的別の変更先

| やりたいこと | 主に変更するファイル |
|---|---|
| 新しい制御器を追加する | `include/controllers`、`src/controllers`、`CMakeLists.txt` |
| サンプル経路の長さや形を変える | `src/trajectory/sample_trajectories.cpp` |
| 経路生成アルゴリズムを変える | `path_generator.hpp/.cpp` |
| 速度計画を変える | `velocity_profile.hpp/.cpp` |
| 車両運動を変える | `models/kinematic_bicycle.hpp/.cpp` |
| 制御周期や誤差定義を変える | `simulation/parking_simulator.hpp/.cpp` |
| ログ項目を増やす | `simulation_result.hpp`、`parking_simulator.cpp`、`csv_writer.cpp` |
| グラフを変える | `python/plot_result.py` |
| 車両アニメーションを変える | `python/animate_trajectory.py` |
| 理論図を変える | `python/doc_figures/generate_doc_figures.py` |
| 共通車両諸元を変える | `include/core/vehicle_parameters.hpp` |
