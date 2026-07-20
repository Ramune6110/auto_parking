# 自動駐車・軌道追従制御シミュレーター

完成済みの目標軌道を入力として、参照点取得、舵角計算、車両運動更新を繰り返す制御ロジックの学習用 C++17 プロジェクトです。現在は曲率フィードフォワード、方位誤差、横偏差、Pure Pursuit、Stanley、LQR、積分器付きLQRと、非線形キネマティック自転車モデルを実装しています。

理論と学習順序は [ドキュメント入口](docs/index.md) から読んでください。

## 構成

```text
include/
├── core/          共通データ型
├── controllers/   制御器インターフェースと各制御器
├── control/       LQRなどで共用する数値計算
├── trajectory/    経路生成、速度設計、軌道参照
├── models/        車両モデル
├── simulation/    制御周期と計算結果
└── io/            CSV 保存
src/
├── control/
├── controllers/
├── trajectory/
├── models/
├── simulation/
├── io/
└── main.cpp       部品の組み立てと実行条件の選択
docs/
├── index.md
├── learning_roadmap.md
├── architecture/
├── fundamentals/
├── controllers/
├── guides/
└── figures/
output/
└── feedforward/
    ├── forward/
    └── reverse/
```

目標経路は位置・方位・曲率という空間情報、目標軌道はそれに時刻・速度・加速度を加えた情報です。両者の生成処理は別モジュールです。

## ビルドと実行

LQRの行列演算にはEigen 3.4.1を使用します。CMakeはインストール済みのEigen 3.4以上を最初に検索し、見つからない場合は初回構成時に公式GitLabから3.4.1を取得します。その場合、初回の`cmake -S`だけネットワーク接続が必要です。

```powershell
cmake -S . -B build -G "MinGW Makefiles" -DCMAKE_BUILD_TYPE=Debug
cmake --build build
ctest --test-dir build --output-on-failure
.\build\parking_trajectory_sim.exe
.\build\parking_trajectory_sim.exe --reverse
.\build\parking_trajectory_sim.exe --forward-lateral-offset
.\build\parking_trajectory_sim.exe --forward-pose-offset
.\build\parking_trajectory_sim.exe --reverse-lateral-offset
.\build\parking_trajectory_sim.exe --reverse-pose-offset
```

## 可視化

```powershell
python .\python\plot_result.py
python .\python\animate_trajectory.py
```

後退結果には `--output-dir .\output\feedforward\reverse` を追加します。詳しくは [実行方法](docs/guides/running_simulation.md) を参照してください。

初期誤差を与えた4ケースは、`output/feedforward` 配下の対応するシナリオ名へ保存されます。条件と比較結果は[実験資料](docs/experiments/feedforward_initial_offset_comparison.md)を参照してください。

方位誤差フィードバック制御を選択する例です。

```powershell
.\build\parking_trajectory_sim.exe --controller yaw-feedback --forward-pose-offset
.\build\parking_trajectory_sim.exe --controller yaw-feedback --reverse-pose-offset
```

結果は `output/yaw_feedback` 配下に保存されます。理論は[方位誤差フィードバック制御](docs/controllers/yaw_feedback.md)、比較結果は[実験資料](docs/experiments/yaw_feedback_comparison.md)を参照してください。

横偏差フィードバック制御を選択する例です。

```powershell
.\build\parking_trajectory_sim.exe --controller lateral-feedback --forward-lateral-offset
.\build\parking_trajectory_sim.exe --controller lateral-feedback --reverse-lateral-offset
```

結果は `output/lateral_feedback` 配下に保存されます。理論は[横偏差フィードバック制御](docs/controllers/lateral_feedback.md)、比較結果は[実験資料](docs/experiments/lateral_feedback_comparison.md)を参照してください。

Pure PursuitとStanleyを比較する例です。

```powershell
.\build\parking_trajectory_sim.exe --controller pure-pursuit --forward-pose-offset
.\build\parking_trajectory_sim.exe --controller stanley --forward-pose-offset
```

理論は[Pure Pursuit](docs/controllers/pure_pursuit.md)と[Stanley](docs/controllers/stanley.md)、結果は[比較実験](docs/experiments/pure_pursuit_stanley_comparison.md)を参照してください。

操舵角・操舵速度制約と一次遅れを含む実行例です。

```powershell
.\build\parking_trajectory_sim.exe --controller pure-pursuit --forward-pose-offset --steering-model actuator
```

理論と比較条件は[操舵制約と操舵一次遅れ](docs/fundamentals/steering_actuator.md)を参照してください。

LQRと積分器付きLQRの実行例です。

```powershell
.\build\parking_trajectory_sim.exe --controller lqr --forward-pose-offset --steering-model actuator
.\build\parking_trajectory_sim.exe --controller lqr-integral --forward-pose-offset --steering-model actuator
```

理論は[線形軌道誤差モデル](docs/fundamentals/linear_error_model.md)と[LQR制御](docs/controllers/lqr.md)、結果は[LQR比較実験](docs/experiments/lqr_comparison.md)を参照してください。

## 制御器を拡張するとき

前進・停止・後退を連続実行する例です。

```powershell
.\build\parking_trajectory_sim.exe --controller pure-pursuit --forward-stop-reverse --steering-model actuator
python .\python\plot_result.py --output-dir .\output\pure_pursuit\forward_stop_reverse\actuator
```

新しい実装は `include/controllers` と `src/controllers` に追加し、軌道・車両モデル・評価処理は共有します。プロジェクトを制御器ごとに複製しないため、同一条件で結果を比較できます。結果だけを `output/<controller>/<scenario>` に分離します。
