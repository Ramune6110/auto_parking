# ビルドと実行

## 1. C++のビルドとテスト

```powershell
cmake -S . -B build -G "MinGW Makefiles" -DCMAKE_BUILD_TYPE=Debug
cmake --build build
ctest --test-dir build --output-on-failure
```

前進基準ケースと後退ケースを実行します。

```powershell
.\build\parking_trajectory_sim.exe
.\build\parking_trajectory_sim.exe --reverse
```

結果はそれぞれ `output/feedforward/forward` と `output/feedforward/reverse` に保存されます。

段階2の初期誤差実験は、次のコマンドで実行します。

```powershell
.\build\parking_trajectory_sim.exe --forward-lateral-offset
.\build\parking_trajectory_sim.exe --forward-pose-offset
.\build\parking_trajectory_sim.exe --reverse-lateral-offset
.\build\parking_trajectory_sim.exe --reverse-pose-offset
```

結果は `output/feedforward` 配下の各シナリオ名に保存されます。

方位誤差フィードバック制御は、各コマンドへ `--controller yaw-feedback` を追加します。

```powershell
.\build\parking_trajectory_sim.exe `
    --controller yaw-feedback --forward-pose-offset
.\build\parking_trajectory_sim.exe `
    --controller yaw-feedback --reverse-pose-offset
```

結果は `output/yaw_feedback` 配下の各シナリオ名に保存されます。

横偏差フィードバック制御は `--controller lateral-feedback` を指定します。

```powershell
.\build\parking_trajectory_sim.exe `
    --controller lateral-feedback --forward-lateral-offset
.\build\parking_trajectory_sim.exe `
    --controller lateral-feedback --reverse-lateral-offset
```

結果は `output/lateral_feedback` 配下の各シナリオ名に保存されます。

Pure PursuitとStanleyは次のように選択します。

```powershell
.\build\parking_trajectory_sim.exe `
    --controller pure-pursuit --forward-pose-offset
.\build\parking_trajectory_sim.exe `
    --controller stanley --forward-pose-offset
```

結果はそれぞれ`output/pure_pursuit`、`output/stanley`配下に保存されます。

### 1.1 操舵モデルを比較する

`--steering-model`で操舵制約と一次遅れを選択できます。

```powershell
.\build\parking_trajectory_sim.exe `
    --controller pure-pursuit --forward-pose-offset `
    --steering-model ideal
.\build\parking_trajectory_sim.exe `
    --controller pure-pursuit --forward-pose-offset `
    --steering-model angle-limit
.\build\parking_trajectory_sim.exe `
    --controller pure-pursuit --forward-pose-offset `
    --steering-model angle-rate-limit
.\build\parking_trajectory_sim.exe `
    --controller pure-pursuit --forward-pose-offset `
    --steering-model actuator
```

操舵モデルを明示した結果は`output/<controller>/<scenario>/<steering_model>`へ保存されます。指定しない場合は従来どおり理想操舵となり、従来の出力先を維持します。

### 1.2 LQRを実行する

```powershell
.\build\parking_trajectory_sim.exe `
    --controller lqr --forward-pose-offset --steering-model actuator
.\build\parking_trajectory_sim.exe `
    --controller lqr-integral --forward-pose-offset --steering-model actuator
```

一定操舵オフセットを評価するときは`--steering-bias-deg 1`を追加します。結果フォルダ名には`_bias`が付きます。

## 2. グラフとアニメーション

```powershell
python .\python\plot_result.py
python .\python\animate_trajectory.py
python .\python\plot_result.py --output-dir .\output\feedforward\reverse
python .\python\animate_trajectory.py --output-dir .\output\feedforward\reverse
```

Python スクリプトの既定入力は前進ケースです。

## 3. 前進・停止・後退の連続実行

```powershell
.\build\parking_trajectory_sim.exe --controller pure-pursuit `
    --forward-stop-reverse --steering-model actuator
python .\python\plot_result.py `
    --output-dir .\output\pure_pursuit\forward_stop_reverse\actuator
```

前進、ギア切り替え停止、後退、最終停止を一つのCSVへ保存します。

状態遷移だけを確認する従来の簡易軌道は、`--forward-stop-reverse-simple`で実行できます。通常の学習・比較には駐車枠へ進入する`--forward-stop-reverse`を使用してください。

## 4. 段階11～14の縦制御比較

`--longitudinal-model`だけを変え、同じ軌道・横制御器で比較します。

```powershell
.\build\parking_trajectory_sim.exe --controller pure-pursuit --forward-stop-reverse --longitudinal-model velocity-feedback
.\build\parking_trajectory_sim.exe --controller pure-pursuit --forward-stop-reverse --longitudinal-model acceleration-limit
.\build\parking_trajectory_sim.exe --controller pure-pursuit --forward-stop-reverse --longitudinal-model jerk-limit
.\build\parking_trajectory_sim.exe --controller pure-pursuit --forward-stop-reverse --longitudinal-model stop-position
.\build\parking_trajectory_sim.exe --controller pure-pursuit --forward-stop-reverse --longitudinal-model coordinated
```

`coordinated`の描画例です。

```powershell
python .\python\plot_result.py --output-dir .\output\pure_pursuit\forward_stop_reverse\coordinated
```

## 5. 段階15の横方向線形MPC

```powershell
.\build\parking_trajectory_sim.exe --controller linear-mpc `
    --forward-stop-reverse --steering-model actuator `
    --longitudinal-model coordinated

python .\python\plot_result.py `
    --output-dir .\output\linear_mpc\forward_stop_reverse\actuator\coordinated
```

横方向MPCは補正舵角を最適化し、加速度は段階14の縦制御器が担当します。
