# フィードフォワード制御の初期誤差比較

## 1. 実験の目的

学習ロードマップの段階2として、既存のforward・reverse軌道へ初期誤差を与え、曲率フィードフォワード制御が誤差を補正しないことを確認します。

## 2. 比較する6ケース

| ケース | 軌道 | 初期状態 $(x,y,\mathrm{yaw})$ | 目的 |
|---|---|---|---|
| `forward` | 前進 | 軌道始点と一致 | 前進の理想基準 |
| `reverse` | 後退 | 軌道始点と一致 | 後退の理想基準 |
| `forward_lateral_offset` | 前進 | $(0.0,0.5,0.0)$ | 横偏差だけの影響 |
| `forward_pose_offset` | 前進 | $(0.3,0.5,5^\circ)$ | 縦・横・方位誤差の複合影響 |
| `reverse_lateral_offset` | 後退 | $(0.0,0.5,0.0)$ | 後退時の横偏差の影響 |
| `reverse_pose_offset` | 後退 | $(0.3,0.5,5^\circ)$ | 後退時の複合初期誤差 |

forwardは「5 m直線、半径4 mの左90度円弧、3 m直線」、reverseは「3 m後退直線、半径4 mの後退円弧」です。

## 3. 誤差の意味

軌道始点の方位は0 radなので、初期状態の各変更は次を意味します。

| 初期状態の変更 | 発生する誤差 |
|---|---|
| $x=+0.3$ m | 縦偏差 $+0.3$ m |
| $y=+0.5$ m | 横偏差 $+0.5$ m |
| $\mathrm{yaw}=+5^\circ$ | 方位誤差は「参照－車両」のため $-5^\circ$ |

横偏差ケースは原因を一つに限定した実験、姿勢誤差ケースはフィードフォワード制御の総合的な限界を見る実験です。

## 4. 実行方法

```powershell
.\build\parking_trajectory_sim.exe
.\build\parking_trajectory_sim.exe --reverse
.\build\parking_trajectory_sim.exe --forward-lateral-offset
.\build\parking_trajectory_sim.exe --forward-pose-offset
.\build\parking_trajectory_sim.exe --reverse-lateral-offset
.\build\parking_trajectory_sim.exe --reverse-pose-offset
```

各ケースの出力フォルダを `--output-dir` へ指定して、グラフとアニメーションを生成します。

## 5. 数値結果

| ケース | 初期位置誤差 [m] | 最終位置誤差 [m] | 最大位置誤差 [m] | 初期方位誤差 [deg] | 最終方位誤差 [deg] |
|---|---:|---:|---:|---:|---:|
| `forward` | 0.000 | 0.009 | 0.012 | 0.000 | 0.046 |
| `reverse` | 0.000 | 0.007 | 0.010 | 0.000 | -0.072 |
| `forward_lateral_offset` | 0.500 | 0.495 | 0.500 | 0.000 | 0.046 |
| `forward_pose_offset` | 0.583 | 1.298 | 1.302 | -5.000 | -4.954 |
| `reverse_lateral_offset` | 0.500 | 0.495 | 0.500 | 0.000 | -0.072 |
| `reverse_pose_offset` | 0.583 | 0.133 | 0.586 | -5.000 | -5.072 |

## 6. 横偏差ケースの解釈

forward・reverseとも、位置誤差は開始時0.5 m、最大でも約0.500029 mであり、ほぼ一定です。車両は目標と同じ速度・曲率で動くため、世界座標上の初期位置ずれを保持したまま走行します。

曲線では参照座標系が回転するため、一定の世界座標位置ずれが横偏差と縦偏差へ分配されます。そのため、最終横偏差だけを見て「補正された」と判断してはいけません。軌跡図と位置誤差を併せて確認します。

## 7. 姿勢誤差ケースの解釈

初期方位誤差はforwardで最終約$-4.954^\circ$、reverseで最終約$-5.072^\circ$です。開始時の$-5^\circ$がほぼ残っており、方位誤差を補正していません。

reverse姿勢誤差ケースでは最終位置誤差が偶然小さくなりますが、これはフィードバックによる収束ではありません。方位誤差が残っており、軌跡の幾何的な交差・接近によって終端で距離が小さく見えているだけです。誤差補正の評価には、終端値だけでなく全時系列を使用します。

## 8. 確認できたこと

- 理想初期状態では、モデルと制御器が一致するため誤差は小さい。
- 横偏差だけを与えると、位置誤差は約0.5 mのまま残る。
- 方位誤差を与えると、約5度の方位誤差が残る。
- フィードフォワード制御は現在状態を使わないため、初期誤差をゼロへ収束させる働きを持たない。
- 曲線では横・縦偏差の成分が変化するため、位置誤差と軌跡も併せて評価する必要がある。

## 9. ソースコードとの対応

| 内容 | ファイル |
|---|---|
| シナリオ選択、初期状態、出力先 | `src/main.cpp` |
| forward・reverse目標軌道 | `src/trajectory/sample_trajectories.cpp` |
| フィードフォワード舵角 | `src/controllers/feedforward_controller.cpp` |
| 誤差計算 | `src/simulation/parking_simulator.cpp` |
| 補正されないことの自動テスト | `tests/parking_simulator_test.cpp` |

## 10. 出力構成

```text
output/feedforward/
├── forward/
├── reverse/
├── forward_lateral_offset/
├── forward_pose_offset/
├── reverse_lateral_offset/
└── reverse_pose_offset/
```

各フォルダには `reference_trajectory.csv`、`vehicle_log.csv`、2種類のPNG、GIFを保存します。

## 11. 次の学習段階

この実験の次段階として方位誤差フィードバックを実装し、同じpose offset条件で比較しました。結果は[方位誤差フィードバック比較実験](yaw_feedback_comparison.md)を参照してください。
