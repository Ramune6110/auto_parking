# 横偏差フィードバック比較実験

## 1. 実験の目的

同じ軌道・初期状態に対し、フィードフォワード、方位誤差フィードバック、横偏差フィードバックを比較します。横偏差制御器は段階3の方位補正も含みます。

## 2. 制御器の違い

| 制御器 | 曲率 | 方位誤差 | 横偏差 |
|---|---:|---:|---:|
| Feedforward | 使用 | 未使用 | 未使用 |
| Yaw feedback | 使用 | 使用 | 未使用 |
| Lateral feedback | 使用 | 使用 | 使用 |

## 3. 実験条件

| 項目 | 値 |
|---|---:|
| 方位誤差ゲイン | 1.0 |
| 横偏差ゲイン | 0.2 rad/m |
| 横偏差ケース | $(x,y,\mathrm{yaw})=(0.0,0.5,0^\circ)$ |
| 姿勢誤差ケース | $(x,y,\mathrm{yaw})=(0.3,0.5,5^\circ)$ |
| 制御周期 | 0.01 s |
| ホイールベース | 2.7 m |

## 4. 実行方法

```powershell
.\build\parking_trajectory_sim.exe `
    --controller lateral-feedback --forward-lateral-offset
.\build\parking_trajectory_sim.exe `
    --controller lateral-feedback --forward-pose-offset
.\build\parking_trajectory_sim.exe `
    --controller lateral-feedback --reverse-lateral-offset
.\build\parking_trajectory_sim.exe `
    --controller lateral-feedback --reverse-pose-offset
```

## 5. 横偏差だけを与えた結果

| 制御器 | 方向 | 初期位置誤差 [m] | 最終位置誤差 [m] | 最終横偏差 [m] |
|---|---|---:|---:|---:|
| Feedforward | 前進 | 0.500 | 0.495 | -0.007 |
| Yaw feedback | 前進 | 0.500 | 0.501 | +0.006 |
| Lateral feedback | 前進 | 0.500 | 0.112 | -0.057 |
| Feedforward | 後退 | 0.500 | 0.495 | -0.005 |
| Yaw feedback | 後退 | 0.500 | 0.501 | +0.003 |
| Lateral feedback | 後退 | 0.500 | 0.215 | -0.118 |

フィードフォワードと方位誤差制御では位置誤差が約0.5 m残ります。横偏差制御では前進・後退とも明確に減少しました。

曲線終端では横偏差と縦偏差の成分が入れ替わるため、最終横偏差だけでなく位置誤差と軌跡を合わせて評価します。

## 6. 複合姿勢誤差を与えた結果

| 制御器 | 方向 | 初期位置誤差 [m] | 最終位置誤差 [m] | 最終方位誤差 [deg] |
|---|---|---:|---:|---:|
| Feedforward | 前進 | 0.583 | 1.298 | -4.954 |
| Yaw feedback | 前進 | 0.583 | 0.773 | -0.039 |
| Lateral feedback | 前進 | 0.583 | 0.297 | -2.570 |
| Feedforward | 後退 | 0.583 | 0.133 | -5.072 |
| Yaw feedback | 後退 | 0.583 | 0.417 | +0.001 |
| Lateral feedback | 後退 | 0.583 | 0.126 | -2.054 |

横偏差制御は位置誤差を減らしますが、経路へ近づくために方位を傾けるので、停止時点では方位誤差が残ります。これは横位置と方位の二つの目的を有限時間内で調整しているためです。

## 7. 最大舵角

| シナリオ | 最大絶対舵角 [deg] |
|---|---:|
| forward lateral offset | 35.263 |
| forward pose offset | 36.515 |
| reverse lateral offset | 35.644 |
| reverse pose offset | 35.649 |

段階6までは舵角制約を導入しないため、制御器出力をそのまま車両モデルへ入力しています。

## 8. 結果の解釈

- 方位誤差制御だけでは、方位が揃っても平行な位置ずれは残る。
- 横偏差項を追加すると、前進・後退とも目標経路へ近づく。
- 終端停止までの時間が有限なため、誤差が完全にゼロになる前に補正が止まる。
- 横偏差ゲインを大きくすれば速く近づくとは限らず、振動や過大操舵とのトレードオフがある。
- 後退でも横偏差項自体の符号は反転しない。

## 9. 出力結果

```text
output/lateral_feedback/
├── forward/
├── reverse/
├── forward_lateral_offset/
├── forward_pose_offset/
├── reverse_lateral_offset/
└── reverse_pose_offset/
```

各フォルダにCSV、軌跡PNG、時系列PNG、GIFを保存しています。

## 10. 次の段階

この実験の次段階としてPure PursuitとStanleyを追加しました。時刻参照だけでなく、車両位置に対する最近傍点や注視点を使用した比較は[Pure Pursuit・Stanley比較実験](pure_pursuit_stanley_comparison.md)を参照してください。
