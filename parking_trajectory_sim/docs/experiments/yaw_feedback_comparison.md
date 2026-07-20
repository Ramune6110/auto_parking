# 方位誤差フィードバック比較実験

## 1. 実験の目的

段階2と同じ初期姿勢誤差を使い、フィードフォワード制御と方位誤差フィードバック制御を比較します。比較条件を揃えるため、軌道、初期状態、車両モデル、周期は変更しません。

## 2. 実験条件

| 項目 | 設定値 |
|---|---:|
| 初期縦偏差 | +0.3 m |
| 初期横偏差 | +0.5 m |
| 初期車両方位オフセット | +5 deg |
| 初期方位誤差 | -5 deg |
| 方位誤差ゲイン | 1.0 |
| 制御周期 | 0.01 s |
| ホイールベース | 2.7 m |

## 3. 実行方法

```powershell
.\build\parking_trajectory_sim.exe `
    --controller yaw-feedback --forward-pose-offset

.\build\parking_trajectory_sim.exe `
    --controller yaw-feedback --reverse-pose-offset
```

## 4. 方位誤差の比較

| 制御器 | シナリオ | 初期方位誤差 [deg] | 最終方位誤差 [deg] | $|e_\psi|<0.5$ degの整定時刻 [s] |
|---|---|---:|---:|---:|
| Feedforward | forward pose | -5.000 | -4.954 | 収束せず |
| Yaw feedback | forward pose | -5.000 | -0.039 | 6.46 |
| Feedforward | reverse pose | -5.000 | -5.072 | 収束せず |
| Yaw feedback | reverse pose | -5.000 | +0.001 | 6.34 |

前進・後退とも、約5度あった方位誤差がほぼゼロまで減少しました。後退時に進行方向符号を掛けているため、補正方向も正しくなっています。

## 5. 位置誤差の比較

| 制御器 | シナリオ | 最終位置誤差 [m] | 最大位置誤差 [m] |
|---|---|---:|---:|
| Feedforward | forward pose | 1.298 | 1.302 |
| Yaw feedback | forward pose | 0.773 | 0.777 |
| Feedforward | reverse pose | 0.133 | 0.586 |
| Yaw feedback | reverse pose | 0.417 | 0.586 |

方位補正によって軌跡は変化しますが、位置誤差はゼロへ収束していません。特にreverseでは、フィードフォワード軌跡が終端付近で偶然目標へ接近していたため、方位を正しくした結果の方が最終距離だけを見ると大きくなります。

これは不具合ではなく、段階3が方位誤差だけを制御しているためです。位置誤差の改善は段階4の横偏差フィードバックで扱います。

## 6. 操舵角

| 制御器 | シナリオ | 最大絶対舵角 [deg] |
|---|---|---:|
| Feedforward | forward pose | 34.019 |
| Yaw feedback | forward pose | 33.998 |
| Feedforward | reverse pose | 34.019 |
| Yaw feedback | reverse pose | 35.801 |

段階6までは操舵角・操舵速度制約を導入しないため、ここでは制御則が出した値をそのまま車両モデルへ入力しています。

## 7. 結論

- 方位誤差フィードバックにより、前進・後退とも初期方位誤差を補正できた。
- 後退では補正舵角の符号反転が必要である。
- 方位誤差が収束しても位置誤差は残る。
- 次に横偏差を制御へ戻す必要がある。

## 8. 出力結果

```text
output/yaw_feedback/
├── forward/
├── reverse/
├── forward_lateral_offset/
├── forward_pose_offset/
├── reverse_lateral_offset/
└── reverse_pose_offset/
```

各フォルダにCSV、軌跡PNG、時系列PNG、GIFを保存しています。
