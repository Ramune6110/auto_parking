# 操舵制約・一次遅れ比較実験

## 1. 実験目的

段階6・7として、同じ制御器と初期姿勢偏差に対し、操舵モデルだけを変更して追従性能への影響を比較します。

## 2. 比較条件

| 設定名 | 最大舵角 | 最大操舵速度 | 一次遅れ |
|---|---:|---:|---:|
| `ideal` | 無効 | 無効 | 無効 |
| `angle-limit` | 35 deg | 無効 | 無効 |
| `angle-rate-limit` | 35 deg | 60 deg/s | 無効 |
| `actuator` | 35 deg | 60 deg/s | 0.3 s |

代表ケースにはPure PursuitとStanleyの`forward_pose_offset`を使います。

## 3. 実行方法

```powershell
.\build\parking_trajectory_sim.exe --controller pure-pursuit --forward-pose-offset --steering-model ideal
.\build\parking_trajectory_sim.exe --controller pure-pursuit --forward-pose-offset --steering-model angle-limit
.\build\parking_trajectory_sim.exe --controller pure-pursuit --forward-pose-offset --steering-model angle-rate-limit
.\build\parking_trajectory_sim.exe --controller pure-pursuit --forward-pose-offset --steering-model actuator
```

`--controller stanley`へ変更するとStanleyを同じ条件で評価できます。

## 4. 出力構成

```text
output/<controller>/forward_pose_offset/
├── ideal/
├── angle_limit/
├── angle_rate_limit/
└── actuator/
```

各フォルダの`vehicle_log.csv`には要求舵角、実舵角、操舵速度、制約作動フラグを保存します。グラフには要求舵角を破線、実舵角を実線で表示します。

## 5. 評価観点

1. 最大舵角が35 deg以内か確認する。
2. 最大操舵速度が60 deg/s以内か確認する。
3. 要求舵角と実舵角の時間差を確認する。
4. 横偏差、方位誤差、終端位置誤差の変化を比較する。
5. 制約が継続して作動する区間を特定する。

## 6. 比較結果

### 6.1 Pure Pursuit

| 操舵モデル | 終端位置誤差 [m] | 終端横偏差 [m] | 終端方位誤差 [deg] | 最大要求舵角 [deg] | 最大実舵角 [deg] |
|---|---:|---:|---:|---:|---:|
| ideal | 0.276 | 0.014 | -3.015 | 53.638 | 53.638 |
| angle-limit | 0.274 | 0.014 | -3.006 | 53.785 | 35.000 |
| angle-rate-limit | 0.274 | 0.014 | -3.010 | 53.908 | 35.000 |
| actuator | 0.251 | -0.003 | 1.479 | 89.175 | 34.992 |

### 6.2 Stanley

| 操舵モデル | 終端位置誤差 [m] | 終端横偏差 [m] | 終端方位誤差 [deg] | 最大要求舵角 [deg] | 最大実舵角 [deg] |
|---|---:|---:|---:|---:|---:|
| ideal | 0.462 | -0.033 | -0.261 | 36.867 | 36.867 |
| angle-limit | 0.422 | -0.203 | -1.685 | 40.040 | 35.000 |
| angle-rate-limit | 0.350 | -0.347 | -4.676 | 45.459 | 35.000 |
| actuator | 0.410 | -0.395 | -6.184 | 47.373 | 35.000 |

速度制約を有効にした全ケースで最大実操舵速度は60 deg/sでした。したがって、設定した最大舵角と最大操舵速度は守られています。

### 6.3 結果の解釈

- Pure Pursuitでは制約後も終端横偏差は小さい一方、一次遅れ込みでは制御器が最大89.175 degを要求しています。制御器が操舵系の遅れや飽和を知らず、残った誤差に対して要求を強めた結果です。
- Stanleyでは制約と遅れを加えるほど終端横偏差・方位誤差が増えました。今回のゲインは理想操舵で調整されており、操舵応答を含めた再調整が必要です。
- 終端位置誤差だけが小さくなっても、横偏差と方位誤差が改善したとは限りません。複数の指標を同時に確認する必要があります。
- このような「要求値は大きいが実現できない」問題を予測段階で扱うことが、後に学ぶMPCの重要な目的です。

## 7. 次段階との関係

要求値と実現値の区別、および操舵状態はLQRとMPCでも共通利用します。MPCでは後段で制限するだけでなく、将来の制約を予測問題の内部へ組み込みます。
