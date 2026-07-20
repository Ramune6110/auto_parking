# Pure Pursuit・Stanley比較実験

## 1. 実験の目的

段階5として、位置基準で経路を参照するPure PursuitとStanleyを、同じ`TrajectoryController`インターフェース、軌道、初期状態、車両モデル、周期で比較します。段階4の横偏差フィードバックも比較対象に含めます。

## 2. 共通インターフェース

全制御器は`ControllerInput`を受け取ります。

```cpp
struct ControllerInput {
    double time;
    const VehicleState& current_state;
    const TrajectoryPoint& time_reference;
    const Trajectory& trajectory;
    std::size_t nearest_index;
};
```

既存制御器は`time_reference`、Pure Pursuitは`trajectory`と`nearest_index`、Stanleyは最近傍点を主に利用します。

## 3. パラメーター

| 制御器 | パラメーター | 値 |
|---|---|---:|
| Lateral feedback | 横偏差ゲイン | 0.2 rad/m |
| Pure Pursuit | 注視距離 | 1.5 m |
| Stanley | 横偏差ゲイン | 0.5 |
| Stanley | 低速安定化速度 | 1.0 m/s |

## 4. 横偏差ケースの結果

| 制御器 | 方向 | 最終位置誤差 [m] | 最大位置誤差 [m] | 最大絶対舵角 [deg] |
|---|---|---:|---:|---:|
| Lateral feedback | 前進 | 0.112 | 0.500 | 35.263 |
| Pure Pursuit | 前進 | 0.021 | 0.500 | 47.203 |
| Stanley | 前進 | 0.053 | 0.500 | 36.235 |
| Lateral feedback | 後退 | 0.215 | 0.500 | 35.644 |
| Pure Pursuit | 後退 | 0.014 | 0.500 | 47.203 |
| Stanley | 後退 | 0.170 | 0.500 | 36.722 |

固定注視距離のPure Pursuitが最も小さい終端位置誤差になりましたが、最大舵角も最も大きくなりました。

## 5. 複合姿勢誤差ケースの結果

| 制御器 | 方向 | 最終位置誤差 [m] | 最大位置誤差 [m] | 最終方位誤差 [deg] | 最大絶対舵角 [deg] |
|---|---|---:|---:|---:|---:|
| Lateral feedback | 前進 | 0.297 | 0.635 | -2.570 | 36.515 |
| Pure Pursuit | 前進 | 0.276 | 0.589 | -3.015 | 53.638 |
| Stanley | 前進 | 0.462 | 0.619 | -0.261 | 36.867 |
| Lateral feedback | 後退 | 0.126 | 0.586 | -2.054 | 35.649 |
| Pure Pursuit | 後退 | 0.319 | 0.586 | -4.426 | 34.920 |
| Stanley | 後退 | 0.281 | 0.586 | -3.829 | 36.669 |

Pure PursuitとStanleyは経路上の最近傍・注視点を使いますが、速度は従来どおり時刻軌道から与えています。縦方向の進み遅れを補正する制御はないため、pose offsetの$x=+0.3$ mは終端位置誤差として残りやすくなります。

## 6. 比較から分かること

- Pure Pursuitは横偏差ケースの終端精度が高い一方、注視距離1.5 mでは操舵角が大きい。
- Stanleyは方位を揃えやすく、操舵角はPure Pursuitより小さいが、今回の後退pose条件では位置誤差が残る。
- 単純横偏差フィードバックは構造を理解しやすく、後退poseでは今回の3方式中で最小の終端位置誤差だった。
- 一つの終端値だけで優劣は決められず、最大誤差、舵角、時系列、前進・後退を併せて評価する必要がある。
- 後から追加した操舵制約を有効にすると、要求舵角が大きいPure Pursuitでは特に結果が変化します。詳細は[操舵制約・一次遅れ比較実験](steering_actuator_comparison.md)を参照してください。

## 7. 出力結果

```text
output/
├── pure_pursuit/
│   └── 6シナリオ/
└── stanley/
    └── 6シナリオ/
```

各シナリオにCSV、軌跡PNG、時系列PNG、GIFを保存しています。

## 8. 次の段階

段階6では操舵角と操舵速度の制約を共通処理として追加します。制御器が計算した生舵角と、制約後に車両へ適用した舵角を分けて記録し、公平に比較します。
