# 縦方向・横方向制御の協調

## 1. 目的

横制御器は舵角、縦制御器は加速度を独立に計算します。協調層は、曲率や追従誤差が大きいときに縦制御の速度目標を下げます。同一評価関数による同時最適化ではありません。

```mermaid
flowchart TD
    Trajectory["目標軌道<br/>x_ref, y_ref, yaw_ref, κ_ref, v_ref"]

    Trajectory --> Lateral["横方向制御器<br/>Pure Pursuit / Stanley / LQR"]
    State["車両状態<br/>x, y, yaw, 実車速 v"] --> Lateral
    Lateral --> RequestedSteering["要求舵角 δ_cmd"]
    RequestedSteering --> SteeringActuator["操舵アクチュエータ<br/>舵角・操舵速度・一次遅れ"]
    SteeringActuator --> AppliedSteering["適用舵角 δ"]

    Trajectory --> Error["横偏差 e_y・方位誤差 e_ψ<br/>曲率 κ_ref"]
    State --> Error
    SteeringActuator --> Saturation["前周期の操舵制約状態"]
    Error --> Coordinator["縦横協調器<br/>複数の速度上限を計算"]
    Saturation --> Coordinator
    Trajectory --> Coordinator
    Coordinator --> CoordinatedVelocity["協調後目標速度 v_coord"]

    CoordinatedVelocity --> StopControl["停止位置制御<br/>残距離による速度上限"]
    State --> StopControl
    StopControl --> VelocityControl["速度P制御"]
    VelocityControl --> RequestedAcceleration["要求加速度 a_cmd"]
    RequestedAcceleration --> AccelerationLimits["加減速度・ジャーク制約"]
    AccelerationLimits --> AppliedAcceleration["適用加速度 a"]

    AppliedSteering --> Vehicle["キネマティック自転車モデル"]
    AppliedAcceleration --> Vehicle
    Vehicle --> NextState["次周期の x, y, yaw, 実車速 v"]
    NextState --> State
```

**図1　横方向制御と縦方向制御を速度上限で接続する詳細制御フロー**

図1で、横制御器と縦制御器は独立しています。`LongitudinalLateralCoordinator`は横偏差、方位誤差、曲率、操舵制約を速度上限へ変換するだけであり、舵角と加速度を同じ評価関数で解いてはいません。

## 2. 曲率による速度上限

横加速度$a_y=v^2|\kappa|$を上限$a_{y,\max}$以内にすると、速度上限は次式です。

<a id="coop-eq-01"></a>

**理論ID：`COOP-EQ-01`**

<a id="coop-eq-01"></a>

**理論ID：`COOP-EQ-01`**

<a id="coop-eq-01"></a>

**理論ID：`COOP-EQ-01`**

$$
v_{\kappa,\max}=\sqrt{\frac{a_{y,\max}}{|\kappa|}}
\tag{1}
$$

## 3. 誤差による速度上限

横偏差が大きいほど速度を下げます。

<a id="coop-eq-02"></a>

**理論ID：`COOP-EQ-02`**

<a id="coop-eq-02"></a>

**理論ID：`COOP-EQ-02`**

<a id="coop-eq-02"></a>

**理論ID：`COOP-EQ-02`**

$$
v_{e_y,\max}=\frac{|v_{\mathrm{ref}}|}{1+K_y|e_y|}
\tag{2}
$$

方位誤差についても同じ考え方です。

<a id="coop-eq-03"></a>

**理論ID：`COOP-EQ-03`**

<a id="coop-eq-03"></a>

**理論ID：`COOP-EQ-03`**

<a id="coop-eq-03"></a>

**理論ID：`COOP-EQ-03`**

$$
v_{e_\psi,\max}=\frac{|v_{\mathrm{ref}}|}{1+K_\psi|e_\psi|}
\tag{3}
$$

操舵制約が作動した場合は倍率$r_\delta\le1$を掛けます。

<a id="coop-eq-04"></a>

**理論ID：`COOP-EQ-04`**

<a id="coop-eq-04"></a>

**理論ID：`COOP-EQ-04`**

<a id="coop-eq-04"></a>

**理論ID：`COOP-EQ-04`**

$$
v_{\delta,\max}=r_\delta|v_{\mathrm{ref}}|
\tag{4}
$$

最終速度はすべての上限の最小値へ前後進符号$q$を戻します。

<a id="coop-eq-05"></a>

**理論ID：`COOP-EQ-05`**

<a id="coop-eq-05"></a>

**理論ID：`COOP-EQ-05`**

<a id="coop-eq-05"></a>

**理論ID：`COOP-EQ-05`**

$$
v_{\mathrm{coord}}=q\min(|v_{\mathrm{ref}}|,v_{\kappa,\max},v_{e_y,\max},v_{e_\psi,\max},v_{\delta,\max})
\tag{5}
$$

## 4. ソースコードとの対応

式 (1)～式 (5)は`LongitudinalLateralCoordinator::coordinate()`に一対一で対応します。`testLongitudinalLateralCoordination()`で、誤差・曲率・操舵制約により速度が低下することを検証します。

## 5. 実装上の1周期の順序

```mermaid
sequenceDiagram
    participant Sim as runParkingScenario
    participant Lat as TrajectoryController
    participant Steer as SteeringActuator
    participant Coord as Coordinator
    participant Lon as VelocityStopController
    participant Model as KinematicBicycle
    Sim->>Lat: 状態・参照軌道
    Lat-->>Sim: 要求舵角
    Sim->>Steer: 要求舵角
    Steer-->>Sim: 適用舵角・制約状態
    Sim->>Coord: 曲率・横偏差・方位誤差・前周期制約
    Coord-->>Sim: 協調後目標速度
    Sim->>Lon: 目標速度・実車速・残距離
    Lon-->>Sim: 制約後加速度
    Sim->>Model: 適用舵角・適用加速度
    Model-->>Sim: 次周期状態
```

**図2　`runParkingScenario()`における呼び出し順序**

## 6. 前後の文書

- 縦制御の入口：[車速状態と速度フィードバック制御](longitudinal_velocity_control.md)
- 終端停止：[停止位置制御](stop_position_control.md)
- プロジェクト内の統合位置：[ソースコードの処理フロー](../architecture/processing_flow.md)
- 将来の同時最適化との差：[学習ロードマップ](../learning_roadmap.md)

