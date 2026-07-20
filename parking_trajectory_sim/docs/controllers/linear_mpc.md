# 段階15　横方向線形MPC

## 1. この章の目的

現在の横偏差と方位誤差から、将来の誤差を有限周期先まで予測し、補正舵角列を計算します。毎周期、列の先頭だけを車両へ適用して再計算するReceding Horizon制御を学びます。

## 2. 状態と入力

横偏差$e_y$と方位誤差$e_\psi$を状態にします。

<a id="mpc-eq-01"></a>

**理論ID：`MPC-EQ-01`**

<a id="mpc-eq-01"></a>

**理論ID：`MPC-EQ-01`**

$$
\boldsymbol{x}_k=\begin{bmatrix}e_{y,k}\\e_{\psi,k}\end{bmatrix}
\tag{1}
$$

実舵角を、参照曲率から求めたフィードフォワード舵角と補正舵角へ分けます。

<a id="mpc-eq-02"></a>

**理論ID：`MPC-EQ-02`**

<a id="mpc-eq-02"></a>

**理論ID：`MPC-EQ-02`**

$$
\delta_k=\delta_{\mathrm{ff},k}+u_k
\tag{2}
$$

LQRで導出した離散線形誤差モデルを使用します。

<a id="mpc-eq-03"></a>

**理論ID：`MPC-EQ-03`**

<a id="mpc-eq-03"></a>

**理論ID：`MPC-EQ-03`**

$$
\boldsymbol{x}_{k+1}=A\boldsymbol{x}_k+B u_k
\tag{3}
$$

## 3. 複数周期先の予測

予測ホライズンを$N$とし、将来の補正舵角を一列へ並べます。

<a id="mpc-eq-04"></a>

**理論ID：`MPC-EQ-04`**

<a id="mpc-eq-04"></a>

**理論ID：`MPC-EQ-04`**

$$
\boldsymbol{U}=\begin{bmatrix}u_0&u_1&\cdots&u_{N-1}\end{bmatrix}^{\mathsf T}
\tag{4}
$$

式 (3)を繰り返すと、初期状態が将来状態へ与える部分を$S_x$へまとめられます。

<a id="mpc-eq-05"></a>

**理論ID：`MPC-EQ-05`**

<a id="mpc-eq-05"></a>

**理論ID：`MPC-EQ-05`**

$$
S_x=\begin{bmatrix}A\\A^2\\\vdots\\A^N\end{bmatrix}
\tag{5}
$$

各時刻の入力が将来状態へ与える部分は、下三角の予測行列$S_u$になります。

<a id="mpc-eq-06"></a>

**理論ID：`MPC-EQ-06`**

<a id="mpc-eq-06"></a>

**理論ID：`MPC-EQ-06`**

$$
S_u=\begin{bmatrix}
B&0&\cdots&0\\
AB&B&\cdots&0\\
\vdots&\vdots&\ddots&\vdots\\
A^{N-1}B&A^{N-2}B&\cdots&B
\end{bmatrix}
\tag{6}
$$

したがって、全予測状態は$\boldsymbol{X}=S_x\boldsymbol{x}_0+S_u\boldsymbol{U}$です。

## 4. 有限ホライズン評価関数

将来の誤差と補正舵角を小さくします。終端状態には大きな重みを設定します。

<a id="mpc-eq-07"></a>

**理論ID：`MPC-EQ-07`**

<a id="mpc-eq-07"></a>

**理論ID：`MPC-EQ-07`**

$$
J=\boldsymbol{X}^{\mathsf T}\bar Q\boldsymbol{X}+\boldsymbol{U}^{\mathsf T}\bar R\boldsymbol{U}
\tag{7}
$$

$\boldsymbol{X}=S_x\boldsymbol{x}_0+S_u\boldsymbol{U}$を式 (7)へ代入すると、入力列だけの二次関数になります。

<a id="mpc-eq-08"></a>

**理論ID：`MPC-EQ-08`**

<a id="mpc-eq-08"></a>

**理論ID：`MPC-EQ-08`**

$$
J=\boldsymbol{U}^{\mathsf T}H\boldsymbol{U}+2\boldsymbol{g}^{\mathsf T}\boldsymbol{U}+\mathrm{constant}
\tag{8}
$$

二次項行列は次式です。

<a id="mpc-eq-09"></a>

**理論ID：`MPC-EQ-09`**

<a id="mpc-eq-09"></a>

**理論ID：`MPC-EQ-09`**

$$
H=S_u^{\mathsf T}\bar Q S_u+\bar R
\tag{9}
$$

一次項は次式です。

<a id="mpc-eq-10"></a>

**理論ID：`MPC-EQ-10`**

<a id="mpc-eq-10"></a>

**理論ID：`MPC-EQ-10`**

$$
\boldsymbol{g}=S_u^{\mathsf T}\bar Q S_x\boldsymbol{x}_0
\tag{10}
$$

## 5. 無制約解と制約付き解

制約がなければ、評価関数の勾配を0にして解けます。

<a id="mpc-eq-11"></a>

**理論ID：`MPC-EQ-11`**

<a id="mpc-eq-11"></a>

**理論ID：`MPC-EQ-11`**

$$
\boldsymbol{U}^*=-H^{-1}\boldsymbol{g}
\tag{11}
$$

実装では逆行列を直接作らず、EigenのLDLT分解で$H\boldsymbol{U}=-\boldsymbol{g}$を解きます。

制約付きでは射影勾配法を使用します。

<a id="mpc-eq-12"></a>

**理論ID：`MPC-EQ-12`**

<a id="mpc-eq-12"></a>

**理論ID：`MPC-EQ-12`**

$$
\boldsymbol{U}^{(i+1)}=\Pi_{\mathcal C}\left(\boldsymbol{U}^{(i)}-\alpha(H\boldsymbol{U}^{(i)}+\boldsymbol{g})\right)
\tag{12}
$$

$\Pi_{\mathcal C}$は、計算結果を許容集合へ戻す操作です。

## 6. 舵角・操舵速度制約

実舵角を機械限界内へ制限します。

<a id="mpc-eq-13"></a>

**理論ID：`MPC-EQ-13`**

<a id="mpc-eq-13"></a>

**理論ID：`MPC-EQ-13`**

$$
-\delta_{\max}\le\delta_{\mathrm{ff}}+u_k\le\delta_{\max}
\tag{13}
$$

隣接周期の舵角変化も制限します。

<a id="mpc-eq-14"></a>

**理論ID：`MPC-EQ-14`**

<a id="mpc-eq-14"></a>

**理論ID：`MPC-EQ-14`**

$$
|\delta_k-\delta_{k-1}|\le\dot\delta_{\max}\Delta t
\tag{14}
$$

## 7. Receding Horizon

最適列をすべて一度に適用せず、先頭入力だけを使用します。

<a id="mpc-eq-15"></a>

**理論ID：`MPC-EQ-15`**

<a id="mpc-eq-15"></a>

**理論ID：`MPC-EQ-15`**

$$
u_k=u_0^*
\tag{15}
$$

次周期には新しい車両状態を測り直し、式 (1)から再計算します。前周期の解を1点ずらした列を初期値にするウォームスタートも使用します。

## 8. ソースコードとの対応

| 理論ID | 実装箇所 | 主な変数 |
|---|---|---|
| `MPC-EQ-01`, `02` | `LinearMpcController::computeCommand()` | `state`, `feedforward_steering` |
| `MPC-EQ-05`, `06` | 同上 | `state_prediction`, `input_prediction` |
| `MPC-EQ-07`～`10` | 同上 | `state_weight`, `hessian`, `gradient` |
| `MPC-EQ-11`, `12` | `solveConstrainedSequence()` | `sequence`, `step_size` |
| `MPC-EQ-13`, `14` | `projectSteeringConstraints()` | `steering`, `maximum_change` |
| `MPC-EQ-15` | `computeCommand()` | `sequence(0)`, `warm_start_` |

検証は`tests/linear_mpc_controller_test.cpp`です。無制約解、舵角制約、操舵速度制約、前進・後退、リセットを確認します。

`vehicle_log.csv`の`controller_computation_time_ms`には各周期のMPC計算時間を保存します。予測ホライズン、反復回数、追従精度、計算時間のトレードオフを確認できます。現在の基準設定は30周期予測、最大15反復です。

## 9. 現段階の限界

- 予測区間内で$A$、$B$、フィードフォワード舵角を一定と仮定します。
- 教育用射影勾配法であり、汎用QPソルバーほど厳密・高速ではありません。
- 最適化する操作量は補正舵角だけで、加速度は段階14の独立した縦制御器が扱います。
- 障害物制約や車体境界はMPC内部へ含めていません。

次の段階16では、舵角と加速度を同じ予測モデル・評価関数で同時最適化します。
