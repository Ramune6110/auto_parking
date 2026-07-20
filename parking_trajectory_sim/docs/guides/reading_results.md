# 結果の読み方

## 1. 出力ファイル

`reference_trajectory.csv` は制御器へ与えた目標値、`vehicle_log.csv` は車両状態・制御指令・追従誤差の時系列です。PNGは軌跡と時系列を確認するためのグラフ、GIFは追従過程のアニメーションです。

## 2. 位置誤差の導出

世界座標上の $X$ 方向位置差と $Y$ 方向位置差を式 (1) で定義します。

$$
\Delta X=X-X_r,\qquad \Delta Y=Y-Y_r
\tag{1}
$$

この二つを直角三角形の横辺と縦辺として考えます。位置誤差は斜辺の長さなので、ピタゴラスの定理から式 (2) です。

$$
e_{\mathrm{position}}^2=\Delta X^2+\Delta Y^2
\tag{2}
$$

距離は負にならないため、式 (2) の正の平方根を取ります。これがCSVの `position_error` です。

$$
e_{\mathrm{position}}
=\sqrt{\Delta X^2+\Delta Y^2}
=\sqrt{(X-X_r)^2+(Y-Y_r)^2}
\tag{3}
$$

## 3. 符号付き誤差を確認する理由

式 (3) は誤差の大きさだけを表し、車両が目標の左右・前後のどちら側にあるかを失います。制御設計では `lateral_error`、`longitudinal_error`、`yaw_error` も確認してください。詳しい導出は[座標系と追従誤差](../fundamentals/coordinate_system.md)の式 (4)、式 (5)、式 (7) にあります。

## 4. グラフを読む順序

1. 目標軌道と車両軌跡を見て、大きな方向違いや発散がないか確認します。
2. 速度と加速度を見て、前進・後退の符号と制限値を確認します。
3. 曲率と舵角を見て、直線・左旋回・右旋回の対応を確認します。
4. 横偏差・縦偏差・方位誤差を見て、誤差の方向と時間変化を確認します。

## 5. 基準ケースの解釈

前進フィードフォワードで初期状態が目標と一致する基準ケースでは、誤差はほぼゼロです。初期状態をずらした評価で誤差が残ることは不具合ではなく、誤差補正を持たないフィードフォワード制御の性質です。

## 6. 段階11～14の縦制御グラフ

1. 速度図で`Trajectory reference`、`Control target`、`Actual`の順に差が生じる理由を確認します。
2. 加速度図で要求値と適用値の差を見て、加減速度制約の作動を確認します。
3. ジャーク図が設定上限内か確認します。
4. 残距離が0へ近づくにつれて制御用目標速度と実車速が0へ収束するか確認します。
5. 停止背景区間へ入る時点で`actual_velocity=0`になっていることをCSVでも確認します。

理論は[車速フィードバック](../controllers/longitudinal_velocity_control.md)、[加減速度・ジャーク制約](../controllers/longitudinal_constraints.md)、[停止位置制御](../controllers/stop_position_control.md)、[縦横協調](../controllers/longitudinal_lateral_coordination.md)を参照してください。
