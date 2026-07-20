// Traceability: ACT-EQ-02〜ACT-EQ-08
#pragma once

/** @brief 操舵角制約、操舵速度制約、一次遅れの設定。 */
struct SteeringActuatorConfig {
    double max_steering_angle{};       // 最大絶対舵角 [rad]
    double max_steering_rate{};        // 最大絶対操舵速度 [rad/s]
    double time_constant{};             // 一次遅れ時定数 [s]
    double steering_bias{};             // 実舵角へ加わる一定オフセット [rad]
    bool enable_angle_limit{};
    bool enable_rate_limit{};
    bool enable_first_order_lag{};
};

/** @brief 1周期の操舵アクチュエータ計算結果。 */
struct SteeringActuatorOutput {
    double requested_steering{};  // 制御器が要求した舵角 [rad]
    double applied_steering{};    // 制約とバイアスを反映した実舵角 [rad]
    double steering_rate{};       // 実現した操舵速度 [rad/s]
    bool angle_limit_active{};    // 舵角制約が作動したか
    bool rate_limit_active{};     // 操舵速度制約が作動したか
};

/** @brief 制御器の要求舵角から、車両へ適用可能な実舵角を計算する。 */
class SteeringActuator {
public:
    /** @param config 再現する操舵機構の制約と応答特性。 */
    explicit SteeringActuator(const SteeringActuatorConfig& config);

    /**
     * @brief 1制御周期分の操舵応答を進める。
     * @param requested_steering 制御器の要求舵角 [rad]。
     * @param dt 制御周期 [s]。
     * @return 車両へ適用する舵角と、作動した制約の情報。
     */
    SteeringActuatorOutput update(double requested_steering, double dt);

    /** @return 現時点で車両へ適用されている舵角 [rad]。 */
    double steering() const noexcept;

private:
    SteeringActuatorConfig config_;
    double applied_steering_{};
};
