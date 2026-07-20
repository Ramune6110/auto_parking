// Traceability: LINERR-EQ-13〜LINERR-EQ-19（線形誤差モデルの離散化）
#include "models/linear_error_model.hpp"

#include <cmath>
#include <stdexcept>

LinearErrorModelMatrices createLinearErrorModel(
    double velocity,
    double feedforward_steering,
    double wheelbase,
    double time_step) {
    if (wheelbase <= 0.0 || time_step <= 0.0) {
        throw std::invalid_argument(
            "wheelbase and linear model time step must be positive");
    }

    const double cosine = std::cos(feedforward_steering);
    if (std::abs(cosine) < 1.0e-6) {
        throw std::invalid_argument(
            "feedforward steering is too close to 90 degrees");
    }

    // e_yは参照経路の左側が正、e_psiは参照方位－車両方位。
    // よって e_y_dot=-v*e_psi、e_psi_dot=-v/(L*cos^2(delta_ff))*Delta_delta。
    LinearErrorModelMatrices model;
    // LINERR-EQ-16〜LINERR-EQ-18：前進Euler法で連続状態行列を離散化する。
    model.state_matrix <<
        1.0, -velocity * time_step,
        0.0, 1.0;
    // LINERR-EQ-12, LINERR-EQ-15, LINERR-EQ-19：舵角偏差に対する離散入力行列。
    model.input_matrix <<
        0.0,
        -velocity * time_step / (wheelbase * cosine * cosine);
    return model;
}
