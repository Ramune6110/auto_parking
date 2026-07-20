// Traceability: COORD-EQ-07（角度差を[-pi, pi]へ正規化）
#pragma once

#include <cmath>

/** @brief 角度を [-pi, pi] の範囲へ正規化する。 */
inline double normalizeAngle(double angle) {
    return std::atan2(std::sin(angle), std::cos(angle));
}
