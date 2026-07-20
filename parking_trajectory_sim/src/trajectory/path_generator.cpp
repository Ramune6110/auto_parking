// Traceability: PATH-EQ-04, PATH-EQ-05, PATH-EQ-10〜PATH-EQ-12
#include "trajectory/path_generator.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace {
constexpr double kCurvatureEpsilon = 1.0e-12;
}

PathGenerator::PathGenerator(double sample_distance, double motion_direction)
    : sample_distance_(sample_distance), motion_direction_(motion_direction) {
    if (sample_distance_ <= 0.0) {
        throw std::invalid_argument("sample_distance must be positive");
    }
    if (motion_direction_ != 1.0 && motion_direction_ != -1.0) {
        throw std::invalid_argument("motion_direction must be +1 or -1");
    }
    // すべてのサンプル経路は原点・方位角0から生成する。
    points_.push_back({0.0, 0.0, 0.0, 0.0, 0.0});
}

void PathGenerator::addStraight(double length) {
    appendSegment(length, 0.0);
}

void PathGenerator::addArc(
    double radius, double turn_angle, double turn_direction) {
    if (radius <= 0.0 || turn_angle <= 0.0) {
        throw std::invalid_argument("arc radius and turn angle must be positive");
    }
    if (turn_direction != 1.0 && turn_direction != -1.0) {
        throw std::invalid_argument("turn_direction must be +1 or -1");
    }
    // PATH-EQ-12：旋回方向の符号と半径から曲率を定義する。
    appendSegment(radius * turn_angle, turn_direction / radius);
}

const GeometricPath& PathGenerator::points() const {
    return points_;
}

void PathGenerator::appendSegment(double length, double curvature) {
    if (length <= 0.0) {
        throw std::invalid_argument("path segment length must be positive");
    }
    // 端点を正確に含めるため、区間長を等分して実際の刻み幅を決める。
    const int step_count = std::max(
        1, static_cast<int>(std::ceil(length / sample_distance_)));
    const double step = length / static_cast<double>(step_count);

    for (int i = 0; i < step_count; ++i) {
        const GeometricPathPoint previous = points_.back();
        GeometricPathPoint next;
        next.s = previous.s + step;
        next.curvature = curvature;

        if (std::abs(curvature) < kCurvatureEpsilon) {
            // 曲率0では直線の運動学を使い、曲率による除算を避ける。
            // PATH-EQ-05：曲率0の直線区間を前後進符号q付きで更新する。
            next.x = previous.x +
                motion_direction_ * step * std::cos(previous.yaw);
            next.y = previous.y +
                motion_direction_ * step * std::sin(previous.yaw);
            next.yaw = previous.yaw;
        } else {
            // 一定曲率の円弧を解析的に積分し、次の姿勢を求める。
            // PATH-EQ-04：一定曲率と距離刻みから次の方位を求める。
            next.yaw = previous.yaw +
                motion_direction_ * curvature * step;
            // PATH-EQ-10, PATH-EQ-11：円弧を解析積分して次の位置を求める。
            next.x = previous.x +
                (std::sin(next.yaw) - std::sin(previous.yaw)) / curvature;
            next.y = previous.y -
                (std::cos(next.yaw) - std::cos(previous.yaw)) / curvature;
        }
        points_.push_back(next);
    }
}
