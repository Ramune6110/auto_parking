// Traceability: PATH-EQ-04, PATH-EQ-05, PATH-EQ-10〜PATH-EQ-12
#pragma once

#include "core/types.hpp"

/**
 * @brief 直線と一定曲率円弧をつないで幾何学的な目標経路を生成する。
 *
 * このクラスは時間・速度を扱わない。前進経路では motion_direction に +1、
 * 後退経路では -1 を指定する。
 */
class PathGenerator {
public:
    PathGenerator(double sample_distance, double motion_direction);

    /** @brief 現在の経路末端へ指定長さの直線を追加する。 */
    void addStraight(double length);

    /**
     * @brief 現在の経路末端へ一定曲率の円弧を追加する。
     * @param radius 円弧半径 [m]。正の値を指定する。
     * @param turn_angle 旋回角の絶対値 [rad]。正の値を指定する。
     * @param turn_direction 左旋回は +1、右旋回は -1。
     */
    void addArc(double radius, double turn_angle, double turn_direction = 1.0);

    const GeometricPath& points() const;

private:
    void appendSegment(double length, double curvature);

    double sample_distance_;
    double motion_direction_;
    GeometricPath points_;
};
