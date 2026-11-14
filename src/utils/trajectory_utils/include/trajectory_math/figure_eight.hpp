#ifndef TRAJECTORY_MATH_FIGURE_EIGHT_HPP
#define TRAJECTORY_MATH_FIGURE_EIGHT_HPP

#include <cmath>

#include <trajectory_math/rest_to_rest.hpp>

namespace trajectory_math
{
    constexpr double kPi = 3.14159265358979323846;
    constexpr double kTwoPi = 2.0 * kPi;

    struct PlanarTrajectoryState
    {
        double x;
        double y;
        double vx;
        double vy;
        double ax;
        double ay;
        double jx;
        double jy;
    };

    // 从 +X 极值点出发并平移到 (0, 0) 的八字轨迹:
    //   x = R(cos(phi) - 1) <= 0
    //   y = -Ay sin(2 phi)
    // 相位轮廓提供 phi 及其一至三阶时间导数。
    // phi=0 处 dx/dphi=0, 配 rest-to-rest 相位轮廓, 爬升→机动边界 p/v/a/j 连续。
    inline PlanarTrajectoryState evaluateNegativeXFigureEight(
        double phi,
        double phi_dot,
        double phi_ddot,
        double phi_dddot,
        double radius,
        double amplitude_y)
    {
        const double sin_phi = std::sin(phi);
        const double cos_phi = std::cos(phi);
        const double sin_2phi = std::sin(2.0 * phi);
        const double cos_2phi = std::cos(2.0 * phi);
        const double phi_dot2 = phi_dot * phi_dot;
        const double phi_dot3 = phi_dot2 * phi_dot;

        const double dx = -radius * sin_phi;
        const double dy = -2.0 * amplitude_y * cos_2phi;
        const double d2x = -radius * cos_phi;
        const double d2y = 4.0 * amplitude_y * sin_2phi;
        const double d3x = radius * sin_phi;
        const double d3y = 8.0 * amplitude_y * cos_2phi;

        PlanarTrajectoryState out;
        out.x = radius * (cos_phi - 1.0);
        out.y = -amplitude_y * sin_2phi;
        out.vx = dx * phi_dot;
        out.vy = dy * phi_dot;
        out.ax = d2x * phi_dot2 + dx * phi_ddot;
        out.ay = d2y * phi_dot2 + dy * phi_ddot;
        out.jx = d3x * phi_dot3 + 3.0 * d2x * phi_dot * phi_ddot +
                 dx * phi_dddot;
        out.jy = d3y * phi_dot3 + 3.0 * d2y * phi_dot * phi_ddot +
                 dy * phi_dddot;
        return out;
    }

    inline double negativeXFigureEightTangentYaw(
        double phi, double radius, double amplitude_y)
    {
        const double dx = -radius * std::sin(phi);
        const double dy = -2.0 * amplitude_y * std::cos(2.0 * phi);
        if (std::hypot(dx, dy) <= 1e-12)
        {
            return 0.0;
        }
        return std::atan2(dy, dx);
    }

    // 机动首拍保持 yaw=0, 之后在机动段内部平滑混入瞬时切线;
    // elapsed >= duration 后返回值恰为切线航向。
    inline double blendYawFromZero(
        double tangent_yaw, double elapsed, double duration)
    {
        if (duration <= 1e-9)
        {
            return tangent_yaw;
        }
        const double weight = smoothP(clamp01(elapsed / duration));
        return weight * std::remainder(tangent_yaw, kTwoPi);
    }
} // namespace trajectory_math

#endif // TRAJECTORY_MATH_FIGURE_EIGHT_HPP
