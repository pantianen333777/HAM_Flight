#include <gtest/gtest.h>

#include <cmath>

#include <trajectory_math/figure_eight.hpp>

namespace
{
constexpr double kTolerance = 1e-12;

TEST(FigureEightTrajectory, XStartsWithZeroSlopeAndNeverBecomesPositive)
{
    constexpr double radius = 1.0;
    constexpr double amplitude_y = 0.7;

    const trajectory_math::PlanarTrajectoryState start =
        trajectory_math::evaluateNegativeXFigureEight(
            0.0, 1.0, 0.0, 0.0, radius, amplitude_y);
    EXPECT_NEAR(start.x, 0.0, kTolerance);
    EXPECT_NEAR(start.y, 0.0, kTolerance);
    EXPECT_NEAR(start.vx, 0.0, kTolerance);

    constexpr int sample_count = 2000;
    for (int i = 1; i <= sample_count; ++i)
    {
        const double phi = trajectory_math::kTwoPi *
                           static_cast<double>(i) /
                           static_cast<double>(sample_count);
        const trajectory_math::PlanarTrajectoryState state =
            trajectory_math::evaluateNegativeXFigureEight(
                phi, 0.0, 0.0, 0.0, radius, amplitude_y);
        EXPECT_LE(state.x, kTolerance);
    }

    const trajectory_math::PlanarTrajectoryState after_start =
        trajectory_math::evaluateNegativeXFigureEight(
            1e-3, 0.0, 0.0, 0.0, radius, amplitude_y);
    EXPECT_LT(after_start.x, 0.0);
}

TEST(FigureEightTrajectory, RestToRestBoundariesKeepAllDerivativesContinuous)
{
    constexpr double radius = 1.0;
    constexpr double amplitude_y = 0.7;

    const trajectory_math::PlanarTrajectoryState start =
        trajectory_math::evaluateNegativeXFigureEight(
            0.0, 0.0, 0.0, 0.0, radius, amplitude_y);
    const trajectory_math::PlanarTrajectoryState end =
        trajectory_math::evaluateNegativeXFigureEight(
            trajectory_math::kTwoPi, 0.0, 0.0, 0.0,
            radius, amplitude_y);

    EXPECT_NEAR(start.x, 0.0, kTolerance);
    EXPECT_NEAR(start.y, 0.0, kTolerance);
    EXPECT_NEAR(end.x, 0.0, kTolerance);
    EXPECT_NEAR(end.y, 0.0, kTolerance);

    EXPECT_NEAR(start.vx, 0.0, kTolerance);
    EXPECT_NEAR(start.vy, 0.0, kTolerance);
    EXPECT_NEAR(start.ax, 0.0, kTolerance);
    EXPECT_NEAR(start.ay, 0.0, kTolerance);
    EXPECT_NEAR(start.jx, 0.0, kTolerance);
    EXPECT_NEAR(start.jy, 0.0, kTolerance);

    EXPECT_NEAR(end.vx, 0.0, kTolerance);
    EXPECT_NEAR(end.vy, 0.0, kTolerance);
    EXPECT_NEAR(end.ax, 0.0, kTolerance);
    EXPECT_NEAR(end.ay, 0.0, kTolerance);
    EXPECT_NEAR(end.jx, 0.0, kTolerance);
    EXPECT_NEAR(end.jy, 0.0, kTolerance);
}

TEST(FigureEightTrajectory, YawIsZeroAtManeuverStartThenReachesTangent)
{
    constexpr double radius = 1.0;
    constexpr double amplitude_y = 0.7;
    constexpr double blend_time = 5.0;
    const double tangent_yaw =
        trajectory_math::negativeXFigureEightTangentYaw(
            0.0, radius, amplitude_y);

    EXPECT_NEAR(tangent_yaw, -trajectory_math::kPi / 2.0, kTolerance);
    EXPECT_NEAR(
        trajectory_math::blendYawFromZero(
            tangent_yaw, 0.0, blend_time),
        0.0, kTolerance);
    EXPECT_NEAR(
        trajectory_math::blendYawFromZero(
            tangent_yaw, blend_time, blend_time),
        tangent_yaw, kTolerance);
}
} // namespace
