#include <gtest/gtest.h>

#include <builtin_interfaces/msg/duration.hpp>

#include "cartesian_postprocess.hpp"

using robross_painter::closestPointOnPolyline;
using robross_painter::deviationComponents;
using robross_painter::stripDerivatives;

TEST(StripDerivatives, RemovesDerivativesKeepsPositionsAndTime)
{
    trajectory_msgs::msg::JointTrajectory jt;
    jt.points.resize(3);
    for (std::size_t i = 0; i < jt.points.size(); ++i) {
        jt.points[i].positions = { 0.1 * i, 0.2 * i };
        jt.points[i].velocities = { 1.0, 1.0 };
        jt.points[i].accelerations = { 2.0, 2.0 };
        jt.points[i].effort = { 3.0, 3.0 };
        jt.points[i].time_from_start.nanosec = 5000000u * i;
    }
    stripDerivatives(jt);
    for (std::size_t i = 0; i < jt.points.size(); ++i) {
        EXPECT_TRUE(jt.points[i].velocities.empty());
        EXPECT_TRUE(jt.points[i].accelerations.empty());
        EXPECT_TRUE(jt.points[i].effort.empty());
        ASSERT_EQ(jt.points[i].positions.size(), 2u);
        EXPECT_DOUBLE_EQ(jt.points[i].positions[0], 0.1 * i);
        EXPECT_EQ(jt.points[i].time_from_start.nanosec, 5000000u * i);
    }
}

TEST(ClosestPointOnPolyline, ProjectsOntoInteriorOfSegment)
{
    const std::vector<Eigen::Vector3d> line = {
        { 0.0, 0.0, 0.0 }, { 1.0, 0.0, 0.0 }
    };
    const Eigen::Vector3d closest =
        closestPointOnPolyline({ 0.25, 0.5, 0.0 }, line);
    EXPECT_NEAR((closest - Eigen::Vector3d(0.25, 0.0, 0.0)).norm(), 0.0,
                1e-12);
}

TEST(ClosestPointOnPolyline, ClampsBeyondEndpointsAndPicksNearestSegment)
{
    const std::vector<Eigen::Vector3d> poly = {
        { 0.0, 0.0, 0.0 }, { 1.0, 0.0, 0.0 }, { 1.0, 1.0, 0.0 }
    };
    EXPECT_NEAR((closestPointOnPolyline({ -1.0, -1.0, 0.0 }, poly) -
                 Eigen::Vector3d(0.0, 0.0, 0.0)).norm(), 0.0, 1e-12);
    EXPECT_NEAR((closestPointOnPolyline({ 2.0, 0.9, 0.0 }, poly) -
                 Eigen::Vector3d(1.0, 0.9, 0.0)).norm(), 0.0, 1e-12);
}

TEST(DeviationComponents, SignedNormalPositiveIntoPaper)
{
    const Eigen::Vector3d normal(0.0, 0.0, 1.0);  // canvas z, into the paper
    const auto inward = deviationComponents({ 0.0, 0.0, 0.0003 }, normal);
    EXPECT_NEAR(inward.normal_signed, 0.0003, 1e-15);
    EXPECT_NEAR(inward.tangential, 0.0, 1e-15);
    const auto outward = deviationComponents({ 0.0, 0.0, -0.0004 }, normal);
    EXPECT_NEAR(outward.normal_signed, -0.0004, 1e-15);
}

TEST(DeviationComponents, NormalAndTangentialAreIndependent)
{
    const Eigen::Vector3d normal(0.0, 0.0, 1.0);
    // Large tangential error with tiny normal error: must not leak into
    // the normal component (and vice versa).
    const auto skewed = deviationComponents({ 0.0015, 0.0, 0.00005 }, normal);
    EXPECT_NEAR(skewed.normal_signed, 0.00005, 1e-15);
    EXPECT_NEAR(skewed.tangential, 0.0015, 1e-15);
    const auto pierced = deviationComponents({ 0.00005, 0.0, 0.0015 }, normal);
    EXPECT_NEAR(pierced.normal_signed, 0.0015, 1e-15);
    EXPECT_NEAR(pierced.tangential, 0.00005, 1e-15);
}
