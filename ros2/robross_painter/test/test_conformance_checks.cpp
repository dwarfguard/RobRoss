#include <gtest/gtest.h>

#include <cmath>

#include "conformance_checks.hpp"

namespace
{

using robross_painter::CartesianCheckPolicy;
using robross_painter::CartesianConformanceError;
using robross_painter::EndpointCheckPolicy;
using robross_painter::EndpointConformanceError;

TEST(ConformanceChecks, CartesianDeviationChecksAreIndependent)
{
    CartesianCheckPolicy policy;
    CartesianConformanceError error;

    error.tangential_mm = 3.0;
    EXPECT_FALSE(robross_painter::cartesianConformanceAccepted(error, policy));
    policy.deviation_enabled = false;
    EXPECT_TRUE(robross_painter::cartesianConformanceAccepted(error, policy));

    policy = CartesianCheckPolicy{};
    error = CartesianConformanceError{};
    error.inward_normal_mm = 0.3;
    EXPECT_FALSE(robross_painter::cartesianConformanceAccepted(error, policy));
    policy.normal_deviation_enabled = false;
    EXPECT_TRUE(robross_painter::cartesianConformanceAccepted(error, policy));

    policy = CartesianCheckPolicy{};
    error = CartesianConformanceError{};
    error.orientation_deg = 3.0;
    EXPECT_FALSE(robross_painter::cartesianConformanceAccepted(error, policy));
    policy.orientation_deviation_enabled = false;
    EXPECT_TRUE(robross_painter::cartesianConformanceAccepted(error, policy));

    policy.deviation_enabled = false;
    policy.normal_deviation_enabled = false;
    EXPECT_FALSE(robross_painter::anyCartesianCheckEnabled(policy));
}

TEST(ConformanceChecks, EndpointChecksAreIndependent)
{
    EndpointCheckPolicy policy;
    EndpointConformanceError error;
    error.hard_limits_ok = true;

    error.max_joint_deg = 3.0;
    EXPECT_FALSE(robross_painter::endpointConformanceAccepted(error, policy));
    policy.joint_error_enabled = false;
    EXPECT_TRUE(robross_painter::endpointConformanceAccepted(error, policy));

    policy = EndpointCheckPolicy{};
    error = EndpointConformanceError{};
    error.hard_limits_ok = true;
    error.tip_mm = 2.0;
    EXPECT_FALSE(robross_painter::endpointConformanceAccepted(error, policy));
    policy.tip_error_enabled = false;
    EXPECT_TRUE(robross_painter::endpointConformanceAccepted(error, policy));

    policy = EndpointCheckPolicy{};
    error = EndpointConformanceError{};
    error.hard_limits_ok = true;
    error.orient_deg = 2.0;
    EXPECT_FALSE(robross_painter::endpointConformanceAccepted(error, policy));
    policy.tip_orientation_error_enabled = false;
    EXPECT_TRUE(robross_painter::endpointConformanceAccepted(error, policy));

    policy.joint_error_enabled = false;
    policy.tip_error_enabled = false;
    EXPECT_DOUBLE_EQ(robross_painter::endpointErrorRatio(error, policy), 0.0);
}

TEST(ConformanceChecks, HardEndpointLimitsCannotBeDisabled)
{
    EndpointCheckPolicy policy;
    policy.joint_error_enabled = false;
    policy.tip_error_enabled = false;
    policy.tip_orientation_error_enabled = false;

    EndpointConformanceError error;
    error.hard_limits_ok = false;
    EXPECT_FALSE(robross_painter::endpointConformanceAccepted(error, policy));
    EXPECT_TRUE(std::isinf(robross_painter::endpointErrorRatio(error, policy)));
}

}  // namespace
