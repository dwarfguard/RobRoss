#include <gtest/gtest.h>

#include <limits>

#include <moveit_msgs/msg/move_it_error_codes.hpp>

#include "executor_safety.hpp"

using robross_painter::DriverHealthAssessment;
using robross_painter::DriverHealthSample;
using robross_painter::MotionOutcome;
using robross_painter::MotionStatus;

namespace
{

DriverHealthSample healthy()
{
    DriverHealthSample sample;
    sample.received = true;
    sample.state = 1;
    sample.fault_code = 0;
    sample.writes_enabled = true;
    sample.finite = true;
    sample.source_age_sec = 0.01;
    sample.callback_age_sec = 0.01;
    sample.receipt_age_sec = 0.01;
    sample.stamp_age_sec = 0.01;
    return sample;
}

}  // namespace

TEST(MotionOutcome, FallbackRequiresExplicitPreSubmissionEligibility)
{
    MotionOutcome preflight;
    preflight.status = MotionStatus::PreSubmissionFailed;
    preflight.joint_fallback_eligible = true;
    EXPECT_TRUE(robross_painter::jointFallbackAllowed(preflight));

    MotionOutcome execution;
    execution.status = MotionStatus::ExecutionFailed;
    execution.execution_started = true;
    execution.joint_fallback_eligible = true;
    EXPECT_FALSE(robross_painter::jointFallbackAllowed(execution));

    MotionOutcome post;
    post.status = MotionStatus::PostExecutionFailed;
    post.execution_started = true;
    post.joint_fallback_eligible = true;
    EXPECT_FALSE(robross_painter::jointFallbackAllowed(post));

    preflight.execution_started = true;
    EXPECT_FALSE(robross_painter::jointFallbackAllowed(preflight));
}

TEST(MotionOutcome, AutomaticPaintRecoveryIsStrictlyBounded)
{
    MotionOutcome outcome;
    outcome.kind = robross_painter::MotionKind::Painting;
    outcome.status = MotionStatus::PreSubmissionFailed;

    EXPECT_TRUE(robross_painter::automaticPaintRecoveryAllowed(
        true, "paint_path", outcome, true, true, 0));
    EXPECT_FALSE(robross_painter::automaticPaintRecoveryAllowed(
        false, "paint_path", outcome, true, true, 0));
    EXPECT_FALSE(robross_painter::automaticPaintRecoveryAllowed(
        true, "paint_stroke", outcome, true, true, 0));
    EXPECT_FALSE(robross_painter::automaticPaintRecoveryAllowed(
        true, "paint_path", outcome, false, true, 0));
    EXPECT_FALSE(robross_painter::automaticPaintRecoveryAllowed(
        true, "paint_path", outcome, true, false, 0));
    EXPECT_FALSE(robross_painter::automaticPaintRecoveryAllowed(
        true, "paint_path", outcome, true, true, 1));

    outcome.execution_started = true;
    EXPECT_FALSE(robross_painter::automaticPaintRecoveryAllowed(
        true, "paint_path", outcome, true, true, 0));
}

TEST(DriverHealth, HealthyFreshSampleIsReady)
{
    EXPECT_EQ(robross_painter::assessDriverHealth(healthy(), 1, 0.5),
              DriverHealthAssessment::Ready);
}

TEST(DriverHealth, FaultDisabledAndStaleFailClosed)
{
    auto sample = healthy();
    sample.fault_code = 4;
    EXPECT_EQ(robross_painter::assessDriverHealth(sample, 1, 0.5),
              DriverHealthAssessment::Faulted);

    sample = healthy();
    sample.writes_enabled = false;
    EXPECT_EQ(robross_painter::assessDriverHealth(sample, 1, 0.5),
              DriverHealthAssessment::WritesDisabled);

    sample = healthy();
    sample.receipt_age_sec = 0.51;
    EXPECT_EQ(robross_painter::assessDriverHealth(sample, 1, 0.5),
              DriverHealthAssessment::Stale);
}

TEST(DriverHealth, AnyNonFiniteTelemetryFailsClosed)
{
    auto sample = healthy();
    sample.source_age_sec = std::numeric_limits<double>::infinity();
    EXPECT_EQ(robross_painter::assessDriverHealth(sample, 1, 0.5),
              DriverHealthAssessment::NonFinite);
}

TEST(ExecutorSafety, MoveItCodeHasStableSymbol)
{
    EXPECT_STREQ(robross_painter::moveItCodeName(
                     moveit_msgs::msg::MoveItErrorCodes::CONTROL_FAILED),
                 "CONTROL_FAILED");
    EXPECT_STREQ(robross_painter::moveItCodeName(123456), "UNKNOWN");
}

TEST(ExecutorSafety, CanvasBoundsIncludeLandscapeEdges)
{
    EXPECT_TRUE(robross_painter::pointInsideCanvas(0.0, 0.0, 525.0, 375.0));
    EXPECT_TRUE(
        robross_painter::pointInsideCanvas(525.0, 375.0, 525.0, 375.0));
    EXPECT_TRUE(
        robross_painter::pointInsideCanvas(262.5, 187.5, 525.0, 375.0));
}

TEST(ExecutorSafety, CanvasBoundsRejectInvalidPointsAndDimensions)
{
    EXPECT_FALSE(
        robross_painter::pointInsideCanvas(-0.01, 100.0, 525.0, 375.0));
    EXPECT_FALSE(
        robross_painter::pointInsideCanvas(525.01, 100.0, 525.0, 375.0));
    EXPECT_FALSE(
        robross_painter::pointInsideCanvas(100.0, 375.01, 525.0, 375.0));
    EXPECT_FALSE(robross_painter::pointInsideCanvas(
        std::numeric_limits<double>::infinity(), 100.0, 525.0, 375.0));
    EXPECT_FALSE(
        robross_painter::pointInsideCanvas(100.0, 100.0, 0.0, 375.0));
}
