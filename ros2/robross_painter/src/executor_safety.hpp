#pragma once

#include <cstdint>
#include <string>

namespace robross_painter
{

enum class MotionKind
{
    Transit,
    Lowering,
    Painting,
    Lifting
};

enum class MotionStatus
{
    Succeeded,
    PreSubmissionFailed,
    ExecutionFailed,
    PostExecutionFailed
};

struct MotionOutcome
{
    MotionKind kind{ MotionKind::Transit };
    MotionStatus status{ MotionStatus::PreSubmissionFailed };
    bool execution_started{ false };
    bool joint_fallback_eligible{ false };
    bool stop_requested{ false };
    std::int32_t moveit_code{ 0 };
    std::string correlation_id;
    std::string detail;

    bool succeeded() const { return status == MotionStatus::Succeeded; }
};

bool jointFallbackAllowed(const MotionOutcome &outcome);
const char *motionKindName(MotionKind kind);
const char *motionStatusName(MotionStatus status);
const char *moveItCodeName(std::int32_t code);

enum class DriverHealthAssessment
{
    Ready,
    Missing,
    NotReady,
    Faulted,
    WritesDisabled,
    Stale,
    NonFinite
};

struct DriverHealthSample
{
    bool received{ false };
    std::uint8_t state{ 0 };
    std::uint16_t fault_code{ 0 };
    bool writes_enabled{ false };
    bool finite{ false };
    double source_age_sec{ 0.0 };
    double callback_age_sec{ 0.0 };
    double receipt_age_sec{ 0.0 };
    double stamp_age_sec{ 0.0 };
};

DriverHealthAssessment assessDriverHealth(const DriverHealthSample &sample,
                                          std::uint8_t ready_state,
                                          double max_age_sec);
const char *driverHealthAssessmentName(DriverHealthAssessment assessment);

}  // namespace robross_painter
