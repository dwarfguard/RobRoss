#include "executor_safety.hpp"

#include <cmath>

#include <moveit_msgs/msg/move_it_error_codes.hpp>

namespace robross_painter
{

bool jointFallbackAllowed(const MotionOutcome &outcome)
{
    return outcome.status == MotionStatus::PreSubmissionFailed &&
           !outcome.execution_started && outcome.joint_fallback_eligible;
}

bool automaticPaintRecoveryAllowed(bool enabled, const std::string &command,
                                   const MotionOutcome &outcome,
                                   bool retreat_succeeded, bool jump_limited,
                                   unsigned int attempts)
{
    return enabled && command == "paint_path" &&
           outcome.kind == MotionKind::Painting &&
           outcome.status == MotionStatus::PreSubmissionFailed &&
           !outcome.execution_started && retreat_succeeded && jump_limited &&
           attempts == 0;
}

bool pointInsideCanvas(double x_mm, double y_mm, double width_mm,
                       double height_mm)
{
    return std::isfinite(x_mm) && std::isfinite(y_mm) &&
           std::isfinite(width_mm) && std::isfinite(height_mm) &&
           width_mm > 0.0 && height_mm > 0.0 && x_mm >= 0.0 &&
           x_mm <= width_mm && y_mm >= 0.0 && y_mm <= height_mm;
}

const char *motionKindName(MotionKind kind)
{
    switch (kind) {
    case MotionKind::Transit:
        return "transit";
    case MotionKind::Lowering:
        return "lowering";
    case MotionKind::Painting:
        return "painting";
    case MotionKind::Lifting:
        return "lifting";
    }
    return "unknown";
}

const char *motionStatusName(MotionStatus status)
{
    switch (status) {
    case MotionStatus::Succeeded:
        return "succeeded";
    case MotionStatus::PreSubmissionFailed:
        return "pre_submission_failed";
    case MotionStatus::ExecutionFailed:
        return "execution_failed";
    case MotionStatus::PostExecutionFailed:
        return "post_execution_failed";
    }
    return "unknown";
}

const char *moveItCodeName(std::int32_t code)
{
    using Codes = moveit_msgs::msg::MoveItErrorCodes;
    switch (code) {
    case Codes::SUCCESS:
        return "SUCCESS";
    case Codes::FAILURE:
        return "FAILURE";
    case Codes::PLANNING_FAILED:
        return "PLANNING_FAILED";
    case Codes::INVALID_MOTION_PLAN:
        return "INVALID_MOTION_PLAN";
    case Codes::MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE:
        return "MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE";
    case Codes::CONTROL_FAILED:
        return "CONTROL_FAILED";
    case Codes::UNABLE_TO_AQUIRE_SENSOR_DATA:
        return "UNABLE_TO_AQUIRE_SENSOR_DATA";
    case Codes::TIMED_OUT:
        return "TIMED_OUT";
    case Codes::PREEMPTED:
        return "PREEMPTED";
    case Codes::START_STATE_IN_COLLISION:
        return "START_STATE_IN_COLLISION";
    case Codes::START_STATE_VIOLATES_PATH_CONSTRAINTS:
        return "START_STATE_VIOLATES_PATH_CONSTRAINTS";
    case Codes::START_STATE_INVALID:
        return "START_STATE_INVALID";
    case Codes::GOAL_IN_COLLISION:
        return "GOAL_IN_COLLISION";
    case Codes::GOAL_VIOLATES_PATH_CONSTRAINTS:
        return "GOAL_VIOLATES_PATH_CONSTRAINTS";
    case Codes::GOAL_CONSTRAINTS_VIOLATED:
        return "GOAL_CONSTRAINTS_VIOLATED";
    case Codes::GOAL_STATE_INVALID:
        return "GOAL_STATE_INVALID";
    case Codes::UNRECOGNIZED_GOAL_TYPE:
        return "UNRECOGNIZED_GOAL_TYPE";
    case Codes::INVALID_GROUP_NAME:
        return "INVALID_GROUP_NAME";
    case Codes::INVALID_GOAL_CONSTRAINTS:
        return "INVALID_GOAL_CONSTRAINTS";
    case Codes::INVALID_ROBOT_STATE:
        return "INVALID_ROBOT_STATE";
    case Codes::INVALID_LINK_NAME:
        return "INVALID_LINK_NAME";
    case Codes::INVALID_OBJECT_NAME:
        return "INVALID_OBJECT_NAME";
    case Codes::FRAME_TRANSFORM_FAILURE:
        return "FRAME_TRANSFORM_FAILURE";
    case Codes::COLLISION_CHECKING_UNAVAILABLE:
        return "COLLISION_CHECKING_UNAVAILABLE";
    case Codes::ROBOT_STATE_STALE:
        return "ROBOT_STATE_STALE";
    case Codes::SENSOR_INFO_STALE:
        return "SENSOR_INFO_STALE";
    case Codes::COMMUNICATION_FAILURE:
        return "COMMUNICATION_FAILURE";
    case Codes::CRASH:
        return "CRASH";
    case Codes::ABORT:
        return "ABORT";
    case Codes::NO_IK_SOLUTION:
        return "NO_IK_SOLUTION";
    default:
        return "UNKNOWN";
    }
}

DriverHealthAssessment assessDriverHealth(const DriverHealthSample &sample,
                                          std::uint8_t ready_state,
                                          double max_age_sec)
{
    if (!sample.received) {
        return DriverHealthAssessment::Missing;
    }
    if (!sample.finite || !std::isfinite(sample.source_age_sec) ||
        !std::isfinite(sample.callback_age_sec) ||
        !std::isfinite(sample.receipt_age_sec) ||
        !std::isfinite(sample.stamp_age_sec)) {
        return DriverHealthAssessment::NonFinite;
    }
    if (sample.fault_code != 0) {
        return DriverHealthAssessment::Faulted;
    }
    if (sample.state != ready_state) {
        return DriverHealthAssessment::NotReady;
    }
    if (!sample.writes_enabled) {
        return DriverHealthAssessment::WritesDisabled;
    }
    if (max_age_sec <= 0.0 || sample.source_age_sec > max_age_sec ||
        sample.callback_age_sec > max_age_sec ||
        sample.receipt_age_sec > max_age_sec || sample.stamp_age_sec < 0.0 ||
        sample.stamp_age_sec > max_age_sec) {
        return DriverHealthAssessment::Stale;
    }
    return DriverHealthAssessment::Ready;
}

const char *driverHealthAssessmentName(DriverHealthAssessment assessment)
{
    switch (assessment) {
    case DriverHealthAssessment::Ready:
        return "ready";
    case DriverHealthAssessment::Missing:
        return "missing";
    case DriverHealthAssessment::NotReady:
        return "not_ready";
    case DriverHealthAssessment::Faulted:
        return "faulted";
    case DriverHealthAssessment::WritesDisabled:
        return "writes_disabled";
    case DriverHealthAssessment::Stale:
        return "stale";
    case DriverHealthAssessment::NonFinite:
        return "non_finite";
    }
    return "unknown";
}

}  // namespace robross_painter
