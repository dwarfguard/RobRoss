#pragma once

#include <algorithm>
#include <limits>

namespace robross_painter
{

struct CartesianCheckPolicy
{
    bool deviation_enabled = true;
    bool normal_deviation_enabled = true;
    bool orientation_deviation_enabled = true;
    double max_deviation_mm = 2.0;
    double max_normal_deviation_mm = 0.2;
    double max_orientation_deviation_deg = 2.0;
};

struct CartesianConformanceError
{
    double inward_normal_mm = 0.0;
    double outward_normal_mm = 0.0;
    double tangential_mm = 0.0;
    double orientation_deg = 0.0;
};

inline bool anyCartesianCheckEnabled(const CartesianCheckPolicy &policy)
{
    return policy.deviation_enabled || policy.normal_deviation_enabled ||
           policy.orientation_deviation_enabled;
}

inline bool cartesianConformanceAccepted(
    const CartesianConformanceError &error,
    const CartesianCheckPolicy &policy)
{
    return (!policy.normal_deviation_enabled ||
            (error.inward_normal_mm <= policy.max_normal_deviation_mm &&
             -error.outward_normal_mm <= policy.max_normal_deviation_mm)) &&
           (!policy.deviation_enabled ||
            error.tangential_mm <= policy.max_deviation_mm) &&
           (!policy.orientation_deviation_enabled ||
            error.orientation_deg <= policy.max_orientation_deviation_deg);
}

struct EndpointCheckPolicy
{
    bool joint_error_enabled = true;
    bool tip_error_enabled = true;
    bool tip_orientation_error_enabled = true;
    double max_joint_error_deg = 2.0;
    double max_tip_error_mm = 1.0;
    double max_tip_orientation_error_deg = 1.0;
};

struct EndpointConformanceError
{
    bool hard_limits_ok = false;
    double max_joint_deg = 0.0;
    double tip_mm = 0.0;
    double orient_deg = 0.0;
};

inline bool endpointConformanceAccepted(const EndpointConformanceError &error,
                                        const EndpointCheckPolicy &policy)
{
    return error.hard_limits_ok &&
           (!policy.joint_error_enabled ||
            error.max_joint_deg <= policy.max_joint_error_deg) &&
           (!policy.tip_error_enabled ||
            error.tip_mm <= policy.max_tip_error_mm) &&
           (!policy.tip_orientation_error_enabled ||
            error.orient_deg <= policy.max_tip_orientation_error_deg);
}

inline double endpointErrorRatio(const EndpointConformanceError &error,
                                 const EndpointCheckPolicy &policy)
{
    if (!error.hard_limits_ok) {
        return std::numeric_limits<double>::infinity();
    }
    double ratio = 0.0;
    if (policy.joint_error_enabled) {
        ratio = std::max(ratio, error.max_joint_deg / policy.max_joint_error_deg);
    }
    if (policy.tip_error_enabled) {
        ratio = std::max(ratio, error.tip_mm / policy.max_tip_error_mm);
    }
    if (policy.tip_orientation_error_enabled) {
        ratio = std::max(
            ratio,
            error.orient_deg / policy.max_tip_orientation_error_deg);
    }
    return ratio;
}

}  // namespace robross_painter
