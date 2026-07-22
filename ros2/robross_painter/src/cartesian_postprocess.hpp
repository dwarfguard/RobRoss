#pragma once

#include <algorithm>
#include <limits>
#include <vector>

#include <Eigen/Core>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

namespace robross_painter
{

// Position-only trajectory points make ROS 2 Humble's spline-based
// joint_trajectory_controller fall back to LINEAR interpolation between
// samples — the same model validateCartesianPath checks. Leaving TOTG's
// velocities/accelerations in place would make the controller execute
// unvalidated quintic splines (remediation plan Section 2.3).
inline void stripDerivatives(trajectory_msgs::msg::JointTrajectory &jt)
{
    for (auto &point : jt.points) {
        point.velocities.clear();
        point.accelerations.clear();
        point.effort.clear();
    }
}

// Closest point on the polyline to `point`. Precondition: non-empty polyline.
inline Eigen::Vector3d closestPointOnPolyline(
    const Eigen::Vector3d &point,
    const std::vector<Eigen::Vector3d> &polyline)
{
    Eigen::Vector3d best = polyline.front();
    double best_dist_sq = (point - best).squaredNorm();
    for (std::size_t i = 1; i < polyline.size(); ++i) {
        const Eigen::Vector3d &start = polyline[i - 1];
        const Eigen::Vector3d segment = polyline[i] - start;
        const double length_sq = segment.squaredNorm();
        Eigen::Vector3d candidate = start;
        if (length_sq > std::numeric_limits<double>::epsilon()) {
            const double t = std::clamp(
                (point - start).dot(segment) / length_sq, 0.0, 1.0);
            candidate = start + t * segment;
        }
        const double dist_sq = (point - candidate).squaredNorm();
        if (dist_sq < best_dist_sq) {
            best_dist_sq = dist_sq;
            best = candidate;
        }
    }
    return best;
}

struct DeviationComponents
{
    double normal_signed;  // along `normal`; positive = same direction
    double tangential;     // remainder perpendicular to `normal`
};

inline DeviationComponents deviationComponents(
    const Eigen::Vector3d &deviation, const Eigen::Vector3d &normal)
{
    const double n = deviation.dot(normal);
    return { n, (deviation - n * normal).norm() };
}

}  // namespace robross_painter
