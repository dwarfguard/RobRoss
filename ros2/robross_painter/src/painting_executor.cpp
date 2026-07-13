// RobRoss painting executor: reads painting_paths.json and drives the Aubo i5
// through MoveIt. Canvas coordinates are millimeters, origin top-left,
// x right, y down (see RobRoss docs/painting-paths-format.md).
//
// Canvas -> robot mapping: the paper can be anywhere in space - flat on a
// table, taped to a wall, or on a slanted stand. The canvas frame has x
// along the page "right", y down the page, z pointing into the paper.
// Its pose in base_link comes from canvas_origin_xyz (top-left corner,
// meters, on the paper surface) plus either canvas_quat_xyzw (full 3D
// orientation, typically produced by teach_canvas.py from touched corners)
// or, when no quaternion is given, the legacy canvas_x_yaw_deg horizontal
// convention.
//
// The pen tip does not have to coincide with ee_link: tool_offset_xyz /
// tool_offset_rpy describe the pen-tip frame in ee_link (for the custom
// pen claw). All canvas targets are pen-tip poses; the executor converts
// them to ee_link poses before planning. The pen axis (tip frame z) is
// kept normal to the canvas; tool_spin_deg rotates the claw about it.

#include <cmath>
#include <fstream>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <json/json.h>

#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>
#include <visualization_msgs/msg/marker.hpp>

namespace {

struct CanvasFrame
{
    tf2::Vector3 origin;  // top-left corner in base_link (m), on the paper
    tf2::Matrix3x3 rot;   // columns: canvas x (right), y (down), z (into paper)

    // Legacy horizontal paper: canvas x direction given as a yaw in the base
    // XY plane, y is the horizontal perpendicular chosen so canvas z points
    // down into the table (top-left / y-down convention).
    void fromYaw(double ox, double oy, double oz, double yaw_rad)
    {
        origin = tf2::Vector3(ox, oy, oz);
        const tf2::Vector3 xc(std::cos(yaw_rad), std::sin(yaw_rad), 0.0);
        const tf2::Vector3 yc(std::sin(yaw_rad), -std::cos(yaw_rad), 0.0);
        const tf2::Vector3 zc = xc.cross(yc);
        rot = tf2::Matrix3x3(xc.x(), yc.x(), zc.x(),
                             xc.y(), yc.y(), zc.y(),
                             xc.z(), yc.z(), zc.z());
    }

    // Arbitrary plane (wall, slanted stand): full orientation, typically
    // taught with teach_canvas.py.
    void fromQuaternion(const tf2::Vector3 &o, const tf2::Quaternion &q)
    {
        origin = o;
        rot = tf2::Matrix3x3(q);
    }

    tf2::Vector3 axis(int i) const { return rot.getColumn(i); }

    tf2::Quaternion orientation() const
    {
        tf2::Quaternion q;
        rot.getRotation(q);
        return q;
    }

    // z_off: 0 = pen contact, >0 = hovering off the paper along -normal.
    tf2::Vector3 toBaseVec(double x_mm, double y_mm, double z_off) const
    {
        return origin + (x_mm / 1000.0) * axis(0) +
               (y_mm / 1000.0) * axis(1) - z_off * axis(2);
    }

    geometry_msgs::msg::Point toBase(double x_mm, double y_mm,
                                     double z_off) const
    {
        const tf2::Vector3 v = toBaseVec(x_mm, y_mm, z_off);
        geometry_msgs::msg::Point p;
        p.x = v.x();
        p.y = v.y();
        p.z = v.z();
        return p;
    }
};

class PaintingExecutor
{
public:
    PaintingExecutor(const rclcpp::Node::SharedPtr &node)
        : node_(node), group_(node, "manipulator")
    {
        marker_pub_ = node_->create_publisher<visualization_msgs::msg::Marker>(
            "robross_markers", rclcpp::QoS(10).transient_local());

        node_->get_parameter_or("paths_file", paths_file_, std::string());
        std::vector<double> origin{ 0.5985, 0.105, 0.15 };
        node_->get_parameter_or("canvas_origin_xyz", origin, origin);
        std::vector<double> quat;
        node_->get_parameter_or("canvas_quat_xyzw", quat, quat);
        if (quat.size() == 4) {
            tf2::Quaternion q(quat[0], quat[1], quat[2], quat[3]);
            q.normalize();
            canvas_.fromQuaternion(
                tf2::Vector3(origin.at(0), origin.at(1), origin.at(2)), q);
            RCLCPP_INFO(node_->get_logger(),
                        "Canvas pose from canvas_quat_xyzw (taught frame)");
        } else {
            double yaw_deg = -90.0;
            node_->get_parameter_or("canvas_x_yaw_deg", yaw_deg, yaw_deg);
            canvas_.fromYaw(origin.at(0), origin.at(1), origin.at(2),
                            yaw_deg * M_PI / 180.0);
            RCLCPP_INFO(node_->get_logger(),
                        "Canvas pose from canvas_x_yaw_deg (flat paper)");
        }

        node_->get_parameter_or("safe_clearance_m", safe_clearance_,
                                safe_clearance_);

        // Pen-tip orientation: pen axis (tip z) normal to the canvas,
        // tip x along the canvas x axis rotated by tool_spin_deg about
        // the pen axis (pick a spin that keeps the claw clear of the arm).
        double spin_deg = 0.0;
        node_->get_parameter_or("tool_spin_deg", spin_deg, spin_deg);
        tf2::Quaternion q_spin;
        q_spin.setRPY(0.0, 0.0, spin_deg * M_PI / 180.0);
        tip_orientation_ = canvas_.orientation() * q_spin;

        // Pen-tip frame in ee_link: where the custom claw holds the pen.
        std::vector<double> toff{ 0.0, 0.0, 0.0 };
        std::vector<double> torpy{ 0.0, 0.0, 0.0 };
        node_->get_parameter_or("tool_offset_xyz", toff, toff);
        node_->get_parameter_or("tool_offset_rpy", torpy, torpy);
        tf2::Quaternion q_off;
        q_off.setRPY(torpy.at(0), torpy.at(1), torpy.at(2));
        tool_offset_inv_ =
            tf2::Transform(q_off,
                           tf2::Vector3(toff.at(0), toff.at(1), toff.at(2)))
                .inverse();

        node_->get_parameter_or("ground_z_m", ground_z_, ground_z_);
        node_->get_parameter_or("canvas_backing_enabled", backing_enabled_,
                                backing_enabled_);
        node_->get_parameter_or("canvas_backing_clearance_m",
                                backing_clearance_, backing_clearance_);
        node_->get_parameter_or("claw_collision_size_xyz", claw_size_,
                                claw_size_);
        node_->get_parameter_or("claw_collision_offset_xyz", claw_offset_,
                                claw_offset_);
        node_->get_parameter_or("velocity_scaling", vel_scale_, vel_scale_);
        node_->get_parameter_or("acceleration_scaling", acc_scale_,
                                acc_scale_);
        node_->get_parameter_or("eef_step_m", eef_step_, eef_step_);
        node_->get_parameter_or("cartesian_jump_threshold", jump_threshold_,
                                jump_threshold_);
        node_->get_parameter_or("dry_run", dry_run_, dry_run_);

        group_.setMaxVelocityScalingFactor(vel_scale_);
        group_.setMaxAccelerationScalingFactor(acc_scale_);
    }

    bool run()
    {
        Json::Value root;
        if (!loadJson(root)) {
            return false;
        }

        canvas_w_mm_ = root["canvas"]["width_mm"].asDouble();
        canvas_h_mm_ = root["canvas"]["height_mm"].asDouble();
        if (!addGroundPlane() || !addCanvasBacking() || !attachClawBox()) {
            return false;
        }
        tool_width_mm_ = root["path_settings"]
                             .get("tool_width_mm", 1.0)
                             .asDouble();
        const Json::Value &commands = root["commands"];
        RCLCPP_INFO(node_->get_logger(),
                    "Loaded %s: canvas %.0fx%.0f mm, %d commands",
                    paths_file_.c_str(), canvas_w_mm_, canvas_h_mm_,
                    static_cast<int>(commands.size()));

        publishCanvasOutline();

        int index = 0;
        for (const auto &cmd : commands) {
            ++index;
            const std::string type = cmd["command"].asString();
            const std::string label = cmd.get("label", "").asString();
            RCLCPP_INFO(node_->get_logger(), "[%d/%d] %s (%s)", index,
                        static_cast<int>(commands.size()), type.c_str(),
                        label.c_str());

            bool ok = true;
            if (type == "select_tool" || type == "dip_paint") {
                // Pen demo v1: nothing to do, the pen is always mounted.
            } else if (type == "move_to") {
                ok = doMoveTo(cmd["x_mm"].asDouble(), cmd["y_mm"].asDouble());
            } else if (type == "lower_tool") {
                ok = doVertical(0.0);
            } else if (type == "lift_tool") {
                ok = doVertical(safe_clearance_);
            } else if (type == "paint_stroke") {
                ok = doStroke(cmd);
            } else if (type == "paint_path") {
                ok = doPath(cmd);
            } else {
                RCLCPP_WARN(node_->get_logger(),
                            "Unknown command '%s', skipping", type.c_str());
            }
            if (!ok) {
                RCLCPP_ERROR(node_->get_logger(),
                             "Command %d ('%s', label '%s') failed, aborting",
                             index, type.c_str(), label.c_str());
                return false;
            }
        }

        RCLCPP_INFO(node_->get_logger(), "Painting finished (%d commands)",
                    index);
        return true;
    }

private:
    // Insert a large flat box into the planning scene so that both
    // joint-space planning and Cartesian path collision checking refuse
    // trajectories where any arm link dips below the mounting surface.
    // The box top sits at ground_z_ (slightly below base_link z=0 by
    // default so the robot base itself is not flagged as colliding).
    bool addGroundPlane()
    {
        moveit_msgs::msg::CollisionObject obj;
        obj.header.frame_id = group_.getPlanningFrame();
        obj.id = "ground_plane";

        shape_msgs::msg::SolidPrimitive box;
        box.type = shape_msgs::msg::SolidPrimitive::BOX;
        constexpr double kSize = 4.0;      // side length, m
        constexpr double kThickness = 0.1; // m
        box.dimensions = { kSize, kSize, kThickness };

        geometry_msgs::msg::Pose pose;
        pose.orientation.w = 1.0;
        pose.position.z = ground_z_ - kThickness / 2.0;

        obj.primitives.push_back(box);
        obj.primitive_poses.push_back(pose);
        obj.operation = moveit_msgs::msg::CollisionObject::ADD;

        if (!scene_.applyCollisionObject(obj)) {
            RCLCPP_ERROR(node_->get_logger(),
                         "Failed to add ground plane to the planning scene, "
                         "refusing to run without it");
            return false;
        }
        RCLCPP_INFO(node_->get_logger(),
                    "Ground plane added at z=%.3f m in frame '%s'", ground_z_,
                    group_.getPlanningFrame().c_str());
        return true;
    }

    // Insert a large box just behind the canvas plane, oriented with the
    // canvas frame. On a wall-mounted paper it models the wall; on a stand
    // it models the board. Its front face sits canvas_backing_clearance_m
    // behind the drawing plane so pen contact itself is not a collision,
    // while any arm/claw link pushing past the paper is rejected.
    bool addCanvasBacking()
    {
        if (!backing_enabled_) {
            RCLCPP_WARN(node_->get_logger(),
                        "Canvas backing plane disabled; planning will not "
                        "protect the wall/board behind the paper");
            return true;
        }

        moveit_msgs::msg::CollisionObject obj;
        obj.header.frame_id = group_.getPlanningFrame();
        obj.id = "canvas_backing";

        shape_msgs::msg::SolidPrimitive box;
        box.type = shape_msgs::msg::SolidPrimitive::BOX;
        constexpr double kSize = 2.0;       // wall patch side length, m
        constexpr double kThickness = 0.05; // m
        box.dimensions = { kSize, kSize, kThickness };

        // Center of the box: canvas center, pushed behind the plane along
        // the canvas normal (+z points into the paper).
        const tf2::Vector3 center =
            canvas_.toBaseVec(canvas_w_mm_ / 2.0, canvas_h_mm_ / 2.0,
                              -(backing_clearance_ + kThickness / 2.0));
        const tf2::Quaternion q = canvas_.orientation();

        geometry_msgs::msg::Pose pose;
        pose.position.x = center.x();
        pose.position.y = center.y();
        pose.position.z = center.z();
        pose.orientation.x = q.x();
        pose.orientation.y = q.y();
        pose.orientation.z = q.z();
        pose.orientation.w = q.w();

        obj.primitives.push_back(box);
        obj.primitive_poses.push_back(pose);
        obj.operation = moveit_msgs::msg::CollisionObject::ADD;

        if (!scene_.applyCollisionObject(obj)) {
            RCLCPP_ERROR(node_->get_logger(),
                         "Failed to add canvas backing plane, refusing to "
                         "run without it");
            return false;
        }
        RCLCPP_INFO(node_->get_logger(),
                    "Canvas backing plane added %.1f mm behind the paper",
                    backing_clearance_ * 1000.0);
        return true;
    }

    // The custom pen claw is not part of the Aubo URDF, so MoveIt cannot
    // collision-check it. Attach a stand-in box to ee_link sized to
    // enclose the claw (claw_collision_size_xyz, centered at
    // claw_collision_offset_xyz in ee_link). Size [0,0,0] disables it.
    bool attachClawBox()
    {
        if (claw_size_.size() != 3 ||
            (claw_size_[0] <= 0.0 && claw_size_[1] <= 0.0 &&
             claw_size_[2] <= 0.0)) {
            RCLCPP_WARN(node_->get_logger(),
                        "No claw collision box configured; the claw is "
                        "invisible to collision checking");
            return true;
        }

        moveit_msgs::msg::AttachedCollisionObject aco;
        aco.link_name = group_.getEndEffectorLink();
        // Links the claw is allowed to touch: it is bolted to the flange,
        // so contact with the mounting link and wrist is not a collision.
        std::vector<std::string> touch{ aco.link_name, "wrist3_Link" };
        node_->get_parameter_or("claw_touch_links", touch, touch);
        aco.touch_links = touch;
        aco.object.header.frame_id = aco.link_name;
        aco.object.id = "pen_claw";

        shape_msgs::msg::SolidPrimitive box;
        box.type = shape_msgs::msg::SolidPrimitive::BOX;
        box.dimensions = { claw_size_[0], claw_size_[1], claw_size_[2] };

        geometry_msgs::msg::Pose pose;
        pose.orientation.w = 1.0;
        if (claw_offset_.size() == 3) {
            pose.position.x = claw_offset_[0];
            pose.position.y = claw_offset_[1];
            pose.position.z = claw_offset_[2];
        }

        aco.object.primitives.push_back(box);
        aco.object.primitive_poses.push_back(pose);
        aco.object.operation = moveit_msgs::msg::CollisionObject::ADD;

        if (!scene_.applyAttachedCollisionObject(aco)) {
            RCLCPP_ERROR(node_->get_logger(),
                         "Failed to attach claw collision box to '%s'",
                         aco.link_name.c_str());
            return false;
        }
        RCLCPP_INFO(node_->get_logger(),
                    "Claw collision box %.0fx%.0fx%.0f mm attached to '%s'",
                    claw_size_[0] * 1000.0, claw_size_[1] * 1000.0,
                    claw_size_[2] * 1000.0, aco.link_name.c_str());
        return true;
    }

    bool loadJson(Json::Value &root)
    {
        std::ifstream in(paths_file_);
        if (!in) {
            RCLCPP_ERROR(node_->get_logger(), "Cannot open paths_file '%s'",
                         paths_file_.c_str());
            return false;
        }
        Json::CharReaderBuilder builder;
        std::string errs;
        if (!Json::parseFromStream(builder, in, &root, &errs)) {
            RCLCPP_ERROR(node_->get_logger(), "JSON parse error: %s",
                         errs.c_str());
            return false;
        }
        if (!root.isMember("commands") || !root["commands"].isArray()) {
            RCLCPP_ERROR(node_->get_logger(),
                         "paths_file has no 'commands' array");
            return false;
        }
        return true;
    }

    bool checkBounds(double x_mm, double y_mm)
    {
        if (x_mm < 0.0 || x_mm > canvas_w_mm_ || y_mm < 0.0 ||
            y_mm > canvas_h_mm_) {
            RCLCPP_ERROR(node_->get_logger(),
                         "Point (%.2f, %.2f) mm outside canvas %.0fx%.0f mm",
                         x_mm, y_mm, canvas_w_mm_, canvas_h_mm_);
            return false;
        }
        return true;
    }

    // Desired pen-tip pose on the canvas, converted to the ee_link pose
    // MoveIt plans for: T_ee = T_tip * T_tool_offset^-1.
    geometry_msgs::msg::Pose makePose(double x_mm, double y_mm, double z_off)
    {
        const tf2::Transform t_tip(tip_orientation_,
                                   canvas_.toBaseVec(x_mm, y_mm, z_off));
        const tf2::Transform t_ee = t_tip * tool_offset_inv_;

        geometry_msgs::msg::Pose pose;
        pose.position.x = t_ee.getOrigin().x();
        pose.position.y = t_ee.getOrigin().y();
        pose.position.z = t_ee.getOrigin().z();
        const tf2::Quaternion q = t_ee.getRotation();
        pose.orientation.x = q.x();
        pose.orientation.y = q.y();
        pose.orientation.z = q.z();
        pose.orientation.w = q.w();
        return pose;
    }

    // Travel move above the paper. The very first motion uses joint-space
    // planning (the arm starts somewhere arbitrary); later travels are
    // straight Cartesian lines at safe height.
    bool doMoveTo(double x_mm, double y_mm)
    {
        if (!checkBounds(x_mm, y_mm)) {
            return false;
        }
        if (pen_down_) {
            RCLCPP_ERROR(node_->get_logger(),
                         "move_to while the pen is down, refusing");
            return false;
        }
        const auto target = makePose(x_mm, y_mm, safe_clearance_);
        bool ok;
        if (first_motion_) {
            ok = moveJointSpace(target);
            first_motion_ = false;
        } else {
            ok = moveCartesian({ target });
            if (!ok) {
                // The pen is up, so a collision-checked joint-space path
                // is a safe alternative when the straight line needs an
                // IK configuration change (or is blocked).
                RCLCPP_WARN(node_->get_logger(),
                            "Straight travel infeasible; replanning this "
                            "travel move in joint space");
                ok = moveJointSpace(target);
            }
        }
        if (ok) {
            cur_x_mm_ = x_mm;
            cur_y_mm_ = y_mm;
            have_position_ = true;
        }
        return ok;
    }

    // Straight vertical motion at the current canvas position.
    bool doVertical(double z_off)
    {
        if (!have_position_) {
            RCLCPP_ERROR(node_->get_logger(),
                         "lower/lift before any move_to, refusing");
            return false;
        }
        if (!moveCartesian({ makePose(cur_x_mm_, cur_y_mm_, z_off) })) {
            return false;
        }
        pen_down_ = (z_off == 0.0);
        return true;
    }

    bool doStroke(const Json::Value &cmd)
    {
        const auto &from = cmd["from_mm"];
        const auto &to = cmd["to_mm"];
        const double fx = from[0].asDouble(), fy = from[1].asDouble();
        const double tx = to[0].asDouble(), ty = to[1].asDouble();
        if (!checkBounds(fx, fy) || !checkBounds(tx, ty)) {
            return false;
        }
        if (!pen_down_) {
            RCLCPP_ERROR(node_->get_logger(),
                         "paint_stroke while the pen is up, refusing");
            return false;
        }
        if (std::hypot(fx - cur_x_mm_, fy - cur_y_mm_) > 0.5) {
            RCLCPP_WARN(node_->get_logger(),
                        "Stroke starts at (%.2f, %.2f) but pen is at "
                        "(%.2f, %.2f); dragging pen to the start point",
                        fx, fy, cur_x_mm_, cur_y_mm_);
            if (!moveCartesian({ makePose(fx, fy, 0.0) })) {
                return false;
            }
        }
        if (!moveCartesian({ makePose(tx, ty, 0.0) })) {
            return false;
        }
        cur_x_mm_ = tx;
        cur_y_mm_ = ty;
        publishStroke(fx, fy, tx, ty);
        return true;
    }

    // Continuous pen-down polyline: all segments are waypoints of ONE
    // Cartesian trajectory, retimed as a whole, so the pen draws through
    // the corners without stopping between segments. This is also how
    // curved lines will execute: a curve densely sampled into points_mm.
    bool doPath(const Json::Value &cmd)
    {
        const auto &pts = cmd["points_mm"];
        if (!pts.isArray() || pts.size() < 2) {
            RCLCPP_ERROR(node_->get_logger(),
                         "paint_path needs a points_mm array of >= 2 points");
            return false;
        }
        std::vector<std::pair<double, double>> points;
        points.reserve(pts.size());
        for (const auto &p : pts) {
            const double x = p[0].asDouble(), y = p[1].asDouble();
            if (!checkBounds(x, y)) {
                return false;
            }
            points.emplace_back(x, y);
        }
        if (!pen_down_) {
            RCLCPP_ERROR(node_->get_logger(),
                         "paint_path while the pen is up, refusing");
            return false;
        }

        std::vector<geometry_msgs::msg::Pose> waypoints;
        waypoints.reserve(points.size());
        if (std::hypot(points[0].first - cur_x_mm_,
                       points[0].second - cur_y_mm_) > 0.5) {
            RCLCPP_WARN(node_->get_logger(),
                        "Path starts at (%.2f, %.2f) but pen is at "
                        "(%.2f, %.2f); dragging pen to the start point",
                        points[0].first, points[0].second, cur_x_mm_,
                        cur_y_mm_);
            waypoints.push_back(
                makePose(points[0].first, points[0].second, 0.0));
        }
        for (std::size_t i = 1; i < points.size(); ++i) {
            waypoints.push_back(
                makePose(points[i].first, points[i].second, 0.0));
        }
        if (!moveCartesian(waypoints)) {
            return false;
        }
        for (std::size_t i = 1; i < points.size(); ++i) {
            publishStroke(points[i - 1].first, points[i - 1].second,
                          points[i].first, points[i].second);
        }
        cur_x_mm_ = points.back().first;
        cur_y_mm_ = points.back().second;
        return true;
    }

    bool moveJointSpace(const geometry_msgs::msg::Pose &target)
    {
        setDryRunStartState();
        group_.setPoseTarget(target);
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        if (group_.plan(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
            RCLCPP_ERROR(node_->get_logger(),
                         "Joint-space planning to approach pose failed");
            return false;
        }
        if (dry_run_) {
            advanceDryRunState(plan.trajectory_);
            return true;
        }
        return group_.execute(plan) == moveit::core::MoveItErrorCode::SUCCESS;
    }

    bool moveCartesian(const std::vector<geometry_msgs::msg::Pose> &waypoints)
    {
        setDryRunStartState();
        moveit_msgs::msg::RobotTrajectory traj;
        // The jump threshold is essential: the Cartesian interpolator only
        // collision-checks the sampled waypoint states. If IK flips arm
        // configuration between two samples, executing the trajectory
        // sweeps the arm through unchecked space (through itself, the
        // ground, or the wall). A nonzero threshold rejects such flips so
        // they surface as a planning failure instead of a dangerous motion.
        const double fraction = group_.computeCartesianPath(
            waypoints, eef_step_, jump_threshold_, traj);
        if (fraction < 0.999) {
            RCLCPP_ERROR(node_->get_logger(),
                         "Cartesian path only %.1f%% feasible (obstacle or "
                         "IK configuration flip)",
                         fraction * 100.0);
            return false;
        }
        // computeCartesianPath ignores the velocity scaling, so retime.
        // Build the start state from the trajectory itself instead of
        // getCurrentState(): the current-state monitor would need this node
        // spun externally, which conflicts with MoveGroupInterface's
        // internal executor.
        moveit::core::RobotState start_state(group_.getRobotModel());
        start_state.setToDefaultValues();
        const auto &jt = traj.joint_trajectory;
        if (!jt.points.empty()) {
            start_state.setVariablePositions(jt.joint_names,
                                             jt.points.front().positions);
        }
        robot_trajectory::RobotTrajectory rt(group_.getRobotModel(),
                                             group_.getName());
        rt.setRobotTrajectoryMsg(start_state, traj);
        trajectory_processing::TimeOptimalTrajectoryGeneration totg;
        if (!totg.computeTimeStamps(rt, vel_scale_, acc_scale_)) {
            RCLCPP_ERROR(node_->get_logger(), "Trajectory retiming failed");
            return false;
        }
        rt.getRobotTrajectoryMsg(traj);
        if (dry_run_) {
            advanceDryRunState(traj);
            return true;
        }
        return group_.execute(traj) == moveit::core::MoveItErrorCode::SUCCESS;
    }

    void setDryRunStartState()
    {
        if (dry_run_state_) {
            group_.setStartState(*dry_run_state_);
        }
    }

    void advanceDryRunState(const moveit_msgs::msg::RobotTrajectory &traj)
    {
        const auto &jt = traj.joint_trajectory;
        if (jt.points.empty()) {
            return;
        }
        if (!dry_run_state_) {
            dry_run_state_ =
                std::make_unique<moveit::core::RobotState>(group_.getRobotModel());
            dry_run_state_->setToDefaultValues();
        }
        dry_run_state_->setVariablePositions(jt.joint_names,
                                             jt.points.back().positions);
        dry_run_state_->update();
    }

    void publishCanvasOutline()
    {
        visualization_msgs::msg::Marker m;
        m.header.frame_id = group_.getPlanningFrame();
        m.header.stamp = node_->now();
        m.ns = "robross_canvas";
        m.id = 0;
        m.type = visualization_msgs::msg::Marker::LINE_STRIP;
        m.action = visualization_msgs::msg::Marker::ADD;
        m.scale.x = 0.002;
        m.color.r = m.color.g = m.color.b = 0.7;
        m.color.a = 1.0;
        m.pose.orientation.w = 1.0;
        const double corners[5][2] = { { 0, 0 },
                                       { canvas_w_mm_, 0 },
                                       { canvas_w_mm_, canvas_h_mm_ },
                                       { 0, canvas_h_mm_ },
                                       { 0, 0 } };
        for (const auto &c : corners) {
            m.points.push_back(canvas_.toBase(c[0], c[1], 0.0));
        }
        marker_pub_->publish(m);
    }

    void publishStroke(double fx, double fy, double tx, double ty)
    {
        strokes_.points.push_back(canvas_.toBase(fx, fy, 0.0));
        strokes_.points.push_back(canvas_.toBase(tx, ty, 0.0));
        strokes_.header.frame_id = group_.getPlanningFrame();
        strokes_.header.stamp = node_->now();
        strokes_.ns = "robross_strokes";
        strokes_.id = 1;
        strokes_.type = visualization_msgs::msg::Marker::LINE_LIST;
        strokes_.action = visualization_msgs::msg::Marker::ADD;
        strokes_.scale.x = tool_width_mm_ / 1000.0;
        strokes_.color.r = strokes_.color.g = strokes_.color.b = 0.0;
        strokes_.color.a = 1.0;
        strokes_.pose.orientation.w = 1.0;
        marker_pub_->publish(strokes_);
    }

    rclcpp::Node::SharedPtr node_;
    moveit::planning_interface::MoveGroupInterface group_;
    moveit::planning_interface::PlanningSceneInterface scene_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    visualization_msgs::msg::Marker strokes_;

    std::string paths_file_;
    CanvasFrame canvas_;
    tf2::Quaternion tip_orientation_;
    tf2::Transform tool_offset_inv_{ tf2::Quaternion::getIdentity() };
    double safe_clearance_{ 0.02 };
    double ground_z_{ -0.005 };
    bool backing_enabled_{ true };
    double backing_clearance_{ 0.005 };
    std::vector<double> claw_size_{ 0.0, 0.0, 0.0 };
    std::vector<double> claw_offset_{ 0.0, 0.0, 0.0 };
    double vel_scale_{ 0.3 };
    double acc_scale_{ 0.3 };
    double eef_step_{ 0.005 };
    double jump_threshold_{ 2.0 };
    bool dry_run_{ false };
    std::unique_ptr<moveit::core::RobotState> dry_run_state_;

    double canvas_w_mm_{ 0.0 };
    double canvas_h_mm_{ 0.0 };
    double tool_width_mm_{ 1.0 };

    bool first_motion_{ true };
    bool have_position_{ false };
    bool pen_down_{ false };
    double cur_x_mm_{ 0.0 };
    double cur_y_mm_{ 0.0 };
};

} // namespace

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>(
        "painting_executor",
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(
            true));

    // No external spinning: MoveGroupInterface (Humble) adds the node to its
    // own internal executor thread; a second executor would steal its action
    // responses ("unknown goal response, ignoring").
    bool ok = false;
    {
        PaintingExecutor painter(node);
        ok = painter.run();
    }

    rclcpp::shutdown();
    return ok ? 0 : 1;
}
