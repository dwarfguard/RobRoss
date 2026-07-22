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

#include <algorithm>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <exception>
#include <fstream>
#include <future>
#include <limits>
#include <memory>
#include <mutex>
#include <stdexcept>
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
#include <moveit_msgs/msg/constraints.hpp>
#include <moveit_msgs/msg/joint_constraint.hpp>
#include <moveit_msgs/srv/get_state_validity.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>
#include <moveit_msgs/action/execute_trajectory.hpp>
#include <moveit_msgs/action/move_group.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>
#include <visualization_msgs/msg/marker.hpp>

#include "cartesian_postprocess.hpp"

namespace {

moveit::planning_interface::MoveGroupInterface
connectMoveGroup(const rclcpp::Node::SharedPtr &node)
{
    constexpr auto timeout = std::chrono::seconds(10);
    RCLCPP_INFO(node->get_logger(),
                "Waiting up to 10 seconds for MoveIt action servers");

    auto move_client =
        rclcpp_action::create_client<moveit_msgs::action::MoveGroup>(
            node, "/move_action");
    if (!move_client->wait_for_action_server(timeout)) {
        throw std::runtime_error(
            "MoveIt action server /move_action is unavailable; verify "
            "move_group is running and ROS_DOMAIN_ID/ROS_LOCALHOST_ONLY "
            "match this terminal");
    }

    auto execute_client =
        rclcpp_action::create_client<moveit_msgs::action::ExecuteTrajectory>(
            node, "/execute_trajectory");
    if (!execute_client->wait_for_action_server(timeout)) {
        throw std::runtime_error(
            "MoveIt action server /execute_trajectory is unavailable; "
            "verify move_group is running and ROS_DOMAIN_ID/"
            "ROS_LOCALHOST_ONLY match this terminal");
    }

    return moveit::planning_interface::MoveGroupInterface(
        node, "manipulator", std::shared_ptr<tf2_ros::Buffer>(),
        rclcpp::Duration::from_seconds(1.0));
}

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
        : node_(node), group_(connectMoveGroup(node))
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
        tool_offset_ = tf2::Transform(
            q_off, tf2::Vector3(toff.at(0), toff.at(1), toff.at(2)));
        tool_offset_inv_ = tool_offset_.inverse();

        node_->get_parameter_or("ground_enabled", ground_enabled_,
                                ground_enabled_);
        node_->get_parameter_or("ground_z_m", ground_z_, ground_z_);
        node_->get_parameter_or("canvas_backing_enabled", backing_enabled_,
                                backing_enabled_);
        node_->get_parameter_or("canvas_backing_clearance_m",
                                backing_clearance_, backing_clearance_);
        node_->get_parameter_or("canvas_backing_size_xy_m", backing_size_xy_,
                                backing_size_xy_);
        node_->get_parameter_or("canvas_backing_margin_m", backing_margin_,
                                backing_margin_);
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
        node_->get_parameter_or("elbow_up_enabled", elbow_up_enabled_,
                                elbow_up_enabled_);
        node_->get_parameter_or("elbow_joint", elbow_joint_, elbow_joint_);
        node_->get_parameter_or("elbow_up_min_deg", elbow_up_min_deg_,
                                elbow_up_min_deg_);
        node_->get_parameter_or("elbow_up_max_deg", elbow_up_max_deg_,
                                elbow_up_max_deg_);
        node_->get_parameter_or("guarded_joints", guarded_joints_,
                                guarded_joints_);
        node_->get_parameter_or("max_guarded_joint_goal_delta_deg",
                                max_guarded_joint_goal_delta_deg_,
                                max_guarded_joint_goal_delta_deg_);
        node_->get_parameter_or("max_guarded_joint_travel_deg",
                                max_guarded_joint_travel_deg_,
                                max_guarded_joint_travel_deg_);
        node_->get_parameter_or("max_guarded_joint_paint_travel_deg",
                                max_guarded_joint_paint_travel_deg_,
                                max_guarded_joint_paint_travel_deg_);
        node_->get_parameter_or("max_guarded_joint_step_deg",
                                max_guarded_joint_step_deg_,
                                max_guarded_joint_step_deg_);
        node_->get_parameter_or("max_cartesian_deviation_mm",
                                max_cartesian_deviation_mm_,
                                max_cartesian_deviation_mm_);
        node_->get_parameter_or("max_cartesian_normal_deviation_mm",
                                max_cartesian_normal_deviation_mm_,
                                max_cartesian_normal_deviation_mm_);
        node_->get_parameter_or("max_cartesian_orientation_deviation_deg",
                                max_cartesian_orientation_deviation_deg_,
                                max_cartesian_orientation_deviation_deg_);
        node_->get_parameter_or("max_execution_tip_error_mm",
                                max_execution_tip_error_mm_,
                                max_execution_tip_error_mm_);
        node_->get_parameter_or("max_execution_tip_orientation_error_deg",
                                max_execution_tip_orientation_error_deg_,
                                max_execution_tip_orientation_error_deg_);
        node_->get_parameter_or("totg_path_tolerance", totg_path_tolerance_,
                                totg_path_tolerance_);
        node_->get_parameter_or("controller_sample_dt", controller_sample_dt_,
                                controller_sample_dt_);
        node_->get_parameter_or("joint_states_topic", joint_states_topic_,
                                joint_states_topic_);
        node_->get_parameter_or("state_validity_service",
                                state_validity_service_,
                                state_validity_service_);
        node_->get_parameter_or("dry_run", dry_run_, dry_run_);

        group_.setMaxVelocityScalingFactor(vel_scale_);
        group_.setMaxAccelerationScalingFactor(acc_scale_);

        // MoveGroupInterface owns an executor for node_. Use a separate node
        // for joint feedback so no callback competes for action responses.
        state_node_ = std::make_shared<rclcpp::Node>(
            "painting_joint_state_monitor",
            rclcpp::NodeOptions().use_global_arguments(false));
        joint_state_sub_ =
            state_node_->create_subscription<sensor_msgs::msg::JointState>(
                joint_states_topic_, rclcpp::QoS(10),
                [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
                    if (msg->name.size() != msg->position.size() ||
                        !std::all_of(msg->position.begin(),
                                     msg->position.end(),
                                     [](double value) {
                                         return std::isfinite(value);
                                     })) {
                        RCLCPP_ERROR(node_->get_logger(),
                                     "Ignoring malformed/non-finite joint "
                                     "state feedback");
                        return;
                    }
                    {
                        std::lock_guard<std::mutex> lock(joint_state_mutex_);
                        joint_state_names_ = msg->name;
                        joint_state_positions_ = msg->position;
                        joint_state_received_at_ =
                            std::chrono::steady_clock::now();
                        ++joint_state_sequence_;
                        have_joint_state_ = true;
                    }
                    joint_state_cv_.notify_all();
                });
        state_validity_client_ =
            state_node_->create_client<moveit_msgs::srv::GetStateValidity>(
                state_validity_service_);
        state_executor_ =
            std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
        state_executor_->add_node(state_node_);
        state_thread_ = std::thread([this]() { state_executor_->spin(); });
    }

    ~PaintingExecutor()
    {
        if (state_executor_) {
            state_executor_->cancel();
        }
        if (state_thread_.joinable()) {
            state_thread_.join();
        }
    }

    bool run()
    {
        if (!initializeMotionPolicy()) {
            return false;
        }

        Json::Value root;
        if (!loadJson(root)) {
            return false;
        }

        canvas_w_mm_ = root["canvas"]["width_mm"].asDouble();
        canvas_h_mm_ = root["canvas"]["height_mm"].asDouble();
        const bool claw_enabled =
            std::any_of(claw_size_.begin(), claw_size_.end(),
                        [](double value) { return value != 0.0; });
        RCLCPP_INFO(node_->get_logger(),
                    "Planning scene config: ground=%s, backing=%s, claw=%s, "
                    "dry_run=%s",
                    ground_enabled_ ? "enabled" : "disabled",
                    backing_enabled_ ? "enabled" : "disabled",
                    claw_enabled ? "enabled" : "disabled",
                    dry_run_ ? "true" : "false");
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
                attemptRetreat();
                return false;
            }
        }

        if (pen_down_) {
            RCLCPP_ERROR(node_->get_logger(),
                         "Path ended with the pen down; retreating and "
                         "reporting failure");
            attemptRetreat();
            return false;
        }

        RCLCPP_INFO(node_->get_logger(), "Painting finished (%d commands)",
                    index);
        return true;
    }

private:
    bool removeWorldObjectIfPresent(const std::string &id,
                                    const char *description)
    {
        if (scene_.getObjects({ id }).empty()) {
            RCLCPP_INFO(node_->get_logger(), "%s already absent", description);
            return true;
        }

        moveit_msgs::msg::CollisionObject obj;
        obj.header.frame_id = group_.getPlanningFrame();
        obj.id = id;
        obj.operation = moveit_msgs::msg::CollisionObject::REMOVE;
        if (!scene_.applyCollisionObject(obj)) {
            RCLCPP_ERROR(node_->get_logger(), "Failed to remove %s",
                         description);
            return false;
        }
        RCLCPP_INFO(node_->get_logger(), "%s removed", description);
        return true;
    }

    bool removeAttachedObjectIfPresent(const std::string &id,
                                       const char *description)
    {
        const auto objects = scene_.getAttachedObjects({ id });
        const auto found = objects.find(id);
        if (found == objects.end()) {
            RCLCPP_INFO(node_->get_logger(), "%s already absent", description);
            return true;
        }

        moveit_msgs::msg::AttachedCollisionObject attached;
        attached.link_name = found->second.link_name;
        attached.object.id = id;
        attached.object.operation = moveit_msgs::msg::CollisionObject::REMOVE;
        if (!scene_.applyAttachedCollisionObject(attached)) {
            RCLCPP_ERROR(node_->get_logger(), "Failed to remove %s",
                         description);
            return false;
        }
        RCLCPP_INFO(node_->get_logger(), "%s removed", description);
        return true;
    }

    // Insert a large flat box into the planning scene so that both
    // joint-space planning and Cartesian path collision checking refuse
    // trajectories where any arm link dips below the mounting surface.
    // The box top sits at ground_z_ (slightly below base_link z=0 by
    // default so the robot base itself is not flagged as colliding).
    bool addGroundPlane()
    {
        if (!ground_enabled_) {
            if (!removeWorldObjectIfPresent("ground_plane", "Ground plane")) {
                return false;
            }
            RCLCPP_WARN(node_->get_logger(),
                        "Ground collision plane disabled and absent; "
                        "planning will not protect the mounting surface "
                        "below the robot");
            return true;
        }

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

    // Insert a box just behind the canvas plane, oriented with the canvas
    // frame. On a wall-mounted paper it models the wall; on a stand or the
    // ground it models the board under the paper. Its front face sits
    // canvas_backing_clearance_m behind the drawing plane so pen contact
    // itself is not a collision, while any arm/claw link pushing past the
    // paper is rejected. The patch size comes from canvas_backing_size_xy_m,
    // or the canvas plus canvas_backing_margin_m per side when unset.
    bool addCanvasBacking()
    {
        if (!backing_enabled_) {
            if (!removeWorldObjectIfPresent("canvas_backing",
                                            "Canvas backing plane")) {
                return false;
            }
            RCLCPP_WARN(node_->get_logger(),
                        "Canvas backing plane disabled and absent; planning "
                        "will not protect the wall/board behind the paper");
            return true;
        }

        moveit_msgs::msg::CollisionObject obj;
        obj.header.frame_id = group_.getPlanningFrame();
        obj.id = "canvas_backing";

        if (backing_size_xy_.size() != 2 || backing_size_xy_[0] < 0.0 ||
            backing_size_xy_[1] < 0.0) {
            RCLCPP_ERROR(node_->get_logger(),
                         "canvas_backing_size_xy_m must be two non-negative "
                         "values ([0, 0] auto-sizes to the canvas)");
            return false;
        }
        double size_x = backing_size_xy_[0];
        double size_y = backing_size_xy_[1];
        if (size_x == 0.0 && size_y == 0.0) {
            size_x = canvas_w_mm_ / 1000.0 + 2.0 * backing_margin_;
            size_y = canvas_h_mm_ / 1000.0 + 2.0 * backing_margin_;
        }

        shape_msgs::msg::SolidPrimitive box;
        box.type = shape_msgs::msg::SolidPrimitive::BOX;
        constexpr double kThickness = 0.05; // m
        box.dimensions = { size_x, size_y, kThickness };

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
                    "Canvas backing plane %.2fx%.2f m added %.1f mm behind "
                    "the paper",
                    size_x, size_y, backing_clearance_ * 1000.0);
        return true;
    }

    // The custom pen claw is not part of the Aubo URDF, so MoveIt cannot
    // collision-check it. Attach a stand-in box to ee_link sized to
    // enclose the claw (claw_collision_size_xyz, centered at
    // claw_collision_offset_xyz in ee_link). Size [0,0,0] disables it.
    bool attachClawBox()
    {
        if (claw_size_.size() != 3 || claw_offset_.size() != 3) {
            RCLCPP_ERROR(node_->get_logger(),
                         "claw_collision_size_xyz and offset_xyz must each "
                         "contain exactly three values");
            return false;
        }
        const bool disabled = claw_size_[0] == 0.0 && claw_size_[1] == 0.0 &&
                              claw_size_[2] == 0.0;
        if (disabled) {
            if (!removeAttachedObjectIfPresent("pen_claw",
                                               "Attached claw collision box")) {
                return false;
            }
            if (!removeWorldObjectIfPresent("pen_claw",
                                            "World claw collision box")) {
                return false;
            }
            RCLCPP_WARN(node_->get_logger(),
                        "No claw collision box configured; stale claw objects "
                        "were removed and the claw is invisible to collision "
                        "checking");
            return true;
        }
        if (!std::all_of(claw_size_.begin(), claw_size_.end(),
                         [](double value) {
                             return std::isfinite(value) && value > 0.0;
                         }) ||
            !std::all_of(claw_offset_.begin(), claw_offset_.end(),
                         [](double value) {
                             return std::isfinite(value);
                         })) {
            RCLCPP_ERROR(node_->get_logger(),
                         "Enabled claw collision dimensions must be finite "
                         "and positive, with a finite offset");
            return false;
        }

        // Contact-geometry sanity check: at pen contact the paper plane is
        // at z=0 in the pen-tip frame (tip z into the paper) and the
        // backing front face at +canvas_backing_clearance_m. If any box
        // corner reaches the backing at contact, every pen-down plan is
        // doomed to fail; refuse early with a clear message instead.
        double max_depth = -std::numeric_limits<double>::infinity();
        for (int corner = 0; corner < 8; ++corner) {
            const tf2::Vector3 p_ee(
                claw_offset_[0] + ((corner & 1) ? 0.5 : -0.5) * claw_size_[0],
                claw_offset_[1] + ((corner & 2) ? 0.5 : -0.5) * claw_size_[1],
                claw_offset_[2] + ((corner & 4) ? 0.5 : -0.5) * claw_size_[2]);
            max_depth = std::max(max_depth, (tool_offset_inv_ * p_ee).z());
        }
        if (backing_enabled_ && max_depth >= backing_clearance_) {
            RCLCPP_ERROR(node_->get_logger(),
                         "Claw collision box reaches %.1f mm past the paper "
                         "plane at pen contact, but the canvas backing sits "
                         "at %.1f mm: every pen-down plan would fail. Fix "
                         "claw_collision_size/offset, tool_offset_xyz, or "
                         "canvas_backing_clearance_m",
                         max_depth * 1000.0, backing_clearance_ * 1000.0);
            return false;
        }
        if (!backing_enabled_ && max_depth > 0.0) {
            RCLCPP_WARN(node_->get_logger(),
                        "Claw collision box models the claw %.1f mm past the "
                        "paper plane at pen contact; with the backing "
                        "disabled planning will NOT catch the real claw "
                        "hitting the wall",
                        max_depth * 1000.0);
        } else if (backing_enabled_ && backing_clearance_ - max_depth < 0.005) {
            RCLCPP_WARN(node_->get_logger(),
                        "Only %.1f mm between the claw collision box and the "
                        "canvas backing at pen contact; expect marginal "
                        "planning failures",
                        (backing_clearance_ - max_depth) * 1000.0);
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
        pose.position.x = claw_offset_[0];
        pose.position.y = claw_offset_[1];
        pose.position.z = claw_offset_[2];

        aco.object.primitives.push_back(box);
        aco.object.primitive_poses.push_back(pose);
        aco.object.operation = moveit_msgs::msg::CollisionObject::ADD;

        if (!removeWorldObjectIfPresent(aco.object.id,
                                        "World claw collision box")) {
            return false;
        }
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
        const auto finite_number = [](const Json::Value &value) {
            return value.isNumeric() && std::isfinite(value.asDouble());
        };
        const auto point = [&finite_number](const Json::Value &value) {
            return value.isArray() && value.size() == 2 &&
                   finite_number(value[0]) && finite_number(value[1]);
        };
        if (!root["canvas"].isObject() ||
            !finite_number(root["canvas"]["width_mm"]) ||
            !finite_number(root["canvas"]["height_mm"]) ||
            root["canvas"]["width_mm"].asDouble() <= 0.0 ||
            root["canvas"]["height_mm"].asDouble() <= 0.0) {
            RCLCPP_ERROR(node_->get_logger(),
                         "paths_file has invalid canvas dimensions");
            return false;
        }
        std::size_t index = 0;
        for (const auto &command : root["commands"]) {
            ++index;
            if (!command.isObject() || !command["command"].isString()) {
                RCLCPP_ERROR(node_->get_logger(),
                             "Command %zu has no string command type", index);
                return false;
            }
            const std::string type = command["command"].asString();
            bool valid = false;
            if (type == "select_tool" || type == "dip_paint" ||
                type == "lower_tool" || type == "lift_tool") {
                valid = true;
            } else if (type == "move_to") {
                valid = finite_number(command["x_mm"]) &&
                        finite_number(command["y_mm"]);
            } else if (type == "paint_stroke") {
                valid = point(command["from_mm"]) &&
                        point(command["to_mm"]);
            } else if (type == "paint_path") {
                const auto &points = command["points_mm"];
                valid = points.isArray() && points.size() >= 2;
                if (valid) {
                    for (const auto &path_point : points) {
                        if (!point(path_point)) {
                            valid = false;
                            break;
                        }
                    }
                }
            }
            if (!valid) {
                RCLCPP_ERROR(node_->get_logger(),
                             "Command %zu ('%s') has invalid or unsupported "
                             "fields",
                             index, type.c_str());
                return false;
            }
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
        if (!moveCartesian({ makePose(cur_x_mm_, cur_y_mm_, z_off) },
                           pen_down_ || z_off == 0.0)) {
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
            cur_x_mm_ = fx;
            cur_y_mm_ = fy;
        }
        if (!moveCartesian({ makePose(tx, ty, 0.0) })) {
            RCLCPP_ERROR(node_->get_logger(),
                         "Stroke rejected; refusing an automatic posture "
                         "change while painting");
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
            RCLCPP_ERROR(node_->get_logger(),
                         "Paint path rejected; refusing an automatic posture "
                         "change while painting");
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

    // Best-effort straight retreat after an abort with the pen on the paper.
    // A joint-space fallback is deliberately forbidden while in contact.
    void attemptRetreat()
    {
        if (dry_run_ || !pen_down_ || !have_position_) {
            return;
        }
        RCLCPP_WARN(node_->get_logger(),
                    "Aborting with the pen down; retreating off the paper");
        if (!refreshTrackedState()) {
            RCLCPP_ERROR(node_->get_logger(),
                         "Cannot read the current pose for straight retreat");
            return;
        }
        const auto &current = tracked_state_->getGlobalLinkTransform(
            group_.getEndEffectorLink());
        geometry_msgs::msg::Pose hover;
        const Eigen::Vector3d retreat =
            current.translation() -
            safe_clearance_ * Eigen::Vector3d(canvas_.axis(2).x(),
                                               canvas_.axis(2).y(),
                                               canvas_.axis(2).z());
        hover.position.x = retreat.x();
        hover.position.y = retreat.y();
        hover.position.z = retreat.z();
        const Eigen::Quaterniond orientation(current.rotation());
        hover.orientation.x = orientation.x();
        hover.orientation.y = orientation.y();
        hover.orientation.z = orientation.z();
        hover.orientation.w = orientation.w();
        if (moveCartesian({ hover })) {
            pen_down_ = false;
            RCLCPP_INFO(node_->get_logger(), "Pen retreated to hover height");
            return;
        }
        RCLCPP_ERROR(node_->get_logger(),
                     "Straight retreat failed: refusing a joint-space move "
                     "with the pen down. Jog it clear manually before the "
                     "next run");
    }

    bool initializeMotionPolicy()
    {
        if (!std::isfinite(safe_clearance_) || safe_clearance_ <= 0.0 ||
            !std::isfinite(vel_scale_) || vel_scale_ <= 0.0 ||
            vel_scale_ > 1.0 || !std::isfinite(acc_scale_) ||
            acc_scale_ <= 0.0 || acc_scale_ > 1.0 ||
            !std::isfinite(eef_step_) || eef_step_ <= 0.0 ||
            !std::isfinite(jump_threshold_) || jump_threshold_ <= 0.0 ||
            joint_states_topic_.empty() || state_validity_service_.empty()) {
            RCLCPP_ERROR(node_->get_logger(),
                         "Invalid motion settings: clearance, scaling, "
                         "eef_step, jump threshold, and joint-state topic "
                         "must be safe non-empty positive values");
            return false;
        }
        if (!std::isfinite(elbow_up_min_deg_) ||
            !std::isfinite(elbow_up_max_deg_) ||
            elbow_up_min_deg_ >= elbow_up_max_deg_) {
            RCLCPP_ERROR(node_->get_logger(), "Invalid elbow-up joint band");
            return false;
        }
        if (guarded_joints_.empty()) {
            RCLCPP_ERROR(node_->get_logger(),
                         "guarded_joints must contain at least one joint");
            return false;
        }
        if (!std::isfinite(max_guarded_joint_goal_delta_deg_) ||
            !std::isfinite(max_guarded_joint_travel_deg_) ||
            !std::isfinite(max_guarded_joint_paint_travel_deg_) ||
            !std::isfinite(max_guarded_joint_step_deg_) ||
            !std::isfinite(max_cartesian_deviation_mm_) ||
            !std::isfinite(max_cartesian_normal_deviation_mm_) ||
            !std::isfinite(max_cartesian_orientation_deviation_deg_) ||
            !std::isfinite(max_execution_tip_error_mm_) ||
            !std::isfinite(max_execution_tip_orientation_error_deg_) ||
            !std::isfinite(totg_path_tolerance_) ||
            !std::isfinite(controller_sample_dt_) ||
            max_guarded_joint_goal_delta_deg_ <= 0.0 ||
            max_guarded_joint_travel_deg_ <= 0.0 ||
            max_guarded_joint_paint_travel_deg_ <= 0.0 ||
            max_guarded_joint_step_deg_ <= 0.0 ||
            max_cartesian_deviation_mm_ <= 0.0 ||
            max_cartesian_normal_deviation_mm_ <= 0.0 ||
            max_cartesian_orientation_deviation_deg_ <= 0.0 ||
            max_execution_tip_error_mm_ <= 0.0 ||
            max_execution_tip_orientation_error_deg_ <= 0.0 ||
            totg_path_tolerance_ <= 0.0 || controller_sample_dt_ <= 0.0) {
            RCLCPP_ERROR(node_->get_logger(),
                         "All guarded-joint motion limits must be positive");
            return false;
        }

        const auto model = group_.getRobotModel();
        const auto *jmg = model->getJointModelGroup(group_.getName());
        if (!jmg) {
            RCLCPP_ERROR(node_->get_logger(), "Planning group '%s' not found",
                         group_.getName().c_str());
            return false;
        }
        const auto &group_variables = jmg->getVariableNames();
        if (elbow_up_enabled_ &&
            std::find(group_variables.begin(), group_variables.end(),
                      elbow_joint_) == group_variables.end()) {
            RCLCPP_ERROR(node_->get_logger(), "Elbow joint '%s' not found",
                         elbow_joint_.c_str());
            return false;
        }
        for (const auto &joint : guarded_joints_) {
            if (std::find(group_variables.begin(), group_variables.end(),
                          joint) == group_variables.end()) {
                RCLCPP_ERROR(node_->get_logger(), "Guarded joint '%s' not found",
                             joint.c_str());
                return false;
            }
        }

        if (!refreshTrackedState()) {
            RCLCPP_ERROR(node_->get_logger(),
                         "No complete current robot state; refusing to plan");
            return false;
        }
        if (elbow_up_enabled_ && !elbowInsideBand(*tracked_state_)) {
            const double elbow_deg =
                tracked_state_->getVariablePosition(elbow_joint_) * 180.0 / M_PI;
            RCLCPP_ERROR(node_->get_logger(),
                         "Current %s is %.1f deg, outside the required "
                         "elbow-up band [%.1f, %.1f]. Position the arm in "
                         "the approved posture before starting",
                         elbow_joint_.c_str(), elbow_deg, elbow_up_min_deg_,
                         elbow_up_max_deg_);
            return false;
        }
        RCLCPP_INFO(node_->get_logger(),
                    "Motion guard active for %zu joint(s): goal %.0f deg, "
                    "travel %.0f deg, paint travel %.0f deg, step %.0f deg",
                    guarded_joints_.size(), max_guarded_joint_goal_delta_deg_,
                    max_guarded_joint_travel_deg_,
                    max_guarded_joint_paint_travel_deg_,
                    max_guarded_joint_step_deg_);
        return true;
    }

    bool stateIsValid(const std::vector<std::string> &names,
                      const std::vector<double> &positions,
                      const char *what)
    {
        if (!state_validity_client_->wait_for_service(
                std::chrono::seconds(2))) {
            RCLCPP_ERROR(node_->get_logger(),
                         "State-validity service '%s' is unavailable",
                         state_validity_service_.c_str());
            return false;
        }
        auto request =
            std::make_shared<moveit_msgs::srv::GetStateValidity::Request>();
        request->group_name = group_.getName();
        request->robot_state.is_diff = true;
        request->robot_state.joint_state.name = names;
        request->robot_state.joint_state.position = positions;
        auto future = state_validity_client_->async_send_request(request);
        if (future.wait_for(std::chrono::seconds(2)) !=
            std::future_status::ready) {
            RCLCPP_ERROR(node_->get_logger(),
                         "%s state-validity check timed out", what);
            return false;
        }
        if (!future.get()->valid) {
            RCLCPP_ERROR(node_->get_logger(), "%s state is in collision",
                         what);
            return false;
        }
        return true;
    }

    bool validateCollisionFree(
        const moveit_msgs::msg::RobotTrajectory &traj)
    {
        const auto &jt = traj.joint_trajectory;
        constexpr double kMaxInterpolationStep = M_PI / 180.0;
        for (std::size_t segment = 0; segment < jt.points.size(); ++segment) {
            const auto &end = jt.points[segment].positions;
            const auto &start =
                segment == 0 ? end : jt.points[segment - 1].positions;
            double max_delta = 0.0;
            for (std::size_t joint = 0; joint < end.size(); ++joint) {
                max_delta =
                    std::max(max_delta, std::abs(end[joint] - start[joint]));
            }
            const int steps = std::max(
                1, static_cast<int>(std::ceil(max_delta /
                                               kMaxInterpolationStep)));
            for (int step = 1; step <= steps; ++step) {
                const double ratio = static_cast<double>(step) / steps;
                std::vector<double> positions(end.size());
                for (std::size_t joint = 0; joint < end.size(); ++joint) {
                    positions[joint] =
                        start[joint] + ratio * (end[joint] - start[joint]);
                }
                if (!stateIsValid(jt.joint_names, positions,
                                  "Retimed trajectory")) {
                    RCLCPP_ERROR(node_->get_logger(),
                                 "Collision at segment %zu/%zu, sample %d/%d",
                                 segment + 1, jt.points.size(), step, steps);
                    return false;
                }
            }
        }
        return true;
    }

    bool validateCartesianPath(
        const moveit_msgs::msg::RobotTrajectory &traj,
        const std::vector<geometry_msgs::msg::Pose> &waypoints) const
    {
        const auto &jt = traj.joint_trajectory;
        if (jt.points.empty() || waypoints.empty()) {
            RCLCPP_ERROR(node_->get_logger(),
                         "Cannot validate an empty Cartesian trajectory");
            return false;
        }

        moveit::core::RobotState state(group_.getRobotModel());
        state.setToDefaultValues();
        state.setVariablePositions(jt.joint_names,
                                   jt.points.front().positions);
        state.update();
        const std::string &eef = group_.getEndEffectorLink();
        const auto eigen_to_tf = [](const Eigen::Isometry3d &transform) {
            const Eigen::Quaterniond q(transform.rotation());
            return tf2::Transform(
                tf2::Quaternion(q.x(), q.y(), q.z(), q.w()),
                tf2::Vector3(transform.translation().x(),
                             transform.translation().y(),
                             transform.translation().z()));
        };
        const auto pose_to_tf = [](const geometry_msgs::msg::Pose &pose) {
            return tf2::Transform(
                tf2::Quaternion(pose.orientation.x, pose.orientation.y,
                                pose.orientation.z, pose.orientation.w),
                tf2::Vector3(pose.position.x, pose.position.y,
                             pose.position.z));
        };
        const auto tip_from_state = [&state, &eef, &eigen_to_tf, this]() {
            return eigen_to_tf(state.getGlobalLinkTransform(eef)) *
                   tool_offset_;
        };
        std::vector<Eigen::Vector3d> reference;
        reference.reserve(waypoints.size() + 1);
        const tf2::Transform initial_tip = tip_from_state();
        reference.emplace_back(initial_tip.getOrigin().x(),
                               initial_tip.getOrigin().y(),
                               initial_tip.getOrigin().z());
        for (const auto &waypoint : waypoints) {
            const tf2::Transform tip = pose_to_tf(waypoint) * tool_offset_;
            reference.emplace_back(tip.getOrigin().x(), tip.getOrigin().y(),
                                   tip.getOrigin().z());
        }

        const tf2::Vector3 normal_tf = canvas_.axis(2);
        const Eigen::Vector3d canvas_normal(normal_tf.x(), normal_tf.y(),
                                            normal_tf.z());

        const tf2::Quaternion desired_tip_q =
            (pose_to_tf(waypoints.front()) * tool_offset_).getRotation();
        Eigen::Quaterniond desired_orientation(
            desired_tip_q.w(), desired_tip_q.x(), desired_tip_q.y(),
            desired_tip_q.z());
        desired_orientation.normalize();
        // Signed canvas-normal deviation: positive = into the paper.
        double max_inward_normal = std::numeric_limits<double>::lowest();
        double min_outward_normal = std::numeric_limits<double>::max();
        double max_tangential_error = 0.0;
        double max_orientation_error = 0.0;
        constexpr double kMaxInterpolationStep = M_PI / 180.0;
        for (std::size_t segment = 0; segment < jt.points.size(); ++segment) {
            const auto &end = jt.points[segment].positions;
            const auto &start =
                segment == 0 ? end : jt.points[segment - 1].positions;
            double max_delta = 0.0;
            for (std::size_t joint = 0; joint < end.size(); ++joint) {
                max_delta =
                    std::max(max_delta, std::abs(end[joint] - start[joint]));
            }
            const int steps = std::max(
                1, static_cast<int>(std::ceil(max_delta /
                                               kMaxInterpolationStep)));
            for (int step = 1; step <= steps; ++step) {
                const double ratio = static_cast<double>(step) / steps;
                std::vector<double> positions(end.size());
                for (std::size_t joint = 0; joint < end.size(); ++joint) {
                    positions[joint] =
                        start[joint] + ratio * (end[joint] - start[joint]);
                }
                state.setVariablePositions(jt.joint_names, positions);
                state.update();
                const tf2::Transform actual_tip = tip_from_state();
                const Eigen::Vector3d actual(actual_tip.getOrigin().x(),
                                             actual_tip.getOrigin().y(),
                                             actual_tip.getOrigin().z());
                const Eigen::Vector3d closest =
                    robross_painter::closestPointOnPolyline(actual,
                                                            reference);
                const auto deviation = robross_painter::deviationComponents(
                    actual - closest, canvas_normal);
                max_inward_normal =
                    std::max(max_inward_normal, deviation.normal_signed);
                min_outward_normal =
                    std::min(min_outward_normal, deviation.normal_signed);
                max_tangential_error =
                    std::max(max_tangential_error, deviation.tangential);
                const tf2::Quaternion actual_q = actual_tip.getRotation();
                const Eigen::Quaterniond actual_orientation(
                    actual_q.w(), actual_q.x(), actual_q.y(), actual_q.z());
                max_orientation_error = std::max(
                    max_orientation_error,
                    desired_orientation.angularDistance(actual_orientation));
            }
        }

        const double inward_mm = max_inward_normal * 1000.0;
        const double outward_mm = min_outward_normal * 1000.0;
        const double tangential_mm = max_tangential_error * 1000.0;
        const double orientation_error_deg =
            max_orientation_error * 180.0 / M_PI;
        RCLCPP_INFO(node_->get_logger(),
                    "Cartesian FK error after retiming: normal %+.3f/%+.3f mm"
                    " (max into/out of paper), tangential %.3f mm, %.3f deg",
                    inward_mm, outward_mm, tangential_mm,
                    orientation_error_deg);
        if (inward_mm > max_cartesian_normal_deviation_mm_ ||
            -outward_mm > max_cartesian_normal_deviation_mm_ ||
            tangential_mm > max_cartesian_deviation_mm_ ||
            orientation_error_deg > max_cartesian_orientation_deviation_deg_) {
            RCLCPP_ERROR(node_->get_logger(),
                         "Retimed Cartesian path exceeds FK limits (normal "
                         "%.3f mm, tangential %.3f mm, %.3f deg)",
                         max_cartesian_normal_deviation_mm_,
                         max_cartesian_deviation_mm_,
                         max_cartesian_orientation_deviation_deg_);
            return false;
        }
        return true;
    }

    bool refreshTrackedState(std::uint64_t newer_than = 0,
                             bool require_newer = false)
    {
        if (dry_run_ && tracked_state_) {
            return true;
        }

        std::vector<std::string> names;
        std::vector<double> positions;
        {
            std::unique_lock<std::mutex> lock(joint_state_mutex_);
            const auto fresh = [this, newer_than, require_newer]() {
                return have_joint_state_ &&
                       (!require_newer ||
                        joint_state_sequence_ > newer_than) &&
                       std::chrono::steady_clock::now() -
                               joint_state_received_at_ <
                           std::chrono::seconds(2);
            };
            if (!fresh() &&
                !joint_state_cv_.wait_for(lock, std::chrono::seconds(5),
                                          fresh)) {
                return false;
            }
            names = joint_state_names_;
            positions = joint_state_positions_;
        }

        const auto *jmg =
            group_.getRobotModel()->getJointModelGroup(group_.getName());
        for (const auto &required : jmg->getVariableNames()) {
            if (std::find(names.begin(), names.end(), required) == names.end()) {
                RCLCPP_ERROR(node_->get_logger(),
                             "Joint state omits required joint '%s'",
                             required.c_str());
                return false;
            }
        }
        if (!tracked_state_) {
            tracked_state_ =
                std::make_unique<moveit::core::RobotState>(group_.getRobotModel());
            tracked_state_->setToDefaultValues();
        }
        try {
            tracked_state_->setVariablePositions(names, positions);
        } catch (const std::exception &error) {
            RCLCPP_ERROR(node_->get_logger(), "Invalid joint state: %s",
                         error.what());
            return false;
        }
        tracked_state_->update();
        if (!tracked_state_->satisfiesBounds(jmg)) {
            RCLCPP_ERROR(node_->get_logger(),
                         "Current joint feedback violates model bounds");
            return false;
        }
        return true;
    }

    std::uint64_t jointStateSequence()
    {
        std::lock_guard<std::mutex> lock(joint_state_mutex_);
        return joint_state_sequence_;
    }

    bool endpointMatches(
        const moveit_msgs::msg::RobotTrajectory &traj)
    {
        const auto &jt = traj.joint_trajectory;
        if (!tracked_state_ || jt.points.empty()) {
            return false;
        }
        constexpr double kEndpointToleranceDeg = 2.0;
        for (std::size_t i = 0; i < jt.joint_names.size(); ++i) {
            const double error_deg =
                std::abs(tracked_state_->getVariablePosition(jt.joint_names[i]) -
                         jt.points.back().positions.at(i)) *
                180.0 / M_PI;
            if (error_deg > kEndpointToleranceDeg) {
                RCLCPP_ERROR(node_->get_logger(),
                             "Executed endpoint for %s differs by %.2f deg "
                             "(limit %.2f deg)",
                             jt.joint_names[i].c_str(), error_deg,
                             kEndpointToleranceDeg);
                return false;
            }
        }
        const auto *jmg =
            group_.getRobotModel()->getJointModelGroup(group_.getName());
        if (!tracked_state_->satisfiesBounds(jmg) ||
            (elbow_up_enabled_ && !elbowInsideBand(*tracked_state_))) {
            RCLCPP_ERROR(node_->get_logger(),
                         "Measured endpoint violates joint or elbow bounds");
            return false;
        }

        moveit::core::RobotState expected(*tracked_state_);
        expected.setVariablePositions(jt.joint_names,
                                      jt.points.back().positions);
        expected.update();
        const auto to_tip = [this](const moveit::core::RobotState &state) {
            const auto &transform = state.getGlobalLinkTransform(
                group_.getEndEffectorLink());
            const Eigen::Quaterniond q(transform.rotation());
            return tf2::Transform(
                       tf2::Quaternion(q.x(), q.y(), q.z(), q.w()),
                       tf2::Vector3(transform.translation().x(),
                                    transform.translation().y(),
                                    transform.translation().z())) *
                   tool_offset_;
        };
        const tf2::Transform actual_tip = to_tip(*tracked_state_);
        const tf2::Transform expected_tip = to_tip(expected);
        const double tip_error_mm =
            (actual_tip.getOrigin() - expected_tip.getOrigin()).length() *
            1000.0;
        const double orientation_error_deg =
            actual_tip.getRotation().angleShortestPath(
                expected_tip.getRotation()) *
            180.0 / M_PI;
        if (tip_error_mm > max_execution_tip_error_mm_ ||
            orientation_error_deg > max_execution_tip_orientation_error_deg_) {
            RCLCPP_ERROR(node_->get_logger(),
                         "Measured pen-tip endpoint differs by %.3f mm and "
                         "%.3f deg (limits %.3f mm, %.3f deg)",
                         tip_error_mm, orientation_error_deg,
                         max_execution_tip_error_mm_,
                         max_execution_tip_orientation_error_deg_);
            return false;
        }
        std::vector<double> measured_positions;
        measured_positions.reserve(jt.joint_names.size());
        for (const auto &joint : jt.joint_names) {
            measured_positions.push_back(
                tracked_state_->getVariablePosition(joint));
        }
        if (!stateIsValid(jt.joint_names, measured_positions,
                          "Measured endpoint")) {
            return false;
        }
        return true;
    }

    bool elbowInsideBand(const moveit::core::RobotState &state) const
    {
        const double value_deg =
            state.getVariablePosition(elbow_joint_) * 180.0 / M_PI;
        constexpr double kToleranceDeg = 1e-6;
        return value_deg >= elbow_up_min_deg_ - kToleranceDeg &&
               value_deg <= elbow_up_max_deg_ + kToleranceDeg;
    }

    moveit_msgs::msg::Constraints elbowBandConstraints() const
    {
        const double lo = elbow_up_min_deg_ * M_PI / 180.0;
        const double hi = elbow_up_max_deg_ * M_PI / 180.0;
        moveit_msgs::msg::JointConstraint jc;
        jc.joint_name = elbow_joint_;
        jc.position = (lo + hi) / 2.0;
        jc.tolerance_above = hi - jc.position;
        jc.tolerance_below = jc.position - lo;
        jc.weight = 1.0;
        moveit_msgs::msg::Constraints constraints;
        constraints.name = "elbow_up";
        constraints.joint_constraints.push_back(jc);
        return constraints;
    }

    void setPlanStartState()
    {
        group_.setStartState(*tracked_state_);
    }

    bool computeIkJointGoal(const geometry_msgs::msg::Pose &target,
                            std::vector<double> &goal)
    {
        if (!refreshTrackedState()) {
            RCLCPP_ERROR(node_->get_logger(),
                         "Cannot refresh current state for IK seeding");
            return false;
        }
        const auto *jmg =
            group_.getRobotModel()->getJointModelGroup(group_.getName());
        std::vector<double> elbow_seeds{ 0.0 };
        if (elbow_up_enabled_) {
            const double lo = elbow_up_min_deg_ * M_PI / 180.0;
            const double hi = elbow_up_max_deg_ * M_PI / 180.0;
            elbow_seeds.front() =
                tracked_state_->getVariablePosition(elbow_joint_);
            elbow_seeds.push_back(lo + 0.25 * (hi - lo));
            elbow_seeds.push_back(lo + 0.50 * (hi - lo));
            elbow_seeds.push_back(lo + 0.75 * (hi - lo));
        }

        std::unique_ptr<moveit::core::RobotState> best;
        double best_cost = std::numeric_limits<double>::infinity();
        for (const double elbow_seed : elbow_seeds) {
            moveit::core::RobotState candidate(*tracked_state_);
            if (elbow_up_enabled_) {
                candidate.setVariablePosition(elbow_joint_, elbow_seed);
                candidate.update();
            }
            if (!candidate.setFromIK(jmg, target,
                                     group_.getEndEffectorLink(), 0.2)) {
                continue;
            }

            // Prefer the equivalent revolution nearest the measured state.
            // The Aubo model permits +/-2pi, so an IK solution can otherwise
            // be mathematically correct but almost a full turn away.
            double cost = 0.0;
            for (const auto &joint : jmg->getVariableNames()) {
                double value = candidate.getVariablePosition(joint);
                const double current =
                    tracked_state_->getVariablePosition(joint);
                const auto &bounds =
                    group_.getRobotModel()->getVariableBounds(joint);
                double nearest = value;
                double nearest_delta = std::abs(value - current);
                if (bounds.position_bounded_) {
                    for (int turn = -2; turn <= 2; ++turn) {
                        const double equivalent = value + turn * 2.0 * M_PI;
                        if (equivalent < bounds.min_position_ ||
                            equivalent > bounds.max_position_) {
                            continue;
                        }
                        const double delta = std::abs(equivalent - current);
                        if (delta < nearest_delta) {
                            nearest = equivalent;
                            nearest_delta = delta;
                        }
                    }
                }
                candidate.setVariablePosition(joint, nearest);
                cost += nearest_delta * nearest_delta;
            }
            candidate.update();
            if (elbow_up_enabled_ && !elbowInsideBand(candidate)) {
                continue;
            }
            if (cost < best_cost) {
                best =
                    std::make_unique<moveit::core::RobotState>(candidate);
                best_cost = cost;
            }
        }

        if (!best) {
            RCLCPP_ERROR(node_->get_logger(),
                         "No nearby IK solution in the elbow-up band");
            return false;
        }
        for (const auto &joint : guarded_joints_) {
            const double delta_deg =
                std::abs(best->getVariablePosition(joint) -
                         tracked_state_->getVariablePosition(joint)) *
                180.0 / M_PI;
            if (delta_deg > max_guarded_joint_goal_delta_deg_) {
                RCLCPP_ERROR(node_->get_logger(),
                             "IK goal moves %s by %.1f deg (limit %.1f deg)",
                             joint.c_str(), delta_deg,
                             max_guarded_joint_goal_delta_deg_);
                return false;
            }
        }
        best->copyJointGroupPositions(jmg, goal);
        return true;
    }

    bool validateTrajectory(const moveit_msgs::msg::RobotTrajectory &traj,
                            bool painting_motion, const char *what) const
    {
        const auto &jt = traj.joint_trajectory;
        if (jt.points.empty()) {
            RCLCPP_ERROR(node_->get_logger(), "%s trajectory is empty", what);
            return false;
        }
        for (const auto &point : jt.points) {
            if (point.positions.size() != jt.joint_names.size() ||
                !std::all_of(point.positions.begin(), point.positions.end(),
                             [](double value) {
                                 return std::isfinite(value);
                             })) {
                RCLCPP_ERROR(node_->get_logger(),
                             "%s trajectory contains malformed/non-finite "
                             "joint positions",
                             what);
                return false;
            }
        }

        if (elbow_up_enabled_) {
            const auto elbow_it =
                std::find(jt.joint_names.begin(), jt.joint_names.end(),
                          elbow_joint_);
            if (elbow_it == jt.joint_names.end()) {
                RCLCPP_ERROR(node_->get_logger(),
                             "%s trajectory omits elbow joint '%s'", what,
                             elbow_joint_.c_str());
                return false;
            }
            const auto elbow_index =
                static_cast<std::size_t>(elbow_it - jt.joint_names.begin());
            for (const auto &point : jt.points) {
                const double value_deg =
                    point.positions.at(elbow_index) * 180.0 / M_PI;
                if (value_deg < elbow_up_min_deg_ ||
                    value_deg > elbow_up_max_deg_) {
                    RCLCPP_ERROR(node_->get_logger(),
                                 "%s trajectory leaves elbow-up band at %.1f "
                                 "deg", what, value_deg);
                    return false;
                }
            }
        }

        const double travel_limit = painting_motion
                                        ? max_guarded_joint_paint_travel_deg_
                                        : max_guarded_joint_travel_deg_;
        for (const auto &joint : guarded_joints_) {
            const auto it = std::find(jt.joint_names.begin(),
                                      jt.joint_names.end(), joint);
            if (it == jt.joint_names.end()) {
                RCLCPP_ERROR(node_->get_logger(),
                             "%s trajectory omits guarded joint '%s'", what,
                             joint.c_str());
                return false;
            }
            const auto index =
                static_cast<std::size_t>(it - jt.joint_names.begin());
            double total = 0.0;
            double max_step = 0.0;
            for (std::size_t i = 1; i < jt.points.size(); ++i) {
                const double step =
                    std::abs(jt.points[i].positions.at(index) -
                             jt.points[i - 1].positions.at(index));
                total += step;
                max_step = std::max(max_step, step);
            }
            const double goal_delta =
                std::abs(jt.points.back().positions.at(index) -
                         jt.points.front().positions.at(index));
            const double total_deg = total * 180.0 / M_PI;
            const double step_deg = max_step * 180.0 / M_PI;
            const double goal_deg = goal_delta * 180.0 / M_PI;
            RCLCPP_INFO(node_->get_logger(),
                        "%s %s motion: goal %.1f deg, total %.1f deg, max "
                        "step %.1f deg", what, joint.c_str(), goal_deg,
                        total_deg, step_deg);
            if (goal_deg > max_guarded_joint_goal_delta_deg_ ||
                total_deg > travel_limit ||
                step_deg > max_guarded_joint_step_deg_) {
                RCLCPP_ERROR(node_->get_logger(),
                             "%s rejected for excessive %s motion (limits: "
                             "goal %.1f, total %.1f, step %.1f deg)",
                             what, joint.c_str(),
                             max_guarded_joint_goal_delta_deg_, travel_limit,
                             max_guarded_joint_step_deg_);
                return false;
            }
        }
        return true;
    }

    void trackTrajectoryEnd(const moveit_msgs::msg::RobotTrajectory &traj)
    {
        const auto &jt = traj.joint_trajectory;
        if (jt.points.empty()) {
            return;
        }
        if (!tracked_state_) {
            tracked_state_ =
                std::make_unique<moveit::core::RobotState>(group_.getRobotModel());
            tracked_state_->setToDefaultValues();
        }
        tracked_state_->setVariablePositions(jt.joint_names,
                                             jt.points.back().positions);
        tracked_state_->update();
    }

    bool executeTrajectory(const moveit_msgs::msg::RobotTrajectory &traj,
                           bool painting_motion, const char *what)
    {
        if (!validateTrajectory(traj, painting_motion, what)) {
            return false;
        }
        if (dry_run_) {
            trackTrajectoryEnd(traj);
            return true;
        }
        const auto execution_result = group_.execute(traj);
        const std::uint64_t sequence_at_completion = jointStateSequence();
        if (!refreshTrackedState(sequence_at_completion, true)) {
            RCLCPP_ERROR(node_->get_logger(),
                         "No fresh joint state received after execution");
            if (painting_motion) {
                pen_down_ = true;
            }
            return false;
        }
        if (execution_result != moveit::core::MoveItErrorCode::SUCCESS) {
            if (painting_motion) {
                pen_down_ = true;
            }
            RCLCPP_ERROR(node_->get_logger(),
                         "Execution failed; contact state is uncertain");
            return false;
        }
        const bool endpoint_ok = endpointMatches(traj);
        if (!endpoint_ok && painting_motion) {
            pen_down_ = true;
        }
        return endpoint_ok;
    }

    double trajectoryJointTravel(
        const moveit_msgs::msg::RobotTrajectory &traj) const
    {
        const auto &points = traj.joint_trajectory.points;
        double total = 0.0;
        for (std::size_t i = 1; i < points.size(); ++i) {
            for (std::size_t joint = 0;
                 joint < points[i].positions.size(); ++joint) {
                total += std::abs(points[i].positions[joint] -
                                  points[i - 1].positions[joint]);
            }
        }
        return total;
    }

    bool moveJointSpace(const geometry_msgs::msg::Pose &target)
    {
        std::vector<double> goal;
        if (!computeIkJointGoal(target, goal)) {
            return false;
        }
        setPlanStartState();
        group_.clearPoseTargets();
        group_.setJointValueTarget(goal);
        if (elbow_up_enabled_) {
            group_.setPathConstraints(elbowBandConstraints());
        }

        constexpr int kAttempts = 4;
        moveit::planning_interface::MoveGroupInterface::Plan best_plan;
        double best_travel = std::numeric_limits<double>::infinity();
        bool have_plan = false;
        for (int attempt = 1; attempt <= kAttempts; ++attempt) {
            moveit::planning_interface::MoveGroupInterface::Plan plan;
            if (group_.plan(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
                RCLCPP_WARN(node_->get_logger(),
                            "Bounded joint-space planning attempt %d/%d failed",
                            attempt, kAttempts);
                continue;
            }
            if (!validateTrajectory(plan.trajectory_, false,
                                    "Joint-space candidate")) {
                RCLCPP_WARN(node_->get_logger(),
                            "Joint-space candidate %d/%d exceeded motion "
                            "limits; replanning to the same bounded goal",
                            attempt, kAttempts);
                continue;
            }
            const double travel = trajectoryJointTravel(plan.trajectory_);
            if (travel < best_travel) {
                best_plan = plan;
                best_travel = travel;
                have_plan = true;
            }
        }
        group_.clearPathConstraints();
        if (!have_plan) {
            RCLCPP_ERROR(node_->get_logger(),
                         "No bounded elbow-up joint-space plan found");
            return false;
        }
        RCLCPP_INFO(node_->get_logger(),
                    "Selected shortest bounded joint-space candidate: %.1f "
                    "deg total joint travel",
                    best_travel * 180.0 / M_PI);
        return executeTrajectory(best_plan.trajectory_, false,
                                 "Joint-space");
    }

    bool moveCartesian(const std::vector<geometry_msgs::msg::Pose> &waypoints,
                       bool contact_motion = false)
    {
        if (!refreshTrackedState()) {
            RCLCPP_ERROR(node_->get_logger(),
                         "Cannot refresh current state for Cartesian planning");
            return false;
        }
        setPlanStartState();
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
        const bool painting_motion = contact_motion || pen_down_;
        if (!validateTrajectory(traj, painting_motion, "Cartesian IK")) {
            return false;
        }
        // computeCartesianPath ignores the velocity scaling, so retime.
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
        // Resample at the controller period: TOTG's own uniform resampling
        // emits exact on-profile positions every controller_sample_dt_, so
        // the linear interpolation the controller falls back to (below) is
        // validated against samples it will actually execute between.
        trajectory_processing::TimeOptimalTrajectoryGeneration totg(
            totg_path_tolerance_, controller_sample_dt_);
        if (!totg.computeTimeStamps(rt, vel_scale_, acc_scale_)) {
            RCLCPP_ERROR(node_->get_logger(), "Trajectory retiming failed");
            return false;
        }
        rt.getRobotTrajectoryMsg(traj);
        // Strip derivatives BEFORE validation and execution so every check
        // below sees exactly the message the controller receives, and the
        // Humble spline controller interpolates positions linearly instead
        // of executing unvalidated quintic splines (remediation plan
        // Section 2.3). Joint-space travel keeps its derivatives.
        robross_painter::stripDerivatives(traj.joint_trajectory);
        if (!validateCartesianPath(traj, waypoints)) {
            return false;
        }
        if (!validateCollisionFree(traj)) {
            return false;
        }
        return executeTrajectory(traj, painting_motion, "Cartesian");
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
    rclcpp::Node::SharedPtr state_node_;
    moveit::planning_interface::MoveGroupInterface group_;
    moveit::planning_interface::PlanningSceneInterface scene_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr
        joint_state_sub_;
    rclcpp::Client<moveit_msgs::srv::GetStateValidity>::SharedPtr
        state_validity_client_;
    std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> state_executor_;
    std::thread state_thread_;
    visualization_msgs::msg::Marker strokes_;
    std::mutex joint_state_mutex_;
    std::condition_variable joint_state_cv_;
    std::vector<std::string> joint_state_names_;
    std::vector<double> joint_state_positions_;
    std::chrono::steady_clock::time_point joint_state_received_at_;
    std::uint64_t joint_state_sequence_{ 0 };
    bool have_joint_state_{ false };

    std::string paths_file_;
    std::string joint_states_topic_{ "/joint_states" };
    std::string state_validity_service_{ "/check_state_validity" };
    CanvasFrame canvas_;
    tf2::Quaternion tip_orientation_;
    tf2::Transform tool_offset_{ tf2::Quaternion::getIdentity() };
    tf2::Transform tool_offset_inv_{ tf2::Quaternion::getIdentity() };
    double safe_clearance_{ 0.02 };
    bool ground_enabled_{ true };
    double ground_z_{ -0.005 };
    bool backing_enabled_{ true };
    double backing_clearance_{ 0.005 };
    // Backing patch size in the canvas plane; [0,0] auto-sizes it to the
    // canvas plus canvas_backing_margin_m on every side.
    std::vector<double> backing_size_xy_{ 0.0, 0.0 };
    double backing_margin_{ 0.05 };
    std::vector<double> claw_size_{ 0.0, 0.0, 0.0 };
    std::vector<double> claw_offset_{ 0.0, 0.0, 0.0 };
    double vel_scale_{ 0.3 };
    double acc_scale_{ 0.3 };
    double eef_step_{ 0.005 };
    double jump_threshold_{ 2.0 };
    // The elbow family is a hard invariant for startup, joint-space plans,
    // and Cartesian trajectories. The executor never changes family
    // automatically.
    bool elbow_up_enabled_{ true };
    std::string elbow_joint_{ "foreArm_joint" };
    double elbow_up_min_deg_{ -5.0 };
    double elbow_up_max_deg_{ 175.0 };
    std::vector<std::string> guarded_joints_{ "shoulder_joint",
                                              "wrist3_joint" };
    double max_guarded_joint_goal_delta_deg_{ 120.0 };
    double max_guarded_joint_travel_deg_{ 150.0 };
    double max_guarded_joint_paint_travel_deg_{ 90.0 };
    double max_guarded_joint_step_deg_{ 45.0 };
    double max_cartesian_deviation_mm_{ 2.0 };
    double max_cartesian_normal_deviation_mm_{ 0.2 };
    double max_cartesian_orientation_deviation_deg_{ 2.0 };
    double max_execution_tip_error_mm_{ 1.0 };
    double max_execution_tip_orientation_error_deg_{ 1.0 };
    double totg_path_tolerance_{ 0.01 };
    double controller_sample_dt_{ 0.005 };
    bool dry_run_{ false };
    std::unique_ptr<moveit::core::RobotState> tracked_state_;

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
    try {
        PaintingExecutor painter(node);
        ok = painter.run();
    } catch (const std::exception &error) {
        RCLCPP_FATAL(node->get_logger(), "Painting executor startup failed: %s",
                     error.what());
    }

    rclcpp::shutdown();
    return ok ? 0 : 1;
}
