// RobRoss painting executor: reads painting_paths.json and drives the Aubo i5
// through MoveIt. Canvas coordinates are millimeters, origin top-left,
// x right, y down (see RobRoss docs/painting-paths-format.md).
//
// Canvas -> robot mapping: the paper lies flat (horizontal). The pose of the
// canvas top-left corner in base_link is given by canvas_origin_xyz (meters,
// z = pen contact height) and canvas_x_yaw_deg (direction of the canvas x
// axis in the base XY plane). The canvas y axis is the horizontal
// perpendicular chosen so canvas z points into the table.

#include <cmath>
#include <fstream>
#include <string>
#include <thread>
#include <vector>

#include <json/json.h>

#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <visualization_msgs/msg/marker.hpp>

namespace {

struct CanvasFrame
{
    double ox, oy, oz;    // top-left corner in base_link (m), oz = contact z
    double xdir_x, xdir_y;// unit vector of canvas x axis in base XY plane
    double ydir_x, ydir_y;// unit vector of canvas y axis in base XY plane

    void fromYaw(double ox_, double oy_, double oz_, double yaw_rad)
    {
        ox = ox_;
        oy = oy_;
        oz = oz_;
        xdir_x = std::cos(yaw_rad);
        xdir_y = std::sin(yaw_rad);
        // Horizontal perpendicular such that x_c cross y_c points down
        // (into the table), matching the top-left / y-down convention.
        ydir_x = std::sin(yaw_rad);
        ydir_y = -std::cos(yaw_rad);
    }

    // z_off: 0 = pen contact, >0 = above the paper.
    geometry_msgs::msg::Point toBase(double x_mm, double y_mm,
                                     double z_off) const
    {
        const double x_m = x_mm / 1000.0;
        const double y_m = y_mm / 1000.0;
        geometry_msgs::msg::Point p;
        p.x = ox + x_m * xdir_x + y_m * ydir_x;
        p.y = oy + x_m * xdir_y + y_m * ydir_y;
        p.z = oz + z_off;
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
        double yaw_deg = -90.0;
        node_->get_parameter_or("canvas_x_yaw_deg", yaw_deg, yaw_deg);
        canvas_.fromYaw(origin.at(0), origin.at(1), origin.at(2),
                        yaw_deg * M_PI / 180.0);

        node_->get_parameter_or("safe_clearance_m", safe_clearance_,
                                safe_clearance_);
        std::vector<double> rpy{ M_PI, 0.0, 0.0 };
        node_->get_parameter_or("tool_rpy", rpy, rpy);
        tf2::Quaternion q;
        q.setRPY(rpy.at(0), rpy.at(1), rpy.at(2));
        tool_orientation_.x = q.x();
        tool_orientation_.y = q.y();
        tool_orientation_.z = q.z();
        tool_orientation_.w = q.w();

        node_->get_parameter_or("velocity_scaling", vel_scale_, vel_scale_);
        node_->get_parameter_or("acceleration_scaling", acc_scale_,
                                acc_scale_);
        node_->get_parameter_or("eef_step_m", eef_step_, eef_step_);
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

    geometry_msgs::msg::Pose makePose(double x_mm, double y_mm, double z_off)
    {
        geometry_msgs::msg::Pose pose;
        pose.position = canvas_.toBase(x_mm, y_mm, z_off);
        pose.orientation = tool_orientation_;
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

    bool moveJointSpace(const geometry_msgs::msg::Pose &target)
    {
        group_.setPoseTarget(target);
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        if (group_.plan(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
            RCLCPP_ERROR(node_->get_logger(),
                         "Joint-space planning to approach pose failed");
            return false;
        }
        if (dry_run_) {
            return true;
        }
        return group_.execute(plan) == moveit::core::MoveItErrorCode::SUCCESS;
    }

    bool moveCartesian(const std::vector<geometry_msgs::msg::Pose> &waypoints)
    {
        moveit_msgs::msg::RobotTrajectory traj;
        const double fraction =
            group_.computeCartesianPath(waypoints, eef_step_, 0.0, traj);
        if (fraction < 0.999) {
            RCLCPP_ERROR(node_->get_logger(),
                         "Cartesian path only %.1f%% feasible",
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
            return true;
        }
        return group_.execute(traj) == moveit::core::MoveItErrorCode::SUCCESS;
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
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    visualization_msgs::msg::Marker strokes_;

    std::string paths_file_;
    CanvasFrame canvas_;
    geometry_msgs::msg::Quaternion tool_orientation_;
    double safe_clearance_{ 0.02 };
    double vel_scale_{ 0.3 };
    double acc_scale_{ 0.3 };
    double eef_step_{ 0.005 };
    bool dry_run_{ false };

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
