#ifndef ROBROSS_PAINTER_CARTESIAN_FAILURE_DIAGNOSTICS_HPP
#define ROBROSS_PAINTER_CARTESIAN_FAILURE_DIAGNOSTICS_HPP

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose.hpp>
#include <moveit_msgs/srv/get_cartesian_path.hpp>
#include <rclcpp/rclcpp.hpp>

namespace robross_painter
{

constexpr double kRequiredCartesianFraction = 0.999;

struct CanvasPoint
{
    double x_mm = 0.0;
    double y_mm = 0.0;
};

std::vector<CanvasPoint> selectPaintPathTargets(
    const std::vector<CanvasPoint> &points, double current_x_mm,
    double current_y_mm, double correction_threshold_mm = 0.5);

struct CartesianAttemptSummary
{
    double fraction = -1.0;
    std::int32_t error_code = 0;
    std::size_t point_count = 0;
    std::string error;
};

enum class CartesianFailureClass
{
    None,
    RequestError,
    JumpLimited,
    CollisionLimited,
    Mixed,
    IkOrKinematic,
    Inconclusive
};

struct SceneBox
{
    std::string id;
    std::string frame_id;
    bool attached = false;
    std::string link_name;
    std::vector<std::string> touch_links;
    std::vector<double> dimensions;
    geometry_msgs::msg::Pose pose;
};

struct CartesianFailureRecord
{
    int schema_version = 1;
    int command_index = 0;
    std::string command;
    std::string label;
    int cartesian_call_ordinal = 0;
    std::string planning_group;
    std::string planning_frame;
    std::string end_effector_link;
    std::vector<std::string> joint_names;
    std::vector<double> joint_positions_rad;
    std::vector<geometry_msgs::msg::Pose> waypoints;
    double eef_step_m = 0.0;
    double jump_threshold = 0.0;
    CartesianAttemptSummary normal;
    CartesianAttemptSummary no_jump;
    CartesianAttemptSummary no_jump_no_collision;
    CartesianFailureClass classification = CartesianFailureClass::Inconclusive;
    std::string retreat_status = "not_attempted";
    bool exact_start_state = true;
    std::string source;
    std::vector<SceneBox> scene_boxes;
    std::vector<std::string> removed_world_objects;
    std::vector<std::string> removed_attached_objects;
    std::vector<std::string> unexpected_world_objects;
    std::vector<std::string> unexpected_attached_objects;
};

bool cartesianAttemptComplete(const CartesianAttemptSummary &attempt);
bool cartesianAttemptSucceeded(const CartesianAttemptSummary &attempt);

CartesianFailureClass classifyCartesianFailure(
    const CartesianAttemptSummary &normal,
    const CartesianAttemptSummary &no_jump,
    const CartesianAttemptSummary &no_jump_no_collision,
    double epsilon = 1e-9);

const char *cartesianFailureClassName(CartesianFailureClass value);
bool cartesianFailureClassFromName(const std::string &name,
                                   CartesianFailureClass &value);

moveit_msgs::srv::GetCartesianPath::Request makeCartesianRequest(
    const CartesianFailureRecord &record, double jump_threshold,
    bool avoid_collisions);

CartesianAttemptSummary runCartesianRequest(
    const rclcpp::Client<moveit_msgs::srv::GetCartesianPath>::SharedPtr &client,
    const moveit_msgs::srv::GetCartesianPath::Request &request,
    std::chrono::milliseconds timeout = std::chrono::seconds(10));

bool writeCartesianFailureRecord(const CartesianFailureRecord &record,
                                 const std::string &path,
                                 std::string &error);

bool readCartesianFailureRecord(const std::string &path,
                                CartesianFailureRecord &record,
                                std::string &error);

}  // namespace robross_painter

#endif
