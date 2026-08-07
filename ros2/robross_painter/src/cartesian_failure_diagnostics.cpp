#include "cartesian_failure_diagnostics.hpp"

#include <cmath>
#include <exception>
#include <fstream>
#include <future>
#include <limits>
#include <set>
#include <utility>

#include <json/json.h>
#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>

namespace robross_painter
{
namespace
{

Json::Value poseToJson(const geometry_msgs::msg::Pose &pose)
{
    Json::Value value;
    value["position"]["x"] = pose.position.x;
    value["position"]["y"] = pose.position.y;
    value["position"]["z"] = pose.position.z;
    value["orientation"]["x"] = pose.orientation.x;
    value["orientation"]["y"] = pose.orientation.y;
    value["orientation"]["z"] = pose.orientation.z;
    value["orientation"]["w"] = pose.orientation.w;
    return value;
}

bool finiteNumber(const Json::Value &value)
{
    return value.isNumeric() && std::isfinite(value.asDouble());
}

bool poseFromJson(const Json::Value &value, geometry_msgs::msg::Pose &pose)
{
    const auto &p = value["position"];
    const auto &q = value["orientation"];
    if (!finiteNumber(p["x"]) || !finiteNumber(p["y"]) ||
        !finiteNumber(p["z"]) || !finiteNumber(q["x"]) ||
        !finiteNumber(q["y"]) || !finiteNumber(q["z"]) ||
        !finiteNumber(q["w"])) {
        return false;
    }
    pose.position.x = p["x"].asDouble();
    pose.position.y = p["y"].asDouble();
    pose.position.z = p["z"].asDouble();
    pose.orientation.x = q["x"].asDouble();
    pose.orientation.y = q["y"].asDouble();
    pose.orientation.z = q["z"].asDouble();
    pose.orientation.w = q["w"].asDouble();
    return true;
}

Json::Value stringVectorToJson(const std::vector<std::string> &values)
{
    Json::Value result(Json::arrayValue);
    for (const auto &value : values) {
        result.append(value);
    }
    return result;
}

Json::Value doubleVectorToJson(const std::vector<double> &values)
{
    Json::Value result(Json::arrayValue);
    for (const double value : values) {
        result.append(value);
    }
    return result;
}

bool stringVectorFromJson(const Json::Value &value,
                          std::vector<std::string> &result)
{
    if (!value.isArray()) {
        return false;
    }
    result.clear();
    for (const auto &entry : value) {
        if (!entry.isString()) {
            return false;
        }
        result.push_back(entry.asString());
    }
    return true;
}

bool doubleVectorFromJson(const Json::Value &value,
                          std::vector<double> &result)
{
    if (!value.isArray()) {
        return false;
    }
    result.clear();
    for (const auto &entry : value) {
        if (!finiteNumber(entry)) {
            return false;
        }
        result.push_back(entry.asDouble());
    }
    return true;
}

Json::Value attemptToJson(const CartesianAttemptSummary &attempt)
{
    Json::Value value;
    value["fraction"] = attempt.fraction;
    value["error_code"] = attempt.error_code;
    value["point_count"] = static_cast<Json::UInt64>(attempt.point_count);
    value["error"] = attempt.error;
    return value;
}

bool attemptFromJson(const Json::Value &value,
                     CartesianAttemptSummary &attempt)
{
    if (!finiteNumber(value["fraction"]) ||
        !value["error_code"].isInt() || !value["point_count"].isUInt64() ||
        !value.get("error", "").isString()) {
        return false;
    }
    attempt.fraction = value["fraction"].asDouble();
    attempt.error_code = value["error_code"].asInt();
    attempt.point_count =
        static_cast<std::size_t>(value["point_count"].asUInt64());
    attempt.error = value.get("error", "").asString();
    return true;
}

bool hasDuplicateStrings(const std::vector<std::string> &values)
{
    return std::set<std::string>(values.begin(), values.end()).size() !=
           values.size();
}

}  // namespace

std::vector<CanvasPoint> selectPaintPathTargets(
    const std::vector<CanvasPoint> &points, double current_x_mm,
    double current_y_mm, double correction_threshold_mm)
{
    std::vector<CanvasPoint> targets;
    if (points.size() < 2) {
        return targets;
    }
    targets.reserve(points.size());
    if (std::hypot(points.front().x_mm - current_x_mm,
                   points.front().y_mm - current_y_mm) >
        correction_threshold_mm) {
        targets.push_back(points.front());
    }
    targets.insert(targets.end(), points.begin() + 1, points.end());
    return targets;
}

bool cartesianAttemptComplete(const CartesianAttemptSummary &attempt)
{
    return cartesianAttemptSucceeded(attempt) &&
           attempt.fraction >= kRequiredCartesianFraction;
}

bool cartesianAttemptSucceeded(const CartesianAttemptSummary &attempt)
{
    return attempt.error.empty() && std::isfinite(attempt.fraction) &&
           attempt.fraction >= 0.0 &&
           attempt.error_code == moveit_msgs::msg::MoveItErrorCodes::SUCCESS;
}

CartesianFailureClass classifyCartesianFailure(
    const CartesianAttemptSummary &normal,
    const CartesianAttemptSummary &no_jump,
    const CartesianAttemptSummary &no_jump_no_collision, double epsilon)
{
    if (!cartesianAttemptSucceeded(normal) ||
        !cartesianAttemptSucceeded(no_jump) ||
        !cartesianAttemptSucceeded(no_jump_no_collision)) {
        return CartesianFailureClass::RequestError;
    }
    if (cartesianAttemptComplete(normal)) {
        return CartesianFailureClass::None;
    }

    // Removing checks cannot legitimately reduce a deterministic result.
    if (no_jump.fraction + epsilon < normal.fraction ||
        no_jump_no_collision.fraction + epsilon < no_jump.fraction) {
        return CartesianFailureClass::Inconclusive;
    }
    if (cartesianAttemptComplete(no_jump)) {
        return CartesianFailureClass::JumpLimited;
    }
    if (cartesianAttemptComplete(no_jump_no_collision)) {
        return no_jump.fraction > normal.fraction + epsilon
                   ? CartesianFailureClass::Mixed
                   : CartesianFailureClass::CollisionLimited;
    }

    const bool jump_improved = no_jump.fraction > normal.fraction + epsilon;
    const bool collision_improved =
        no_jump_no_collision.fraction > no_jump.fraction + epsilon;
    if (jump_improved || collision_improved) {
        return CartesianFailureClass::Mixed;
    }
    if (std::abs(no_jump.fraction - normal.fraction) <= epsilon &&
        std::abs(no_jump_no_collision.fraction - no_jump.fraction) <= epsilon) {
        return CartesianFailureClass::IkOrKinematic;
    }
    return CartesianFailureClass::Inconclusive;
}

bool cartesianFailureClassFromName(const std::string &name,
                                   CartesianFailureClass &value)
{
    for (const auto candidate : {
             CartesianFailureClass::None,
             CartesianFailureClass::RequestError,
             CartesianFailureClass::JumpLimited,
             CartesianFailureClass::CollisionLimited,
             CartesianFailureClass::Mixed,
             CartesianFailureClass::IkOrKinematic,
             CartesianFailureClass::Inconclusive,
         }) {
        if (name == cartesianFailureClassName(candidate)) {
            value = candidate;
            return true;
        }
    }
    return false;
}

const char *cartesianFailureClassName(CartesianFailureClass value)
{
    switch (value) {
    case CartesianFailureClass::None:
        return "none";
    case CartesianFailureClass::RequestError:
        return "request_error";
    case CartesianFailureClass::JumpLimited:
        return "jump_limited";
    case CartesianFailureClass::CollisionLimited:
        return "collision_limited";
    case CartesianFailureClass::Mixed:
        return "mixed";
    case CartesianFailureClass::IkOrKinematic:
        return "ik_or_kinematic";
    case CartesianFailureClass::Inconclusive:
        return "inconclusive";
    }
    return "inconclusive";
}

moveit_msgs::srv::GetCartesianPath::Request makeCartesianRequest(
    const CartesianFailureRecord &record, double jump_threshold,
    bool avoid_collisions)
{
    moveit_msgs::srv::GetCartesianPath::Request request;
    request.header.frame_id = record.planning_frame;
    request.start_state.is_diff = true;
    request.start_state.joint_state.name = record.joint_names;
    request.start_state.joint_state.position = record.joint_positions_rad;
    for (const auto &box : record.scene_boxes) {
        if (!box.attached) {
            continue;
        }
        moveit_msgs::msg::AttachedCollisionObject attached;
        attached.link_name = box.link_name;
        attached.touch_links = box.touch_links;
        attached.object.header.frame_id = box.frame_id;
        attached.object.id = box.id;
        shape_msgs::msg::SolidPrimitive primitive;
        primitive.type = shape_msgs::msg::SolidPrimitive::BOX;
        primitive.dimensions.insert(primitive.dimensions.end(),
                                    box.dimensions.begin(),
                                    box.dimensions.end());
        attached.object.primitives.push_back(primitive);
        attached.object.primitive_poses.push_back(box.pose);
        attached.object.operation = moveit_msgs::msg::CollisionObject::ADD;
        request.start_state.attached_collision_objects.push_back(
            std::move(attached));
    }
    request.group_name = record.planning_group;
    request.link_name = record.end_effector_link;
    request.waypoints = record.waypoints;
    request.max_step = record.eef_step_m;
    request.jump_threshold = jump_threshold;
    request.avoid_collisions = avoid_collisions;
    return request;
}

CartesianAttemptSummary runCartesianRequest(
    const rclcpp::Client<moveit_msgs::srv::GetCartesianPath>::SharedPtr &client,
    const moveit_msgs::srv::GetCartesianPath::Request &request,
    std::chrono::milliseconds timeout)
{
    CartesianAttemptSummary summary;
    summary.error_code = moveit_msgs::msg::MoveItErrorCodes::FAILURE;
    if (!client || !client->wait_for_service(timeout)) {
        summary.error = "/compute_cartesian_path service unavailable";
        return summary;
    }
    try {
        auto future = client->async_send_request(
            std::make_shared<moveit_msgs::srv::GetCartesianPath::Request>(
                request));
        if (future.wait_for(timeout) != std::future_status::ready) {
            client->remove_pending_request(future);
            summary.error = "/compute_cartesian_path request timed out";
            return summary;
        }
        const auto response = future.get();
        summary.fraction = response->fraction;
        summary.error_code = response->error_code.val;
        summary.point_count =
            response->solution.joint_trajectory.points.size();
    } catch (const std::exception &exception) {
        summary.error = std::string("/compute_cartesian_path exception: ") +
                        exception.what();
    }
    return summary;
}

bool writeCartesianFailureRecord(const CartesianFailureRecord &record,
                                 const std::string &path, std::string &error)
{
    Json::Value root;
    root["schema_version"] = record.schema_version;
    root["command_index"] = record.command_index;
    root["command"] = record.command;
    root["label"] = record.label;
    root["cartesian_call_ordinal"] = record.cartesian_call_ordinal;
    root["planning_group"] = record.planning_group;
    root["planning_frame"] = record.planning_frame;
    root["end_effector_link"] = record.end_effector_link;
    root["joint_names"] = stringVectorToJson(record.joint_names);
    root["joint_positions_rad"] =
        doubleVectorToJson(record.joint_positions_rad);
    root["eef_step_m"] = record.eef_step_m;
    root["jump_threshold"] = record.jump_threshold;
    root["classification"] = cartesianFailureClassName(record.classification);
    root["retreat_status"] = record.retreat_status;
    root["exact_start_state"] = record.exact_start_state;
    root["source"] = record.source;
    root["attempts"]["normal"] = attemptToJson(record.normal);
    root["attempts"]["no_jump"] = attemptToJson(record.no_jump);
    root["attempts"]["no_jump_no_collision"] =
        attemptToJson(record.no_jump_no_collision);
    root["removed_world_objects"] =
        stringVectorToJson(record.removed_world_objects);
    root["removed_attached_objects"] =
        stringVectorToJson(record.removed_attached_objects);
    root["unexpected_world_objects"] =
        stringVectorToJson(record.unexpected_world_objects);
    root["unexpected_attached_objects"] =
        stringVectorToJson(record.unexpected_attached_objects);

    Json::Value waypoints(Json::arrayValue);
    for (const auto &waypoint : record.waypoints) {
        waypoints.append(poseToJson(waypoint));
    }
    root["waypoints_ee"] = waypoints;

    Json::Value boxes(Json::arrayValue);
    for (const auto &box : record.scene_boxes) {
        Json::Value value;
        value["id"] = box.id;
        value["frame_id"] = box.frame_id;
        value["attached"] = box.attached;
        value["link_name"] = box.link_name;
        value["touch_links"] = stringVectorToJson(box.touch_links);
        value["dimensions"] = doubleVectorToJson(box.dimensions);
        value["pose"] = poseToJson(box.pose);
        boxes.append(value);
    }
    root["scene_boxes"] = boxes;

    std::ofstream out(path);
    if (!out) {
        error = "cannot open artifact for writing: " + path;
        return false;
    }
    Json::StreamWriterBuilder builder;
    builder["indentation"] = "  ";
    builder["precision"] = std::numeric_limits<double>::max_digits10;
    std::unique_ptr<Json::StreamWriter> writer(builder.newStreamWriter());
    if (writer->write(root, &out) != 0 || !out.good()) {
        error = "failed to write artifact: " + path;
        return false;
    }
    out << '\n';
    return true;
}

bool readCartesianFailureRecord(const std::string &path,
                                CartesianFailureRecord &record,
                                std::string &error)
{
    std::ifstream in(path);
    if (!in) {
        error = "cannot open artifact: " + path;
        return false;
    }
    Json::Value root;
    Json::CharReaderBuilder builder;
    if (!Json::parseFromStream(builder, in, &root, &error)) {
        return false;
    }
    if (root.get("schema_version", 0).asInt() != 1 ||
        !root["command_index"].isInt() || !root["command"].isString() ||
        !root["label"].isString() ||
        !root["cartesian_call_ordinal"].isInt() ||
        !root["planning_group"].isString() ||
        !root["planning_frame"].isString() ||
        !root["end_effector_link"].isString() ||
        !finiteNumber(root["eef_step_m"]) ||
        !finiteNumber(root["jump_threshold"])) {
        error = "artifact has missing or invalid request metadata";
        return false;
    }

    CartesianFailureRecord parsed;
    parsed.command_index = root["command_index"].asInt();
    parsed.command = root["command"].asString();
    parsed.label = root["label"].asString();
    parsed.cartesian_call_ordinal = root["cartesian_call_ordinal"].asInt();
    parsed.planning_group = root["planning_group"].asString();
    parsed.planning_frame = root["planning_frame"].asString();
    parsed.end_effector_link = root["end_effector_link"].asString();
    parsed.eef_step_m = root["eef_step_m"].asDouble();
    parsed.jump_threshold = root["jump_threshold"].asDouble();
    parsed.retreat_status = root.get("retreat_status", "unknown").asString();
    parsed.exact_start_state = root.get("exact_start_state", true).asBool();
    parsed.source = root.get("source", "").asString();
    if (!stringVectorFromJson(root["joint_names"], parsed.joint_names) ||
        !doubleVectorFromJson(root["joint_positions_rad"],
                              parsed.joint_positions_rad) ||
        parsed.joint_names.empty() ||
        parsed.joint_names.size() != parsed.joint_positions_rad.size() ||
        hasDuplicateStrings(parsed.joint_names) || parsed.eef_step_m <= 0.0 ||
        parsed.jump_threshold <= 0.0) {
        error = "artifact has an invalid start state or planning parameters";
        return false;
    }
    if (!root["waypoints_ee"].isArray() || root["waypoints_ee"].empty()) {
        error = "artifact has no Cartesian waypoints";
        return false;
    }
    for (const auto &value : root["waypoints_ee"]) {
        geometry_msgs::msg::Pose pose;
        if (!poseFromJson(value, pose)) {
            error = "artifact contains a malformed waypoint";
            return false;
        }
        parsed.waypoints.push_back(pose);
    }

    const auto &attempts = root["attempts"];
    if (!attemptFromJson(attempts["normal"], parsed.normal) ||
        !attemptFromJson(attempts["no_jump"], parsed.no_jump) ||
        !attemptFromJson(attempts["no_jump_no_collision"],
                         parsed.no_jump_no_collision)) {
        error = "artifact contains malformed attempt summaries";
        return false;
    }
    if (!stringVectorFromJson(root["removed_world_objects"],
                              parsed.removed_world_objects) ||
        !stringVectorFromJson(root["removed_attached_objects"],
                              parsed.removed_attached_objects) ||
        !stringVectorFromJson(root["unexpected_world_objects"],
                              parsed.unexpected_world_objects) ||
        !stringVectorFromJson(root["unexpected_attached_objects"],
                              parsed.unexpected_attached_objects)) {
        error = "artifact contains malformed scene object lists";
        return false;
    }

    if (!root["scene_boxes"].isArray()) {
        error = "artifact scene_boxes must be an array";
        return false;
    }
    for (const auto &value : root["scene_boxes"]) {
        SceneBox box;
        if (!value["id"].isString() || !value["frame_id"].isString() ||
            !value["attached"].isBool() || !value["link_name"].isString() ||
            !stringVectorFromJson(value["touch_links"], box.touch_links) ||
            !doubleVectorFromJson(value["dimensions"], box.dimensions) ||
            box.dimensions.size() != 3 || !poseFromJson(value["pose"], box.pose)) {
            error = "artifact contains a malformed scene box";
            return false;
        }
        box.id = value["id"].asString();
        box.frame_id = value["frame_id"].asString();
        box.attached = value["attached"].asBool();
        box.link_name = value["link_name"].asString();
        parsed.scene_boxes.push_back(std::move(box));
    }
    if (!root["classification"].isString() ||
        !cartesianFailureClassFromName(root["classification"].asString(),
                                       parsed.classification)) {
        error = "artifact contains an invalid classification";
        return false;
    }
    record = std::move(parsed);
    return true;
}

}  // namespace robross_painter
