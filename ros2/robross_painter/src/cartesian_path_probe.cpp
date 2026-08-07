#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <thread>
#include <vector>

#include <json/json.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <rclcpp/rclcpp.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>

#include "cartesian_failure_diagnostics.hpp"

namespace
{

bool loadSeed(const std::string &path,
              robross_painter::CartesianFailureRecord &record,
              std::string &error)
{
    if (path.empty()) {
        return true;
    }
    std::ifstream in(path);
    if (!in) {
        error = "cannot open start-state seed: " + path;
        return false;
    }
    Json::Value root;
    Json::CharReaderBuilder builder;
    if (!Json::parseFromStream(builder, in, &root, &error)) {
        return false;
    }
    if (root.get("schema_version", 0).asInt() != 1 ||
        !root["command_index"].isInt() ||
        root["command_index"].asInt() != record.command_index) {
        error = "start-state seed schema or command index does not match the "
                "artifact";
        return false;
    }
    if (!root["joint_names"].isArray() ||
        !root["joint_positions_rad"].isArray() ||
        root["joint_names"].size() != root["joint_positions_rad"].size() ||
        root["joint_names"].empty()) {
        error = "start-state seed has invalid joint arrays";
        return false;
    }
    std::vector<std::string> names;
    std::vector<double> positions;
    for (Json::ArrayIndex i = 0; i < root["joint_names"].size(); ++i) {
        const auto &name = root["joint_names"][i];
        const auto &position = root["joint_positions_rad"][i];
        if (!name.isString() || !position.isNumeric() ||
            !std::isfinite(position.asDouble())) {
            error = "start-state seed contains malformed joint data";
            return false;
        }
        names.push_back(name.asString());
        positions.push_back(position.asDouble());
    }
    if (std::set<std::string>(names.begin(), names.end()).size() !=
        names.size()) {
        error = "start-state seed contains duplicate joint names";
        return false;
    }
    if (std::set<std::string>(names.begin(), names.end()) !=
        std::set<std::string>(record.joint_names.begin(),
                              record.joint_names.end())) {
        error = "start-state seed joints do not match the artifact";
        return false;
    }
    std::map<std::string, double> by_name;
    for (std::size_t i = 0; i < names.size(); ++i) {
        by_name.emplace(names[i], positions[i]);
    }
    record.joint_positions_rad.clear();
    for (const auto &name : record.joint_names) {
        record.joint_positions_rad.push_back(by_name.at(name));
    }
    record.exact_start_state = root.get("exact_start_state", false).asBool();
    record.source = root.get("source", path).asString();
    return true;
}

bool removeAttachedObject(
    moveit::planning_interface::PlanningSceneInterface &scene,
    const std::string &id)
{
    const auto objects = scene.getAttachedObjects({ id });
    const auto found = objects.find(id);
    if (found == objects.end()) {
        return true;
    }
    moveit_msgs::msg::AttachedCollisionObject remove;
    remove.link_name = found->second.link_name;
    remove.object.id = id;
    remove.object.operation = moveit_msgs::msg::CollisionObject::REMOVE;
    return scene.applyAttachedCollisionObject(remove);
}

class SceneRestoreGuard
{
public:
    SceneRestoreGuard(
        moveit::planning_interface::PlanningSceneInterface &scene,
        const robross_painter::CartesianFailureRecord &record)
        : scene_(scene)
    {
        std::set<std::string> unique_ids;
        for (const auto &box : record.scene_boxes) {
            unique_ids.insert(box.id);
        }
        unique_ids.insert(record.removed_world_objects.begin(),
                          record.removed_world_objects.end());
        unique_ids.insert(record.removed_attached_objects.begin(),
                          record.removed_attached_objects.end());
        ids_.assign(unique_ids.begin(), unique_ids.end());
        original_world_ = scene_.getObjects(ids_);
        original_attached_ = scene_.getAttachedObjects(ids_);
    }

    ~SceneRestoreGuard()
    {
        try {
            restore();
        } catch (...) {
            // Destructors must not terminate the process during cleanup.
        }
    }

    bool restore()
    {
        if (restored_) {
            return true;
        }
        try {
            bool ok = true;
            for (const auto &id : ids_) {
                ok = removeAttachedObject(scene_, id) && ok;
                moveit_msgs::msg::CollisionObject remove;
                remove.id = id;
                remove.operation = moveit_msgs::msg::CollisionObject::REMOVE;
                ok = scene_.applyCollisionObject(remove) && ok;
            }
            for (const auto &entry : original_world_) {
                ok = scene_.applyCollisionObject(entry.second) && ok;
            }
            for (const auto &entry : original_attached_) {
                ok = scene_.applyAttachedCollisionObject(entry.second) && ok;
            }
            restored_ = ok;
            return ok;
        } catch (...) {
            return false;
        }
    }

private:
    moveit::planning_interface::PlanningSceneInterface &scene_;
    std::vector<std::string> ids_;
    std::map<std::string, moveit_msgs::msg::CollisionObject> original_world_;
    std::map<std::string, moveit_msgs::msg::AttachedCollisionObject>
        original_attached_;
    bool restored_{ false };
};

class ExecutorThreadGuard
{
public:
    explicit ExecutorThreadGuard(
        rclcpp::executors::SingleThreadedExecutor &executor)
        : executor_(executor), thread_([this]() { executor_.spin(); })
    {
    }

    ~ExecutorThreadGuard() { stop(); }

    void stop()
    {
        if (stopped_) {
            return;
        }
        executor_.cancel();
        if (thread_.joinable()) {
            thread_.join();
        }
        stopped_ = true;
    }

private:
    rclcpp::executors::SingleThreadedExecutor &executor_;
    std::thread thread_;
    bool stopped_{ false };
};

bool hasUnexpectedCurrentObjects(
    moveit::planning_interface::PlanningSceneInterface &scene)
{
    const std::set<std::string> known_world{
        "ground_plane", "canvas_backing", "pen_claw"
    };
    for (const auto &entry : scene.getObjects()) {
        if (known_world.count(entry.first) == 0) {
            return true;
        }
    }
    for (const auto &entry : scene.getAttachedObjects()) {
        if (entry.first != "pen_claw") {
            return true;
        }
    }
    return false;
}

bool applyRecordedScene(
    moveit::planning_interface::PlanningSceneInterface &scene,
    const robross_painter::CartesianFailureRecord &record)
{
    for (const auto &id : record.removed_attached_objects) {
        if (!removeAttachedObject(scene, id)) {
            return false;
        }
    }
    for (const auto &id : record.removed_world_objects) {
        moveit_msgs::msg::CollisionObject remove;
        remove.header.frame_id = record.planning_frame;
        remove.id = id;
        remove.operation = moveit_msgs::msg::CollisionObject::REMOVE;
        if (!scene.applyCollisionObject(remove)) {
            return false;
        }
    }

    for (const auto &box : record.scene_boxes) {
        if (!removeAttachedObject(scene, box.id)) {
            return false;
        }
        moveit_msgs::msg::CollisionObject remove_world;
        remove_world.id = box.id;
        remove_world.operation = moveit_msgs::msg::CollisionObject::REMOVE;
        if (!scene.applyCollisionObject(remove_world)) {
            return false;
        }
        shape_msgs::msg::SolidPrimitive primitive;
        primitive.type = shape_msgs::msg::SolidPrimitive::BOX;
        primitive.dimensions.insert(primitive.dimensions.end(),
                                    box.dimensions.begin(),
                                    box.dimensions.end());
        if (box.attached) {
            moveit_msgs::msg::AttachedCollisionObject attached;
            attached.link_name = box.link_name;
            attached.touch_links = box.touch_links;
            attached.object.header.frame_id = box.frame_id;
            attached.object.id = box.id;
            attached.object.primitives.push_back(primitive);
            attached.object.primitive_poses.push_back(box.pose);
            attached.object.operation = moveit_msgs::msg::CollisionObject::ADD;
            if (!scene.applyAttachedCollisionObject(attached)) {
                return false;
            }
        } else {
            moveit_msgs::msg::CollisionObject object;
            object.header.frame_id = box.frame_id;
            object.id = box.id;
            object.primitives.push_back(primitive);
            object.primitive_poses.push_back(box.pose);
            object.operation = moveit_msgs::msg::CollisionObject::ADD;
            if (!scene.applyCollisionObject(object)) {
                return false;
            }
        }
    }
    return true;
}

}  // namespace

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>(
        "cartesian_path_probe",
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(
            true));

    std::string artifact_path;
    std::string start_state_path;
    int repetitions = 1;
    bool allow_unexpected_scene_objects = false;
    bool confirm_isolated_move_group = false;
    node->get_parameter_or("recorded_request_file", artifact_path,
                           artifact_path);
    node->get_parameter_or("start_state_file", start_state_path,
                           start_state_path);
    node->get_parameter_or("repetitions", repetitions, repetitions);
    node->get_parameter_or("allow_unexpected_scene_objects",
                           allow_unexpected_scene_objects,
                           allow_unexpected_scene_objects);
    node->get_parameter_or("confirm_isolated_move_group",
                           confirm_isolated_move_group,
                           confirm_isolated_move_group);

    if (artifact_path.empty() || repetitions <= 0 ||
        !confirm_isolated_move_group) {
        RCLCPP_FATAL(node->get_logger(),
                     "recorded_request_file is required and repetitions must "
                     "be positive; confirm_isolated_move_group must be true");
        rclcpp::shutdown();
        return 2;
    }

    robross_painter::CartesianFailureRecord record;
    std::string error;
    if (!robross_painter::readCartesianFailureRecord(artifact_path, record,
                                                      error) ||
        !loadSeed(start_state_path, record, error)) {
        RCLCPP_FATAL(node->get_logger(), "%s", error.c_str());
        rclcpp::shutdown();
        return 2;
    }
    if ((!record.unexpected_world_objects.empty() ||
         !record.unexpected_attached_objects.empty()) &&
        !allow_unexpected_scene_objects) {
        RCLCPP_FATAL(node->get_logger(),
                     "Artifact contains unexpected planning-scene objects; "
                     "exact reconstruction is unavailable");
        rclcpp::shutdown();
        return 2;
    }

    moveit::planning_interface::PlanningSceneInterface scene;
    if (hasUnexpectedCurrentObjects(scene) &&
        !allow_unexpected_scene_objects) {
        RCLCPP_FATAL(node->get_logger(),
                     "Replay move_group contains unexpected planning-scene "
                     "objects; refusing to mutate a non-isolated scene");
        rclcpp::shutdown();
        return 2;
    }
    SceneRestoreGuard scene_restore(scene, record);
    if (!applyRecordedScene(scene, record)) {
        RCLCPP_FATAL(node->get_logger(),
                     "Failed to reconstruct the recorded planning scene");
        scene_restore.restore();
        rclcpp::shutdown();
        return 2;
    }

    auto client = node->create_client<moveit_msgs::srv::GetCartesianPath>(
        "/compute_cartesian_path");
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    ExecutorThreadGuard executor_thread(executor);

    bool normal_complete_every_time = true;
    bool stable_classification = true;
    bool request_error = false;
    robross_painter::CartesianFailureClass first_class =
        robross_painter::CartesianFailureClass::Inconclusive;
    try {
        for (int repetition = 1; repetition <= repetitions; ++repetition) {
            const auto normal = robross_painter::runCartesianRequest(
                client,
                robross_painter::makeCartesianRequest(
                    record, record.jump_threshold, true));
            const auto no_jump = robross_painter::runCartesianRequest(
                client,
                robross_painter::makeCartesianRequest(record, 0.0, true));
            const auto no_jump_no_collision =
                robross_painter::runCartesianRequest(
                    client,
                    robross_painter::makeCartesianRequest(record, 0.0, false));
            const auto classification =
                robross_painter::classifyCartesianFailure(
                    normal, no_jump, no_jump_no_collision);
            if (!robross_painter::cartesianAttemptSucceeded(normal) ||
                !robross_painter::cartesianAttemptSucceeded(no_jump) ||
                !robross_painter::cartesianAttemptSucceeded(
                    no_jump_no_collision)) {
                request_error = true;
            }
            if (repetition == 1) {
                first_class = classification;
            } else if (classification != first_class) {
                stable_classification = false;
            }
            normal_complete_every_time &=
                robross_painter::cartesianAttemptComplete(normal);
            RCLCPP_INFO(
                node->get_logger(),
                "probe %d/%d: normal=%.6f/%zu no_jump=%.6f/%zu "
                "no_jump_no_collision=%.6f/%zu class=%s",
                repetition, repetitions, normal.fraction, normal.point_count,
                no_jump.fraction, no_jump.point_count,
                no_jump_no_collision.fraction,
                no_jump_no_collision.point_count,
                robross_painter::cartesianFailureClassName(classification));
            if (request_error) {
                break;
            }
        }
    } catch (const std::exception &exception) {
        RCLCPP_ERROR(node->get_logger(), "Probe request failed: %s",
                     exception.what());
        request_error = true;
    }

    executor_thread.stop();
    if (!scene_restore.restore()) {
        RCLCPP_ERROR(node->get_logger(),
                     "Failed to restore the pre-probe planning scene");
        request_error = true;
    }
    rclcpp::shutdown();
    if (request_error) {
        return 2;
    }
    if (!stable_classification) {
        return 3;
    }
    return normal_complete_every_time ? 0 : 1;
}
