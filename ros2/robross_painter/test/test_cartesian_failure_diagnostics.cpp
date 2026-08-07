#include <gtest/gtest.h>

#include <cstdio>
#include <string>

#include <moveit_msgs/msg/move_it_error_codes.hpp>

#include "cartesian_failure_diagnostics.hpp"

namespace
{

robross_painter::CartesianAttemptSummary attempt(double fraction,
                                                  std::size_t points = 1)
{
    robross_painter::CartesianAttemptSummary value;
    value.fraction = fraction;
    value.error_code = moveit_msgs::msg::MoveItErrorCodes::SUCCESS;
    value.point_count = points;
    return value;
}

robross_painter::CartesianFailureRecord record()
{
    robross_painter::CartesianFailureRecord value;
    value.command_index = 157;
    value.command = "paint_path";
    value.label = "line_art_line_39";
    value.cartesian_call_ordinal = 1;
    value.planning_group = "manipulator";
    value.planning_frame = "base_link";
    value.end_effector_link = "ee_link";
    value.joint_names = { "joint_a", "joint_b" };
    value.joint_positions_rad = { 1.25, -0.5 };
    geometry_msgs::msg::Pose pose;
    pose.position.x = 0.1;
    pose.orientation.w = 1.0;
    value.waypoints.push_back(pose);
    value.eef_step_m = 0.0002;
    value.jump_threshold = 2.0;
    value.normal = attempt(0.036, 2);
    value.no_jump = attempt(1.0, 28);
    value.no_jump_no_collision = attempt(1.0, 28);
    value.classification = robross_painter::CartesianFailureClass::JumpLimited;
    value.retreat_status = "succeeded";
    value.source = "test";
    robross_painter::SceneBox box;
    box.id = "canvas_backing";
    box.frame_id = "base_link";
    box.dimensions = { 0.31, 0.40, 0.05 };
    box.pose.orientation.w = 1.0;
    value.scene_boxes.push_back(box);
    return value;
}

}  // namespace

TEST(CartesianFailureDiagnostics, ClassifiesJumpLimited)
{
    EXPECT_EQ(robross_painter::classifyCartesianFailure(
                  attempt(0.036), attempt(1.0), attempt(1.0)),
              robross_painter::CartesianFailureClass::JumpLimited);
}

TEST(CartesianFailureDiagnostics, ClassifiesCollisionLimited)
{
    EXPECT_EQ(robross_painter::classifyCartesianFailure(
                  attempt(0.036), attempt(0.036), attempt(1.0)),
              robross_painter::CartesianFailureClass::CollisionLimited);
}

TEST(CartesianFailureDiagnostics, ClassifiesKinematicFailure)
{
    EXPECT_EQ(robross_painter::classifyCartesianFailure(
                  attempt(0.036), attempt(0.036), attempt(0.036)),
              robross_painter::CartesianFailureClass::IkOrKinematic);
}

TEST(CartesianFailureDiagnostics, ClassifiesMixedAndInconclusiveResults)
{
    EXPECT_EQ(robross_painter::classifyCartesianFailure(
                  attempt(0.1), attempt(0.5), attempt(1.0)),
              robross_painter::CartesianFailureClass::Mixed);
    EXPECT_EQ(robross_painter::classifyCartesianFailure(
                  attempt(0.5), attempt(0.4), attempt(1.0)),
              robross_painter::CartesianFailureClass::Inconclusive);
    EXPECT_EQ(robross_painter::classifyCartesianFailure(
                  attempt(-1.0), attempt(0.0), attempt(0.0)),
              robross_painter::CartesianFailureClass::RequestError);
    auto service_failure = attempt(0.5);
    service_failure.error_code = moveit_msgs::msg::MoveItErrorCodes::FAILURE;
    EXPECT_EQ(robross_painter::classifyCartesianFailure(
                  service_failure, attempt(1.0), attempt(1.0)),
              robross_painter::CartesianFailureClass::RequestError);
}

TEST(CartesianFailureDiagnostics, BuildsExactPlanningOnlyRequestVariants)
{
    const auto value = record();
    const auto normal =
        robross_painter::makeCartesianRequest(value, 2.0, true);
    const auto no_jump =
        robross_painter::makeCartesianRequest(value, 0.0, true);
    const auto no_checks =
        robross_painter::makeCartesianRequest(value, 0.0, false);

    EXPECT_EQ(normal.header.frame_id, "base_link");
    EXPECT_TRUE(normal.start_state.is_diff);
    EXPECT_EQ(normal.start_state.joint_state.name, value.joint_names);
    EXPECT_EQ(normal.start_state.joint_state.position,
              value.joint_positions_rad);
    EXPECT_DOUBLE_EQ(normal.max_step, 0.0002);
    EXPECT_DOUBLE_EQ(normal.jump_threshold, 2.0);
    EXPECT_TRUE(normal.avoid_collisions);
    EXPECT_DOUBLE_EQ(no_jump.jump_threshold, 0.0);
    EXPECT_TRUE(no_jump.avoid_collisions);
    EXPECT_DOUBLE_EQ(no_checks.jump_threshold, 0.0);
    EXPECT_FALSE(no_checks.avoid_collisions);
}

TEST(CartesianFailureDiagnostics, FullStartStateCarriesAttachedClaw)
{
    auto value = record();
    robross_painter::SceneBox claw;
    claw.id = "pen_claw";
    claw.frame_id = "ee_link";
    claw.attached = true;
    claw.link_name = "ee_link";
    claw.touch_links = { "ee_link", "wrist3_Link" };
    claw.dimensions = { 0.02, 0.06, 0.02 };
    claw.pose.orientation.w = 1.0;
    value.scene_boxes.push_back(claw);

    const auto request =
        robross_painter::makeCartesianRequest(value, 2.0, true);
    ASSERT_EQ(request.start_state.attached_collision_objects.size(), 1u);
    const auto &attached = request.start_state.attached_collision_objects[0];
    EXPECT_EQ(attached.object.id, "pen_claw");
    EXPECT_EQ(attached.link_name, "ee_link");
    EXPECT_EQ(attached.touch_links, claw.touch_links);
    ASSERT_EQ(attached.object.primitives.size(), 1u);
    EXPECT_EQ(attached.object.primitives[0].dimensions.size(), 3u);
}

TEST(CartesianFailureDiagnostics, Line39OmitsAlreadyReachedFirstPoint)
{
    const std::vector<robross_painter::CanvasPoint> line39{
        { 145.42, 74.29 },
        { 145.17, 74.54 },
        { 142.36, 74.54 },
        { 142.11, 74.29 },
        { 140.58, 74.29 },
    };
    const auto targets = robross_painter::selectPaintPathTargets(
        line39, 145.42, 74.29);
    ASSERT_EQ(targets.size(), 4u);
    EXPECT_DOUBLE_EQ(targets.front().x_mm, 145.17);
    EXPECT_DOUBLE_EQ(targets.back().x_mm, 140.58);
}

TEST(CartesianFailureDiagnostics, ArtifactRoundTripsWithoutPrecisionLoss)
{
    const auto expected = record();
    const std::string path =
        std::string(::testing::TempDir()) + "/cartesian_failure.json";
    std::string error;
    ASSERT_TRUE(robross_painter::writeCartesianFailureRecord(expected, path,
                                                              error))
        << error;

    robross_painter::CartesianFailureRecord actual;
    ASSERT_TRUE(robross_painter::readCartesianFailureRecord(path, actual,
                                                             error))
        << error;
    EXPECT_EQ(actual.command_index, expected.command_index);
    EXPECT_EQ(actual.joint_names, expected.joint_names);
    EXPECT_EQ(actual.joint_positions_rad, expected.joint_positions_rad);
    ASSERT_EQ(actual.waypoints.size(), 1u);
    EXPECT_DOUBLE_EQ(actual.waypoints[0].position.x,
                     expected.waypoints[0].position.x);
    ASSERT_EQ(actual.scene_boxes.size(), 1u);
    EXPECT_EQ(actual.scene_boxes[0].dimensions,
              expected.scene_boxes[0].dimensions);
    EXPECT_EQ(actual.classification,
              robross_painter::CartesianFailureClass::JumpLimited);
    std::remove(path.c_str());
}

TEST(CartesianFailureDiagnostics, RequestTemplatePreservesStoredClassification)
{
    auto expected = record();
    expected.classification = robross_painter::CartesianFailureClass::None;
    expected.no_jump = {};
    expected.no_jump_no_collision = {};
    const std::string path =
        std::string(::testing::TempDir()) + "/cartesian_request.json";
    std::string error;
    ASSERT_TRUE(robross_painter::writeCartesianFailureRecord(expected, path,
                                                              error))
        << error;
    robross_painter::CartesianFailureRecord actual;
    ASSERT_TRUE(robross_painter::readCartesianFailureRecord(path, actual,
                                                             error))
        << error;
    EXPECT_EQ(actual.classification,
              robross_painter::CartesianFailureClass::None);
    std::remove(path.c_str());
}
