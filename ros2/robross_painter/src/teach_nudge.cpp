// Teaching-time nudge helper: repeatable sub-millimeter steps along the
// pen axis for the final approach to a paper corner.
//
// Freedrive is only good to ~10 mm (breakaway force) and the pendant's
// continuous jog makes consistent 0.2 mm depth steps an operator skill.
// This node turns the final approach into counted, commanded steps:
// hover a few mm off the corner, then call ~/nudge_in until the pen body
// first visibly moves relative to the claw (spring just engaged = true
// paper surface), record the corner with teach_canvas.py, and ~/nudge_out
// clear again.
//
// Deliberately standalone: it never reads a canvas, paths, or bounds, and
// it is run only during teaching (kill it together with teach_canvas.py),
// so no nudge service can be live while painting and the safety-critical
// painting_executor stays untouched. A nudge is a pure translation, so
// the tip moves exactly as ee_link does and tool_offset_xyz is not needed
// here — only the pen axis direction (tool_offset_rpy applied to ee +Z).
//
// Requires move_group running and joint_trajectory_controller ACTIVE
// (disable freedrive first). Test the in/out sign well clear of the paper
// after any claw or tool_offset_rpy change.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>
#include <moveit_msgs/action/execute_trajectory.hpp>
#include <moveit_msgs/action/move_group.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>

namespace {

constexpr double kMinStepMm = 0.05;
constexpr double kMaxStepMm = 2.0;

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

class TeachNudge
{
public:
    // node: services + parameters, spun by main(). move_node: handed to
    // MoveGroupInterface, which spins it on its own internal executor —
    // that split is what lets a service callback block on planning and
    // execution without starving the action-response callbacks.
    TeachNudge(const rclcpp::Node::SharedPtr &node,
               const rclcpp::Node::SharedPtr &move_node)
        : node_(node), group_(connectMoveGroup(move_node))
    {
        // node_ auto-declares parameters from launch overrides, so guard each
        // declare: use the launch value when present, else this default.
        declareIfAbsent("nudge_step_mm", 0.5);
        // Pen axis in ee_frame: this rotation applied to +Z. Must match the
        // executor's tool_offset_rpy (the offset itself is irrelevant for a
        // pure translation).
        declareIfAbsent("tool_offset_rpy",
                        std::vector<double>{ 0.0, 0.0, 0.0 });
        declareIfAbsent("ee_frame", std::string("ee_link"));
        declareIfAbsent("velocity_scaling", 0.05);
        declareIfAbsent("acceleration_scaling", 0.05);
        // Never 0: 0 disables the guard against IK configuration flips,
        // which execute as unchecked sweeps (same rule as the executor).
        declareIfAbsent("jump_threshold", 2.0);
        declareIfAbsent("joint_states_topic", std::string("/joint_states"));

        in_srv_ = node_->create_service<std_srvs::srv::Trigger>(
            "~/nudge_in",
            [this](const std_srvs::srv::Trigger::Request::SharedPtr,
                   std_srvs::srv::Trigger::Response::SharedPtr res) {
                nudge(+1.0, *res);
            });
        out_srv_ = node_->create_service<std_srvs::srv::Trigger>(
            "~/nudge_out",
            [this](const std_srvs::srv::Trigger::Request::SharedPtr,
                   std_srvs::srv::Trigger::Response::SharedPtr res) {
                nudge(-1.0, *res);
            });

        // MoveGroupInterface's CurrentStateMonitor does not reliably receive
        // /joint_states in this MoveIt Humble stack, so getCurrentState()
        // returns null. Mirror painting_executor: run a dedicated node
        // subscribed to joint feedback on its own executor thread and build
        // the current RobotState from it (see currentState()).
        // use_global_arguments(false) keeps the launch node remap from
        // renaming/colliding this side node.
        const auto joint_states_topic =
            node_->get_parameter("joint_states_topic").as_string();
        state_node_ = std::make_shared<rclcpp::Node>(
            "teach_nudge_joint_state_monitor",
            rclcpp::NodeOptions().use_global_arguments(false));
        joint_state_sub_ =
            state_node_->create_subscription<sensor_msgs::msg::JointState>(
                joint_states_topic, rclcpp::QoS(10),
                [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
                    if (msg->name.size() != msg->position.size() ||
                        !std::all_of(msg->position.begin(),
                                     msg->position.end(), [](double value) {
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
                        have_joint_state_ = true;
                    }
                    joint_state_cv_.notify_all();
                });
        state_executor_ =
            std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
        state_executor_->add_node(state_node_);
        state_thread_ = std::thread([this]() { state_executor_->spin(); });

        RCLCPP_INFO(
            node_->get_logger(),
            "Teach nudge ready: ~/nudge_in steps the pen along its axis "
            "toward the paper, ~/nudge_out away; step size = nudge_step_mm "
            "(ros2 param set). Requires joint_trajectory_controller active "
            "(freedrive OFF). Test the direction sign well clear of the "
            "paper first.");
    }

    ~TeachNudge()
    {
        if (state_executor_) {
            state_executor_->cancel();
        }
        if (state_thread_.joinable()) {
            state_thread_.join();
        }
    }

private:
    void nudge(double sign, std_srvs::srv::Trigger::Response &res)
    {
        std::unique_lock<std::mutex> lock(busy_, std::try_to_lock);
        if (!lock.owns_lock()) {
            fail(res, "A nudge is already in progress");
            return;
        }

        double step_mm = node_->get_parameter("nudge_step_mm").as_double();
        if (!std::isfinite(step_mm) || step_mm < kMinStepMm ||
            step_mm > kMaxStepMm) {
            fail(res, "nudge_step_mm " + std::to_string(step_mm) +
                          " outside [" + std::to_string(kMinStepMm) + ", " +
                          std::to_string(kMaxStepMm) + "] mm, refusing");
            return;
        }
        const double jump_threshold =
            node_->get_parameter("jump_threshold").as_double();
        if (!std::isfinite(jump_threshold) || jump_threshold <= 0.0) {
            fail(res, "jump_threshold must be > 0 (0 disables the "
                      "configuration-flip guard), refusing");
            return;
        }
        const double vel =
            node_->get_parameter("velocity_scaling").as_double();
        const double acc =
            node_->get_parameter("acceleration_scaling").as_double();
        const auto rpy =
            node_->get_parameter("tool_offset_rpy").as_double_array();
        const auto ee_frame = node_->get_parameter("ee_frame").as_string();
        if (rpy.size() != 3) {
            fail(res, "tool_offset_rpy must have 3 elements");
            return;
        }

        moveit::core::RobotState state(group_.getRobotModel());
        state.setToDefaultValues();
        std::string state_err;
        if (!currentState(state, state_err)) {
            fail(res, state_err);
            return;
        }
        if (!state.knowsFrameTransform(ee_frame)) {
            fail(res, "Frame '" + ee_frame + "' is not in the robot model");
            return;
        }
        const std::string tip_link = group_.getEndEffectorLink();
        if (tip_link.empty()) {
            fail(res, "Planning group has no end-effector link");
            return;
        }

        // Pen axis: tool_offset_rpy applied to +Z of ee_frame, expressed
        // in the planning frame via the current state.
        tf2::Quaternion q_tool;
        q_tool.setRPY(rpy[0], rpy[1], rpy[2]);
        const tf2::Vector3 axis_ee =
            tf2::Matrix3x3(q_tool) * tf2::Vector3(0.0, 0.0, 1.0);
        const Eigen::Isometry3d ee_tf = state.getFrameTransform(ee_frame);
        const Eigen::Vector3d dir =
            (ee_tf.rotation() *
             Eigen::Vector3d(axis_ee.x(), axis_ee.y(), axis_ee.z()))
                .normalized();

        const Eigen::Isometry3d tip_tf = state.getFrameTransform(tip_link);
        const Eigen::Vector3d target_pos =
            tip_tf.translation() + sign * (step_mm / 1000.0) * dir;
        const Eigen::Quaterniond tip_q(tip_tf.rotation());

        geometry_msgs::msg::Pose target;
        target.position.x = target_pos.x();
        target.position.y = target_pos.y();
        target.position.z = target_pos.z();
        target.orientation.x = tip_q.x();
        target.orientation.y = tip_q.y();
        target.orientation.z = tip_q.z();
        target.orientation.w = tip_q.w();

        group_.setStartState(state);
        moveit_msgs::msg::RobotTrajectory traj;
        const double eef_step = std::max(step_mm / 4000.0, 1e-4);
        const double fraction = group_.computeCartesianPath(
            { target }, eef_step, jump_threshold, traj);
        if (fraction < 0.999) {
            fail(res, "Cartesian step only " +
                          std::to_string(fraction * 100.0) +
                          "% feasible (obstacle or IK configuration flip), "
                          "not moving");
            return;
        }

        // computeCartesianPath ignores velocity scaling, so retime. Start
        // state from the trajectory's own first waypoint, not
        // getCurrentState() (same MoveIt-Humble rule as the executor).
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
        trajectory_processing::TimeOptimalTrajectoryGeneration totg(0.01,
                                                                    0.02);
        if (!totg.computeTimeStamps(rt, vel, acc)) {
            fail(res, "Trajectory retiming failed");
            return;
        }
        rt.getRobotTrajectoryMsg(traj);

        if (group_.execute(traj) !=
            moveit::core::MoveItErrorCode::SUCCESS) {
            fail(res, "Execution failed (is joint_trajectory_controller "
                      "active and freedrive off?)");
            return;
        }

        char msg[128];
        std::snprintf(msg, sizeof(msg),
                      "Nudged %s %.2f mm along the pen axis",
                      sign > 0 ? "IN" : "OUT", step_mm);
        res.success = true;
        res.message = msg;
        RCLCPP_INFO(node_->get_logger(), "%s", msg);
    }

    void fail(std_srvs::srv::Trigger::Response &res, const std::string &why)
    {
        res.success = false;
        res.message = why;
        RCLCPP_ERROR(node_->get_logger(), "%s", why.c_str());
    }

    // Fill `out` from the dedicated joint-state monitor's latest feedback
    // (not group_.getCurrentState() — MoveGroupInterface's CurrentStateMonitor
    // does not reliably receive /joint_states in this MoveIt Humble stack, the
    // same reason painting_executor keeps its own feedback). Returns false with
    // a message in `err` if no fresh feedback arrives or a group joint is
    // missing.
    bool currentState(moveit::core::RobotState &out, std::string &err)
    {
        std::vector<std::string> names;
        std::vector<double> positions;
        {
            std::unique_lock<std::mutex> lock(joint_state_mutex_);
            const auto fresh = [this]() {
                return have_joint_state_ &&
                       std::chrono::steady_clock::now() -
                               joint_state_received_at_ <
                           std::chrono::seconds(2);
            };
            if (!fresh() &&
                !joint_state_cv_.wait_for(lock, std::chrono::seconds(5),
                                          fresh)) {
                err = "No current robot state (joint states not streaming, "
                      "or move_group not running)";
                return false;
            }
            names = joint_state_names_;
            positions = joint_state_positions_;
        }

        const auto *jmg =
            out.getRobotModel()->getJointModelGroup(group_.getName());
        for (const auto &required : jmg->getVariableNames()) {
            if (std::find(names.begin(), names.end(), required) ==
                names.end()) {
                err = "Joint feedback omits required joint '" + required + "'";
                return false;
            }
        }
        try {
            out.setVariablePositions(names, positions);
        } catch (const std::exception &error) {
            err = std::string("Invalid joint feedback: ") + error.what();
            return false;
        }
        out.update();
        return true;
    }

    template <typename T>
    void declareIfAbsent(const std::string &name, const T &default_value)
    {
        if (!node_->has_parameter(name)) {
            node_->declare_parameter(name, default_value);
        }
    }

    rclcpp::Node::SharedPtr node_;
    moveit::planning_interface::MoveGroupInterface group_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr in_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr out_srv_;
    std::mutex busy_;

    // Dedicated joint-feedback monitor, spun on its own thread (see
    // currentState()). Torn down before the other members in ~TeachNudge.
    rclcpp::Node::SharedPtr state_node_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr
        joint_state_sub_;
    rclcpp::executors::SingleThreadedExecutor::SharedPtr state_executor_;
    std::thread state_thread_;
    std::mutex joint_state_mutex_;
    std::condition_variable joint_state_cv_;
    std::vector<std::string> joint_state_names_;
    std::vector<double> joint_state_positions_;
    std::chrono::steady_clock::time_point joint_state_received_at_;
    bool have_joint_state_{ false };
};

} // namespace

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    // Service + parameter node: receives the launch-provided robot_description
    // / robot_description_semantic and nudge_* params, hosts /teach_nudge/*,
    // and is the node WE spin. auto-declare so launch overrides land on it.
    auto node = std::make_shared<rclcpp::Node>(
        "teach_nudge",
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(
            true));

    // MoveGroupInterface builds its RobotModel from robot_description /
    // _semantic on ITS node. Forward them from the launch-populated params
    // above; a bare `ros2 run` provides neither, so fail with a clear hint
    // instead of MoveIt's cryptic "Unable to parse SRDF".
    const std::string urdf =
        node->has_parameter("robot_description")
            ? node->get_parameter("robot_description").as_string()
            : std::string();
    const std::string srdf =
        node->has_parameter("robot_description_semantic")
            ? node->get_parameter("robot_description_semantic").as_string()
            : std::string();
    if (urdf.empty() || srdf.empty()) {
        RCLCPP_FATAL(
            node->get_logger(),
            "robot_description/robot_description_semantic are empty. Start this "
            "node with 'ros2 launch robross_painter teach_nudge.launch.py' "
            "(with move_group running), not 'ros2 run'.");
        rclcpp::shutdown();
        return 1;
    }

    // MoveGroupInterface (Humble) spins its node on an internal executor, so
    // give it a private node: blocking inside our service callbacks (on node,
    // spun below) then cannot starve its action responses. use_global_arguments
    // (false) keeps the launch __node:=teach_nudge remap from renaming and
    // colliding this node; the robot model is injected explicitly as overrides.
    auto move_node = std::make_shared<rclcpp::Node>(
        "teach_nudge_move_client",
        rclcpp::NodeOptions()
            .use_global_arguments(false)
            .automatically_declare_parameters_from_overrides(true)
            .parameter_overrides(
                { rclcpp::Parameter("robot_description", urdf),
                  rclcpp::Parameter("robot_description_semantic", srdf) }));

    int ret = 0;
    try {
        TeachNudge nudge(node, move_node);
        rclcpp::spin(node);
    } catch (const std::exception &error) {
        RCLCPP_FATAL(node->get_logger(), "Teach nudge startup failed: %s",
                     error.what());
        ret = 1;
    }
    rclcpp::shutdown();
    return ret;
}
