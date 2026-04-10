/**
 * @file bridge_node.cpp
 * @brief Aubo Robot ROS2 Bridge Node Implementation
 */

#include "aubo_bridge/bridge_node.hpp"

#include <cmath>
#include <memory>
#include <chrono>

using namespace std::chrono_literals;

namespace aubo_bridge
{

AuboBridgeNode::AuboBridgeNode()
  : Node("aubo_bridge"),
    robot_host_(this->declare_parameter("robot_host", DEFAULT_HOST)),
    robot_port_(this->declare_parameter("robot_port", DEFAULT_PORT)),
    connected_(false),
    initialized_(false),
    stop_monitor_(false)
{
  RCLCPP_INFO(this->get_logger(), "Aubo Bridge Node initializing...");

  // Initialize robot service interface
  robot_service_ = std::make_unique<ServiceInterface>();

  // Publishers
  event_pub_ = this->create_publisher<aubo_bridge_msgs::msg::RobotEvent>(
    "/aubo/events", 10);
  joint_state_pub_ = this->create_publisher<aubo_bridge_msgs::msg::JointStateEx>(
    "/aubo/joint_states_ex", 10);
  status_pub_ = this->create_publisher<aubo_bridge_msgs::msg::RobotStatus>(
    "/aubo/status", 10);
  basic_joint_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
    "/joint_states", 10);

  // Subscriptions
  traj_sub_ = this->create_subscription<aubo_bridge_msgs::msg::TrajectoryCommand>(
    "/aubo/trajectory_command", 10,
    std::bind(&AuboBridgeNode::onTrajectoryCommand, this, std::placeholders::_1));

  joint_move_sub_ = this->create_subscription<aubo_bridge_msgs::msg::TrajectoryPoint>(
    "/aubo/joint_move_command", 10,
    std::bind(&AuboBridgeNode::onJointMoveCommand, this, std::placeholders::_1));

  // Services
  move_pose_srv_ = this->create_service<aubo_bridge_msgs::srv::MoveToPose>(
    "/aubo/move_to_pose",
    std::bind(&AuboBridgeNode::onMoveToPose, this,
      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

  move_joint_srv_ = this->create_service<aubo_bridge_msgs::srv::MoveToJointAngles>(
    "/aubo/move_to_joint_angles",
    std::bind(&AuboBridgeNode::onMoveToJointAngles, this,
      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

  clear_error_srv_ = this->create_service<aubo_bridge_msgs::srv::ClearError>(
    "/aubo/clear_error",
    std::bind(&AuboBridgeNode::onClearError, this,
      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

  // Attempt robot initialization
  if (initializeRobot())
  {
    // Start state monitoring thread
    monitor_thread_ = std::thread(&AuboBridgeNode::stateMonitorLoop, this);
  }
  else
  {
    RCLCPP_WARN(this->get_logger(), "Robot not connected. Will retry on demand.");
  }

  RCLCPP_INFO(this->get_logger(), "Aubo Bridge Node initialized.");
}

AuboBridgeNode::~AuboBridgeNode()
{
  stop_monitor_ = true;
  if (monitor_thread_.joinable())
  {
    monitor_thread_.join();
  }
  shutdownRobot();
}

bool AuboBridgeNode::initializeRobot()
{
  std::lock_guard<std::mutex> lock(robot_mutex_);

  int ret = robot_service_->robotServiceLogin(
    robot_host_.c_str(), robot_port_, "aubo", "123456");

  if (ret != aubo_robot_namespace::InterfaceCallSuccCode)
  {
    RCLCPP_ERROR(this->get_logger(), "Robot login failed.");
    connected_ = false;
    return false;
  }

  RCLCPP_INFO(this->get_logger(), "Robot login successful.");
  connected_ = true;

  // Initialize robot
  aubo_robot_namespace::ROBOT_SERVICE_STATE result;
  aubo_robot_namespace::ToolDynamicsParam toolDynParam{};
  memset(&toolDynParam, 0, sizeof(toolDynParam));

  ret = robot_service_->rootServiceRobotStartup(
    toolDynParam,     // tool dynamics
    6,               // collision level
    true,            // allow reading poses
    true,            // default
    1000,            // default
    result);         // initialization result

  if (ret != aubo_robot_namespace::InterfaceCallSuccCode)
  {
    RCLCPP_ERROR(this->get_logger(), "Robot initialization failed.");
    initialized_ = false;
    return false;
  }

  // Register callbacks
  robot_service_->robotServiceRegisterRealTimeRoadPointCallback(
    &AuboBridgeNode::waypointCallback, this);
  robot_service_->robotServiceRegisterRealTimeEndSpeedCallback(
    &AuboBridgeNode::endSpeedCallback, this);
  robot_service_->robotServiceRegisterRobotEventInfoCallback(
    &AuboBridgeNode::eventCallback, this);

  initialized_ = true;
  RCLCPP_INFO(this->get_logger(), "Robot initialized successfully.");
  return true;
}

void AuboBridgeNode::shutdownRobot()
{
  std::lock_guard<std::mutex> lock(robot_mutex_);

  if (initialized_)
  {
    robot_service_->robotServiceRobotShutdown();
    initialized_ = false;
  }

  if (connected_)
  {
    robot_service_->robotServiceLogout();
    connected_ = false;
  }
}

void AuboBridgeNode::stateMonitorLoop()
{
  rclcpp::Rate rate(10);  // 10 Hz state publishing

  while (rclcpp::ok() && !stop_monitor_)
  {
    if (connected_ && initialized_)
    {
      publishJointState();
      publishRobotStatus();
    }
    rate.sleep();
  }
}

void AuboBridgeNode::publishJointState()
{
  std::lock_guard<std::mutex> lock(robot_mutex_);

  aubo_robot_namespace::JointStatus jointStatus[DOF];
  int ret = robot_service_->robotServiceGetRobotJointStatus(jointStatus, DOF);

  if (ret != aubo_robot_namespace::InterfaceCallSuccCode)
  {
    return;
  }

  // Get current joint angles
  aubo_robot_namespace::wayPoint_S currentWaypoint;
  robot_service_->robotServiceGetCurrentWaypointInfo(currentWaypoint);

  // Publish basic joint state
  sensor_msgs::msg::JointState basic_joint;
  basic_joint.header.stamp = this->now();
  basic_joint.name = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"};
  for (int i = 0; i < DOF; ++i)
  {
    basic_joint.position.push_back(currentWaypoint.jointpos[i]);
    basic_joint.velocity.push_back(0.0);  // jointvel not available directly
  }
  basic_joint_pub_->publish(basic_joint);

  // Publish extended joint state
  aubo_bridge_msgs::msg::JointStateEx joint_state_ex;
  for (int i = 0; i < DOF; ++i)
  {
    joint_state_ex.position[i] = currentWaypoint.jointpos[i];
    joint_state_ex.velocity[i] = 0.0;
    joint_state_ex.effort[i] = 0.0;
    joint_state_ex.joint_temperatures[i] = static_cast<double>(jointStatus[i].jointCurTemp);
    joint_state_ex.joint_currents[i] = static_cast<double>(jointStatus[i].jointCurrentI) / 1000.0;
    joint_state_ex.joint_error_flags[i] = (jointStatus[i].jointErrorNum != 0);
  }
  joint_state_pub_->publish(joint_state_ex);
}

void AuboBridgeNode::publishRobotStatus()
{
  std::lock_guard<std::mutex> lock(robot_mutex_);

  aubo_bridge_msgs::msg::RobotStatus status;

  if (!connected_)
  {
    status.robot_state = aubo_bridge_msgs::msg::RobotStatus::DISCONNECTED;
  }
  else if (!initialized_)
  {
    status.robot_state = aubo_bridge_msgs::msg::RobotStatus::BOOTING;
  }
  else
  {
    status.robot_state = aubo_bridge_msgs::msg::RobotStatus::IDLE;
  }

  // Get TCP pose
  aubo_robot_namespace::wayPoint_S wp;
  robot_service_->robotServiceGetCurrentWaypointInfo(wp);
  status.tool_position_x = wp.cartPos.position.x;
  status.tool_position_y = wp.cartPos.position.y;
  status.tool_position_z = wp.cartPos.position.z;

  status_pub_->publish(status);
}

void AuboBridgeNode::publishRobotEvent(const aubo_robot_namespace::RobotEventInfo &event)
{
  aubo_bridge_msgs::msg::RobotEvent ros_event;
  ros_event.event_type = static_cast<uint8_t>(event.eventType);
  ros_event.severity = 1;  // WARN by default
  ros_event.description = event.eventContent;
  ros_event.timestamp_nanosec = 0;  // Could use steady_clock

  event_pub_->publish(ros_event);
}

void AuboBridgeNode::onTrajectoryCommand(
  const aubo_bridge_msgs::msg::TrajectoryCommand::SharedPtr msg)
{
  RCLCPP_INFO(this->get_logger(), "Received trajectory command: %d", msg->command);

  switch (msg->command)
  {
    case aubo_bridge_msgs::msg::TrajectoryCommand::EXECUTE_TRAJECTORY:
      executeTrajectory(msg);
      break;
    case aubo_bridge_msgs::msg::TrajectoryCommand::STOP_TRAJECTORY:
      // TODO: Implement stop
      break;
    case aubo_bridge_msgs::msg::TrajectoryCommand::PAUSE_TRAJECTORY:
      // TODO: Implement pause
      break;
    case aubo_bridge_msgs::msg::TrajectoryCommand::RESUME_TRAJECTORY:
      // TODO: Implement resume
      break;
    default:
      RCLCPP_WARN(this->get_logger(), "Unknown trajectory command: %d", msg->command);
  }
}

void AuboBridgeNode::onJointMoveCommand(
  const aubo_bridge_msgs::msg::TrajectoryPoint::SharedPtr msg)
{
  executeJointMove(msg->joint_positions.data(), true);
}

int AuboBridgeNode::executeTrajectory(
  const aubo_bridge_msgs::msg::TrajectoryCommand::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(robot_mutex_);

  if (!initialized_)
  {
    RCLCPP_ERROR(this->get_logger(), "Robot not initialized");
    return -1;
  }

  // Set motion profile
  robot_service_->robotServiceInitGlobalMoveProfile();

  aubo_robot_namespace::JointVelcAccParam jointMaxAcc;
  for (int i = 0; i < DOF; ++i)
  {
    jointMaxAcc.jointPara[i] = msg->max_joint_acceleration;
  }
  robot_service_->robotServiceSetGlobalMoveJointMaxAcc(jointMaxAcc);

  aubo_robot_namespace::JointVelcAccParam jointMaxVelc;
  for (int i = 0; i < DOF; ++i)
  {
    jointMaxVelc.jointPara[i] = msg->max_joint_velocity;
  }
  robot_service_->robotServiceSetGlobalMoveJointMaxVelc(jointMaxVelc);

  // Execute trajectory waypoints
  int ret = aubo_robot_namespace::InterfaceCallSuccCode;
  for (const auto &point : msg->trajectory)
  {
    ret = robot_service_->robotServiceJointMove(
      const_cast<double *>(point.joint_positions.data()), true);

    if (ret != aubo_robot_namespace::InterfaceCallSuccCode)
    {
      RCLCPP_ERROR(this->get_logger(), "Trajectory execution failed at waypoint");
      return ret;
    }
  }

  RCLCPP_INFO(this->get_logger(), "Trajectory execution completed");
  return ret;
}

int AuboBridgeNode::executeJointMove(const double *joint_angles, bool blocking)
{
  std::lock_guard<std::mutex> lock(robot_mutex_);

  if (!initialized_)
  {
    RCLCPP_ERROR(this->get_logger(), "Robot not initialized");
    return -1;
  }

  int ret = robot_service_->robotServiceJointMove(
    const_cast<double *>(joint_angles), blocking);

  if (ret != aubo_robot_namespace::InterfaceCallSuccCode)
  {
    RCLCPP_ERROR(this->get_logger(), "Joint move failed");
    return ret;
  }

  return ret;
}

// Service handlers
void AuboBridgeNode::onMoveToPose(
  const std::shared_ptr<rmw_request_id_t>,
  const std::shared_ptr<aubo_bridge_msgs::srv::MoveToPose::Request> request,
  const std::shared_ptr<aubo_bridge_msgs::srv::MoveToPose::Response> response)
{
  RCLCPP_INFO(this->get_logger(), "Service: move_to_pose - using joint move");
  // Convert pose to joint angles using IK (simplified - would need IK solver)
  // For now, respond with not implemented
  (void)request;
  response->success = false;
  response->message = "Pose move not yet implemented - use joint angles";
}

void AuboBridgeNode::onMoveToJointAngles(
  const std::shared_ptr<rmw_request_id_t>,
  const std::shared_ptr<aubo_bridge_msgs::srv::MoveToJointAngles::Request> request,
  const std::shared_ptr<aubo_bridge_msgs::srv::MoveToJointAngles::Response> response)
{
  RCLCPP_INFO(this->get_logger(), "Service: move_to_joint_angles");

  int ret = executeJointMove(request->joint_angles.data(), request->blocking);

  response->success = (ret == aubo_robot_namespace::InterfaceCallSuccCode);
  response->message = response->success ? "Move completed" : "Move failed";
}

void AuboBridgeNode::onClearError(
  const std::shared_ptr<rmw_request_id_t>,
  const std::shared_ptr<aubo_bridge_msgs::srv::ClearError::Request>,
  const std::shared_ptr<aubo_bridge_msgs::srv::ClearError::Response> response)
{
  RCLCPP_INFO(this->get_logger(), "Service: clear_error - not implemented");
  response->success = false;
  response->message = "Clear error not implemented";
}

// Aubo SDK callbacks
void AuboBridgeNode::waypointCallback(
  const aubo_robot_namespace::wayPoint_S *waypoint, void *arg)
{
  (void)waypoint;
  (void)arg;
  // Could publish waypoint for trajectory recording
}

void AuboBridgeNode::endSpeedCallback(double speed, void *arg)
{
  (void)speed;
  (void)arg;
  // Could publish end-effector speed
}

void AuboBridgeNode::eventCallback(
  const aubo_robot_namespace::RobotEventInfo *event, void *arg)
{
  auto node = static_cast<AuboBridgeNode *>(arg);
  node->publishRobotEvent(*event);
}

}  // namespace aubo_bridge

// Main entry point
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<aubo_bridge::AuboBridgeNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
