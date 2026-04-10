/**
 * @file bridge_node.hpp
 * @brief Aubo Robot ROS2 Bridge Node - Unified C++/Python control
 *
 * Bridges Aubo C++ SDK with ROS2 topics for trajectory control and event monitoring.
 * Target application: Material handling (搬运)
 */

#ifndef AUBO_BRIDGE_BRIDGE_NODE_HPP
#define AUBO_BRIDGE_BRIDGE_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <aubo_bridge_msgs/msg/trajectory_command.hpp>
#include <aubo_bridge_msgs/msg/trajectory_point.hpp>
#include <aubo_bridge_msgs/msg/robot_event.hpp>
#include <aubo_bridge_msgs/msg/joint_state_ex.hpp>
#include <aubo_bridge_msgs/msg/robot_status.hpp>
#include <aubo_bridge_msgs/srv/move_to_pose.hpp>
#include <aubo_bridge_msgs/srv/move_to_joint_angles.hpp>
#include <aubo_bridge_msgs/srv/clear_error.hpp>

#include "serviceinterface.h"
#include "AuboRobotMetaType.h"
#include <atomic>
#include <memory>
#include <thread>
#include <mutex>

namespace aubo_bridge
{

constexpr char DEFAULT_HOST[] = "127.0.0.1";
constexpr int DEFAULT_PORT = 8899;
constexpr int DOF = 6;  // Degrees of freedom

class AuboBridgeNode : public rclcpp::Node
{
public:
  AuboBridgeNode();
  ~AuboBridgeNode();

  /// Initialize connection to robot
  bool initializeRobot();

  /// Shutdown robot connection
  void shutdownRobot();

private:
  // ROS2 subscriptions
  void onTrajectoryCommand(const aubo_bridge_msgs::msg::TrajectoryCommand::SharedPtr msg);
  void onJointMoveCommand(const aubo_bridge_msgs::msg::TrajectoryPoint::SharedPtr msg);

  // ROS2 service handlers
  void onMoveToPose(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<aubo_bridge_msgs::srv::MoveToPose::Request> request,
    const std::shared_ptr<aubo_bridge_msgs::srv::MoveToPose::Response> response);

  void onMoveToJointAngles(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<aubo_bridge_msgs::srv::MoveToJointAngles::Request> request,
    const std::shared_ptr<aubo_bridge_msgs::srv::MoveToJointAngles::Response> response);

  void onClearError(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<aubo_bridge_msgs::srv::ClearError::Request> request,
    const std::shared_ptr<aubo_bridge_msgs::srv::ClearError::Response> response);

  // Aubo SDK callbacks
  static void waypointCallback(const aubo_robot_namespace::wayPoint_S *waypoint, void *arg);
  static void endSpeedCallback(double speed, void *arg);
  static void eventCallback(const aubo_robot_namespace::RobotEventInfo *event, void *arg);

  // Internal helpers
  void publishJointState();
  void publishRobotStatus();
  void publishRobotEvent(const aubo_robot_namespace::RobotEventInfo &event);
  void stateMonitorLoop();

  int executeTrajectory(const aubo_bridge_msgs::msg::TrajectoryCommand::SharedPtr msg);
  int executeJointMove(const double *joint_angles, bool blocking);

  // Robot connection
  std::unique_ptr<ServiceInterface> robot_service_;
  std::string robot_host_;
  int robot_port_;
  std::atomic<bool> connected_;
  std::atomic<bool> initialized_;

  // Subscriptions
  rclcpp::Subscription<aubo_bridge_msgs::msg::TrajectoryCommand>::SharedPtr traj_sub_;
  rclcpp::Subscription<aubo_bridge_msgs::msg::TrajectoryPoint>::SharedPtr joint_move_sub_;

  // Publishers
  rclcpp::Publisher<aubo_bridge_msgs::msg::RobotEvent>::SharedPtr event_pub_;
  rclcpp::Publisher<aubo_bridge_msgs::msg::JointStateEx>::SharedPtr joint_state_pub_;
  rclcpp::Publisher<aubo_bridge_msgs::msg::RobotStatus>::SharedPtr status_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr basic_joint_pub_;

  // Services
  rclcpp::Service<aubo_bridge_msgs::srv::MoveToPose>::SharedPtr move_pose_srv_;
  rclcpp::Service<aubo_bridge_msgs::srv::MoveToJointAngles>::SharedPtr move_joint_srv_;
  rclcpp::Service<aubo_bridge_msgs::srv::ClearError>::SharedPtr clear_error_srv_;

  // State monitoring
  std::thread monitor_thread_;
  std::atomic<bool> stop_monitor_;
  std::mutex robot_mutex_;
};

}  // namespace aubo_bridge

#endif  // AUBO_BRIDGE_BRIDGE_NODE_HPP
