#!/usr/bin/env python3
"""
Aubo Bridge Node - Pure Python ROS2 Node

This node provides a ROS2 interface to the Aubo robot using the Python binding.
It bridges the libpyauboi5 Python SDK with ROS2 topics/services for trajectory
control and event monitoring.

Actual robot control is performed via robotcontrol SDK.
"""

import sys
import os
import time
import threading
from math import pi

# Add Python binding to path (relative to this script's location)
SCRIPT_DIR = os.path.dirname(os.path.realpath(__file__))
AUBO_BRIDGE_DIR = os.path.dirname(SCRIPT_DIR)
PYTHON_BINDING_PATH = os.path.join(
    AUBO_BRIDGE_DIR,
    "libpyauboi5-v1.5.1.x64-for-python3.x",
    "python3.x",
)
sys.path.insert(0, PYTHON_BINDING_PATH)

import rclpy
from rclpy.node import Node

# Import Aubo Python SDK
try:
    import libpyauboi5
    from robotcontrol import (
        Auboi5Robot,
        RobotErrorType,
        RobotEventType,
        RobotError,
    )

    AUBO_SDK_AVAILABLE = True
except ImportError as e:
    print(f"Warning: Aubo Python SDK not available: {e}")
    AUBO_SDK_AVAILABLE = False

# Message imports
from aubo_bridge_msgs.msg import (
    TrajectoryCommand,
    TrajectoryPoint,
    RobotEvent,
    JointStateEx,
    RobotStatus,
)

# Service imports
from aubo_bridge_msgs.srv import (
    MoveToPose,
    MoveToJointAngles,
    ClearError,
    GetRobotInfo,
    MoveJoint,
    MoveLine,
)


class AuboBridgeNode(Node):
    """ROS2 Bridge Node for Aubo Robot using robotcontrol SDK."""

    def __init__(self):
        super().__init__("aubo_bridge")

        # Parameters
        self.declare_parameter("robot_host", "192.168.3.250")
        self.declare_parameter("robot_port", 8899)
        self.declare_parameter("collision_level", 6)

        self.robot_host = self.get_parameter("robot_host").value
        self.robot_port = self.get_parameter("robot_port").value
        self.collision_level = self.get_parameter("collision_level").value

        # Robot SDK handle
        self.robot: Auboi5Robot = None
        self.connected = False
        self.initialized = False
        self.lock = threading.Lock()

        # Publishers
        self.event_pub = self.create_publisher(RobotEvent, "/aubo/events", 10)
        self.joint_state_pub = self.create_publisher(
            JointStateEx, "/aubo/joint_states_ex", 10
        )
        self.status_pub = self.create_publisher(RobotStatus, "/aubo/status", 10)

        # Subscriptions
        self.traj_sub = self.create_subscription(
            TrajectoryCommand,
            "/aubo/trajectory_command",
            self.on_trajectory_command,
            10,
        )
        self.joint_move_sub = self.create_subscription(
            TrajectoryPoint,
            "/aubo/joint_move_command",
            self.on_joint_move_command,
            10,
        )

        # Services - original
        self.move_pose_srv = self.create_service(
            MoveToPose, "/aubo/move_to_pose", self.on_move_to_pose
        )
        self.move_joint_angles_srv = self.create_service(
            MoveToJointAngles,
            "/aubo/move_to_joint_angles",
            self.on_move_to_joint_angles,
        )
        self.clear_error_srv = self.create_service(
            ClearError, "/aubo/clear_error", self.on_clear_error
        )

        # Services - new SDK-level
        self.get_robot_info_srv = self.create_service(
            GetRobotInfo, "/aubo/get_robot_info", self.on_get_robot_info
        )
        self.move_joint_srv = self.create_service(
            MoveJoint, "/aubo/move_joint", self.on_move_joint
        )
        self.move_line_srv = self.create_service(
            MoveLine, "/aubo/move_line", self.on_move_line
        )

        # Initialize robot SDK
        if AUBO_SDK_AVAILABLE:
            self.initialize_robot()
            # Start state monitoring thread
            self.monitor_thread = threading.Thread(
                target=self.state_monitor_loop, daemon=True
            )
            self.monitor_thread.start()
        else:
            self.get_logger().error("Aubo SDK not available, node will not connect")

        self.get_logger().info(
            f"Aubo Bridge Node initialized (robot={self.robot_host}:{self.robot_port})"
        )

    def initialize_robot(self):
        """Initialize connection to robot via robotcontrol SDK."""
        try:
            self.get_logger().info("Initializing Aubo SDK...")
            result = Auboi5Robot.initialize()
            if result != RobotErrorType.RobotError_SUCC:
                self.get_logger().error(f"SDK init failed: {result}")
                return False

            self.robot = Auboi5Robot()
            handle = self.robot.create_context()
            self.get_logger().info(f"Created context: {handle}")

            self.get_logger().info(
                f"Connecting to {self.robot_host}:{self.robot_port}..."
            )
            result = self.robot.connect(self.robot_host, self.robot_port)
            if result != RobotErrorType.RobotError_SUCC:
                self.get_logger().error(f"Connect failed: {result}")
                return False

            self.connected = True
            self.get_logger().info("Robot connected successfully")

            # Startup robot
            self.get_logger().info("Starting up robot...")
            startup_result = self.robot.robot_startup(collision=self.collision_level)
            if startup_result != RobotErrorType.RobotError_SUCC:
                self.get_logger().warn(f"Robot startup returned: {startup_result}")
            else:
                self.initialized = True
                self.get_logger().info("Robot initialized and ready")

            # Initialize motion profile
            self.robot.init_profile()

            return True

        except Exception as e:
            self.get_logger().error(f"Failed to initialize robot: {e}")
            self.connected = False
            return False

    def state_monitor_loop(self):
        """Publish robot state periodically at 10 Hz."""
        rate = self.create_rate(10)
        while rclpy.ok() and self.connected:
            try:
                self.publish_joint_state()
                self.publish_robot_status()
            except Exception as e:
                self.get_logger().warn(f"State monitor error: {e}")
            rate.sleep()

    def publish_joint_state(self):
        """Publish current joint state from SDK."""
        if not self.robot:
            return

        msg = JointStateEx()
        msg.header.stamp = self.get_clock().now().to_msg()

        try:
            waypoint = self.robot.get_current_waypoint()
            if waypoint and "joint" in waypoint:
                msg.position = list(waypoint["joint"])
            msg.velocity = [0.0] * 6
            msg.effort = [0.0] * 6
            msg.joint_temperatures = [0.0] * 6
            msg.joint_currents = [0.0] * 6
            msg.joint_error_flags = [False] * 6
        except Exception as e:
            self.get_logger().warn(f"Failed to get joint state: {e}")

        self.joint_state_pub.publish(msg)

    def publish_robot_status(self):
        """Publish robot status."""
        msg = RobotStatus()
        if not self.connected:
            msg.robot_state = RobotStatus.DISCONNECTED
        elif not self.initialized:
            msg.robot_state = RobotStatus.BOOTING
        else:
            msg.robot_state = RobotStatus.IDLE

            # Fill in current TCP position
            try:
                waypoint = self.robot.get_current_waypoint()
                if waypoint and "pos" in waypoint:
                    msg.tool_position_x = waypoint["pos"][0]
                    msg.tool_position_y = waypoint["pos"][1]
                    msg.tool_position_z = waypoint["pos"][2]
            except Exception:
                pass

        self.status_pub.publish(msg)

    def on_trajectory_command(self, msg: TrajectoryCommand):
        """Handle trajectory command."""
        self.get_logger().info(f"Trajectory command: {msg.command}")

        if msg.command == TrajectoryCommand.EXECUTE_TRAJECTORY:
            self.execute_trajectory(msg)
        elif msg.command == TrajectoryCommand.STOP_TRAJECTORY:
            self.stop_trajectory()
        elif msg.command == TrajectoryCommand.PAUSE_TRAJECTORY:
            self.pause_trajectory()
        elif msg.command == TrajectoryCommand.RESUME_TRAJECTORY:
            self.resume_trajectory()

    def on_joint_move_command(self, msg: TrajectoryPoint):
        """Handle direct joint move command."""
        self.get_logger().info(f"Joint move command: {msg.joint_positions}")
        with self.lock:
            try:
                self.robot.move_joint(tuple(msg.joint_positions), issync=True)
                self.get_logger().info("Joint move completed")
            except Exception as e:
                self.get_logger().error(f"Joint move failed: {e}")

    def execute_trajectory(self, msg: TrajectoryCommand):
        """Execute a trajectory via SDK."""
        with self.lock:
            try:
                for i, point in enumerate(msg.trajectory):
                    self.get_logger().info(
                        f"Waypoint {i}: {point.joint_positions}"
                    )
                    self.robot.move_joint(
                        tuple(point.joint_positions), issync=True
                    )
                self.get_logger().info("Trajectory completed")
            except Exception as e:
                self.get_logger().error(f"Trajectory error: {e}")

    def stop_trajectory(self):
        """Stop current trajectory."""
        self.get_logger().info("Stop trajectory")

    def pause_trajectory(self):
        """Pause current trajectory."""
        self.get_logger().info("Pause trajectory")

    def resume_trajectory(self):
        """Resume paused trajectory."""
        self.get_logger().info("Resume trajectory")

    def on_move_to_pose(self, request, response):
        """Service handler for move to pose (inverse kinematics + joint move)."""
        self.get_logger().info("Service: move_to_pose (IK -> joint move)")
        response.success = False
        response.message = "Use /aubo/move_joint or /aubo/move_line instead"
        return response

    def on_move_to_joint_angles(self, request, response):
        """Service handler for move to joint angles."""
        self.get_logger().info(f"Service: move_to_joint_angles: {request.joint_angles}")
        try:
            with self.lock:
                if not self.connected:
                    response.success = False
                    response.message = "Robot not connected"
                    return response

                # Set motion parameters
                max_acc = tuple(request.max_acceleration) * 6 if hasattr(request, 'max_acceleration') and len(request.max_acceleration) == 1 else tuple(request.max_acceleration)
                max_vel = tuple(request.max_velocity) * 6 if hasattr(request, 'max_velocity') and len(request.max_velocity) == 1 else tuple(request.max_velocity)
                self.robot.set_joint_maxacc(max_acc)
                self.robot.set_joint_maxvelc(max_vel)

                # Execute move
                result = self.robot.move_joint(
                    tuple(request.joint_angles), issync=True
                )

                if result == RobotErrorType.RobotError_SUCC:
                    response.success = True
                    response.message = "Move completed"
                else:
                    response.success = False
                    response.message = f"Move failed: {result}"

        except Exception as e:
            response.success = False
            response.message = str(e)

        return response

    def on_clear_error(self, request, response):
        """Service handler for clear error."""
        self.get_logger().info("Service: clear_error")
        response.success = False
        response.message = "Clear error not implemented"
        return response

    def on_get_robot_info(self, request, response):
        """Get robot info: joint status, max acc/vel, current waypoint."""
        self.get_logger().info("Service: get_robot_info")
        try:
            if not self.connected:
                response.success = False
                response.message = "Robot not connected"
                return response

            waypoint = self.robot.get_current_waypoint()
            if waypoint:
                response.current_joint = list(waypoint.get("joint", [0.0] * 6))
                response.current_pos = list(waypoint.get("pos", [0.0] * 3))
                response.current_ori = list(waypoint.get("ori", [1.0, 0.0, 0.0, 0.0]))
            else:
                response.current_joint = [0.0] * 6
                response.current_pos = [0.0] * 3
                response.current_ori = [1.0, 0.0, 0.0, 0.0]

            response.joint_maxacc = list(self.robot.get_joint_maxacc())
            response.joint_maxvelc = list(self.robot.get_joint_maxvelc())

            joint_status = self.robot.get_joint_status()
            response.joint_status = int(joint_status) if joint_status else 0

            response.success = True
            response.message = "Robot info retrieved"

        except Exception as e:
            response.success = False
            response.message = str(e)

        return response

    def on_move_joint(self, request, response):
        """Move to target joint angles (joint space motion)."""
        self.get_logger().info(f"Service: move_joint: {list(request.target_joint)}")
        try:
            if not self.connected:
                response.success = False
                response.message = "Robot not connected"
                return response

            # Set motion parameters
            if request.max_acc:
                self.robot.set_joint_maxacc(tuple(request.max_acc))
            if request.max_vel:
                self.robot.set_joint_maxvelc(tuple(request.max_vel))

            # Optional: set base coordinate
            self.robot.set_base_coord()

            if request.enable_move:
                with self.lock:
                    result = self.robot.move_joint(
                        tuple(request.target_joint), issync=True
                    )

                if result == RobotErrorType.RobotError_SUCC:
                    response.success = True
                    response.message = "Move completed"
                    waypoint = self.robot.get_current_waypoint()
                    if waypoint and "joint" in waypoint:
                        response.result_joint = list(waypoint["joint"])
                    else:
                        response.result_joint = list(request.target_joint)
                else:
                    response.success = False
                    response.message = f"Move failed: {result}"
                    response.result_joint = [0.0] * 6
            else:
                # IK only
                response.success = True
                response.message = "IK computed (move disabled)"
                response.result_joint = list(request.target_joint)

        except Exception as e:
            response.success = False
            response.message = str(e)
            response.result_joint = [0.0] * 6

        return response

    def on_move_line(self, request, response):
        """Move along a straight line in Cartesian space."""
        self.get_logger().info(f"Service: move_line: pos={list(request.relative_pos)}")
        try:
            if not self.connected:
                response.success = False
                response.message = "Robot not connected"
                return response

            # Set motion parameters
            if request.max_acc:
                self.robot.set_joint_maxacc(tuple(request.max_acc))
            if request.max_vel:
                self.robot.set_joint_maxvelc(tuple(request.max_vel))

            # Set base coordinate
            self.robot.set_base_coord()

            # Get current waypoint
            waypoint = self.robot.get_current_waypoint()
            if not waypoint:
                response.success = False
                response.message = "Failed to get current waypoint"
                return response

            current_joint = tuple(waypoint["joint"])
            current_pos = tuple(waypoint["pos"])
            current_ori = tuple(waypoint["ori"])

            # Compute target position
            target_pos = (
                current_pos[0] + request.relative_pos[0],
                current_pos[1] + request.relative_pos[1],
                current_pos[2] + request.relative_pos[2],
            )

            # Use provided orientation or keep current
            if all(v == 0 for v in request.relative_ori[1:]):
                # Only w=1 means no rotation change
                target_ori = current_ori
            else:
                target_ori = tuple(request.relative_ori)

            response.target_pos = list(target_pos)
            response.target_ori = list(target_ori)

            # Inverse kinematics
            ik_result = self.robot.inverse_kin(current_joint, target_pos, target_ori)
            if not ik_result or "joint" not in ik_result:
                response.success = False
                response.message = "Inverse kinematics failed - target unreachable"
                response.result_joint = [0.0] * 6
                return response

            target_joint = tuple(ik_result["joint"])
            response.result_joint = list(target_joint)

            if request.enable_move:
                with self.lock:
                    result = self.robot.move_line(target_joint)

                if result == RobotErrorType.RobotError_SUCC:
                    response.success = True
                    response.message = "Line move completed"
                else:
                    response.success = False
                    response.message = f"Move failed: {result}"
            else:
                response.success = True
                response.message = "IK computed (move disabled)"

        except Exception as e:
            response.success = False
            response.message = str(e)
            response.result_joint = [0.0] * 6
            response.target_pos = [0.0] * 3
            response.target_ori = [1.0, 0.0, 0.0, 0.0]

        return response

    def shutdown(self):
        """Clean shutdown of robot connection."""
        if self.robot and self.connected:
            try:
                self.robot.disconnect()
                self.robot.uninitialize()
            except Exception as e:
                self.get_logger().info(f"Robot shutdown: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = AuboBridgeNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
