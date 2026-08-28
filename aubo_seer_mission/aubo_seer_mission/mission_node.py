#!/usr/bin/env python3
"""Joint mission node: SEER AGV navigation + AUBO manipulator actions.

Executes a config-driven chain of stations. For each station:
  1. Navigate the AGV to agv_target_id via the SEER TCP SDK (AgvControl),
     polling task_status until it reports arrival (4/5) or timeout.
  2. Execute arm_actions sequentially via aubo_bridge services
     (/aubo/move_to_joint_angles, /aubo/move_to_pose), then return to arm_home.

Interfaces:
  /mission/start  std_srvs/Trigger  - start the mission chain
  /mission/stop   std_srvs/Trigger  - cancel mission, arm to home position
  /mission/status aubo_interfaces/AgvStatus - mission state machine state
  /mission/progress std_msgs/String - human-readable progress
"""

import math
import os
import threading

import yaml

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException

from std_srvs.srv import Trigger
from std_msgs.msg import String
from aubo_interfaces.msg import AgvStatus
from aubo_bridge_msgs.msg import RobotStatus
from aubo_bridge_msgs.srv import MoveToJointAngles, MoveToPose

try:
    from seer_robot_driver.AgvControl import AgvControl
    SEER_SDK_AVAILABLE = True
except ImportError as e:
    SEER_SDK_AVAILABLE = False
    print(f"Warning: seer_robot_driver not available: {e}")

# SEER task_status values, see seer_robot_driver/scripts/back_to_charge.py
SEER_TASK_DONE = (4, 5)

# RobotStatus.robot_state constants (aubo_bridge_msgs/RobotStatus)
ARM_STATE_IDLE = 2
ARM_STATE_ERROR = 4
ARM_STATE_EMERGENCY_STOP = 5

# Default config shipped with the package (resolved via ament index; falls back
# to the source tree when running directly from the repo)
def _default_mission_yaml():
    try:
        from ament_index_python.packages import get_package_share_directory

        return os.path.join(
            get_package_share_directory("aubo_seer_mission"), "config", "mission.yaml"
        )
    except Exception:
        return os.path.join(
            os.path.dirname(os.path.dirname(os.path.realpath(__file__))),
            "config",
            "mission.yaml",
        )


DEFAULT_MISSION_YAML = _default_mission_yaml()


class MissionNode(Node):
    def __init__(self):
        super().__init__("mission_node")

        self.declare_parameter("mission_yaml", DEFAULT_MISSION_YAML)
        self.declare_parameter("poll_interval", 0.5)
        self.declare_parameter("arm_service_timeout", 10.0)

        yaml_path = self.get_parameter("mission_yaml").value
        self.poll_interval = self.get_parameter("poll_interval").value
        self.arm_service_timeout = self.get_parameter("arm_service_timeout").value

        with open(yaml_path, "r") as f:
            self.config = yaml.safe_load(f)
        self.stations = self.config.get("stations", [])
        self.default_arm_home = self.config.get(
            "default_arm_home", [0.0, 0.0, 90.0, 0.0, 90.0, 0.0]
        )

        self.agv = AgvControl() if SEER_SDK_AVAILABLE else None

        self.arm_state = None
        self.stop_requested = False
        self.mission_thread = None
        self.mission_lock = threading.Lock()

        self.status_pub = self.create_publisher(AgvStatus, "/mission/status", 10)
        self.progress_pub = self.create_publisher(String, "/mission/progress", 10)

        self.create_subscription(
            RobotStatus,
            "/aubo/status",
            self._on_arm_status,
            10,
        )

        self.move_joint_client = self.create_client(MoveToJointAngles, "/aubo/move_to_joint_angles")
        self.move_pose_client = self.create_client(MoveToPose, "/aubo/move_to_pose")

        self.start_srv = self.create_service(Trigger, "/mission/start", self._on_start)
        self.stop_srv = self.create_service(Trigger, "/mission/stop", self._on_stop)

        self._publish_status(AgvStatus.WAITING, "ready")

    # ---------------- status helpers ----------------

    def _publish_status(self, state, text):
        msg = AgvStatus()
        msg.status = state
        msg.message = text
        self.status_pub.publish(msg)
        progress = String()
        progress.data = text
        self.progress_pub.publish(progress)
        self.get_logger().info(text)

    def _on_arm_status(self, msg):
        self.arm_state = msg.robot_state

    # ---------------- service callbacks ----------------

    def _on_start(self, request, response):
        with self.mission_lock:
            if self.mission_thread is not None and self.mission_thread.is_alive():
                response.success = False
                response.message = "mission already running"
                return response
            self.stop_requested = False
            self.mission_thread = threading.Thread(target=self._run_mission, daemon=True)
            self.mission_thread.start()
        response.success = True
        response.message = "mission started"
        return response

    def _on_stop(self, request, response):
        self.stop_requested = True
        response.success = True
        response.message = "stop requested; arm will return home"
        return response

    # ---------------- mission execution ----------------

    def _run_mission(self):
        try:
            self._run_mission_inner()
        except Exception as e:  # noqa: BLE001 - mission thread must not die silently
            self._publish_status(AgvStatus.FAILED, f"mission crashed: {e}")

    def _run_mission_inner(self):
        if not SEER_SDK_AVAILABLE:
            self._publish_status(AgvStatus.FAILED, "seer_robot_driver not available")
            return
        if not self.stations:
            self._publish_status(AgvStatus.FAILED, "no stations configured")
            return

        for idx, station in enumerate(self.stations):
            if self.stop_requested:
                self._arm_to_home(station)
                self._publish_status(AgvStatus.CANCELED, "mission canceled by request")
                return

            name = station.get("name", f"station_{idx}")
            if not self._navigate(station):
                self._publish_status(AgvStatus.FAILED, f"{name}: navigation failed")
                return
            if not self._execute_arm_actions(station):
                self._publish_status(
                    AgvStatus.FAILED, f"{name}: arm action failed (arm_state={self.arm_state})"
                )
                return
            self._publish_status(AgvStatus.RUNNING, f"{name}: completed ({idx + 1}/{len(self.stations)})")

        self._publish_status(AgvStatus.COMPLETED, "mission completed")

    def _navigate(self, station):
        name = station.get("name", "station")
        target_id = str(station.get("agv_target_id", ""))
        source_id = str(station.get("agv_source_id", ""))
        timeout = float(station.get("navigation_timeout", 300.0))

        self._publish_status(AgvStatus.RUNNING, f"{name}: navigating to point {target_id}")
        try:
            self.agv.AGV_Navigation(source_id, target_id)
        except OSError as e:
            self.get_logger().error(f"AGV_Navigation failed: {e}")
            return False

        elapsed = 0.0
        while elapsed < timeout:
            if self.stop_requested:
                return False
            try:
                status = self.agv.AGV_Status()
            except (OSError, KeyError) as e:
                self.get_logger().warn(f"AGV_Status poll failed: {e}")
                status = None
            if status in SEER_TASK_DONE:
                self.get_logger().info(f"{name}: AGV reached target (task_status={status})")
                return True
            rclpy.sleep(self.poll_interval)
            elapsed += self.poll_interval

        self.get_logger().error(f"{name}: navigation timeout after {timeout}s")
        return False

    def _execute_arm_actions(self, station):
        name = station.get("name", "station")
        for i, action in enumerate(station.get("arm_actions", [])):
            if self.stop_requested:
                return False
            action_name = action.get("name", f"action_{i}")
            self._publish_status(
                AgvStatus.RUNNING, f"{name}: arm action '{action_name}' ({i + 1}/"
                f"{len(station.get('arm_actions', []))})"
            )
            if action.get("type") == "joint":
                ok = self._call_move_joint(action)
            elif action.get("type") == "pose":
                ok = self._call_move_pose(action)
            else:
                self.get_logger().error(f"unknown arm action type: {action.get('type')}")
                return False
            if not ok:
                return False
            if not self._wait_arm_idle():
                return False

        self._arm_to_home(station)
        return self._wait_arm_idle()

    def _arm_to_home(self, station):
        home = station.get("arm_home", self.default_arm_home) if station else self.default_arm_home
        self.get_logger().info("arm returning to home position")
        self._call_move_joint({"target_joint": home})

    def _wait_arm_idle(self):
        """Wait for arm to leave RUNNING and settle at IDLE; False on error states."""
        # Give the bridge a moment to flip to RUNNING if the service returned early
        rclpy.sleep(self.poll_interval)
        while True:
            if self.stop_requested:
                return False
            state = self.arm_state
            if state in (ARM_STATE_ERROR, ARM_STATE_EMERGENCY_STOP):
                self.get_logger().error(f"arm in error state ({state}), aborting")
                return False
            if state == ARM_STATE_IDLE:
                return True
            rclpy.sleep(self.poll_interval)

    # ---------------- arm service calls ----------------

    def _wait_for_service(self, client):
        if not client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error(f"service {client.srv_name} unavailable")
            return False
        return True

    def _call_move_joint(self, action):
        if not self._wait_for_service(self.move_joint_client):
            return False
        req = MoveToJointAngles.Request()
        req.target_joint = [math.radians(v) for v in action["target_joint"]]
        acc = action.get("max_acc")
        vel = action.get("max_vel")
        req.max_acc = [math.radians(v) for v in acc] if acc else [1.0] * 6
        req.max_vel = [math.radians(v) for v in vel] if vel else [1.0] * 6
        req.enable_move = True
        future = self.move_joint_client.call_async(req)
        if not self._spin_until_done(future):
            return False
        result = future.result()
        if not result.success:
            self.get_logger().error(f"move_to_joint_angles failed: {result.message}")
        return result.success

    def _call_move_pose(self, action):
        if not self._wait_for_service(self.move_pose_client):
            return False
        req = MoveToPose.Request()
        req.target_pose.position.x = float(action["position"][0])
        req.target_pose.position.y = float(action["position"][1])
        req.target_pose.position.z = float(action["position"][2])
        # mission.yaml stores quaternion as xyzw, geometry_msgs expects xyzw
        req.target_pose.orientation.x = float(action["orientation"][0])
        req.target_pose.orientation.y = float(action["orientation"][1])
        req.target_pose.orientation.z = float(action["orientation"][2])
        req.target_pose.orientation.w = float(action["orientation"][3])
        req.max_velocity = float(action.get("max_velocity", 0.2))
        req.max_acceleration = float(action.get("max_acceleration", 0.5))
        req.blend_radius = float(action.get("blend_radius", 0.0))
        req.blocking = True
        future = self.move_pose_client.call_async(req)
        if not self._spin_until_done(future):
            return False
        result = future.result()
        if not result.success:
            self.get_logger().error(f"move_to_pose failed: {result.message}")
        return result.success

    def _spin_until_done(self, future):
        """Wait for a service future from a non-callback thread."""
        import time
        deadline = time.time() + self.arm_service_timeout
        while not future.done():
            if self.stop_requested:
                return False
            if time.time() > deadline:
                self.get_logger().error("arm service call timed out")
                return False
            time.sleep(0.05)
        return True


def main(args=None):
    rclpy.init(args=args)
    node = MissionNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:
        # context may already be torn down by an external signal
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
