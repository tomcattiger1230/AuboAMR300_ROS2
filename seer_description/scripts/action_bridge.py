#!/usr/bin/env python
# coding=UTF-8
"""
Author: Wei Luo
Date: 2026-04-09 16:50:17
LastEditors: Wei Luo
LastEditTime: 2026-04-09 16:50:19
Note: Note
"""
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory


class TrajectoryBridge(Node):
    def __init__(self):
        super().__init__("action_to_topic_bridge")

        # 1. 伪装成 Action Server，接听 MoveIt 的请求
        self._action_server = ActionServer(
            self,
            FollowJointTrajectory,
            "/aubo_arm_controller/follow_joint_trajectory",
            self.execute_callback,
        )

        # 2. 创建一个 Publisher，给 Isaac Sim 发送 Topic
        self.publisher_ = self.create_publisher(
            JointTrajectory, "/aubo_arm_controller/joint_trajectory", 10
        )
        self.get_logger().info(
            "🚀 赛博翻译官已启动：等待 MoveIt 动作，准备转发给 Isaac Sim..."
        )

    def execute_callback(self, goal_handle):
        self.get_logger().info("✅ 收到 MoveIt 完美轨迹！正在瞬间转发给 Isaac Sim...")

        # 提取轨迹消息并发布给 Isaac Sim 的 Subscribe 节点
        traj_msg = goal_handle.request.trajectory
        self.publisher_.publish(traj_msg)

        # 欺骗 MoveIt 告诉它“我已经执行成功啦！” (让 MoveIt 安心结束规划)
        goal_handle.succeed()
        result = FollowJointTrajectory.Result()
        result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
        return result


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryBridge()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
