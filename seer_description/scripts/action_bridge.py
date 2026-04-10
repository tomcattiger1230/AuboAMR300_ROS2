#!/usr/bin/env python
# coding=UTF-8
"""
Author: Wei Luo
Date: 2026-04-09 16:50:17
LastEditors: Wei Luo
LastEditTime: 2026-04-10 10:22:14
Note: Note
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from control_msgs.action import FollowJointTrajectory
from sensor_msgs.msg import JointState  # <== 关键改变：引入 JointState 照片级消息
import time


class TrajectoryBridge(Node):
    def __init__(self):
        super().__init__("action_to_joint_state_bridge")

        # 1. 依然伪装成 Action Server 稳住 MoveIt
        self._action_server = ActionServer(
            self,
            FollowJointTrajectory,
            "/aubo_arm_controller/follow_joint_trajectory",
            self.execute_callback,
        )

        # 2. 创建 JointState 发布者 (专门喂给 Isaac Sim)
        self.publisher_ = self.create_publisher(
            JointState,
            "/isaac_joint_commands",  # <== 我们换一个极其清晰的专线话题名
            10,
        )
        self.get_logger().info(
            "🚀 V2 终极翻译官：Action -> JointState 实时插值引擎已启动！"
        )

    def execute_callback(self, goal_handle):
        self.get_logger().info(
            "✅ 收到轨迹！开始像放电影一样向 Isaac Sim 发送实时关节流..."
        )

        traj = goal_handle.request.trajectory
        joint_names = traj.joint_names

        # 记录系统启动播放的时间点
        sys_start = time.time()

        # 遍历 MoveIt 算好的每一个路径点
        for point in traj.points:
            # 计算这个点应该在未来的第几秒执行
            time_from_start = (
                point.time_from_start.sec + point.time_from_start.nanosec / 1e9
            )

            # 等待，直到真实时间到达这个点的时间 (完美复现 MoveIt 规划的速度)
            target_sys_time = sys_start + time_from_start
            sleep_duration = target_sys_time - time.time()
            if sleep_duration > 0:
                time.sleep(sleep_duration)

            # 组装单帧的 "JointState 照片" 并发布
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name = joint_names
            msg.position = point.positions
            if point.velocities:
                msg.velocity = point.velocities

            self.publisher_.publish(msg)

        # 播放完毕，告诉 MoveIt 任务成功
        goal_handle.succeed()
        result = FollowJointTrajectory.Result()
        result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
        self.get_logger().info("🏁 电影放映完毕！机械臂应该已经完美到达目标！")
        return result


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryBridge()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
