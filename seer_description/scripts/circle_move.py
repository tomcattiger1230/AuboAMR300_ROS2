#!/usr/bin/env python
# coding=UTF-8
"""
Author: Wei Luo
Date: 2026-03-20 15:40:20
LastEditors: Wei Luo
LastEditTime: 2026-03-20 15:40:21
Note: Note
"""
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class CircleMoveNode(Node):
    def __init__(self):
        super().__init__("circle_move_node")
        # 创建一个发布者，发布到 /cmd_vel 话题
        self.publisher_ = self.create_publisher(Twist, "/cmd_vel", 10)
        # 每 0.1 秒执行一次 timer_callback
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.get_logger().info("控制节点已启动，机器人开始画圆移动！")

    def timer_callback(self):
        msg = Twist()
        # 线速度 (向前前进 0.3 m/s)
        msg.linear.x = 0.3
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        # 角速度 (绕Z轴旋转 0.5 rad/s)
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = 0.5

        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = CircleMoveNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # 按下 Ctrl+C 时停止机器人
        msg = Twist()
        node.publisher_.publish(msg)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
