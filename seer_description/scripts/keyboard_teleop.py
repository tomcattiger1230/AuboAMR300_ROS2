#!/usr/bin/env python3
# coding=UTF-8
"""
Author: Wei Luo
Date: 2026-03-22 21:23:38
LastEditors: Wei Luo
LastEditTime: 2026-03-22 21:23:39
Note: Note
"""


import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import termios
import tty
import select

# 车辆运动学限制 (对应 URDF 配置)
MAX_LIN_VEL = 5.0  # 最大线速度 m/s
MAX_STEER = 0.5  # 最大转向角 rad

# 步进值
LIN_STEP = 0.2
STEER_STEP = 0.05

msg = """
控制你的 Ackermann 小车!
---------------------------
移动按键:
        w
   a    s    d
        x

w/x : 增加/减少 线速度 (+/- 0.2 m/s)
a/d : 向左/向右 转向   (+/- 0.05 rad)
s   : 紧急停止 (所有速度归零)

CTRL-C 退出
"""


def get_key(settings):
    """非阻塞读取键盘输入"""
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ""
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


class KeyboardTeleop(Node):
    def __init__(self):
        super().__init__("keyboard_teleop")
        self.publisher_ = self.create_publisher(Twist, "/cmd_vel", 10)
        self.target_linear_vel = 0.0
        self.target_steer_angle = 0.0

    def publish_twist(self):
        twist = Twist()
        twist.linear.x = self.target_linear_vel
        # 对于 Ackermann 模型，Twist.angular.z 通常映射为前轮转向角
        twist.angular.z = self.target_steer_angle
        self.publisher_.publish(twist)


def main(args=None):
    settings = termios.tcgetattr(sys.stdin)
    rclpy.init(args=args)
    node = KeyboardTeleop()

    print(msg)

    try:
        while rclpy.ok():
            key = get_key(settings)

            if key == "w":
                node.target_linear_vel = min(
                    node.target_linear_vel + LIN_STEP, MAX_LIN_VEL
                )
                print(
                    "当前指令 -> 线速度: {0} m/s, 转向角: {1} rad\r".format(
                        node.target_linear_vel, node.target_steer_angle
                    )
                )
            elif key == "x":
                node.target_linear_vel = max(
                    node.target_linear_vel - LIN_STEP, -MAX_LIN_VEL
                )
                print(
                    "当前指令 -> 线速度: {0} m/s, 转向角: {1} rad\r".format(
                        node.target_linear_vel, node.target_steer_angle
                    )
                )
            elif key == "a":
                node.target_steer_angle = min(
                    node.target_steer_angle + STEER_STEP, MAX_STEER
                )
                print(
                    "当前指令 -> 线速度: {0} m/s, 转向角: {1} rad\r".format(
                        node.target_linear_vel, node.target_steer_angle
                    )
                )
            elif key == "d":
                node.target_steer_angle = max(
                    node.target_steer_angle - STEER_STEP, -MAX_STEER
                )
                print(
                    "当前指令 -> 线速度: {0} m/s, 转向角: {1} rad\r".format(
                        node.target_linear_vel, node.target_steer_angle
                    )
                )
            elif key == "s":
                node.target_linear_vel = 0.0
                node.target_steer_angle = 0.0
                # print(
                # f"【停止】 -> 线速度: {node.target_linear_vel:.2f} m/s, 转向角: {node.target_steer_angle:.2f} rad\r"
                # )
            elif key == "\x03":  # CTRL-C
                break

            node.publish_twist()
            rclpy.spin_once(node, timeout_sec=0.0)

    except Exception as e:
        print(e)

    finally:
        # 退出前发送停止指令
        node.target_linear_vel = 0.0
        node.target_steer_angle = 0.0
        node.publish_twist()

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
