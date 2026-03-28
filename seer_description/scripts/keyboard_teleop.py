#!/usr/bin/env python3
# coding=UTF-8
"""
Author: Wei Luo
Date: 2026-03-22 21:23:38
LastEditors: Wei Luo
LastEditTime: 2026-03-22 21:23:39
Note: Note
"""

#!/usr/bin/env python3
# coding=UTF-8

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys, select, termios, tty

# 控制提示信息
msg = """
========================================
🚀 SEER 差速底盘控制终端启动！
========================================
按键说明:
        W (加速前进)
A (左转)    S (急刹车)    D (右转)
        X (加速后退)

W/X : 增加/减少 线速度 (URDF最高限速: 1.5 m/s)
A/D : 增加/减少 角速度 (URDF最高限速: 2.0 rad/s)
S   : 强制归零停止

CTRL-C 退出
========================================
"""


class SeerTeleopKeyboard(Node):
    def __init__(self):
        super().__init__("seer_teleop_keyboard")
        # 创建发布者，发布到 cmd_vel 话题
        self.publisher_ = self.create_publisher(Twist, "/cmd_vel", 10)

        # 当前速度状态
        self.linear_vel = 0.0
        self.angular_vel = 0.0

        # 精确匹配你 URDF 中的物理限制
        self.max_lin = 1.5
        self.max_ang = 2.0

        # 每次按键的加速度步长 (可以根据手感自己调)
        self.step_lin = 0.1
        self.step_ang = 0.2

    def publish_twist(self):
        twist = Twist()
        twist.linear.x = self.linear_vel
        twist.linear.y = 0.0  # 差速车不能横移
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = self.angular_vel
        self.publisher_.publish(twist)


# 读取单个字符输入的魔法函数（不需要按回车）
def getKey(settings):
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def main(args=None):
    # 保存当前终端设置
    settings = termios.tcgetattr(sys.stdin)

    rclpy.init(args=args)
    node = SeerTeleopKeyboard()

    print(msg)

    try:
        while True:
            key = getKey(settings)

            # 逻辑判断与限速
            if key == "w":
                node.linear_vel = min(node.max_lin, node.linear_vel + node.step_lin)
            elif key == "x":
                node.linear_vel = max(-node.max_lin, node.linear_vel - node.step_lin)
            elif key == "a":
                node.angular_vel = min(node.max_ang, node.angular_vel + node.step_ang)
            elif key == "d":
                node.angular_vel = max(-node.max_ang, node.angular_vel - node.step_ang)
            elif key == "s":
                node.linear_vel = 0.0
                node.angular_vel = 0.0
            elif key == "\x03":  # 监听到 CTRL-C
                break
            else:
                continue

            # 发布速度并刷新终端显示
            node.publish_twist()
            print(
                f"\r当前下发指令: 线速度 {node.linear_vel: .2f} m/s | 角速度 {node.angular_vel: .2f} rad/s   ",
                end="",
            )

    except Exception as e:
        print(e)
    finally:
        # 退出前发一个全 0 的刹车包，防止车停不下来撞墙
        node.linear_vel = 0.0
        node.angular_vel = 0.0
        node.publish_twist()
        rclpy.shutdown()
        # 恢复终端设置
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


if __name__ == "__main__":
    main()
