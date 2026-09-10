#!/usr/bin/env python3
"""Stop Isaac's latched wheel commands when velocity input becomes stale."""
import math
import time
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from geometry_msgs.msg import Twist


class VelocityWatchdog(Node):
    def __init__(self):
        super().__init__('isaac_velocity_watchdog')
        self.declare_parameter('timeout', .5)
        self.last = -math.inf
        self.command = Twist()
        self.pub = self.create_publisher(Twist, '/isaac_cmd_vel', 1)
        self.sub = self.create_subscription(Twist, '/cmd_vel', self.receive, 1)
        self.timer = self.create_timer(.05, self.publish, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def receive(self, msg):
        if not all(math.isfinite(v) for v in (msg.linear.x,msg.angular.z)):
            self.last = -math.inf
            return
        self.command = msg
        self.last = time.monotonic()

    def publish(self):
        self.pub.publish(self.command if time.monotonic()-self.last < self.get_parameter('timeout').value else Twist())


def main():
    rclpy.init();node=VelocityWatchdog()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.pub.publish(Twist())
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':main()
