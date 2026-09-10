#!/usr/bin/env python3
"""Normalize RTX invalid returns for ROS consumers without inventing free space."""
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


class ScanFilter(Node):
    def __init__(self):
        super().__init__('isaac_scan_filter')
        self.declare_parameter('max_range', 20.0)
        self.scan_publishers = {}
        self.subscriptions_ = []
        for name in ('front_lidar', 'back_lidar'):
            self.scan_publishers[name] = self.create_publisher(LaserScan, f'/{name}/scan_filtered', qos_profile_sensor_data)
            self.subscriptions_.append(self.create_subscription(
                LaserScan, f'/{name}/scan', lambda msg, name=name: self.filter(msg, name), qos_profile_sensor_data))

    def filter(self, msg, name):
        msg.range_max = min(msg.range_max, self.get_parameter('max_range').value)
        # NaN means unknown, whereas +inf may clear costmap obstacles. RTX -1
        # includes self-occlusion, so it must never be converted to free space.
        msg.ranges = [r if math.isfinite(r) and msg.range_min <= r <= msg.range_max
                      else float('nan') for r in msg.ranges]
        msg.angle_max = msg.angle_min + (len(msg.ranges) - 1) * msg.angle_increment
        self.scan_publishers[name].publish(msg)


def main():
    rclpy.init()
    node = ScanFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
