#!/usr/bin/env python3
"""Explicit simulation-only Nav2 route test; writes actual feedback and paths."""
import argparse
import json
import math
import time
from pathlib import Path

import rclpy
from rclpy.signals import SignalHandlerOptions
from rclpy.action import ActionClient
from rclpy.parameter import Parameter
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import ComputePathToPose, NavigateToPose
from tf2_ros import Buffer, TransformListener


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--execute', action='store_true', help='Actually drive the simulated base')
    parser.add_argument('--output', default='navigation_result.json')
    args = parser.parse_args()
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node('isaac_navigation_test')
    node.set_parameters([Parameter('use_sim_time', value=True)])
    tf = Buffer()
    listener = TransformListener(tf, node)
    planner = ActionClient(node, ComputePathToPose, '/compute_path_to_pose')
    navigator = ActionClient(node, NavigateToPose, '/navigate_to_pose')
    stop = node.create_publisher(Twist, '/cmd_vel', 10)
    report = []
    active = None

    def wait(future, seconds):
        deadline = time.monotonic() + seconds
        while not future.done() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.05)
        if not future.done():
            raise TimeoutError('ROS action timed out')
        return future.result()

    def pose(x, y, yaw=0.):
        p = PoseStamped()
        p.header.frame_id = 'map'
        p.header.stamp = node.get_clock().now().to_msg()
        p.pose.position.x, p.pose.position.y = float(x), float(y)
        p.pose.orientation.z, p.pose.orientation.w = math.sin(yaw/2), math.cos(yaw/2)
        return p

    try:
        if not planner.wait_for_server(timeout_sec=15) or not navigator.wait_for_server(timeout_sec=15):
            raise RuntimeError('Start isaac_mapping and isaac_navigation first')
        deadline = time.monotonic()+5
        while time.monotonic()<deadline:
            rclpy.spin_once(node, timeout_sec=.1)
        origin = tf.lookup_transform('map', 'base_footprint', rclpy.time.Time()).transform.translation
        # This route is for warehouse_stick_demo, whose central aisle is clear.
        if math.hypot(origin.x, origin.y) > .4:
            raise RuntimeError('Warehouse route requires robot near map origin; reset the demo first')
        for x, y, yaw in [(1.5,0.,0.), (1.5,1.5,math.pi/2), (0.,1.5,math.pi), (0.,0.,0.)]:
            goal = ComputePathToPose.Goal()
            goal.goal = pose(x,y,yaw)
            goal.planner_id = 'GridBased'
            handle = wait(planner.send_goal_async(goal),10)
            if not handle.accepted:
                raise RuntimeError('Planner rejected goal')
            result = wait(handle.get_result_async(),20)
            path = result.result.path.poses
            item = {'goal':[x,y,yaw], 'planning_status':result.status,
                    'path_points':len(path), 'path':[[p.pose.position.x,p.pose.position.y] for p in path]}
            report.append(item)
            if result.status != 4 or not path:
                raise RuntimeError('No collision-free known-space path')
            if args.execute:
                goal = NavigateToPose.Goal()
                goal.pose = pose(x,y,yaw)
                active = wait(navigator.send_goal_async(goal),10)
                if not active.accepted:
                    raise RuntimeError('Navigator rejected goal')
                result = wait(active.get_result_async(),240)
                active = None
                actual = tf.lookup_transform('map','base_footprint',rclpy.time.Time()).transform
                item.update(status=result.status, error_code=result.result.error_code,
                            actual=[actual.translation.x,actual.translation.y],
                            position_error=math.hypot(actual.translation.x-x,actual.translation.y-y))
                if result.status!=4 or item['position_error']>.25:
                    raise RuntimeError(f'Navigation failed: {item}')
            print(json.dumps({k:v for k,v in item.items() if k!='path'}),flush=True)
        # East wall is physically occupied. The planner must reject this goal.
        goal = ComputePathToPose.Goal()
        goal.goal = pose(12.05,0.)
        goal.planner_id = 'GridBased'
        handle = wait(planner.send_goal_async(goal),10)
        result = wait(handle.get_result_async(),20) if handle.accepted else None
        rejected = result is None or result.status != 4
        report.append({'occupied_goal_rejected':rejected})
        if not rejected:
            raise RuntimeError('Planner accepted a wall goal')
    finally:
        if active is not None and active.accepted:
            wait(active.cancel_goal_async(),5)
        for _ in range(5):
            stop.publish(Twist())
            rclpy.spin_once(node,timeout_sec=.05)
        Path(args.output).write_text(json.dumps(report,indent=2))
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__=='__main__':
    main()
