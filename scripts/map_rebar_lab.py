#!/usr/bin/env python3
"""Survey the Isaac rebar lab using lidar and odometry; no arm or Nav2 actions."""
import argparse
import json
import math
import os
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs.msg import LaserScan, JointState
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from tf2_ros import Buffer, TransformListener


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--execute', action='store_true', required=True)
    parser.add_argument('--output', required=True)
    parser.add_argument('--waypoints', default='[[1.5,0],[1.5,2],[8.5,2],[8.5,6.5],[0,6.5],[0,2],[-5,2],[-5,-1.7],[1.5,-1.7],[1.5,0],[0.4,0]]')
    args = parser.parse_args()
    if os.environ.get('ROS_DOMAIN_ID') != '133':
        raise RuntimeError('This survey requires the isolated Isaac simulation domain 133')
    rclpy.init()
    node = rclpy.create_node('rebar_lab_mapping_survey')
    publisher = node.create_publisher(Twist, '/cmd_vel', 10)
    tf = Buffer(); listener = TransformListener(tf, node)
    state = {}; scans = {}; joints = {}; trail = []
    report = {'scene': 'warehouse_finger_rebar_lab_demo.usda', 'success': False,
              'waypoints': [], 'speed_limit_m_s': .3, 'angular_limit_rad_s': .4}

    def odometry(msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        state.update(pose=(p.x, p.y, math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))), received=time.monotonic())

    def joint_state(msg):
        for name, value in zip(msg.name, msg.position):
            if 'wheel' not in name and 'caster' not in name:
                lo, hi = joints.get(name, (value, value))
                joints[name] = (min(lo, value), max(hi, value))

    def grid(msg):
        report['map'] = {'width': msg.info.width, 'height': msg.info.height,
                         'resolution': msg.info.resolution,
                         'known_cells': sum(v >= 0 for v in msg.data),
                         'total_cells': len(msg.data)}

    subscriptions = [node.create_subscription(Odometry, '/odom', odometry, qos_profile_sensor_data),
                     node.create_subscription(JointState, '/joint_states', joint_state, qos_profile_sensor_data),
                     node.create_subscription(OccupancyGrid, '/map', grid, QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))]
    for name in ('front_lidar', 'back_lidar'):
        subscriptions.append(node.create_subscription(LaserScan, f'/{name}/scan_filtered',
            lambda msg, name=name: scans.update({name: (msg, time.monotonic())}), qos_profile_sensor_data))

    def clearance(turning):
        points = []
        for msg, received in scans.values():
            transform = tf.lookup_transform('base_footprint', msg.header.frame_id, rclpy.time.Time()).transform
            q = transform.rotation
            for index in range(0, len(msg.ranges), 4):
                distance = msg.ranges[index]
                if not math.isfinite(distance): continue
                angle = msg.angle_min + index * msg.angle_increment
                sx, sy = distance * math.cos(angle), distance * math.sin(angle)
                x = transform.translation.x + (1-2*(q.y*q.y+q.z*q.z))*sx + 2*(q.x*q.y-q.w*q.z)*sy
                y = transform.translation.y + 2*(q.x*q.y+q.w*q.z)*sx + (1-2*(q.x*q.x+q.z*q.z))*sy
                # Rear lidar sees parts of the robot itself. Ignore returns
                # within the chassis footprint, retaining external obstacles.
                if abs(x) <= .55 and abs(y) <= .45: continue
                points.append((x, y))
        if not points: raise RuntimeError('No valid lidar returns')
        if turning:
            return min(math.hypot(x, y) for x, y in points) > .75
        return not any(.15 < x < 1.0 and abs(y) < .55 for x, y in points)

    started = time.monotonic(); last_print = 0
    try:
        deadline = started + 30
        def ready():
            return ('pose' in state and len(scans) == 2 and 'map' in report
                    and all(tf.can_transform('base_footprint', msg.header.frame_id, rclpy.time.Time())
                            for msg, _ in scans.values()))
        while time.monotonic() < deadline and not ready():
            rclpy.spin_once(node, timeout_sec=.1)
        if not ready(): raise RuntimeError('Map, lidar, odometry or sensor TF did not become ready')
        if 'pose' not in state or math.hypot(*state['pose'][:2]) > .4:
            raise RuntimeError('Survey requires the fresh lab scene near the origin')
        for target in json.loads(args.waypoints):
            deadline = time.monotonic() + 300
            while True:
                rclpy.spin_once(node, timeout_sec=.04)
                now = time.monotonic()
                if now - state['received'] > 1.5 or len(scans) != 2 or any(now-t > 1.5 for _,t in scans.values()):
                    raise RuntimeError('Odometry or lidar feedback became stale')
                x, y, yaw = state['pose']
                distance = math.hypot(target[0]-x, target[1]-y)
                if distance < .12: break
                if now > deadline: raise TimeoutError(f'Survey waypoint timed out: {target}')
                error = wrap(math.atan2(target[1]-y, target[0]-x) - yaw)
                turning = abs(error) > .25
                if not clearance(turning): raise RuntimeError(f'Lidar clearance blocked at {(x,y)} toward {target}')
                command = Twist()
                command.angular.z = max(-.4, min(.4, 1.2*error))
                command.linear.x = 0.0 if turning else min(.3, .6*distance)
                publisher.publish(command)
                if now-last_print > 8:
                    trail.append([round(x,3),round(y,3),round(yaw,3)])
                    print(json.dumps({'target':target,'pose':trail[-1], 'map':report.get('map')}), flush=True)
                    last_print = now
            publisher.publish(Twist())
            report['waypoints'].append({'target':target, 'actual':state['pose']})
            print(f'Completed mapping waypoint {target}', flush=True)
        report['success'] = True
    except Exception as exc:
        report['error'] = str(exc)
        raise
    finally:
        for _ in range(10):
            publisher.publish(Twist()); rclpy.spin_once(node, timeout_sec=.05)
        report.update(wall_seconds=time.monotonic()-started, trail=trail,
                      final_pose=state.get('pose'), joint_spans={k:hi-lo for k,(lo,hi) in joints.items()})
        Path(args.output).write_text(json.dumps(report, indent=2))
        node.destroy_node(); rclpy.shutdown()


if __name__ == '__main__':
    main()
