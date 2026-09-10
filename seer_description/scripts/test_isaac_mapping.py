#!/usr/bin/env python3
"""Perform one measured turn at the empty warehouse origin to initialize SLAM."""
import argparse
import json
import math
import time
from pathlib import Path
import rclpy
from rclpy.signals import SignalHandlerOptions
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from rclpy.qos import qos_profile_sensor_data


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--execute',action='store_true',required=True)
    parser.add_argument('--output',default='mapping_turn.json')
    args=parser.parse_args()
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node=rclpy.create_node('isaac_mapping_turn');pub=node.create_publisher(Twist,'/cmd_vel',1)
    state={};total=0.;previous=None

    def receive(msg):
        nonlocal total,previous
        q=msg.pose.pose.orientation
        yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        if previous is not None:total+=math.atan2(math.sin(yaw-previous),math.cos(yaw-previous))
        previous=yaw;state.update(msg=msg,received=time.monotonic())

    sub=node.create_subscription(Odometry,'/odom',receive,qos_profile_sensor_data)
    start=time.monotonic();last=0.;success=False
    try:
        while time.monotonic()-start<150:
            rclpy.spin_once(node,timeout_sec=.02)
            if abs(total)>2*math.pi-.04:
                success=True;break
            cmd=Twist()
            if state and time.monotonic()-state['received']<.5:
                p=state['msg'].pose.pose.position
                if math.hypot(p.x,p.y)>.3:raise RuntimeError('Turn requires clear warehouse origin')
                cmd.angular.z=.25 if abs(total)<5.8 else .12
            pub.publish(cmd)
            if time.monotonic()-last>10:
                print(f'Measured rotation: {total:.3f} rad',flush=True);last=time.monotonic()
        if not success:raise RuntimeError('Mapping turn timed out or odometry unavailable')
    finally:
        for _ in range(5):pub.publish(Twist());rclpy.spin_once(node,timeout_sec=.05)
        Path(args.output).write_text(json.dumps({'success':success,'rotation_rad':total,'wall_seconds':time.monotonic()-start},indent=2))
        node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
