#!/usr/bin/env python3
"""Regression checks for invalid RTX returns and lost velocity commands.

Run in an unused ROS_DOMAIN_ID, e.g. ROS_DOMAIN_ID=136 python3 this_file.py.
"""
import importlib.util
from pathlib import Path
import math
import time
import unittest
import rclpy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from rclpy.qos import qos_profile_sensor_data

SCRIPTS=Path(__file__).resolve().parents[1]/'scripts'

def load(name):
    spec=importlib.util.spec_from_file_location(name,SCRIPTS/(name+'.py'))
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    return module


class AdaptersTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):rclpy.init()

    @classmethod
    def tearDownClass(cls):rclpy.shutdown()

    def test_unknown_returns_do_not_clear_obstacles(self):
        node=load('scan_filter').ScanFilter();observer=rclpy.create_node('filtered_scan_observer');messages=[]
        sub=observer.create_subscription(LaserScan,'/front_lidar/scan_filtered',messages.append,qos_profile_sensor_data)
        msg=LaserScan();msg.range_min=.3;msg.range_max=200.;msg.angle_min=-1.;msg.angle_increment=.1
        msg.ranges=[-1.,float('inf'),float('nan'),.1,.5,5.,25.]
        end=time.monotonic()+3
        try:
            while not messages and time.monotonic()<end:
                node.filter(msg,'front_lidar');rclpy.spin_once(observer,timeout_sec=.1)
            self.assertTrue(messages)
            out=messages[-1]
            self.assertTrue(all(math.isnan(out.ranges[i]) for i in (0,1,2,3,6)))
            self.assertAlmostEqual(out.ranges[4],.5)
            self.assertAlmostEqual(out.angle_max,-.4,places=5)
        finally:observer.destroy_node();node.destroy_node()

    def test_velocity_stops_without_new_input(self):
        node=load('cmd_vel_watchdog').VelocityWatchdog();observer=rclpy.create_node('velocity_observer');messages=[]
        sub=observer.create_subscription(Twist,'/isaac_cmd_vel',messages.append,10)
        cmd=Twist();cmd.linear.x=.2
        end=time.monotonic()+2
        try:
            while not messages and time.monotonic()<end:
                node.receive(cmd);node.publish();rclpy.spin_once(observer,timeout_sec=.05)
            self.assertTrue(messages)
            self.assertAlmostEqual(messages[-1].linear.x,.2)
            end=time.monotonic()+.8
            while time.monotonic()<end:
                rclpy.spin_once(node,timeout_sec=.02);rclpy.spin_once(observer,timeout_sec=.02)
            self.assertEqual(messages[-1].linear.x,0.)
            cmd.linear.x=float('nan');node.receive(cmd);node.publish();rclpy.spin_once(observer,timeout_sec=.1)
            self.assertEqual(messages[-1].linear.x,0.)
        finally:observer.destroy_node();node.destroy_node()


if __name__=='__main__':unittest.main()
