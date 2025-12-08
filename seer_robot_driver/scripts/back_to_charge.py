#!/usr/bin/env python
# coding=UTF-8
"""
Author: Wei Luo
Date: 2025-11-25 11:23:54
LastEditors: Wei Luo
LastEditTime: 2025-12-08 16:52:09
Note: Note
"""

import rclpy
import os
from rclpy.node import Node
from std_srvs.srv import SetBool
from seer_robot_driver.AgvControl import AgvControl


class BackToCharge(Node):
    def __init__(self):
        super().__init__("back_to_charge_server")
        self.agv_controller = AgvControl()
        self.srv = self.create_service(SetBool, "charging", self.charging_callback)

    def charging_callback(self, request, response):
        if request.data is True:
            # process the charging codes
            # get the closest waypoint and generate a list of waypoint to charging station
            # self.agv_controller.AGV_Navigation()
            response.success = True
            self.get_logger().info("Calling back to charge service")

        return response

    def driving(self, start_points, end_points, sleep_time=1):
        self.agv_controller.AGV_Navigation(start_points, end_points)
        while True:
            rclpy.sleep(sleep_time)
            agv_status = self.agv_controller.AGV_Status()
            if agv_status == 4 or agv_status == 5:
                break


if __name__ == "__main__":
    rclpy.init()
    btc_node = BackToCharge()
    rclpy.spin(btc_node)
    rclpy.shutdown()
