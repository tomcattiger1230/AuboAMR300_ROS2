#!/usr/bin/env python
# coding=UTF-8
"""
Author: Wei Luo
Date: 2025-11-25 11:23:54
LastEditors: Wei Luo
LastEditTime: 2025-12-08 15:00:50
Note: Note
"""

import rclpy
import os
from rclpy.node import Node
from std_srvs.srv import SetBool


class BackToCharge(Node):
    def __init__(self):
        super().__init__("back_to_charge_server")
        self.srv = self.create_service(SetBool, "charging", self.charging_callback)

    def charging_callback(self, request, response):
        if request.data is True:
            # process the charging codes
            response.success = True
            self.get_logger().info("Calling back to charge service")

        return response


if __name__ == "__main__":
    rclpy.init()
    btc_node = BackToCharge()
    rclpy.spin(btc_node)
    rclpy.shutdown()
