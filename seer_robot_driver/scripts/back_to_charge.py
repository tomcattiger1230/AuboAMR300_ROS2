#!/usr/bin/env python
# coding=UTF-8
"""
Author: Wei Luo
Date: 2025-11-25 11:23:54
LastEditors: Wei Luo
LastEditTime: 2025-11-25 11:42:39
Note: Note
"""

import rclpy
from rclpy.node import Node


class BackToCharge(Node):
    def __init__(self):
        super().__init__("back_to_charge_subscriber")


if __name__ == "__main__":
    rclpy.init()
    btc_node = BackToCharge()
