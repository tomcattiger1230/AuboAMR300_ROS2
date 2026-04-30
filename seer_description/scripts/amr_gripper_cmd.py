#!/usr/bin/env python
# coding=UTF-8
"""
Author: Wei Luo
Date: 2026-04-30 16:25:01
LastEditors: Wei Luo
LastEditTime: 2026-04-30 16:27:46
Note: Note
"""

import rclpy
import time
from moveit.planning import MoveItPy


def main(args=None):
    rclpy.init(args=args)

    print("🚀 正在初始化 MoveItPy 节点...")
    # 实例化 MoveItPy
    aubo_moveit = MoveItPy(node_name="gripper_cmd_node")

    # 获取你的夹爪规划组 (确保这里的名字和你在 SRDF 里的一致)
    gripper = aubo_moveit.get_planning_component("gripper")

    def execute_pose(pose_name):
        print(f"\n---> 准备将夹爪设置为快捷动作: [{pose_name}]")

        # 1. 将起点强制设置为当前真实状态
        gripper.set_start_state_to_current_state()

        # 2. 目标状态直接写入快捷动作的名字 ("open" 或 "close")
        gripper.set_goal_state(configuration_name=pose_name)

        # 3. 生成轨迹规划
        print("🧠 正在计算运动轨迹...")
        plan_result = gripper.plan()

        # 4. 下发到底层控制器执行
        if plan_result:
            print("✅ 规划成功，正在执行...")
            aubo_moveit.execute(plan_result.trajectory, controllers=[])
            print(f"🎉 [{pose_name}] 动作执行完毕！")
        else:
            print(f"❌ 规划失败！无法执行 [{pose_name}]。")

    # 延时等待与 Gazebo 和 MoveIt 的状态同步
    time.sleep(1.0)

    # 🎬 开始自动化测试
    execute_pose("gripper_open")  # 调用张开动作

    print("⏳ 保持张开状态 3 秒钟...")
    time.sleep(3.0)

    execute_pose("gripper_close")  # 调用闭合动作

    print("\n👋 脚本运行结束。")
    rclpy.shutdown()


if __name__ == "__main__":
    main()
