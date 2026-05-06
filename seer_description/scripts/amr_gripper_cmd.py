#!/usr/bin/env python
# coding=UTF-8
#!/usr/bin/env python
# coding=UTF-8
"""
Author: Wei Luo
Date: 2026-04-30 22:55:54
LastEditors: Wei Luo
LastEditTime: 2026-04-30 23:55:45
Note: Note
"""
"""
Author: Wei Luo
Date: 2026-04-30 16:38:10
LastEditors: Wei Luo
LastEditTime: 2026-04-30 16:59:55
Note: Note
"""

import rclpy
import time
from moveit.planning import MoveItPy
from moveit_configs_utils import MoveItConfigsBuilder


def main(args=None):
    rclpy.init(args=args)

    print("🔍 正在加载 MoveIt 配置参数...")

    # 1. 主动去读取你的 MoveIt 配置包
    # 注意：请确保 "seer_aubo_moveit_config" 是你的配置包真名
    # file_path 必须指向你真实的 xacro 模型文件（如果在 urdf 文件夹下，就是 "urdf/你的模型.urdf.xacro"）
    moveit_config_dict = (
        MoveItConfigsBuilder("seer_aubo_stick")
        .robot_description(
            file_path="config/seer_aubo_composite.urdf.xacro"
        )  # <== 这里检查一下路径对不对！
        .planning_pipelines(pipelines=["ompl", "pilz_industrial_motion_planner"])
        .to_dict()
    )

    # 强制同步仿真时间
    moveit_config_dict.update(
        {
            # "use_sim_time": True,
            "planning_pipelines": {
                "pipeline_names": ["ompl", "pilz_industrial_motion_planner"]
            },
        }
    )

    print("🚀 正在初始化 MoveItPy 节点...")

    # 2. 【极其关键的修复】把参数字典通过 config_dict 喂给 MoveItPy！
    aubo_moveit = MoveItPy(
        node_name="gripper_cmd_node",
        config_dict=moveit_config_dict,  # <== 就是少了这一句！
    )

    # 3. 获取规划组
    gripper = aubo_moveit.get_planning_component("gripper")

    def execute_pose(pose_name):
        print(f"\n---> 准备将夹爪设置为快捷动作: [{pose_name}]")

        # 强制更新起点为真实状态
        gripper.set_start_state_to_current_state()

        # 设置目标状态
        gripper.set_goal_state(configuration_name=pose_name)

        print("🧠 正在计算运动轨迹...")
        plan_result = gripper.plan()

        if plan_result:
            print("✅ 规划成功，正在向底层发送指令...")
            aubo_moveit.execute(plan_result.trajectory, controllers=[])
            print(f"🎉 [{pose_name}] 动作执行完毕！")
        else:
            print(f"❌ 规划失败！无法执行 [{pose_name}]。")

    # 稍微等 1 秒，确保节点与 Gazebo 的时间线对齐
    time.sleep(1.0)

    # 🎬 执行测试动作
    execute_pose("gripper_open")

    print("⏳ 保持张开状态 13 秒钟...")
    time.sleep(13.0)

    execute_pose("gripper_close")

    print("\n👋 指令发送结束，退出脚本。")
    rclpy.shutdown()


if __name__ == "__main__":
    main()
