# 钢筋上车并导航到拉伸测试台

本阶段只完成两步：从起点取料工位抓取钢筋，放入车载第 1 槽并松爪；
随后载筋沿固定路线驶到拉伸测试台前 `(6.0, 3.05)` m、朝向 −90°。
到位后锁住底盘，等待下一条指令。不会竖直化、插入钢筋或操作测试机抱爪。

## 启动

在 Ubuntu 启动实验室场景与 ROS 2 / MoveIt：

```bash
source /opt/ros/lyrical/setup.bash
source ~/Develop/ROS_ws/hongshi_mm_ws/install/setup.bash
ros2 run seer_description start_warehouse_finger_rebar_lab_demo.sh --no-rviz --domain-id 133
```

另开终端，使用相同的 `ROS_DOMAIN_ID`，按顺序执行：

```bash
source /opt/ros/lyrical/setup.bash
source ~/Develop/ROS_ws/hongshi_mm_ws/install/setup.bash
export ROS_DOMAIN_ID=133

# 第 1 步：抓筋、放入车载第 1 槽、松爪并验证稳定
ros2 run seer_description test_rebar_grasp.py \
  --onboard-slot 1 --output /tmp/rebar_onboard_step1.json

# 第 2 步：确认钢筋仍在槽内，再行驶并停在测试台前
ros2 run seer_description navigate_onboard.py \
  --onboard-slot 1 --output /tmp/rebar_onboard_step2.json
```

每步退出码为 0 才执行下一步。第 2 步使用固定绕行点
`(0,0) → (4.5,0) → (4.5,2.6) → (6,2.6) → (6,3.05)`，
以 `/cmd_vel` 和里程计闭环行驶；目前没有接入 Nav2。导航节点在出发前
确认机械臂已松爪、钢筋位于指定料槽，并在每个途经点复核钢筋相对车体
的位置和方向。任一检查失败即停止车轮命令。

原 [自动装填流程](README_REBAR_TESTER_LOAD.md)中的 `02_pick.py`、
`03_navigate.py` 采用**机械臂持续夹持**钢筋的运输方式，不是这里的车载
料槽流程。后续从料槽重新抓取并装入测试机，需要另行实现和验证。

## 本轮 Ubuntu 仿真结果（2026-09-26）

| 步骤 | 结果 | 报告 | 末端相机图 |
|---|---|---|---|
| 取筋并放入第 1 槽 | 25 项通过、0 失败；松爪后钢筋中心 `(0.2927, -0.0021, 0.7150)` m（车体系），机械臂撤回后 3 秒位移约 0.004 mm | [JSON](test/results/rebar_onboard_step1_20260926.json) | [PNG](test/results/rebar_onboard_step1_20260926.png) |
| 载筋到测试台前 | 13 项通过、0 失败；底盘 `(6.000, 3.034)` m，朝向约 −90°；钢筋仍在第 1 槽，槽位误差 2.3 mm | [JSON](test/results/rebar_onboard_step2_20260926.json) | [PNG](test/results/rebar_onboard_step2_20260926.png) |

两张图来自机械臂末端黑白相机，只显示车载料槽的局部视角。固定点位置
由里程计和钢筋 TF 检查报告确认。
