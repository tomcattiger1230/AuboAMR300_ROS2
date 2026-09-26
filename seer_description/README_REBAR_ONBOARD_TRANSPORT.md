# 钢筋上车、导航并送至拉伸测试位置

流程分三步：从起点取料工位抓取钢筋并放入车载第 1 槽；载筋沿固定路线
驶到拉伸测试台前 `(6.0, 3.05)` m、朝向 −90°；从车载槽重新抓取钢筋，
将其竖直送至测试机夹持线。第 3 步结束时钢筋仍由机器人临时抓取约束保持，
测试机抱爪不闭合，机器人不松爪。后续交接等待单独指令。

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

# 第 3 步 A：从车载第 1 槽重新夹取，抬升并建立运输约束
ros2 run seer_description pickup_onboard.py \
  --onboard-slot 1 --state-file /tmp/rebar_onboard_tester_state.json \
  --output /tmp/rebar_onboard_step3_pick.json

# 第 3 步 B：转成竖直，送至测试机夹持线，到位后停止
ros2 run seer_description 04_insert.py \
  --state-file /tmp/rebar_onboard_tester_state.json \
  --output /tmp/rebar_onboard_step3_insert.json
```

每步退出码为 0 才执行下一步。第 2 步使用固定绕行点
`(0,0) → (4.5,0) → (4.5,2.6) → (6,2.6) → (6,3.05)`，
以 `/cmd_vel` 和里程计闭环行驶；目前没有接入 Nav2。导航节点在出发前
确认机械臂已松爪、钢筋位于指定料槽，并在每个途经点复核钢筋相对车体
的位置和方向。任一检查失败即停止车轮命令。

第 3 步 A 先确认底盘锁在固定点、钢筋仍在车载槽内且夹爪张开；闭爪后
先直上约 4 cm 脱离弧形托座，再执行碰撞检查过的抬升路径。托座内起始
姿态与闭合夹爪有接触，因此**仅这一小段直上脱座**关闭 MoveIt 碰撞检查；
钢筋实测未跟随夹爪抬起时立即停止。第 3 步 B 会根据检查点中的
`source: onboard_slot` 使用车载取筋专用竖直化暂存位、绕钢筋轴线的夹爪
转向和经过碰撞检查的预插入关节路径；最终仍须通过钢筋位置、轴向检查。
两段使用同一个 `--state-file`；仿真重启后应从第 1 步重新开始。

**当前限制：**从新场景连续执行四段时，最后的
`onboard_preposition` 逆解可能失败并安全停止。复现状态、返回分支和
待解决事项见 [逆解问题记录](REBAR_ONBOARD_PREPOSITION_IK_ISSUE.md)。

原 [自动装填流程](README_REBAR_TESTER_LOAD.md)中的 `02_pick.py`、
`03_navigate.py` 采用**机械臂持续夹持**钢筋的运输方式，不是这里的车载
料槽流程。第 3 步复用其 `04_insert.py`，通过检查点选择车载取筋专用路径。

## 本轮 Ubuntu 仿真结果（2026-09-26）

| 步骤 | 结果 | 报告 | 图片 |
|---|---|---|---|
| 取筋并放入第 1 槽 | 25 项通过、0 失败；松爪后钢筋中心 `(0.2927, -0.0021, 0.7150)` m（车体系），机械臂撤回后 3 秒位移约 0.004 mm | [JSON](test/results/rebar_onboard_step1_20260926.json) | [末端相机](test/results/rebar_onboard_step1_20260926.png) |
| 载筋到测试台前 | 13 项通过、0 失败；底盘 `(6.000, 3.034)` m，朝向约 −90°；钢筋仍在第 1 槽，槽位误差 2.3 mm | [JSON](test/results/rebar_onboard_step2_20260926.json) | [末端相机](test/results/rebar_onboard_step2_20260926.png) |
| 从车载槽重新夹取 | 脱座、抬升、抓取约束和高位运输均通过；钢筋抬至车体系约 `(0.292, -0.002, 1.044)` m | [JSON](test/results/rebar_onboard_step3_pick_20260926.json) | [末端相机](test/results/rebar_onboard_step3_pick_20260926.png) |
| 送至测试位置 | 钢筋中心 `(6.004, 3.901, 1.498)` m，轴线竖直；机器人仍保持钢筋，测试机未接管 | [插入报告](test/results/rebar_onboard_step3_insert_20260926.json) / [夹爪转向报告](test/results/rebar_onboard_step3_roll_20260926.json) | [实测位置示意图](test/results/rebar_onboard_step3_schematic_20260926.png) |

本轮第 3 步经过安全停止后的断点续跑：首次取筋后需要移除 MoveIt 中的旧
车载钢筋碰撞体，随后从已夹住钢筋的状态使用 `pickup_onboard.py --resume-clamped`
完成抬升；插入时又根据规划探针调整竖直化暂存位和
关节分支，最终成功。上述 JSON 分别记录成功的子段，**尚未验证从车载槽
到夹持线的一次连续运行**。

车载槽和重新取筋的 PNG 来自机械臂末端黑白相机，只显示局部视角。
插入终点时相机被测试机近处表面遮挡；示意图按本轮 Isaac/ROS 实测位置
绘制，不是相机截图。位置和状态由里程计、钢筋 TF 与测试机反馈确认。
