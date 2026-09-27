# 提前竖直化钢筋，再驱动底盘靠近拉伸测试机

本实验解决车载钢筋流程中最后一段大幅关节预定位偶发无逆解的问题。
机械臂在测试机外的开阔点 `(6.08, 2.6)` m，从车载第 1 槽取筋、竖直化、
进入正面预插入关节姿态；底盘保持这一姿态低速倒车到 `(6.08, 3.05)` m；
停车后末端分两段短距离移动至夹持线。末端工具轴沿世界 +Y，垂直于
测试台的 XZ 正面；夹爪沿 X 方向开合。第 8 段由测试机上下抱爪接管钢筋，
确认接管后机械臂松爪。机械臂尚未退回。

## 节点与运行方法

启动 [实验室场景](README_REBAR_ONBOARD_TRANSPORT.md#启动)，另开 Ubuntu
终端执行：

```bash
source /opt/ros/lyrical/setup.bash
source ~/Develop/ROS_ws/hongshi_mm_ws/install/setup.bash
export ROS_DOMAIN_ID=133

# 1. 起点取筋、放入车载第 1 槽
ros2 run seer_description test_rebar_grasp.py \
  --onboard-slot 1 --output /tmp/rebar_early_01.json

# 2. 载筋至开阔转向点，停放并锁定底盘
ros2 run seer_description 02_navigate_standoff.py \
  --onboard-slot 1 --output /tmp/rebar_early_02.json

# 3. 从车载槽重新夹取
ros2 run seer_description pickup_onboard.py \
  --onboard-slot 1 --at-standoff \
  --state-file /tmp/rebar_early_state.json --output /tmp/rebar_early_03.json

# 4. 在开阔点将钢筋竖直化
ros2 run seer_description 03_verticalize_standoff.py \
  --state-file /tmp/rebar_early_state.json \
  --output /tmp/rebar_early_04_verticalize.json

# 5. 在开阔点预先设好接近测试机的关节姿态
ros2 run seer_description 04_preposition_standoff.py \
  --state-file /tmp/rebar_early_state.json \
  --output /tmp/rebar_early_05_preposition.json

# 6. 保持竖直持筋姿态，低速驶到测试机停靠点
ros2 run seer_description 05_drive_vertical_rebar.py \
  --state-file /tmp/rebar_early_state.json \
  --output /tmp/rebar_early_06_drive.json

# 7. 停车后末端作两段短距离微调，到夹持线即停止
ros2 run seer_description 06_micro_insert.py \
  --state-file /tmp/rebar_early_state.json \
  --output /tmp/rebar_early_07_micro.json

# 8. 测试机上下抱爪合紧，确认接管后机器人松爪
ros2 run seer_description 05_handoff.py \
  --state-file /tmp/rebar_early_state.json \
  --output /tmp/rebar_early_08_handoff.json
```

每段退出码为 0 才运行下一段。所有节点共享状态文件，场景重启后从第 1 段
重新开始。第 2 段沿绕行路线到 `(6.08, 2.6)` m，之后不继续驶向机器。
第 3 段复用车载取筋节点的 `--at-standoff` 模式。第 5 段使用在本场景实测
过的关节候选，但**先以 FK 验证末端目标，再经 MoveIt 碰撞检查规划**；
不满足验证则停止。第 6 段速度上限 `0.08 m/s`，循环监测钢筋相对车体
的位置与轴线、底盘横向误差和朝向，异常时停车。第 7 段不调用旧的
`onboard_preposition` 逆解，直接从第 5 段已完成的姿态作两次短距离
笛卡尔移动，并验证夹持线位置。
第 8 段沿用既有 `05_handoff.py`，现在接受第 7 段写入的
`micro_insert` 检查点。它先复核钢筋位置，再依次命令下抱爪高度
`1.12 m`、上抱爪高度 `1.87 m` 以及两抱爪闭合；只有测试机反馈已接管
且机器人抓取约束解除后，才张开机器人夹爪，并再次检查钢筋仍在夹持线。

导航目前基于 `/cmd_vel` 和里程计闭环，未接入 Nav2。竖直钢筋行驶的
通行空间按当前实验室场景验证；更改货架、设备或路线后须重新检查。

## 正面接近实测（2026-09-27）

从新场景按 8 段依次执行，**100 项检查通过、0 失败**。底盘停在
`(6.063, 3.047)` m，钢筋最终中心为 `(6.001, 3.906, 1.498)` m；
测试机接管后机器人夹爪已张开。交接后的 TF 测得末端工具轴世界方向
约 `(-0.006, 1.000, -0.001)`，即朝向测试台正面。静置测量通过，
钢筋世界坐标无可测位移；见[稳定性数据](test/results/rebar_front_stability_20260927.json)。

| 段 | 报告 |
|---|---|
| 1 起点装车 | [JSON](test/results/rebar_front_01_20260927.json) |
| 2 到开阔点 | [JSON](test/results/rebar_front_02_20260927.json) |
| 3 车载取筋 | [JSON](test/results/rebar_front_03_20260927.json) |
| 4 竖直化 | [JSON](test/results/rebar_front_04_20260927.json) |
| 5 正面预定位 | [JSON](test/results/rebar_front_05_20260927.json) |
| 6 竖直持筋行驶 | [JSON](test/results/rebar_front_06_20260927.json) |
| 7 正面短距离微调 | [JSON](test/results/rebar_front_07_20260927.json) |
| 8 测试机接管 | [JSON](test/results/rebar_front_08_20260927.json) |

以下两张图依据交接后的实时 TF 位姿在相同 USD 场景中离线重建，
展示正面朝向与设备布局，不是运行中相机的直接截图：

![正面接近的机器人与测试台](test/results/rebar_front_handoff_workcell_20260927.png)

![夹爪与竖直钢筋的正面交接细节](test/results/rebar_front_handoff_detail_20260927.png)

## 此前侧向接近实测（2026-09-27，历史记录）

从新启动的场景按前 7 段依次执行，共 92 项检查通过、0 失败；随后在同一
仿真场景执行第 8 段，**累计 100 项检查通过、0 失败**：

| 段 | 结果 | 报告 |
|---|---|---|
| 1 装车 | 钢筋稳定落在车载第 1 槽 | [JSON](test/results/rebar_early_01.json) |
| 2 到开阔点 | 停在 `(5.983, 2.603)` m，钢筋仍在槽内 | [JSON](test/results/rebar_early_02.json) |
| 3 车载取筋 | 脱离托座并抬高，机器人建立抓取约束 | [JSON](test/results/rebar_early_03.json) |
| 4 竖直化 | 钢筋轴线 Z 分量约 `1.0`，夹持位置偏差约 `5 mm` | [JSON](test/results/rebar_early_04_verticalize.json) |
| 5 提前预定位 | 碰撞检查轨迹 71 点，执行后钢筋车体系约 `(-0.567, 0.007, 1.499)` m | [JSON](test/results/rebar_early_05_preposition.json) |
| 6 竖直持筋行驶 | 底盘至 `(5.982, 3.042)` m；沿途钢筋相对车体偏差约 `7 mm` | [JSON](test/results/rebar_early_06_drive.json) |
| 7 末端微调 | 两段笛卡尔轨迹均完成，钢筋中心世界坐标约 `(6.005, 3.903, 1.498)` m，轴线 Z 分量 `0.99999` | [JSON](test/results/rebar_early_07_micro.json) |
| 8 测试机接管、机械臂松爪 | 上下抱爪合紧并确认接管；机器人夹爪张开至约 `0.0001 m`，钢筋位置不变 | [JSON](test/results/rebar_early_08_handoff.json) |

到位后再静置 3 秒，钢筋中心位移约 `0.0025 mm`，底盘锁定，机器人仍持筋，
测试机未夹持；见 [稳定性数据](test/results/rebar_early_final_stability_20260927.json)。
第 8 段完成后再次静置 3 秒，反馈显示机器人抓取约束为假、测试机夹持
为真、底盘锁定，钢筋没有可测位移；见
[交接后稳定性数据](test/results/rebar_early_handoff_stability_20260927.json)。

![开阔点竖直夹持钢筋的末端相机画面](test/results/rebar_early_vertical_camera_20260927.png)

![竖直持筋驶到测试台前的末端相机画面](test/results/rebar_early_drive_camera_20260927.png)

终点时腕部相机被测试机近处结构遮挡。下图根据本轮 ROS/Isaac 实测坐标
绘制，是位置示意图，不是相机截图：

![测试位置实测示意图](test/results/rebar_early_vertical_result_20260927.png)

测试机接管后，末端相机可看到钢筋两侧张开的机器人夹爪：

![测试机接管后机器人夹爪张开](test/results/rebar_early_handoff_camera_20260927.png)

为查看工位整体布局，按交接后的 ROS 位姿在相同 USD 场景中离线重建了
两个斜上方视角。这些图不是运行中相机的直接截图：

![机器人与测试机工位全局视角](test/results/rebar_handoff_global_workcell_20260927.png)

![包含周边设备的更宽全局视角](test/results/rebar_handoff_global_wide_20260927.png)

这是一轮成功的完整场景实验；尚需多轮重跑来评估规划与物理仿真的
重复性。机械臂退回、底盘驶离和测试机拉伸动作尚未执行。
